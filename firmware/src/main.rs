#![deny(warnings)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![feature(used)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
// extern crate either;
extern crate panic_abort;
extern crate shared;
extern crate stm32f103xx;
extern crate stm32f103xx_hal as hal;

// use aligned::Aligned;
// use either::Either;
use hal::dma::{dma1, CircBuffer, Event, Transfer, R};
use hal::prelude::*;
use hal::pwm::{C1, Pwm};
use hal::serial::{Rx, Serial, Tx};
use hal::time::{Bps, Hertz};
use hal::timer::Timer;
use rtfm::{app, Resource, Threshold};
use shared::State;
use stm32f103xx::Interrupt;
use stm32f103xx::{TIM2, USART1, DWT};

// CONFIGURATION
const _0: u8 = 3;
const _1: u8 = 7;
const BAUD_RATE: Bps = Bps(115_200);
const LATCH_DELAY: Hertz = Hertz(20_000); // 50us == 20KHz
const LOG_FREQUENCY: Hertz = Hertz(1);
const WS2812B_FREQUENCY: Hertz = Hertz(400_000);

#[allow(non_camel_case_types)]
type TX_BUF = &'static mut [u8; 13];
type TX = Option<Either<(TX_BUF, dma1::C4, Tx<USART1>), Transfer<R, TX_BUF, dma1::C4, Tx<USART1>>>>;

// TASKS AND RESOURCES
app! {
    device: stm32f103xx,

    resources: {
        static BUSY: bool = false;
        static CONTEXT_SWITCHES: u16 = 0;
        static DWT: DWT;
        static PWM: Pwm<TIM2, C1>;
        static FRAMES: u8 = 0;
        static RGB_ARRAY: Aligned<u32, [u8; 72]> = Aligned([0; 72]);
        static RX: Rx<USART1>;
        static RX_BUFFER: [[u8; 72]; 2] = [[0; 72]; 2];
        static RX_CB: CircBuffer<[u8; 72], dma1::C5>;
        static SLEEP_CYCLES: u32 = 0;
        static TX: TX;
        static TX_BUFFER: [u8; 13] = [0u8; 13];
        static WS2812B_BUFFER: [u8; 577] = [0; 577];
    },

    init: {
        resources: [TX_BUFFER],
    },

    idle: {
        resources: [
            SLEEP_CYCLES,
            DWT,
        ],
    },

    tasks: {
        DMA1_CHANNEL2: {
            path: frame_tail_start,
            resources: [
                CONTEXT_SWITCHES,
                WS2812B_BUFFER,
            ],
        },

        DMA1_CHANNEL4: {
            path: tx_transfer_done,
            resources: [
                CONTEXT_SWITCHES,
                TX,
            ],
        },

        DMA1_CHANNEL5: {
            path: rx,
            resources: [
                BUSY,
                CONTEXT_SWITCHES,
                RGB_ARRAY,
                RX_BUFFER,
                RX,
                RX_CB,
            ],
        },

        EXTI0: {
            path: frame_start,
            resources: [
                CONTEXT_SWITCHES,
                RGB_ARRAY,
                WS2812B_BUFFER,
                PWM
            ],
        },

        TIM1_UP_TIM10: {
            path: frame_end,
            resources: [
                BUSY,
                CONTEXT_SWITCHES,
                FRAMES,
            ],
        },

        TIM3: {
            path: log,
            resources: [
                CONTEXT_SWITCHES,
                FRAMES,
                SLEEP_CYCLES,
                TX,
                DWT,
            ],
        },
    },
}

// INITIALIZATION
fn init(p: init::Peripherals, r: init::Resources) -> init::LateResources {
    let dwt = p.core.DWT;
    dwt.enable_cycle_counter();
    let mut rcc = p.device.RCC.constrain();
    let flash = p.device.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut afio = p.device.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = p.device.GPIOA.split(&mut rcc.apb2);

    let timer1 = Timer::tim1(p.device.TIM1, LATCH_DELAY, clocks, &mut rcc.apb1); // where is Timer::tim1 for TIM1?
    let timer3 = Timer::tim3(p.device.TIM3, LOG_FREQUENCY, clocks, &mut rcc.apb1);
    // use these timers; make something listen to them and fire the interrupts instead of passing them around as resources?

    // correct pins?
    let pa9 = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
    let pa10 = gpioa.pa10;

    let serial = Serial::usart1(
        p.device.USART1,
        (pa9, pa10),
        &mut afio.mapr,
        BAUD_RATE,
        clocks,
        &mut rcc.apb2,
    );
    let (tx, rx) = serial.split();

    let mut pwm = p.device.TIM2.pwm(
        gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl), // XXX which pin?
        &mut afio.mapr,
        WS2812B_FREQUENCY,
        clocks,
        &mut rcc.apb1,
    );
    pwm.enable();

    let mut channels = p.device.DMA1.split(&mut rcc.ahb);

    channels.2.listen(Event::TransferComplete);
    channels.4.listen(Event::TransferComplete);
    channels.5.listen(Event::TransferComplete);

    let rx_cb = rx.circ_read(channels.5, r.RX_BUFFER); // figure out new-dma

    // timer3.resume();

    init::LateResources {
        TX: Some(Either::Left((r.TX_BUFFER, channels.4, tx))),
        RX: rx,
        RX_CB: rx_cb,
        DWT: dwt,
        PWM: pwm,
    }
}

// IDLE LOOP
fn idle(t: &mut Threshold, mut r: idle::Resources) -> ! {
    loop {
        rtfm::atomic(t, |t| {
            let dwt = r.DWT.borrow(t);
            let sleep_cycles = r.SLEEP_CYCLES.borrow_mut(t);

            // Sleep
            let before = dwt.cyccnt.read();
            rtfm::wfi();
            let after = dwt.cyccnt.read();

            let elapsed = after.wrapping_sub(before);
            *sleep_cycles += elapsed;
        });

        // interrupts are serviced here
    }
}

// TASKS
fn log(_t: &mut Threshold, r: TIM3::Resources) {
    // let timer = &*r.TIM3;

    let (buf, c, tx) = match r.TX.take().unwrap() {
        Either::Left((buf, c, tx)) => (buf, c, tx),
        Either::Right(trans) => trans.wait(),
    };

    // timer.wait().unwrap();

    let snapshot = r.DWT.cyccnt.read();
    let state = State {
        context_switches: *r.CONTEXT_SWITCHES,
        frames: *r.FRAMES,
        sleep_cycles: *r.SLEEP_CYCLES,
        snapshot: snapshot,
    };

    state.serialize(&mut buf);
    *r.TX = Some(Either::Right(tx.write_all(c, buf)));

    *r.CONTEXT_SWITCHES = 0;
    *r.FRAMES = 0;
    *r.SLEEP_CYCLES = 0;
}

fn tx_transfer_done(_t: &mut Threshold, r: DMA1_CHANNEL4::Resources) {
    *r.CONTEXT_SWITCHES += 1;

    let (buf, c, tx) = match r.TX.take().unwrap() {
        Either::Left((buf, c, tx)) => (buf, c, tx),
        Either::Right(trans) => trans.wait(),
    };

    *r.TX = Some(Either::Left((buf, c, tx)));
}

fn rx(_t: &mut Threshold, r: DMA1_CHANNEL5::Resources) {
    *r.CONTEXT_SWITCHES += 1;

    let cb = r.RX_CB;
    cb.peek(|buf, _| r.RGB_ARRAY.array.copy_from_slice(buf));
    rtfm::set_pending(Interrupt::EXTI0);
}

fn frame_start(_t: &mut Threshold, r: EXTI0::Resources) {
    *r.CONTEXT_SWITCHES += 1;

    let pwm = *r.PWM;

    // Construct and send WS2812B frame
    for (rgb, bits) in r.RGB_ARRAY
        .array
        .chunks(3)
        .zip((*r.WS2812B_BUFFER).borrow_mut().chunks_mut(24))
    {
        let r = rgb[0];
        let g = rgb[1];
        let b = rgb[2];

        // NOTE these LEDs use the GRB format
        for (mut byte, bits) in [g, r, b].iter().cloned().zip(bits.chunks_mut(8)) {
            for bit in bits.iter_mut().rev() {
                *bit = if byte & 1 == 0 { _0 } else { _1 };

                byte = byte >> 1;
            }
        }
    }

    pwm.set_duties(r.DMA1, C1, r.WS2812B_BUFFER).unwrap();
}

fn frame_tail_start(_t: &mut Threshold, r: DMA1_CHANNEL2::Resources) {
    *r.CONTEXT_SWITCHES += 1;

    let timer = Timer(&*r.TIM1);

    r.WS2812B_BUFFER.release(r.DMA1).unwrap();

    timer.resume();
    timer.restart();
}

fn frame_end(_t: &mut Threshold, r: TIM1_UP_TIM10::Resources) {
    *r.CONTEXT_SWITCHES += 1;

    let timer = Timer(&*r.TIM1);

    timer.wait().unwrap();

    timer.pause();

    *r.BUSY = false;
    *r.FRAMES += 1;
}
