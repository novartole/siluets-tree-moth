//! Tree moth
//!
//! Input: move detection -> d7 (interrupt on change)
//! Output: d5 -> MOSFET (PWM via Timer0)
//! Interrupt: on change of d7
//! Error: toggle d13 led twise per second

/*
* Timer0: 5,6
* Timer1: 9,10
* Timer2: 3,11
*
* PWM pins: 3, 5, 6, 9, 10, 11
*
* Port A: a0..a5
* Port B: d8..d13
* Port C: d0..d7
*/

#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::sync::atomic::{AtomicBool, Ordering};

use arduino_hal::{delay_ms, Peripherals, simple_pwm::*};
use avr_device::interrupt;

static mut FLAG: u8 = 0;
static MOVED: AtomicBool = AtomicBool::new(true);
static CHECKING: AtomicBool = AtomicBool::new(false);

enum Moth {
    First,
    Second,
    Third
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    interrupt::disable();

    // SAFETY: Because main() already has references to the peripherals this is an unsafe
    // operation - but because no other code can run after the panic handler was called,
    // we know it is okay.
    let dp = unsafe { Peripherals::steal() };
    let pins = arduino_hal::pins!(dp);

    let mut error_led = pins.d13.into_output();
    loop {
        error_led.toggle();
        delay_ms(500);
    }
}

#[arduino_hal::entry]
fn main() -> ! {
    let moth = match option_env!("MOTH_ID")
        .expect("MOTH_ID must be set")
        .parse::<u8>()
    {
        Ok(id) if id == 0 => Moth::First,
        Ok(id) if id == 1 => Moth::Second,
        Ok(id) if id == 2 => Moth::Third,
        Err(_) => panic!("MOTH_ID must be a number"),
        _ => panic!("MOTH_ID value must be in inclusive range from 0 to 2"),
    };

    let dp = Peripherals::take().unwrap();
    let pins = arduino_hal::pins!(dp);

    // TC0 controls PWM of d5 pin. 
    // Arduino will be connected to MOSFET chip via d5 pin.
    //
    let timer0 = Timer0Pwm::new(dp.TC0, Prescaler::Prescale1024);
    let mut mosfet = pins.d5.into_output().into_pwm(&timer0);
    mosfet.enable();

    // Adrduino will be connected to move detection chip via d7 pin.
    // Any move detection causes an interrupt.
    // 
    dp.EXINT.pcicr.write(|w|
        // enable PCINT2 (Port D) pin change interrupt
        unsafe { w.bits(0b100) }
    );
    dp.EXINT.pcmsk2.write(|w|
        // enable pin change interrupt on PCINT23 (d7)
        w.bits(1 << 7)
    );

    // Enable interrupts globally.
    // SAFETY: we are not inside a critical section.
    unsafe { interrupt::enable() };

    loop {
        if MOVED.load(Ordering::Acquire) {
            MOVED.store(false, Ordering::Release);

            match moth {
                Moth::First => (), // no delay here
                Moth::Second => delay_ms(1000 * 1),
                Moth::Third => delay_ms(1000 * 2),
            }

            for duty in (0..=255).rev().step_by(2) 
                .chain((0..=255).step_by(32))       
                .chain((0..=255).rev().step_by(8)) 
            {
                mosfet.set_duty(duty);
                delay_ms(10);
            }

            match moth {
                Moth::First => delay_ms(1000 * 3),
                Moth::Second => delay_ms(1000 * 2),
                Moth::Third => delay_ms(1000 * 1),
            }

            for duty in (0..=255).step_by(4) {
                mosfet.set_duty(duty);
                delay_ms(10);
            }

            // wait for _10_ secs before next move detection
            delay_ms(10_000);
            CHECKING.store(true, Ordering::SeqCst);
        }
    }
}

#[interrupt(atmega328p)]
fn PCINT2() {
    if CHECKING.load(Ordering::SeqCst) {
        // Prevent from MOVE is being set twice.
        // SAFETY: access to FLAG happens only inside the current function.
        unsafe {
            FLAG = 1 - FLAG;
            if FLAG == 0 {
                return;
            }
        }

        MOVED.store(true, Ordering::SeqCst);
    }
}
