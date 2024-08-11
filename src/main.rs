#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{pac, prelude::*, serial::config::Config};
use core::fmt::Write;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    let gpioa = dp.GPIOA.split();

    // Setup UART on pins A9 (TX) and A10 (RX)
    let tx = gpioa.pa9.into_alternate();
    let rx = gpioa.pa10.into_alternate();

    let mut serial = dp.USART1.serial(
        (tx, rx),
        Config::default().baudrate(115200.bps()),
        &clocks
    ).unwrap();

    let mut counter = 0;

    loop {
        writeln!(serial, "Hello, Matek Wing F405 WTE! Count: {}", counter).unwrap();
        counter += 1;
        cortex_m::asm::delay(168_000_000); // Delay for approximately 1 second at 168MHz
    }
}