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

    let gpioc = dp.GPIOC.split(); // GPIOC for USART6 pins

    // Setup UART6 on pins PC6 (TX) and PC7 (RX)
    let tx = gpioc.pc6.into_alternate(); // TX6 (USART6)
    let rx = gpioc.pc7.into_alternate(); // RX6 (USART6)

    let mut serial = dp.USART6.serial(
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
