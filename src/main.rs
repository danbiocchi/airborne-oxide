#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{pac, prelude::*};

// Bring in defmt for logging
use defmt_rtt as _;

// For testing, we need to bring in some additional crates
#[cfg(test)]
use defmt_test as _;

/// LedControl struct: Our abstraction for controlling the LED
struct LedControl {
    led: stm32f4xx_hal::gpio::gpioa::PA3<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>,
}

impl LedControl {
    /// Create a new LedControl instance
    fn new(gpioa: pac::GPIOA) -> Self {
        let gpioa = gpioa.split();
        let led = gpioa.pa3.into_push_pull_output();
        LedControl { led }
    }

    /// Turn the LED on
    fn set_high(&mut self) {
        self.led.set_high();
    }

    /// Turn the LED off
    fn set_low(&mut self) {
        self.led.set_low();
    }
}

/// Function to introduce a delay
fn delay(cycles: u32) {
    cortex_m::asm::delay(cycles);
}

#[entry]
fn main() -> ! {
    defmt::println!("Program start");

    let dp = pac::Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

    let mut led_control = LedControl::new(dp.GPIOA);

    loop {
        defmt::println!("LED on");
        led_control.set_high();
        delay(24_000_000);

        defmt::println!("LED off");
        led_control.set_low();
        delay(24_000_000);
    }
}

// Test module
#[cfg(test)]
mod tests {
    use super::*;

    #[defmt_test::tests]
    mod test {
        use super::*;

        #[test]
        fn test_led_control() {
            defmt::println!("Starting LED control test");

            let dp = unsafe { pac::Peripherals::steal() };
            let mut led_control = LedControl::new(dp.GPIOA);

            led_control.set_high();
            defmt::println!("LED turned on");
            cortex_m::asm::nop();

            led_control.set_low();
            defmt::println!("LED turned off");
            cortex_m::asm::nop();

            defmt::println!("LED control test completed successfully");
        }

        #[test]
        fn test_delay() {
            defmt::println!("Starting delay test");

            defmt::println!("Delaying for 1000 cycles...");
            delay(1000);
            defmt::println!("Delay completed");

            cortex_m::asm::nop();

            defmt::println!("Delay test completed successfully");
        }
    }
}