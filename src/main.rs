// Disable the standard library for embedded development
#![no_std]
// Disable the main function for embedded systems
#![no_main]

// Import panic handler for halting on panic
use panic_halt as _;
// Import entry point macro for Cortex-M runtime
use cortex_m_rt::entry;
// Import STM32F4 HAL and its prelude
use stm32f4xx_hal::{pac, prelude::*};

// Import defmt for logging over RTT (Real-Time Transfer)
use defmt_rtt as _;

// Conditionally import defmt_test for testing
#[cfg(test)]
use defmt_test as _;

/// LedControl struct: Abstraction for controlling the LED
struct LedControl {
    // PA3 pin configured as push-pull output
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

/// Function to introduce a delay using CPU cycles
fn delay(cycles: u32) {
    cortex_m::asm::delay(cycles);
}

/// Entry point of the program
#[entry]
fn main() -> ! {
    defmt::println!("Program start");

    // Get access to the device-specific peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Configure the clock system
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

    // Initialize LED control
    let mut led_control = LedControl::new(dp.GPIOA);

    // Main loop
    loop {
        defmt::println!("LED on");
        led_control.set_high();
        delay(24_000_000); // Delay for about 0.5 seconds at 48MHz

        defmt::println!("LED off");
        led_control.set_low();
        delay(24_000_000); // Delay for about 0.5 seconds at 48MHz
    }
}

// Test module
#[cfg(test)]
mod tests {
    use super::*;

    #[defmt_test::tests]
    mod test {
        use super::*;

        /// Test LED control functionality
        #[test]
        fn test_led_control() {
            defmt::println!("Starting LED control test");

            // Safely get access to the device peripherals
            let dp = unsafe { pac::Peripherals::steal() };
            let mut led_control = LedControl::new(dp.GPIOA);

            led_control.set_high();
            defmt::println!("LED turned on");
            cortex_m::asm::nop(); // No operation, used for debugging

            led_control.set_low();
            defmt::println!("LED turned off");
            cortex_m::asm::nop(); // No operation, used for debugging

            defmt::println!("LED control test completed successfully");
        }

        /// Test delay function
        #[test]
        fn test_delay() {
            defmt::println!("Starting delay test");

            defmt::println!("Delaying for 1000 cycles...");
            delay(1000);
            defmt::println!("Delay completed");

            cortex_m::asm::nop(); // No operation, used for debugging

            defmt::println!("Delay test completed successfully");
        }
    }
}