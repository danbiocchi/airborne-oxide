#![no_std]
#![no_main]
extern crate alloc;

use alloc::format;
use core::fmt::Write;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{config::Config, Event, Serial},
};
use cortex_m::delay::Delay;
use heapless::String;
use rand_core::{RngCore, SeedableRng};
use rand_chacha::ChaCha8Rng;
use stm32f4xx_hal::timer::Timer;

// Constants for Wi-Fi configuration
const WIFI_SSID: &str = "F405WTE";
const WIFI_PASSWORD: &str = "f405wte";
const UDP_PORT: u16 = 14550; // Common port for MAVLink

#[entry]
fn main() -> ! {
    // Initialize the device peripherals
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure the clock system
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // Split the GPIOA peripheral into individual pins
    let gpioa = dp.GPIOA.split();

    // Configure UART1 pins for ESP8266 communication (TX1/RX1)
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();

    // Initialize UART1 for communication with ESP8266
    let mut serial = Serial::new(
        dp.USART1,
        (tx_pin, rx_pin),
        Config::default().baudrate(115_200.bps()),
        &clocks
    ).unwrap();

    // Enable RX interrupts
    serial.listen(Event::Rxne);

    // Initialize delay
    let mut delay = Timer::new(dp.TIM5, &clocks).delay();

    // Initialize RNG
    let seed = 0x13371337; // You might want to use a more random seed in production
    let mut rng = ChaCha8Rng::seed_from_u64(seed);

    // Connect to Wi-Fi
    connect_wifi(&mut serial, &mut delay);

    // Setup UDP connection
    setup_udp(&mut serial, &mut delay);

    // Initialize heartbeat counter
    let mut heartbeat_counter = 0;

    // Main loop to send heartbeat messages
    loop {
        heartbeat_counter += 1;
        let random_data = rng.next_u32();

        // Construct the heartbeat message
        let message = construct_heartbeat_message(heartbeat_counter, random_data);

        // Send the heartbeat message
        send_udp_message(&mut serial, &message, &mut delay);

        // Wait for 1 second before sending the next heartbeat
        delay.delay_ms(1000u32);
    }
}

fn connect_wifi(serial: &mut Serial<pac::USART1, (pac::USART1, pac::USART1)>, delay: &mut Delay) {
    // Reset ESP8266
    send_at_command(serial, "AT+RST", delay);
    delay.delay_ms(2000u32);

    // Set ESP8266 to station mode
    send_at_command(serial, "AT+CWMODE=1", delay);

    // Connect to Wi-Fi
    let connect_command = format!("AT+CWJAP=\"{}\",\"{}\"", WIFI_SSID, WIFI_PASSWORD);
    send_at_command(serial, &connect_command, delay);
    delay.delay_ms(5000u32); // Wait for connection

    // Check connection status
    send_at_command(serial, "AT+CIFSR", delay);
}

fn setup_udp(serial: &mut Serial<pac::USART1, (pac::USART1, pac::USART1)>, delay: &mut Delay) {
    // Enable multiple connections
    send_at_command(serial, "AT+CIPMUX=1", delay);

    // Setup UDP connection
    let udp_command = format!("AT+CIPSTART=0,\"UDP\",\"255.255.255.255\",{},{}",
                              UDP_PORT, UDP_PORT);
    send_at_command(serial, &udp_command, delay);
}

fn construct_heartbeat_message(counter: u32, random_data: u32) -> String<128> {
    let mut message: String<128> = String::new();
    write!(
        message,
        "F405 Heartbeat #{}: Matek F405 Wing is alive! Random: 0x{:08X}",
        counter,
        random_data
    )
        .unwrap();
    message
}

fn send_udp_message(serial: &mut Serial<pac::USART1, (pac::USART1, pac::USART1)>, 
                    message: &str, delay: &mut Delay) {
    // Prepare to send UDP data
    let send_command = format!("AT+CIPSEND=0,{}", message.len());
    send_at_command(serial, &send_command, delay);

    // Send the actual message
    send_at_command(serial, message, delay);
}

fn send_at_command(serial: &mut Serial<pac::USART1, (pac::USART1, pac::USART1)>, 
                   command: &str, delay: &mut Delay) {
    let _ = serial.write_str(command);
    let _ = serial.write_str("\r\n");
    delay.delay_ms(500u32);
}