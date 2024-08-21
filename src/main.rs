#![no_std] // This program will not use the Rust standard library
#![no_main] // This program doesn't use the standard main function entry point
extern crate alloc; // Import the alloc crate for heap allocation support

use alloc::format;
use core::fmt::Write;
use cortex_m_rt::entry;
use panic_halt as _; // Use panic-halt for panic handling
use stm32f4xx_hal::{pac, prelude::*, serial::{config::Config, Serial, Tx}};
use heapless::String;

// Import the LockedHeap allocator
use linked_list_allocator::LockedHeap;

// Define a static memory region for the heap
// This allocates 1024 bytes in the .heap section of memory
#[link_section = ".heap"]
static mut HEAP: [u8; 1024] = [0; 1024];

// Define a global allocator using LockedHeap
// This will be used for dynamic memory allocation
#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// Wi-Fi configuration constants
const WIFI_SSID: &str = "F405WTE";
const WIFI_PASSWORD: &str = "f405wte";
const UDP_PORT: u16 = 14550;

// Global static variable for the USART1 transmitter
// It's wrapped in Option because it will be initialized later
static mut SERIAL: Option<Tx<pac::USART1>> = None;

#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    
    // Configure the system clock
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // Configure GPIO pins for USART1
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();

    // Configure and split the USART1 peripheral
    let serial = Serial::new(
        dp.USART1,
        (tx_pin, rx_pin),
        Config::default().baudrate(115_200.bps()),
        &clocks
    ).unwrap();

    let (tx, _rx) = serial.split();
    
    // Store the transmitter in the global SERIAL variable
    unsafe {
        SERIAL = Some(tx);
    }

    // Initialize the heap allocator with the static HEAP
    unsafe { ALLOCATOR.lock().init(HEAP.as_ptr() as *mut u8, HEAP.len()) }

    // Connect to Wi-Fi network
    connect_wifi();
    
    // Set up UDP connection
    setup_udp();

    let mut heartbeat_counter = 0;

    // Main program loop
    loop {
        heartbeat_counter += 1;
        let message = construct_heartbeat_message(heartbeat_counter);
        send_udp_message(&message);
        // Delay for approximately 1 second at 168MHz
        cortex_m::asm::delay(168_000_000);
    }
}

// Function to connect to Wi-Fi network
fn connect_wifi() {
    send_at_command("AT+RST"); // Reset the ESP8266 module
    cortex_m::asm::delay(336_000_000); // 2 second delay
    send_at_command("AT+CWMODE=1"); // Set ESP8266 to station mode
    let connect_command = format!("AT+CWJAP=\"{}\",\"{}\"", WIFI_SSID, WIFI_PASSWORD);
    send_at_command(&connect_command); // Connect to Wi-Fi network
    cortex_m::asm::delay(840_000_000); // 5 second delay
    send_at_command("AT+CIFSR"); // Get IP address
}

// Function to set up UDP connection
fn setup_udp() {
    send_at_command("AT+CIPMUX=1"); // Enable multiple connections
    let udp_command = format!("AT+CIPSTART=0,\"UDP\",\"255.255.255.255\",{},{}", UDP_PORT, UDP_PORT);
    send_at_command(&udp_command); // Start UDP connection
}

// Function to construct the heartbeat message
fn construct_heartbeat_message(counter: u32) -> String<128> {
    let mut message: String<128> = String::new();
    write!(message, "F405 Heartbeat #{}: Matek F405 Wing is alive!", counter).unwrap();
    message
}

// Function to send UDP message
fn send_udp_message(message: &str) {
    let send_command = format!("AT+CIPSEND=0,{}", message.len());
    send_at_command(&send_command); // Prepare to send data
    send_at_command(message); // Send the actual message
}

// Function to send AT commands to the ESP8266 module
fn send_at_command(command: &str) {
    unsafe {
        if let Some(tx) = SERIAL.as_mut() {
            let _ = tx.write_str(command);
            let _ = tx.write_str("\r\n"); // Add carriage return and newline
        }
    }
    cortex_m::asm::delay(84_000_000); // 0.5 second delay
}