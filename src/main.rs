#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _; // Panic handler
use stm32f4xx_hal::{pac, prelude::*, serial::{config::Config, Serial, Tx, Rx}};
use core::fmt::Write;
use heapless::String;

// Constants
const SYSTEM_ID: u8 = 1; // ID of this system
const COMPONENT_ID: u8 = 1; // ID of this component
const MESSAGE_INTERVAL: u32 = 168_000_000; // Roughly 1 second at 168MHz

// MAVLink-like message IDs (these are example IDs, not official MAVLink IDs)
const HEARTBEAT_MSG_ID: u8 = 0;
const ATTITUDE_MSG_ID: u8 = 30;

// Global static variables for UART
static mut TX: Option<Tx<pac::USART1>> = None;
static mut RX: Option<Rx<pac::USART1>> = None;

#[entry]
fn main() -> ! {
    // Initialize the device peripherals
    let dp = pac::Peripherals::take().unwrap();

    // Configure the system clock
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // Configure GPIOA
    let gpioa = dp.GPIOA.split();

    // Configure USART1 pins
    let tx_pin = gpioa.pa9.into_alternate();
    let rx_pin = gpioa.pa10.into_alternate();

    // Configure and split the USART peripheral
    let serial = Serial::new(
        dp.USART1,
        (tx_pin, rx_pin),
        Config::default().baudrate(921_600.bps()),
        &clocks
    ).unwrap();
    let (tx, rx) = serial.split();

    // Store UART Tx and Rx in global variables
    unsafe {
        TX = Some(tx);
        RX = Some(rx);
    }

    let mut counter = 0;
    let mut attitude = Attitude { pitch: 0.0, roll: 0.0, yaw: 0.0 };

    loop {
        // Send heartbeat message
        send_heartbeat(counter);

        // Update and send attitude
        attitude.pitch += 0.1;
        attitude.roll += 0.05;
        attitude.yaw += 0.025;
        send_attitude(&attitude);

        counter += 1;

        // Delay before next iteration
        cortex_m::asm::delay(MESSAGE_INTERVAL);
    }
}

// Struct to hold attitude data
struct Attitude {
    pitch: f32,
    roll: f32,
    yaw: f32,
}

// Function to send a heartbeat message
fn send_heartbeat(counter: u32) {
    let mut message: String<64> = String::new();
    write!(message, "HB,{},{},{}", SYSTEM_ID, COMPONENT_ID, counter).unwrap();
    send_mavlink_message(HEARTBEAT_MSG_ID, message.as_bytes());
}

// Function to send an attitude message
fn send_attitude(attitude: &Attitude) {
    let mut message: String<64> = String::new();
    write!(message, "ATT,{:.2},{:.2},{:.2}", attitude.pitch, attitude.roll, attitude.yaw).unwrap();
    send_mavlink_message(ATTITUDE_MSG_ID, message.as_bytes());
}

// Function to send a MAVLink-like message
fn send_mavlink_message(msg_id: u8, payload: &[u8]) {
    // Simple MAVLink-like header (just for demonstration)
    // In a real MAVLink implementation, this would include more fields and a checksum
    let header: [u8; 6] = [0xFE, payload.len() as u8, 0, msg_id, SYSTEM_ID, COMPONENT_ID];

    unsafe {
        if let Some(tx) = TX.as_mut() {
            // Send header
            for &byte in &header {
                let _ = tx.write(byte);
            }
            // Send payload
            for &byte in payload {
                let _ = tx.write(byte);
            }
            // Send a newline for readability (not part of actual MAVLink protocol)
            let _ = tx.write(b'\n');
        }
    }
}