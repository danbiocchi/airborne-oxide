#![no_std] // Specify that we're not using the standard library
#![no_main] // Indicate that we're not using the standard main function

// Import necessary dependencies
use cortex_m_rt::entry; // Provides the entry point attribute for Cortex-M processors
use panic_halt as _; // Panic handler that halts on panic
use stm32f4xx_hal::{pac, prelude::*, serial::{config::Config, Serial, Tx, Rx}}; // Hardware abstraction layer for STM32F4 series
use heapless::Vec; // Provides a stack-allocated vector implementation

// Constants for MAVLink communication
const SYSTEM_ID: u8 = 1; // Unique identifier for this system in the MAVLink network
const COMPONENT_ID: u8 = 1; // Unique identifier for this component within the system

// MAVLink message IDs
const HEARTBEAT_MSG_ID: u8 = 0; // ID for the heartbeat message
const ATTITUDE_MSG_ID: u8 = 30; // ID for the attitude message

// Global static variables for UART communication
static mut TX: Option<Tx<pac::USART1>> = None; // Transmit part of USART1
static mut RX: Option<Rx<pac::USART1>> = None; // Receive part of USART1

// Global variables for MAVLink communication
static mut SEQUENCE_NUMBER: u8 = 0; // Keeps track of the MAVLink message sequence
static mut SYSTEM_TIME: u32 = 0; // Simulates system uptime in milliseconds

/// Main entry point for the program.
///
/// This function initializes the hardware, sets up UART communication,
/// and enters an infinite loop where it continuously sends heartbeat
/// and attitude messages.
///
/// The function performs the following steps:
/// 1. Initializes device peripherals and configures the system clock.
/// 2. Sets up GPIOA and configures USART1 pins for communication.
/// 3. Initializes UART with a baud rate of 921,600 bps.
/// 4. Enters a loop that:
///    a. Increments a simulated system time.
///    b. Sends a heartbeat message.
///    c. Updates and sends an attitude message.
///    d. Delays for approximately 1 second.
///
/// This function never returns, as indicated by the '!' return type.
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
    let tx_pin = gpioa.pa9.into_alternate(); // TX pin
    let rx_pin = gpioa.pa10.into_alternate(); // RX pin

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

    // Initialize attitude data
    let mut attitude = Attitude { pitch: 0.0, roll: 0.0, yaw: 0.0 };

    // Main program loop
    loop {
        // Increment system time (approximate milliseconds)
        unsafe { SYSTEM_TIME += 1000; } // Assuming 1 second delay

        // Send heartbeat message
        send_heartbeat();

        // Update and send attitude
        attitude.pitch += 0.1;
        attitude.roll += 0.05;
        attitude.yaw += 0.025;
        send_attitude(&attitude);

        // Delay for about 1 second at 168MHz
        cortex_m::asm::delay(168_000_000);
    }
}

/// Represents the attitude of the system.
///
/// This struct holds the pitch, roll, and yaw angles in radians.
/// These values represent the orientation of the system in 3D space.
struct Attitude {
    pitch: f32, // Pitch angle in radians
    roll: f32,  // Roll angle in radians
    yaw: f32,   // Yaw angle in radians
}

/// Sends a MAVLink heartbeat message.
///
/// The heartbeat message is a vital part of the MAVLink protocol,
/// indicating that the system is alive and functioning. It contains
/// basic status information about the system.
///
/// This function performs the following steps:
/// 1. Creates a new message buffer.
/// 2. Populates the buffer with the MAVLink header and heartbeat data.
/// 3. Sets the sequence number for the message.
/// 4. Calculates and appends the CRC for message integrity.
/// 5. Sends the complete message using the UART.
///
/// The heartbeat message includes:
/// - System and component IDs
/// - Type of the system (set to generic)
/// - Autopilot type (set to generic)
/// - System mode and status
fn send_heartbeat() {
    // Create a new vector to hold the message bytes
    let mut message = Vec::<u8, 17>::new();
    
    // Extend the vector with the MAVLink header and heartbeat message data
    message.extend_from_slice(&[
        0xFE, // MAVLink v1 start marker
        9,    // Payload length
        0,    // Sequence number (will be set later)
        SYSTEM_ID,
        COMPONENT_ID,
        HEARTBEAT_MSG_ID,
        0,    // Type: MAV_TYPE_GENERIC
        0,    // Autopilot: MAV_AUTOPILOT_GENERIC
        0,    // Base mode: None
        0, 0, 0, 0, // Custom mode (unused)
        0,    // System status: MAV_STATE_UNINIT
    ]).unwrap();
    
    // Set the sequence number
    message[2] = get_next_sequence_number();

    // Calculate and append the CRC
    let crc = calculate_crc(&message[1..]);
    message.extend_from_slice(&crc.to_le_bytes()).unwrap();

    // Send the complete MAVLink message
    send_mavlink_message(&message);
}

/// Sends a MAVLink attitude message.
///
/// This function constructs and sends an attitude message containing
/// the current orientation (pitch, roll, yaw) of the system.
///
/// The function performs these steps:
/// 1. Creates a new message buffer.
/// 2. Adds the MAVLink header and attitude message ID.
/// 3. Appends the system boot time.
/// 4. Adds the current roll, pitch, and yaw values.
/// 5. Adds placeholder values for angular velocities (set to 0).
/// 6. Sets the sequence number for the message.
/// 7. Calculates and appends the CRC.
/// 8. Sends the complete message via UART.
///
/// Parameters:
/// - attitude: A reference to an Attitude struct containing current orientation data.
fn send_attitude(attitude: &Attitude) {
    // Create a new vector to hold the message bytes
    let mut message = Vec::<u8, 36>::new();
    
    // Extend the vector with the MAVLink header and attitude message data
    message.extend_from_slice(&[
        0xFE, // MAVLink v1 start marker
        28,   // Payload length
        0,    // Sequence number (will be set later)
        SYSTEM_ID,
        COMPONENT_ID,
        ATTITUDE_MSG_ID,
    ]).unwrap();

    // Append system boot time
    let time_boot_ms = unsafe { SYSTEM_TIME };
    message.extend_from_slice(&time_boot_ms.to_le_bytes()).unwrap();

    // Append roll, pitch, and yaw values
    message.extend_from_slice(&(attitude.roll as f32).to_le_bytes()).unwrap();
    message.extend_from_slice(&(attitude.pitch as f32).to_le_bytes()).unwrap();
    message.extend_from_slice(&(attitude.yaw as f32).to_le_bytes()).unwrap();

    // Append placeholder values (0) for angular velocities
    message.extend_from_slice(&[0u8; 12]).unwrap();

    // Set the sequence number
    message[2] = get_next_sequence_number();

    // Calculate and append the CRC
    let crc = calculate_crc(&message[1..]);
    message.extend_from_slice(&crc.to_le_bytes()).unwrap();

    // Send the complete MAVLink message
    send_mavlink_message(&message);
}

/// Sends a MAVLink message over UART.
///
/// This function takes a byte slice containing a complete MAVLink message
/// and sends it over the UART interface.
///
/// The function uses unsafe code to access the global TX variable,
/// which holds the UART transmit interface. It iterates through each
/// byte of the message and writes it to the UART.
///
/// Parameters:
/// - message: A slice of bytes containing the MAVLink message to be sent.
///
/// Note: This function assumes that the global TX variable has been
/// properly initialized before being called.
fn send_mavlink_message(message: &[u8]) {
    unsafe {
        if let Some(tx) = TX.as_mut() {
            for &byte in message {
                let _ = tx.write(byte); // Write each byte to the UART
            }
        }
    }
}

/// Generates the next sequence number for MAVLink messages.
///
/// This function is responsible for maintaining and incrementing
/// the global sequence number used in MAVLink communication.
///
/// The sequence number is an 8-bit value that wraps around to 0
/// after reaching 255. This wrapping behavior ensures that the
/// sequence number always stays within the valid range of 0-255.
///
/// Returns:
/// - The next sequence number as a u8.
///
/// Note: This function uses unsafe code to access and modify
/// the global SEQUENCE_NUMBER variable. Proper synchronization
/// should be ensured if used in a multi-threaded context.
fn get_next_sequence_number() -> u8 {
    unsafe {
        // Increment the sequence number, wrapping around to 0 after 255
        SEQUENCE_NUMBER = SEQUENCE_NUMBER.wrapping_add(1);
        SEQUENCE_NUMBER
    }
}

/// Calculates the CRC (Cyclic Redundancy Check) for a MAVLink message.
///
/// This function implements the CRC-16/MCRF4XX algorithm, also known as X25,
/// which is used in the MAVLink protocol to ensure message integrity.
///
/// The CRC calculation process:
/// 1. Starts with an initial CRC value of 0xFFFF.
/// 2. For each byte in the input data:
///    a. XORs the byte with the low byte of the current CRC value.
///    b. For each bit in the resulting byte:
///       - If the least significant bit of the CRC is 1, shifts right
///         and XORs with the polynomial 0x8408.
///       - Otherwise, just shifts right.
/// 3. Inverts the final CRC value before returning.
///
/// Parameters:
/// - data: A slice of bytes for which to calculate the CRC.
///
/// Returns:
/// - The calculated CRC as a u16.
fn calculate_crc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF; // Initial CRC value
    for &byte in data {
        crc ^= byte as u16; // XOR byte with low byte of CRC
        for _ in 0..8 {
            if (crc & 0x0001) != 0 { // If LSB is set
                crc = (crc >> 1) ^ 0x8408; // Shift right and XOR with polynomial
            } else {
                crc >>= 1; // Just shift right
            }
        }
    }
    !crc // Invert final CRC value
}