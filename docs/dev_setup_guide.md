
# Comprehensive Development Setup Guide for Matek Wing F405 WTE

## Table of Contents

1. [Set up the Development Environment](#1-set-up-the-development-environment)
2. [Install DFU Tools](#2-install-dfu-tools)
3. [Set Up the Project](#3-set-up-the-project)
4. [Configure the Build System](#4-configure-the-build-system)
5. [Write the Program](#5-write-the-program)
6. [Build the Project](#6-build-the-project)
7. [Prepare for Flashing](#7-prepare-for-flashing)
8. [Flash the Program](#8-flash-the-program)
9. [Verify and Debug](#9-verify-and-debug)

## 1. Set up the Development Environment

### a. Install Rust and Rust Rover IDE

1. Download and install Rust from https://www.rust-lang.org/tools/install
2. Follow the installation prompts, ensuring Rust is added to your system PATH
3. Download and install Rust Rover IDE from the JetBrains website
4. Open Rust Rover IDE and ensure it recognizes your Rust installation

### b. Install the GNU Arm Embedded Toolchain

1. Go to https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
2. Download the installer for your operating system
3. Run the installer, ensuring "Add path to environment variable" is checked
4. Open a new command prompt and verify the installation by running:
   ```
   arm-none-eabi-gcc --version
   ```

## 2. Install DFU Tools

### a. Install dfu-util

1. Here's a step-by-step process to download and install this version:

   Download the file:
   
   Go to the dfu-util download page (http://dfu-util.sourceforge.net/releases/)
   Click on "dfu-util-0.11-binaries.tar.xz" to download it

   Extract the archive:
   
   You'll need a program that can handle .tar.xz files. 7-Zip is a good free option if you don't already have one.
   Install 7-Zip if you don't have it (from https://www.7-zip.org/)
   Right-click on the downloaded .tar.xz file and choose "7-Zip > Extract Here"
   You may need to extract twice, once for the .tar and once for the resulting folder

   Locate the Windows binaries:
   
   In the extracted folder, navigate to the Windows subfolder
   You should see files like dfu-util.exe, dfu-suffix.exe, etc.

   Move the binaries:
   
   Create a new folder in a convenient location, e.g., C:\dfu-util
   Copy all the .exe files from the extracted folder to this new folder

   Add to PATH:
   
   Follow the steps I provided earlier to add C:\dfu-util (or wherever you placed the files) to your system PATH

   Verify the installation:
   
   Open a new Command Prompt
   Type dfu-util --version and press Enter
   You should see the version information for dfu-util

2. For macOS:
   ```
   brew install dfu-util
   ```

3. For Linux:
   ```
   sudo apt-get install dfu-util
   ```

### b. Install ST-Link drivers

1. Visit https://www.st.com/en/development-tools/stsw-link009.html
2. Download the driver package
3. Extract the downloaded file and run the installer
4. Follow the prompts to complete the installation

## 3. Set Up the Project

### a. Create a new project in Rust Rover IDE

1. Open Rust Rover IDE
2. Go to File > New Project
3. Select "Rust" and then "Binary (application)"
4. Name your project (e.g., "matek_wing_f405_project") and choose a location
5. Click "Create" to generate the project

### b. Configure the project for embedded development

1. Open the Cargo.toml file in your project root
2. Replace its contents with:

```toml
[package]
name = "matek_wing_f405_project"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = "0.7.7"
cortex-m-rt = "0.7.3"
panic-halt = "0.2.0"
stm32f4xx-hal = { version = "0.15.0", features = ["stm32f405", "rt"] }

[[bin]]
name = "matek_wing_f405_project"
test = false
bench = false

[profile.release]
codegen-units = 1
debug = true
lto = true
```

3. Create a new file named `.cargo/config.toml` in your project root with the following content:

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[build]
target = "thumbv7em-none-eabihf"
```

## 4. Configure the Build System

### a. Add the thumbv7em-none-eabihf target

1. Open a terminal in Rust Rover (View > Tool Windows > Terminal)
2. Run the following command:
   ```
   rustup target add thumbv7em-none-eabihf
   ```

## 5. Write the Program

Replace the contents of src/main.rs with the following program that uses UART to send messages:

```rust
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
```

## 6. Build the Project

### a. Compile the program

In the Rust Rover terminal, run:
```
cargo build --release
```

## 7. Prepare for Flashing

### a. Generate binary file

After building your project, generate a binary file:

```
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/matek_wing_f405_project matek_wing_f405_project.bin
```

### b. Connect the Matek Wing F405 WTE to your computer

Connect the board directly to your computer using a USB cable. Use the USB port on the board labeled "USB" or "BOOT".

## 8. Flash the Program

### a. Put the board in DFU mode

1. Disconnect the USB cable from the board
2. Press and hold the BOOT button on the board
3. While holding the BOOT button, reconnect the USB cable
4. Release the BOOT button

### b. Verify DFU mode

In a terminal, run:

```
dfu-util -l
```




You should see a device listed with "STM32 BOOTLOADER" or similar.

### c. Flash the binary

Run the following command:

```
dfu-util -a 0 -s 0x08000000:leave -D matek_wing_f405_project.bin
```

This command uploads your binary to the correct address and tells the board to start running the new program after flashing.

## 9. Verify and Debug

### a. Check the program output

1. Connect a USB-to-UART converter to your computer and the Matek Wing F405 WTE board:
    - Connect GND of the converter to a GND pin on the board
    - Connect the RX of the converter to the TX pin (PA9) on the board
    - Connect the TX of the converter to the RX pin (PA10) on the board

2. Open a serial terminal program (like PuTTY, screen, or Arduino IDE's Serial Monitor) on your computer.

3. Configure the serial connection with the following settings:
    - Baud rate: 115200
    - Data bits: 8
    - Stop bits: 1
    - Parity: None
    - Flow control: None

4. You should see "Hello, Matek Wing F405 WTE! Count: X" messages appearing every second, with X incrementing each time.

### b. Advanced Debugging (if necessary)

1. Install OpenOCD:
    - Windows: Download from http://openocd.org/ and add to PATH
    - macOS: `brew install openocd`
    - Linux: `sudo apt-get install openocd`

2. Create an OpenOCD configuration file named `openocd.cfg` in your project root:

```
source [find interface/stlink.cfg]
source [find target/stm32f4x.cfg]
```

3. Start OpenOCD in a terminal:

```
openocd
```

4. In another terminal, start GDB:

```
arm-none-eabi-gdb target/thumbv7em-none-eabihf/release/matek_wing_f405_project
```

5. In the GDB prompt, connect to OpenOCD:

```
target remote :3333
```

6. Use GDB commands to set breakpoints, step through code, and inspect variables

Remember to modify your code in Rust Rover, rebuild, and reflash as needed during the development process. This guide provides a complete setup for developing, flashing, and testing custom firmware on the Matek Wing F405 WTE board using Rust and DFU mode.