# Matek Wing F405 WTE Compile and Upload Cheatsheet

## 1. Compile the Program

1. Open terminal in project root
2. Run:
   ```
   cargo build --release
   ```

## 2. Generate Binary

Run:
```
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/matek_wing_f405_project [SOFTWARE_NAME].bin
```
Replace [PROJECT_NAME] with your actual project name.

## 3. Enter DFU Mode

1. Disconnect USB
2. Hold BOOT button
3. Reconnect USB
4. Release BOOT button

## 4. Verify DFU Mode

Run:
```
dfu-util -l
```
Should see "STM32 BOOTLOADER" or similar.

## 5. Flash Binary

Run:
```
dfu-util -a 0 -s 0x08000000:leave -D [SOFTWARE_NAME].bin
```
Replace [PROJECT_NAME] with your actual project name.

## 6. Verify (WiFi Method)

1. Power on the Matek Wing F405 WTE board
2. Connect your computer to the "ArduPilot" WiFi network
    - SSID: ArduPilot
    - Password: ardupilot
3. Use a UDP listener on your computer:
    - Port: 14550
    - Command (Linux/macOS): `nc -ul 14550`
    - Windows: Use a tool like Hercules or PuTTY
4. Check for expected UDP messages from the board

## Troubleshooting

- If flashing fails, re-enter DFU mode and try again
- Ensure correct binary name in commands
- Verify USB connection is stable during flashing
- Check that dfu-util is properly installed and in PATH
- For WiFi verification:
    - Ensure you're connected to the correct network
    - Check firewall settings if no data is received
    - Verify the board is powered and running the new program