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
arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/[PROJECT_NAME] [PROJECT_NAME].bin
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
dfu-util -a 0 -s 0x08000000:leave -D [PROJECT_NAME].bin
```
Replace [PROJECT_NAME] with your actual project name.

## 6. Verify (WiFi Method using PuTTY)

1. Power on the Matek Wing F405 WTE board
2. Connect your computer to the "ArduPilot" WiFi network
    - SSID: ArduPilot
    - Password: ardupilot
3. Open PuTTY
4. Configure PuTTY for UDP:
    - In the main PuTTY window, select "Connection type: Other"
    - In the "Connection type:" field, enter "UDP"
    - For "Host Name (or IP address)", enter "localhost"
    - For "Port", enter "14550"
5. Optional: Save the session for future use
    - Enter a name in the "Saved Sessions" field
    - Click "Save"
6. Click "Open" to start the UDP listener
7. You should now see messages from your Matek Wing F405 WTE board in the PuTTY terminal window

## Troubleshooting

- If flashing fails, re-enter DFU mode and try again
- Ensure correct binary name in commands
- Verify USB connection is stable during flashing
- Check that dfu-util is properly installed and in PATH
- For WiFi verification:
    - Ensure you're connected to the correct network
    - Check firewall settings if no data is received
    - Verify the board is powered and running the new program
    - If using PuTTY, make sure you've selected UDP and are using the correct port
    - Try restarting PuTTY if you don't see any messages