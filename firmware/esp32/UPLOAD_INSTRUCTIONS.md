# Upload Motor Firmware to ESP32-S3

## Quick Upload

### Method 1: Using Batch Script (Easiest)

1. **Put ESP32 in bootloader mode:**
   - Hold the **BOOT** button
   - Press and release **RESET** button
   - Release **BOOT** button

2. **Run the upload script:**
   ```bash
   upload_motor_firmware.bat
   ```

### Method 2: Using PlatformIO

1. **Put ESP32 in bootloader mode** (same as above)

2. **Upload firmware:**
   ```bash
   pio run --target upload --upload-port COM8
   ```

## Bootloader Mode Steps

The ESP32-S3 needs to be in **download/bootloader mode** to accept new firmware:

1. **Locate the buttons:**
   - **BOOT** button (usually labeled "BOOT" or "IO0")
   - **RESET** button (usually labeled "RST" or "RESET")

2. **Enter bootloader mode:**
   - **Hold** the BOOT button
   - **Press and release** the RESET button (while still holding BOOT)
   - **Release** the BOOT button

3. **Verify:**
   - ESP32 should now be in bootloader mode
   - You have about 10 seconds to start the upload

## Troubleshooting

### "Wrong boot mode detected"
- **Solution**: Put ESP32 in bootloader mode (see steps above)
- Make sure you release BOOT button AFTER pressing RESET

### "Failed to connect"
- **Check**: COM port is correct (COM8)
- **Check**: USB cable is connected
- **Check**: No other program is using COM8 (close Serial Monitor, Arduino IDE, etc.)

### "Port not found"
- **Check**: Device Manager for correct COM port
- **Try**: Different USB port or cable
- **Check**: ESP32 drivers are installed

### Upload starts but fails
- **Try**: Hold BOOT button during entire upload
- **Try**: Lower upload speed in platformio.ini
- **Try**: Different USB cable (some cables don't support data transfer)

## After Upload

Once upload is successful:

1. **ESP32 will automatically reset**
2. **Open Serial Monitor** (460800 baud) to see:
   ```
   === MIRROR HYBRID - ESP32 ===
   Body + Hand Tracking v1.0
   Init servos (32 channels)...
   READY! Waiting for data...
   ```

3. **Run Motor GUI:**
   ```bash
   python motors/gui/motor_gui.py
   ```

## Firmware Details

- **Board**: ESP32-S3 DevKit
- **Servos**: 32 servos (2x PCA9685)
- **Baud Rate**: 460800
- **Packet Type**: 0x02 (Motor/Servo data)
- **Packet Size**: 67 bytes (header + 32 servos × 2 bytes)

## Manual Upload (Alternative)

If PlatformIO doesn't work, you can use esptool directly:

```bash
# Put ESP32 in bootloader mode first!

esptool.py --chip esp32s3 --port COM8 --baud 460800 write_flash -z 0x10000 .pio\build\esp32s3\firmware.bin
```

## Status

- ✅ Firmware compiled successfully
- ⏳ Waiting for ESP32 bootloader mode
- ⏳ Ready to upload when ESP32 is in bootloader mode

