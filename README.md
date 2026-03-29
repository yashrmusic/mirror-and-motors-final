# Kinetic Mirror - LED Control System

A high-performance kinetic art display utilizing real-time body silhouette tracking to drive a 32x64 LED matrix wall.

## 🚀 Quick Start

1. **Prerequisites**
   - Python 3.9+
   - USB Web Camera
   - ESP32-S3 with 32x64 WS2812B LED Matrix (Hardware Mode)

2. **Installation**
   ```bash
   pip install -r requirements.txt
   ```

3. **Run Application**
   Double-click `run.bat` or run:
   ```bash
   python -m apps.gui.main
   ```

## 🎮 Features

- **Optimized Silhouette Tracking**: Uses high-speed MediaPipe segmentation with temporal smoothing and morphological hole-filling for solid body detection.
- **Zero-Lag Processing**: Fully optimized pipeline with lower-resolution processing paths for real-time responsiveness on the LED wall.
- **Advanced Visualization**: Integrated LED simulator to preview the output before sending to hardware.
- **Hardware Integration**:
  - Auto-detection of ESP32-S3 devices.
  - Support for dual-pin LED output (GPIO 5 & 18).
  - High-speed 460800 baud serial communication.
- **Test Mode**: Built-in test patterns (Solid, Rainbow, Scan) for LED panel verification and troubleshooting.

## 🏗 System Architecture

- **GUI**: Modern dark-themed Tkinter interface.
- **Tracking Engine**: Custom `BodySegmenter` using MediaPipe Task API.
- **Data Path**: 32x64 silhouette mask -> Linear Serpentine Mapping -> Serial Packet.
- **Firmware**: Compatible with the provided ESP32-S3 LED control firmware.

---
© 2026 Hookkapaani Kinetic Arts
