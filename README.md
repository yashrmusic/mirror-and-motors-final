# Kinetic Mirror - Motor Control System

A high-performance kinetic sculpture control system utilizing real-time body silhouette tracking to drive an 8x8 matrix of servo motors.

## 🚀 Quick Start

1. **Prerequisites**
   - Python 3.9+
   - USB Web Camera
   - ESP32-S3 with PCA9685 Motor Drivers (Hardware Mode)

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
- **Zero-Lag Processing**: Fully optimized pipeline with lower-resolution processing paths and silenced console hot-paths for real-time responsiveness.
- **Advanced Visualization**: High-contrast Blue (Background) and Green (Human) overlay with 8x8 grid projection.
- **Hardware Integration**:
  - Auto-detection of ESP32-S3 devices.
  - Integrated firmware flasher for easy deployment.
  - Support for 64 servo motors across 4 PCA9685 drivers.
- **Manual & Test Modes**: Comprehensive tools for testing motor range, wiring verification, and custom wave patterns.

## 🏗 System Architecture

- **GUI**: Modern dark-themed Tkinter interface.
- **Tracking Engine**: Custom `BodySegmenter` using MediaPipe Task API.
- **Communication**: Optimized serial packet protocol (0xAA 0xBB header).
- **Firmware**: Compatible with the provided ESP32-S3 motor control firmware.

## 🛠 Calibration

- **Live Mode**: Motors will follow your silhouette. Ensure adequate lighting for the camera.
- **Test Mode**: Use the "Wave" or "Test" buttons to verify all 64 motors are responding correctly.

---
© 2026 Hookkapaani Kinetic Arts
