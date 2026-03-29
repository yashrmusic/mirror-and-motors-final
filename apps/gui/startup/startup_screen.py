#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Startup Screen for Mirror Body Animations
Visual hardware detection and configuration
"""

import cv2
import serial.tools.list_ports
import json
from pathlib import Path

def run_startup_screen():
    """
    Run the startup screen for hardware detection
    Returns configuration dict or None if cancelled
    """
    print("\n🔍 Detecting hardware...")

    # Detect cameras
    cameras = []
    for i in range(5):  # Check first 5 camera indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                cameras.append(i)
            cap.release()

    # Detect ESP32 serial ports
    ports = serial.tools.list_ports.comports()
    esp32_ports = []
    esp32s3_port = None  # Track if we found an ESP32-S3 specifically
    
    for port in ports:
        desc = port.description or ""
        hwid = port.hwid or ""
        hwid_lower = hwid.lower()
        
        # Check for ESP32-S3 native USB first (VID 303A is Espressif)
        if "303a" in hwid_lower:
            esp32_ports.insert(0, port.device)  # Prioritize ESP32-S3
            esp32s3_port = port.device
            print(f"✅ ESP32-S3 native USB detected on {port.device}")
        elif 'CP210' in desc or 'CH340' in desc or 'USB' in desc:
            esp32_ports.append(port.device)

    # Load existing config
    repo_root = Path(__file__).resolve().parents[2]
    config_file = repo_root / "config" / "config.json"
    config_file.parent.mkdir(exist_ok=True)
    config = {}
    if config_file.exists():
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
        except:
            pass

    # Auto-select best options
    selected_camera = config.get('camera_index', cameras[0] if cameras else 0)
    selected_port = config.get('serial_port', esp32_ports[0] if esp32_ports else 'AUTO')

    print(f"📹 Cameras found: {cameras}")
    print(f"🔌 ESP32 ports found: {esp32_ports}")
    print(f"🎯 Selected camera: {selected_camera}")
    print(f"🎯 Selected port: {selected_port}")

    return {
        "esp32_type": "ESP32-S3",
        "esp32_port": selected_port,
        "cameras": cameras,
        "selected_camera": selected_camera,
        "ready": True,
        "config": config,
    }