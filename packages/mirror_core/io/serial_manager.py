#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Serial Manager for Mirror Body Animations
Handles threaded serial communication with ESP32
"""

import serial
import threading
import time
import queue
import serial.tools.list_ports
try:
    from ..simulation.mock_serial import MockSerial, get_virtual_device_instance
except ImportError:
    MockSerial = None


class SerialManager:
    def __init__(self, port='AUTO', baudrate=460800):
        self.port = port
        self.baudrate = baudrate
        self.ser = None  # This is what the code expects
        self.connected = False
        self.running = True
        self.last_error = None

        # Threaded communication
        self.send_queue = queue.Queue()
        self.receive_thread = None

        # Connect to serial port
        self.connect()

        # Start communication thread - DISABLED TEMPORARILY FOR STABILITY
        # if self.connected:
        #     self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
        #     self.receive_thread.start()

    def _attempt_port(self, port_name, accept_partial=False):
        """Attempt to open a specific port and wait for READY."""
        ser = None
        try:
            # Check if this is ESP32-S3 native USB (VID 303A)
            port_info = next((p for p in serial.tools.list_ports.comports() if p.device == port_name), None)
            is_esp32s3_native = port_info and port_info.hwid and "303a" in port_info.hwid.lower()
            
            ser = serial.Serial(port_name, self.baudrate, timeout=1, write_timeout=0.1)

            if is_esp32s3_native:
                # ESP32-S3 native USB - don't toggle DTR/RTS as it causes issues
                print(f"[INFO] ESP32-S3 native USB detected on {port_name}")
                ser.dtr = False
                ser.rts = False
                time.sleep(2)  # Longer wait for ESP32-S3 native USB boot
            else:
                # Standard USB-to-serial bridge - use DTR/RTS reset
                ser.dtr = False
                ser.rts = False
                time.sleep(0.1)
                ser.dtr = True  # Assert DTR to reset
                ser.rts = True
                time.sleep(0.1)
                ser.dtr = False  # Release
                ser.rts = False
                time.sleep(1)  # Wait for boot
                time.sleep(1)  # Wait for connection to stabilize

            ready_received = False
            bootloader_detected = False
            start_time = time.time()
            timeout = 12 if is_esp32s3_native else 10  # Longer timeout for ESP32-S3
            
            while time.time() - start_time < timeout:
                if ser.in_waiting:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                    except Exception:
                        line = ''
                    if line:
                        print(f"[SERIAL:{port_name}] {line}")
                        
                        # Check for bootloader mode (firmware not running)
                        if 'waiting for download' in line.lower() or 'download(usb' in line.lower():
                            print(f"[ERROR] ESP32-S3 on {port_name} is in BOOTLOADER mode!")
                            print(f"[ERROR] Firmware needs to be flashed. Run: pio run -t upload")
                            bootloader_detected = True
                            break
                            
                    if 'READY' in line.upper():
                        ready_received = True
                        break
                time.sleep(0.1)

            if bootloader_detected:
                self.last_error = f"ESP32-S3 on {port_name} is in bootloader mode - firmware not flashed"
                ser.close()
                return False

            if ready_received or accept_partial:
                if ready_received:
                    print(f"[OK] Connected to ESP32 on {port_name}")
                else:
                    print(f"[WARN] No READY from {port_name}, using port anyway")
                if self.ser and self.ser is not ser:
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                self.ser = ser
                self.port = port_name
                self.connected = True
                self.last_error = None
                return True

            ser.close()
            return False
        except Exception as e:
            print(f"[ERROR] Serial connection failed on {port_name}: {e}")
            if ser:
                try:
                    ser.close()
                except Exception:
                    pass
            return False
    def connect(self):
        """Connect to ESP32 serial port"""
        try:
            if self.port == 'AUTO':
                ports = list(serial.tools.list_ports.comports())
                if not ports:
                    print("[ERROR] No serial ports detected - is the ESP32 connected?")
                    self.last_error = "No serial ports detected"
                    return False

                print("[INFO] Available serial ports:")
                for port in ports:
                    print(f"   - {port.device}: {port.description} | HWID: {port.hwid}")

                # ESP32-S3 native USB uses VID 303A (Espressif), PID 1001
                # Check for this first as it's the most reliable indicator
                esp32s3_vid = "303a"  # Espressif VID for ESP32-S3 native USB
                keywords = ("cp210", "ch340", "usb", "silicon", "uart", "esp32", "wch", "ftdi", "serial")
                skip_terms = ("bluetooth",)

                candidates = []
                for port in ports:
                    desc_lower = port.description.lower() if port.description else ""
                    hwid_lower = port.hwid.lower() if getattr(port, "hwid", None) else ""
                    if any(term in desc_lower for term in skip_terms):
                        continue
                    # Score: lower is better
                    # -1 = ESP32-S3 native USB (best match - VID 303A)
                    #  0 = Known ESP32 USB bridge (CP210x, CH340, etc)
                    #  1 = Generic USB device
                    score = 1
                    if esp32s3_vid in hwid_lower:  # ESP32-S3 native USB
                        score = -1
                        print(f"[INFO] Detected ESP32-S3 native USB on {port.device}")
                    elif any(keyword in desc_lower or keyword in hwid_lower for keyword in keywords):
                        score = 0
                    candidates.append((score, port.device))

                if not candidates:
                    # Everything was skipped (likely Bluetooth). Fall back to full list.
                    candidates = [(1, port.device) for port in ports]

                candidates.sort(key=lambda item: (item[0], item[1]))

                fallback_port = None
                for score, port_name in candidates:
                    if self._attempt_port(port_name):
                        return True
                    if fallback_port is None:
                        fallback_port = port_name

                if fallback_port and self._attempt_port(fallback_port, accept_partial=True):
                    print(f"[WARN] Using fallback port {fallback_port}. Set config['led_serial_port'] to lock it.")
                    return True

                print("[ERROR] Unable to connect to any serial port. Set config['led_serial_port'] manually.")
                self.last_error = "Auto-detect failed"
                self.connected = False
                return False

            if self.port == 'SIMULATOR':
                if MockSerial is None:
                    print("[ERROR] Simulation package not found")
                    return False
                self.ser = MockSerial(self.port, self.baudrate)
                self.connected = True
                print(f"[OK] Simulation started on {self.port}")
                return True

            return self._attempt_port(self.port, accept_partial=True)

        except Exception as e:
            print(f"[ERROR] Serial connection failed: {e}")
            self.last_error = f"Serial connection failed: {e}"
            self.connected = False
            return False

    def _reconnect(self):
        """Best-effort reconnect used after write failures."""
        try:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None
            self.connected = False
            return self.connect()
        except Exception as e:
            self.last_error = f"Reconnect failed: {e}"
            self.connected = False
            return False

    def _receive_loop(self):
        """Background thread to handle incoming serial data"""
        while self.running and self.ser:
            try:
                if self.ser.in_waiting:
                    data = self.ser.readline().decode('utf-8').strip()
                    if data:
                        print(f"ESP32: {data}")
            except:
                pass
            time.sleep(0.01)

    def send_servo(self, packet):
        """Send servo packet to ESP32 with defensive error handling"""
        if not self.connected:
            # Log warning so user knows packets are being dropped
            print("[WARN] Servo packet dropped - serial not connected!")
            return False
            
        if not self.ser:
            print("[WARN] Serial port object is None")
            self.connected = False
            return False
        
        # CRITICAL: Check if port is actually open before writing
        try:
            if not self.ser.is_open:
                print("[WARN] Serial port closed unexpectedly - attempting reconnect")
                self.connected = False
                # Try to reopen
                try:
                    self.ser.open()
                    self.connected = True
                    print("[OK] Serial port reopened")
                except Exception as e:
                    print(f"[ERROR] Failed to reopen serial port: {e}")
                    return False
        except Exception as e:
            print(f"[WARN] Error checking serial port state: {e}")
            self.connected = False
            return False
        
        # Now attempt the write with full exception handling
        try:
            # DEFENSIVE: One final check
            if self.ser and self.ser.is_open:
                self.ser.write(packet)
                return True
            else:
                print("[WARN] Port closed between check and write")
                self.connected = False
                return False
        except serial.SerialTimeoutException:
            print("[WARN] Serial write timed out (buffer full?) - skipping packet")
            try:
                self.ser.reset_output_buffer()
            except:
                pass
            return False
        except OSError as e:
            # Common for disconnected devices
            print(f"[ERROR] Serial device error: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"[ERROR] Servo send error: {e}")
            self.connected = False
            # Don't try to reconnect here - too risky
            return False

    def send_led(self, packet):
        """Send LED packet to ESP32 with defensive error handling"""
        if not self.connected:
            print("[WARN] LED packet dropped - serial not connected!")
            return False
            
        if not self.ser:
            print("[WARN] LED packet dropped - serial port object is None!")
            self.connected = False
            return False
        
        try:
            if not self.ser.is_open:
                self.connected = False
                return False
                
            self.ser.write(packet)
            self.last_error = None
            return True
        except serial.SerialTimeoutException:
            # Don't print for LED packets to avoid log spam, just skip
            return False
        except OSError as e:
            print(f"[ERROR] LED device error: {e}")
            self.last_error = f"LED device error: {e}"
            self.connected = False

            # One reconnect + retry attempt (handles WinError 22 / transient disconnects)
            if self._reconnect() and self.ser and self.ser.is_open:
                try:
                    self.ser.write(packet)
                    self.last_error = None
                    return True
                except Exception as e2:
                    self.last_error = f"LED retry failed: {e2}"
                    self.connected = False
            return False
        except Exception as e:
            print(f"[ERROR] LED send error: {e}")
            self.last_error = f"LED send error: {e}"
            self.connected = False
            return False

    def close(self):
        """Close serial connection"""
        self.running = False
        if self.ser:
            self.ser.close()
        self.connected = False

    def stop(self):
        """Alias for close() used by GUI shutdown"""
        self.close()

    def start(self):
        """Start the serial manager (for compatibility)"""
        pass

    def get_simulation_instance(self):
        """Get access to the underlying virtual ESP32 instance (if in simulation mode)"""
        if self.port == 'SIMULATOR':
            try:
                from ..simulation.mock_serial import get_virtual_device_instance
                return get_virtual_device_instance()
            except ImportError:
                return None
        return None
