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

    def connect(self):
        """Connect to ESP32 serial port"""
        try:
            if self.port == 'AUTO':
                # Auto-detect ESP32 port
                ports = serial.tools.list_ports.comports()
                esp32_port = None
                if not ports:
                    print("[ERROR] No serial ports detected - is the ESP32 connected?")
                    self.last_error = "No serial ports detected"
                    return False

                print("[INFO] Available serial ports:")
                for port in ports:
                    print(f"   - {port.device}: {port.description}")

                keywords = ("cp210", "ch340", "usb", "silicon", "uart", "esp32", "wch", "ftdi")
                for port in ports:
                    desc_lower = port.description.lower() if port.description else ""
                    hwid_lower = port.hwid.lower() if getattr(port, "hwid", None) else ""
                    if any(keyword in desc_lower or keyword in hwid_lower for keyword in keywords):
                        esp32_port = port.device
                        break
                if not esp32_port:
                    esp32_port = ports[0].device
                    print(f"[WARN] No known ESP32 bridge detected. Falling back to {esp32_port}. "
                          "Set config['led_serial_port'] to override.")
                self.port = esp32_port

            if self.port == 'SIMULATOR':
                 if MockSerial is None:
                     print("[ERROR] Simulation package not found")
                     return False
                 self.ser = MockSerial(self.port, self.baudrate)
                 self.connected = True # Mock serial is always connected immediately
                 print(f"[OK] Simulation started on {self.port}")
                 return True

            self.ser = serial.Serial(self.port, self.baudrate, timeout=1, write_timeout=1.0)
            
            # Force ESP32 Reset via DTR/RTS
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.1)
            self.ser.dtr = True  # Assert DTR to reset
            self.ser.rts = True
            time.sleep(0.1)
            self.ser.dtr = False # Release
            self.ser.rts = False
            time.sleep(1)        # Wait for boot
            
            time.sleep(1)  # Wait for connection to stabilize

            # Test connection by waiting for READY
            ready_received = False
            start_time = time.time()
            while time.time() - start_time < 15:  # Wait up to 15 seconds for full ESP32 boot (WiFi + servos)
                try:
                    if self.ser.in_waiting:
                        line = self.ser.readline().decode('utf-8', errors='replace').strip()
                        if 'READY' in line:
                            ready_received = True
                            break
                except (OSError, serial.SerialException):
                    break  # Port disappeared during boot wait
                time.sleep(0.1)

            if ready_received:
                self.connected = True
                self.last_error = None
                print(f"[OK] Connected to ESP32 on {self.port}")
                return True
            else:
                print("[WARN] ESP32 connected but no READY signal - proceeding anyway")
                self.connected = True  # Proceed even without READY for compatibility
                self.last_error = None
                return True

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
                if not self.ser.is_open:
                    break
                if self.ser.in_waiting:
                    data = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if data:
                        print(f"ESP32: {data}")
            except (OSError, serial.SerialException):
                break  # Port disconnected
            except Exception:
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
            except Exception:
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
            try:
                self.ser.reset_output_buffer()
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
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
