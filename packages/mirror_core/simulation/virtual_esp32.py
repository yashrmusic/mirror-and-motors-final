
"""
Virtual ESP32 Firmware
Mimics the behavior of the real ESP32 firmware for LEDs and Motors.
"""

import threading
import time
import queue
from collections import deque

class VirtualESP32:
    def __init__(self):
        self.running = True
        self.led_state = [0] * (2048)  # 2048 LEDs (brightness)
        self.motor_angles = [90] * 64   # 64 Servos (0-180 degrees)
        self.buffer = deque()
        self.state_lock = threading.Lock()
        
        # Output queue (to send data back to PC, e.g. "READY")
        self.output_queue = queue.Queue()
        
        # Simulate boot delay
        self.boot_timer = threading.Timer(1.0, self._boot_complete)
        self.boot_timer.start()

    def _boot_complete(self):
        """Called when 'boot' is complete"""
        self.output_queue.put(b"READY\n")

    def write(self, data):
        """Receive data from PC (bytes)"""
        with self.state_lock:
            self.buffer.extend(data)
            self._process_buffer()

    def read(self):
        """Send data to PC"""
        try:
            return self.output_queue.get_nowait()
        except queue.Empty:
            return None

    def _process_buffer(self):
        """Parse the internal buffer for packets"""
        # Simple finite state machine or just look for headers
        # Protocol: 
        #   Header: AA BB
        #   Type:   01 (LED) or 02 (Servo)
        #   Data:   ...
        
        while len(self.buffer) > 2:
             # Look for Header AA BB
            if self.buffer[0] != 0xAA or self.buffer[1] != 0xBB:
                 # Pop byte and continue
                self.buffer.popleft()
                continue
            
            # Found Header
            if len(self.buffer) < 3:
                return # Need more data
                
            packet_type = self.buffer[2]
            
            if packet_type == 0x01: # LED Packet
                # Needs 2048 bytes + 3 header bytes = 2051 bytes
                if len(self.buffer) < 2051:
                    return # Wait for more data
                
                # Extract LED data
                led_data = list(self.buffer)[3:2051]
                self.led_state = led_data # Update state
                
                # Consume packet
                # Consume packet
                for _ in range(2051):
                    self.buffer.popleft()
                
            elif packet_type == 0x02: # Servo Packet
                # Needs 64 * 2 bytes + 3 header bytes = 131 bytes
                # (Or dynamic length, but let's assume fixed 64 for now based on controller)
                num_servos = 64
                payload_size = num_servos * 2
                total_size = payload_size + 3
                
                if len(self.buffer) < total_size:
                    return # Wait for more data
                
                buf_list = list(self.buffer)
                
                # Extract Servo data
                # 64 servos, 2 bytes each (High byte, Low byte)
                # Value 0-1000 maps to 0-180 degrees
                new_angles = []
                for i in range(num_servos):
                    idx = 3 + (i * 2)
                    hi = buf_list[idx]
                    lo = buf_list[idx+1]
                    val = (hi << 8) | lo
                    val = max(0, min(1000, val))  # Bounds check
                    
                    # Map 0-1000 -> 0-180
                    angle = (val / 1000.0) * 180.0
                    new_angles.append(angle)
                
                # Ensure we have enough slots
                if len(self.motor_angles) < num_servos:
                     self.motor_angles = [90] * num_servos

                for i in range(min(len(new_angles), len(self.motor_angles))):
                    self.motor_angles[i] = new_angles[i]
                
                # Consume packet
                for _ in range(total_size):
                    self.buffer.popleft()
                
            else:
                 # Unknown packet type, skip header
                self.buffer.popleft()
                self.buffer.popleft()

    def get_server_state(self):
        """Thread-safe access to state for visualizer"""
        with self.state_lock:
            return {
                "leds": list(self.led_state),
                "motors": list(self.motor_angles)
            }
