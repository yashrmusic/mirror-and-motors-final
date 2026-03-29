#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED Controller for Mirror Body Animations
Handles LED matrix rendering and packet packing for ESP32 control

Hardware Configuration:
- 8 panels of 16x16 LEDs = 2048 total LEDs  
- GPIO 5 (LEFT): Controls 1024 LEDs
- GPIO 18 (RIGHT): Controls 1024 LEDs
- WS2812B panels in a 2-column x 4-row grid

MAPPING MODES:
- Mode 0: RAW (no transformation)
- Mode 1: Row-based pin split (rows 0-31 → Pin5, rows 32-63 → Pin18)
- Mode 2: Column-based pin split (left column → Pin5, right column → Pin18)
- Mode 3: Column-based + serpentine within panels
- Mode 4: Full custom mapping
"""

import numpy as np
import cv2


class LEDController:
    # Panel configuration
    PANEL_WIDTH = 16
    PANEL_HEIGHT = 16
    PANELS_COLS = 2
    PANELS_ROWS = 4
    LEDS_PER_PIN = 1024
    
    # Mapping modes
    MODE_RAW = 0
    MODE_ROW_SPLIT = 1
    MODE_COLUMN_SPLIT = 2
    MODE_COLUMN_SERPENTINE = 3
    MODE_FULL_CUSTOM = 4
    
    def __init__(self, width=32, height=64, mapping_mode=3):
        self.width = width
        self.height = height
        
        # MAPPING CONFIGURATION
        # Try different modes until you find the one that works!
        self.mapping_mode = mapping_mode
        
        # Legacy settings (used by some modes)
        self.flip_x = False
        self.flip_y = False
        
        # Per-panel serpentine (common in WS2812B matrices)
        self.serpentine_rows = True
        
        # Panel order on each pin (for MODE_COLUMN_SPLIT and MODE_COLUMN_SERPENTINE)
        # Left pin (GPIO 5) has panels 1,3,5,7 from top to bottom
        # Right pin (GPIO 18) has panels 2,4,6,8 from top to bottom
        self.left_pin_panels = [1, 3, 5, 7]   # Panels on GPIO 5
        self.right_pin_panels = [2, 4, 6, 8]  # Panels on GPIO 18
        
        # Auto-detected mapping (loaded from file if exists)
        self.panel_mapping = None
        self._load_calibration_mapping()
        
        # Print current mode
        mode_names = {0: "RAW", 1: "ROW_SPLIT", 2: "COLUMN_SPLIT", 
                      3: "COLUMN_SERPENTINE", 4: "FULL_CUSTOM", 5: "AUTO_CALIBRATED"}
        if self.panel_mapping:
            self.mapping_mode = 5  # Use auto-calibrated mode
        print(f"[LEDController] Mapping mode: {mode_names.get(self.mapping_mode, 'UNKNOWN')}")
    
    def _load_calibration_mapping(self):
        """Load auto-calibrated mapping from JSON file if it exists."""
        import json
        from pathlib import Path
        
        # Look for mapping file in data directory
        possible_paths = [
            Path(__file__).parents[3] / "data" / "led_mapping.json",
            Path("data/led_mapping.json"),
        ]
        
        for mapping_path in possible_paths:
            if mapping_path.exists():
                try:
                    with open(mapping_path, 'r') as f:
                        config = json.load(f)
                    
                    if "mapping" in config:
                        self.panel_mapping = {
                            int(k): v for k, v in config["mapping"].items()
                        }
                        print(f"[LEDController] Loaded calibration from {mapping_path}")
                        print(f"[LEDController] Mapping: {self.panel_mapping}")
                        return
                except Exception as e:
                    print(f"[LEDController] Failed to load mapping: {e}")

    def remap_for_hardware(self, frame):
        """
        Remap LED frame to match physical hardware wiring.
        
        Args:
            frame: 64x32 numpy array (height x width)
        Returns:
            Remapped 64x32 numpy array ready for transmission
        """
        if self.mapping_mode == self.MODE_RAW:
            return frame.copy()
            
        elif self.mapping_mode == self.MODE_ROW_SPLIT:
            # Original simple mapping with optional flips
            remapped = frame.copy()
            if self.flip_y:
                remapped = np.flip(remapped, axis=0)
            if self.flip_x:
                remapped = np.flip(remapped, axis=1)
            return remapped
            
        elif self.mapping_mode == self.MODE_COLUMN_SPLIT:
            # Column-based pin split (left column → first 1024, right column → second 1024)
            return self._remap_column_split(frame, serpentine=False)
            
        elif self.mapping_mode == self.MODE_COLUMN_SERPENTINE:
            # Column-based + serpentine within panels
            return self._remap_column_split(frame, serpentine=True)
            
        elif self.mapping_mode == self.MODE_FULL_CUSTOM:
            return self._remap_full_custom(frame)
        
        elif self.mapping_mode == 5 and self.panel_mapping:
            # AUTO_CALIBRATED mode: use detected panel mapping
            return self._remap_auto_calibrated(frame)
            
        else:
            return frame.copy()
    
    def _remap_auto_calibrated(self, frame):
        """
        Remap frame using auto-calibrated panel positions.
        Uses self.panel_mapping: {logical_panel -> physical_position}
        """
        output = np.zeros_like(frame)
        
        for logical_panel in range(1, 9):
            physical_pos = self.panel_mapping.get(logical_panel, logical_panel)
            
            # Source: where we READ from (logical layout)
            src_row = (logical_panel - 1) // 2
            src_col = (logical_panel - 1) % 2
            src_y = src_row * 16
            src_x = src_col * 16
            
            # Destination: where we WRITE to (physical layout)
            dst_row = (physical_pos - 1) // 2
            dst_col = (physical_pos - 1) % 2
            dst_y = dst_row * 16
            dst_x = dst_col * 16
            
            # Copy panel
            output[dst_y:dst_y+16, dst_x:dst_x+16] = frame[src_y:src_y+16, src_x:src_x+16]
        
        return output
    
    def _remap_column_split(self, frame, serpentine=True):
        """
        Remap for column-based pin split:
        - Left column (panels 1,3,5,7) → indices 0-1023
        - Right column (panels 2,4,6,8) → indices 1024-2047
        
        Within each panel, rows may be serpentine (alternate direction)
        """
        output = np.zeros((self.height, self.width), dtype=np.uint8)
        
        # Process left column (x: 0-15) → first half of output
        for panel_idx, panel_num in enumerate(self.left_pin_panels):
            src_row = (panel_num - 1) // 2  # Source panel row
            src_col = (panel_num - 1) % 2   # Should be 0 for left panels
            
            src_y_start = src_row * self.PANEL_HEIGHT
            src_x_start = src_col * self.PANEL_WIDTH
            
            # Destination is sequential panels in first half
            dst_y_start = panel_idx * self.PANEL_HEIGHT
            dst_x_start = 0
            
            for local_y in range(self.PANEL_HEIGHT):
                for local_x in range(self.PANEL_WIDTH):
                    src_y = src_y_start + local_y
                    src_x = src_x_start + local_x
                    
                    # Apply serpentine if needed
                    if serpentine and (local_y & 1):  # Odd rows reversed
                        dst_local_x = (self.PANEL_WIDTH - 1) - local_x
                    else:
                        dst_local_x = local_x
                    
                    dst_y = dst_y_start + local_y
                    dst_x = dst_x_start + dst_local_x
                    
                    output[dst_y, dst_x] = frame[src_y, src_x]
        
        # Process right column (x: 16-31) → second half of output (y: 0-63, x: 16-31)
        for panel_idx, panel_num in enumerate(self.right_pin_panels):
            src_row = (panel_num - 1) // 2
            src_col = (panel_num - 1) % 2  # Should be 1 for right panels
            
            src_y_start = src_row * self.PANEL_HEIGHT
            src_x_start = src_col * self.PANEL_WIDTH
            
            # Destination is sequential panels in second half (starts at x=16)
            dst_y_start = panel_idx * self.PANEL_HEIGHT
            dst_x_start = 16
            
            for local_y in range(self.PANEL_HEIGHT):
                for local_x in range(self.PANEL_WIDTH):
                    src_y = src_y_start + local_y
                    src_x = src_x_start + local_x
                    
                    if serpentine and (local_y & 1):
                        dst_local_x = (self.PANEL_WIDTH - 1) - local_x
                    else:
                        dst_local_x = local_x
                    
                    dst_y = dst_y_start + local_y
                    dst_x = dst_x_start + dst_local_x
                    
                    output[dst_y, dst_x] = frame[src_y, src_x]
        
        return output
    
    def _remap_full_custom(self, frame):
        """
        Full custom pixel-by-pixel remapping.
        Override this method for completely custom wiring.
        """
        # For now, just apply flips
        remapped = frame.copy()
        if self.flip_y:
            remapped = np.flip(remapped, axis=0)
        if self.flip_x:
            remapped = np.flip(remapped, axis=1)
        return remapped

    def render_frame(self, pose_results, seg_mask):
        """
        Render LED frame from pose results and segmentation mask
        Creates a silhouette for the LED matrix
        """
        led_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        try:
            if pose_results and pose_results.pose_landmarks:
                # Use segmentation mask if available and valid
                if seg_mask is not None and hasattr(seg_mask, 'shape') and len(seg_mask.shape) >= 2:
                    try:
                        # Resize with explicit type handling
                        mask_resized = cv2.resize(seg_mask, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
                        binary_mask = (mask_resized > 0.5).astype(np.uint8) * 255
                        led_frame[:, :, 0] = binary_mask
                        led_frame[:, :, 1] = binary_mask
                        led_frame[:, :, 2] = binary_mask
                    except Exception as e:
                        print(f"[LEDController] Mask resize failed: {e}")
                        # Fallback to landmarks
                        self._render_landmarks(led_frame, pose_results)
                else:
                    # Fallback: draw pose landmarks as silhouette
                    self._render_landmarks(led_frame, pose_results)
        except Exception as e:
            print(f"[LEDController] render_frame error: {e}")
            
        return led_frame

    def _render_landmarks(self, led_frame, pose_results):
        """Helper to render landmarks when mask fails"""
        h, w = self.height, self.width
        for landmark in pose_results.pose_landmarks.landmark:
            if landmark.visibility > 0.6:  # Lowered threshold slightly
                x = int(landmark.x * w)
                y = int(landmark.y * h)
                # Expand to 3x3 dot for visibility
                if 0 <= x < w and 0 <= y < h:
                    led_frame[y, x] = [255, 255, 255]
                    # Draw minimal cross pattern
                    if x > 0: led_frame[y, x-1] = [100, 100, 100]
                    if x < w-1: led_frame[y, x+1] = [100, 100, 100]
                    if y > 0: led_frame[y-1, x] = [100, 100, 100]
                    if y < h-1: led_frame[y+1, x] = [100, 100, 100]

    def pack_led_packet(self, led_frame):
        """
        Pack LED frame into firmware-compatible packet.
        Firmware expects:
          [0xAA, 0xBB, 0x01, 2048 bytes of brightness]
        """
        if led_frame.shape[:2] != (self.height, self.width):
            raise ValueError(f"LED frame must be {self.height}x{self.width}, got {led_frame.shape[:2]}")

        # Convert to grayscale brightness
        if led_frame.ndim == 3 and led_frame.shape[2] == 3:
            brightness = led_frame.max(axis=2)
        else:
            brightness = led_frame

        brightness = np.clip(brightness, 0, 255).astype(np.uint8)
        
        # Apply hardware mapping before transmission
        brightness = self.remap_for_hardware(brightness)

        # Flatten in row-major order
        flat = brightness.flatten().tolist()
        if len(flat) != self.width * self.height:
            raise ValueError("LED frame does not contain expected number of pixels")

        packet = [0xAA, 0xBB, 0x01]
        packet.extend(int(v) for v in flat)

        return bytes(packet)

    def pack_led_packet_1bit(self, led_frame, threshold=128):
        """
        Pack LED frame into 1-bit compressed packet.
        8x smaller than full brightness! (256 bytes vs 2048 bytes)
        
        Firmware expects:
          [0xAA, 0xBB, 0x03, 256 bytes of packed bits]
        
        Each byte contains 8 pixels (MSB first)
        Pixel order: row-major, left to right, top to bottom
        """
        if led_frame.shape[:2] != (self.height, self.width):
            raise ValueError(f"LED frame must be {self.height}x{self.width}")

        # Convert to grayscale if needed
        if led_frame.ndim == 3 and led_frame.shape[2] == 3:
            brightness = led_frame.max(axis=2)
        else:
            brightness = led_frame

        # Apply hardware mapping
        brightness = self.remap_for_hardware(brightness)
        
        # Threshold to binary
        binary = (brightness > threshold).astype(np.uint8)
        flat = binary.flatten()
        
        # Pack 8 pixels per byte
        packed = []
        for i in range(0, len(flat), 8):
            byte_val = 0
            for bit in range(8):
                if i + bit < len(flat) and flat[i + bit]:
                    byte_val |= (1 << (7 - bit))
            packed.append(byte_val)
        
        # Header: 0xAA 0xBB 0x03 (0x03 = 1-bit mode)
        packet = bytes([0xAA, 0xBB, 0x03]) + bytes(packed)
        return packet

    def pack_led_packet_rle(self, led_frame, threshold=128):
        """
        Pack LED frame using Run-Length Encoding.
        Extremely efficient for silhouettes with large solid areas.
        
        Firmware expects:
          [0xAA, 0xBB, 0x04, length_hi, length_lo, RLE data...]
        
        RLE format: alternating (count, value) pairs
        - count: 1-255 (0 = end marker)
        - value: 0 or 255
        """
        if led_frame.shape[:2] != (self.height, self.width):
            raise ValueError(f"LED frame must be {self.height}x{self.width}")

        # Convert to grayscale if needed
        if led_frame.ndim == 3 and led_frame.shape[2] == 3:
            brightness = led_frame.max(axis=2)
        else:
            brightness = led_frame

        # Apply hardware mapping
        brightness = self.remap_for_hardware(brightness)
        
        # Threshold to binary (0 or 255)
        binary = ((brightness > threshold).astype(np.uint8)) * 255
        flat = binary.flatten()
        
        # RLE encode
        rle = []
        i = 0
        while i < len(flat):
            val = flat[i]
            count = 0
            while i < len(flat) and flat[i] == val and count < 255:
                count += 1
                i += 1
            rle.extend([count, val])
        
        # Header: 0xAA 0xBB 0x04 length(2 bytes) data...
        rle_len = len(rle)
        packet = bytes([0xAA, 0xBB, 0x04, (rle_len >> 8) & 0xFF, rle_len & 0xFF]) + bytes(rle)
        return packet


# Quick test
if __name__ == "__main__":
    print("Testing LED Controller modes...")
    
    for mode in range(5):
        led = LEDController(mapping_mode=mode)
        pattern = np.zeros((64, 32), dtype=np.uint8)
        pattern[0:16, 0:16] = 255  # Light panel 1
        packet = led.pack_led_packet(pattern)
        print(f"  Mode {mode}: Packet size = {len(packet)}")
    
    print("Done!")
