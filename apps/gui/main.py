"""
Dance Mirror - LED Control GUI
Real-time body silhouette tracking for LED wall

Camera always runs, LED output controlled by Launch/Stop
"""
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import queue
import serial
import serial.tools.list_ports
from PIL import Image, ImageTk
import numpy as np
import cv2
import mediapipe as mp
import time
from packages.mirror_core.controllers.led_controller import LEDController
from packages.mirror_core.simulation.mock_serial import MockSerial


class BodySegmenter:
    """
    Optimized body segmentation for LED wall
    - Clean silhouette extraction
    - Temporal smoothing (reduces flicker)
    - Hole filling (solid body)
    - Edge smoothing
    """
    def __init__(self):
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision
        from pathlib import Path
        import urllib.request
        
        model_path = Path("data/selfie_segmenter.tflite")
        model_path.parent.mkdir(exist_ok=True)
        
        if not model_path.exists():
            print("Downloading segmentation model...")
            url = "https://storage.googleapis.com/mediapipe-models/image_segmenter/selfie_segmenter/float16/latest/selfie_segmenter.tflite"
            urllib.request.urlretrieve(url, model_path)
            print("Model downloaded")
        
        base_options = mp_python.BaseOptions(model_asset_path=str(model_path))
        options = mp_vision.ImageSegmenterOptions(
            base_options=base_options,
            running_mode=mp_vision.RunningMode.VIDEO,
            output_category_mask=True
        )
        self.segmenter = mp_vision.ImageSegmenter.create_from_options(options)
        self.frame_count = 0
        
        # Temporal smoothing buffers
        self.mask_buffer = None
        self.smoothing = 0.10  # LOWER = more snappy/crisp, higher = smoother/more lag
        
        # Morphology kernels (pre-computed for speed)
        self.kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        self.kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
    def get_body_mask(self, frame):
        """
        Extract clean body silhouette with performance optimizations
        Returns binary mask (0 or 255)
        """
        self.frame_count += 1
        h, w = frame.shape[:2]
        
        # PERFORMANCE: Processing at lower resolution significantly reduces lag
        proc_w, proc_h = 256, 144
        small_rgb = cv2.resize(frame, (proc_w, proc_h), interpolation=cv2.INTER_LINEAR)
        small_rgb = cv2.cvtColor(small_rgb, cv2.COLOR_BGR2RGB)
        
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=small_rgb)
        
        # Run segmentation
        timestamp_ms = int(self.frame_count * 33)
        result = self.segmenter.segment_for_video(mp_image, timestamp_ms)
        
        if result.category_mask is None:
            return np.zeros((h, w), dtype=np.uint8)
        
        # Get raw mask at processing resolution
        mask = result.category_mask.numpy_view()
        mask = (mask > 0).astype(np.float32)
        
        # Temporal smoothing at low res (faster)
        if self.mask_buffer is None:
            self.mask_buffer = mask.copy()
        else:
            self.mask_buffer = self.smoothing * self.mask_buffer + (1.0 - self.smoothing) * mask
        
        binary = (self.mask_buffer > 0.4).astype(np.uint8) * 255
        
        # Faster Morphology at lower resolution
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.kernel_close)
        
        # Solidify (Fast at low res)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            binary = np.zeros_like(binary)
            cv2.drawContours(binary, [largest], -1, 255, cv2.FILLED)
        
        binary = cv2.dilate(binary, self.kernel_dilate, iterations=1)
        
        # Upscale mask back to original size for UI/matching
        return cv2.resize(binary, (w, h), interpolation=cv2.INTER_NEAREST)
    
    def get_led_mask(self, frame, led_width=32, led_height=64):
        """
        Get body mask directly resized for LED matrix
        Optimized single-call for LED output
        """
        # Get full resolution mask
        body_mask = self.get_body_mask(frame)
        
        # Resize to LED dimensions with area interpolation (best for downscaling)
        led_mask = cv2.resize(body_mask, (led_width, led_height), interpolation=cv2.INTER_AREA)
        
        # Final threshold to ensure clean binary
        led_mask = (led_mask > 128).astype(np.uint8) * 255
        
        return led_mask
    
    def close(self):
        self.segmenter.close()


class LEDSimulatorPanel:
    """Embedded LED Simulator display"""
    def __init__(self, parent):
        self.parent = parent
        self.led_state = [0] * 2048
        
        self.pixel_size = 6
        self.canvas = tk.Canvas(parent, width=32*6, height=64*6, bg="black")
        self.canvas.pack(pady=10, expand=True)
        self._update_loop()
    
    def update_leds(self, led_data):
        if led_data and len(led_data) == 2048:
            self.led_state = list(led_data)
    
    def clear(self):
        self.led_state = [0] * 2048
    
    def _update_loop(self):
        try:
            self.canvas.delete("all")
            for i, b in enumerate(self.led_state):
                if b > 5:
                    x, y = i % 32, i // 32
                    if y >= 64: break
                    c = int(min(255, max(0, b)))
                    color = f"#{c:02x}{c:02x}{c:02x}"
                    x1, y1 = x * self.pixel_size, y * self.pixel_size
                    self.canvas.create_rectangle(x1, y1, x1+5, y1+5, fill=color, outline="")
        except: pass
        self.parent.after(33, self._update_loop)


class LEDControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Dance Mirror")
        self.root.geometry("1100x650+50+50")
        
        # State
        self.running = True
        self.camera_running = False
        self.led_output_active = False
        self.test_mode = False  # When True, test patterns override body tracking
        self.serial_port = None
        self.cap = None
        self.body_segmenter = None
        
        # LED Controller - Use RAW mode (0) since firmware handles serpentine mapping
        self.led_controller = LEDController(width=32, height=64, mapping_mode=0)
        
        # Detect hardware
        print(f"[DEBUG] === Starting hardware detection ===")
        self.available_cameras = self._detect_cameras()
        print(f"[DEBUG] Available cameras: {self.available_cameras}")
        self.available_ports = self._detect_ports()
        print(f"[DEBUG] Available ports: {self.available_ports}")
        print(f"[DEBUG] === Hardware detection complete ===")
        
        # Build UI
        self._create_ui()
        
        # Auto-start camera
        self.root.after(500, self._start_camera)
        
        # Start Port Monitor
        self.last_ports = self.available_ports[:]
        self.root.after(500, self._monitor_ports)
        
        # Cleanup on close
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _detect_cameras(self):
        """Find available cameras"""
        cameras = []
        for i in range(5):
            try:
                cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
                if cap.isOpened():
                    ret, _ = cap.read()
                    if ret:
                        cameras.append(i)
                    cap.release()
            except: pass
        return cameras if cameras else [0]

    def _detect_ports(self):
        """Find serial ports - list all available ports (connection tested when user clicks LAUNCH)"""
        ports = []
        esp32s3_port = None
        print(f"[DEBUG] Scanning for serial ports...")
        for p in serial.tools.list_ports.comports():
            hwid = p.hwid or ""
            print(f"[DEBUG] Found: {p.device} - {p.description} | HWID: {hwid}")
            # Check for ESP32-S3 native USB (VID 303A is Espressif)
            if "303a" in hwid.lower():
                esp32s3_port = p.device
                print(f"[DEBUG] ESP32-S3 detected on {p.device}")
            ports.append(p.device)
        
        # Move ESP32-S3 to front of list if found
        if esp32s3_port and esp32s3_port in ports:
            ports.remove(esp32s3_port)
            ports.insert(0, esp32s3_port)
            
        ports.append("SIMULATOR")
        print(f"[DEBUG] Final port list: {ports}")
        return ports

    def _refresh_ports(self):
        """Refresh the list of available serial ports - manual refresh button"""
        self.available_ports = self._detect_ports()
        self.port_combo['values'] = self.available_ports
        
        # Get port details for logging
        real_ports = [p for p in self.available_ports if p != "SIMULATOR"]
        
        # Auto-select best port (ESP32-S3 is now first if detected)
        if real_ports:
            # Check for ESP32-S3 first (VID 303A)
            esp32s3_port = None
            for p in real_ports:
                info = next((port for port in serial.tools.list_ports.comports() if port.device == p), None)
                if info and info.hwid and "303a" in info.hwid.lower():
                    esp32s3_port = p
                    break
            
            if esp32s3_port:
                self.port_combo.set(esp32s3_port)
                self._log(f"ESP32-S3 detected on {esp32s3_port}")
            elif "COM6" in real_ports:
                self.port_combo.set("COM6")
            else:
                self.port_combo.set(real_ports[0])
            
            # Log with descriptions
            port_details = []
            for p in real_ports:
                info = next((port for port in serial.tools.list_ports.comports() if port.device == p), None)
                desc = info.description if info else "Unknown"
                port_details.append(f"{p} ({desc})")
            self._log(f"Found ports: {', '.join(port_details)}")
        else:
            self.port_combo.set("SIMULATOR")
            self._log("No real ports found")

    def _monitor_ports(self):
        """Periodically check for port changes - runs every 500ms"""
        if not self.running: return
        
        print(f"[DEBUG] Port monitor running...")
        
        try:
            current_ports = self._detect_ports()
            print(f"[DEBUG] Current ports: {current_ports}")
            print(f"[DEBUG] Last ports: {self.last_ports}")
            
            # Convert to sets for easy comparison (ignoring SIMULATOR for logic)
            old_set = set(p for p in self.last_ports if p != "SIMULATOR")
            new_set = set(p for p in current_ports if p != "SIMULATOR")
            
            print(f"[DEBUG] Old set: {old_set}")
            print(f"[DEBUG] New set: {new_set}")
            
            # Check for changes
            added = new_set - old_set
            removed = old_set - new_set
            
            print(f"[DEBUG] Added: {added}")
            print(f"[DEBUG] Removed: {removed}")
            
            if added or removed:
                print(f"[DEBUG] Ports changed! Updating UI...")
                self.available_ports = current_ports
                self.port_combo['values'] = self.available_ports
                self.last_ports = current_ports
                
                # Log changes with details
                for p in added:
                    port_info = next((port for port in serial.tools.list_ports.comports() if port.device == p), None)
                    desc = port_info.description if port_info else "Unknown"
                    self._log(f"Device connected: {p} ({desc})")
                for p in removed:
                    self._log(f"Device disconnected: {p}")
                    
                # Handle selection logic
                current_selection = self.port_var.get()
                print(f"[DEBUG] Current selection: {current_selection}")
                
                # If currently selected port was removed
                if current_selection in removed:
                    if new_set:
                        # Fallback to another real port
                        new_port = list(new_set)[-1] # Pick last one
                        self.port_combo.set(new_port)
                        self._log(f"Auto-switched to {new_port}")
                    else:
                        self.port_combo.set("SIMULATOR")
                        self._log("Switched to SIMULATOR")
                        
                # If new port added and we're on SIMULATOR, auto-select it
                elif added and current_selection == "SIMULATOR":
                    new_port = list(added)[-1]
                    self.port_combo.set(new_port)
                    self._log(f"Auto-selected {new_port}")
            else:
                print(f"[DEBUG] No port changes detected")
                    
        except Exception as e:
            print(f"Port monitor error: {e}")
            import traceback
            traceback.print_exc()
            
        # Schedule next check (every 500ms for faster detection)
        self.root.after(500, self._monitor_ports)

    def _create_ui(self):
        """Build the UI"""
        main = ttk.Frame(self.root, padding="10")
        main.pack(fill=tk.BOTH, expand=True)
        
        # === LEFT: Controls + Camera ===
        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0,10))
        
        # --- Launch Controls (single row) ---
        ctrl_frame = ttk.LabelFrame(left, text="Launch Controls", padding="10")
        ctrl_frame.pack(fill=tk.X, pady=(0,10))
        
        # Port
        ttk.Label(ctrl_frame, text="Port:").pack(side=tk.LEFT, padx=(0,5))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(ctrl_frame, textvariable=self.port_var, width=12, values=self.available_ports)
        
        print(f"[DEBUG] Creating port combo with values: {self.available_ports}")
        
        # Default to real port if available, otherwise Simulator
        real_ports = [p for p in self.available_ports if p != "SIMULATOR"]
        print(f"[DEBUG] Real ports (excluding SIMULATOR): {real_ports}")
        if real_ports:
            # Prefer COM6 if present (common for ESP32)
            if "COM6" in real_ports:
                self.port_combo.set("COM6")
                print(f"[DEBUG] Selected COM6")
            else:
                self.port_combo.set(real_ports[-1]) # Use last port (often the most recently plugged in)
                print(f"[DEBUG] Selected {real_ports[-1]}")
        else:
            self.port_combo.set("SIMULATOR")
            print(f"[DEBUG] Selected SIMULATOR")
            
        self.port_combo.pack(side=tk.LEFT, padx=(0,5))
        
        # Refresh Button
        self.refresh_btn = ttk.Button(ctrl_frame, text="↻", width=3, command=self._refresh_ports)
        self.refresh_btn.pack(side=tk.LEFT, padx=(0,15))
        
        # Camera
        ttk.Label(ctrl_frame, text="Camera:").pack(side=tk.LEFT, padx=(0,5))
        self.cam_var = tk.StringVar()
        self.cam_combo = ttk.Combobox(ctrl_frame, textvariable=self.cam_var, width=5, values=self.available_cameras)
        # Default to camera 1 if available, otherwise first available
        default_cam = 1 if 1 in self.available_cameras else self.available_cameras[0]
        self.cam_combo.set(default_cam)
        self.cam_combo.pack(side=tk.LEFT, padx=(0,15))
        
        # Launch Button
        self.launch_btn = ttk.Button(ctrl_frame, text="▶ LAUNCH", command=self._toggle_output, width=12)
        self.launch_btn.pack(side=tk.LEFT, padx=(0,10))
        
        # Status
        self.status_var = tk.StringVar(value="Ready")
        self.status_label = ttk.Label(ctrl_frame, textvariable=self.status_var, foreground="gray")
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # --- Camera Feed ---
        cam_frame = ttk.LabelFrame(left, text="Camera Feed (Always On)", padding="5")
        cam_frame.pack(fill=tk.BOTH, expand=True, pady=(0,10))
        
        self.video_label = ttk.Label(cam_frame, text="Starting camera...")
        self.video_label.pack(fill=tk.BOTH, expand=True)
        
        # --- Log ---
        log_frame = ttk.LabelFrame(left, text="Log", padding="5")
        log_frame.pack(fill=tk.X)
        
        self.log_text = tk.Text(log_frame, height=3, wrap=tk.WORD)
        self.log_text.pack(fill=tk.X)
        
        # === RIGHT: LED Simulator + Test Patterns ===
        right = ttk.Frame(main)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
        
        # LED Simulator
        sim_frame = ttk.LabelFrame(right, text="LED Output (32x64)", padding="10")
        sim_frame.pack(fill=tk.BOTH, expand=True)
        
        self.led_simulator = LEDSimulatorPanel(sim_frame)
        
        self.output_info_var = tk.StringVar(value="Press LAUNCH to start")
        ttk.Label(sim_frame, textvariable=self.output_info_var).pack(pady=5)
        
        # === TEST PATTERNS PANEL ===
        test_frame = ttk.LabelFrame(right, text="LED Test Patterns", padding="5")
        test_frame.pack(fill=tk.X, pady=(10, 0))
        
        # Mode selection row
        mode_row = ttk.Frame(test_frame)
        mode_row.pack(fill=tk.X, pady=(0, 5))
        
        self.mode_var = tk.StringVar(value="LIVE")
        self.live_btn = ttk.Button(mode_row, text="👤 LIVE MODE", width=14, command=self._set_live_mode)
        self.live_btn.pack(side=tk.LEFT, padx=2)
        self.test_btn = ttk.Button(mode_row, text="🔧 TEST MODE", width=14, command=self._set_test_mode)
        self.test_btn.pack(side=tk.LEFT, padx=2)
        self.mode_label = ttk.Label(mode_row, text="Mode: LIVE", foreground="green", font=('Arial', 9, 'bold'))
        self.mode_label.pack(side=tk.LEFT, padx=10)
        
        # Row 1: Basic patterns
        row1 = ttk.Frame(test_frame)
        row1.pack(fill=tk.X, pady=2)
        
        ttk.Button(row1, text="1-8", width=5, command=lambda: self._test_pattern("panels")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row1, text="Grid", width=5, command=lambda: self._test_pattern("grid")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row1, text="Corner", width=6, command=lambda: self._test_pattern("corners")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row1, text="Check", width=5, command=lambda: self._test_pattern("checker")).pack(side=tk.LEFT, padx=2)
        
        # Row 2: Arrows
        row2 = ttk.Frame(test_frame)
        row2.pack(fill=tk.X, pady=2)
        
        ttk.Button(row2, text="↑", width=3, command=lambda: self._test_pattern("arrow_up")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="↓", width=3, command=lambda: self._test_pattern("arrow_down")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="←", width=3, command=lambda: self._test_pattern("arrow_left")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="→", width=3, command=lambda: self._test_pattern("arrow_right")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="H-Bar", width=5, command=lambda: self._test_pattern("hbars")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row2, text="V-Bar", width=5, command=lambda: self._test_pattern("vbars")).pack(side=tk.LEFT, padx=2)
        
        # Row 3: Text and special
        row3 = ttk.Frame(test_frame)
        row3.pack(fill=tk.X, pady=2)
        
        ttk.Button(row3, text="HOOKKAPANI", width=12, command=lambda: self._test_pattern("text")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row3, text="Count 1-8", width=8, command=lambda: self._test_pattern("count")).pack(side=tk.LEFT, padx=2)
        ttk.Button(row3, text="Clear", width=5, command=lambda: self._test_pattern("clear")).pack(side=tk.LEFT, padx=2)
        
        # Scrolling text controls
        row4 = ttk.Frame(test_frame)
        row4.pack(fill=tk.X, pady=2)
        
        ttk.Button(row4, text="▶ Scroll Text", width=12, command=self._start_scroll_text).pack(side=tk.LEFT, padx=2)
        ttk.Button(row4, text="■ Stop", width=6, command=self._stop_scroll_text).pack(side=tk.LEFT, padx=2)
        
        self.scroll_text_var = tk.StringVar(value="HOOKKAPANI STUDIO")
        ttk.Entry(row4, textvariable=self.scroll_text_var, width=20).pack(side=tk.LEFT, padx=5)
        
        # State for scrolling
        self.scroll_active = False
        self.scroll_offset = 0

    def _log(self, msg):
        try:
            self.log_text.insert(tk.END, f"{msg}\n")
            self.log_text.see(tk.END)
            if int(self.log_text.index('end-1c').split('.')[0]) > 30:
                self.log_text.delete('1.0', '2.0')
        except: pass

    # =========================================================================
    # MODE SWITCHING
    # =========================================================================
    
    def _set_live_mode(self):
        """Switch to live body tracking mode"""
        self.test_mode = False
        self.scroll_active = False  # Stop any scrolling text
        self.mode_label.config(text="Mode: LIVE", foreground="green")
        self._log("Switched to LIVE mode - body tracking active")
    
    def _set_test_mode(self):
        """Switch to test pattern mode"""
        self.test_mode = True
        self.mode_label.config(text="Mode: TEST", foreground="orange")
        self._log("Switched to TEST mode - select a pattern below")

    # =========================================================================
    # TEST PATTERN METHODS
    # =========================================================================
    
    def _create_test_frame(self, pattern_type):
        """Create a test pattern frame (32x64)"""
        WIDTH, HEIGHT = 32, 64
        frame = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        if pattern_type == "panels":
            # Show panel numbers 1-8
            panels = [(1, 4, 12), (2, 20, 12), (3, 4, 28), (4, 20, 28),
                      (5, 4, 44), (6, 20, 44), (7, 4, 60), (8, 20, 60)]
            for num, x, y in panels:
                cv2.putText(frame, str(num), (x, y), font, 0.5, 255, 1)
                
        elif pattern_type == "grid":
            # Grid at panel boundaries
            frame[0, :] = 255
            frame[15, :] = 128
            frame[16, :] = 128
            frame[31, :] = 128
            frame[32, :] = 128
            frame[47, :] = 128
            frame[48, :] = 128
            frame[63, :] = 255
            frame[:, 0] = 255
            frame[:, 15] = 128
            frame[:, 16] = 128
            frame[:, 31] = 255
            
        elif pattern_type == "corners":
            # Corner markers
            cv2.putText(frame, "TL", (1, 10), font, 0.3, 255, 1)
            cv2.putText(frame, "TR", (20, 10), font, 0.3, 255, 1)
            cv2.putText(frame, "BL", (1, 60), font, 0.3, 255, 1)
            cv2.putText(frame, "BR", (20, 60), font, 0.3, 255, 1)
            frame[0:3, 0:3] = 255
            frame[0:3, 29:32] = 255
            frame[61:64, 0:3] = 255
            frame[61:64, 29:32] = 255
            
        elif pattern_type == "checker":
            for y in range(HEIGHT):
                for x in range(WIDTH):
                    if ((x // 4) + (y // 4)) % 2 == 0:
                        frame[y, x] = 255
                        
        elif pattern_type == "arrow_up":
            cx, cy = WIDTH // 2, HEIGHT // 2
            pts = np.array([[cx, cy-20], [cx-10, cy+10], [cx+10, cy+10]])
            cv2.fillPoly(frame, [pts], 255)
            cv2.putText(frame, "UP", (10, 60), font, 0.4, 255, 1)
            
        elif pattern_type == "arrow_down":
            cx, cy = WIDTH // 2, HEIGHT // 2
            pts = np.array([[cx, cy+20], [cx-10, cy-10], [cx+10, cy-10]])
            cv2.fillPoly(frame, [pts], 255)
            cv2.putText(frame, "DN", (10, 10), font, 0.4, 255, 1)
            
        elif pattern_type == "arrow_left":
            cx, cy = WIDTH // 2, HEIGHT // 2
            pts = np.array([[cx-12, cy], [cx+5, cy-12], [cx+5, cy+12]])
            cv2.fillPoly(frame, [pts], 255)
            cv2.putText(frame, "L", (26, 32), font, 0.4, 255, 1)
            
        elif pattern_type == "arrow_right":
            cx, cy = WIDTH // 2, HEIGHT // 2
            pts = np.array([[cx+12, cy], [cx-5, cy-12], [cx-5, cy+12]])
            cv2.fillPoly(frame, [pts], 255)
            cv2.putText(frame, "R", (2, 32), font, 0.4, 255, 1)
            
        elif pattern_type == "hbars":
            for y in range(0, HEIGHT, 4):
                frame[y:y+2, :] = 255
                
        elif pattern_type == "vbars":
            for x in range(0, 16, 4):
                frame[:, x:x+2] = 255
            for x in range(16, 32, 2):
                frame[:, x] = 255
                
        elif pattern_type == "text":
            # HOOKKAPANI text
            cv2.putText(frame, "HOOKKA", (1, 25), font, 0.35, 255, 1)
            cv2.putText(frame, "PANI", (4, 40), font, 0.4, 255, 1)
            cv2.putText(frame, "STUDIO", (1, 55), font, 0.35, 255, 1)
            
        elif pattern_type == "clear":
            pass  # Already zeros
            
        return frame
    
    def _send_test_frame(self, frame):
        """Send test frame to simulator and hardware"""
        # Update simulator
        self.led_simulator.update_leds(frame.flatten().tolist())
        
        # Send to hardware if connected
        if self.serial_port:
            try:
                packet = self.led_controller.pack_led_packet_1bit(frame)
                self.serial_port.write(packet)
            except Exception as e:
                self._log(f"Send error: {e}")
    
    def _test_pattern(self, pattern_type):
        """Display a test pattern (auto-switches to test mode)"""
        # Auto-switch to test mode when selecting a pattern
        if not self.test_mode:
            self._set_test_mode()
        
        if pattern_type == "count":
            # Count through numbers 1-8
            self._log("Counting 1-8...")
            self._count_sequence(1)
            return
        
        if pattern_type == "clear":
            # Clear and switch back to live mode
            frame = self._create_test_frame(pattern_type)
            self._send_test_frame(frame)
            self._log("Cleared - switching to LIVE mode")
            self._set_live_mode()
            return
            
        frame = self._create_test_frame(pattern_type)
        self._send_test_frame(frame)
        self._log(f"Test: {pattern_type}")
    
    def _count_sequence(self, num):
        """Animate counting 1-8"""
        if num > 8:
            self._log("Count complete")
            return
        
        # Create number frame
        frame = np.zeros((64, 32), dtype=np.uint8)
        text = str(num)
        font = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), _ = cv2.getTextSize(text, font, 1.5, 2)
        x = (32 - tw) // 2
        y = (64 + th) // 2
        cv2.putText(frame, text, (x, y), font, 1.5, 255, 2)
        
        self._send_test_frame(frame)
        
        # Schedule next number
        self.root.after(800, lambda: self._count_sequence(num + 1))
    
    def _start_scroll_text(self):
        """Start scrolling text animation (auto-switches to test mode)"""
        if not self.test_mode:
            self._set_test_mode()
        self.scroll_active = True
        self.scroll_offset = 0
        self._log(f"Scrolling: {self.scroll_text_var.get()}")
        self._scroll_text_loop()
    
    def _stop_scroll_text(self):
        """Stop scrolling text"""
        self.scroll_active = False
        self._log("Scroll stopped")
    
    def _scroll_text_loop(self):
        """Animation loop for scrolling text"""
        if not self.scroll_active:
            return
        
        text = self.scroll_text_var.get()
        WIDTH, HEIGHT = 32, 64
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Get text dimensions
        (tw, th), _ = cv2.getTextSize(text, font, 0.5, 1)
        
        # Create wide canvas
        canvas_width = tw + WIDTH * 2
        canvas = np.zeros((HEIGHT, canvas_width), dtype=np.uint8)
        cv2.putText(canvas, text, (WIDTH, HEIGHT // 2 + th // 2), font, 0.5, 255, 1)
        
        # Extract visible portion
        start_x = self.scroll_offset % (tw + WIDTH)
        frame = canvas[:, start_x:start_x + WIDTH]
        
        if frame.shape[1] < WIDTH:
            padded = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
            padded[:, :frame.shape[1]] = frame
            frame = padded
        
        self._send_test_frame(frame)
        
        self.scroll_offset += 1
        self.root.after(50, self._scroll_text_loop)

    def _start_camera(self):
        """Start camera feed (runs continuously)"""
        try:
            cam_idx = int(self.cam_combo.get())
        except:
            cam_idx = 0
        
        self.cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not self.cap.isOpened():
            self._log(f"Failed to open camera {cam_idx}")
            return
        
        # Init segmenter
        try:
            self.body_segmenter = BodySegmenter()
            self._log("Segmentation ready")
        except Exception as e:
            self._log(f"Segmenter error: {e}")
            self.body_segmenter = None
        
        self.camera_running = True
        self._log(f"Camera {cam_idx} started")
        
        # Start camera loop
        threading.Thread(target=self._camera_loop, daemon=True).start()

    def _camera_loop(self):
        """Camera processing loop - always runs"""
        while self.running and self.camera_running and self.cap:
            try:
                self.cap.grab()
                ret, frame = self.cap.retrieve()
                if not ret:
                    continue
                
                frame = cv2.flip(frame, 1)  # Mirror
                
                # Get body mask (full res for preview)
                if self.body_segmenter:
                    body_mask = self.body_segmenter.get_body_mask(frame)
                    # Get LED-sized mask directly (optimized)
                    led_mask = cv2.resize(body_mask, (32, 64), interpolation=cv2.INTER_AREA)
                    led_mask = (led_mask > 128).astype(np.uint8) * 255
                else:
                    body_mask = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
                    led_mask = np.zeros((64, 32), dtype=np.uint8)
                
                # If LED output is active AND not in test mode, send body tracking to LEDs
                if self.led_output_active and not self.test_mode:
                    # Update simulator
                    self.led_simulator.update_leds(led_mask.flatten().tolist())
                    
                    # Send to hardware with optimized 1-bit packet
                    if self.serial_port:
                        try:
                            # Use 1-bit packet for ESP32 (8x smaller: 259 bytes vs 2051)
                            packet = self.led_controller.pack_led_packet_1bit(led_mask)
                            self.serial_port.write(packet)
                        except: pass
                
                # Create preview
                preview = frame.copy()
                
                # Overlay body mask in green
                if self.body_segmenter and body_mask is not None:
                    green = np.zeros_like(preview)
                    green[:, :, 1] = body_mask
                    cv2.addWeighted(preview, 0.7, green, 0.4, 0, preview)
                    
                    # Draw contours
                    contours, _ = cv2.findContours(body_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(preview, contours, -1, (0, 255, 0), 2)
                
                # Status text
                status = "● LIVE" if self.led_output_active else "○ STANDBY"
                color = (0, 255, 0) if self.led_output_active else (128, 128, 128)
                cv2.putText(preview, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                # Update UI
                preview_small = cv2.resize(preview, (480, 360))
                preview_rgb = cv2.cvtColor(preview_small, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(preview_rgb)
                imgtk = ImageTk.PhotoImage(image=img)
                self.root.after(0, self._update_video, imgtk)
                
            except Exception as e:
                pass

    def _update_video(self, imgtk):
        try:
            self.video_label.imgtk = imgtk
            self.video_label.configure(image=imgtk)
        except: pass

    def _toggle_output(self):
        """Toggle LED output on/off"""
        if self.led_output_active:
            self._stop_output()
        else:
            self._start_output()

    def _start_output(self):
        """Start LED output - with handshake verification for COM ports"""
        port = self.port_var.get()
        
        try:
            if port == "SIMULATOR":
                self.serial_port = MockSerial(port=port, baudrate=460800)
                self.led_controller.mapping_mode = 0
                self._log("Connected: SIMULATOR")
            else:
                # Connect to real COM port
                self.status_var.set(f"Connecting to {port}...")
                self.root.update()
                
                # Check if this is ESP32-S3 native USB (VID 303A)
                port_info = next((p for p in serial.tools.list_ports.comports() if p.device == port), None)
                is_esp32s3_native = port_info and port_info.hwid and "303a" in port_info.hwid.lower()
                
                ser = serial.Serial(
                    port=port, baudrate=460800,
                    bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE, timeout=2
                )
                
                # ESP32-S3 native USB needs special handling - don't toggle DTR/RTS
                # as it can cause USB re-enumeration issues
                if is_esp32s3_native:
                    self._log("ESP32-S3 native USB detected - using extended boot wait")
                    ser.dtr = False
                    ser.rts = False
                    time.sleep(1.5)  # Longer wait for ESP32-S3 native USB
                else:
                    # Standard USB-to-serial bridge - shorter wait
                    time.sleep(0.5)
                
                ser.reset_input_buffer()
                
                # Send PING packet to verify connection
                self._log(f"Verifying ESP32 on {port}...")
                ping_packet = bytes([0xAA, 0xBB, 0x05])  # Header + PING type
                ser.write(ping_packet)
                ser.flush()
                
                # Wait for PONG response - longer timeout for ESP32-S3
                response = ""
                timeout = 4.0 if is_esp32s3_native else 2.0
                start_time = time.time()
                while time.time() - start_time < timeout:
                    if ser.in_waiting > 0:
                        data = ser.readline().decode('utf-8', errors='ignore').strip()
                        self._log(f"ESP32: {data}")
                        if "PONG" in data:
                            response = data
                            break
                        # Detect bootloader mode - firmware not running
                        if "waiting for download" in data.lower() or "download(usb" in data.lower():
                            ser.close()
                            self._log("ESP32-S3 is in BOOTLOADER mode - firmware not running!")
                            messagebox.showerror("Firmware Not Running", 
                                f"ESP32-S3 on {port} is in bootloader mode.\n\n"
                                "The firmware needs to be flashed:\n"
                                "1. Open PlatformIO in VS Code\n"
                                "2. Build and Upload the firmware\n"
                                "3. Press the RESET button on ESP32-S3\n\n"
                                "See FLASH_INSTRUCTIONS.txt for details.")
                            self.status_var.set("Firmware not loaded")
                            return
                        # Also check for READY signal (firmware sending boot complete)
                        if "READY" in data.upper():
                            self._log("ESP32 boot complete, retrying PING...")
                            ser.write(ping_packet)
                            ser.flush()
                        response += data
                    time.sleep(0.05)
                
                if "PONG" not in response:
                    # Try one more time with a fresh PING
                    self._log("Retrying connection...")
                    ser.reset_input_buffer()
                    ser.write(ping_packet)
                    ser.flush()
                    time.sleep(0.5)
                    
                    start_time = time.time()
                    while time.time() - start_time < 2.0:
                        if ser.in_waiting > 0:
                            data = ser.readline().decode('utf-8', errors='ignore').strip()
                            self._log(f"ESP32: {data}")
                            if "PONG" in data:
                                response = data
                                break
                            # Check for bootloader mode again
                            if "waiting for download" in data.lower():
                                response = "BOOTLOADER"
                                break
                        time.sleep(0.05)
                
                if "PONG" not in response:
                    ser.close()
                    
                    # Provide specific error message based on what we detected
                    if "BOOTLOADER" in response or "download" in response.lower():
                        self._log("ESP32-S3 firmware not loaded")
                        messagebox.showerror("Firmware Not Running", 
                            f"ESP32-S3 on {port} needs firmware.\n\n"
                            "Flash the firmware using PlatformIO:\n"
                            "1. Open firmware/esp32 folder\n"
                            "2. Run 'pio run -t upload'\n"
                            "3. Press RESET on ESP32-S3")
                    else:
                        self._log(f"No response from ESP32 (got: '{response}')")
                        messagebox.showerror("Connection Failed", 
                            f"ESP32 not responding on {port}.\n\n"
                            "Check:\n"
                            "• Is the ESP32-S3 powered on?\n"
                            "• Is the correct COM port selected?\n"
                            "• Is the firmware flashed?\n\n"
                            "Try pressing the RESET button.")
                    self.status_var.set("Connection failed")
                    return
                
                self._log(f"ESP32 verified on {port}")
                self.serial_port = ser
                # Use RAW mode - firmware handles serpentine mapping internally
                self.led_controller.mapping_mode = 0
                self._log(f"Connected: {port}")
            
            self.serial_port.reset_input_buffer()
            self.led_output_active = True
            
            self.launch_btn.config(text="■ STOP")
            self.status_var.set(f"LIVE → {port}")
            self.status_label.config(foreground="green")
            self.output_info_var.set(f"Outputting to {port}")
            
            # Disable port/camera selection while active
            self.port_combo.config(state="disabled")
            self.cam_combo.config(state="disabled")
            
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Cannot open {port}:\n{e}")
            self._log(f"Serial error: {e}")
            self.status_var.set("Error")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to connect: {e}")
            self._log(f"Error: {e}")
            self.status_var.set("Error")

    def _stop_output(self):
        """Stop LED output"""
        self.led_output_active = False
        
        # Clear LEDs
        self.led_simulator.clear()
        
        # Close serial
        if self.serial_port:
            try:
                # Send black frame
                frame = np.zeros((64, 32, 3), dtype=np.uint8)
                packet = self.led_controller.pack_led_packet(frame)
                self.serial_port.write(packet)
                self.serial_port.close()
            except: pass
            self.serial_port = None
        
        self.launch_btn.config(text="▶ LAUNCH")
        self.status_var.set("Ready")
        self.status_label.config(foreground="gray")
        self.output_info_var.set("Press LAUNCH to start")
        
        # Re-enable selection
        self.port_combo.config(state="normal")
        self.cam_combo.config(state="normal")
        
        self._log("Output stopped")

    def _on_close(self):
        """Clean shutdown"""
        self.running = False
        self.camera_running = False
        self.led_output_active = False
        
        if self.body_segmenter:
            try: self.body_segmenter.close()
            except: pass
        
        if self.cap:
            try: self.cap.release()
            except: pass
        
        if self.serial_port:
            try: self.serial_port.close()
            except: pass
        
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = LEDControlApp(root)
    root.mainloop()
