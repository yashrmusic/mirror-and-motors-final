"""
PC-Side Performance Benchmark
==============================
Tests every parameter in the camera → segmentation → serial pipeline.
No ESP32 needed — measures PC-side timings only.

Usage:
    python -m packages.mirror_core.testing.pc_benchmark
"""

import cv2
import numpy as np
import time
import json
import os
import sys
from datetime import datetime


def time_it(func, runs=30, warmup=5):
    """Time a function, return stats in ms."""
    for _ in range(warmup):
        func()
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        func()
        times.append((time.perf_counter() - t0) * 1000)
    return {
        'min_ms': round(min(times), 2),
        'max_ms': round(max(times), 2),
        'avg_ms': round(sum(times) / len(times), 2),
        'median_ms': round(sorted(times)[len(times) // 2], 2),
    }


# ======================= TEST 1: Camera Capture =======================

def test_camera_fps(cam_idx=1):
    """Test actual camera FPS at different resolutions."""
    print("\n" + "=" * 60)
    print("TEST 1: CAMERA CAPTURE SPEED")
    print("=" * 60)
    results = []

    resolutions = [
        (160, 120), (320, 240), (480, 360), (640, 480),
    ]
    
    for w, h in resolutions:
        print(f"\n--- {w}x{h} ---")
        cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_idx)
        if not cap.isOpened():
            print(f"  ✗ Cannot open camera {cam_idx}")
            break

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        cap.set(cv2.CAP_PROP_FPS, 120)  # Request max
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Warm up
        for _ in range(10):
            cap.read()

        # Measure
        frames = 60
        t0 = time.perf_counter()
        for _ in range(frames):
            ret, frame = cap.read()
        elapsed = time.perf_counter() - t0
        fps = frames / elapsed

        result = {
            'requested': f"{w}x{h}",
            'actual': f"{actual_w}x{actual_h}",
            'fps': round(fps, 1),
            'frame_time_ms': round(1000 / fps, 1),
        }
        results.append(result)
        print(f"  Actual: {actual_w}x{actual_h} | FPS: {fps:.1f} | Frame time: {1000/fps:.1f}ms")

        cap.release()
        time.sleep(0.3)

    return results


# ======================= TEST 2: Resize Speed =======================

def test_resize_speed():
    """Test cv2.resize with different interpolation methods."""
    print("\n" + "=" * 60)
    print("TEST 2: RESIZE INTERPOLATION SPEED")
    print("=" * 60)
    results = []

    # Simulate camera frame
    src = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

    methods = [
        ('INTER_NEAREST', cv2.INTER_NEAREST),
        ('INTER_LINEAR', cv2.INTER_LINEAR),
        ('INTER_AREA', cv2.INTER_AREA),
        ('INTER_CUBIC', cv2.INTER_CUBIC),
    ]

    targets = [(128, 96), (160, 120), (256, 192), (320, 240)]

    for target_w, target_h in targets:
        for name, method in methods:
            stats = time_it(lambda m=method, tw=target_w, th=target_h: 
                           cv2.resize(src, (tw, th), interpolation=m))
            result = {'target': f"{target_w}x{target_h}", 'method': name, **stats}
            results.append(result)
            print(f"  {target_w}x{target_h} {name:16s}: {stats['avg_ms']:.2f}ms")

    return results


# ======================= TEST 3: Segmentation Speed =======================

def test_segmentation_speed():
    """Test MediaPipe segmentation at different resolutions."""
    print("\n" + "=" * 60)
    print("TEST 3: MEDIAPIPE SEGMENTATION SPEED")
    print("=" * 60)
    results = []

    try:
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision
        from pathlib import Path
        import urllib.request
        import ssl

        model_path = Path("data/selfie_segmenter.tflite")
        if not model_path.exists():
            print("  Downloading model...")
            url = "https://storage.googleapis.com/mediapipe-models/image_segmenter/selfie_segmenter/float16/latest/selfie_segmenter.tflite"
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            model_path.parent.mkdir(exist_ok=True)
            with urllib.request.urlopen(url, context=ctx) as u, open(model_path, 'wb') as f:
                f.write(u.read())

        resolutions = [(128, 96), (160, 120), (192, 144), (256, 192), (320, 240)]

        for w, h in resolutions:
            print(f"\n--- Segmentation at {w}x{h} ---")

            base_options = mp_python.BaseOptions(model_asset_path=str(model_path))
            options = mp_vision.ImageSegmenterOptions(
                base_options=base_options,
                running_mode=mp_vision.RunningMode.VIDEO,
                output_category_mask=True
            )
            segmenter = mp_vision.ImageSegmenter.create_from_options(options)

            # Create test frame
            test_frame = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
            frame_count = [0]

            def run_seg():
                frame_count[0] += 1
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=test_frame)
                result = segmenter.segment_for_video(mp_img, frame_count[0] * 33)
                return result

            stats = time_it(run_seg, runs=50, warmup=10)
            max_fps = round(1000 / stats['avg_ms'], 1)
            result = {'resolution': f"{w}x{h}", 'max_fps': max_fps, **stats}
            results.append(result)
            print(f"  Avg: {stats['avg_ms']:.1f}ms | Max FPS: {max_fps} | "
                  f"Min: {stats['min_ms']:.1f}ms | Max: {stats['max_ms']:.1f}ms")

            segmenter.close()

    except ImportError:
        print("  ✗ MediaPipe not installed")
        return [{'error': 'MediaPipe not installed'}]

    return results


# ======================= TEST 4: Morphology Speed =======================

def test_morphology_speed():
    """Test morphological operations with different kernel sizes."""
    print("\n" + "=" * 60)
    print("TEST 4: MORPHOLOGY KERNEL SPEED")
    print("=" * 60)
    results = []

    mask = np.random.randint(0, 255, (240, 320), dtype=np.uint8)

    kernel_sizes = [3, 5, 7, 9, 11]
    for k in kernel_sizes:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))

        def run_morph(kernel=kernel):
            out = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            out = cv2.dilate(out, kernel, iterations=1)
            return out

        stats = time_it(run_morph, runs=100, warmup=10)
        result = {'kernel_size': k, **stats}
        results.append(result)
        print(f"  Kernel {k}x{k}: {stats['avg_ms']:.2f}ms")

    return results


# ======================= TEST 5: Queue Latency =======================

def test_queue_latency():
    """Test queue put/get latency with different maxsizes."""
    print("\n" + "=" * 60)
    print("TEST 5: QUEUE LATENCY")
    print("=" * 60)
    import queue
    import threading
    results = []

    frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)

    for maxsize in [1, 2, 3, 5]:
        q = queue.Queue(maxsize=maxsize)

        # Test put+get latency
        times = []
        for _ in range(1000):
            # Clear queue
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

            t0 = time.perf_counter()
            try:
                q.put_nowait(frame)
            except queue.Full:
                pass
            try:
                q.get_nowait()
            except queue.Empty:
                pass
            times.append((time.perf_counter() - t0) * 1000)

        avg = sum(times) / len(times)
        med = sorted(times)[len(times) // 2]
        result = {
            'maxsize': maxsize,
            'avg_us': round(avg * 1000, 1),  # microseconds
            'median_us': round(med * 1000, 1),
        }
        results.append(result)
        print(f"  maxsize={maxsize}: avg={result['avg_us']}µs median={result['median_us']}µs")

    return results


# ======================= TEST 6: Full Pipeline =======================

def test_full_pipeline(cam_idx=1):
    """Test full pipeline: camera read → resize → segment → compute position."""
    print("\n" + "=" * 60)
    print("TEST 6: FULL PIPELINE (camera → segment → position)")
    print("=" * 60)
    results = []

    try:
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision
        from pathlib import Path

        model_path = Path("data/selfie_segmenter.tflite")
        if not model_path.exists():
            print("  ✗ No model file, skipping")
            return results

        configs = [
            {'seg_w': 128, 'seg_h': 96, 'smooth': 0.0, 'label': 'MIN: 128x96, no smooth'},
            {'seg_w': 128, 'seg_h': 96, 'smooth': 0.3, 'label': 'LOW: 128x96, smooth 0.3'},
            {'seg_w': 192, 'seg_h': 144, 'smooth': 0.0, 'label': 'MED: 192x144, no smooth'},
            {'seg_w': 256, 'seg_h': 192, 'smooth': 0.0, 'label': 'CURRENT: 256x192, no smooth'},
            {'seg_w': 256, 'seg_h': 192, 'smooth': 0.3, 'label': 'CURRENT+SM: 256x192, smooth 0.3'},
            {'seg_w': 320, 'seg_h': 240, 'smooth': 0.0, 'label': 'HIGH: 320x240, no smooth'},
        ]

        cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(cam_idx)
        if not cap.isOpened():
            print(f"  ✗ Cannot open camera {cam_idx}")
            return results

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        cap.set(cv2.CAP_PROP_FPS, 60)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Warm up camera
        for _ in range(15):
            cap.read()

        for cfg in configs:
            print(f"\n--- {cfg['label']} ---")

            base_options = mp_python.BaseOptions(model_asset_path=str(model_path))
            options = mp_vision.ImageSegmenterOptions(
                base_options=base_options,
                running_mode=mp_vision.RunningMode.VIDEO,
                output_category_mask=True
            )
            segmenter = mp_vision.ImageSegmenter.create_from_options(options)

            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
            kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask_buffer = None
            frame_count = 0

            times_capture = []
            times_resize = []
            times_segment = []
            times_morph = []
            times_total = []

            runs = 50
            # Warmup
            for _ in range(10):
                ret, frame = cap.read()
                if ret:
                    small = cv2.resize(frame, (cfg['seg_w'], cfg['seg_h']))
                    small_rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
                    mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=small_rgb)
                    frame_count += 1
                    segmenter.segment_for_video(mp_img, frame_count * 33)

            for _ in range(runs):
                t_total = time.perf_counter()

                # Capture
                t0 = time.perf_counter()
                ret, frame = cap.read()
                times_capture.append((time.perf_counter() - t0) * 1000)
                if not ret:
                    continue

                # Resize
                t0 = time.perf_counter()
                small = cv2.resize(frame, (cfg['seg_w'], cfg['seg_h']), interpolation=cv2.INTER_LINEAR)
                small_rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
                times_resize.append((time.perf_counter() - t0) * 1000)

                # Segment
                t0 = time.perf_counter()
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=small_rgb)
                frame_count += 1
                result = segmenter.segment_for_video(mp_img, frame_count * 33)
                times_segment.append((time.perf_counter() - t0) * 1000)

                # Post-process (morph + position)
                t0 = time.perf_counter()
                if result.category_mask is not None:
                    mask = result.category_mask.numpy_view()
                    mask_float = (mask > 0).astype(np.float32)
                    if cfg['smooth'] > 0 and mask_buffer is not None:
                        mask_buffer = cfg['smooth'] * mask_buffer + (1 - cfg['smooth']) * mask_float
                    else:
                        mask_buffer = mask_float
                    binary = (mask_buffer > 0.4).astype(np.uint8) * 255
                    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel_close)
                    binary = cv2.dilate(binary, kernel_dilate, iterations=1)
                    # Compute body X position
                    cols = np.where(binary.any(axis=0))[0]
                    if len(cols) > 0:
                        body_x = (cols[0] + cols[-1]) / 2 / binary.shape[1]
                times_morph.append((time.perf_counter() - t0) * 1000)
                times_total.append((time.perf_counter() - t_total) * 1000)

            segmenter.close()

            avg = lambda lst: round(sum(lst) / len(lst), 1) if lst else 0
            result = {
                'config': cfg['label'],
                'capture_ms': avg(times_capture),
                'resize_ms': avg(times_resize),
                'segment_ms': avg(times_segment),
                'postproc_ms': avg(times_morph),
                'total_ms': avg(times_total),
                'max_fps': round(1000 / avg(times_total), 1) if avg(times_total) > 0 else 0,
            }
            results.append(result)
            print(f"  Capture: {result['capture_ms']}ms | Resize: {result['resize_ms']}ms | "
                  f"Segment: {result['segment_ms']}ms | Post: {result['postproc_ms']}ms | "
                  f"TOTAL: {result['total_ms']}ms ({result['max_fps']} FPS)")

        cap.release()

    except ImportError:
        print("  ✗ MediaPipe not installed")
        return [{'error': 'MediaPipe not installed'}]

    return results


# ======================= TEST 7: Serial Packet Build =======================

def test_packet_build_speed():
    """Test motor packet building speed."""
    print("\n" + "=" * 60)
    print("TEST 7: MOTOR PACKET BUILD SPEED")
    print("=" * 60)
    import struct

    angles = [90.0] * 64

    # Method 1: Current (struct pack loop)
    def build_loop():
        header = b'\xAA\xBB\x02'
        data = b''
        for angle in angles:
            value = int(angle * 1000 / 180)
            value = max(0, min(1000, value))
            data += struct.pack('>H', value)
        return header + data

    # Method 2: struct.pack all at once
    def build_batch():
        header = b'\xAA\xBB\x02'
        values = [max(0, min(1000, int(a * 1000 / 180))) for a in angles]
        data = struct.pack('>' + 'H' * len(values), *values)
        return header + data

    # Method 3: numpy
    def build_numpy():
        header = b'\xAA\xBB\x02'
        arr = np.clip(np.array(angles) * 1000 / 180, 0, 1000).astype(np.uint16)
        arr = arr.byteswap()  # big-endian
        return header + arr.tobytes()

    methods = [
        ('Loop + struct.pack', build_loop),
        ('Batch struct.pack', build_batch),
        ('NumPy tobytes', build_numpy),
    ]

    results = []
    for name, func in methods:
        stats = time_it(func, runs=1000, warmup=100)
        result = {'method': name, **stats}
        results.append(result)
        print(f"  {name:25s}: avg={stats['avg_ms']:.3f}ms median={stats['median_ms']:.3f}ms")

    return results


# ======================= MAIN =======================

def main():
    print("╔══════════════════════════════════════════════════╗")
    print("║   PC-SIDE PERFORMANCE BENCHMARK                 ║")
    print("║   Mirror & Motors Project                       ║")
    print("╚══════════════════════════════════════════════════╝")
    print(f"\nTimestamp: {datetime.now().isoformat()}\n")

    all_results = {
        'timestamp': datetime.now().isoformat(),
    }

    # Find camera index (prefer camera 1 which is usually external)
    cam_idx = 1
    cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cam_idx = 0
        cap = cv2.VideoCapture(cam_idx, cv2.CAP_DSHOW)
    if cap.isOpened():
        cap.release()
    else:
        print("⚠ No camera found, camera tests will be skipped")
        cam_idx = -1

    all_results['camera_idx'] = cam_idx

    # Test 1: Camera FPS
    if cam_idx >= 0:
        try:
            all_results['camera_fps'] = test_camera_fps(cam_idx)
        except Exception as e:
            print(f"  ✗ Camera test failed: {e}")
            all_results['camera_fps'] = {'error': str(e)}

    # Test 2: Resize speed
    try:
        all_results['resize_speed'] = test_resize_speed()
    except Exception as e:
        print(f"  ✗ Resize test failed: {e}")
        all_results['resize_speed'] = {'error': str(e)}

    # Test 3: Segmentation speed
    try:
        all_results['segmentation_speed'] = test_segmentation_speed()
    except Exception as e:
        print(f"  ✗ Segmentation test failed: {e}")
        all_results['segmentation_speed'] = {'error': str(e)}

    # Test 4: Morphology speed
    try:
        all_results['morphology_speed'] = test_morphology_speed()
    except Exception as e:
        print(f"  ✗ Morphology test failed: {e}")
        all_results['morphology_speed'] = {'error': str(e)}

    # Test 5: Queue latency
    try:
        all_results['queue_latency'] = test_queue_latency()
    except Exception as e:
        print(f"  ✗ Queue test failed: {e}")
        all_results['queue_latency'] = {'error': str(e)}

    # Test 6: Full pipeline
    if cam_idx >= 0:
        try:
            all_results['full_pipeline'] = test_full_pipeline(cam_idx)
        except Exception as e:
            print(f"  ✗ Pipeline test failed: {e}")
            import traceback
            traceback.print_exc()
            all_results['full_pipeline'] = {'error': str(e)}

    # Test 7: Packet build
    try:
        all_results['packet_build'] = test_packet_build_speed()
    except Exception as e:
        print(f"  ✗ Packet build test failed: {e}")
        all_results['packet_build'] = {'error': str(e)}

    # Save results
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'logs')
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, 'pc_benchmark_results.json')

    with open(log_file, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)

    print(f"\n{'=' * 60}")
    print(f"Results saved to: {os.path.abspath(log_file)}")
    print(f"{'=' * 60}")


if __name__ == '__main__':
    main()
