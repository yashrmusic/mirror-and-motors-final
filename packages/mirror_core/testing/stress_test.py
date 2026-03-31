"""
Motor Real-Time Performance Stress Test
========================================
Tests every parameter in the camera→servo pipeline to find the
best combo for real-time motor control with zero lag.

Usage:
    python -m packages.mirror_core.testing.stress_test --port COM3

Requires: stress_test_firmware.ino flashed to ESP32
"""

import serial
import struct
import time
import json
import os
import sys
import argparse
from datetime import datetime


# ======================= CONFIG =======================
BAUD_RATES = [460800, 921600, 1000000, 2000000]
MOTOR_SEND_RATES = [30, 60, 120, 200, 500]  # Hz
WRITE_TIMEOUTS = [0.05, 0.1, 0.5, 1.0]  # seconds
TEST_DURATION = 5  # seconds per test

NUM_SERVOS = 64
SERVO_PACKET_SIZE = 3 + NUM_SERVOS * 2  # Header(3) + Data(128) = 131 bytes


def build_servo_packet(angles):
    """Build a servo packet identical to production format."""
    header = b'\xAA\xBB\x02'
    data = b''
    for angle in angles:
        value = int(angle * 1000 / 180)  # 0-180 → 0-1000
        value = max(0, min(1000, value))
        data += struct.pack('>H', value)
    return header + data


def wait_for_ready(ser, timeout=25):
    """Wait for ESP32 to send READY after boot.
    
    Reads raw bytes and buffers lines manually because the ESP32
    prints WiFi dots without newlines, which blocks readline().
    """
    start = time.time()
    buf = b''
    while time.time() - start < timeout:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            buf += chunk
            # Process complete lines
            while b'\n' in buf:
                line_bytes, buf = buf.split(b'\n', 1)
                line = line_bytes.decode('utf-8', errors='replace').strip()
                if line:
                    print(f"  ESP32: {line}")
                    if 'READY' in line or 'STRESS' in line or 'FPS' in line:
                        return True
        else:
            time.sleep(0.05)
    # Check remaining buffer
    if buf:
        line = buf.decode('utf-8', errors='replace').strip()
        if line:
            print(f"  ESP32: {line}")
            if 'READY' in line or 'STRESS' in line or 'FPS' in line:
                return True
    return False


def read_esp_stats(ser, timeout=0.5):
    """Read all available stats from ESP32."""
    lines = []
    start = time.time()
    while time.time() - start < timeout:
        if ser.in_waiting:
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                if line:
                    lines.append(line)
            except Exception:
                pass
        else:
            time.sleep(0.01)
    return lines


def send_command(ser, cmd):
    """Send a command to the stress test firmware."""
    ser.write(f"{cmd}\n".encode())
    time.sleep(0.1)
    return read_esp_stats(ser, timeout=0.5)


def parse_stats(lines):
    """Parse ESP32 stats lines into a dict."""
    stats = {}
    for line in lines:
        if 'STATS|' in line:
            # Format: STATS|FPS:xx|PKTS:xx|ERRS:xx|LATENCY:xx
            parts = line.split('|')
            for part in parts[1:]:
                if ':' in part:
                    key, val = part.split(':', 1)
                    try:
                        stats[key.strip()] = float(val.strip())
                    except ValueError:
                        stats[key.strip()] = val.strip()
    return stats


# ======================= TESTS =======================

def test_baud_rates(port):
    """Test different baud rates to find max reliable speed."""
    print("\n" + "=" * 60)
    print("TEST 1: BAUD RATE SWEEP")
    print("=" * 60)
    results = []

    for baud in BAUD_RATES:
        print(f"\n--- Testing {baud} baud ---")
        try:
            ser = serial.Serial(port, baud, timeout=1, write_timeout=1.0)
            time.sleep(0.5)

            if not wait_for_ready(ser, timeout=3):
                print(f"  ⚠ ESP32 not ready at {baud} baud (may need firmware set to this rate)")
                ser.close()
                results.append({
                    'baud': baud, 'status': 'NO_RESPONSE',
                    'fps': 0, 'errors': -1
                })
                continue

            # Send motor packets at 60 Hz for TEST_DURATION seconds
            packets_sent = 0
            errors = 0
            start = time.time()

            while time.time() - start < TEST_DURATION:
                try:
                    # Sweep angle pattern
                    t = time.time() - start
                    angle = 90 + 45 * __import__('math').sin(t * 2)
                    packet = build_servo_packet([angle] * NUM_SERVOS)
                    ser.write(packet)
                    packets_sent += 1
                except (serial.SerialTimeoutException, OSError) as e:
                    errors += 1

                # 60 Hz
                time.sleep(1.0 / 60)

            # Get stats from ESP32
            esp_lines = read_esp_stats(ser, timeout=1.5)
            stats = parse_stats(esp_lines)

            result = {
                'baud': baud,
                'status': 'OK',
                'packets_sent': packets_sent,
                'errors': errors,
                'esp_fps': stats.get('FPS', 0),
                'esp_packets': stats.get('PKTS', 0),
                'drop_rate': f"{(1 - stats.get('PKTS', 0) / max(packets_sent, 1)) * 100:.1f}%"
            }
            results.append(result)

            print(f"  Sent: {packets_sent} | ESP recv FPS: {stats.get('FPS', '?')} | "
                  f"Errors: {errors} | Drop: {result['drop_rate']}")

            ser.close()
            time.sleep(0.3)

        except serial.SerialException as e:
            print(f"  ✗ Failed to open port at {baud}: {e}")
            results.append({'baud': baud, 'status': 'FAIL', 'error': str(e)})

    return results


def test_motor_send_rate(port, baud=460800):
    """Test motor packet send rate from 30 to 500 Hz."""
    print("\n" + "=" * 60)
    print(f"TEST 2: MOTOR SEND RATE SWEEP (at {baud} baud)")
    print("=" * 60)
    results = []

    try:
        ser = serial.Serial(port, baud, timeout=1, write_timeout=1.0)
        time.sleep(0.5)

        if not wait_for_ready(ser, timeout=3):
            print("  ⚠ ESP32 not responding")
            ser.close()
            return results

        for rate in MOTOR_SEND_RATES:
            print(f"\n--- Testing {rate} Hz send rate ---")

            # Reset ESP32 stats
            send_command(ser, "RESET")
            time.sleep(0.2)

            packets_sent = 0
            errors = 0
            start = time.time()
            interval = 1.0 / rate

            while time.time() - start < TEST_DURATION:
                try:
                    t = time.time() - start
                    angle = 90 + 45 * __import__('math').sin(t * 3)
                    packet = build_servo_packet([angle] * NUM_SERVOS)
                    ser.write(packet)
                    packets_sent += 1
                except (serial.SerialTimeoutException, OSError):
                    errors += 1

                # Precise timing
                elapsed = time.time() - start
                next_time = (packets_sent + 1) * interval
                sleep_time = next_time - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            # Read stats
            time.sleep(0.5)  # Let ESP32 report
            esp_lines = read_esp_stats(ser, timeout=1.5)
            stats = parse_stats(esp_lines)

            result = {
                'target_rate': rate,
                'packets_sent': packets_sent,
                'actual_rate': packets_sent / TEST_DURATION,
                'errors': errors,
                'esp_fps': stats.get('FPS', 0),
                'esp_packets': stats.get('PKTS', 0),
            }
            results.append(result)

            recv_rate = stats.get('PKTS', 0) / TEST_DURATION if stats.get('PKTS') else 0
            print(f"  Target: {rate} Hz | Actual send: {result['actual_rate']:.0f} Hz | "
                  f"ESP recv: {recv_rate:.0f} Hz | Errors: {errors}")

        ser.close()

    except serial.SerialException as e:
        print(f"  ✗ Serial error: {e}")

    return results


def test_latency(port, baud=460800):
    """Measure round-trip latency for motor packets."""
    print("\n" + "=" * 60)
    print(f"TEST 3: ROUND-TRIP LATENCY (at {baud} baud)")
    print("=" * 60)

    try:
        ser = serial.Serial(port, baud, timeout=0.1, write_timeout=1.0)
        time.sleep(0.5)

        if not wait_for_ready(ser, timeout=3):
            print("  ⚠ ESP32 not responding")
            ser.close()
            return {}

        # Enable ACK mode
        send_command(ser, "ACK:ON")
        time.sleep(0.2)

        latencies = []
        for i in range(100):
            # Flush input
            while ser.in_waiting:
                ser.read(ser.in_waiting)

            packet = build_servo_packet([90] * NUM_SERVOS)
            t0 = time.perf_counter()
            ser.write(packet)

            # Wait for ACK
            ack_received = False
            while time.perf_counter() - t0 < 0.1:  # 100ms timeout
                if ser.in_waiting:
                    data = ser.readline().decode('utf-8', errors='replace').strip()
                    if 'ACK' in data:
                        t1 = time.perf_counter()
                        latencies.append((t1 - t0) * 1000)  # ms
                        ack_received = True
                        break

            if not ack_received:
                latencies.append(-1)  # Timeout

            time.sleep(0.02)  # 50 Hz

        # Disable ACK mode
        send_command(ser, "ACK:OFF")
        ser.close()

        valid = [l for l in latencies if l > 0]
        if valid:
            result = {
                'min_ms': round(min(valid), 2),
                'max_ms': round(max(valid), 2),
                'avg_ms': round(sum(valid) / len(valid), 2),
                'median_ms': round(sorted(valid)[len(valid) // 2], 2),
                'timeouts': len([l for l in latencies if l < 0]),
                'samples': len(latencies)
            }
            print(f"  Min: {result['min_ms']}ms | Max: {result['max_ms']}ms | "
                  f"Avg: {result['avg_ms']}ms | Median: {result['median_ms']}ms | "
                  f"Timeouts: {result['timeouts']}/100")
        else:
            result = {'error': 'No ACKs received (firmware may not support ACK mode)'}
            print("  ✗ No ACKs received - flash stress_test_firmware.ino first")

        return result

    except serial.SerialException as e:
        print(f"  ✗ Serial error: {e}")
        return {'error': str(e)}


def test_write_timeout(port, baud=460800):
    """Test different write timeout values."""
    print("\n" + "=" * 60)
    print(f"TEST 4: WRITE TIMEOUT SWEEP (at {baud} baud)")
    print("=" * 60)
    results = []

    for timeout in WRITE_TIMEOUTS:
        print(f"\n--- Testing write_timeout={timeout}s ---")
        try:
            ser = serial.Serial(port, baud, timeout=1, write_timeout=timeout)
            time.sleep(0.3)

            packets_sent = 0
            timeout_errors = 0
            other_errors = 0
            start = time.time()

            while time.time() - start < TEST_DURATION:
                try:
                    t = time.time() - start
                    angle = 90 + 45 * __import__('math').sin(t * 3)
                    packet = build_servo_packet([angle] * NUM_SERVOS)
                    ser.write(packet)
                    packets_sent += 1
                except serial.SerialTimeoutException:
                    timeout_errors += 1
                except OSError:
                    other_errors += 1

                time.sleep(1.0 / 60)  # 60 Hz

            result = {
                'write_timeout': timeout,
                'packets_sent': packets_sent,
                'timeout_errors': timeout_errors,
                'other_errors': other_errors,
                'success_rate': f"{packets_sent / max(packets_sent + timeout_errors, 1) * 100:.1f}%"
            }
            results.append(result)

            print(f"  Sent: {packets_sent} | Timeouts: {timeout_errors} | "
                  f"Other: {other_errors} | Success: {result['success_rate']}")

            ser.close()
            time.sleep(0.2)

        except serial.SerialException as e:
            print(f"  ✗ Failed: {e}")
            results.append({'write_timeout': timeout, 'error': str(e)})

    return results


def test_firmware_params(port, baud=460800):
    """Test firmware parameters: smoothing alpha, loop delay, I2C speed, PWM freq."""
    print("\n" + "=" * 60)
    print(f"TEST 5: FIRMWARE PARAMETER SWEEP (at {baud} baud)")
    print("=" * 60)
    print("  Requires stress_test_firmware.ino on ESP32\n")
    results = {}

    try:
        ser = serial.Serial(port, baud, timeout=1, write_timeout=1.0)
        time.sleep(0.5)

        if not wait_for_ready(ser, timeout=3):
            print("  ⚠ ESP32 not responding")
            ser.close()
            return results

        # --- Test smoothing alpha ---
        print("  [Alpha sweep]")
        alpha_results = []
        for alpha in [0.3, 0.5, 0.8, 1.0]:
            resp = send_command(ser, f"ALPHA:{alpha}")
            print(f"    ALPHA={alpha}: {resp}")
            time.sleep(0.3)

            # Send ramp pattern and measure response
            send_command(ser, "RESET")
            start = time.time()
            while time.time() - start < 2:
                t = time.time() - start
                angle = 45 + 90 * (t / 2)  # Ramp from 45 to 135
                ser.write(build_servo_packet([angle] * NUM_SERVOS))
                time.sleep(1.0 / 60)

            time.sleep(0.5)
            stats = parse_stats(read_esp_stats(ser, timeout=1))
            alpha_results.append({'alpha': alpha, **stats})

        results['alpha'] = alpha_results

        # --- Test loop delay ---
        print("\n  [Loop delay sweep]")
        delay_results = []
        for delay_ms in [0, 1, 2, 5]:
            resp = send_command(ser, f"DELAY:{delay_ms}")
            print(f"    DELAY={delay_ms}ms: {resp}")
            send_command(ser, "RESET")

            start = time.time()
            while time.time() - start < 2:
                t = time.time() - start
                angle = 90 + 45 * __import__('math').sin(t * 4)
                ser.write(build_servo_packet([angle] * NUM_SERVOS))
                time.sleep(1.0 / 60)

            time.sleep(0.5)
            stats = parse_stats(read_esp_stats(ser, timeout=1))
            delay_results.append({'delay_ms': delay_ms, **stats})

        results['delay'] = delay_results

        # --- Test I2C speed ---
        print("\n  [I2C speed sweep]")
        i2c_results = []
        for speed_khz in [100, 400, 1000]:
            resp = send_command(ser, f"I2C:{speed_khz}")
            print(f"    I2C={speed_khz}kHz: {resp}")
            send_command(ser, "RESET")

            start = time.time()
            while time.time() - start < 2:
                t = time.time() - start
                angle = 90 + 45 * __import__('math').sin(t * 4)
                ser.write(build_servo_packet([angle] * NUM_SERVOS))
                time.sleep(1.0 / 60)

            time.sleep(0.5)
            stats = parse_stats(read_esp_stats(ser, timeout=1))
            i2c_results.append({'i2c_khz': speed_khz, **stats})

        results['i2c'] = i2c_results

        # --- Test PWM frequency ---
        print("\n  [PWM freq sweep]")
        pwm_results = []
        for freq in [50, 100, 200, 330]:
            resp = send_command(ser, f"PWMFREQ:{freq}")
            print(f"    PWMFREQ={freq}Hz: {resp}")
            send_command(ser, "RESET")

            start = time.time()
            while time.time() - start < 2:
                t = time.time() - start
                angle = 90 + 45 * __import__('math').sin(t * 4)
                ser.write(build_servo_packet([angle] * NUM_SERVOS))
                time.sleep(1.0 / 60)

            time.sleep(0.5)
            stats = parse_stats(read_esp_stats(ser, timeout=1))
            pwm_results.append({'pwm_freq': freq, **stats})

        results['pwm_freq'] = pwm_results

        # Reset to defaults
        send_command(ser, "ALPHA:0.3")
        send_command(ser, "DELAY:5")
        send_command(ser, "I2C:100")
        send_command(ser, "PWMFREQ:50")

        ser.close()

    except serial.SerialException as e:
        print(f"  ✗ Serial error: {e}")

    return results


# ======================= MAIN =======================

def main():
    parser = argparse.ArgumentParser(description='Motor Real-Time Stress Test')
    parser.add_argument('--port', required=True, help='Serial port (e.g. COM3)')
    parser.add_argument('--baud', type=int, default=460800, help='Base baud rate')
    parser.add_argument('--test', type=int, default=0,
                        help='Run specific test (1-5), 0=all')
    parser.add_argument('--duration', type=int, default=5,
                        help='Test duration in seconds')
    args = parser.parse_args()

    global TEST_DURATION
    TEST_DURATION = args.duration

    print("╔══════════════════════════════════════════════════╗")
    print("║   MOTOR REAL-TIME PERFORMANCE STRESS TEST       ║")
    print("║   Mirror & Motors Project                       ║")
    print("╚══════════════════════════════════════════════════╝")
    print(f"\nPort: {args.port} | Base baud: {args.baud} | Duration: {TEST_DURATION}s per test")
    print(f"Timestamp: {datetime.now().isoformat()}\n")

    all_results = {
        'timestamp': datetime.now().isoformat(),
        'port': args.port,
        'base_baud': args.baud,
        'duration': TEST_DURATION,
    }

    tests = {
        1: ('baud_rates', lambda: test_baud_rates(args.port)),
        2: ('motor_send_rate', lambda: test_motor_send_rate(args.port, args.baud)),
        3: ('latency', lambda: test_latency(args.port, args.baud)),
        4: ('write_timeout', lambda: test_write_timeout(args.port, args.baud)),
        5: ('firmware_params', lambda: test_firmware_params(args.port, args.baud)),
    }

    if args.test == 0:
        # Run all tests
        for test_num in sorted(tests.keys()):
            name, func = tests[test_num]
            try:
                all_results[name] = func()
            except Exception as e:
                print(f"\n ✗ Test {test_num} failed: {e}")
                all_results[name] = {'error': str(e)}
    else:
        if args.test in tests:
            name, func = tests[args.test]
            try:
                all_results[name] = func()
            except Exception as e:
                print(f"\n ✗ Test failed: {e}")
                all_results[name] = {'error': str(e)}
        else:
            print(f"Unknown test {args.test}. Valid: 1-5")
            return

    # Save results
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'logs')
    os.makedirs(log_dir, exist_ok=True)
    log_file = os.path.join(log_dir, 'stress_test_results.json')

    with open(log_file, 'w') as f:
        json.dump(all_results, f, indent=2, default=str)

    print(f"\n{'=' * 60}")
    print(f"Results saved to: {os.path.abspath(log_file)}")
    print(f"{'=' * 60}")


if __name__ == '__main__':
    main()
