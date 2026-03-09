#!/usr/bin/env python3
"""
Two-stage position control test over CAN.

Architecture:
  - STM32 (inner loop): Velocity PID via SimpleFOC (~10kHz)
  - This script (outer loop): Position P-controller over CAN (~50Hz)
    Reads AS5600 position feedback, outputs velocity commands.

The AS5600 is too slow for direct position PID on the motor side,
but works well for a host-side position loop that generates smooth
velocity setpoints for the fast inner loop.

Usage: python3 position_move_test.py [--vel-limit 5.0] [--move-deg 360]
"""

import socket
import struct
import time
import math
import sys
import argparse

# CAN config
CAN_INTERFACE = "can0"
DEVICE_ID = 1

# CAN IDs (match firmware)
CAN_ID_NMT       = 0x001
CAN_ID_PDO2_TX   = 0x301   # Host -> motor: [velocity_f32, unused_f32]
CAN_ID_PDO2_RX   = 0x281   # Motor -> host: [position_f32, velocity_f32]
CAN_ID_HEARTBEAT = 0x701

# Motor modes
MODE_DISABLED = 0x00
MODE_VELOCITY = 0x12


class CANBus:
    def __init__(self, interface):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))

    def send(self, can_id, data=b""):
        dlc = len(data)
        self.sock.send(struct.pack("<IB3x8s", can_id, dlc, data.ljust(8, b'\x00')))

    def recv(self, timeout_s=0.1):
        self.sock.settimeout(timeout_s)
        try:
            raw = self.sock.recv(16)
            can_id, dlc = struct.unpack("<IB3x", raw[:8])
            return (can_id & 0x7FF, raw[8:8 + dlc])
        except socket.timeout:
            return None

    def drain(self):
        self.sock.settimeout(0)
        try:
            while True:
                self.sock.recv(16)
        except (BlockingIOError, socket.timeout):
            pass

    def send_heartbeat(self):
        self.send(CAN_ID_HEARTBEAT, b'')

    def send_nmt(self, mode):
        self.send(CAN_ID_NMT, struct.pack('BB', mode, DEVICE_ID))

    def send_velocity(self, vel_rad_s):
        self.send(CAN_ID_PDO2_TX, struct.pack('<ff', vel_rad_s, 0.0))

    def close(self):
        self.sock.close()


def read_feedback(can, timeout_s=0.05):
    """Read one feedback frame. Returns (position, velocity) or None."""
    result = can.recv(timeout_s)
    if result is None:
        return None
    can_id, data = result
    if can_id == CAN_ID_PDO2_RX and len(data) >= 8:
        pos = struct.unpack('<f', data[0:4])[0]
        vel = struct.unpack('<f', data[4:8])[0]
        return (pos, vel)
    # Got a different frame (heartbeat echo, etc), not feedback
    return None


def drain_and_get_latest(can):
    """Drain all pending frames and return the latest feedback."""
    latest = None
    while True:
        fb = read_feedback(can, timeout_s=0)
        if fb is None:
            break
        latest = fb
    return latest


def main():
    parser = argparse.ArgumentParser(description="Position move test over CAN")
    parser.add_argument('--vel-limit', type=float, default=5.0,
                        help='Max velocity command (rad/s), default 5.0')
    parser.add_argument('--move-deg', type=float, default=360.0,
                        help='Move distance in degrees, default 360')
    parser.add_argument('--kp', type=float, default=2.0,
                        help='Position P gain (vel_cmd = Kp * pos_error), default 2.0')
    parser.add_argument('--settle-time', type=float, default=2.0,
                        help='Time to wait at each end (seconds), default 2.0')
    parser.add_argument('--cycles', type=int, default=1,
                        help='Number of back-and-forth cycles, default 1')
    parser.add_argument('--accel-limit', type=float, default=10.0,
                        help='Max acceleration (rad/s^2), default 10.0')
    args = parser.parse_args()

    move_rad = math.radians(args.move_deg)
    vel_limit = args.vel_limit
    kp = args.kp
    accel_limit = args.accel_limit

    print(f"=== Position Move Test ===")
    print(f"  Move: {args.move_deg:.0f} deg ({move_rad:.2f} rad)")
    print(f"  Vel limit: {vel_limit:.1f} rad/s")
    print(f"  Kp: {kp:.1f}")
    print(f"  Accel limit: {accel_limit:.1f} rad/s^2")
    print(f"  Cycles: {args.cycles}")
    print()

    # Connect to CAN
    can = CANBus(CAN_INTERFACE)

    # Activate motor: send heartbeats, then enable velocity mode
    print("Connecting to motor...")
    for _ in range(10):
        can.send_heartbeat()
        time.sleep(0.05)

    can.drain()

    # Read initial position
    print("Reading initial position...")
    can.send_heartbeat()
    time.sleep(0.1)
    pos_start = None
    for _ in range(20):
        fb = read_feedback(can, timeout_s=0.1)
        if fb is not None:
            pos_start = fb[0]
            break
        can.send_heartbeat()

    if pos_start is None:
        print("ERROR: No feedback from motor. Check CAN connection.")
        can.close()
        return 1

    print(f"  Start position: {math.degrees(pos_start):.1f} deg ({pos_start:.3f} rad)")

    # Enable velocity mode
    print("Enabling velocity mode...")
    can.send_velocity(0.0)  # Zero velocity first
    time.sleep(0.05)
    can.send_nmt(MODE_VELOCITY)
    time.sleep(0.3)
    can.send_heartbeat()
    print("  Motor enabled in velocity mode")
    print()

    # Position control loop
    current_vel_cmd = 0.0
    pos_target = pos_start  # Start at current position

    # Build list of moves: forward, wait, back, wait (repeated)
    moves = []
    for _ in range(args.cycles):
        moves.append(("forward", pos_start + move_rad))
        moves.append(("reverse", pos_start))

    last_hb_time = 0
    last_print_time = 0

    print(f"{'time':>5s}  {'target':>9s}  {'actual':>9s}  {'error':>8s}  {'vel_cmd':>8s}  {'vel_act':>8s}")
    print("-" * 62)

    try:
        for move_name, target in moves:
            pos_target = target
            print(f"\n--- {move_name}: target = {math.degrees(pos_target):.1f} deg ---")

            move_start = time.time()
            settled = False
            settle_start = None

            while True:
                now = time.time()

                # Send heartbeat every 80ms
                if now - last_hb_time >= 0.08:
                    can.send_heartbeat()
                    last_hb_time = now

                # Get latest feedback (drain queue)
                latest = drain_and_get_latest(can)
                if latest is None:
                    # Try blocking read
                    latest = read_feedback(can, timeout_s=0.02)
                if latest is None:
                    continue

                pos_actual, vel_actual = latest

                # Position P controller with velocity limiting
                pos_error = pos_target - pos_actual
                vel_desired = kp * pos_error

                # Clamp velocity to limit
                vel_desired = max(-vel_limit, min(vel_limit, vel_desired))

                # Rate-limit acceleration
                dt = 0.02  # ~50Hz feedback rate
                max_dv = accel_limit * dt
                dv = vel_desired - current_vel_cmd
                if abs(dv) > max_dv:
                    dv = max_dv if dv > 0 else -max_dv
                current_vel_cmd += dv

                # Send velocity command
                can.send_velocity(current_vel_cmd)

                # Print status at ~4Hz
                if now - last_print_time >= 0.25:
                    print(f"{now - move_start:5.1f}s  "
                          f"{math.degrees(pos_target):+8.1f}d  "
                          f"{math.degrees(pos_actual):+8.1f}d  "
                          f"{math.degrees(pos_error):+7.1f}d  "
                          f"{current_vel_cmd:+7.2f}r/s  "
                          f"{vel_actual:+7.2f}r/s")
                    last_print_time = now

                # Check if settled (within 2 degrees for settle_time seconds)
                if abs(pos_error) < math.radians(2.0) and abs(vel_actual) < 0.5:
                    if settle_start is None:
                        settle_start = now
                    elif now - settle_start >= args.settle_time:
                        settled = True
                else:
                    settle_start = None

                if settled:
                    print(f"  Settled at {math.degrees(pos_actual):.1f} deg "
                          f"(error: {math.degrees(pos_error):+.1f} deg)")
                    break

                # Safety timeout
                if now - move_start > 30:
                    print("  TIMEOUT - move took too long!")
                    break

    except KeyboardInterrupt:
        print("\n\nInterrupted!")

    # Stop motor
    print("\nStopping motor...")
    can.send_velocity(0.0)
    time.sleep(0.1)
    can.send_velocity(0.0)
    time.sleep(0.1)
    can.send_nmt(MODE_DISABLED)
    time.sleep(0.1)

    # Final position
    can.send_heartbeat()
    time.sleep(0.1)
    fb = drain_and_get_latest(can)
    if fb:
        print(f"Final position: {math.degrees(fb[0]):.1f} deg")

    print("Done.")
    can.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
