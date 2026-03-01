#!/usr/bin/env python3
"""
Linux-side position controller using TORQUE mode with fast CAN feedback.

Uses FAST_FRAME (TPDO4 at 0x481) for position feedback,
sends torque commands via PDO3 (RPDO3 at 0x401).

Usage: python3 position_test.py [target_degrees] [duration_seconds]
"""

import socket
import struct
import time
import math
import sys

DEVICE_ID = 1
CAN_INTERFACE = "can0"

# Motor parameters
TORQUE_CONSTANT = 0.08958

# Control parameters
TORQUE_LIMIT = 0.3       # Nm
I_LIMIT = 5.0
FAST_FRAME_HZ = 500

# PD gains
KP = 1.5       # Nm/rad
KD = 0.02      # Nm/(rad/s)
KI = 0.3       # Nm/(rad*s)

# Trajectory limits
MAX_VEL = 5.0         # rad/s max trajectory velocity
MAX_ACCEL = 20.0      # rad/s^2 max trajectory acceleration

# CAN IDs
FUNC_NMT = 0
FUNC_RECEIVE_PDO_3 = 8  # 0x401 = [position_target, torque_target]
FUNC_TRANSMIT_PDO_4 = 9  # 0x481 = fast frame [position, velocity]
FUNC_TRANSMIT_SDO = 0xB
FUNC_RECEIVE_SDO = 0xC
FUNC_FLASH = 0xD
FUNC_HEARTBEAT = 0xE


def make_can_id(func, dev_id):
    return (func << 7) | dev_id


class CANBus:
    def __init__(self, interface):
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))

    def send(self, can_id, data=b""):
        dlc = len(data)
        self.sock.send(struct.pack("<IB3x8s", can_id, dlc, data.ljust(8, b'\x00')))

    def recv(self, timeout_s=0.1):
        """Receive a CAN frame with timeout. Returns (can_id, data) or None."""
        self.sock.settimeout(timeout_s)
        try:
            raw = self.sock.recv(16)
            can_id, dlc = struct.unpack("<IB3x", raw[:8])
            return (can_id & 0x7FF, raw[8:8 + dlc])
        except socket.timeout:
            return None

    def drain(self):
        """Drain all pending frames."""
        self.sock.settimeout(0)
        try:
            while True:
                self.sock.recv(16)
        except (BlockingIOError, socket.timeout):
            pass

    def sdo_read_float(self, param_id):
        self.send(make_can_id(FUNC_RECEIVE_SDO, DEVICE_ID),
                  struct.pack('<BH', 0x40, param_id))
        expected = make_can_id(FUNC_TRANSMIT_SDO, DEVICE_ID)
        deadline = time.time() + 0.5
        while time.time() < deadline:
            r = self.recv(0.05)
            if r and r[0] == expected and len(r[1]) >= 4:
                return struct.unpack('<f', r[1][:4])[0]
        return None

    def sdo_write_float(self, param_id, value):
        iv = struct.unpack('<I', struct.pack('<f', value))[0]
        self.send(make_can_id(FUNC_RECEIVE_SDO, DEVICE_ID),
                  struct.pack('<BHxI', 0x20, param_id, iv))

    def sdo_write_u32(self, param_id, value):
        self.send(make_can_id(FUNC_RECEIVE_SDO, DEVICE_ID),
                  struct.pack('<BHxI', 0x20, param_id, value))

    def sdo_read_u32(self, param_id):
        self.send(make_can_id(FUNC_RECEIVE_SDO, DEVICE_ID),
                  struct.pack('<BH', 0x40, param_id))
        expected = make_can_id(FUNC_TRANSMIT_SDO, DEVICE_ID)
        deadline = time.time() + 0.5
        while time.time() < deadline:
            r = self.recv(0.05)
            if r and r[0] == expected and len(r[1]) >= 4:
                return struct.unpack('<I', r[1][:4])[0]
        return None

    def send_nmt(self, mode):
        self.send(make_can_id(FUNC_NMT, 0), struct.pack('BB', mode, DEVICE_ID))

    def send_heartbeat(self):
        self.send(make_can_id(FUNC_HEARTBEAT, DEVICE_ID), b'')

    def send_torque(self, torque):
        """Send torque command via PDO3: [position_target(ignored), torque_target]."""
        self.send(make_can_id(FUNC_RECEIVE_PDO_3, DEVICE_ID),
                  struct.pack('<ff', 0.0, torque))

    def close(self):
        self.sock.close()


def main():
    target_deg = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    can = CANBus(CAN_INTERFACE)

    # Reset
    print("Resetting motor...")
    can.sdo_write_u32(0x014, 0)
    can.send_heartbeat()
    can.send_nmt(0x00)  # DISABLED
    time.sleep(0.5)
    for _ in range(5):
        can.send_heartbeat()
        time.sleep(0.1)
    can.send_nmt(0x01)  # IDLE
    time.sleep(0.5)
    for _ in range(5):
        can.send_heartbeat()
        time.sleep(0.1)

    mode = can.sdo_read_u32(0x010)
    err = can.sdo_read_u32(0x014)
    print(f"  mode=0x{mode:02X}, error=0x{err:08X}")

    # Configure
    can.sdo_write_float(0x074, I_LIMIT)
    can.sdo_write_float(0x030, TORQUE_LIMIT)
    can.sdo_write_u32(0x00C, FAST_FRAME_HZ)
    time.sleep(0.2)

    # Read starting position
    can.drain()
    pos_start = can.sdo_read_float(0x134)
    if pos_start is None:
        print("ERROR: Cannot read position")
        can.close()
        return 1

    target_rad = pos_start + math.radians(target_deg)
    print(f"Start: {math.degrees(pos_start):.1f} deg")
    print(f"Target: {math.degrees(target_rad):.1f} deg  (+{target_deg} deg)")
    print(f"Gains: Kp={KP}, Kd={KD}, Ki={KI}")
    print()

    # Enter TORQUE mode
    can.send_torque(0.0)
    time.sleep(0.05)
    can.send_nmt(0x11)  # TORQUE
    time.sleep(0.3)
    can.send_heartbeat()
    can.drain()

    mode = can.sdo_read_u32(0x010)
    err = can.sdo_read_u32(0x014)
    print(f"  Torque mode: mode=0x{mode:02X}, error=0x{err:08X}")
    if mode != 0x11:
        print("ERROR: Failed to enter TORQUE mode!")
        can.close()
        return 1

    # Control state
    prev_pos = pos_start
    prev_time = time.time()
    vel_filtered = 0.0
    vel_alpha = 0.15
    integral = 0.0
    integral_limit = TORQUE_LIMIT * 0.5

    # Trajectory state
    traj_pos = pos_start   # current trajectory setpoint
    traj_vel = 0.0         # current trajectory velocity

    fast_frame_id = make_can_id(FUNC_TRANSMIT_PDO_4, DEVICE_ID)
    start = time.time()
    last_print = 0
    last_hb = 0
    frame_count = 0

    print(f"{'time':>5s}  {'error':>8s}  {'pos':>9s}  {'setpt':>9s}  {'vel':>8s}  {'torque':>8s}")
    print("-" * 58)

    try:
        while time.time() - start < duration:
            frame = can.recv(timeout_s=0.01)
            now = time.time()

            # Heartbeat every 50ms
            if now - last_hb >= 0.05:
                can.send_heartbeat()
                last_hb = now

            if frame is None:
                continue

            can_id, data = frame
            if can_id != fast_frame_id or len(data) < 8:
                continue

            frame_count += 1
            pos = struct.unpack('<f', data[0:4])[0]

            # Velocity estimation
            dt = now - prev_time
            if dt < 0.0005:
                continue
            vel_raw = (pos - prev_pos) / dt
            vel_filtered = vel_alpha * vel_raw + (1 - vel_alpha) * vel_filtered
            prev_pos = pos
            prev_time = now

            # Trajectory generator: ramp traj_pos toward target_rad
            pos_to_go = target_rad - traj_pos
            direction = 1.0 if pos_to_go > 0 else -1.0

            # Deceleration distance: v^2 / (2*a)
            decel_dist = traj_vel * traj_vel / (2.0 * MAX_ACCEL)

            if abs(pos_to_go) <= 0.001 and abs(traj_vel) < 0.1:
                # Close enough, hold position
                traj_pos = target_rad
                traj_vel = 0.0
            elif abs(pos_to_go) <= decel_dist:
                # Decelerate
                traj_vel -= direction * MAX_ACCEL * dt
                if direction * traj_vel < 0:
                    traj_vel = 0.0
            else:
                # Accelerate toward target (up to max vel)
                traj_vel += direction * MAX_ACCEL * dt
                traj_vel = max(-MAX_VEL, min(MAX_VEL, traj_vel))

            traj_pos += traj_vel * dt

            # PD+I controller tracks trajectory setpoint
            pos_error = traj_pos - pos
            vel_error = traj_vel - vel_filtered

            integral += KI * pos_error * dt
            integral = max(-integral_limit, min(integral_limit, integral))

            torque = KP * pos_error + KD * vel_error + integral
            torque = max(-TORQUE_LIMIT, min(TORQUE_LIMIT, torque))

            # Send torque via PDO3
            can.send_torque(torque)

            # Print
            elapsed = now - start
            if elapsed - last_print >= 0.25:
                print(f"{elapsed:5.1f}s  {math.degrees(pos_error):+7.1f}deg  "
                      f"{math.degrees(pos):8.1f}deg  "
                      f"{math.degrees(traj_pos):8.1f}deg  "
                      f"{vel_filtered:+7.1f}r/s  "
                      f"{torque:+.4f}Nm")
                last_print = elapsed

    except KeyboardInterrupt:
        print("\nInterrupted!")

    # Stop
    print("\nStopping...")
    can.send_torque(0.0)
    time.sleep(0.1)
    can.send_nmt(0x02)  # DAMPING

    elapsed = time.time() - start
    print(f"Frames: {frame_count} ({frame_count/max(elapsed,0.1):.0f} Hz)")

    final_pos = can.sdo_read_float(0x134)
    if final_pos is not None:
        print(f"Final: {math.degrees(final_pos):.1f} deg  (error: {math.degrees(target_rad - final_pos):+.1f} deg)")

    print("Done.")
    can.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
