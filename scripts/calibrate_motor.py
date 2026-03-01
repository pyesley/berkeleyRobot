#!/usr/bin/env python3
"""
Recoil Motor Calibration Script (CAN-only)

Sends calibration command via CAN, polls motor mode via SDO reads
to detect completion, then saves config to flash.

Usage: python3 calibrate_motor.py
"""

import socket
import struct
import time
import sys

DEVICE_ID = 1
CAN_INTERFACE = "can0"

# CAN function codes (shifted into ID)
FUNC_NMT            = 0x0
FUNC_TRANSMIT_SDO   = 0xB   # Response from motor
FUNC_RECEIVE_SDO    = 0xC   # Request to motor
FUNC_FLASH           = 0xD
FUNC_HEARTBEAT       = 0xE

# Modes
MODE_DISABLED    = 0x00
MODE_IDLE        = 0x01
MODE_CALIBRATION = 0x05
MODE_POSITION    = 0x13

# SDO parameters
PARAM_MODE  = 0x010
PARAM_ERROR = 0x014

MODE_NAMES = {
    0x00: "DISABLED",
    0x01: "IDLE",
    0x02: "DAMPING",
    0x05: "CALIBRATION",
    0x10: "CURRENT",
    0x11: "TORQUE",
    0x12: "VELOCITY",
    0x13: "POSITION",
    0x20: "VABC_OVERRIDE",
    0x21: "VALPHABETA_OVERRIDE",
    0x22: "VQD_OVERRIDE",
    0x80: "DEBUG",
}


def make_can_id(func, dev_id):
    return (func << 7) | dev_id


def can_send(sock, can_id, data=b""):
    """Send a CAN frame."""
    dlc = len(data)
    data_padded = data.ljust(8, b'\x00')
    frame = struct.pack("<IB3x8s", can_id, dlc, data_padded)
    sock.send(frame)


def can_recv(sock, timeout_ms=100):
    """Receive a CAN frame. Returns (can_id, data) or None."""
    sock.settimeout(timeout_ms / 1000.0)
    try:
        raw = sock.recv(16)
        can_id, dlc = struct.unpack("<IB3x", raw[:8])
        can_id &= 0x7FF  # Mask to 11-bit standard ID
        data = raw[8:8 + dlc]
        return (can_id, data)
    except socket.timeout:
        return None


def sdo_read(sock, param_id):
    """Read a 32-bit parameter via SDO. Returns uint32 value or None."""
    sdo_id = make_can_id(FUNC_RECEIVE_SDO, DEVICE_ID)
    # CCS=2 (upload/read) in bits 5-7 of byte 0 = 0x40
    # Bytes 1-2: parameter ID (little-endian)
    data = struct.pack("<BH", 0x40, param_id)
    can_send(sock, sdo_id, data)

    # Wait for response on FUNC_TRANSMIT_SDO
    expected_id = make_can_id(FUNC_TRANSMIT_SDO, DEVICE_ID)
    deadline = time.time() + 0.5
    while time.time() < deadline:
        result = can_recv(sock, timeout_ms=50)
        if result is None:
            continue
        rx_id, rx_data = result
        if rx_id == expected_id and len(rx_data) >= 4:
            return struct.unpack("<I", rx_data[:4])[0]
    return None


def read_mode(sock):
    """Read current motor mode via SDO."""
    val = sdo_read(sock, PARAM_MODE)
    if val is not None:
        return val & 0xFF
    return None


def read_error(sock):
    """Read current error flags via SDO."""
    return sdo_read(sock, PARAM_ERROR)


def main():
    print("=== Recoil Motor Calibration (CAN-only) ===")
    print()

    # Open CAN socket
    print(f"Opening CAN interface {CAN_INTERFACE}...")
    can_sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_sock.bind((CAN_INTERFACE,))

    # Step 0: Verify communication by reading current mode
    print("Checking motor communication...")
    mode = read_mode(can_sock)
    if mode is None:
        print("ERROR: Cannot communicate with motor via CAN SDO.")
        print("Check CAN wiring and motor power.")
        can_sock.close()
        return 1
    print(f"  Current mode: 0x{mode:02X} ({MODE_NAMES.get(mode, 'UNKNOWN')})")

    err = read_error(can_sock)
    if err is not None and err != 0:
        print(f"  Current error: 0x{err:04X}")

    # Step 1: Send IDLE mode first
    print()
    print("[1/4] Setting motor to IDLE mode...")
    nmt_id = make_can_id(FUNC_NMT, 0)  # NMT uses ID 0
    can_send(can_sock, nmt_id, struct.pack("BB", MODE_IDLE, DEVICE_ID))
    time.sleep(0.5)

    mode = read_mode(can_sock)
    if mode is not None:
        print(f"  Mode after IDLE command: 0x{mode:02X} ({MODE_NAMES.get(mode, 'UNKNOWN')})")

    # Step 2: Send calibration mode
    print()
    print("[2/4] Starting calibration (MODE_CALIBRATION = 0x05)...")
    print("       The motor will slowly rotate to find encoder offset.")
    print("       DO NOT touch the motor during calibration!")
    print()
    can_send(can_sock, nmt_id, struct.pack("BB", MODE_CALIBRATION, DEVICE_ID))
    time.sleep(0.5)

    # Step 3: Poll mode via SDO to detect when calibration finishes
    print("Polling motor mode via SDO...")
    start = time.time()
    timeout = 60  # 60 second max
    calibration_done = False
    last_print = 0

    while time.time() - start < timeout:
        # Send heartbeat to keep watchdog alive
        hb_id = make_can_id(FUNC_HEARTBEAT, DEVICE_ID)
        can_send(can_sock, hb_id, b"")

        mode = read_mode(can_sock)
        elapsed = time.time() - start

        if elapsed - last_print >= 2.0:
            mode_name = MODE_NAMES.get(mode, "UNKNOWN") if mode is not None else "NO RESPONSE"
            err = read_error(can_sock)
            err_str = f"  error=0x{err:04X}" if err else ""
            print(f"  [{elapsed:.0f}s] mode=0x{mode:02X} ({mode_name}){err_str}" if mode is not None
                  else f"  [{elapsed:.0f}s] No SDO response")
            last_print = elapsed

        if mode is not None and mode != MODE_CALIBRATION:
            # Calibration finished — motor switched to a different mode
            if mode == MODE_IDLE:
                print(f"\n  Calibration completed! Motor returned to IDLE mode.")
                calibration_done = True
            elif mode == MODE_DISABLED:
                err = read_error(can_sock)
                print(f"\n  *** Motor entered DISABLED mode (error=0x{err:04X}). Calibration may have failed. ***")
            else:
                print(f"\n  Motor changed to mode 0x{mode:02X} ({MODE_NAMES.get(mode, 'UNKNOWN')})")
                calibration_done = True
            break

        time.sleep(0.2)

    if not calibration_done:
        print(f"\nCalibration did not complete within {timeout}s.")
        print("Check motor connections and power supply.")
        can_sock.close()
        return 1

    # Step 3b: Save config to flash
    print()
    print("[3/4] Saving calibration to flash...")
    time.sleep(0.5)
    flash_id = make_can_id(FUNC_FLASH, DEVICE_ID)
    can_send(can_sock, flash_id, struct.pack("B", 1))
    time.sleep(1.0)

    # Step 4: Set position mode
    print("[4/4] Setting motor to POSITION mode (0x13)...")
    can_send(can_sock, nmt_id, struct.pack("BB", MODE_POSITION, DEVICE_ID))
    time.sleep(0.5)

    mode = read_mode(can_sock)
    if mode is not None:
        print(f"  Final mode: 0x{mode:02X} ({MODE_NAMES.get(mode, 'UNKNOWN')})")

    print()
    print("=== Calibration complete! ===")
    print("Motor should now respond to position commands.")
    print()
    print("To make calibration persistent:")
    print("  1. Set LOAD_*_FROM_FLASH = 1 in motor_controller_conf.h")
    print("  2. Rebuild and reflash firmware")

    can_sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
