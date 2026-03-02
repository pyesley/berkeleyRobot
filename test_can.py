"""Test script to verify CAN bus communication with B-G431B-ESC1.

The firmware sends telemetry (0x110) at 100Hz and status (0x111) at 10Hz.
This script listens for those messages to confirm the CAN link works,
then sends a test command to verify RX on the board side.
"""

import struct
import time

# Ensure libusb backend is available for gs_usb on Windows
import libusb_package
import usb.core
usb.core.find(backend=libusb_package.get_libusb1_backend())

import can

# CAN IDs (must match can_interface.h)
CAN_ID_SET_POSITION  = 0x101
CAN_ID_SET_PID_GAINS = 0x102
CAN_ID_COMMAND       = 0x103
CAN_ID_TELEMETRY     = 0x110
CAN_ID_STATUS        = 0x111

# Command bytes
CAN_CMD_ENABLE  = 0x01
CAN_CMD_DISABLE = 0x02
CAN_CMD_HOME    = 0x03
CAN_CMD_ESTOP   = 0x04


def open_bus():
    bus = can.Bus(interface='gs_usb', channel=0, bitrate=1000000)
    print("CAN bus opened (gs_usb, 1 Mbit/s)")
    return bus


def decode_telemetry(data):
    actual_deg = struct.unpack_from('<f', data, 0)[0]
    target_deg = struct.unpack_from('<f', data, 4)[0]
    return actual_deg, target_deg


def decode_status(data):
    speed_rpm = struct.unpack_from('<f', data, 0)[0]
    state = data[4]
    mc_state = data[5]
    faults = data[6] | (data[7] << 8)

    state_names = {0: "DISABLED", 1: "IDLE", 2: "STARTING",
                   3: "RUNNING", 4: "STOPPING", 5: "FAULT"}
    mc_state_names = {0: "IDLE", 1: "CHARGE_BOOT", 2: "OFFSET_CALIB",
                      3: "CLEAR", 4: "START", 5: "ALIGN", 6: "STARTUP",
                      7: "SWITCH_OVER", 8: "RUN", 9: "STOP",
                      10: "FAULT_NOW", 11: "FAULT_OVER", 12: "WAIT_STOP_MOTOR"}
    return {
        "speed_rpm": speed_rpm,
        "state": state_names.get(state, f"?{state}"),
        "mc_state": mc_state_names.get(mc_state, f"?{mc_state}"),
        "faults": faults,
    }


def main():
    print("=== CAN Bus Test ===\n")

    bus = open_bus()

    # Phase 1: Listen for messages from the board
    print("\nListening for board messages (5 seconds)...")
    print("  Expected: telemetry (0x110) at 100Hz, status (0x111) at 10Hz\n")

    telem_count = 0
    status_count = 0
    other_count = 0
    start = time.time()

    while time.time() - start < 5.0:
        msg = bus.recv(timeout=0.5)
        if msg is None:
            continue

        if msg.arbitration_id == CAN_ID_TELEMETRY:
            telem_count += 1
            if telem_count <= 3:  # Print first few
                actual, target = decode_telemetry(msg.data)
                print(f"  [0x110] Telemetry: actual={actual:.2f} deg, target={target:.2f} deg")
        elif msg.arbitration_id == CAN_ID_STATUS:
            status_count += 1
            if status_count <= 3:
                s = decode_status(msg.data)
                print(f"  [0x111] Status: state={s['state']}, mc={s['mc_state']}, "
                      f"speed={s['speed_rpm']:.1f}, faults=0x{s['faults']:04X}")
        else:
            other_count += 1
            if other_count <= 3:
                print(f"  [0x{msg.arbitration_id:03X}] Unknown: {msg.data.hex()}")

    elapsed = time.time() - start
    print(f"\nResults ({elapsed:.1f}s):")
    print(f"  Telemetry (0x110): {telem_count} messages ({telem_count/elapsed:.0f}/s, expected ~100/s)")
    print(f"  Status    (0x111): {status_count} messages ({status_count/elapsed:.0f}/s, expected ~10/s)")
    if other_count:
        print(f"  Other:             {other_count} messages")

    if telem_count == 0 and status_count == 0:
        print("\n  WARNING: No messages received from board!")
        print("  Check: CAN transceiver wiring, termination resistor, FDCAN init on board")
    else:
        print("\n  CAN RX from board: OK!")

    # Phase 2: Send a command and check response
    print("\n--- Testing TX to board ---")
    print("Sending DISABLE command (0x103)...")
    msg = can.Message(
        arbitration_id=CAN_ID_COMMAND,
        data=[CAN_CMD_DISABLE],
        is_extended_id=False,
    )
    try:
        bus.send(msg)
        print("  Sent OK")
    except can.CanError as e:
        print(f"  Send FAILED: {e}")

    bus.shutdown()
    print("\nDone.")


if __name__ == "__main__":
    main()
