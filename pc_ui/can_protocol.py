"""CAN protocol message encoding/decoding for B-G431B-ESC1 position controller."""

import struct

# CAN message IDs
CAN_ID_SET_POSITION  = 0x101
CAN_ID_SET_PID_GAINS = 0x102
CAN_ID_COMMAND       = 0x103
CAN_ID_TELEMETRY     = 0x110
CAN_ID_STATUS        = 0x111

# Command bytes
CMD_ENABLE  = 0x01
CMD_DISABLE = 0x02
CMD_HOME    = 0x03
CMD_ESTOP   = 0x04

# Position controller states
POS_STATES = {
    0: "DISABLED",
    1: "IDLE",
    2: "STARTING",
    3: "RUNNING",
    4: "STOPPING",
    5: "FAULT",
}

# MCSDK motor states
MC_STATES = {
    0: "IDLE",
    2: "ALIGNMENT",
    4: "START",
    6: "RUN",
    8: "STOP",
    10: "FAULT_NOW",
    11: "FAULT_OVER",
    12: "ICLWAIT",
    16: "CHARGE_BOOT_CAP",
    17: "OFFSET_CALIB",
    19: "SWITCH_OVER",
    20: "WAIT_STOP_MOTOR",
}


def encode_set_position(target_deg: float) -> tuple[int, bytes]:
    return CAN_ID_SET_POSITION, struct.pack("<f", target_deg)


def encode_set_pid_gains(kp: float, ki: float) -> tuple[int, bytes]:
    return CAN_ID_SET_PID_GAINS, struct.pack("<ff", kp, ki)


def encode_command(cmd: int) -> tuple[int, bytes]:
    return CAN_ID_COMMAND, bytes([cmd])


def decode_telemetry(data: bytes) -> dict:
    actual_deg, target_deg = struct.unpack("<ff", data[:8])
    return {"actual_deg": actual_deg, "target_deg": target_deg}


def decode_status(data: bytes) -> dict:
    speed_rpm = struct.unpack("<f", data[:4])[0]
    pos_state = data[4]
    mc_state = data[5]
    faults = data[6] | (data[7] << 8)
    return {
        "speed_rpm": speed_rpm,
        "pos_state": POS_STATES.get(pos_state, f"UNKNOWN({pos_state})"),
        "mc_state": MC_STATES.get(mc_state, f"UNKNOWN({mc_state})"),
        "faults": faults,
    }
