"""Test script to verify AS5600 encoder reads via ST-Link.

Reads encoder data from MCU memory every second. Manually rotate the motor
shaft to see values change. Press Ctrl+C to stop.
"""

import subprocess
import struct
import time

PROGRAMMER = (
    "C:/ST/STM32CubeIDE_1.18.1/STM32CubeIDE/plugins/"
    "com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.100.202412061334"
    "/tools/bin/STM32_Programmer_CLI.exe"
)

# Memory address (from .map file)
ADDR_ENCODER = 0x20000A10

# g_encoder struct offsets
OFF_RAW_ANGLE   = 0x04  # uint16
OFF_TURN_COUNT  = 0x08  # int32
OFF_MOTOR_DEG   = 0x10  # float
OFF_OUTPUT_DEG  = 0x14  # float
OFF_INITIALIZED = 0x18  # bool


def run_cli(*args):
    cmd = [PROGRAMMER, "-c", "port=SWD", "mode=HOTPLUG"] + list(args)
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    return result.stdout + result.stderr


def read_mem(addr, size):
    """Read memory and return raw bytes."""
    out = run_cli("-r32", f"0x{addr:08X}", str(size))
    data = bytearray()
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("0x") and ":" in line:
            parts = line.split(":")[1].strip().split()
            for word in parts:
                data.extend(struct.pack("<I", int(word, 16)))
    return bytes(data)


def read_encoder():
    """Read and decode the encoder struct."""
    data = read_mem(ADDR_ENCODER, 0x1C)
    if len(data) < 0x1C:
        print(f"  ERROR: only got {len(data)} bytes")
        return None

    raw_angle = struct.unpack_from("<H", data, OFF_RAW_ANGLE)[0]
    turn_count = struct.unpack_from("<i", data, OFF_TURN_COUNT)[0]
    motor_deg = struct.unpack_from("<f", data, OFF_MOTOR_DEG)[0]
    output_deg = struct.unpack_from("<f", data, OFF_OUTPUT_DEG)[0]
    initialized = data[OFF_INITIALIZED]

    return {
        "raw_angle": raw_angle,
        "turn_count": turn_count,
        "motor_deg": motor_deg,
        "output_deg": output_deg,
        "initialized": bool(initialized),
    }


def main():
    print("=== AS5600 Encoder Test ===")
    print("Rotate the motor shaft by hand to see values change.")
    print("Press Ctrl+C to stop.\n")

    enc = read_encoder()
    if not enc:
        print("ERROR: Could not read encoder data")
        return
    if not enc["initialized"]:
        print("ERROR: Encoder not initialized!")
        return

    print(f"Encoder initialized OK. Starting continuous read...\n")

    try:
        while True:
            enc = read_encoder()
            if enc:
                print(f"  raw={enc['raw_angle']:4d}/4096  "
                      f"motor={enc['motor_deg']:8.1f} deg  "
                      f"output={enc['output_deg']:8.2f} deg  "
                      f"turns={enc['turn_count']}")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nDone.")


if __name__ == "__main__":
    main()
