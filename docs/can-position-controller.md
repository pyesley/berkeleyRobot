# CAN Bus Position Controller

Motor control system using SimpleFOC firmware on an STM32 microcontroller, communicating via CAN bus with a host Linux controller. Implements precision position control for a BLDC motor with a magnetic encoder through a 3D-printed gearbox.

## Hardware

| Component | Details |
|-----------|---------|
| MCU board | ST B-G431B-ESC1 (STM32G431CBU6, 170 MHz) |
| Motor | 14 pole-pair BLDC with 3D-printed gearbox |
| Sensor | AS5600 magnetic encoder on motor shaft, before gearbox (I2C) |
| CAN adapter | FYSETC UCAN (gs_usb driver) on host Linux PC |
| Power supply | 24V — limited current capacity |
| CAN bus | 1 Mbps classic CAN |
| CAN pins | PA11=RX, PB9=TX, PC11=CAN_SHDN, PC14=CAN_TERM (120 ohm) |

## File Locations

### Host-side controller (this repo)
- **Source**: `src/` — `can_interface.cpp/hpp`, `motor_protocol.cpp/hpp`, `main.cpp`, `speed_test.cpp`, `open_loop_test.cpp`, `position_test.cpp`
- **Scripts**: `scripts/setup_can.sh` (CAN interface init), `scripts/position_test.py` (Python reference), `scripts/calibrate_motor.py`
- **GUI**: `gui/position_gui.py` — Tkinter GUI with matplotlib real-time plotting
- **Build**: `mkdir -p build && cd build && cmake .. && make`

### SimpleFOC firmware (STM32, PlatformIO)
- **Location**: `/home/pyesley/Documents/PlatformIO/Projects/SimpleFOC/`
- **Source**: `src/main.cpp` — complete firmware with FDCAN + SimpleFOC angle mode + fault detection
- **Build/upload**: `~/.platformio/penv/bin/pio run --target upload` (from project dir)
- **Framework**: SimpleFOC v2.4.0 on Arduino/PlatformIO

## CAN Protocol

| Message | CAN ID | Direction | Payload | Rate |
|---------|--------|-----------|---------|------|
| NMT | 0x001 | Host -> Motor | `[mode, device_id]` | On command |
| PDO2_TX | 0x301 | Host -> Motor | `[position_f32, velocity_limit_f32]` | ~50 Hz |
| PDO2_RX | 0x281 | Motor -> Host | `[position_f32, velocity_f32]` | 50 Hz |
| Heartbeat | 0x701 | Bidirectional | 0 bytes | < 500 ms |

### Motor Modes (NMT byte 0)
- `0x00` DISABLED — motor off
- `0x01` IDLE — motor off
- `0x12` VELOCITY — velocity control, PDO2_TX byte 0-3 = velocity target
- `0x13` POSITION — angle control, PDO2_TX = [position_rad, velocity_limit_rad_s]

## Control Architecture

SimpleFOC `MotionControlType::angle` on STM32 — three-tier cascade:

1. **Position P** (outer): P=10.0 -> outputs velocity setpoint, clamped by velocity_limit
2. **Velocity PID** (inner): P=0.15, I=1.5, D=0, output_ramp=75, limit=9.0V
3. **Voltage torque** (lowest): voltage_limit=9.0V on 24V supply

Host sends position targets + velocity_limit over CAN. All closed-loop control runs on the STM32. All position/velocity values are in motor-shaft coordinates (before the 14:1 gearbox). Divide by 14 to get output-side values.

### Firmware Parameters (as of 2026-03-29)
```
voltage_limit = 9.0V              # Was 6.0V, increased for faster response
voltage_sensor_align = 0.5V       # MUST stay low - higher causes brownout during calibration
PID_velocity: P=0.15, I=1.5, D=0, ramp=75, limit=9.0
P_angle: P=10.0
velocity_limit = 40.0             # Ceiling - actual limit sent by host via CAN
LPF_velocity.Tf = 0.05
motion_downsample = 5
```

## Fault Detection (firmware)

Added to firmware to handle motor faults gracefully without requiring power cycle.

### Fault Types
| Fault | Threshold | Description |
|-------|-----------|-------------|
| VELOCITY_INSANE | > 100 rad/s | Sensor giving garbage readings |
| STALL | error > 0.5 rad, vel < 0.3 rad/s for 2s | Motor stuck but trying to move |
| CALIBRATION | shaft_angle near 0 after init | initFOC() calibration failed |

### Fault Behavior
- Motor is disabled, targets zeroed, watchdog deactivated
- `loopFOC()` and `move()` are skipped while faulted
- Serial commander and CAN RX still process (allows remote recovery)
- Serial status line shows `FAULT:<code>` when active

### Recovery Commands (serial)
- `R` — full software re-init (re-runs sensor init + `initFOC()`)
- `C` — clear fault, leaves motor disabled for manual re-enable
- `E` — enable motor (blocked while fault is active)

### Known Gap
Fault detection only runs when motor is enabled (`checkFaults()` returns early if `!motor.enabled`). A frozen/insane sensor reading while disabled won't trigger a fault — this can mask problems until the motor is re-enabled.

## Performance Results

Tested at 360 deg (1 turn) each direction, 10 rad/s (2026-03-29):

| Direction | Settle time | Peak speed | Final error |
|-----------|-------------|------------|-------------|
| Forward | 1.12s | 10.2 rad/s | -0.2 deg |
| Reverse | 1.66s | 10.4 rad/s | -1.6 deg |

Tested at 2520 deg (7 turns) each direction (2026-03-08):

| Vel limit | Move time | Peak speed | Settling error |
|-----------|-----------|------------|----------------|
| 10 rad/s | 4.4s | 10 rad/s | < 2 deg |
| 13 rad/s | 3.4s | 13 rad/s | < 0.5 deg |
| 16 rad/s | 2.9s | 16 rad/s | < 0.5 deg |
| 20 rad/s | 2.1s | 21 rad/s | < 0.5 deg |
| 25 rad/s | 1.8s | 27 rad/s | < 0.5 deg |
| 30 rad/s | 1.5s | 32 rad/s | < 1 deg |

## Critical Technical Notes

### AS5600 Sensor
- Works reliably up to 30+ rad/s — original assumption of 12 rad/s limit was wrong
- Speed bottleneck is power supply current, not sensor bandwidth
- If velocity reads as a large constant (e.g., 4754 rad/s) after crash, the sensor state is corrupted — power cycle required

### Calibration
- `voltage_sensor_align` MUST be <= 0.5V — higher draws too much current during `initFOC()`, causing brownout reset loop
- After firmware upload, sometimes need reset button or power cycle for `initFOC()` to calibrate correctly
- Check for "Start position: 0 deg" or "Ang: 0.0d" = failed calibration

### FDCAN Configuration (critical fixes from development)
- Clock source: `MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, RCC_CCIPR_FDCANSEL_1)` — must use PCLK1, not HSE
- DLC: `txHeader.DataLength = len` — NOT shifted by 16
- AutoRetransmission: DISABLE — prevents TX FIFO blocking when no bus partner
- PA11 must be re-configured as AF9 (FDCAN1_RX) after all other init — USB init reclaims it

### CAN Bus Recovery
- If CAN bus goes BUS-OFF: unplug/replug UCAN USB adapter, then re-run `setup_can.sh`
- "No buffer space available" errors indicate TX queue overflow — transient, not fatal

### Failed Experiment: FusedSensor
Attempted custom Sensor class wrapping AS5600 with velocity-based prediction. Failed because overriding `getAngle()`/`getVelocity()` fights SimpleFOC's internal state management, creating destabilizing feedback loops during FOC calibration. If fusion is needed, do it externally — keep raw AS5600 for FOC commutation.

## Quick Reference

```bash
# Setup CAN interface
sudo ./scripts/setup_can.sh

# Build host software
mkdir -p build && cd build && cmake .. && make

# Run speed test (360 deg, 10 rad/s, 1 cycle)
./build/speed_test 360 10 1

# Run speed test (7 turns, 30 rad/s)
./build/speed_test 2520 30 1

# Monitor serial output from STM32
screen /dev/ttyACM0 115200

# Flash firmware
cd ~/Documents/PlatformIO/Projects/SimpleFOC
~/.platformio/penv/bin/pio run --target upload

# Monitor CAN bus
candump can0
```
