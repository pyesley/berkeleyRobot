# CAN Bus Position Controller

## Date: 2026-03-08 (session 3 — SimpleFOC angle mode + speed testing)

## Project Layout
- **SimpleFOC firmware (STM32, PlatformIO)**: `/home/pyesley/Documents/PlatformIO/Projects/SimpleFOC/`
  - Source: `src/main.cpp` — complete firmware with FDCAN + SimpleFOC angle mode
  - Build/upload: `~/.platformio/penv/bin/pio run --target upload` (from project dir)
  - Framework: SimpleFOC v2.4.0 on Arduino/PlatformIO
- **Host-side controller (C++ on Linux)**: `/tmp/berkeleyRobot/`
  - Source: `src/position_move_test.cpp` — sends position targets over CAN
  - Also: `src/can_interface.cpp`, `src/motor_protocol.cpp`, etc.
  - Build: `cd build && cmake .. && make position_move_test`
  - Run: `./position_move_test --move-deg 2520 --vel-limit 30 --cycles 1 --settle 1.5`
  - **GitHub repo**: https://github.com/pyesley/berkeleyRobot
- **CAN setup script**: `/tmp/berkeleyRobot/scripts/setup_can.sh` — run with sudo after UCAN plug-in

## Hardware
- **MCU board**: ST B-G431B-ESC1 (STM32G431CBU6, 170MHz)
- **Motor**: 14 pole pair BLDC with 3D-printed gearbox (infinite rotation, friction/cogging)
- **Sensor**: AS5600 magnetic encoder on motor shaft via I2C
- **CAN adapter**: FYSETC UCAN (gs_usb driver) on host Linux PC
- **Power supply**: 24V — limited current capacity, browns out above ~6V voltage_limit at high speed
- **CAN bus**: 1 Mbps, classic CAN, PA11=RX, PB9=TX, PC11=CAN_SHDN, PC14=CAN_TERM(120Ω)

## Control Architecture (WORKING)
SimpleFOC `MotionControlType::angle` on STM32 — full cascade controller:
- **Position P** (outer): P=10.0 → outputs velocity setpoint, clamped by velocity_limit
- **Velocity PID** (inner): P=0.15, I=1.5, output_ramp=75, limit=6.0V
- **Voltage torque** (lowest): voltage_limit=6.0V on 24V supply
- Host just sends position targets + velocity_limit over CAN

## CAN Protocol
- NMT (0x001): [mode, device_id] — MODE_POSITION=0x13, MODE_VELOCITY=0x12, MODE_DISABLED=0x00
- PDO2_TX (0x301): Host→motor [position_f32, velocity_limit_f32] (8 bytes)
- PDO2_RX (0x281): Motor→host [position_f32, velocity_f32] (8 bytes, 50Hz)
- Heartbeat (0x701): keepalive (0 bytes), 500ms timeout

## Current Firmware Settings (main.cpp)
```
voltage_limit = 6.0V          # Safe for current power supply
voltage_sensor_align = 0.5V   # MUST stay low — higher causes brownout on boot
PID_velocity: P=0.15, I=1.5, D=0, ramp=75, limit=6.0
P_angle: P=10.0
velocity_limit = 40.0         # Ceiling — actual limit sent by host via --vel-limit
LPF_velocity.Tf = 0.05
motion_downsample = 5
```

## Performance Results (2520° = 7 full turns, each way)
| vel-limit | Move time | Peak speed | Settling error |
|-----------|-----------|------------|----------------|
| 10 rad/s  | 4.4s      | 10 rad/s   | <2°            |
| 13 rad/s  | 3.4s      | 13 rad/s   | <0.5°          |
| 16 rad/s  | 2.9s      | 16 rad/s   | <0.5°          |
| 20 rad/s  | 2.1s      | 21 rad/s   | <0.5°          |
| 25 rad/s  | 1.8s      | 27 rad/s   | <0.5°          |
| 30 rad/s  | 1.5s      | 32 rad/s   | <1°            |

## Key Findings
- **AS5600 works up to 30+ rad/s** — original assumption of ~12 rad/s limit was wrong
- **Speed bottleneck is power supply current**, not sensor
- **voltage_sensor_align MUST be ≤0.5V** — higher draws too much current during initFOC() calibration, causing brownout reset loop
- **velocity_limit is settable via CAN** — no need to reflash to change speed, just use --vel-limit flag
- **After firmware upload, sometimes need reset button or power cycle** for initFOC() to calibrate correctly (check for "Start position: 0 deg" = failed calibration)
- **If CAN bus goes BUS-OFF**: unplug/replug UCAN USB adapter, then re-run setup_can.sh

## Critical FDCAN Fixes (from earlier sessions)
- Clock source: `MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, RCC_CCIPR_FDCANSEL_1)` — PCLK1 not HSE
- DLC: `txHeader.DataLength = len` — NOT shifted by 16
- AutoRetransmission: DISABLE — prevents TX FIFO blocking when no bus partner
- PA11 re-init after USB: must re-configure as AF9 FDCAN1_RX after all other init

## Failed Experiment: FusedSensor
Attempted to create a custom Sensor class wrapping AS5600 with velocity-based prediction for higher speeds. Failed because:
- Overriding getAngle()/getVelocity()/getMechanicalAngle() fights SimpleFOC's internal state management
- Prediction errors create positive feedback loops (bad prediction → reject sensor → worse prediction)
- initFOC() calibration (sensor_direction, zero_electric_angle) depends on sensor behavior
- **Conclusion**: don't override SimpleFOC's Sensor class. If fusion needed, do it externally for position control only, keeping raw AS5600 for FOC commutation.

## Next Steps
- Upgrade power supply for higher voltage_limit (>6V) to achieve faster speeds
- Consider sensor fusion approach: keep raw AS5600 for FOC, use separate fused estimate for host-side position commands only
- Consider upgrading to a faster sensor (SPI encoder) to eliminate AS5600 I2C bottleneck entirely
