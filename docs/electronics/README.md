# Electronics & Power

Compute, sensing, and actuation hardware, how it is wired, and the power budget.

---

## 1. Component overview

| # | Component | Role | Interface | CAD / ref |
|---|-----------|------|-----------|-----------|
| 1 | Raspberry Pi 5 (arm64) | Main compute (perception, navigation) | — | deploy target in `build_n_deploy.sh` |
| 2 | STM32 (MCU) | Real-time actuation: motor + servo + battery | SPI slave | Exact part number is not recorded in the source tree |
| 3 | Raspberry Pi Camera Module 3 | Vision sensing | CSI (libcamera) | `CAD/RaspburryPi_Camera_Module3.SLDPRT` |
| 4 | RPLIDAR S3 | Distance & geometry sensing | UART `/dev/ttyAMA0` @ 1 Mbaud | `CAD/RPLIDAR_S3.SLDASM` |
| 5 | SparkFun OTOS (PAA5160E1) | Optical odometry (pose) | I²C | `CAD/SparkFunOpticalTracking...SLDPRT` |
| 6 | CN3903 | Power / battery management | — | `CAD/CN3903.SLDASM` |
| 7 | MG90S servo | Steering actuator | PWM (driven by STM32) | `CAD/MG90/MG90S_Servo.STEP` |
| 8 | N20 gear-motor | Drive / propulsion | motor driver (driven by STM32) | `CAD/Motor_N20.SLDPRT` |

The compute is **split**: the Pi 5 runs perception + navigation; a separate
**STM32** executes the low-level motor/servo commands and reports battery
voltage, connected to the Pi over **SPI** (15 MHz, mode 0). Camera, LiDAR, and
OTOS have driver modules on the Pi; the servo and motor are commanded through the
SPI link — see the [SPI doc](../spi/README.md).

---

## 2. Software ↔ hardware map

| Subsystem | Hardware | Bus / port | Code |
|-----------|----------|-----------|------|
| Vision | Camera Module 3 | CSI via libcamera/LCCV | [`modules/camera`](../../../../code/modules/camera) |
| LiDAR | RPLIDAR S3 | UART `/dev/ttyAMA0`, 1,000,000 baud | [`modules/lidar`](../../../../code/modules/lidar) |
| Odometry | OTOS | I²C (`/dev/i2c-*`) | [`modules/otos`](../../../../code/modules/otos) |
| Actuation link | STM32 | SPI master (15 MHz, mode 0) | [`modules/spi`](../../../../code/modules/spi) |
| Steering | MG90S servo | PWM from STM32 (pulse 1000–2100 µs) | via SPI `SERVO_PULSE` |
| Drive | N20 motor | motor driver from STM32 | via SPI `M1_POW` |

---

## 3. Wiring diagram

The repository does not contain a wiring drawing. The electrical topology
implemented by the software is: battery and regulation feed the Pi, camera,
LiDAR, and OTOS; the Pi communicates with the STM32 over SPI; and the STM32
drives the motor and steering servo. Pin-level wiring and voltage rails must be
checked against the assembled robot before publication.
>
The final wiring record should show, at minimum: battery → CN3903 / regulation
→ Pi 5, and how the camera (CSI), LiDAR (USB), OTOS (I²C), servo (PWM), and
motor driver are connected, including voltage rails.

---

## 4. Power budget

The source tree does not include measured current data or a battery
specification, so a numerical power budget would be misleading. Record the
values from the assembled hardware before using this table for electrical
sign-off:

| Load | Voltage | Typical current | Peak current | Notes |
|------|---------|-----------------|--------------|-------|
| Raspberry Pi 5 | 5 V | not recorded | not recorded | Higher under CV load |
| RPLIDAR S3 | not recorded | not recorded | not recorded | Motor + sensor |
| Camera Module 3 | 3.3 V | not recorded | not recorded | CSI camera power is supplied by the Pi |
| OTOS | 3.3 V | not recorded | not recorded | I²C sensor |
| MG90S servo | not recorded | not recorded | not recorded | Stall current matters |
| N20 motor | not recorded | not recorded | not recorded | Via driver |
| **Total** | | **not calculated** | **not calculated** | |

Also state: **battery** chemistry / voltage / capacity (mAh), how it is
regulated (the CN3903 / any buck converters), and the resulting **estimated run
time**.

---

## 5. Calibration & noise considerations

The vision intrinsics currently live in `camera_processor.hpp`. The repository
does not include the calibration record. OTOS mounting offsets, electrical-noise
mitigation, grounding, shielding, and lighting tests are also not recorded and
must be documented from the assembled robot and its test results.

---

## 6. Decision log (rubric criterion 2 & 4)

Already visible in the code:

- **Raspberry Pi 5 as sole compute** — enough headroom to run OpenCV vision and
  LiDAR geometry on-board; cross-compiled to keep the Pi free of build load.
- **RPLIDAR at 1 Mbaud over USB serial** — the S3's rate for dense scans.
- **OTOS on I²C in meters/radians** — matches LiDAR/camera units for fusion.

The split architecture keeps perception and navigation on the Pi while the
STM32 handles time-sensitive actuation. Camera, LiDAR, and OTOS provide
complementary semantic, geometric, and motion information. Current, weight,
and processing-time measurements are not stored in the source tree.
