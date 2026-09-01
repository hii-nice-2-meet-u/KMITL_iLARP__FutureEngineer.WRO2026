# SPI Link (RPi ↔ STM32)

The Raspberry Pi 5 is the high-level brain; a separate **STM32** microcontroller
drives the motors and steering servo and reads the battery. They communicate
over **SPI**, with the Pi as master. This document describes the master side.

**Source code:** [`code/modules/spi`](../../../../code/modules/spi)
· [`spi_master.cpp`](../../../../code/modules/spi/spi_master.cpp) / [`spi_master.hpp`](../../../../code/modules/spi/spi_master.hpp)

---

## 1. Why a split controller

Real-time actuation (PWM for the servo, motor duty/encoder loops) is better done
on a dedicated MCU with deterministic timing than on Linux, which is not
real-time. The Pi runs perception and navigation; the STM32 executes the
low-level motor/servo commands the Pi sends. SPI gives a fast, simple
master/slave link between them.

---

## 2. Bus configuration

`SPI::initialize()` opens a Linux `spidev` device with:

| Parameter | Default |
|-----------|---------|
| Chip select | 0 |
| Mode | `SPI_MODE_0` |
| Bits per word | 8 |
| Speed | 15 MHz |

---

## 3. Frame format

Every transfer is a fixed **3-byte frame**: one `Command` byte + a 16-bit data
field. Signed values (e.g. motor power) are encoded as 16-bit two's complement
in the data field; responses are decoded with `decode_data()`.

---

## 4. Command set (`spi::Command`)

| Group | Commands |
|-------|----------|
| Test / debug | `NULL_ (0x00)`, `ECTO_TEST (0x01)`, `DEBUG (0xFF)` |
| Motor control | `M_ENABLE/M_DISABLE`, `M1_POW/M2_POW`, `M1_DUTY/M2_DUTY` |
| Motor encoder | `M1_SPD/M2_SPD`, `M_Brake`, `M_ENC_ENABLE/DISABLE/INVERTED` |
| Servo | `SERVO_PULSE (0x40)`, `SERVO_ANGLE (0x41)` |
| Power | `VOL_CHECK (0xA0)` |

Public API (from `spi_master.hpp`):

- `echo_test(value, response)` — link sanity check.
- `enable_motors()` / `disable_motors()`.
- `set_motor_power(Motor, percent)` — signed −100…+100.
- `set_motor_speed(Motor, percent)` — 0…100.
- `set_servo_pulse_us(pulse_us)` — 1000…2100 µs.
- `set_servo_angle(angle_deg)` — 0…180°, centre 90°.
- `read_voltage_v()` — battery voltage (controller returns unsigned mV).

---

## 5. How navigation reaches the motors

The Open Challenge actuator layer
([`open_challenge_actuator.hpp`](../../../../code/app/_challenge/open/open_challenge_actuator.hpp))
converts a `NavigationCommand` into SPI calls:

- `target_speed_mps` → percent of `full_scale_speed_mps` (0.85) → `set_motor_power(M1, …)`.
- `steering_rad` → servo pulse in `[1000, 2100] µs` (centre ≈ 1550) →
  `set_servo_pulse_us(…)`.

It also arms/disarms and performs an **emergency stop** (motor power 0) on
destruction or any SPI failure — a safety-first default.

---

## 6. Decision log (rubric criterion 2 & 4)

Visible in the code:

- **Two-controller split** — deterministic actuation on the STM32, heavy compute
  on the Pi; SPI is the fast link between them.
- **Fixed 3-byte framing** — trivial to implement identically on both sides and
  cheap to parse on the MCU.
- **Fail-safe actuator** — any SPI error or object destruction forces motor
  power to zero, so a comms glitch cannot leave the car driving.

This repository documents the Raspberry Pi master only. STM32 timer/PWM
channels, encoder wiring, and the physical MOSI/MISO/SCLK/CS pinout are not
recorded in the source tree and must be taken from the STM32 firmware and the
assembled wiring before publication.
