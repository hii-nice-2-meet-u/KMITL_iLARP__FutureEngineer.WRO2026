# Odometry System (OTOS)

Provides short-term pose estimation — position and heading — from a SparkFun
**Optical Tracking Odometry Sensor (OTOS, PAA5160E1)**. This is the motion
reference the control layer uses to hold a heading and to estimate how far the
car has traveled between LiDAR/camera updates.

**Source code:** [`code/modules/otos`](../../../../code/modules/otos)
· Device wrapper: [`otos.cpp`](../../../../code/modules/otos/otos.cpp) /
[`otos.hpp`](../../../../code/modules/otos/otos.hpp)
· Linux I²C backend: [`linux_i2c.cpp`](../../../../code/modules/otos/linux_i2c.cpp) /
[`linux_i2c.hpp`](../../../../code/modules/otos/linux_i2c.hpp)

---

## 1. Why an optical odometry sensor

The OTOS is a self-contained optical-flow + IMU module that reports fused
`x, y, heading` directly, instead of the robot having to integrate wheel
encoders and a separate gyro itself. On a steered (Ackermann-style) car, wheel
odometry alone is noisy because of tire slip and steering geometry; an optical
ground-tracking sensor sidesteps wheel slip entirely.

---

## 2. Architecture

The code reuses SparkFun's cross-platform **SparkFun Toolkit** driver
(`sfDevOTOS`) and only supplies the two platform-specific pieces the toolkit
needs on a Raspberry Pi:

- **`otos::OTOS`** (extends `sfDevOTOS`) — provides the `delayMs()` the driver
  requires (implemented with `std::this_thread::sleep_for`) and an
  `initialize(bus_num)` convenience method.
- **`LinuxI2C`** (extends `sfTkII2C`) — implements the toolkit's I²C interface
  on top of the Linux `/dev/i2c-*` character device (`open`, `ioctl(I2C_SLAVE)`,
  `read`/`write`, register read/write, `ping`).

This is the standard way to port a SparkFun Arduino-style driver to Linux: keep
the device logic from the vendor, replace only the bus and timing shims.

---

## 3. Initialization sequence

From `OTOS::initialize()`:

1. `bus_.openBus(bus_num)` — open the Linux I²C bus.
2. `bus_.setAddress(kDefaultAddress)` — select the OTOS default address.
3. `sfDevOTOS::begin(&bus_)` — hand the bus to the vendor driver; abort on
   error.
4. Set units to the values the rest of the code expects:
   - **linear unit = meters**
   - **angular unit = radians**

Using meters + radians here keeps the odometry in the same units as the LiDAR
(meters) and the camera bearing (radians), so no unit conversion is needed when
the data is fused.

---

## 4. Integration notes

- The module is compiled as part of the main build
  ([`code/modules/otos/CMakeLists.txt`](../../../../code/modules/otos/CMakeLists.txt))
  and depends on the SparkFun Toolkit and OTOS SDK vendored under
  [`code/external`](../../../../code/external).
- Because units are set to meters/radians at init, the pose it returns can be
  consumed directly by the (future) control loop for heading hold and distance
  tracking.

---

## 5. Decision log (rubric criterion 4)

Already visible in the code:

- **OTOS over raw wheel encoders + gyro** — one fused optical pose output,
  immune to wheel slip, less integration code to get wrong.
- **Reusing the SparkFun Toolkit** — only the I²C/timing shims are custom, so the
  well-tested device logic is inherited rather than reimplemented.
- **Meters + radians at init** — matches LiDAR and camera units so fusion needs
  no conversions.

The Linux wrapper uses the bus number passed to `OTOS::initialize()`; the
current default is bus 1. Physical wiring, mounting height, calibration
scalars, and measured drift are hardware-test records and are not present in
this source snapshot. The pose is consumed by navigation when a valid pose is
available; the Open Challenge loop uses it for map preview and telemetry.
