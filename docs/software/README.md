# Software Architecture

How the code is organized, how the pieces run concurrently, and how it is built
and deployed to the robot.

**Source tree:** [`code/`](../../../../code)

---

## 1. Layered structure

The system is written in **C++17** and organised into hardware modules,
stateless processors, decision/control layers, and applications:

```
code/
├── common/         direction.hpp (DrivingDirection, TurnDirection)
│
├── modules/        Hardware ownership + I/O (stateful, threaded) and processing
│   ├── camera/     CameraModule → frames ; CameraProcessor → objects
│   ├── lidar/      LidarModule  → scans  ; LidarProcessor  → walls/obstacles
│   ├── otos/       OTOS + LinuxI2C → pose
│   ├── perception/ Perception → fuse camera + LiDAR (+ pose) → FusedObstacle
│   ├── navigation/ NavigationController (state machine), init_direction, track_map
│   ├── spi/        SPI master → STM32 (motor/servo/battery)
│   └── logging/    AsyncCsvWriter, telemetry/wall loggers (Logger V1)
│
├── control/        PID, StanleyController (used by navigation)
│
├── app/
│   ├── _challenge/open/   open_challenge_main (learn+replay) + actuator
│   ├── test_camera, test_lidar, test_perception, test_otos, test_spi
│   └── adjust_HSV, adjust_white, camera_tuner
│
├── external/       Vendored SDKs (LCCV, rplidar_sdk, OTOS, SparkFunToolkit) — submodules
├── utils/          RingBuffer.hpp
├── CMakeLists.txt
├── Dockerfile.compile / Dockerfile.cross
└── build_n_deploy.sh
```

The guiding rule: **modules touch hardware and hold state; processors are pure
functions.** A processor (camera, lidar, perception) takes timestamped inputs
and returns a structured result with no side effects, which makes it testable on
recorded data. The navigation controller holds the run state machine; the
control layer holds reusable PID/Stanley primitives.

For the full end-to-end signal flow diagram, see the
[documentation index](../README.md).

---

## 2. Concurrency model

Each sensor module runs its own **capture/scan thread** so that acquisition is
never blocked by processing:

- The thread pushes each new sample into a fixed-size
  [`RingBuffer<T, N>`](../../../../code/utils/RingBuffer.hpp) (camera N=30, lidar
  N=10) under a mutex.
- A `condition_variable` plus a monotonically increasing **sequence counter**
  lets a consumer:
  - `get_latest()` — take the newest sample without blocking, or
  - `wait_for_frame()` / `wait_for_data()` — block until a genuinely new sample
    arrives (the sequence counter guarantees the same sample is not processed
    twice).

`RingBuffer` itself is intentionally **not** internally synchronized — the owning
module holds the mutex. This keeps the buffer a simple, allocation-free
`std::array` and puts locking in one place.

Every sample carries a `steady_clock` **timestamp in microseconds**, captured at
acquisition time, so camera and LiDAR results can be aligned in time by the
fusion/navigation layer.

---

## 3. Data types (contracts between layers)

| Type | Produced by | Consumed by |
|------|-------------|-------------|
| `TimedFrameData` | CameraModule | CameraProcessor |
| `ProcessedCameraData` (color, bbox, bearing) | CameraProcessor | Perception |
| `TimedLidarData` (polar points) | LidarModule | LidarProcessor |
| `ProcessedLidarData` (segments, walls, parking wall, obstacles) | LidarProcessor | Perception, Navigation |
| OTOS pose (x, y, heading), speed | OTOS | Navigation |
| `PerceptionData` (walls, `FusedObstacle[]`) | Perception | Navigation (obstacle challenge) |
| `NavigationCommand` (speed, steering, accel) | NavigationController | Actuator |
| `ReplayHint` | TrackMap | NavigationController |
| SPI `Command` frames | Actuator / SPI master | STM32 |
| `TelemetryRow` / `WallRow` / `CornerRow` | app loop | logging (CSV) |

---

## 4. Build system

- **CMake ≥ 3.10**, `CMAKE_CXX_STANDARD 17`.
- The top-level [`code/CMakeLists.txt`](../../../../code/CMakeLists.txt) finds OpenCV
  and libcamera, then adds the module and app subdirectories.
- Dependencies: **OpenCV** (vision + geometry math), **libcamera/LCCV**
  (camera), **rplidar_sdk** (LiDAR), **SparkFun Toolkit + OTOS SDK** (odometry).

### Cross-compilation with Docker

The robot runs on a **Raspberry Pi 5 (arm64)**, but building on the Pi is slow.
The project cross-compiles inside a Docker image instead:

- [`Dockerfile.cross`](../../../../code/Dockerfile.cross) — the arm64 cross toolchain.
- [`Dockerfile.compile`](../../../../code/Dockerfile.compile) — native compile image.

---

## 5. Deployment (`build_n_deploy.sh`)

[`build_n_deploy.sh`](../../../../code/build_n_deploy.sh) automates the
build-and-ship loop:

| Command | Action |
|---------|--------|
| `./build_n_deploy.sh` | Incremental cross-build (Ninja + ccache) then deploy |
| `./build_n_deploy.sh clean` | Remove build/install artifacts |
| `./build_n_deploy.sh clean-all` | Also clear the ccache |
| `./build_n_deploy.sh cross` | Rebuild the cross-compiler Docker image |

Pipeline:

1. Ensure the `cross-pi` Docker image exists (build it if missing).
2. Run CMake + Ninja **inside the container**, with `ccache` for fast
   incremental rebuilds, installing to a local `dist/install`.
3. `rsync` the install tree to the Pi over SSH
   (`PI_USER@PI_IP:PI_TARGET_DIR`), deleting stale files.

> The Pi's user/IP/target directory are set at the top of the script and must be
> updated for your own robot before first deploy.

---

## 6. Current status & roadmap

**Implemented today (full Open Challenge loop):**

- sensor acquisition — camera, LiDAR, OTOS;
- processing — camera object detection, LiDAR wall/obstacle/parking resolution;
- **[perception](../perception/README.md)** — camera↔LiDAR fusion with colour;
- **[navigation](../navigation/README.md)** — four-state machine, CW/CCW
  direction search, geometric corner trajectory with Ackermann feed-forward,
  Stanley wall following, command conditioning;
- **[control](../control/README.md)** — reusable PID + Stanley;
- **[track map](../navigation/README.md#7-track-map--learn-on-lap-1-replay-on-laps-23)** —
  learn on lap 1, replay on laps 2–3;
- **[SPI](../spi/README.md)** actuation to the STM32, with a fail-safe actuator;
- **[logging](../logging/README.md)** — async CSV telemetry (Logger V1);
- one Open Challenge run program (`open_challenge_main`, learn+replay) and a
  full set of test/calibration tools;
- the full cross-compile + deploy pipeline.

**Still open (main gaps for the Obstacle Challenge):**

- **obstacle avoidance is not yet wired into `navigation_controller::update()`**
  — perception already outputs coloured, positioned obstacles, but the
  controller does not yet act on them;
- **parallel parking** manoeuvre;
- traffic-light landmarks exist in `TrackMap` but are not used by the loops yet.

The current Open Challenge executable does not yet consume fused coloured
obstacles for avoidance and does not implement parallel parking. Those are
separate Obstacle Challenge work items, not behaviours provided by the current
executable.
