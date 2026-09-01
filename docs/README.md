# Documentation Index — KMITL iLARP · WRO Future Engineers 2026

This folder contains the detailed engineering documentation for each subsystem
of the robot. The top-level [`README.md`](../README.md) is the system overview;
the documents below go into the reasoning, parameters, and implementation of
each module.

> **Note on scope.** Every technical document here is derived directly from the
> source code in [`code/`](../../../code). Where a value or rationale is not present
> in the codebase (for example measured current draw or final mechanical
> justification), the document states that limitation instead of inventing a
> measurement.

## Signal flow (current architecture)

```
                 ┌──────────┐   ┌──────────┐   ┌────────┐
   sensors  ───▶ │ camera   │   │ lidar    │   │ otos   │
                 │ module   │   │ module   │   │ (pose) │
                 └────┬─────┘   └────┬─────┘   └───┬────┘
                      ▼              ▼             │
                 CameraProcessor  LidarProcessor   │
                      │              │             │
                      └──────┬───────┘             │
                             ▼                     │
                        ┌──────────┐               │
                        │perception│◀──────────────┤ (pose for world mapping)
                        │ (fusion) │               │
                        └────┬─────┘               │
                             ▼                     ▼
                        ┌─────────────────────────────┐
                        │ navigation_controller        │
                        │  · init_direction (CW/CCW)   │
                        │  · Stanley + PID (control/)   │
                        │  · geometric corner + FF      │
                        │  · track_map (learn/replay)   │
                        └───────────────┬──────────────┘
                                        ▼ NavigationCommand (speed, steering)
                        ┌───────────────────────────────┐
                        │ actuator (open challenge app)  │
                        │  → spi_master → STM32 (motor/servo)
                        └───────────────────────────────┘
                                        │
                        logging/ taps every stage → CSV + events
```

## Subsystem documents

| # | Document | Covers | Source code |
|---|----------|--------|-------------|
| 1 | [Vision](vision/README.md) | Camera capture, HSV segmentation, object detection, bearing | [`code/modules/camera`](../../../code/modules/camera) |
| 2 | [LiDAR](lidar/README.md) | Scan acquisition, line fitting, wall/obstacle/parking resolution | [`code/modules/lidar`](../../../code/modules/lidar) |
| 3 | [Odometry](odometry/README.md) | SparkFun OTOS optical pose over I²C | [`code/modules/otos`](../../../code/modules/otos) |
| 4 | [Perception (fusion)](perception/README.md) | Camera↔LiDAR association, obstacle colour, world mapping | [`code/modules/perception`](../../../code/modules/perception) |
| 5 | [Navigation](navigation/README.md) | State machine, corner trajectory, direction search, track map | [`code/modules/navigation`](../../../code/modules/navigation) |
| 6 | [Control](control/README.md) | PID + Stanley steering controllers | [`code/control`](../../../code/control) |
| 7 | [SPI link](spi/README.md) | RPi ↔ STM32 command protocol | [`code/modules/spi`](../../../code/modules/spi) |
| 8 | [Logging](logging/README.md) | Async CSV telemetry, events, walls, corners | [`code/modules/logging`](../../../code/modules/logging) |
| 9 | [Software Architecture](software/README.md) | Module/processor split, threading, build & deploy | [`code/`](../../../code) |
| 10 | [Mechanical](mechanical/README.md) | Chassis, steering, drivetrain, CAD inventory | [`CAD/`](../../../CAD) |
| 11 | [Electronics](electronics/README.md) | Compute, sensors, wiring, power budget | — |

## How this maps to the WRO documentation rubric (Appendix C)

The WRO Future Engineers rubric scores the repository on five criteria
(0/2/4/6 each, 30 total):

| Rubric criterion | Where to look |
|------------------|---------------|
| 1. Mobility & mechanical design | [Mechanical](mechanical/README.md) |
| 2. Power & sensor architecture | [Electronics](electronics/README.md), [Odometry](odometry/README.md) |
| 3. Software architecture & obstacle strategy | [Navigation](navigation/README.md), [Control](control/README.md), [Perception](perception/README.md), [Vision](vision/README.md), [LiDAR](lidar/README.md) |
| 4. Systems thinking & engineering decisions | "Decision log" sections in each document |
| 5. Reproducibility & GitHub quality | This index + per-module READMEs + top-level README |
