# LiDAR System

Turns raw 2-D laser scans into a compact geometric description of the track:
the left, right, and front walls, plus any obstacle markers standing inside the
lane.

**Source code:** [`code/modules/lidar`](../../../../code/modules/lidar)
· Acquisition: [`lidar_module.cpp`](../../../../code/modules/lidar/lidar_module.cpp)
· Processing: [`lidar_processor.cpp`](../../../../code/modules/lidar/lidar_processor.cpp)
· Shared types: [`lidar_struct.hpp`](../../../../code/modules/lidar/lidar_struct.hpp)

Sensor: **RPLIDAR S3** (SLAMTEC), driven through the official `rplidar_sdk`.

### Physical mount convention

The assembled robot mounts the RPLIDAR with its raw-angle-zero arrow pointing
to the rear. RPLIDAR's raw 0° is the direction of that arrow, so the robot's
front is raw 180°. `polar2cartesian()` intentionally applies the 180° mount
compensation (`x = -d·sin(angle)`, `y = -d·cos(angle)`), leaving **+Y as the
robot-forward axis**. This was confirmed by M-1 on 2026-09-02; do not remove
the minus signs.

---

## 1. Responsibilities

Like the vision subsystem, LiDAR is split into acquisition and processing:

- **`LidarModule`** — owns the serial connection and the scan thread. It
  converts the SDK's fixed-point measurement nodes into plain
  `LidarPoint{angle_deg, distance_m, quality}` values and publishes timestamped
  scans.
- **`LidarProcessor`** — a stateless pipeline that converts one scan into
  `ProcessedLidarData`: fitted line segments, resolved walls, and obstacle
  segments.

---

## 2. Acquisition (`LidarModule`)

| Parameter | Default | Notes |
|-----------|---------|-------|
| Serial port | `/dev/ttyUSB0` (module default) | The Open Challenge apps use `/dev/ttyAMA0` (Pi GPIO UART) |
| Baud rate | 1,000,000 | RPLIDAR S3 |
| Buffer | `RingBuffer<TimedLidarData, 10>` | Newest-wins |

Flow (`lidar_module.cpp`):

1. `initialize()` creates a serial channel and the SLAMTEC driver, then
   `connect()`s. Failures are reported and cleaned up.
2. `start()` sets the motor speed and calls `startScan()`, then spawns
   `scan_loop()` on its own thread.
3. `scan_loop()` repeatedly `grabScanDataHq()` → `ascendScanData()` (sort by
   angle) → `processScan()`.
4. **Health watchdog.** If `grabScan` fails 3 times in a row, `checkHealth()` is
   queried and the device health / error code is logged. This surfaces motor or
   comms problems instead of silently stalling.

### Fixed-point conversion

The SDK returns fixed-point fields, converted in `processScan()`:

```
angle_deg   = angle_z_q14 * 90 / 2^14
distance_m  = dist_mm_q2 / 1000 / 2^2
quality     = node.quality
```

Consumers use `get_latest()` (non-blocking) or `wait_for_data()` (blocks until a
new scan), the same pattern as the camera module.

---

## 3. Processing pipeline (`LidarProcessor`)

```
TimedLidarData (polar points)
   │
   ▼  filter invalid points (quality, min distance)
   ▼  polar → cartesian
Cartesian points
   │
   ▼  split-and-merge → point groups
   ▼  PCA line fit per group → LineSegment
   ▼  merge collinear aligned segments
Line segments
   │
   ├─▶ resolve_track_walls → left / right / front walls
   ├─▶ find_parking_wall  → optional parking-lot wall
   └─▶ detect_obstacles   → obstacle markers (with bearing_rad())
ProcessedLidarData
```

`ProcessedLidarData` now carries `line_segments`, `walls` (`ResolvedWalls`), an
optional `parking_wall`, and `obstacles`. Each `ObstacleObject` exposes
`bearing_rad() = atan2(center.x, center.y)`, which the perception layer uses to
associate it with a camera detection.

### 3.0 Motion deskew (P-22 enabled)

Each scan is deskewed into its scan-end frame using the measured OTOS forward
velocity, yaw rate, and LiDAR revolution period. M-1 confirms that forward is
robot +Y, matching the sign in `LidarProcessor::deskew()`. The first scan, which
has no measured period yet, intentionally remains un-deskewed.

The old controller-injected `wall_correction_rad` path has been removed from
LiDAR processing. Geometry is no longer rotated using the navigation target
heading; the telemetry field is retained as a deprecated zero column for CSV
schema compatibility.

### 3.1 Point filtering & coordinate frame

`process()` now takes the tuning constants and optional `ScanMotion`; it no
longer accepts `heading_error_rad`. Full signature:

```cpp
process(data, min_segment_point = 5, max_line_error_m = 0.035,
        max_point_gap_m = 0.10, max_angle_diff = 3.0,
        max_collinear_error_m = 0.03, max_segment_gap_m = 0.05,
        motion = ScanMotion{})
```

> **Logged geometry is now in the deskewed robot frame.** The deprecated
> `wall_correction_rad` telemetry column is always zero after P-22; it remains
> only to preserve the existing CSV column layout.

`is_valid_point()` drops points with `quality < 10`, non-finite range/angle,
`distance < 0.015 m`, or `distance > 3.0 m`. The decision lives in
`classify_point()`, which also tags each rejection as quality- or range-based so
the per-scan `lidar_points_rejected_quality`/`_range` telemetry counts a mis-set
threshold without persisting the raw scan.

`polar2cartesian()` uses the robot frame **+X = right, +Y = forward**:

```
x = distance · sin(angle)
y = distance · cos(angle)
```

### 3.2 Split-and-merge line fitting

Implemented recursively in `split_line_segments_recursive()`. For a run of
points it:

1. **Gap split** — if the distance between two consecutive points exceeds
   `MAX_POINT_GAP_M`, split there (the wall is not continuous).
2. **Deviation split** — otherwise find the point farthest from the
   straight line joining the run's endpoints; if that deviation exceeds
   `MAX_LINE_ERROR_M`, split at that point (Douglas–Peucker style).
3. Otherwise the run is accepted as one segment.

Runs shorter than `MIN_SEGMENT_POINTS` are discarded.

| Constant | Value |
|----------|-------|
| `MIN_SEGMENT_POINTS` | 5 |
| `MAX_LINE_ERROR_M` | 0.035 m |
| `MAX_POINT_GAP_M` | 0.11 m |

### 3.3 Line fitting (`fit_line_segment`)

Each accepted group is fitted with **total least squares (PCA)**, not ordinary
least squares, so vertical walls are handled correctly:

```
θ = 0.5 · atan2( 2·Sxy , Sxx − Syy )
```

The segment stores its direction/normal, the line constant `c`, its two
projected endpoints, and its **RMS error**. Segments whose RMS error exceeds
`MAX_LINE_ERROR_M` are rejected after fitting as well.

### 3.4 Merging aligned segments (`merge_aligned_segments`)

A real wall often breaks into several segments. Two segments are merged when
**all** of these hold:

| Test | Threshold |
|------|-----------|
| Angle difference | ≤ 5° |
| Collinearity (perp. distance of one center to the other's line) | ≤ 0.02 m |
| End-to-end gap | ≤ 0.05 m |

The merged segment spans the extreme projections of all four endpoints along the
common direction.

### 3.5 Resolving walls (`resolve_track_walls`)

Segments longer than `MIN_WALL_LENGTH_M = 0.25 m` are classified by orientation
(within `±15°`):

- **Vertical-ish** segments → **left** wall if their center x < 0, else
  **right** wall.
- **Horizontal-ish** segments with center y > 0 → **front** wall.

For each side the wall with the **smallest perpendicular distance** (i.e. the
closest) wins. The result is a `ResolvedWalls{left, right, front}`, each
optional.

`resolve_inner_outer()` maps left/right to **inner/outer** depending on the
`DrivingDirection` (CLOCKWISE vs COUNTER_CLOCKWISE), so the same logic works
regardless of which way the round is run.

### 3.6 Obstacle detection (`detect_obstacle_segments`)

A segment is treated as an obstacle marker only if it is **not** part of a wall
and is the right size:

- rejected if it *is* one of the resolved walls (`is_same_segment`) or is a
  fragment lying on a wall (`is_wall_fragment`);
- kept only if `0.028 m ≤ length ≤ 0.08 m` and its center is in front
  (`y > 0`);
- nearby detections (within 0.08 m) are merged so one marker yields one
  obstacle.

| Constant | Value |
|----------|-------|
| `MIN_OBSTACLE_SEGMENT_LENGTH_M` | 0.028 m |
| `MAX_OBSTACLE_SEGMENT_LENGTH_M` | 0.08 m |
| `SAME_OBSTACLE_DISTANCE` | 0.08 m |
| Wall-fragment max distance | 0.04 m |

---

## 4. Debug tools

- [`code/app/test_lidar`](../../../../code/app/test_lidar) renders a top-down debug map
  (1 m = 300 px): raw segments in gray, resolved walls in green, obstacles in
  red with distance/bearing labels, plus per-frame processing time.
- [`code/app/test_perception`](../../../../code/app/test_perception) exercises the
  full LiDAR + camera **fusion** path — see the
  [Perception](../perception/README.md) doc.

These are the main tools for verifying the pipeline against a real track.

---

## 5. Decision log (rubric criterion 4)

Already visible in the code:

- **Split-and-merge + PCA** instead of a Hough transform — gives explicit,
  bounded line segments with endpoints and an RMS error to threshold on, which
  is exactly what wall/obstacle resolution needs.
- **Total least squares** rather than y-on-x least squares — the latter blows up
  for near-vertical walls (infinite slope); TLS does not.
- **Closest-segment-wins** for wall selection — the nearest parallel surface is
  the actual track wall; farther parallel returns are noise or the opposite
  wall.
- **Inner/outer abstraction** — decouples the geometry from the run direction so
  navigation logic is written once.

The thresholds are defined in `LidarProcessor::process()` and describe the
current geometric filters. The repository contains no tuning log, measured
processing rate, real-track scan count, or failure-case report, so those values
are not presented as measured performance here.
