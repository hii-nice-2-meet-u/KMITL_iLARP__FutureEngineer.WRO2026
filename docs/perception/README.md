# Perception (Sensor Fusion)

Associates camera detections with LiDAR obstacle clusters so each obstacle
carries both a **geometric position** (from LiDAR) and a **colour / required
pass side** (from the camera). It also forwards the resolved track walls and the
parking wall, and maps obstacles into world coordinates when a pose is
available.

**Source code:** [`code/modules/perception`](../../../../code/modules/perception)
· [`perception.cpp`](../../../../code/modules/perception/perception.cpp) / [`perception.hpp`](../../../../code/modules/perception/perception.hpp)

---

## 1. Why fuse

- **LiDAR** gives accurate distance/position and width, but no colour — it
  cannot tell a red marker from a green one.
- **Camera** gives colour and a bearing, but poor distance.

The Obstacle Challenge rule is colour-dependent (pass red on one side, green on
the other), so a marker's position *and* colour must be known together. Fusion
attaches the camera colour to the LiDAR geometry.

---

## 2. Pipeline (`Perception::process`)

Inputs: `ProcessedLidarData`, `ProcessedCameraData`, and an optional vehicle
`MapPose`.

1. **Time sync gate** — a camera frame is only fused with a LiDAR scan when
   their timestamps differ by ≤ `max_sensor_time_difference_us = 100 ms`.
2. **Sanity gates** — reject NaN/Inf and out-of-range LiDAR points
   (`minimum_lidar_distance_m = 0.05`, `maximum_lidar_distance_m = 3.00`) and
   invalid camera objects, on top of the processors' own filters.
3. **Frame transform** — each LiDAR obstacle is moved from the LiDAR mount frame
   into the robot frame using `SensorMount{right_m, forward_m, yaw_rad}`.
4. **Bearing association** — for each LiDAR obstacle a **predicted camera
   bearing** is computed from its robot-frame position (via the camera mount)
   and compared to each camera object's `bearing_rad`. A pair is accepted only
   if the bearing error is within
   `max_bearing_difference_rad = 8°`.
5. **Confidence** — a `fusion_confidence` is formed from the bearing agreement;
   a pair is `frame_confirmed` only if it clears
   `minimum_confirmed_confidence = 0.55`.
6. **Colour → side** — a confirmed camera colour is mapped to
   `TrafficColor{RED,GREEN}` and a `PassSide{LEFT,RIGHT}`.

Output is `PerceptionData`: track walls, optional parking wall, a list of
`FusedObstacle`, the indices of unmatched camera detections, and a
`PerceptionDiagnostics` block.

---

## 3. `FusedObstacle` (what downstream gets)

Each obstacle reports its source (`LIDAR_ONLY` or `LIDAR_CAMERA_FUSED`),
robot-frame position, optional world position, distance, width, LiDAR bearing,
and — when fused — camera colour, `traffic_color`, `required_pass_side`, the
predicted-vs-measured bearing error, and the fusion confidence.

> **Important semantics (from the header):** `frame_confirmed` means the camera
> and LiDAR agreed **in this one frame**. It is *not* temporal tracking — a
> caller should still require the same obstacle to be confirmed over several
> frames before committing it as a map landmark. LiDAR-only obstacles are kept
> in the output (never silently dropped) but are never marked confirmed.

---

## 4. Diagnostics

`PerceptionDiagnostics` exposes counts at every stage (LiDAR input/valid/
rejected, camera input/valid/rejected, matched, frame-confirmed, unmatched on
each side) plus the timestamps and whether the pose and camera time were valid.
This is what makes the `test_perception` tool able to explain *why* an object
was or wasn't fused, instead of just showing the result.

---

## 5. Decision log (rubric criterion 4)

Visible in the code:

- **Predicted-bearing association** — projecting the LiDAR obstacle into the
  camera's expected bearing and matching against detections is robust to the
  camera's weak range, and needs only the mount geometry, not a full extrinsic
  calibration.
- **Frame-confirmed ≠ tracked** — the module is explicit that a single-frame
  match is not proof of a stable object, pushing temporal confirmation to the
  caller where lap context lives.
- **Keep unmatched detections** — unmatched LiDAR and camera objects stay in the
  output for debugging rather than being discarded.

The fusion code uses `lidar_mount` and `camera_mount` values from
`PerceptionConfig`; the current source keeps the defaults in
`code/modules/perception/perception.hpp` and does not provide a measured mount
survey. `FusedObstacle` is produced for downstream obstacle handling, but the
current Open Challenge navigation controller consumes LiDAR walls rather than
the fused obstacle list. No measured fusion-accuracy result is stored in the
repository.
