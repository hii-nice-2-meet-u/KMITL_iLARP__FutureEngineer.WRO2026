# Logging & Telemetry

Records what the robot sensed, decided, and commanded on every run, so behaviour
can be analysed and re-tuned offline without a debugger on the track.

**Source code:** [`code/modules/logging`](../../../../code/modules/logging)
· Async writer: [`async_csv_writer.cpp`](../../../../code/modules/logging/async_csv_writer.cpp)
· Row types & CSV: [`log_types.cpp`](../../../../code/modules/logging/log_types.cpp) / [`.hpp`](../../../../code/modules/logging/log_types.hpp)
· Loggers: [`telemetry_logger.cpp`](../../../../code/modules/logging/telemetry_logger.cpp), [`wall_logger.cpp`](../../../../code/modules/logging/wall_logger.cpp), [`segment_logger.cpp`](../../../../code/modules/logging/segment_logger.cpp)
· Run metadata: [`run_metadata.cpp`](../../../../code/modules/logging/run_metadata.cpp)
· Offline tools: [`code/tools`](../../../../code/tools)

---

## 1. Design goal

Logging must **not** slow the control loop. All file I/O is pushed to a
background thread; the control thread only enqueues rows.

---

## 2. `AsyncCsvWriter` — the backbone

- Opens a CSV file, writes the header, and spawns a **worker thread**.
- `push(row)` appends to an in-memory queue under a mutex and returns
  immediately.
- The queue is **bounded** (`maximum_queued_rows = 400`, drop-oldest): if the
  disk can't keep up, the oldest rows are dropped and counted rather than
  blocking the robot.
- A background **flush every 1 s** limits data loss on a crash without putting
  I/O on the control thread.
- Write failures are latched and reportable (`has_write_error()`).

> **Durability note:** `flush()` provides OS page-cache durability, not
> `fsync`-level guarantees. A power cut can still lose the last unflushed rows,
> plus whatever remained in the in-memory queue.

---

## 3. The missing-value convention (read this first)

The one rule the schema depends on, documented at the top of `log_types.hpp`:

> **A numeric column of `0` means "measured, and the value was zero" — never
> "not measured".** Any group that can be absent carries a paired `*_valid`
> boolean, written in **every** navigation mode, not only the mode that computes
> it. Consumers must gate on the flag before plotting or averaging.

This exists because the controller builds a fresh `NavigationDebug` each tick, so
an unwritten field would otherwise log a plausible `0`. Before this was enforced,
wall distances read `0` through every corner and made plots look broken. Key
pairs: `outer_distance_m`/`outer_wall_valid`, `inner_distance_m`/
`inner_wall_valid`, `distance_error_m`/`wall_following_active`,
`effective_turn_trigger_m` & `wall_corner_*`/`turn_trigger_evaluated`,
`battery_voltage_v`/`battery_valid`, `camera_process_us`/`camera_process_valid`.
Absence is **never** encoded as `NaN` (it parses inconsistently across tools).

---

## 4. Per-run outputs

Each run gets its own directory (`make_run_directory("logs")`), containing:

| File | Row type | Cadence | Purpose |
|------|----------|---------|---------|
| `run_meta.json` | — | once at start (+ drop counts at exit) | provenance: git commit/dirty, build time, executable, telemetry field count, full config dump (navigation/stanley/PID/actuator, and perception/obstacle for the obstacle app), OTOS scalars actually applied |
| `telemetry.csv` | `TelemetryRow` | per tick (~20 Hz) | full per-tick state (§5) |
| `walls.csv` | `WallRow` | **every mode** | resolved wall segments, with a `mode` column to filter |
| `segments.csv` | `SegmentRow` | every mode | **all** fitted line segments (not just the 3 resolved walls), each tagged LEFT/RIGHT/FRONT/NONE — debugs a mis-classified wall offline |
| `corners.csv` | `CornerRow` | on exit | `TrackMap` corner landmarks snapshot |
| `events.log` | (EventLogger) | sparse | mode transitions and faults, timestamped from run start |

A run **refuses to start** if `run_meta.json` cannot be written — an
unattributable log is worse than none. Drop counts for all three async writers
(telemetry, walls, events) are printed at exit and recorded back into
`run_meta.json`.

---

## 5. `TelemetryRow` contents (94 columns)

Grouped by what each group answers. Every group follows §3.

- **Pose & motion:** `pos_x_m, pos_y_m, heading_rad, measured_speed_mps`.
- **Raw OTOS channels:** `pose_timestamp_us` (same `steady_clock` epoch as the
  LiDAR scan, so scan-to-pose skew is measurable), `otos_velocity_x/y_mps`,
  `otos_yaw_rate_rps`, `otos_accel_x/y_mps2` — the signed velocity, yaw rate and
  acceleration the controller's unsigned `hypot` speed throws away.
- **Wall following:** `outer_distance_m, inner_distance_m, distance_error_m,
  wall_angle_rad, angle_error_rad` + validity flags; `wall_correction_rad` (the
  heading de-rotation applied before classification — subtract to recover raw
  geometry).
- **Turn & corner tracking:** `heading_error_rad, turn_progress,
  turn_feedforward_rad, effective_turn_trigger_m, wall_corner_*`,
  `turn_trigger_source/frames/armed/evaluated`, `replay_gate_suppressed`.
- **Controller internals:** Stanley cross-track/heading terms and integral,
  turn-heading PID output/integral — so PID gains are tunable from a log, not
  only from the summed command.
- **Raw vs shaped command:** `raw_steering_rad` vs `steering_rad`,
  `raw_target_speed_mps` vs `target_speed_mps` — makes the conditioning stage
  tunable.
- **Timing:** `update_dt_s` (clamped) and `raw_update_dt_s` (pre-clamp);
  `lidar_process_us`, `camera_process_us` (per-stage wall-clock).
- **Battery:** `battery_voltage_v` + age, sampled ~1 Hz (blocking SPI read).
- **Obstacle avoidance & fusion:** obstacle active/colour/side/geometry/world
  position; camera-LiDAR fusion counts; `lidar_points_total` and the
  `rejected_quality`/`rejected_range` tallies (a mis-set LiDAR threshold shows
  up here without persisting the raw scan).
- **Actuator output:** `wheel_rpm`, `servo_pulse_us` (sent) and
  `commanded_servo_pulse_us` (pre-step-limit — reveals when `maximum_servo_step_us`
  is clipping). Empty for monitor-only apps.

---

## 6. Clean shutdown

`install_stop_signal_handlers()` / `notify_stop_requested()` let a
`SIGINT`/`SIGTERM` stop a run cleanly — the challenge apps call
`notify_stop_requested()` from their signal handler so the writer threads flush
and join instead of being killed mid-write. `SIGKILL` cannot be caught and loses
the flush interval plus the in-memory queue.

---

## 7. Offline tools ([`code/tools`](../../../../code/tools))

- **`plot_run.py <telemetry.csv> [out]`** — top-down map (speed-coloured
  trajectory, obstacles, per-lap overlay) plus validity-gated signal time series.
  Reads columns from the file header and gates each series on its `*_valid` flag,
  so it renders old and new schemas and draws gaps, not false zeros.
- **`check_run.py <run_dir>`** — automated QA; non-zero exit on any failure.
  Flags per-mode zero-fill, `row_index` gaps, writer drop counts, `dt` outliers,
  a `measured/target` speed ratio outside `[0.85, 1.15]`, header/`run_meta`
  field-count mismatch, and value/validity-flag contradictions.

---

## 8. Decision log (rubric criterion 4)

- **Async, bounded, drop-oldest** — logging never blocks or grows unbounded;
  under I/O pressure the robot keeps driving and the loss is measured.
- **Evidence, not conclusions** — every tier-2 value is a function of the raw
  inputs plus the config; the missing-value convention keeps "not measured"
  distinct from "measured zero".
- **Per-run provenance** — `run_meta.json` makes a log attributable to an exact
  build and config, so a plot from two weeks ago is still interpretable.
- **Raw + shaped signals together** — makes control/tuning data-driven.
- **Geometry logged in every mode** — walls and segments are no longer
  NORMAL-only, so corners and direction search are debuggable too.
