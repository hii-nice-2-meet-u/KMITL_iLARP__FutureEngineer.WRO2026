# Logging & Telemetry

Records what the robot sensed, decided, and commanded on every run, so behaviour
can be analysed and replayed offline without a debugger on the track. This is
the "Logger V1" subsystem.

**Source code:** [`code/modules/logging`](../../../../code/modules/logging)
· Async writer: [`async_csv_writer.cpp`](../../../../code/modules/logging/async_csv_writer.cpp)
· Row types & CSV: [`log_types.cpp`](../../../../code/modules/logging/log_types.cpp) / [`.hpp`](../../../../code/modules/logging/log_types.hpp)
· Loggers: [`telemetry_logger.cpp`](../../../../code/modules/logging/telemetry_logger.cpp), [`wall_logger.cpp`](../../../../code/modules/logging/wall_logger.cpp)

---

## 1. Design goal

Logging must **not** slow down the control loop. All file I/O is pushed to a
background thread; the control thread only enqueues rows.

---

## 2. `AsyncCsvWriter` — the backbone

- Opens a CSV file, writes the header, and spawns a **worker thread**
  (`writer_loop`).
- `push(row)` appends to an in-memory queue under a mutex and returns
  immediately.
- The queue is **bounded** (`maximum_queued_rows = 400`, drop-oldest): if the
  disk can't keep up, the oldest rows are dropped and counted
  (`dropped_row_count()`) rather than blocking the robot.
- A background **flush every 1 s** (`FLUSH_INTERVAL_MS = 1000`) limits data loss
  on a crash without putting I/O on the control thread.
- Write failures are latched and reportable (`has_write_error()`).

> **Durability note (from the code):** `flush()` provides OS page-cache
> durability, not `fsync`-level guarantees. A power cut can still lose the last
> unflushed rows.

---

## 3. Four log streams

Each run gets its own directory (`make_run_directory("logs")`), containing:

| File | Row type | Cadence | Purpose |
|------|----------|---------|---------|
| `telemetry.csv` | `TelemetryRow` | per tick (~20 Hz) | full state: pose, wall errors, corner tracking, raw vs shaped command, map preview, actuator output |
| `events.log` | (EventLogger) | sparse | mode transitions and faults, timestamped from run start |
| `walls.csv` | `WallRow` | NORMAL mode | wall segments (world frame) for heading-drift analysis |
| `corners.csv` | `CornerRow` | on demand | `TrackMap` corner landmarks snapshot (`dump_corners`) |

`make_telemetry_row(...)` builds a `TelemetryRow` from the pose, measured speed,
navigation state, navigation result, obstacle count, and an optional actuator
`OutputSnapshot` (motor power %, servo pulse µs).

---

## 4. `TelemetryRow` contents

The row mirrors the controller's `NavigationDebug` so every internal signal is
recoverable offline: pose (`pos_x_m, pos_y_m, heading_rad`), measured speed,
wall-follow errors (`outer_distance_m, distance_error_m, wall_angle_rad,
angle_error_rad`), validity flags, heading-hold/lost-wall state, turn tracking
(`heading_error_rad, turn_progress, turn_feedforward_rad`), **raw vs shaped**
command (`raw_steering_rad` vs `steering_rad`, `raw_target_speed_mps` vs
`target_speed_mps`), corner speed, `update_dt_s`, map preview state, obstacle
count, and the optional actuator output fields.

Having *raw and shaped* side by side is what makes the command-conditioning stage
tunable from logs.

---

## 5. Clean shutdown

`install_stop_signal_handlers()` / `notify_stop_requested()` /
`stop_requested()` let a `SIGINT`/`SIGTERM` stop the run cleanly — the challenge
apps call `notify_stop_requested()` from their signal handler so the writer
threads flush and join instead of being killed mid-write.

---

## 6. Decision log (rubric criterion 4)

Visible in the code:

- **Async, bounded, drop-oldest** — logging never blocks or unbounded-grows;
  under I/O pressure the robot keeps driving and the loss is measured.
- **Raw + shaped signals logged together** — makes the control/tuning loop
  data-driven instead of guesswork.
- **Per-run directory + four focused streams** — keeps high-rate telemetry
  separate from sparse events and from geometry meant for drift analysis.

Actuator telemetry fields are populated only in contexts that run the actuator;
monitor-only applications leave them empty. `flush()` transfers data to the
stream but does not call `fsync`, so it is not a power-loss guarantee. The
repository contains no analysis scripts or plots.
