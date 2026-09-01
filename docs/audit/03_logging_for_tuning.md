# Audit Report 3 — Logging for Tuning

Question this report answers: **after a failed run, can the team explain what
happened and re-tune from the logs alone, without re-running the robot?**
Depends on reports 01/02. Read-only audit; no source files were modified.

Ground truth for what is actually logged: `code/modules/logging/log_types.hpp`
(`TelemetryRow`, `CornerRow`, `WallRow`), `code/modules/logging/log_types.cpp`
(`telemetry_csv_header()`, `make_telemetry_row()`), and the two call sites in
`code/app/_challenge/open/main.cpp` / `code/app/_challenge/obstacle/main.cpp`.

---

## 1. D1 — Column coverage matrix

Rows = every tunable parameter in `NavigationConfig`, `StanleyConfig`,
`PIDConfig` (×3: `stanley.heading_pid`, `turn_heading_pid`, `speed_pid`), and
`ObstacleConfig`. "Observable" is applied strictly, per the plan: the log must
show the parameter's *effect*, not just a combined downstream number.

| Parameter | Effect observable in telemetry? | Column(s) | If not, what's missing |
|---|---|---|---|
| `target_outer_distance_m` | Yes | `outer_distance_m`, `distance_error_m` | — |
| `follow_corridor_center` | Yes | `corridor_center_active`, `inner_distance_m` | — |
| `search_center_kp` | Partial | `steering_rad` during `SEARCH_DIRECTION` rows | The centring *error* itself (`right_distance - left_distance` before the kp multiply) is not logged; only the final steering is, so `search_center_kp`'s effect is entangled with the clamp and cannot be isolated. |
| `search_preserve_initial_offset` | No | — | Neither the latched `search_initial_center_error_m_` nor a flag indicating the latch is active is logged anywhere; F-15's failure mode is invisible in telemetry. |
| `stanley.k` (cross-track gain) | **No** | `raw_steering_rad` (combined) | `raw_steering_rad` is `heading_term + cross_track_term` already summed inside `StanleyController::calculate`; the plan's own example (`stanley.k` needs the cross-track term seen *separately* from the heading term) is exactly this case. Neither term is logged individually. |
| `stanley.softening_speed_mps` | No | — | Same as above — folds into the un-separated cross-track term. |
| `stanley.max_steering_rad` | Partial | `steering_rad` saturating at a constant | Saturation is visible only as "steering_rad stops changing at a fixed value," which is ambiguous with `max_steering_rate_rad_s` slew-limiting or `maximum_steering_command_deg` actuator clamp reaching the same visual plateau. |
| `stanley.heading_pid.{kp,ki,kd}` | **No** | `raw_steering_rad`, `angle_error_rad` (input, not output) | The PID's own output (`heading_term`) and its integral value are not logged; only its *input* (`angle_error_rad`) and the *combined* output with the cross-track term are. Cannot separate kp/ki/kd contributions. |
| `search_front_slowdown_distance_m`, `search_front_minimum_distance_m`, `search_minimum_speed_mps` | Partial | `target_speed_mps`, `front_wall_valid` | The front-wall distance driving the slowdown is not logged during `SEARCH_DIRECTION` (only `outer_distance_m`/etc. from `NORMAL`); effect inferable only indirectly from the speed trace. |
| `approach_distance_m` | Yes | `active_approach_speed_mps`, `target_speed_mps` vs position | — |
| `turn_trigger_distance_m` | Yes | `effective_turn_trigger_m`, `front_wall_valid` | — |
| `turn_rearm_distance_m` | **No** | — | `state_.turn_armed` itself is never logged (no `TelemetryRow` field); the rearm gate's effect (F-10-adjacent "stuck disarmed" risk) is invisible. |
| `turn_preview_time_s` | Partial | `effective_turn_trigger_m` (already includes the preview term, not separable) | — |
| `turn_trigger_confirm_frames` | **No** | — | `turn_trigger_frames_` (the live counter) is not logged; only the resulting `wall_corner_trigger_active`/`front_wall_fallback_active` booleans are, once the count clears the threshold. |
| `use_wall_corner_trigger`, `front_wall_fallback_distance_m` | Yes | `wall_corner_trigger_active`, `front_wall_fallback_active` | — |
| `lidar_lateral_offset_m`, `lidar_forward_offset_m` | Partial | `wall_corner_lateral_m`/`wall_corner_forward_m` (already include the offset, not separable from the raw candidate) | — |
| `wall_corner_to_path_offset_m` | Yes | `effective_turn_trigger_m` (`calculate_geometric_turn_trigger_m` folds it in — same non-separability caveat as above) | — |
| `wall_corner_min_forward_m`, `wall_corner_max_forward_m`, `wall_corner_min_inner_length_m` | **No** | — | These are gates inside `find_inner_wall_corner_candidate`; a candidate rejected by them looks identical in the log to "no inner wall segment at all" (`wall_corner_candidate_valid=false` either way). |
| `wall_corner_stability_tolerance_m` | Partial | `wall_corner_stability_error_m` | The error is logged, but the *threshold* it is compared against is not in the row (must be cross-referenced against source/config offline). |
| `wall_corner_association_distance_m`, `wall_corner_filter_weight` | **No** | — | No column distinguishes "confirmed and freshly associated" from "confirmed and coasting on missed frames" beyond `wall_corner_confirmed`(bool). |
| `wall_corner_collinear_angle_rad`, `wall_corner_collinear_offset_m`, `wall_corner_continuation_gap_m` | **No** | — | `has_forward_wall_continuation`'s reject/accept decision is not logged at all; cannot tell offline why a candidate was rejected as "wall continues forward." |
| `wall_corner_confirm_frames` | Yes | `wall_corner_confirm_frames` | — |
| `wall_corner_max_missed_frames` | **No** | — | `wall_corner_missed_frames_` (the live counter) is not logged. |
| `wheelbase_m`, `corner_radius_m` | Partial | `turn_feedforward_rad` (combined effect) | The two constants' individual values are not logged (they are static per-run, so a config dump would suffice — see D5). |
| `turn_entry_blend_rad`, `turn_exit_blend_rad`, `exit_acceleration_blend_rad` | Partial | `turn_feedforward_rad`, `turn_progress` (post-blend combined values) | The blend *weights* themselves (`entry_weight`/`exit_weight`/`exit_acceleration_weight`) are not logged, only their end effect on the feedforward/speed — this is exactly the kind of column that would have made F-01/S1 visible from a single log (a flat `exit_acceleration_weight≈1` trace for the whole turn would have been an immediate tell). |
| `turn_heading_pid.{kp,ki,kd}` | **No** | `heading_tracking_error_rad` (input, not output), `turn_feedforward_rad` (feed-forward only, PID term not separated) | Same separability gap as `stanley.heading_pid`. |
| `heading_tolerance_rad`, `heading_confirm_frames` | Yes | `heading_error_rad`, mode transitions in `events.log` | — |
| `clockwise_turn_delta_rad`/`counter_clockwise_turn_delta_rad`, `heading_to_steering_sign` | Yes (indirectly) | Sign of `turn_feedforward_rad`/`steering_rad` vs `direction` in `events.log` | — |
| `search_speed_mps`, `normal_speed_mps`, `approach_speed_mps`, `turning_speed_mps`, `lost_wall_speed_mps` | Yes | `active_normal_speed_mps`, `active_approach_speed_mps`, `target_speed_mps`, `raw_target_speed_mps` | — |
| `enable_replay_speed_factors`, `lap2_speed_factor`, `lap3_speed_factor`, `replay_approach_factor_weight`, `maximum_replay_speed_mps` | Yes | `replay_speed_factor`, `active_normal_speed_mps`, `active_approach_speed_mps` | — |
| `replay_turn_gate_distance_m`, `replay_front_safety_override_distance_m` | **No** | — | The gate's activation (F-10) is entirely invisible: no column shows `trigger_condition` was forced `false` by the replay gate specifically (as opposed to simply not yet true). |
| `max_heading_hold_s` | Yes | `heading_hold_active`, `lost_wall_time_s` | — |
| `max_lateral_acceleration_mps2` | Yes | `corner_speed_mps` | — |
| `steering_filter_time_constant_s`, `max_steering_rate_rad_s` | Yes | `raw_steering_rad` vs `steering_rad` (before/after conditioning) | This is the one place the design explicitly succeeds at the "raw vs shaped" standard (`docs/logging/README.md` §4). |
| `max_acceleration_mps2`, `max_deceleration_mps2` | Yes | `raw_target_speed_mps` vs `target_speed_mps` | — |
| `speed_pid.{kp,ki,kd}` | **No** (and moot per F-09) | `target_acceleration_mps2` (combined, and never actuated) | Same non-separability as the other two PIDs, compounded by F-09: even a perfect log of this PID's internals would not explain robot *motion*, since the value never reaches the actuator. |
| `nominal_update_period_s`, `min_update_period_s`, `max_update_period_s` | Partial | `update_dt_s` (already clamped) | The pre-clamp raw `dt_s` is not recoverable — see D2 item 7. |
| `total_turns` | Yes (indirectly) | `lap`, `corner_index`, `FINISHED` transition in `events.log` | — |
| **`ObstacleConfig`** (all fields: `activation_distance_m`, `release_forward_m`, `pass_clearance_m`, `minimum_lookahead_m`, `avoidance_speed_mps`, `maximum_avoidance_steering_rad`, `emergency_distance_m`, `emergency_speed_mps`, `emergency_steering_rad`, `observation_merge_distance_m`, `confirmation_frames`, `maximum_confirmation_missed_frames`) | **No** | `obstacle_count` only (a raw LiDAR-obstacle count, not the fused/avoidance state) | See D2 item 1 — none of `ObstacleStatus`'s fields (`active`, `color`, `pass_side`, `forward_m`, `right_m`, `target_right_m`, `steering_rad`, `confidence`) are written to `TelemetryRow`/CSV at all; they exist only in the obstacle app's console print every 250 ms. Every `ObstacleConfig` field is therefore untunable from logs alone. |

**Summary:** distance/speed-profile parameters (turn triggers, approach/corner
speed, output-shaping rate limits) are well covered under the plan's strict
standard. All three PID controllers fail the strict standard — none logs its
proportional/integral/derivative terms separately, only combined outputs
and/or raw inputs. The entire Obstacle Challenge tunable surface
(`ObstacleConfig`) is unobservable from the CSV.

---

## 2. D2 — Blind spots

| # | Item | Status | Column(s) if present |
|---|---|---|---|
| 1 | Obstacle-avoidance state (active flag, colour, pass side, relative forward/right, target offset, avoidance steering, confidence, *why* activated/released) | **MISSING** | None. `ObstacleStatus` (`obstacle_controller.hpp:29-40`) is fully populated in memory every tick but never passed into `logging::make_telemetry_row(...)` (`obstacle/main.cpp:332-333` passes only `fused.obstacles.size()` as `obstacle_count`). `event_log->event(...)` records only the two string events "obstacle avoidance activated"/"released" (`obstacle/main.cpp:285-292`) — a timestamp and a sentence, not the state itself, and not in the per-tick CSV. Compounded by S3/F-02: during the very ticks avoidance is active, `TelemetryRow`'s *other* navigation fields are also all zero (default-constructed `NavigationDebug`), so an avoidance event is a near-total blackout in `telemetry.csv`. |
| 2 | Perception fusion diagnostics (`PerceptionDiagnostics`, printed to console every 250 ms) | **MISSING** | None. `PerceptionDiagnostics` (lidar/camera input/valid/rejected counts, matched/frame-confirmed/unmatched counts, `camera_time_synchronized`) is computed every tick in `Perception::process` and printed at `obstacle/main.cpp:342-347`, but no `TelemetryRow` field carries any of it. |
| 3 | Stanley internals (cross-track term, heading term, PID integral) | **MISSING** | `angle_error_rad`/`distance_error_m` are logged (the *inputs*), and `raw_steering_rad` is logged (the *combined output*), but the individual `cross_track_term` and `heading_term` inside `StanleyController::calculate` (`stanley_controller.cpp:13-19`), and `heading_pid_`'s integral, are not separately exposed anywhere the caller could log them — `StanleyController` has no accessor for its internal PID state. |
| 4 | Turn-heading PID internals during a corner | **MISSING** | Same gap as Stanley: `heading_tracking_error_rad` (input) and `turn_feedforward_rad` (feed-forward term only) are logged; `turn_heading_pid_`'s own output and integral are folded into `steering_rad`/`raw_steering_rad` with no separate column. |
| 5 | Speed PID internals (error, integral, output) | **PARTIAL** | `target_acceleration_mps2` is the PID's *output* and is logged — but its error/integral are not, and (per F-09) the output itself is never actuated, so even this one logged field cannot be correlated against real motor behaviour. |
| 6 | Commanded vs achieved servo pulse (is `limit_servo_pulse_step` clipping?) | **PARTIAL** | `servo_pulse_us` (the *achieved*, post-step-limit value) is logged via `OutputSnapshot`. The *commanded* (pre-step-limit) `target_servo_pulse_us` computed inside `ActuatorOutput::apply` (`open_challenge_actuator.hpp:85-88`) is a local variable never returned or logged, so clipping cannot be detected from the CSV — only inferred indirectly by comparing consecutive `servo_pulse_us` deltas against the known `maximum_servo_step_us=500` constant offline. |
| 7 | Loop timing — actual iteration period; time in LiDAR vs camera processing | **PARTIAL / MISSING** | `update_dt_s` is logged, but it is explicitly the *clamped* value (`NavigationController::calculate_dt_s`, clamped to `[min_update_period_s, max_update_period_s]`). The raw, pre-clamp `dt_s` is computed locally in the same function (`navigation_controller.cpp:1084-1101`) and discarded — **not recoverable** from the log; a stalled loop iteration is indistinguishable from a normal one once clamped at 0.12 s. Time spent in LiDAR processing vs camera processing is **MISSING** entirely — no timing instrumentation around `LidarProcessor::process`/`CameraProcessor::process` was found in scope. |
| 8 | Raw or semi-raw LiDAR scan persistence | **MISSING** | Only the three *resolved* walls are logged (`WallRow`, NORMAL-mode only — see item 9). No raw `LidarPoint` array or intermediate `CartesianPoint` list is ever written to disk; a run cannot be re-processed offline with different `LidarProcessor` thresholds. |
| 9 | All fitted line segments (not only the three resolved walls) | **PARTIAL** | `WallRow` logs `left`/`right`/`front` (the three *resolved* walls) — `ProcessedLidarData::line_segments` (every fitted/merged segment, including ones `resolve_track_walls` rejected) is never logged. Debugging a mis-classified wall (e.g. why a real wall didn't get picked as `front`) is not possible offline; it would require re-running `test_lidar` live. Also note `WallLogger::record` only fires when `mode == NavigationMode::NORMAL` (`wall_logger.cpp:16`) — no wall geometry is logged during `SEARCH_DIRECTION`, `TURNING`, or obstacle avoidance at all. |
| 10 | Camera frames (annotated JPEGs, ≤ every 500 ms, only when an object is visible) | **PARTIAL** | `obstacle/main.cpp:203-233` saves an annotated capture at most every 500 ms and only when `obstacle_candidate_visible` (a LiDAR or camera object exists). A missed detection — the camera saw nothing and LiDAR saw nothing, but a human reviewing the run knows an obstacle should have been there — produces **no image at all**, since the capture is gated on a detection already having happened. Raw (non-annotated, unconditional-cadence) frames are not persisted, so a "why didn't we see it" investigation has no visual record for the exact missed moment. |
| 11 | Battery voltage during the run | **MISSING** (start/end only) | `actuators.get_voltage()` is called once before the run loop starts and once after it ends in both `main.cpp`s (e.g. `open/main.cpp:102-111` and `:437-441`); no per-tick or even periodic voltage sample exists in `TelemetryRow` or any other stream. A mid-run voltage sag (a common cause of speed/torque loss late in a run) is invisible. |
| 12 | Which turn-trigger mechanism fired (`turn_source`) | **MISSING from CSV** | `turn_source` (`"INNER_CORNER"`/`"FRONT_FALLBACK"`/`"LEGACY_FRONT"`/`"SEARCH"`) is computed and printed to console in `open/main.cpp:227-248,382-385`, but no `TelemetryRow` field carries it — only the two booleans `wall_corner_trigger_active`/`front_wall_fallback_active` are in the CSV, from which `LEGACY_FRONT` can only be inferred as "neither boolean is true" (ambiguous with "no trigger evaluated yet" in early-lap frames). |

---

## 3. D3 — Data integrity

- **Where does `AsyncCsvWriter` drop rows, oldest or newest?**
  `push()` (`async_csv_writer.cpp:58-74`): when `queue_.size() >=
  maximum_queued_rows_` (400), it does `queue_.pop_front(); ++dropped_rows_;`
  *before* `queue_.push_back(std::move(row))` — **drop-oldest**, confirmed,
  matching `docs/logging/README.md`'s description.
- **Does the CSV show a gap marker?** No. Rows are appended to the file in
  the order they are dequeued by the writer thread
  (`writer_loop`, `async_csv_writer.cpp:98-172`); a dropped row simply never
  appears. There is no sequence number, timestamp-continuity check, or
  gap-marker row in `TelemetryRow`/`WallRow`/`CornerRow`. A dropped-rows
  event is **silently non-contiguous**: `timestamp_us` will jump by more than
  one nominal period at the drop point, but nothing distinguishes that from a
  genuinely slow loop iteration (`update_dt_s` would also be large in both
  cases) unless the reader cross-references `dropped_row_count()` (only
  available in memory/console, see next point) against the CSV by hand.
- **Are dropped-row counts reported per-writer at exit? Which are checked and
  which are not?** `open/main.cpp:427-432` checks and prints
  `telemetry_log.dropped_row_count()` and `wall_log.dropped_row_count()` to
  `stderr` if either is nonzero. **`obstacle/main.cpp` does not call
  `dropped_row_count()` on either writer at all** — the obstacle-challenge
  app has no exit-time report of dropped telemetry or wall rows, even though
  both writers exist and could drop under load (and that app additionally
  writes camera-capture JPEGs and runs OpenCV processing on the same
  process, which is more likely to create I/O/CPU pressure than the
  open-challenge app). Neither app checks `EventLogger`'s drop count anywhere
  (its queue cap is a much smaller 64 rows, `telemetry_logger.cpp:73-75`,
  making it the writer most likely to silently drop under a burst of
  mode-change events).
- **What happens to the last second of data on `SIGKILL`?** `SIGKILL` cannot
  be caught (`install_stop_signal_handlers`/`notify_stop_requested` only
  handle `SIGINT`/`SIGTERM`, `telemetry_logger.cpp:27-32`, wired via
  `std::signal(SIGINT/SIGTERM, request_stop)` in both `main.cpp`s). On
  `SIGKILL` the process terminates immediately: the writer thread's periodic
  flush (`FLUSH_INTERVAL_MS=1000`, `async_csv_writer.cpp:130-145`) means up
  to ~1 second of already-*written* (via `operator<<`) but not yet
  `flush()`-ed data can be lost to OS buffering, **plus** the entire contents
  of the in-memory `queue_` at the moment of the kill (anything pushed but
  not yet dequeued by the writer thread — bounded by up to `maximum_queued_rows`
  more rows, i.e. potentially several more seconds of telemetry at ~20 Hz)
  is lost outright, since it was never written to the `ofstream` at all. This
  matches `docs/logging/README.md`'s documented durability caveat
  ("`flush()` provides OS page-cache durability, not `fsync`-level
  guarantees") but the audit plan's specific question — the in-memory queue
  contents — is a *larger* loss window than the flush-interval caveat alone
  suggests, since it stacks the flush interval on top of however much of the
  400-row queue is unwritten at kill time.
- **Is `precision(6)` fixed enough?** `to_csv_row` sets `std::ios::fixed` and
  `precision(6)` (`log_types.cpp:55-56` etc.) for every numeric field,
  **including `timestamp_us`**, which is a `std::uint64_t` streamed through
  `operator<<` — integers are unaffected by float precision/fixed-notation
  settings in `iostream`, so `timestamp_us` is written exactly (no precision
  loss). For `float` fields: 6 decimal digits after the point is generous for
  metre-scale distances (µm resolution) and for small angles in radians
  (`heading_error_rad` at 6 decimals resolves ~0.00006°, far finer than any
  sensor's real accuracy) — **no meaningful resolution loss found** for any
  field in `TelemetryRow`/`WallRow`/`CornerRow`.
- **Can `mode` (or any field) contain a comma and break CSV parsing?**
  `row.mode` is always one of the four fixed literals returned by
  `navigation_mode_name()` (`"SEARCH_DIRECTION"`, `"NORMAL"`, `"TURNING"`,
  `"FINISHED"`) — never free text, so it cannot contain a comma. `WallRow::wall_role`
  is similarly always one of the fixed literals `"LEFT"`/`"RIGHT"`/`"FRONT"`.
  No `TelemetryRow`/`WallRow`/`CornerRow` field is ever populated from
  unescaped free-form/user text in the CSV writers in scope, so **no field was
  found that can break CSV parsing** (this part of the plan's concern is
  effectively REFUTED for the current schema — but would become a real risk
  if a future column ever logged a free-text string, e.g. an OTOS error
  message, without CSV-escaping).

---

## 4. D4 — Offline tooling fit

`/home/jukkruw/iLARP/plot_run.py` reads two CSVs and references these
columns:

- From `telemetry.csv` (its `tele_path` argument): `pos_x`, `pos_y`,
  `speed_mps`, `mode`, `lap` (via `t.pos_x.values`, `t.pos_y.values`,
  `t.speed_mps.values`, `t["mode"]`, `t.groupby("lap")`).
- From `obstacles.csv` (its `obs_path` argument, "obstacles.csv" by CLI
  default): `obs_world_x`, `obs_world_y`, `obs_color`.

Cross-checked against `telemetry_csv_header()`
(`code/modules/logging/log_types.cpp:19-38`), the actual columns are
`pos_x_m`, `pos_y_m`, `measured_speed_mps` (not `pos_x`/`pos_y`/`speed_mps`),
plus `mode` and `lap` (these two do match). **`plot_run.py` would raise
`KeyError`/`AttributeError` on `t.pos_x`, `t.pos_y`, and `t.speed_mps` if run
against a real `telemetry.csv` produced by this codebase today** — the column
names it expects do not exist in the actual header.

There is also **no `obstacles.csv` produced anywhere in the C++ code** — the
only CSVs the robot writes are `telemetry.csv`, `walls.csv`, and `corners.csv`
(`docs/logging/README.md` §3, confirmed against the three `AsyncCsvWriter`
instantiations in `TelemetryLogger`/`WallLogger`/`dump_corners`). `plot_run.py`
tolerates a missing `obstacles.csv` (`except Exception: o =
pd.DataFrame(...)`), so the script would run in that degraded mode, but the
`obs_world_x`/`obs_world_y`/`obs_color` columns it expects do not correspond
to any column the robot ever writes even if such a file existed — obstacle
world positions are never logged at all (see D2 item 1), so
`plot_run.py`'s "obstacle scatter" panel could never be populated from
real data as things stand, confirming its own docstring's assumption
("obstacle world coords are logged directly … computed once on the robot")
is currently false.

`/home/jukkruw/iLARP/gen_sample.py` (the synthetic-data generator used to
demo `plot_run.py`) independently invents the same `pos_x`/`pos_y`/`speed_mps`
naming and a from-scratch `obstacles.csv` schema
(`timestamp_us,lap,corner_index,pos_x,pos_y,heading_rad,obs_world_x,obs_world_y,obs_distance_m,obs_color`)
— it is self-consistent with `plot_run.py` but neither script's column
naming was ever reconciled against the actual C++ `telemetry_csv_header()`.

**Which D2 blind spots would need a new plot (not just a schema fix) to be
useful, once columns exist:**
- D2 #1 (obstacle avoidance state) — needs its own panel/overlay entirely
  distinct from `plot_run.py`'s current obstacle-scatter concept, since
  avoidance state is a time series (active/steering/target_right over time),
  not a static point set.
- D2 #3/#4/#5 (PID internals) — needs a new multi-line time-series plot per
  controller (P/I/D terms + setpoint + measured), which `plot_run.py` has no
  analogue of today (it only plots position/speed/mode).
- D2 #7 (loop timing) — needs a histogram/time-series of `dt_s`
  (raw, once logged) separate from the trajectory plots.
- D2 #9 (all fitted segments) — needs a per-frame geometry replay view (closer
  to `test_lidar`'s live rendering than to `plot_run.py`'s post-hoc plot),
  since segments are inherently per-tick 2-D geometry, not a scalar time
  series.

---

## 5. D5 — Proposed schema change (not implemented)

**New `TelemetryRow` columns, grouped by the blind spot they close:**

- *Stanley internals (D1, D2#3):* `stanley_cross_track_term_rad`,
  `stanley_heading_term_rad`, `stanley_heading_integral` (4 floats incl. one
  already-planned; ~16 bytes).
- *Turn-heading PID internals (D1, D2#4):* `turn_heading_pid_output_rad`,
  `turn_heading_pid_integral` (2 floats; 8 bytes).
- *Speed PID internals (D1, D2#5):* `speed_pid_error_mps`,
  `speed_pid_integral` (2 floats; 8 bytes) — logged alongside a fix to F-09
  so the numbers are actionable, not merely descriptive.
- *Servo clip visibility (D2#6):* `commanded_servo_pulse_us` (the
  pre-`limit_servo_pulse_step` value; 1 `uint16`; 2 bytes) next to the
  existing `servo_pulse_us`.
- *Loop timing (D2#7):* `raw_update_dt_s` (pre-clamp; 1 float; 4 bytes),
  `lidar_process_us`, `camera_process_us` (2 `uint32`; 8 bytes) measured
  around the respective `process()` calls in each `main.cpp`.
- *Turn-trigger diagnostics (D1):* `turn_trigger_frames` (live counter,
  1 `int`; 4 bytes), `turn_armed` (1 bool; 1 byte), `turn_source` (closes
  D2#12 — either a small enum/int rather than a string to avoid CSV/parsing
  overhead; 1 byte).
- *Replay-gate visibility (D1, F-10):* `replay_gate_suppressed` (1 bool;
  1 byte).
- *Obstacle-avoidance state (D2#1):* `obstacle_active`, `obstacle_color`,
  `obstacle_pass_side` (bools/small enums; 3 bytes), `obstacle_forward_m`,
  `obstacle_right_m`, `obstacle_target_right_m`, `obstacle_steering_rad`,
  `obstacle_confidence` (5 floats; 20 bytes) — populated from
  `ObstacleStatus` at the `obstacle/main.cpp` call site regardless of which
  branch (`priority_command` vs `navigation.update()`) produced the tick's
  command (this also requires the F-02 fix so `NavigationDebug`'s other
  fields are not simultaneously blanked on the same rows).
- *Perception diagnostics (D2#2):* `lidar_valid_count`, `camera_valid_count`,
  `matched_count`, `frame_confirmed_count` (4 `uint16`; 8 bytes) from
  `PerceptionDiagnostics`.
- *Battery (D2#11):* `battery_voltage_v` (1 float; 4 bytes), sampled at a
  low cadence (e.g. once per second, not every tick — `read_voltage_v()` is a
  blocking SPI round-trip with a 2 ms response delay per
  `spi_master.cpp:21,162`, too slow to call at 20 Hz without stealing loop
  time).

**New CSV files:**

- `segments.csv` — one row per fitted `LineSegment` per tick (closes D2#9),
  proposed header: `timestamp_us,segment_index,start_x_m,start_y_m,end_x_m,
  end_y_m,angle_rad,rms_error_m,role` (`role` = `LEFT`/`RIGHT`/`FRONT`/`NONE`,
  cheaper than re-deriving classification offline). Written every tick
  regardless of mode (unlike today's NORMAL-only `walls.csv`).
- `scan.csv` or a binary equivalent — one row (or record) per raw `LidarPoint`
  per scan (closes D2#8), header: `timestamp_us,angle_deg,distance_m,quality`.
  Given ~600-1000 points per RPLIDAR S3 scan at 1200 RPM / 20 Hz, this is by
  far the highest-volume proposed stream (see cost estimate below); a binary
  framed format (fixed-size records) rather than CSV text is recommended if
  this is added, specifically to keep the per-scan write cost and queue
  memory bounded.

**Estimated per-row byte cost and rows/second at the current ~20 Hz loop rate:**

- Current `TelemetryRow` CSV line is already ~50 fields; at ~12-15 bytes/field
  average (mix of small ints/bools and 6-decimal floats) that is roughly
  650-750 bytes/row today, ×20 Hz ≈ 13-15 KB/s.
- The proposed `TelemetryRow` additions above total ~25 new fields, roughly
  200-250 bytes/row, raising the per-row cost to ~850-1000 bytes and the
  telemetry stream to ~17-20 KB/s (+30-35%) — well within what
  `AsyncCsvWriter`'s 400-row queue (already sized for ~20 s of buffering at
  20 Hz) and a 1 s flush interval can absorb without new drops, *provided*
  disk write throughput is not already the bottleneck (unverified in this
  audit — no measured write-latency data exists in the repository).
- `segments.csv`: typically 3-8 resolved+rejected segments per tick × ~60
  bytes/row × 20 Hz ≈ 3.6-9.6 KB/s.
- `scan.csv` (if added as text CSV): ~800 points/scan × ~25 bytes/row × 20 Hz
  ≈ **400 KB/s** — over an order of magnitude larger than every other stream
  combined, and the reason a binary format is recommended above rather than
  CSV text if raw-scan persistence is pursued; even a binary record
  (`uint64+float32+float32+uint8` ≈ 17 bytes/point) is still ~800×17×20 ≈
  272 KB/s, which is the single largest storage/queue-sizing decision in this
  proposal and should be sized against the actual SD-card/eMMC write
  bandwidth on the Pi before committing to per-run raw-scan capture.
