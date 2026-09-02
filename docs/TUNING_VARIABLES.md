# Tuning Variables — the full list, per-run recorded in `run_meta.json`

> **สรุปไทย** — รายการตัวแปรจูนทั้งหมด แยกตามระบบ ทุก run เขียนค่าที่ใช้จริงลง
> `run_meta.json` ในโฟลเดอร์ log ของ run นั้น ดังนั้นตอนจูนคนละรอบใช้เลขคนละชุด
> log แต่ละชุดจะจำค่าของตัวเองไว้ครบ เอาไปวิเคราะห์ย้อนหลังได้ว่ารันนั้นใช้เลขอะไร
>
> **สถานะ ณ commit `f11b581`: ทุกกลุ่มถูกบันทึกครบแล้ว** ไม่มีช่องโหว่เหลือ

**Where it is stored.** Each run creates a log directory and writes
`run_meta.json` into it (`run_metadata.cpp` + the app-side serialisers). It
records the full config that run used, plus the git SHA and build time, so a
log from any tuning round is self-describing. It is written twice — once at
startup, once at shutdown with the drop counters — via atomic rename.

**How to read it back.** Open `run_meta.json` from the run's log folder. Every
value below appears there under the object named in its section header.

**Source of the default values.** `make_navigation_config()` in
`open_challenge_common.hpp` (open), the overrides in `obstacle/main.cpp`
(obstacle), `ActuatorConfig` defaults in `open_challenge_actuator.hpp`.

> ⚠️ Correction to an earlier draft of this file: an earlier version marked
> many fields "not logged". That was a false alarm from a text search that
> missed keys wrapped onto a second line. **Every group below is recorded.**

---

## What `run_meta.json` contains, by top-level object

| JSON object | Serialiser | Covers | Written by |
|---|---|---|---|
| `navigation_config` | `navigation_config_json` | all of NavigationConfig (§1) + nested `stanley`, `turn_heading_pid` | both apps |
| `actuator_config` | `actuator_config_json` | all of ActuatorConfig (§3) | both apps |
| `lidar_process_params` | `lidar_process_params_json` | the 6 LidarProcessor::process thresholds (§4) | both apps |
| `otos` | in `run_metadata.cpp` | measured linear/angular scalars + validity | both apps |
| `obstacle_config` | `obstacle_config_json` | all of ObstacleConfig (§5) | obstacle app only |
| `perception_config` | `perception_config_json` | mounts, sync/bearing/confidence gates (§6) | obstacle app only |
| `schema`, `build`, `git_dirty` | `run_metadata.cpp` | schema version, build stamp, dirty-tree flag | both apps |
| `logging` | app shutdown | writer drop counts | both apps |

> Note: the obstacle app overrides several NavigationConfig speeds
> (`normal_speed_mps=0.20`, `approach_speed_mps=0.17`, `turning_speed_mps=0.20`,
> `search_speed_mps=0.15`, `maximum_replay_speed_mps=0.42`,
> `enable_replay_speed_factors=false`). Because it builds the config first and
> then serialises it, its `navigation_config` object reflects **the overridden
> values**, not the open-challenge base — so it is correct per run.

---

## 1. NavigationConfig (object `navigation_config`) — current open values

**Corridor following & speed**

| Variable | Open value | Note |
|---|---|---|
| `target_outer_distance_m` | 0.27 | fallback wall target |
| `follow_corridor_center` | true | |
| `max_steering_rad` | 38° | global steering clamp (F-11) |
| `search_center_kp` | 1.5 | |
| `search_preserve_initial_offset` | true | F-15 latch |
| `search_front_slowdown_distance_m` | 0.70 | |
| `search_front_minimum_distance_m` | 0.20 | |
| `search_minimum_speed_mps` | 0.06 | |
| `search_speed_mps` | 0.19 | |
| `normal_speed_mps` | 0.45 | |
| `approach_speed_mps` | 0.26 | |
| `turning_speed_mps` | 0.28 | binds corner speed (post T-03) |
| `lost_wall_speed_mps` | default | |
| `max_lateral_acceleration_mps2` | 0.50 | inert today (exceeded 274%) |

**Turn trigger & corner geometry**

| Variable | Open value | Note |
|---|---|---|
| `approach_distance_m` | 0.90 | |
| `turn_trigger_distance_m` | 0.65 | LEGACY_FRONT trigger |
| `turn_rearm_distance_m` | 0.80 | |
| `turn_preview_time_s` | 0.10 | latency-compensation guess |
| `turn_trigger_confirm_frames` | 2 | |
| `use_wall_corner_trigger` | true | |
| `front_wall_fallback_distance_m` | 0.56 | |
| `lidar_lateral_offset_m` | 0.0 | |
| `lidar_forward_offset_m` | 0.081875 | also in perception mount |
| `wall_corner_to_path_offset_m` | 0.02 | |
| `wall_corner_min_forward_m` | 0.08 | |
| `wall_corner_max_forward_m` | 1.50 | covers the post-T-03 ~0.50 m trigger |
| `wall_corner_min_inner_length_m` | 0.20 | |
| `wall_corner_stability_tolerance_m` | 0.04 | ≈ scan-skew floor (~20 mm) |
| `wall_corner_association_distance_m` | 0.12 | |
| `wall_corner_filter_weight` | 0.25 | EMA weight |
| `wall_corner_collinear_angle_rad` | 8° | |
| `wall_corner_collinear_offset_m` | 0.05 | |
| `wall_corner_continuation_gap_m` | 0.20 | |
| `wall_corner_confirm_frames` | 2 | |
| `wall_corner_max_missed_frames` | 3 | |

**Corner trajectory & completion**

| Variable | Open value | Note |
|---|---|---|
| `wheelbase_m` | 0.16375 | measurement, not a gain |
| `corner_radius_m` | 0.45 | feeds feed-forward, corner speed, trigger |
| `turn_entry_blend_rad` | 22.5° | ~5× larger than the servo needs |
| `turn_exit_blend_rad` | 32° | |
| `exit_acceleration_blend_rad` | 20° | the former F-01 param (now correct units) |
| `heading_tolerance_rad` | 16.5° | |
| `heading_confirm_frames` | 2 | |
| `clockwise_turn_delta_rad` | −90° | sign convention |
| `counter_clockwise_turn_delta_rad` | +90° | |
| `heading_to_steering_sign` | −1 | sign convention |

**Output shaping & timing**

| Variable | Open value | Note |
|---|---|---|
| `steering_filter_time_constant_s` | 0.035 | low-pass τ |
| `max_steering_rate_rad_s` | 3.0 | slew limit; future κ̇-limit source |
| `max_acceleration_mps2` | 5.0 | never binds (real ~0.78) |
| `max_deceleration_mps2` | 5.0 | never binds (real ~0.48) |
| `nominal_update_period_s` | 0.05 | |
| `min_update_period_s` | 0.005 | dt clamp |
| `max_update_period_s` | 0.12 | dt clamp |
| `total_turns` | 12 | |

**Replay (laps 2–3)**

| Variable | Open value | Note |
|---|---|---|
| `enable_replay_speed_factors` | true | master switch |
| `lap2_speed_factor` | 1.10 | |
| `lap3_speed_factor` | 1.15 | |
| `replay_approach_factor_weight` | 0.50 | |
| `maximum_replay_speed_mps` | 0.55 | replay speed cap |
| `replay_turn_gate_distance_m` | 0.40 | F-10 gate |
| `replay_front_safety_override_distance_m` | 0.25 | F-10 override |
| `max_heading_hold_s` | 0.30 | |

## 2. Nested controllers (inside `navigation_config`)

**`stanley`** — `k` 1.00, `softening_speed_mps` 0.20, `max_steering_rad` 38°,
and nested `heading_pid`: `kp` 1.00, `ki` 0.08, `kd` 0.075, `min/max_output`
±45°, `min/max_integral` ±0.5, `max_dt_s` 0.10.

**`turn_heading_pid`** — `kp` 0.30, `ki` 0.08, `kd` 0.048, `min/max_output`
±15° (narrowed in T-04), `min/max_integral` ±0.5, `max_dt_s` 0.10.

> `speed_pid` was deleted (P-03 / F-09) — no longer a variable.

## 3. ActuatorConfig (object `actuator_config`)

| Variable | Value | Note |
|---|---|---|
| `wheel_diameter_m` | 0.053 | speed↔RPM conversion |
| `maximum_wheel_rpm` | 1500 | |
| `motor_rpm_command_scale` | 0.571 | provisional 1.75× drivetrain correction |
| `servo_min_pulse_us` | 950 | steering→pulse map |
| `servo_center_pulse_us` | 1475 | |
| `servo_max_pulse_us` | 2000 | |
| `maximum_servo_step_us` | 500 | servo slew guard |
| `steering_to_servo_sign` | 1.0 | sign convention |
| `maximum_steering_command_deg` | 45 | pulse-map range (M-4 relevant) |
| `spi_chip_select`, `spi_speed_hz` | 0, 15 MHz | link config |

## 4. LidarProcessParams (object `lidar_process_params`)

The `LidarProcessor::process()` thresholds, hoisted out of the call site
(`LidarProcessParams` in `open_challenge_common.hpp`):

| Variable | Value | Note |
|---|---|---|
| `min_segment_point` | 4 | min points to accept a segment |
| `max_line_error_m` | 0.035 | RMS gate on a line fit |
| `max_point_gap_m` | 0.12 | split threshold (P-23 replaces with adaptive) |
| `max_angle_diff_deg` | 5.0 | merge-alignment tolerance |
| `max_collinear_error_m` | 0.04 | merge-collinearity tolerance |
| `max_segment_gap_m` | 0.10 | merge gap |

> Not recorded: the compile-time `constexpr` thresholds *inside*
> `lidar_processor.cpp` (`MIN_WALL_LENGTH_M = 0.25`, obstacle width gates,
> parking-wall constants). They are not runtime-tunable, so they change only
> with the build — the `build`/`git_dirty` fields in `run_meta.json` identify
> which source produced them.

## 5. ObstacleConfig (object `obstacle_config`, obstacle app only)

`activation_distance_m` 1.50, `release_forward_m` −0.10, `pass_clearance_m`
0.32, `minimum_lookahead_m` 0.35, `avoidance_speed_mps` 0.25,
`maximum_avoidance_steering_rad` 32°, `emergency_distance_m` 0.35,
`emergency_speed_mps` 0.16, `emergency_steering_rad` 32°,
`observation_merge_distance_m` 0.18, `confirmation_frames` 1 (deliberate, D3),
`maximum_confirmation_missed_frames` 5.

## 6. PerceptionConfig (object `perception_config`, obstacle app only)

`lidar_mount` {0, 0.081875, 0}, `camera_mount` {0, 0, 0},
`max_sensor_time_difference_us` 100000, `max_bearing_difference_rad` 8°,
`minimum_confirmed_confidence` 0.55, `min/max_lidar_distance_m` gates.

---

## Per-tick vs per-run — where each kind of number lives

- **Per-run config** (this document) → `run_meta.json`, once per run. The
  *values you tuned*.
- **Per-tick telemetry** → `telemetry.csv`, one row per control tick. The
  *effect* of those values over time (93 columns; see
  `docs/audit/03_logging_for_tuning.md` and the `O-*` logging work). Includes
  the controller internals (`stanley_*_term_rad`, `turn_heading_pid_*`),
  turn-trigger state, obstacle state, fusion counts, and `row_index` for
  drop detection.

To analyse a tuning round: read `run_meta.json` for **what values were set**,
then `telemetry.csv` / `segments.csv` / `walls.csv` for **what they did**.
