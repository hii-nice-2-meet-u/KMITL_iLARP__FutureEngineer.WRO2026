# Tuning Variables — the full list, and what the log captures

> **สรุปไทย** — รายการตัวแปรจูนทั้งหมด แยกตามระบบ พร้อมบอกว่าตัวไหน `run_metadata`
> เก็บไว้ใน log แล้ว (✓) ตัวไหนยัง**ไม่เก็บ** (✗) เวลาเอา log ไปวิเคราะห์จะได้รู้ว่า
> รันนั้นใช้ค่าอะไร และตัวไหนที่ยัง "มองไม่เห็น" จาก log
>
> ✗ = ต้องเพิ่มเข้า `code/modules/logging/run_metadata.cpp` ถ้าอยากให้ log จำค่าได้

**Where the log stores these:** each run writes `run_meta.json` in its log
directory (`run_metadata.cpp`). Values marked ✓ appear there. Values marked ✗
are compiled in but **not recorded**, so a log alone cannot tell you what they
were — you must cross-reference the source at that commit.

**Source of truth for the values:** `open_challenge_common.hpp`
(`make_navigation_config`) for navigation; `obstacle/main.cpp` for the obstacle
overrides; `ActuatorConfig` defaults in `open_challenge_actuator.hpp`.

---

## 1. NavigationConfig — corridor following & speed

| Variable | Open value | Logged | Note |
|---|---|---|---|
| `target_outer_distance_m` | 0.27 | ✓ | fallback wall target |
| `follow_corridor_center` | true | ✓ | |
| `max_steering_rad` | 38° | ✓ | global steering clamp (F-11) |
| `search_center_kp` | 1.5 | ✓ | |
| `search_preserve_initial_offset` | true | **✗** | F-15 latch — invisible in log |
| `search_front_slowdown_distance_m` | 0.70 | ✓ | |
| `search_front_minimum_distance_m` | 0.20 | ✓ | |
| `search_minimum_speed_mps` | 0.06 | ✓ | |
| `search_speed_mps` | 0.19 | ✓ | |
| `normal_speed_mps` | 0.45 | ✓ | |
| `approach_speed_mps` | 0.26 | ✓ | |
| `turning_speed_mps` | 0.28 | ✓ | binds corner speed (post T-03) |
| `lost_wall_speed_mps` | default | ✓ | |
| `max_lateral_acceleration_mps2` | 0.50 | ✓ | inert today (exceeded 274%) |

## 2. NavigationConfig — turn trigger & corner geometry

| Variable | Open value | Logged | Note |
|---|---|---|---|
| `approach_distance_m` | 0.90 | ✓ | |
| `turn_trigger_distance_m` | 0.65 | ✓ | LEGACY_FRONT trigger |
| `turn_rearm_distance_m` | 0.80 | ✓ | |
| `turn_preview_time_s` | 0.10 | ✓ | latency compensation guess |
| `turn_trigger_confirm_frames` | default 2 | **✗** | |
| `use_wall_corner_trigger` | true | ✓ | |
| `front_wall_fallback_distance_m` | 0.56 | ✓ | |
| `lidar_lateral_offset_m` | 0.0 | ✓ | |
| `lidar_forward_offset_m` | 0.081875 | ✓ | duplicated in perception mount |
| `wall_corner_to_path_offset_m` | 0.02 | ✓ | |
| `wall_corner_min_forward_m` | default 0.08 | **✗** | corner-candidate gate |
| `wall_corner_max_forward_m` | default 1.50 | **✗** | corner-candidate gate |
| `wall_corner_min_inner_length_m` | 0.20 | ✓ | |
| `wall_corner_stability_tolerance_m` | 0.04 | ✓ | ≈ scan-skew floor (~20 mm) |
| `wall_corner_association_distance_m` | default 0.12 | ✓ | |
| `wall_corner_filter_weight` | default 0.25 | **✗** | EMA weight |
| `wall_corner_collinear_angle_rad` | default 8° | ✓ | |
| `wall_corner_collinear_offset_m` | default 0.05 | ✓ | |
| `wall_corner_continuation_gap_m` | default 0.20 | ✓ | |
| `wall_corner_confirm_frames` | 2 | **✗** | |
| `wall_corner_max_missed_frames` | default 3 | ✓ | |

## 3. NavigationConfig — corner trajectory & completion

| Variable | Open value | Logged | Note |
|---|---|---|---|
| `wheelbase_m` | 0.16375 | ✓ | measurement, not a gain |
| `corner_radius_m` | 0.45 | ✓ | feeds 3 things at once (§ study) |
| `turn_entry_blend_rad` | 22.5° | ✓ | ~5× larger than the servo needs |
| `turn_exit_blend_rad` | 32° | ✓ | |
| `exit_acceleration_blend_rad` | 20° | **✗** | **the F-01 param — NOT logged** |
| `heading_tolerance_rad` | 16.5° | ✓ | |
| `heading_confirm_frames` | 2 | ✓ | |
| `clockwise_turn_delta_rad` | −90° | **✗** | sign convention |
| `counter_clockwise_turn_delta_rad` | +90° | ✓ | |
| `heading_to_steering_sign` | −1 | **✗** | sign convention |

## 4. NavigationConfig — output shaping & timing

| Variable | Open value | Logged | Note |
|---|---|---|---|
| `steering_filter_time_constant_s` | 0.035 | ✓ | low-pass τ |
| `max_steering_rate_rad_s` | 3.0 | **✗** | slew limit — source of the future κ̇ limit |
| `max_acceleration_mps2` | 5.0 | ✓ | never binds (real ~0.78) |
| `max_deceleration_mps2` | 5.0 | ✓ | never binds (real ~0.48) |
| `nominal_update_period_s` | default 0.05 | **✗** | |
| `min_update_period_s` | default 0.005 | ✓ | dt clamp |
| `max_update_period_s` | default 0.12 | ✓ | dt clamp |
| `total_turns` | 12 | ✓ | |

## 5. NavigationConfig — replay (laps 2–3)

| Variable | Open value | Logged | Note |
|---|---|---|---|
| `enable_replay_speed_factors` | true (open) | **✗** | master switch — NOT logged |
| `lap2_speed_factor` | 1.10 | ✓ | |
| `lap3_speed_factor` | 1.15 | ✓ | |
| `replay_approach_factor_weight` | 0.50 | ✓ | |
| `maximum_replay_speed_mps` | 0.55 | **✗** | replay speed cap |
| `replay_turn_gate_distance_m` | 0.40 | ✓ | F-10 gate |
| `replay_front_safety_override_distance_m` | 0.25 | ✓ | F-10 override |
| `max_heading_hold_s` | 0.30 | ✓ | |

## 6. StanleyConfig (`config.stanley`)

| Variable | Open value | Logged | Note |
|---|---|---|---|
| `k` | 1.00 | ✓ | cross-track gain |
| `softening_speed_mps` | default 0.20 | ✓ | speed-dependent gain shaping |
| `max_steering_rad` | 38° | ✓ | Stanley's own internal clamp |
| `heading_pid.kp` | 1.00 | ✓ | |
| `heading_pid.ki` | 0.08 | ✓ | |
| `heading_pid.kd` | 0.075 | ✓ | |
| `heading_pid.{min,max}_output` | ±45° | ✓ | |
| `heading_pid.{min,max}_integral` | ±0.5 | ✓ | |
| `heading_pid.max_dt_s` | 0.10 | ✓ | |

## 7. turn_heading_pid (`config.turn_heading_pid`)

| Variable | Open value | Logged | Note |
|---|---|---|---|
| `kp` | 0.30 | ✓ | tuned while output was clamped — re-check |
| `ki` | 0.08 | ✓ | |
| `kd` | 0.048 | ✓ | |
| `{min,max}_output` | ±15° | ✓ | narrowed in T-04 |
| `{min,max}_integral`, `max_dt_s` | defaults | ✓ | |

> `speed_pid` was **deleted** (P-03 / F-09). No longer a tuning variable.

---

## 8. LidarProcessor::process() arguments — ✗ NONE LOGGED

Passed positionally in `process_scan` (`open_challenge_common.hpp`):
`processor.process(scan, wall_correction_rad, 4, 0.035f, 0.12f, 5.0f, 0.04f, 0.10f)`

| Position | Parameter | Value | Logged |
|---|---|---|---|
| 3 | `min_segment_point` | 4 | **✗** |
| 4 | `max_line_error_m` | 0.035 | **✗** |
| 5 | `max_point_gap_m` | 0.12 | **✗** (P-23 replaces this) |
| 6 | `max_angle_diff` (deg) | 5.0 | **✗** |
| 7 | `max_collinear_error_m` | 0.04 | **✗** |
| 8 | `max_segment_gap_m` | 0.10 | **✗** |

**These are magic numbers at the call site — the entire LiDAR geometry pipeline
is untunable from a log.** Highest-priority logging gap after ActuatorConfig.
Also note the internal `constexpr` thresholds inside `lidar_processor.cpp`
(`MIN_WALL_LENGTH_M = 0.25`, obstacle width gates, etc.) are compile-time and
never logged.

## 9. ActuatorConfig — ✗ NONE LOGGED

| Variable | Value | Logged | Note |
|---|---|---|---|
| `wheel_diameter_m` | 0.053 | **✗** | speed↔RPM conversion |
| `maximum_wheel_rpm` | 1500 | **✗** | |
| `motor_rpm_command_scale` | **0.571** | **✗** | **the 1.75× drivetrain correction — invisible in log** |
| `servo_min_pulse_us` | 950 | **✗** | |
| `servo_center_pulse_us` | 1475 | **✗** | |
| `servo_max_pulse_us` | 2000 | **✗** | |
| `maximum_servo_step_us` | 500 | **✗** | servo slew guard |
| `steering_to_servo_sign` | 1.0 | **✗** | sign convention |
| `maximum_steering_command_deg` | 45 | **✗** | pulse-map range (M-4 relevant) |
| `spi_speed_hz` | 15 MHz | **✗** | |

**Without these, a log cannot explain the robot's actual speed or steering**:
`motor_rpm_command_scale` and `wheel_diameter_m` set the commanded-speed→RPM
map, and the servo pulse triplet + `maximum_steering_command_deg` set the
steering-angle→pulse map. Both are central to every M-* analysis.

## 10. ObstacleConfig (`obstacle/main.cpp`) — ✗ NONE LOGGED

| Variable | Value | Logged | Note |
|---|---|---|---|
| `activation_distance_m` | 1.50 | **✗** | |
| `pass_clearance_m` | 0.32 | **✗** | |
| `minimum_lookahead_m` | 0.35 | **✗** | |
| `avoidance_speed_mps` | 0.25 | **✗** | |
| `maximum_avoidance_steering_rad` | 32° | **✗** | |
| `confirmation_frames` | 1 | **✗** | deliberate (D3), but unrecorded |
| `emergency_distance_m` | 0.35 | **✗** | |
| `emergency_speed_mps` | 0.16 | **✗** | |
| `emergency_steering_rad` | 32° | **✗** | |
| `observation_merge_distance_m` | default 0.18 | **✗** | |
| `release_forward_m` | default −0.10 | **✗** | |
| `maximum_confirmation_missed_frames` | default 5 | **✗** | |

The obstacle app's `make_navigation_config()` overrides
(`normal_speed_mps=0.20`, `approach_speed_mps=0.17`, `turning_speed_mps=0.20`,
`search_speed_mps=0.15`, `maximum_replay_speed_mps=0.42`,
`enable_replay_speed_factors=false`) are also **not** captured — the obstacle
run's `run_meta.json`, if written, would show the *base* config, not these.

## 11. PerceptionConfig (`obstacle/main.cpp`) — ✗ NONE LOGGED

`lidar_mount {0,0.081875,0}`, `camera_mount {0,0,0}`,
`max_sensor_time_difference_us = 100000`, `max_bearing_difference_rad = 8°`,
`minimum_confirmed_confidence = 0.55`, plus the distance sanity gates.

---

## Priority to add to `run_metadata`

Ranked by how much a missing value blocks analysis:

1. **ActuatorConfig** (§9) — without `motor_rpm_command_scale`,
   `wheel_diameter_m`, and the servo/steering map, no speed or steering log is
   interpretable. This is the top gap.
2. **LidarProcessor::process args** (§8) — the whole geometry pipeline is
   untunable from a log; six numbers.
3. **`exit_acceleration_blend_rad`, `search_preserve_initial_offset`,
   `enable_replay_speed_factors`, `max_steering_rate_rad_s`,
   `maximum_replay_speed_mps`** (§§3–5) — behaviourally significant Navigation
   fields with known failure modes, currently invisible.
4. **ObstacleConfig + the obstacle-app config overrides** (§10) — the entire
   Obstacle Challenge tuning surface, once that mode is run for real.
5. **PerceptionConfig** (§11).

> Every addition to `run_metadata.cpp` is JSON only — it does not touch the
> per-tick telemetry schema, so no field-count discipline is needed there. Add
> the ObstacleConfig / ActuatorConfig serialisers alongside the existing
> `navigation_config_json`.
