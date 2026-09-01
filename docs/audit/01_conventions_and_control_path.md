# Audit Report 1 — Conventions and Control Path

Scope: drive/steering control path (Subsystem A) as defined in
`docs/AUDIT_PLAN_DRIVE_AND_LOGGING.md`. Working tree at commit `4f9f970` plus
uncommitted changes. Read-only audit; no source files were modified.

---

## 1. Phase A — Convention ledger

| # | Convention | Defined where | Formula as written in code | Consumers | Agrees with ledger? |
|---|---|---|---|---|---|
| 1 | LiDAR raw angle meaning | `mark.txt`, `docs/lidar/README.md` | "0° = front (หน้า), 90° = right (ขวา)" | `LidarProcessor::polar2cartesian` | **NO — see §2.1** |
| 2 | LiDAR polar → robot Cartesian | `LidarProcessor::polar2cartesian` (`code/modules/lidar/lidar_processor.cpp:135-143`) | `x_m = distance_m * -sin(rad); y_m = distance_m * -cos(rad);` where `rad = angle_deg * PI/180` | All of `LidarProcessor` (split/merge, `resolve_track_walls`, `detect_obstacles`, `find_parking_wall`) | **NO** — `mark.txt` and `docs/lidar/README.md` both specify `x = d·sin(angle), y = d·cos(angle)` (no negation). The code's formula is the point-reflection (180°-rotated) of the documented one. See §2.1. |
| 3 | Robot frame axes (`lidar::LineSegment`, `ObstacleObject`) | Implicit from `polar2cartesian` output | `+x_m`/`+y_m` are whatever `polar2cartesian` produces (see row 2) | `resolve_track_walls` (`center.x<0`→left, `center.y>0`→front), `detect_obstacles`, `ObstacleObject::bearing_rad() = atan2(center.x, center.y)` | Internally self-consistent (all LiDAR-side code uses the same, possibly-rotated, frame) but not necessarily aligned with `mark.txt`'s stated "+X=right, +Y=forward" (see §2.1) |
| 4 | `perception::RobotPoint` axes | `perception.hpp:49-52`, comment lines 14-17 | `right_m` = robot-frame right, `forward_m` = robot-frame forward. Comment: "Bearing is positive to the right" | `Perception::lidar_to_robot`, `predicted_camera_bearing`, `ObstacleController::to_robot`/`nearest_observation` | `lidar_to_robot()` maps LiDAR's `cv::Point2f{x,y}` → `RobotPoint{right_m, forward_m}` via `right=x·cos(yaw)+y·sin(yaw)`, `forward = -x·sin(yaw)+y·cos(yaw)` — i.e. it assumes LiDAR's `x` axis **is already** "right" and `y` **is already** "forward" (mount yaw = 0 in both live apps). This assumption is only correct if `polar2cartesian` follows the documented (non-negated) convention. Given row 2, this is a place where a wrong `polar2cartesian` silently propagates: `lidar_to_robot` does not re-derive right/forward from raw angle, it trusts the LiDAR module's `x/y` labels. |
| 5 | Wall line angle | `LidarProcessor::fit_line_segment` (PCA), `resolve_track_walls` | `theta = 0.5*atan2(2*Sxy, Sxx-Syy)`; classification: `|corrected_angle| ≈ 90°` → left/right, `|corrected_angle| ≈ 0°` and `center.y>0` → front | `resolve_track_walls`, `NavigationController::calculate_wall_heading_error`, `has_forward_wall_continuation` | Self-consistent within the (possibly rotated) LiDAR frame from row 2. If row 2's frame is 180°-rotated from the documented one, "front" as classified here corresponds to physical **behind**, not physical front, unless the raw-angle convention itself differs from `mark.txt` at the mount level (see §2.1 discussion — cannot fully resolve without hardware). |
| 6 | Steering sign | `navigation_controller.hpp:121-129` comment; `stanley_controller.hpp:46-48` | "steering < 0 = LEFT, steering > 0 = RIGHT" | `StanleyController::calculate`, `NavigationController::calculate_cross_track_error/calculate_wall_heading_error/calculate_search_steering`, `condition_command`, `ActuatorOutput::to_servo_pulse_us` (`steering_to_servo_sign=1.0`, positive steering → positive `signed_command_deg` → pulse above `servo_center_pulse_us`), `ObstacleController::apply` (`side=+1`→RIGHT) | **Consistent** everywhere it is used within scope — see B1/B2 traces. `heading_to_steering_sign=-1` and `turn_heading_sign_` are the two explicit flips (row 7). |
| 7 | Sign flips of the steering convention | `heading_to_steering_sign` (`navigation_controller.hpp:129`, default `-1`); `turn_heading_sign_` (member, set in `start_turn()`); `steering_to_servo_sign` (`open_challenge_actuator.hpp:26`, `1.0`) | `heading_to_steering_sign` converts OTOS heading-error sign (CCW+) to steering sign; `turn_heading_sign_ = heading_delta>=0 ? +1 : -1` selects which way the moving reference sweeps; `steering_to_servo_sign` is a final pass-through multiplier (`1.0`, i.e. currently a no-op) before the servo pulse map | `update_normal` (lost-wall heading hold), `update_turning` (feedforward + tracking term), `to_servo_pulse_us` | Consistent chain: OTOS heading-error (rad, CCW+) × `heading_to_steering_sign(-1)` → steering convention (+ = RIGHT) × `steering_to_servo_sign(1.0)` → servo command. No contradiction found in this chain. |
| 8 | OTOS heading sign | `navigation_controller.hpp:112-116` comment: "+heading = CCW" | `clockwise_turn_delta_rad = -PI/2`, `counter_clockwise_turn_delta_rad = +PI/2` | `start_turn`, `calculate_turn_heading_error`, `update_normal`'s lost-wall heading hold, `perception::robot_to_world` (comment: "positive heading turns toward +world Y") | **Assumed but never independently verified in software** — this is a hardware fact about the OTOS unit's firmware/mounting; the code is internally consistent with the assumption but nothing in the repository proves the physical sensor actually reports CCW-positive. This is the single hardware measurement needed to settle S4 (see report 02). |
| 9 | Robot → world transform (forward) | Three implementations, compared algebraically below | See §2.2 | `perception::Perception::robot_to_world`, `NavigationController::update_wall_corner_landmark` (forward), `ObstacleController::to_robot` (inverse) | **Two of the three implementations disagree by a 90° rotation** — see §2.2. `CONFIRMED` as S4 in report 02. |
| 10 | Bearing convention | `perception.hpp:14-17` comment: "Bearing is positive to the right" | `ObstacleObject::bearing_rad() = atan2(center.x, center.y)` (lidar_struct); `Perception::predicted_camera_bearing = atan2(relative_right, relative_forward) - camera_yaw` | `Perception::process` bearing-matching loop, camera fusion | Consistent: both use `atan2(right-like, forward-like)`, so positive bearing = to the right, matching the doc comment. Depends on row 2/3 being correct for the LiDAR side. |
| 11 | Units — steering/angle config fields | `navigation_controller.hpp`, `open_challenge_common.hpp` | Correctly-converted fields consistently use `deg * PI / 180.0f` (e.g. `turn_entry_blend_rad`, `turn_exit_blend_rad`, `heading_tolerance_rad`, `wall_corner_collinear_angle_rad`) | All angle-typed `NavigationConfig`/`ObstacleConfig` fields | **One violation found: `exit_acceleration_blend_rad = 20.0f` at `open_challenge_common.hpp:95` has no `* PI / 180.0f`.** `CONFIRMED` as S1 in report 02. |
| 12 | Units — distance/speed fields | Field names carry `_m`, `_mps`, `_mps2`, `_s` suffixes throughout `NavigationConfig`, `ObstacleConfig`, `ActuatorConfig` | — | — | All distance/speed/time fields observed in scope carry a unit suffix in their name. No un-suffixed ambiguous field was found. |
| 13 | `StanleyConfig::max_steering_rad` default-value comment | `stanley_controller.hpp:18` | `float max_steering_rad{0.785398f}; // 30 deg` | Comment only (no functional effect) | **Comment is wrong.** `0.785398 rad = 45.0°`, not 30°. `docs/control/README.md` independently states the default is "30°", inheriting the same stale comment. Minor doc-drift (LOW), noted under duplicated/stale documentation, not a behavioural bug. |

### 2.1 Convention disagreement — LiDAR polar→Cartesian (row 1/2/3)

`code/modules/lidar/lidar_processor.cpp:135-143`:

```cpp
CartesianPoint LidarProcessor::polar2cartesian(const LidarPoint &point) const {
	CartesianPoint result;
	result.distance_m = point.distance_m;
	const float rad = point.angle_deg * static_cast<float>(M_PI) / 180.f;
	result.x_m = point.distance_m * -std::sin(rad);
	result.y_m = point.distance_m * -std::cos(rad);
	return result;
}
```

`/home/jukkruw/iLARP/mark.txt` (the load-bearing convention document named in the
audit plan) states:

```
LiDAR raw angle          Robot Cartesian
0° = หน้า (front)    →    +X = ขวา (right)
90° = ขวา (right)    →    +Y = หน้า (forward)
```

and `docs/lidar/README.md:109-114` independently states the same formula
without negation: `x = distance·sin(angle), y = distance·cos(angle)`.

Evaluating both at `angle_deg = 0` (documented as "front"):
- Documented formula: `x=0, y=+distance` (point ahead, on the +Y/forward axis). Correct.
- Code: `x = -distance·sin(0) = 0`, `y = -distance·cos(0) = -distance` (point placed **behind** the robot).

At `angle_deg = 90` (documented as "right"):
- Documented formula: `x=+distance, y=0` (point to the right).
- Code: `x = -distance·sin(90°) = -distance`, `y = -distance·cos(90°) = 0` (point placed to the **left**).

The code's formula is `(x,y) = -(documented x, documented y)` at every angle —
a 180° rotation (point reflection) of the documented mapping, not merely a
single-axis sign flip. This is internally self-consistent (it is still a valid
right-handed 2-D frame; `resolve_track_walls`, `detect_obstacles`, etc. all
consume the same rotated frame), so it does not necessarily mean the robot
mis-detects walls — **if** the physical LiDAR unit's raw `angle_deg=0` axis is
actually mounted pointing toward the robot's rear (a mounting choice, not a bug),
the negation would be the correct compensation and `mark.txt`/`docs/lidar/README.md`
would be the stale documents.

This cannot be resolved from source alone. It is carried into report 02 as a
confirmed convention *disagreement* between code and both cited documents; the
single test that would settle it is: point the LiDAR's silkscreen/cable-exit
reference mark (raw `angle_deg=0`) at a known wall and confirm in `test_lidar`
whether that wall renders in front of (+Y) or behind (-Y) the drawn robot
origin.

### 2.2 Robot → world transform — three implementations compared (row 9)

**`perception::Perception::robot_to_world`** (`code/modules/perception/perception.cpp:231-246`):

```cpp
world.x = pose.x + forward_m * cos(heading) + right_m * sin(heading);
world.y = pose.y + forward_m * sin(heading) - right_m * cos(heading);
```

Comment at line 237-239: "OTOS/world convention: heading 0 points along +world
X and positive heading turns toward +world Y. Robot-right is therefore
-world Y at heading 0." This is algebraically a proper rotation matrix applied
to the orthonormal basis (forward = +X at heading 0, right = -Y at heading 0):
at `heading=0`, `world = pose + (forward, -right)`, matching the stated
convention exactly.

**`NavigationController::update_wall_corner_landmark`** forward transform
(`code/modules/navigation/navigation_controller.cpp:730-736`):

```cpp
const float cosine = std::cos(map_pose->heading_rad);
const float sine = std::sin(map_pose->heading_rad);
const cv::Point2f candidate_world{
	map_pose->x_m - forward_m * sine + right_m * cosine,
	map_pose->y_m + forward_m * cosine + right_m * sine};
```

i.e. `world.x = pose.x - forward*sin(h) + right*cos(h)`, `world.y = pose.y +
forward*cos(h) + right*sin(h)`. At `heading=0`: `world = pose + (right,
forward)` — **forward maps onto +world Y and right maps onto +world X**, the
opposite axis assignment from `perception::robot_to_world`'s stated convention
(which puts forward on +world X at heading 0). The two formulas differ by a
constant 90° rotation for all headings (verified algebraically by rotating the
`perception` basis vectors `(1,0)` and `(0,-1)` by 90°: `(1,0)→(0,1)`,
`(0,-1)→(1,0)`, which reproduces exactly the `right→+X, forward→+Y` mapping
used in `navigation_controller.cpp`).

The same function's **inverse** (world→robot, used for
`wall_corner_forward_m`/`wall_corner_lateral_m`, lines 798-803):

```cpp
debug.wall_corner_forward_m = -delta_x * sine + delta_y * cosine;
debug.wall_corner_lateral_m = delta_x * cosine + delta_y * sine;
```

is the correct algebraic inverse of its own forward transform above (internally
consistent with itself), so the 90° discrepancy is solely between
`navigation_controller.cpp`'s convention and `perception.cpp`'s convention, not
an internal bug within `navigation_controller.cpp`.

**`obstacle_challenge::ObstacleController::to_robot`**
(`code/app/_challenge/obstacle/obstacle_controller.hpp:232-239`):

```cpp
static RobotPosition to_robot(const Observation &observation, const navigation::MapPose &pose) {
	const float dx = observation.x_m - pose.x_m;
	const float dy = observation.y_m - pose.y_m;
	const float cosine = std::cos(pose.heading_rad);
	const float sine = std::sin(pose.heading_rad);
	return {dx * sine - dy * cosine, dx * cosine + dy * sine};
}
```

Algebraically inverting `perception::robot_to_world`'s formula for
`(forward,right)` given `(dx,dy)=(world-pose)` yields exactly `forward =
dx·cos(h)+dy·sin(h)`, `right = dx·sin(h)-dy·cos(h)` — which is precisely what
`to_robot` computes (`RobotPosition{right_m, forward_m}` = `{dx·sin-dy·cos,
dx·cos+dy·sin}`). **`to_robot` is the exact inverse of `perception::robot_to_world`,
not of `navigation_controller.cpp`'s forward transform.**

**Conclusion:** two of the three implementations in scope
(`perception::robot_to_world` and `obstacle_controller.hpp::to_robot`) agree
with each other; `navigation_controller.cpp`'s wall-corner-landmark transform
is the outlier, rotated 90° from the other two. This is strong internal
evidence (not proof) that the wall-corner-landmark transform is the wrong one,
but confirming which convention matches the *physical* OTOS output still
needs hardware: drive the robot in a straight line with heading held at 0 and
confirm the world X/Y the OTOS reports moves along the axis the team calls
"forward" on the field diagram. Full detail and severity carried to report 02
as **S4 — CONFIRMED (convention mismatch), NEEDS HARDWARE (which is physically correct)**.

---

## 2. Phase B1 — Three end-to-end traces

### B1.a — NORMAL mode, one loop iteration

1. **LiDAR scan arrives** (`lidar.wait_for_data(scan)` in `open/main.cpp:139`).
2. **OTOS pose read** (`otos.getPosVelAcc`) → `heading_rad`, `speed_mps =
   hypot(velocity.x, velocity.y)`.
3. **Wall-heading correction angle** computed:
   `wall_correction_rad = normalize_angle(heading_rad - state.target_heading_rad)`
   (`open/main.cpp:188-189`).
4. **`LidarProcessor::process(scan, wall_correction_rad, 4, 0.035, 0.12, 5.0, 0.04, 0.10)`**
   (`open_challenge_common.hpp:127-131`, `process_scan`) — produces
   `ProcessedLidarData{line_segments, walls{left,right,front}, obstacles,
   parking_wall}`. Internally: `polar2cartesian` → split/merge → PCA fit
   (`fit_line_segment`) → `merge_aligned_segments` → `resolve_track_walls`
   (heading-corrected by `wall_correction_rad`) → `detect_obstacles`.
5. **`NavigationController::update(processed, heading_rad, speed_mps,
   replay_hint, map_pose)`** (`navigation_controller.cpp:16`):
   a. `dt_s = calculate_dt_s(scan.timestamp_us)` — clamped to
      `[min_update_period_s, max_update_period_s] = [0.005, 0.12]` s.
   b. Mode is `NORMAL` → `update_normal(...)`:
      - `resolve_track_walls(lidar_data.walls)` maps left/right → inner/outer
        by `state_.direction` (CW: outer=LEFT, inner=RIGHT; CCW: outer=RIGHT,
        inner=LEFT).
      - Rearm check: if `!state_.turn_armed` and (no front wall, or front wall
        `> turn_rearm_distance_m`) → re-arm and zero `turn_trigger_frames_`.
      - **`should_start_turn(...)`** evaluated every iteration (see B5); if it
        returns `false` (usual case mid-straight), continue.
      - If `track_walls.outer == nullptr` → lost-wall branch (`stanley_.reset()`,
        speed forced to `lost_wall_speed_mps`, steering from heading-hold or 0)
        — **returns early**, skipping the Stanley/speed-profile code below.
      - Otherwise: `cross_track_error_m` from
        `calculate_center_cross_track_error` (corridor-center mode, both walls
        seen) or `calculate_cross_track_error` (outer-only). `heading_error_rad`
        similarly averaged or outer-only.
      - **`command.steering_rad = stanley_.calculate(cross_track_error_m,
        heading_error_rad, speed_mps, dt_s)`** — inside `StanleyController::calculate`
        (`stanley_controller.cpp:8-23`): `cross_track_term =
        atan2(k·cross_track_error_m, speed+softening_speed_mps)`;
        `heading_term = heading_pid_.calculate(0, -heading_error_rad, dt_s)`
        (a full PID, `mutable`, called even though the method is `const`);
        `steering = heading_term + cross_track_term`, clamped to
        `±config_.stanley.max_steering_rad` (38° in the open-challenge config).
      - `command.target_speed_mps = calculate_active_normal_speed_mps()`
        (base `normal_speed_mps`, or replay-factor-scaled on laps ≥2 if
        `enable_replay_speed_factors`); reduced toward
        `calculate_approach_speed_mps(...)` if the wall-corner landmark is
        confirmed and near, or if the front wall is inside
        `approach_distance_m`; further capped by the learned-map preview
        blend if `replay_hint->approach_recommended`.
   c. Back in `update()`: `result.debug.raw_steering_rad =
      result.command.steering_rad` (pre-conditioning snapshot), similarly for
      `raw_target_speed_mps`.
   d. **`condition_command(result.command, speed_mps, dt_s, stop_immediately=false)`**
      (`navigation_controller.cpp:1104-1161`):
      - Speed: rate-limited toward `requested_speed_mps` by
        `max_acceleration_mps2`/`max_deceleration_mps2` × `dt_s` →
        `conditioned_speed_mps_`.
      - Steering: `clamp_steering()` first (±38°), then a first-order
        low-pass filter (`time_constant_s = steering_filter_time_constant_s`,
        `filter_weight = dt/(tc+dt)`), then a slew-rate clamp
        (`max_steering_rate_rad_s × dt_s`), then `clamp_steering()` again →
        `conditioned_steering_rad_`.
      - `acceleration_mps2 = speed_pid_.calculate(conditioned_speed_mps_,
        actual_speed_mps, dt_s)` — **computed but see B6/S8: never actuated.**
      - Returns `{conditioned_speed_mps_, conditioned_steering_rad_,
        acceleration_mps2}`.
6. **`ActuatorOutput::apply(result.command, wheel_rpm_override)`**
   (`open_challenge_actuator.hpp:79-105`):
   - `target_servo_pulse_us = to_servo_pulse_us(command.steering_rad)`:
     convert rad→deg, clamp to `±maximum_steering_command_deg` (45°), apply
     `steering_to_servo_sign` (1.0, no-op), map linearly onto
     `[servo_min_pulse_us, servo_max_pulse_us] = [950, 2000]` around
     `servo_center_pulse_us = 1475`.
   - `servo_pulse_us = limit_servo_pulse_step(target_servo_pulse_us)` — clamps
     the **per-call** pulse delta to `±maximum_servo_step_us = 500 µs`
     (a second, independent rate limiter beyond `max_steering_rate_rad_s`).
   - `wheel_rpm = to_wheel_rpm(command.target_speed_mps)` (open-loop m/s→RPM
     via `wheel_diameter_m`, clamped to `±maximum_wheel_rpm`), unless
     `wheel_rpm_override` is set (search-launch boost).
   - `bus_.set_servo_pulse_us(servo_pulse_us)` → SPI `Command::SERVO_PULSE`
     (3-byte frame). `bus_.set_motor_speed(M1/M2, wheel_rpm)` → SPI
     `Command::M1_SPD`/`M2_SPD` — the **closed-loop** RPM command consumed by
     the STM32's own encoder loop (see B6).
7. **Final integer written to the bus:** a 3-byte SPI frame `{command_byte,
   data_hi, data_lo}` for the servo pulse (`std::uint16_t` µs, e.g. `1475` at
   dead-ahead) and a second 3-byte frame for each motor's signed RPM
   (`std::int16_t`, two's-complement).

### B1.b — TURNING mode, one loop iteration

Entered via `start_turn(heading_rad)` (called from inside `update_normal` when
`should_start_turn()` is true) then immediately falls through to
`update_turning()` in the same tick (`navigation_controller.cpp:209-215`).

1. `start_turn()`: advances `state_.target_heading_rad` by
   `±90°` (`clockwise_turn_delta_rad`/`counter_clockwise_turn_delta_rad`),
   computes `signed_turn_angle_rad` from the *cardinal target*, not the raw
   instantaneous OTOS heading (comment: avoids accumulating a few degrees of
   error per corner). Resets `turn_heading_pid_`, the wall-corner tracker,
   `lost_wall_timer_s_`; **does not reset `stanley_`** (see S13). Sets
   `turn_entry_steering_rad_ = conditioned_steering_rad_` (continuity with the
   last commanded steering). Sets `state_.mode = TURNING`.
2. `update_turning(heading_rad, speed_mps, dt_s, debug)`
   (`navigation_controller.cpp:362-486`):
   - `radius_m = max(0.05, corner_radius_m)` (0.12 m in open config).
   - `corner_speed_mps = calculate_corner_speed_mps()` =
     `min(turning_speed_mps, sqrt(max_lateral_acceleration_mps2 · radius_m))`.
   - Moving reference: `turn_reference_progress_rad_ += min(remaining,
     reference_speed_mps/radius_m · dt_s)` — an angular-rate model driven by
     *measured* speed clamped to `corner_speed_mps`.
   - `heading_error_rad = normalize_angle(state_.target_heading_rad -
     heading_rad)` (OTOS convention). `tracking_error_rad =
     normalize_angle(turn_reference_heading_rad_ - heading_rad)`.
   - Entry/exit blend weights via `smoothstep(progress/blend_rad)` using
     `turn_entry_blend_rad` (22.5°) and `turn_exit_blend_rad` (32°).
   - `feedforward_rad = heading_to_steering_sign · turn_heading_sign_ ·
     atan2(max(0.01,wheelbase_m), radius_m) · min(entry_weight, exit_weight)`
     — **magnitude atan2(0.16375, 0.12) = 53.8° before any blending/clamp**
     (see S9 in report 02).
   - `entry_steering_rad = turn_entry_steering_rad_ · (1 - entry_weight)` —
     continuity term that decays away as the entry blend completes.
   - `tracking_steering_rad = turn_heading_pid_.calculate(0, -tracking_error_rad,
     dt_s) · heading_to_steering_sign`.
   - `command.steering_rad = clamp_steering(entry_steering_rad + feedforward_rad
     + tracking_steering_rad)` — clamp to ±38° (open config).
   - Speed: `exit_acceleration_weight = 1 - smoothstep(|heading_error_rad| /
     exit_acceleration_blend_rad)`; `command.target_speed_mps = corner_speed_mps
     + (active_normal_speed_mps - corner_speed_mps) · exit_acceleration_weight`
     — **with the current mis-set `exit_acceleration_blend_rad=20.0` (raw
     radians, not 20°), this weight is ≈1 for the entire turn, not just near
     exit; see S1.**
   - `is_turn_complete(heading_error_rad)`: true when
     (`reference_complete` and `heading_confirm_frames` streak reached) OR
     (`turn_heading_sign_·heading_error_rad ≤ 0` and reference progressed
     ≥80%) — i.e. the machine can also exit by "crossing" the target even
     without formal confirmation frames. On completion: `++turn_count`,
     `corner_index/lap` recomputed, mode→`NORMAL` (or `FINISHED` at
     `total_turns=12`), `state_.turn_armed=false` to block immediate
     re-trigger from the old corner's wall.
3. `condition_command(...)` runs identically to B1.a (steering filter + slew
   limit + clamp, speed ramp, unused `speed_pid_` output).
4. Same actuator path as B1.a to the final SPI frames.

### B1.c — Obstacle-avoidance path (`code/app/_challenge/obstacle/main.cpp`)

1. LiDAR scan, OTOS pose read (same as B1.a).
2. `processed_lidar = process_scan(...)` (LiDAR pipeline, same as B1.a).
3. Camera frame processed if available; `fused =
   perception.process(processed_lidar, processed_camera, pose)` — LiDAR/camera
   fusion, bearing association, `frame_confirmed` flags.
4. Confirmed fused obstacles are folded into `track_map.observe_traffic_light(...)`.
5. `priority_command.target_speed_mps = obstacle_config.avoidance_speed_mps`
   (0.25 m/s) is pre-seeded; `obstacle_controller.apply(fused, pose,
   navigation.state().mode, track_map.traffic_landmarks(),
   navigation.state().lap>=1, priority_command)` (`obstacle_controller.hpp:46-135`):
   - Skipped entirely only when `mode==FINISHED`.
   - Tracks a `candidate_` obstacle (nearest confirmed fused obstacle, or
     nearest confident map landmark once `lap>=1`), promotes it to `active_`
     after `confirmation_frames` (1, per `obstacle/main.cpp:67` override) or
     immediately if within `emergency_distance_m` (0.35 m).
   - Once `active_`: `relative = to_robot(*active_, pose)` (world→robot,
     consistent with `perception::robot_to_world`, see §2.2).
   - `side = pass_side==RIGHT ? +1 : -1`. `target_right_m = relative.right_m +
     side·pass_clearance_m` (0.32 m). `avoidance_steering_rad =
     clamp(atan2(target_right_m, lookahead_m), ±maximum_avoidance_steering_rad)`
     (32°).
   - **If `relative.forward_m ≤ emergency_distance_m`:**
     `command.steering_rad = side · emergency_steering_rad` (full lock, 32°,
     ignoring `target_right_m`/geometry entirely — S7); speed clamped to
     `emergency_speed_mps` (0.16 m/s).
   - **Else:** `command.steering_rad = avoidance_steering_rad` (unconditional
     assignment, replacing whatever steering value was already in `command` —
     S5); speed clamped to `avoidance_speed_mps` (0.25 m/s).
6. **Branch point (`obstacle/main.cpp:252-258`):**
   ```cpp
   navigation::NavigationResult result;
   if (obstacle_status.active) {
       result.command = priority_command;
   } else {
       result = navigation.update(processed_lidar, heading_rad, speed_mps, replay_hint, pose);
   }
   ```
   When avoidance is active, `navigation.update()` — and therefore
   `condition_command()` (steering low-pass + slew limit + speed ramp),
   `calculate_dt_s`'s `previous_timestamp_us_` bookkeeping, and the entire
   NORMAL/TURNING state machine — **does not run at all** this tick (S2).
   `result.debug` stays default-constructed (all zeros/false) because
   `NavigationResult result;` default-constructs both members and only
   `result.command` is overwritten (S3).
7. `actuators.apply(result.command, wheel_rpm_override)` — **the avoidance
   steering value reaches the servo-pulse mapping directly, without ever
   passing through `NavigationController::clamp_steering()` (38°),
   the steering low-pass filter, or the `max_steering_rate_rad_s` slew limit.**
   The only steering-side protections still in effect are `obstacle_controller`'s
   own clamp (32° ObstacleConfig, tighter than 38° so the servo-command clamp
   (45°) and actuator's `maximum_servo_step_us` (500 µs/tick, ≈ a large but
   real single-tick pulse-jump cap) still apply — those two are actuator-layer
   protections, not navigation-layer ones, and they do not filter/smooth, only
   hard-clip.
8. Final SPI frames as in B1.a (`set_servo_pulse_us`, `set_motor_speed` ×2).

---

## 3. B2 — Who owns `steering_rad`?

| Writer | Mode(s) active | Add-to or replace? | Upstream/downstream of `condition_command()` |
|---|---|---|---|
| `StanleyController::calculate` (via `update_normal`) | `NORMAL` (wall visible) | Replace (`command.steering_rad = stanley_.calculate(...)`) | Upstream — filtered/slew-limited/clamped afterward |
| `update_normal` lost-wall branch | `NORMAL` (no outer wall) | Replace (`heading_to_steering_sign·heading_error_rad` or `0`) | Upstream |
| `calculate_search_steering` (via `update_search_direction`) | `SEARCH_DIRECTION` | Replace | Upstream (still passed through `condition_command` at the end of `update()`) |
| `update_turning` (feedforward + entry-continuity + tracking PID, summed then clamped) | `TURNING` | Replace (assigns the sum) | Upstream |
| `ObstacleController::apply` | Any mode except `FINISHED` (including `TURNING`, `SEARCH_DIRECTION`) — see S6 | **Replace**, and unconditionally overwrites whatever `priority_command.steering_rad` held before the call (which is always `0.0f`, the default-constructed value, since `priority_command` is freshly constructed each tick in `obstacle/main.cpp:246`) | **Entirely outside `NavigationController::condition_command()`** — see B1.c step 7 |

**Can two writers be active in the same iteration?** Within
`NavigationController` itself, no — `update()` dispatches to exactly one of
`update_search_direction`/`update_normal`/`update_turning` per call via the
mode switch, so only one navigation-side writer runs per tick. But at the
application level (`obstacle/main.cpp`), the branch at lines 252-258 is an
`if/else`: `ObstacleController::apply()` **always** runs (any non-`FINISHED`
mode) and conditionally *pre-seeds* `priority_command`; `navigation.update()`
only runs in the `else` branch. So the two "writers" — the navigation state
machine and the obstacle controller — are mutually exclusive within a tick by
construction of the `if/else`, not by any coordination inside
`NavigationController`. The consequence is not two writers fighting in the
same tick; it is that **the obstacle controller can pre-empt the navigation
state machine indefinitely**, including while `state.mode == TURNING`, and the
navigation controller has no visibility into or control over that pre-emption
(confirmed S2/S6, detailed in report 02).

---

## 4. B3 — State that goes stale when `update()` is skipped

`obstacle/main.cpp` is the only call site in scope that conditionally skips
`navigation.update()` (confirmed at `obstacle/main.cpp:252-258`, see B1.c/S2).
`open/main.cpp` calls `navigation.update()` unconditionally every iteration.

| Member | Updated in | Effect of skipped iterations | Severity |
|---|---|---|---|
| `previous_timestamp_us_` | `calculate_dt_s()`, called at the top of `update()` | Freezes at the timestamp of the last tick before avoidance began. When `update()` resumes, `dt_s` is computed from the *entire* avoidance duration, then clamped to `max_update_period_s=0.12s` — so the very next NORMAL/TURNING tick after avoidance ends uses an artificially large `dt_s` (up to the clamp), not the true inter-frame time. | MEDIUM (one-tick PID/derivative distortion on resume) |
| `last_elapsed_update_s_` | `calculate_dt_s()` | Same staleness; also feeds `lost_wall_timer_s_ += last_elapsed_update_s_` on the *next* NORMAL tick if the wall is still lost, understating/overstating the lost-wall duration by the skipped time. | LOW–MEDIUM |
| `conditioned_steering_rad_`, `conditioned_speed_mps_` | `condition_command()` | Frozen at their pre-avoidance values. Because avoidance steering bypasses `condition_command()` entirely (B1.c step 7), when `update()` resumes the filter "restarts" from the stale pre-avoidance value, not from whatever steering the robot was actually last commanded to (the avoidance steering). This produces a discontinuous filter input on resume. | HIGH |
| `command_conditioner_initialized_` | `condition_command()` | Stays `true`; harmless on its own, but combines with the two rows above to make the resume discontinuity silent (no re-init happens to smooth it out). | LOW |
| `wall_corner_*` (anchor/filtered/stable/missed/confirmed) | `update_wall_corner_landmark()`, called only from `should_start_turn()`, called only from `update_normal()` | Frozen. If avoidance runs long enough to cross `wall_corner_max_missed_frames` worth of *wall-clock* time in one shot, the tracker does **not** reset during that gap (resets only happen on a call to `update_wall_corner_landmark`, which does not happen). On resume it looks at the landmark exactly as it was, potentially long stale, and the very next real observation is treated as "compared against a fresh anchor" rather than "3+ frames missed" — the miss-counter logic is bypassed by the skip, not honored. | MEDIUM |
| `turn_trigger_frames_` | `should_start_turn()` | Frozen; on resume, if it had partially accumulated confirmation frames before avoidance began, that partial count is still there and combines with fresh frames after resume as if no gap occurred — a corner trigger could be confirmed using frames that straddle an arbitrarily long avoidance gap. | MEDIUM |
| `lost_wall_timer_s_` | `update_normal()` (increment) / reset on wall reacquire | Frozen during the skip; see `last_elapsed_update_s_` row — resumes accumulating from its pre-avoidance value using a possibly-inflated `dt_s`. | LOW–MEDIUM |
| `state_.mode` | `update()`'s dispatch + `start_turn()`/turn-complete transition | Frozen — if avoidance activates mid-`TURNING`, the state machine simply never advances the corner arc (`turn_reference_progress_rad_` also frozen, see next row) until avoidance releases. The robot can be well past the corner's true heading (steered by the obstacle controller instead) while the navigation controller still believes it is at the same point in the turn it was at when avoidance started. | HIGH (feeds directly into S6) |
| `turn_reference_progress_rad_`, `turn_reference_heading_rad_`, `turn_total_angle_rad_`, `turn_heading_sign_`, `turn_entry_steering_rad_` | `update_turning()` (progress), `start_turn()` (the rest) | Frozen while `TURNING`; see previous row. On resume, the moving-reference model picks up as though no time passed, so the reference sweeps from where it left off using the *current* `dt_s` (post-resume, possibly the inflated one from the timestamp row) — the corner arc effectively "teleports" the reference forward by one inflated step rather than by the true elapsed angle. | HIGH |
| `stanley_` (PID + config) | `StanleyController::calculate()` (its internal `heading_pid_`), reset in `update_normal`'s lost-wall branch and in `NavigationController::reset()` | Not called at all while avoidance is active (only reachable from `update_normal`), so `stanley_`'s internal PID `previous_time_`/`previous_error_` are simply not touched — no staleness accumulates in the PID's own dt-tracking because it uses the explicit `dt_s` overload (see B4), not the wall-clock overload. Low risk from the skip itself. | LOW |
| `turn_heading_pid_` | `update_turning()` (calculate), `start_turn()` (reset) | Same as `stanley_` — not called while skipped, uses explicit `dt_s`, so no wall-clock drift internal to the PID. But see the `state_.mode`/`turn_reference_progress_rad_` rows: the *inputs* to this PID on resume (tracking error against a frozen-then-jumped reference) are what actually misbehave, not the PID's own timing. | MEDIUM (via its inputs) |
| `speed_pid_` (unused output, see S8) | `condition_command()` | Frozen; irrelevant to actuation since its output is never sent to the actuator regardless (S8), but its logged `target_acceleration_mps2` will show a discontinuity on resume for the same reason as `conditioned_speed_mps_`. | LOW (logging-only consequence) |

**Cross-reference:** yes — `obstacle/main.cpp:252-258` is exactly the branch
that skips `navigation.update()`, confirmed above in B1.c/B2. This is the
basis for **S2 = CONFIRMED** and **S3 = CONFIRMED** in report 02.

---

## 5. Notes carried forward to report 02

- B4 (PID/Stanley lifecycle), B5 (turn-trigger mechanisms), B6 (speed path),
  B7 (steering-limit ordering), B8 (config override audit), Phase C
  (coupling/dependency audit), and the S1–S14 verdicts are written up in
  `docs/audit/02_coupling_and_conflicts.md`, which depends on the ledger and
  traces above.
