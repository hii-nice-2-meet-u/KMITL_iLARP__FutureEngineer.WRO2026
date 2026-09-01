# Navigation

The decision-making core of the robot. It consumes processed LiDAR (walls,
obstacles) plus the OTOS heading/speed, and produces a `NavigationCommand`
(target speed, steering angle, target acceleration) for the actuator layer.

**Source code:** [`code/modules/navigation`](../../../../code/modules/navigation)
· State machine: [`navigation_controller.cpp`](../../../../code/modules/navigation/navigation_controller.cpp) / [`.hpp`](../../../../code/modules/navigation/navigation_controller.hpp)
· State types: [`navigation_state.hpp`](../../../../code/modules/navigation/navigation_state.hpp)
· Direction search: [`init_direction.cpp`](../../../../code/modules/navigation/init_direction.cpp)
· Learned map: [`track_map.cpp`](../../../../code/modules/navigation/track_map.cpp)

---

## 1. State machine

The controller is a four-state machine (`NavigationMode` in
`navigation_state.hpp`):

```
        ┌──────────────────┐
        │ SEARCH_DIRECTION │  centre in corridor, decide CW vs CCW
        └────────┬─────────┘
                 │ direction confirmed (init_direction)
                 ▼
        ┌──────────────────┐   front wall within trigger distance
        │      NORMAL       │ ─────────────────────────────┐
        │ wall-follow       │◀───────────────┐             │
        └────────┬─────────┘   turn complete │             ▼
                 │                            │      ┌────────────┐
                 │                            └──────│  TURNING   │
                 │                                   │ 90° corner │
                 │  turn_count == total_turns (12)   └────────────┘
                 ▼
        ┌──────────────────┐
        │     FINISHED      │  stop
        └──────────────────┘
```

`NavigationController::update()` dispatches to `update_search_direction()`,
`update_normal()`, or `update_turning()` each tick, then runs the result
through `condition_command()` (output shaping) before returning.

---

## 2. SEARCH_DIRECTION — deciding CW vs CCW

The driving direction is randomised each round, so the robot must infer it.
While unknown, the robot drives slowly (`search_speed_mps = 0.25`) and centres
itself between the side walls (`search_center_kp = 0.8`).

`InitialDirectionEstimator` ([`init_direction.cpp`](../../../../code/modules/navigation/init_direction.cpp))
decides the direction from LiDAR geometry:

- For each side (left = would-turn-LEFT, right = would-turn-RIGHT) it checks
  whether that side wall **continues forward** or is **cut off by a
  perpendicular wall** — the open side is the direction the track turns.
- Evidence is scored across several geometric tests (side-wall validity,
  forward continuation, a connecting perpendicular wall, front-wall support)
  and **confirmed over multiple frames** (`required_confirm_frames = 3`,
  `score_decay = 0.7`) so a single noisy scan cannot flip the decision.
- Once confirmed, the direction is locked and the machine moves to NORMAL.

Key thresholds (`InitialDirectionConfig`): collinearity `≤ 8°` / `0.04 m`,
continuation gap `≤ 0.20 m`, perpendicular tolerance `≤ 15°`, minimum candidate
length `0.15 m`.

---

## 3. NORMAL — wall following

The robot follows the **outer** wall at a target lateral distance
(`target_outer_distance_m = 0.30`). `resolve_track_walls()` maps the LiDAR
left/right walls to inner/outer using the known driving direction.

Steering comes from a **Stanley controller** (see [Control](../control/README.md)):

- `calculate_cross_track_error()` — lateral offset from the target line to the
  outer wall.
- `calculate_wall_heading_error()` — angle between the wall and the robot.

These feed `StanleyController::calculate(cross_track, heading_error, speed, dt)`,
which combines a heading term with a speed-scaled cross-track term.

**Lost-wall fallback.** If the outer wall momentarily disappears (e.g. at a
corner mouth), the controller holds the last valid wall heading for up to
`max_heading_hold_s = 0.30 s`; longer losses fall back to
`lost_wall_speed_mps = 0.25` with zero steering, rather than steering on stale
data.

Cruise speed in NORMAL is `normal_speed_mps = 0.85`.

---

## 4. Turn triggering

`should_start_turn()` fires when the front wall is within an **effective**
trigger distance that grows with speed to compensate for perception + steering
latency:

```
effective_trigger = turn_trigger_distance_m + speed · turn_preview_time_s
                    (base 0.50 m)                    (0.08 s)
```

The trigger must hold for `turn_trigger_confirm_frames = 2` consecutive frames
before the machine commits to TURNING. As the front wall approaches, speed is
ramped down from `approach_speed_mps = 0.72` toward the corner speed.

---

## 5. TURNING — geometric corner trajectory

Rather than snapping to the final 90° heading (which saturates the steering and
cuts the corner), the controller tracks a **moving heading reference** that
sweeps smoothly through the corner:

- **Ackermann feed-forward.** A constant-radius arc (`corner_radius_m = 0.40`,
  `wheelbase_m = 0.18`) gives the nominal steering angle
  `δ_ff = atan(wheelbase / radius)`; the corner radius comes from the WRO outer
  corner geometry (0.10 m wall radius, path 0.30 m inside → ≈0.40 m).
- **Entry/exit blending.** `smoothstep` blends the steering in over
  `turn_entry_blend_rad = 10°` and out over `turn_exit_blend_rad = 22°`, so the
  command has no steps.
- **Heading PID correction.** `turn_heading_pid` corrects the vehicle heading
  onto the moving reference, on top of the feed-forward.
- **Corner speed limit.** `calculate_corner_speed_mps()` caps speed at
  `sqrt(max_lateral_acceleration · radius)` with
  `max_lateral_acceleration_mps2 = 1.40` (raise only after grip testing on the
  real surface). Base turning speed is `turning_speed_mps = 0.65`.

A turn is complete when the heading error is within
`heading_tolerance_rad = 5°` for `heading_confirm_frames = 3` frames; the turn
count increments and the machine returns to NORMAL. After
`total_turns = 12` (three laps × four corners) it goes to FINISHED.

Heading conventions (OTOS): `+heading = CCW`; a CW turn is `−90°`, a CCW turn is
`+90°`; `heading_to_steering_sign = −1` maps heading error to the steering
convention (`− = LEFT`, `+ = RIGHT`).

---

## 6. Command conditioning (output shaping)

`condition_command()` removes jitter and enforces actuator limits before the
command leaves the controller:

- steering low-pass filter (`steering_filter_time_constant_s = 0.035`) and slew
  limit (`max_steering_rate_rad_s = 7.0`);
- speed converted to an acceleration request by `speed_pid`, bounded by
  `max_acceleration_mps2 = 1.8` / `max_deceleration_mps2 = 3.0`;
- `dt` is derived from timestamps and clamped to
  `[min_update_period_s, max_update_period_s] = [0.005, 0.12]` so a stalled
  frame cannot produce a huge integral or derivative kick.

---

## 7. Track map — learn on lap 1, replay on laps 2–3

`TrackMap` ([`track_map.cpp`](../../../../code/modules/navigation/track_map.cpp))
records each corner's entry/exit pose, preferred trigger distance, radius, and
safe speed as **`CornerLandmark`s** (four corners, `TRACK_CORNER_COUNT = 4`),
blended across observations with an EMA (`update_weight = 0.25`).

On later laps, `replay_hint(pose, next_corner_index)` returns a `ReplayHint`
(distance to the next corner entry, preferred trigger/radius/safe speed,
confidence) when confidence ≥ `minimum_replay_confidence = 0.60` and the robot
is within `replay_preview_distance_m = 1.10 m` and roughly aligned
(`replay_max_heading_error_rad = 0.45`). This lets the controller start slowing
and setting up for a corner *before* it can see the wall, which is more stable
than reacting purely live.

The hint is computed in the Open Challenge app loop **before** `update()` and
passed into it in the same tick (see `code/app/_challenge/open/main.cpp`), so
the map preview and the control step stay in sync.

> Traffic-light landmarks (`TrafficLandmark`, `observe_traffic_light`) exist in
> `TrackMap` for the Obstacle Challenge but are **not yet wired into the Open
> Challenge loops**.

---

## 8. Entry points (Open Challenge)

The Open Challenge run program uses the shared controller
([`code/app/_challenge/open`](../../../../code/app/_challenge/open)):

| Program | Behaviour |
|---------|-----------|
| `open_challenge_main` | **Lap 1 learns** the track map, **laps 2–3 replay** it, SPI active |

It wires perception → navigation → actuator (`spi_master`) and logs telemetry.

---

## 9. Decision log (rubric criterion 4)

Visible in the code:

- **Geometric corner + feed-forward instead of "steer to 90°"** — avoids
  steering saturation and corner-cutting; the moving reference keeps the corner
  smooth and repeatable.
- **Speed-scaled turn trigger and lateral-acceleration speed cap** — the corner
  is entered at a physically safe speed instead of a fixed guess.
- **Learn-then-replay track map** — a three-lap event gives few observations, so
  the map is a conservative EMA that lets later laps anticipate corners while
  live wall-following still runs underneath.
- **Command conditioning as a single stage** — one place enforces smoothness and
  actuator limits, so individual state handlers stay simple.
- **Multi-frame confirmation** for both direction search and turn completion —
  robustness against single-scan noise.

The source tree contains controller constants and a deterministic corner
simulation, but it does not contain a real-track measurement log, tuning report,
state-machine image, or an obstacle-challenge controller. These items must be
added from the team's hardware tests before claiming measured performance.
