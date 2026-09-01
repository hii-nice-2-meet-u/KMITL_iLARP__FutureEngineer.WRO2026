# Audit Plan — Drive/Steering Control Path & Tuning Telemetry

> **วิธีใช้ (ภาษาไทย)** — ไฟล์นี้เป็นใบสั่งงานสำหรับ agent ที่จะมาตรวจโค้ด
> ไม่ใช่เอกสารประกอบรีโป งานคือ **ตรวจอย่างเดียว ห้ามแก้โค้ด**
> เป้าหมายสองข้อ: (1) หาว่าอัลกอริทึมในเส้นทางควบคุมการขับเคลื่อนทำงานขัดกันตรงไหน
> และมีส่วนไหนผูกกันแน่นเกินไป (2) หาว่า logging ขาดข้อมูลอะไรที่จำเป็นต่อการจูน
> ผลลัพธ์คือรายงาน `docs/audit/` สามไฟล์ ตามรูปแบบในหัวข้อ Deliverables ท้ายไฟล์นี้
>
> อ่านหัวข้อ **Ground Rules** ให้จบก่อนเริ่มเสมอ

**Repository state this plan was written against:** working tree at commit `4f9f970`
plus uncommitted changes to `navigation_controller.{hpp,cpp}`,
`open_challenge_common.hpp`, `open_challenge_actuator.hpp`, `lidar_processor.cpp`,
`open/main.cpp`, and the untracked directory `code/app/_challenge/obstacle/`.

**Do not `git stash`, `git checkout`, or otherwise disturb the working tree.**
The uncommitted work *is* the subject of the audit.

---

## Ground Rules

1. **Read-only.** Do not edit, format, refactor, or "fix while you're there."
   The only files you create are the three reports in `docs/audit/`.
   If you believe a fix is obvious, write it in the report as a *proposal*, not
   as a patch applied to the tree.
2. **Verify, don't trust.** Section 6 lists suspected defects that were found
   during a fast read. Each one is a **hypothesis to confirm or refute**, not a
   fact. For every one, either confirm it with a quoted code path, or mark it
   `REFUTED` and explain why. A refuted hypothesis is a useful result.
3. **Every finding needs a trace.** A finding is only acceptable if it names
   `file:line`, quotes the relevant lines, and describes a concrete scenario
   (inputs / state → wrong behaviour). "This looks fragile" is not a finding.
4. **Do not run the robot.** There is no hardware attached. You may compile if a
   toolchain is available, but compiling is optional and not the goal.
5. **Stay in scope.** Only the two subsystems below. If you notice a serious
   problem outside scope (e.g. in `camera_processor`), write one line in
   `OUT_OF_SCOPE.md` at the end of the coupling report and move on.
6. **When two pieces of code disagree, say which one you believe is correct and
   why.** If you cannot tell without hardware, say so explicitly and describe
   the one measurement that would settle it. Never guess silently.

---

## 1. Scope

### Subsystem A — drive, steering, and navigation control

| Layer | Files |
|---|---|
| Command producer | `code/modules/navigation/navigation_controller.{hpp,cpp}` |
| Controllers | `code/control/pid.{hpp,cpp}`, `code/control/stanley_controller.{hpp,cpp}` |
| State/contract | `code/modules/navigation/navigation_state.hpp`, `code/common/direction.hpp` |
| Map preview | `code/modules/navigation/track_map.{hpp,cpp}` |
| Direction search | `code/modules/navigation/init_direction.{hpp,cpp}` |
| Obstacle override | `code/app/_challenge/obstacle/obstacle_controller.hpp` |
| Geometry input | `code/modules/lidar/lidar_processor.{hpp,cpp}` (walls + obstacles only) |
| Fusion input | `code/modules/perception/perception.{hpp,cpp}` |
| Loop + config | `code/app/_challenge/open/main.cpp`, `open_challenge_common.hpp`, `code/app/_challenge/obstacle/main.cpp`, `main2.cpp` |
| Actuation | `code/app/_challenge/open/open_challenge_actuator.hpp`, `code/modules/spi/spi_master.{hpp,cpp}` |

### Subsystem B — logging and raw-data capture

| Layer | Files |
|---|---|
| Row schemas | `code/modules/logging/log_types.{hpp,cpp}` |
| Writers | `code/modules/logging/async_csv_writer.{hpp,cpp}`, `telemetry_logger.{hpp,cpp}`, `wall_logger.{hpp,cpp}` |
| Call sites | both `main.cpp` files above |
| Offline tools | `/home/jukkruw/iLARP/plot_run.py`, `/home/jukkruw/iLARP/gen_sample.py` (outside the repo — read only, note what columns they expect) |

### Supporting documents to read first

Read these before touching code. They state the intended conventions, and half
the audit is checking whether the code obeys them.

- `/home/jukkruw/iLARP/mark.txt` — the LiDAR/robot frame convention. Short but load-bearing.
- `/home/jukkruw/iLARP/docs/NAVIGATION_TUNING_GUIDE_TH.txt` — how the team currently tunes.
- `/home/jukkruw/iLARP/tune.txt` — recorded tuning history.
- `docs/navigation/README.md`, `docs/control/README.md`, `docs/lidar/README.md`,
  `docs/logging/README.md`, `docs/perception/README.md`, `docs/spi/README.md`.

**Note:** the docs describe the *committed* code. The working tree has moved on.
Where a doc and the code disagree, that is itself a finding (record it under
"doc drift"), but the code is the ground truth for behaviour.

---

## 2. Phase A — Build the convention ledger (do this first)

Nothing else in this audit is reliable until this table exists. Do not skip it
and do not shorten it.

Create a table with one row per coordinate/sign/unit convention used anywhere in
the scope, with these columns:

| Convention | Defined where | Formula as written in code | Consumers | Agrees with ledger? |
|---|---|---|---|---|

Rows you must fill in at minimum:

1. **LiDAR polar → robot Cartesian** — `LidarProcessor::polar2cartesian`
2. **Robot frame axes** — what `+x` and `+y` mean in `lidar::LineSegment`,
   `ObstacleObject`, and `perception::RobotPoint` (note that `RobotPoint` uses
   named fields `right_m`/`forward_m` while `cv::Point2f` uses `x`/`y` —
   confirm they map the way each consumer assumes)
3. **Wall line angle** — `fit_line_segment`, and what `resolve_track_walls`
   assumes when it classifies left/right/front
4. **Steering sign** — where "negative = LEFT, positive = RIGHT" is established,
   and every place that flips it (`heading_to_steering_sign`,
   `steering_to_servo_sign`, `turn_heading_sign_`)
5. **OTOS heading sign** — is `+heading` CCW? Which code depends on that?
6. **Robot → world transform** — collect *every* implementation. There are at
   least three. Write each formula out and compare them algebraically:
   - `perception::Perception::robot_to_world` (`perception.cpp`)
   - `NavigationController::update_wall_corner_landmark` (`navigation_controller.cpp:~734`, forward transform)
   - the inverse in the same function (world → robot, used for `wall_corner_forward_m`)
   - `obstacle_challenge::ObstacleController::to_robot` (`obstacle_controller.hpp:232`)
7. **Bearing** — `CameraProcessor::calculate_bearing`,
   `ObstacleObject::bearing_rad()`, `Perception::predicted_camera_bearing`
8. **Units** — every config field: which are radians, which are degrees, which
   are metres. Flag any field whose *name* does not state its unit.

**Deliverable for Phase A:** the ledger table, plus an explicit list of every
place two conventions disagree. This list drives Phase B.

---

## 3. Phase B — Control-path audit

Work through these questions in order. Answer each in the report with evidence.

### B1. Trace one command end to end

Pick a single loop iteration in `NORMAL` mode and write the complete chain from
LiDAR scan to SPI byte, naming every function and every transformation of the
steering value and the speed value. Include:

- where the value is produced,
- every clamp, filter, rate limit, and sign flip it passes through,
- the final integer written to the bus.

Do the same for `TURNING` mode and for the obstacle-avoidance path in
`code/app/_challenge/obstacle/main.cpp`. Three traces total.

This trace is the single most valuable artifact of the audit. Write it before
you write any findings.

### B2. Who owns `steering_rad`?

List every writer of the final steering command in one table:
writer / mode(s) active / does it *add to* or *replace* the previous value /
is it downstream of `condition_command()` or upstream.

Then answer: **can two writers be active in the same iteration, and what
happens if they are?** Pay attention to which navigation modes the obstacle
controller is allowed to run in.

### B3. State that goes stale when a branch is skipped

For every piece of member state in `NavigationController` that is updated inside
`update()`, determine what happens if `update()` is **not called** for some
iterations. Build a table:

| Member | Updated in | Effect of skipped iterations | Severity |
|---|---|---|---|

Cover at minimum: `previous_timestamp_us_`, `last_elapsed_update_s_`,
`conditioned_steering_rad_`, `conditioned_speed_mps_`,
`command_conditioner_initialized_`, `wall_corner_*`, `turn_trigger_frames_`,
`lost_wall_timer_s_`, `state_.mode`, and the three PID objects
(`stanley_`, `turn_heading_pid_`, `speed_pid_`).

Then check: **does any code path in either `main.cpp` actually skip
`navigation.update()`?** If yes, cross-reference this table against it.

### B4. PID and Stanley lifecycle

- Where is each of the three PID instances reset? Where is it *not* reset but
  arguably should be? Specifically: crossing `NORMAL → TURNING → NORMAL`,
  losing and reacquiring the outer wall, and entering/leaving obstacle avoidance.
- `StanleyController::calculate()` is declared `const` but holds
  `mutable PID heading_pid_`. Identify every caller and confirm none of them
  assumes the call is side-effect free.
- In `PID::calculate`, verify the anti-windup branch
  (`output == unsaturated_output || signbit(error) != signbit(unsaturated_output - output)`)
  behaves correctly in these four cases: not saturated; saturated high with
  error pushing further high; saturated high with error reversing; `dt_s <= 0`.
- Check the two-argument `PID::calculate(setpoint, current)` overload that uses
  `steady_clock` internally. Is it called anywhere in the control path? If a
  mix of the clock-based and dt-based overloads reaches the same PID instance,
  that is a finding.
- Verify integral limits vs output limits are consistent for each configured
  PID (`stanley.heading_pid`, `turn_heading_pid`, `speed_pid`). Note any case
  where `ki * max_integral` alone can saturate the output.

### B5. Turn trigger — how many independent triggers exist?

There are at least three mechanisms that can start or suppress a turn:
the wall-corner landmark trigger, the front-wall fallback, and the replay
distance gate. Map them:

- Under what exact conditions does each become the active decision-maker?
- Can the replay gate suppress a turn that the wall-corner trigger correctly
  requested? What is the failure consequence if it does?
- Is `turn_trigger_frames_` shared between mechanisms? If the active mechanism
  changes between iterations, does the frame counter carry over inappropriately?
- Trace `state_.turn_armed` and `turn_rearm_distance_m`: after a turn completes,
  what re-arms it, and can the robot get stuck disarmed?

### B6. Speed path

- Follow `target_acceleration_mps2` from `speed_pid_` output to the actuator.
  **Does anything actuate it?** If it terminates unused, say so plainly and
  state what the consequence is for tuning `speed_pid`.
- Compare `max_acceleration_mps2` / `max_deceleration_mps2` in
  `condition_command()` against `speed_pid.min_output` / `max_output`. Are they
  describing the same physical quantity with different limits?
- `to_wheel_rpm()` converts m/s to RPM open-loop using `wheel_diameter_m`.
  Cross-check that constant against `docs/mechanical` and against the
  `wheelbase_m` used by the corner feed-forward. Note that the STM32 runs its
  own closed-loop speed control (`M1_SPD`) — describe how the Pi-side speed PID
  and the MCU-side speed loop interact, and whether they can fight.
- Check `wheel_rpm_override` in the obstacle app: it bypasses `to_wheel_rpm()`
  entirely. Under what conditions is it set, and can it be set while
  `command.target_speed_mps` says something contradictory?

### B7. Steering limits

Collect every steering saturation value in the system into one table and check
for ordering violations (an inner limit larger than an outer one makes the
inner one dead code; an outer limit smaller than the feed-forward makes the
feed-forward permanently saturated):

- `NavigationConfig::stanley.max_steering_rad`
- `NavigationController::clamp_steering()` (note *which* config field it reads)
- `ObstacleConfig::maximum_avoidance_steering_rad`
- `ObstacleConfig::emergency_steering_rad`
- `ActuatorConfig::maximum_steering_command_deg`
- `ActuatorConfig::servo_min_pulse_us` / `servo_center_pulse_us` / `servo_max_pulse_us`
- `ActuatorConfig::maximum_servo_step_us`
- `NavigationConfig::max_steering_rate_rad_s`
- the corner feed-forward magnitude `atan2(wheelbase_m, corner_radius_m)`

Compute the feed-forward magnitude with the **currently configured** values and
state whether it fits inside the clamp.

### B8. Configuration override audit

`NavigationConfig` declares defaults; `make_navigation_config()` overrides many
of them; `obstacle/main.cpp` overrides some again. Produce a three-column table
(default / open-challenge value / obstacle-challenge value) for every field that
is overridden anywhere, and flag:

- fields whose default is never used (dead default),
- fields overridden in one app but not the other where the difference looks
  unintentional,
- **unit mismatches**: any override written as a bare number where the default
  was written as `deg * PI / 180`. Check every single one.

---

## 4. Phase C — Coupling and dependency audit

### C1. Dependency map

Produce a module-level dependency graph (text or mermaid) from the
`CMakeLists.txt` files plus the actual `#include`s. Then answer:

- Which module has the most inbound dependencies? Is that justified?
- Are there dependencies that only exist to borrow a type? (For example,
  `perception` includes `track_map.hpp` — find out what it actually needs from
  it, and whether a shared types header would break the cycle risk.)
- Does any *processor* (which the architecture doc says must be a pure function)
  depend on a *module* (stateful)? List violations.

### C2. Over-coupling checklist

For each item, state COUPLED / ACCEPTABLE / DECOUPLED with evidence:

1. `NavigationController::clamp_steering()` reads `config_.stanley.max_steering_rad`,
   but is used by search mode, turning mode, and command conditioning — none of
   which involve Stanley. Is the Stanley config effectively acting as a global
   steering limit? What breaks if someone tunes Stanley?
2. `NavigationDebug` is both the diagnostic output *and* an in/out parameter
   that `should_start_turn()` and `update_wall_corner_landmark()` write control
   state into (`wall_corner_confirmed`, `effective_turn_trigger_m`), which
   `update_normal()` then *reads back* to pick a speed. Trace this. Is control
   flow passing through a struct named "Debug"? What happens if a future change
   stops populating a debug field?
3. The application `main.cpp` files own the state machine's outer bookkeeping
   (lap/corner recording into `TrackMap`, launch boost, mode-change events).
   How much logic lives in `main.cpp` that arguably belongs in the controller?
   Quantify: how many behavioural branches are in each `main.cpp`?
4. `open_challenge_actuator.hpp` and `open_challenge_common.hpp` are included by
   the obstacle app. Is the naming now wrong, and more importantly: does the
   obstacle app inherit open-challenge tuning it does not want?
5. `TrackMap` is written by `main.cpp`, read by `NavigationController` via
   `ReplayHint`, and now also written by the obstacle app's traffic-light
   observations. Three owners. Is there any ordering hazard within one iteration?
6. `perception::PerceptionConfig::lidar_mount.forward_m` and
   `NavigationConfig::lidar_forward_offset_m` both describe the same physical
   mounting distance. Confirm whether they are the same value and whether
   changing the robot would require editing both.

### C3. Duplicated logic

Find every place the same computation is implemented more than once and list
them with both locations. Known starting points — confirm and extend:

- angle normalisation (`normalize_angle`) — count the implementations
- angle difference / collinearity checks (`init_direction.cpp` vs
  `navigation_controller::has_forward_wall_continuation`)
- "same segment" endpoint comparison
- world↔robot transforms (see Phase A item 6)
- `navigation_mode_name` vs `open_challenge::mode_name`
- launch-boost logic in the two `main.cpp` files (they differ — say how)

For each, state whether divergence between the copies is currently harmful.

---

## 5. Phase D — Logging and raw-data audit

The purpose of this phase is a single question:
**after a failed run, can the team explain what happened and re-tune from the
logs alone, without re-running the robot?**

### D1. Column coverage matrix

Build a matrix: rows = every tunable parameter in `NavigationConfig`,
`StanleyConfig`, `PIDConfig` (x3), and `ObstacleConfig`. Columns:

| Parameter | Is its *effect* observable in telemetry? | Which column(s) | If not, what's missing |
|---|---|---|---|

A parameter is "observable" only if the log contains enough to see the parameter
doing its job. For example, tuning `stanley.k` requires seeing the cross-track
term and the heading term **separately**; a single combined `raw_steering_rad`
is not sufficient. Apply that standard strictly.

### D2. Blind spots to check explicitly

For each, state PRESENT / MISSING / PARTIAL and name the column if present:

1. Obstacle-avoidance state — active flag, colour, pass side, relative
   forward/right, target offset, avoidance steering, confidence, and *why* it
   activated or released
2. Perception fusion diagnostics — the counters in `PerceptionDiagnostics`
   (currently printed to console every 250 ms; is any of it in CSV?)
3. Stanley internals — cross-track term, heading term, PID integral value
4. Turn-heading PID internals during a corner
5. Speed PID internals — error, integral, output
6. The commanded vs achieved servo pulse (is `limit_servo_pulse_step` clipping,
   and can you tell from the log?)
7. Loop timing — actual iteration period, and time spent in LiDAR processing
   vs camera processing. Note that `update_dt_s` is the *clamped* value; is the
   raw value recoverable?
8. Raw LiDAR scan — is any raw or semi-raw scan persisted? Could a run be
   re-processed offline with different `LidarProcessor` parameters?
9. All fitted line segments (not only the three resolved walls) — needed to
   debug why a wall was mis-classified
10. Camera frames — the obstacle app writes annotated JPEGs at most every
    500 ms and only when an object is visible. Is that enough to debug a missed
    detection? Are the raw frames recoverable?
11. Battery voltage during the run (currently read only at start and end)
12. Which turn-trigger mechanism fired (the `turn_source` string exists in
    `open/main.cpp` console output — is it in the CSV?)

### D3. Data integrity

- `AsyncCsvWriter` drops rows when the queue exceeds `maximum_queued_rows`
  (default 400). Where exactly does the drop happen — oldest or newest? Does the
  CSV contain any marker showing a gap, or does the row sequence silently
  become non-contiguous?
- Are dropped-row counts reported per-writer at exit? Which writers are checked
  and which are not?
- What happens to the last second of data if the process is killed with
  `SIGKILL` rather than `SIGINT`?
- `to_csv_row` writes `precision(6)` fixed. Is that enough for
  `timestamp_us`-derived values and for small angles? Check whether any field
  loses meaningful resolution.
- Is `mode` written as a bare string that could contain a comma? Confirm no
  field can break CSV parsing.

### D4. Offline tooling fit

Read `/home/jukkruw/iLARP/plot_run.py`. List the columns it consumes and check
each still exists with the same name in `telemetry_csv_header()`. Report any
mismatch. Then state which of the D2 blind spots would need a new plot to be
useful, so the logging work and the plotting work can be planned together.

### D5. Proposal

Close the logging report with a concrete proposed schema change:

- new columns to add to `TelemetryRow`, grouped by which blind spot they close
- any new CSV file that should exist (e.g. a per-scan segments file, a raw scan
  file) with its proposed header
- estimated per-row byte cost and rows/second at the current loop rate, so the
  team can judge the storage and queue impact

Do not implement it. Propose it.

---

## 6. Pre-identified suspects — confirm or refute each

These were spotted during a fast read and are **unverified**. For each: state
`CONFIRMED` / `REFUTED` / `NEEDS HARDWARE`, quote the code, and describe the
concrete consequence. If confirmed, rate severity as
`CRITICAL` (can crash the robot into something) / `HIGH` (silently wrong control)
/ `MEDIUM` (makes tuning unreliable) / `LOW` (hygiene).

**S1 — Unit mismatch in `exit_acceleration_blend_rad`.**
`open_challenge_common.hpp:95` sets `config.exit_acceleration_blend_rad = 20.0f;`
while the default at `navigation_controller.hpp:98` is
`15.0f * PI / 180.0f` and every neighbouring blend value in the same function is
written as `deg * PI / 180.0f`. Determine whether 20.0 is intended as radians.
Then compute what `exit_acceleration_weight` evaluates to at corner entry
(`navigation_controller.cpp:443`) under both readings, and state how the corner
speed profile differs.

**S2 — Navigation `update()` is skipped during obstacle avoidance.**
`obstacle/main.cpp:254-257` selects `result.command = priority_command;` in an
`if` branch whose `else` is the only call to `navigation.update()`. Cross-check
against your Phase B3 stale-state table and enumerate every consequence.
Pay particular attention to: `condition_command()` never running (so steering
rate limiting and the low-pass filter are bypassed), `dt` accounting, and
whether the navigation state machine can miss a corner while avoiding.

**S3 — `result.debug` is all zeros during avoidance.**
In the same branch, `result` is default-constructed. Trace what
`make_telemetry_row()` writes for those iterations. Consequence for tuning the
avoidance behaviour itself.

**S4 — Two different world-frame conventions read the same OTOS pose.**
Compare `perception::robot_to_world` (`perception.cpp`) against the forward
transform at `navigation_controller.cpp:~734`. Write both out algebraically.
They appear to differ by a 90° rotation. Then check
`obstacle_controller.hpp:232 to_robot()` — determine which of the two it is
consistent with. State which convention matches the physical OTOS output, or
mark `NEEDS HARDWARE` and name the single test that would settle it.

**S5 — Obstacle controller replaces steering instead of biasing it.**
`obstacle_controller.hpp` `apply()` assigns `command.steering_rad = ...`
unconditionally when active. Determine whether any wall-following influence
survives during avoidance, and what prevents the robot from steering into the
outer wall while passing an obstacle.

**S6 — Obstacle controller runs during `TURNING`.**
`apply()` returns early only for `FINISHED`. Determine what happens if an
obstacle is confirmed mid-corner and the avoidance steering replaces the corner
feed-forward.

**S7 — Emergency steering ignores the computed geometry.**
In the emergency branch, `command.steering_rad = side * emergency_steering_rad`
uses full lock toward the pass side and discards `target_right_m`. Check the
sign convention against Phase A item 4 and state whether full lock is toward the
correct side. This one is `CRITICAL` if the sign is wrong.

**S8 — `target_acceleration_mps2` is computed but never actuated.**
`speed_pid_` output flows into `NavigationCommand::target_acceleration_mps2`
(`navigation_controller.cpp:1157`), but `ActuatorOutput::apply()` uses only
`command.target_speed_mps` via `to_wheel_rpm()`. Confirm the value reaches no
SPI command, and state the implication for anyone tuning `speed_pid`.

**S9 — Corner feed-forward may exceed the steering clamp.**
With the current `wheelbase_m` and `corner_radius_m`, compute
`atan2(wheelbase_m, corner_radius_m)` in degrees and compare against
`stanley.max_steering_rad` (which `clamp_steering()` uses). If the feed-forward
alone saturates, the turn-heading PID has no authority left. Confirm.

**S10 — Replay turn gate can suppress a legitimate turn.**
`should_start_turn()` (the new block around `navigation_controller.cpp:857`)
forces `trigger_condition = false` when the replay hint says the corner is still
far, unless the front wall is within
`replay_front_safety_override_distance_m`. Determine what happens if the learned
map is wrong (bad OTOS drift on lap 1) — can this block a turn until the front
wall is 0.25 m away, and is that recoverable at the configured speed?

**S11 — `map_preview_valid` written in two places.**
`update_normal()` now sets `debug.map_preview_valid` near the top *and* in the
later replay block. Confirm whether the two writes can disagree and whether the
logged value means what the column name says.

**S12 — `search_preserve_initial_offset` captures a one-shot bias.**
`calculate_search_steering()` latches the first measured centring error and
subtracts it forever after. Determine when the latch is reset, and what happens
if the first frame is taken while the robot is mid-slide or sees a bad wall fit.

**S13 — Stanley integral survives a corner.**
`start_turn()` resets `turn_heading_pid_` but does not reset `stanley_`. Confirm
and state whether the integral accumulated before a corner is still applied on
the following straight.

**S14 — `stanley_.reset()` on every wall-loss frame.**
`update_normal()` calls `stanley_.reset()` each iteration the outer wall is
missing. If the wall flickers, is the integral being repeatedly destroyed?
Determine the practical effect on wall-following stability.

---

## 7. Deliverables

Create the directory `docs/audit/` and write exactly these three files.
Use plain Markdown. Do not add a fourth file.

### `docs/audit/01_conventions_and_control_path.md`

1. The Phase A convention ledger (full table)
2. The list of convention disagreements
3. The three end-to-end traces from B1
4. The steering-ownership table from B2
5. The stale-state table from B3

### `docs/audit/02_coupling_and_conflicts.md`

1. Findings, one section per finding, sorted **most severe first**, each in this
   exact format:

   ```
   ### F-nn — <one-line title>

   - **Severity:** CRITICAL | HIGH | MEDIUM | LOW
   - **Status:** CONFIRMED | REFUTED | NEEDS HARDWARE
   - **Location:** file:line (plus any secondary locations)
   - **What the code does:** <quote or paraphrase with the quoted lines>
   - **Failure scenario:** <concrete inputs/state → wrong output>
   - **Why it conflicts:** <which other algorithm it fights, and how>
   - **Proposed direction:** <one paragraph, no patch>
   ```

2. The S1–S14 verdict table (id / status / severity / one-line result) so the
   reader can see at a glance which hypotheses survived
3. The C1 dependency map and the C2 coupling checklist
4. The C3 duplicated-logic list
5. `OUT_OF_SCOPE` — one line per out-of-scope issue noticed

### `docs/audit/03_logging_for_tuning.md`

1. The D1 coverage matrix
2. The D2 blind-spot list with PRESENT/MISSING/PARTIAL
3. The D3 data-integrity findings
4. The D4 tooling-fit report
5. The D5 proposed schema change

---

## 8. Working order

Do not parallelise. Each phase depends on the one before it.

1. Read the supporting documents in §1.
2. Phase A — convention ledger. **Stop and write it down before continuing.**
3. Phase B — control path, B1 first.
4. Phase C — coupling.
5. Section 6 — work through S1–S14 with everything you now know.
6. Phase D — logging.
7. Write the three reports.

Budget roughly: 30 % of effort on Phase A + B1, since every later finding
depends on them being right.

## 9. Explicit non-goals

- Do not implement obstacle avoidance improvements.
- Do not implement parallel parking.
- Do not add logging columns.
- Do not change tuning values.
- Do not reformat or apply `.clang-format`.
- Do not touch `code/external/`.
- Do not evaluate the camera colour thresholds or the mechanical design.
- Do not write anything to the WRO documentation (`README.md`, `docs/*/README.md`).
