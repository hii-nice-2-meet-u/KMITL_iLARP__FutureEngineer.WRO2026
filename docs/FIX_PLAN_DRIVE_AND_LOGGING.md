# Fix Plan — Drive/Steering Control Path & Tuning Telemetry

> **วิธีใช้ (ภาษาไทย)** — ไฟล์นี้เป็นใบสั่งงาน **แก้โค้ด** ต่อจากรายงานตรวจสอบใน `docs/audit/`
> จัดลำดับตามการพึ่งพากัน ไม่ใช่ตามความรุนแรง เพราะบางอย่างต้องแก้ก่อนถึงจะทดสอบอย่างอื่นได้
>
> **สิ่งที่ต้องอ่านก่อนเริ่ม:** `docs/audit/01`, `02`, `03` ทั้งสามไฟล์
> **สิ่งที่ต้องทำก่อนเริ่ม:** ตอบคำถามในหัวข้อ §2 (Blocking decisions) ให้ครบ — มี 6 ข้อที่ agent
> ตัดสินใจเองไม่ได้ ต้องให้คนในทีมตอบ
>
> ลำดับหลักคือ: **แก้บั๊กหน่วย/เรขาคณิตของโค้ง → เพิ่ม logging ให้มองเห็น → แก้สถาปัตยกรรมการหลบกล่อง → เก็บกวาด**
> เหตุผลที่การหลบกล่อง (งานที่ทีมกำลังทำอยู่) ไม่ได้อยู่ลำดับแรก อธิบายไว้ใน §3

**Source of truth:** `docs/audit/02_coupling_and_conflicts.md` finding IDs (F-01…F-18).
Every task below cites the finding it closes. If a task and the audit disagree,
the audit wins — re-read it before improvising.

---

## 1. Ground rules

1. **One task = one commit.** Never bundle two task IDs in one commit. Commit
   message format: `fix(nav): T-03 clamp corner_radius_m to a feasible arc (F-04)`.
2. **Do not fix anything not listed here.** The audit found 18 findings; this
   plan deliberately defers some. If you spot something new, append it to
   `docs/audit/NEW_FINDINGS.md` and keep going.
3. **Behaviour-changing tasks and refactors never share a commit.** Stage 6 is
   pure refactor — it must not alter a single computed value.
4. **After every task, state what you verified and how.** "Compiles" is not
   verification for a behavioural change; see §8.
5. **Never silently change a tuning value as a side effect.** If a fix forces a
   tuning value to move (T-03 does), say so explicitly in the commit body and
   record the old value.
6. **All six decisions in §2 are answered — every task is unblocked.** Use the
   exact values given there; do not substitute your own judgement for a decision
   the team has already made. The only thing still pending is a hardware
   measurement (§2, "Still open"), which gates nothing in this plan.

---

## 2. Decisions — ANSWERED, no longer blocking

All six were answered by the team. Every task below is unblocked. Treat these as
given; do not re-open them.

| # | Decision | Consequence for the tasks |
|---|---|---|
| D1 | `wheelbase_m = 0.16375 m` is confirmed correct. | See the **still-open** note below — the question about the *physical steering angle* was not answered, but with D2's value it no longer blocks anything. |
| D2 | **`corner_radius_m = 0.45`** | T-03. Feed-forward becomes 20.0°, well inside the 38° clamp with 18° of headroom. See §3. |
| D3 | **`confirmation_frames = 1` is deliberate** — detection arrives late and often lasts only a single frame. | T-14 keeps it at 1 and records the reason. Because temporal filtering is unavailable, a *spatial* gate replaces it — see T-14. |
| D4 | Field is a small square inside a large square. Traffic pillars sit on the lines **400 mm or 600 mm** from the inner square's edge (corridor is ~1 m wide). | T-11 gates avoidance to `NORMAL` + `SEARCH_DIRECTION`; `TURNING` keeps sole authority over steering. See §3.1 for the handover rule. |
| D5 | **Delete the Pi-side `speed_pid`.** The STM32 already ramps with its own PID. | T-16 becomes a deletion, plus a documentation correction. |
| D6 | Left to judgement. **Chosen: a confirmed wall-corner trigger bypasses the replay gate.** | T-15. Rationale in §3.2. |

> **Still open, not blocking:** D1 asked whether the steering rack physically
> reaches 45°; the answer given was the wheelbase instead. This no longer gates
> any task, but it is still worth measuring, because
> `ActuatorConfig::maximum_steering_command_deg = 45` is what maps a steering
> angle onto the servo pulse range. If the linkage actually binds at, say, 35°,
> then **every** steering command in every mode is scaled wrong — a 20°
> feed-forward would produce noticeably less than 20° at the wheel. Add this to
> `docs/audit/HARDWARE_CHECKS.md` alongside the F-07/F-08 tests (§4).

---

## 3. Why corner geometry comes before the obstacle work

The team's current focus is obstacle avoidance, but two corner bugs (F-01, F-04)
corrupt **every corner of every run**. Until they are fixed, an avoidance test
that ends in a crash cannot be attributed — the corner may have been the cause.
Fix the corner first; it is two config lines.

### The `corner_radius_m` problem, computed

`corner_radius_m` currently feeds **three** independent things:

| Consumer | Formula | At `R = 0.12` (current) |
|---|---|---|
| Steering feed-forward | `atan2(wheelbase_m, R)` | **53.8°** |
| Corner speed cap | `sqrt(max_lateral_acceleration_mps2 · R)` | 0.245 m/s |
| Geometric turn trigger | `R + wall_corner_to_path_offset_m + v·turn_preview_time_s` | 0.185 m at 0.45 m/s |

With `wheelbase_m = 0.16375`, the vehicle's **minimum achievable turning radius** is:

- at the 38° steering clamp: `0.16375 / tan(38°)` = **0.210 m**
- at the 45° actuator limit: `0.16375 / tan(45°)` = **0.164 m**

**`corner_radius_m = 0.12 m` is below both.** The configuration asks the car to
drive an arc it cannot physically drive at any steering angle. This is the root
cause of F-04 (feed-forward 53.8° pinned against a 38° clamp) — not a clamp that
is too tight, but a radius that is impossible.

Options considered for **D2**, computed with `max_lateral_acceleration_mps2 = 0.50`:

| `corner_radius_m` | Feed-forward | Lateral-limited speed | Turn trigger @0.45 m/s | Note |
|---|---|---|---|---|
| 0.12 (current) | 53.8° | 0.245 | 0.185 m | **Physically impossible** |
| 0.21 | 37.9° | 0.324 | 0.275 m | Exactly at the 38° clamp — no PID headroom |
| 0.30 | 28.6° | 0.387 | 0.365 m | ~9° of headroom |
| **0.45 — CHOSEN (D2)** | **20.0°** | **0.474** | **0.515 m** | 18° of headroom; comfortably inside the physical limit |

### 3.1 What `corner_radius_m = 0.45` changes

| Quantity | Before (0.12) | After (0.45) |
|---|---|---|
| Feed-forward magnitude | 53.8° — **pinned against the 38° clamp** | 20.0° — 18° of headroom for `turn_heading_pid` |
| Headroom left for the tracking PID | 0° (saturated) | 18° |
| Lateral-accel speed limit | 0.245 m/s | 0.474 m/s |
| **Effective corner speed** | 0.245 (lateral-accel bound) | **0.280 (now bound by `turning_speed_mps`)** |
| Geometric turn trigger @0.28 m/s | 0.185 m | **0.498 m** |
| Geometric turn trigger @0.45 m/s | 0.185 m | **0.515 m** |

Three consequences the agent must carry forward:

1. **The turn now triggers ~2.7× further from the corner** (0.185 → ~0.50 m).
   This is the largest single behavioural change in the whole plan. Confirm
   `wall_corner_max_forward_m = 1.50` still covers it (it does) and that the
   inner-wall corner is reliably detected at 0.5 m — that is a *detection*
   question, not a geometry one, and it is the first thing to check in the
   Stage 1 test session.
2. **`max_lateral_acceleration_mps2` stops being the binding constraint.**
   `turning_speed_mps = 0.28` now sets corner speed, not `sqrt(a·R) = 0.474`.
   If grip allows, `turning_speed_mps` can rise as far as ~0.47 before the
   lateral limit binds again. Do **not** raise it as part of this plan — note
   it in `FIX_LOG.md` as available headroom for a later tuning session.
3. **`approach_distance_m = 0.90` is now only 0.39 m ahead of the trigger**
   (was 0.72 m). The approach speed ramp has much less room to work. If the
   Stage 1 test shows the robot arriving at the corner still too fast, raise
   `approach_distance_m` rather than lowering `corner_radius_m` back.

### 3.2 Rationale for the two judgement calls

**D4 — avoidance is blocked during `TURNING`.** With pillars on the 400/600 mm
lines and a trigger now firing ~0.5 m before the corner, an obstacle and a
corner *can* overlap in time. Two designs were possible:

- *(chosen)* `TURNING` keeps sole authority over steering; avoidance runs in
  `NORMAL` and `SEARCH_DIRECTION` only. Simple, and it preserves the corner
  trajectory design exactly as documented.
- *(deferred)* Let avoidance add a small bounded bias (±8°) **on top of** the
  corner feed-forward instead of replacing it. More elegant, harder to tune.

The chosen option needs one handover rule so the robot does not enter a corner
with an un-passed obstacle in front of it — see T-11b. Be aware this is the same
shape as F-10 (a gate suppressing a legitimate turn), so it is deliberately
capped tight. If the field test shows it misbehaving, switch to the deferred
bias design rather than widening the cap.

**D6 — a confirmed wall-corner bypasses the replay gate.** A confirmed
wall-corner landmark is *live* geometric evidence; the replay hint is a *learned*
estimate that inherits lap-1 OTOS drift. Letting stale learned data veto live
evidence is what makes F-10 dangerous. Widening
`replay_front_safety_override_distance_m` would only shrink the window, not fix
the inversion of authority.

> **Warning to carry into testing:** T-02 and T-03 together will make cornering
> visibly slower, wider, and *earlier* than it is today. That is the *point* —
> the current behaviour is fast because two bugs cancel a safety cap. Treat
> every tuning value that was set to compensate for the old behaviour as
> suspect, especially `turn_entry_blend_rad`, `heading_tolerance_rad`, and
> `approach_distance_m`.

---

## 4. Do NOT touch — hardware verification required first

These two are **CONFIRMED disagreements** but the audit could not determine which
side is physically correct. Both are currently self-consistent, so neither is
actively breaking the robot. **Changing them from source reading alone is more
likely to break a working robot than to fix it.**

| Finding | What is wrong | The one test that settles it |
|---|---|---|
| **F-07** — `polar2cartesian` is 180° from `mark.txt` and `docs/lidar/README.md` | `x = -d·sin(θ)`, `y = -d·cos(θ)` vs the documented un-negated form | Point the RPLIDAR's raw-angle-zero reference mark at a known wall, run `test_lidar`, and see whether that wall draws in front of (+Y) or behind (−Y) the robot origin |
| **F-08** — `update_wall_corner_landmark`'s world transform is 90° from `perception::robot_to_world` | Two conventions read the same OTOS pose; `obstacle_controller::to_robot` agrees with `perception`, so nav is the outlier | Drive straight with heading held near 0 and confirm which world axis the OTOS-reported position advances along |

**Action for now:** create `docs/audit/HARDWARE_CHECKS.md` containing exactly
these two tests written as a step-by-step procedure a student can run in ten
minutes, with a blank result line to fill in. That is the deliverable — not a
code change.

When the results come back, F-08 becomes a real task (rewrite the nav transform
to match `perception`), because the moment anyone feeds `wall_corner_filtered_world_`
into `TrackMap` or `ObstacleController` the 90° error becomes live.

---

## 5. Stage plan

### Stage 0 — Safety net (do first, no exceptions)

| ID | Task | Detail |
|---|---|---|
| T-00 | **Commit the current working tree as a baseline** | `code/app/_challenge/obstacle/` is **untracked** and 8 files are modified. Commit them as-is (`feat(obstacle): baseline before audit fixes`) so every later change is diffable and revertible. Do not "clean up" while committing. |
| T-01 | **Tag the baseline** | `git tag pre-audit-fixes`. If a fix session goes wrong, this is the way back. |

### Stage 1 — Corner geometry and unit bugs

| ID | Fixes | Files | Change | Verify |
|---|---|---|---|---|
| T-02 | F-01 | `open_challenge_common.hpp:95` | `exit_acceleration_blend_rad = 20.0f` → `20.0f * PI / 180.0f` | Expected `exit_acceleration_weight` after the fix: 90°→0.000, 45°→0.000, 20°→0.000, 10°→0.500, 5°→0.844. Reproduce these five numbers and put them in the commit body. |
| T-03 | F-04 | `open_challenge_common.hpp:84` | `corner_radius_m = 0.12f` → **`0.45f`** (D2) | Reproduce §3.1's table: feed-forward 20.0°, lateral limit 0.474 m/s, effective corner speed 0.280 m/s, trigger 0.498–0.515 m. Record all of them. **Do not** touch `stanley.max_steering_rad` — 38° is now correct with 18° of margin. |
| T-04 | F-16 | `navigation_controller.hpp:105-106` | Narrow `turn_heading_pid`'s `min_output`/`max_output` from ±45° to **±15°**. | Show `20.0° (feed-forward) + 15° (PID) = 35° < 38° (clamp)`. The PID can now always use its full authority without the composite clamp binding first — which was the whole point of F-16. |

**Stop here and run one Open Challenge test session before continuing.**
Cornering behaviour has changed materially; validate it in isolation, while the
logging is still the version you already know how to read.

### Stage 2 — Logging, so the rest is verifiable

Do this before the obstacle rework. Every Stage 3+ task is validated from logs,
and right now the avoidance path logs nothing (D2 #1) and blanks every other
column while active (S3/F-03).

| ID | Fixes | Change |
|---|---|---|
| T-05 | D4 | **Fix `plot_run.py` first.** It references `pos_x`, `pos_y`, `speed_mps`; the real columns are `pos_x_m`, `pos_y_m`, `measured_speed_mps`. It would `KeyError` on every real log today. Also make the missing-`obstacles.csv` path explicit rather than a bare `except Exception`. |
| T-06 | D2 #3, #4 | Add accessors to `control::PID` (`integral()`, `last_output()`) and to `StanleyController` (`last_cross_track_term()`, `last_heading_term()`). Pure additions — no behaviour change. |
| T-07 | D2 #3, #4 | Add the PID-internals columns from audit §D5 to `TelemetryRow` + `telemetry_csv_header()` + `make_telemetry_row()`: `stanley_cross_track_term_rad`, `stanley_heading_term_rad`, `stanley_heading_integral`, `turn_heading_pid_output_rad`, `turn_heading_pid_integral`. **Skip the `speed_pid_*` columns the audit proposed** — D5 deletes that controller (T-16), so adding them would create work that T-16 immediately reverses. **Keep header and row-writer field order identical** — they are two hand-maintained lists in `log_types.cpp` and a mismatch silently shifts every column. |
| T-08 | D2 #7, #12, D1 | Add `raw_update_dt_s` (pre-clamp), `turn_trigger_frames`, `turn_armed`, `turn_source`, `replay_gate_suppressed`. These are the columns that would have made F-01 and F-10 visible from a single run. |
| T-09 | D2 #1, #2 | Add the `obstacle_*` and perception-diagnostics columns. Populate from `ObstacleStatus` and `PerceptionDiagnostics` at the `obstacle/main.cpp` call site, **unconditionally every tick**, independent of which branch produced the command. |
| T-10 | D3 | Add a monotonic `row_index` to `TelemetryRow` so a dropped row is detectable as a gap, and make `obstacle/main.cpp` report `dropped_row_count()` at exit the way `open/main.cpp` already does. |

> Deliberately **deferred**: `segments.csv` and raw-scan capture (audit D5). Raw
> scan capture is ~272 KB/s even in binary and needs a storage-bandwidth
> decision first. Revisit after Stage 4.

### Stage 3 — Obstacle avoidance architecture (the main event)

This is one coherent redesign, not five patches. Read F-02, F-03, F-05, F-06
together before writing anything.

**Target shape:** `navigation.update()` runs **every tick, unconditionally**
(as `open/main.cpp` already does). The obstacle controller becomes an
*override applied to the navigation result*, not a replacement for the call.
The overridden command then still flows through the navigation controller's
output conditioning.

| ID | Fixes | Change |
|---|---|---|
| T-11 | F-02, F-05 | Restructure `obstacle/main.cpp:252-258`: call `navigation.update(...)` **unconditionally**, then apply the obstacle override to `result.command`. Per **D4**, apply the override only when `state.mode` is `NORMAL` or `SEARCH_DIRECTION`; in `TURNING` the corner keeps sole authority. This alone unfreezes `dt`, the corner state machine, and the wall-corner tracker. |
| T-11b | D4 handover | Add one rule so the robot does not enter a corner with an un-passed obstacle ahead: while avoidance is active **and** `relative.forward_m > 0`, suppress the *start* of a new turn. **Cap it hard** — the suppression must lift once the front wall is inside `front_wall_fallback_distance_m`, reusing that existing path rather than adding a third independent gate. This is deliberately the same shape as F-10, so keep the cap tight and log it (T-08's `replay_gate_suppressed` column should get a sibling, `obstacle_turn_suppressed`). |
| T-12 | F-03 | Ensure the overridden steering still passes through output shaping. Two acceptable designs — pick one and say why: **(a)** expose the conditioning stage so the app can call it after the override, or **(b)** pass an optional steering override *into* `NavigationController::update()` so conditioning happens in its existing place. **(b) is cleaner** and keeps `condition_command()` private, but touches the controller's signature. |
| T-13 | F-06 | Bound `target_right_m` against the live outer wall. `PerceptionData::track_walls` is already a parameter of `apply()` — use it. The avoidance path must never request a line closer to the outer wall than `target_outer_distance_m`. **This task is now load-bearing** — see T-14. |
| T-14 | F-17 | Per **D3**, `confirmation_frames` **stays at 1** — detection is late and often single-frame. Add a comment at `obstacle/main.cpp:67` recording that reason so it is not "corrected" later. Because temporal confirmation is unavailable, replace it with a **spatial** gate that costs zero frames: reject any observation whose position falls outside the corridor implied by `PerceptionData::track_walls` (a pillar cannot be inside a wall or beyond the far wall). Combined with T-13 this is what keeps a single false positive from throwing the robot at a wall. |

**Verification for Stage 3** (all from logs, now that Stage 2 exists):

- `update_dt_s` stays in its normal band across an avoidance event — no spike on release
- `obstacle_active` transitions align with `mode` continuing to advance (not freezing)
- `raw_steering_rad` vs `steering_rad` shows the filter/slew limit still acting during avoidance
- no telemetry row with `obstacle_active=1` has all-zero navigation columns

### Stage 4 — Controller lifecycle

| ID | Fixes | Change |
|---|---|---|
| T-15 | F-10 | Per **D6**: a **confirmed wall-corner trigger bypasses the replay gate entirely**. In `should_start_turn()`, apply the replay suppression only when the trigger came from the front-wall fallback, never when `wall_corner_confirmed` is true. Live geometry outranks a learned estimate. |
| T-16 | F-09 | Per **D5**: **delete it.** Remove `speed_pid` from `NavigationConfig`, remove the `speed_pid_` member and its calls in `condition_command()`, and drop `target_acceleration_mps2` from `NavigationCommand`, `NavigationDebug`, and `TelemetryRow`/`telemetry_csv_header()`. Then **update `tune.txt` and `docs/control/README.md`** — both currently instruct the team to tune a gain that reaches no actuator. Leaving stale tuning guidance in place is worse than the dead code itself. Note in the commit that speed ramping is now owned entirely by `condition_command()`'s `max_acceleration_mps2`/`max_deceleration_mps2` on the Pi side plus the STM32's own PID. |
| T-17 | F-13 | Add `stanley_.reset()` to `start_turn()`, matching `turn_heading_pid_`'s treatment. |
| T-18 | F-14 | Track `bool outer_wall_was_valid_` and reset `stanley_` only on the `true → false` transition, not every frame the wall is missing. |
| T-19 | F-15 | Require a settle window before latching `search_initial_center_error_m_` — discard the first N frames after `reset()`, or average the first few valid frames. |
| T-20 | F-18 | Delete the duplicated `map_preview_valid` write block (`navigation_controller.cpp:182-186`). Confirmed harmless today, so this is hygiene only. |

### Stage 5 — Decoupling (behaviour-preserving; do last)

Every task here must produce **byte-identical telemetry** on a replayed input.
If any computed value changes, you have made an error.

| ID | Fixes | Change |
|---|---|---|
| T-21 | F-11 | Add `NavigationConfig::max_steering_rad` and have `clamp_steering()` read it instead of `stanley.max_steering_rad`. Initialise it to the current effective value so behaviour is unchanged. |
| T-22 | F-12 | Split control state out of `NavigationDebug`. Introduce a `TurnTriggerState` threaded explicitly through `update_normal()`; populate the diagnostics struct once at the end of `update()`. |
| T-23 | C1, C2 #6 | Extract `MapPose`, `TrafficColor`, `PassSide` from `track_map.hpp` into a light `navigation_types.hpp` so `perception` stops depending on the full `TrackMap`. Make `lidar_forward_offset_m` and `perception_config.lidar_mount.forward_m` read one shared constant. |
| T-24 | C2 #4, C3 | Rename `open_challenge_common.hpp` / `open_challenge_actuator.hpp` to challenge-neutral names, and give the obstacle app an explicit config function instead of silently inheriting open-challenge tuning. Reconcile the two divergent launch-boost implementations (0.7 s vs 0.3 s) or document why they differ. |
| T-25 | C3 | Unify the three `same_segment` endpoint tolerances (0.005 / 0.001 / 0.001 m) behind one named constant, and collapse the four identical `normalize_angle` copies. |

---

## 6. Dependency graph

```
T-00 → T-01
        ├→ T-02, T-03, T-04            (Stage 1: corner)  ── TEST SESSION ──┐
        └→ T-05 … T-10                 (Stage 2: logging)                   │
                                            ↓                               │
                    T-11 → T-11b → T-12 → T-13 → T-14   (Stage 3)  ←────────┘
                                            ↓
                              T-15 … T-20                (Stage 4)
                                            ↓
                              T-21 … T-25                (Stage 5)
```

- **T-11 blocks T-11b and T-12.** Conditioning cannot be routed and the turn
  handover cannot be written until `update()` is called every tick again.
- **T-13 blocks T-14.** Both harden the same single-frame activation path; the
  wall clamp must exist before the spatial gate is tuned against it.
- **T-07 must skip the `speed_pid_*` columns** because **T-16 deletes that
  controller.** Adding them in Stage 2 would only be reverted in Stage 4.
- **T-11b adds `obstacle_turn_suppressed`**, so it depends on T-08 having
  already established the pattern for boolean gate-state columns.

---

## 7. Per-task commit template

```
<type>(<scope>): T-nn <summary> (F-nn)

Finding:   F-nn — <one line from the audit>
Change:    <what was edited, in one or two sentences>
Values:    <old → new for any tuning value touched, with units>
Verify:    <what you checked and the numbers you got>
Risk:      <what could regress, and what to watch in the next run>
Decision:  <D-n answer this depended on, if any>
```

---

## 8. Verification rules

- **Config/unit fix (T-02, T-03, T-04):** hand-compute the affected formula at
  three representative inputs and put the numbers in the commit body. Do not
  claim "correct" without arithmetic.
- **Logging addition (T-05…T-10):** confirm `telemetry_csv_header()` field count
  equals the field count written by `to_csv_row()`. Count them. A silent
  off-by-one here corrupts every column after the insertion point and is very
  hard to spot in a plot.
- **Behavioural change (Stage 3, Stage 4):** state which telemetry column would
  show the change, and what value you expect it to take. If no column would show
  it, the logging work is incomplete — go back to Stage 2.
- **Refactor (Stage 5):** identical output required. If you cannot demonstrate
  that, mark the task incomplete rather than asserting it.
- **Compilation is necessary but never sufficient.** Say so plainly if
  compilation is all you were able to do.

---

## 9. Final report

When the plan is finished (or you run out of unblocked tasks), write
`docs/audit/FIX_LOG.md` with:

- one row per task: ID / status (`DONE` / `SKIPPED` / `BLOCKED`) / commit hash /
  what was verified
- every tuning value that moved, old → new, with the reason
- **a re-tuning checklist**: which parameters in `tune.txt` are now stale because
  the behaviour they were tuned against has changed. T-02 and T-03 alone make
  several stale — at minimum check `turn_entry_blend_rad`,
  `turn_exit_blend_rad`, `heading_tolerance_rad`, `approach_distance_m`, and
  `turn_heading_pid.kp`, all of which were tuned against a saturated 53.8°
  feed-forward and a corner-speed cap that never took effect
- the available headroom noted in §3.1 item 2 (`turning_speed_mps` can rise
  toward ~0.47 before the lateral-accel limit binds), flagged as a *future*
  tuning session, not something this plan changed
- the two hardware checks from §4 plus the steering-angle measurement from §2,
  and whether results came back

---

## 10. Explicit non-goals

- Do not implement parallel parking.
- Do not touch `code/external/`.
- Do not change the camera colour thresholds.
- Do not act on F-07 or F-08 (§4) before the hardware checks come back.
- Do not add raw-scan logging (deferred pending a storage-bandwidth decision).
- Do not rewrite the WRO documentation (`README.md`, `docs/*/README.md`) except
  where T-16 requires correcting stale tuning guidance.
