# Brief — Logging philosophy & the tuning workbench

> เอกสารนี้เขียนเพื่อเอาไปคุยกับโมเดลอื่นต่อ ไม่ใช่ใบสั่งงาน
> สถานะโค้ด: หลัง Stage 2 ของ `docs/FIX_PLAN_DRIVE_AND_LOGGING.md` (commit `9e71a90`)
> อ้างอิงหลัก: `docs/audit/03_logging_for_tuning.md`, `docs/audit/FIX_LOG.md`

Two halves of one system:

- **On-robot** — capture only what tuning and analysis need, minimum processing,
  raw-first.
- **Off-robot** — the part that matters most: draw the map, draw the path, draw
  analysis plots, and a **tuning workbench** that does not exist yet (replay,
  compute gains from logs, simulate a path from a parameter set).

---

## 1. Is the logging clean? What philosophy is it using?

**Honest answer: it is now *useful*, but it is not *clean*, and it has no stated
philosophy.** What exists is a debug print that happens to be CSV-shaped.

Evidence:

- One flat 75-column table mixes three different kinds of thing — sensor
  measurement, controller intermediate, and actuator command — with nothing
  marking which is which. A consumer cannot tell what is evidence and what is a
  conclusion.
- Streams are split **by module** (`telemetry` / `walls` / `corners` / `events`),
  not by rate or by rawness. So `walls.csv` writes at loop rate, `corners.csv`
  writes four times per run, and both are "logging".
- `WallLogger::record` returns early unless `mode == NORMAL`
  (`wall_logger.cpp:16`). **There is no wall geometry at all during `TURNING`,
  `SEARCH_DIRECTION`, or obstacle avoidance** — precisely the phases being
  debugged.
- **No run records the configuration that produced it.** Every gain, every
  distance, every blend angle is compiled in. A log from two weeks ago cannot be
  interpreted without a `git checkout`. This is the single biggest gap for
  everything in §4.

### Proposed philosophy (one sentence)

> **The robot writes evidence, not conclusions — and never writes anything that
> cannot be reconstructed offline from the evidence plus the config.**

Tier by **rate + rawness**, not by module:

| Tier | Content | Rate | Rule |
|---|---|---|---|
| **1 — Raw** | Sensor output before any filtering: LiDAR points, OTOS pose/vel/accel, camera frames | Sensor rate | Never post-processed on the robot. Binary, not CSV. |
| **2 — Derived** | Controller state, decisions, commands (today's `telemetry.csv`) | Loop rate | Every field must be a pure function of tier 1 + tier 3. If it is not, that is a bug. |
| **3 — Static** | Full config dump, git SHA, build time, calibration constants | Once per run | JSON. Without this, tiers 1–2 are uninterpretable. |

The value of the invariant: it makes tier 2 **testable**. Replay tier 1 through
the real controller offline and every tier-2 column must reproduce exactly. That
is also the foundation of §4.2 and §4.3.

---

## 2. Is any code blocking or distorting the raw data?

**Yes — in six places, all verified in source.** Ordered by how much they hurt.

| # | Where | What is lost |
|---|---|---|
| 1 | **No raw stream exists at all** | The only LiDAR output that reaches disk is the three *resolved* walls. `ProcessedLidarData::line_segments` (every fitted segment, including the rejected ones) and the raw point cloud are discarded every tick. A run cannot be re-processed with different `LidarProcessor` thresholds. |
| 2 | `LidarProcessor::is_valid_point` (`lidar_processor.cpp:117`) | Points are dropped **before** anything downstream sees them: `quality < 10`, `distance < 0.015 m`, and `distance > 3.0 m`. These are tuning decisions applied irreversibly at the sensor boundary. If the 3.0 m cutoff is wrong, no log can prove it. |
| 3 | `speed_mps = hypot(velocity.x, velocity.y)` (`open/main.cpp:160`) | **Speed is unsigned.** Reversing is indistinguishable from driving forward, and the velocity *direction* — which carries sideslip, i.e. the entire tyre model — is thrown away before logging. |
| 4 | `getPosVelAcc(position, velocity, acceleration)` | **`acceleration` is read every tick and never used or logged.** The OTOS already measures it; this is free data for powertrain identification being discarded. |
| 5 | `process_scan(..., wall_correction_rad, ...)` | Wall resolution is rotated by `heading − target_heading` before classification. The logged `wall_angle_rad` is therefore **heading-corrected geometry, not measured geometry**. Fine for control, wrong as evidence. |
| 6 | `ActuatorOutput::apply` | `target_servo_pulse_us` (before `limit_servo_pulse_step`) is a local variable. Only the post-clip value is logged, so **you cannot tell from a log whether the step limiter was clipping** — which is exactly the question when steering feels sluggish. |

Two more that are absences rather than distortions:

- **No encoder readback.** `spi::Command` has `M1_SPD`/`M2_SPD` as *setters* and
  `M_ENC_ENABLE/DISABLE/INVERTED` as config; there is no read-speed command. The
  STM32 closes its own loop and reports nothing back. The Pi has **zero
  measurement of actual wheel speed** — OTOS ground velocity is the only truth.
- **One timestamp for two sensors.** `scan.timestamp_us` labels the OTOS row
  too, but the OTOS is read *after* the scan arrives. Pose and geometry are
  logged as simultaneous when they are not. This is a systematic error that will
  show up directly as a bias in §4.3.

**Fixes 3, 4 and 6 are each one or two lines and should happen before any of
§4.** They are the difference between "we have data" and "we have evidence".

---

## 3. Motion equations, by category

Your 3.1 and 3.2 are right but they are two of **six**. The four missing ones are
what a simulator actually needs.

### 3.1 Powertrain — accel and brake *(yours, partly)*

```
v_cmd[k] = v_cmd[k-1] + clamp(v_target - v_cmd[k-1], ±a_max·dt)     a_max = 5.0
rpm      = v_cmd · 60 / (π · d)                                      d = 0.053 m
                    ↓ SPI M1_SPD/M2_SPD
STM32 closed-loop PID → motor → wheel                                ← BLACK BOX
```

Two things to know:

- **There is no brake model.** `brake()` is only called by `emergency_stop()`.
  Ordinary deceleration is just commanding a lower RPM and letting the STM32
  plus friction do whatever they do. So "การเบรค" is currently **not modelled at
  all** — it is an emergent property of an off-repo controller.
- The STM32's PID is not in this repository. Any simulator has to treat it as an
  identified first-order lag (see §4.3), not as known dynamics.

### 3.2 Steering *(yours)*

```
Straight:  δ = PID_heading(−e_ψ) + atan2(k·e_ct, v + v_soft)          clamp ±38°
Corner:    δ = δ_entry·(1−w_in) + sign·atan2(L, R)·min(w_in, w_out)
               + PID_turn(−e_track)                                   clamp ±38°
Shaping:   low-pass τ=0.035 s → slew ≤3.0 rad/s → clamp
Servo:     pulse = center + (δ_deg / 45) · span → step limit ±500 µs/tick
```

Note the last line assumes **pulse maps linearly to wheel angle**. An Ackermann
linkage is geometrically nonlinear. This assumption is unverified and it is the
highest-value thing to identify from data (§4.3).

### 3.3 Vehicle kinematics — **missing, and it is the bridge**

```
ψ̇ = v · tan(δ) / L        ẋ = v · cos ψ        ẏ = v · sin ψ        L = 0.16375
```

This bicycle model appears **nowhere in the codebase** except inverted inside the
corner feed-forward (`atan2(L, R)`). It is the only thing connecting §3.1 to
§3.2, and a simulator is essentially this equation plus the controller.

### 3.4 Measurement model — **missing, and the biggest hidden cost**

The controller's input is not pose — it is **wall distance and wall angle from
LiDAR**. So a closed-loop simulator must simulate what the robot *sees*: given a
field and a pose, produce the `ResolvedWalls` the LiDAR would have reported.
Without this you can only simulate open-loop.

**This is the main thing to decide before building §4.2** — see the two-tier
proposal there.

### 3.5 Latency — missing

Sensor timestamp → command reaching the servo. Currently unmeasurable (§2, one
shared timestamp). `turn_preview_time_s = 0.1` exists specifically to compensate
for it, and is a guess.

### 3.6 Tyre / grip — missing

`max_lateral_acceleration_mps2 = 0.50` is a placeholder. It is identifiable from
logs (see §4.3) and currently no longer even the binding constraint after the
Stage 1 fix.

---

## 4. The tuning workbench — feasibility

Overall: **4.1 is easy, 4.2 is feasible in two tiers, 4.3 is real but must be
scoped down.** One blocker and one architectural decision dominate everything.

### The blocker

**Nothing records the configuration a run used.** 4.1 ("show the parameters in
use at that time") is impossible today. Fix: write `config.json` into each run
directory alongside `telemetry.csv`, containing the full `NavigationConfig`,
`StanleyConfig`, all `PIDConfig`s, `ObstacleConfig`, `ActuatorConfig`, plus git
SHA and build timestamp. Small task, unblocks all three features.

### The architectural decision

**The simulator must call the real C++ controller, not a Python
re-implementation.** `NavigationController` is ~1200 lines of stateful logic with
subtle ordering (as the audit showed). A Python copy will silently drift from the
robot within weeks, and then the workbench lies to you — worse than having no
workbench.

Recommended: a thin **pybind11** wrapper exposing `NavigationController::update()`
and the config structs. The controller is already well-isolated (it takes
`ProcessedLidarData` + pose + speed and returns a command), so this is a small
wrapper, not a refactor. Everything in 4.2 and 4.3 then runs the same code the
robot runs.

### 4.1 — Read back the last 5 runs, draw them, show their parameters

**Easy once `config.json` exists.** `plot_run.py` already draws trajectory,
speed, mode and obstacles. Needs: run selector, config diff view between runs
(*"what changed between run 3 and run 4"* is the question actually being asked),
and the new Stage 2 columns plotted — `stanley_cross_track_term_rad` vs
`stanley_heading_term_rad` separately is what makes `k` and `kp` tunable at all.

### 4.2 — Simulate a path from a parameter set

Split into two tiers. **Tier A gives ~80% of the value for ~20% of the work** and
should come first.

**Tier A — replay (open-loop).** Feed the *logged* wall geometry back into the
real controller with new gains and compare the commands it would now produce
against what it actually produced. Answers *"if kp were 0.5 instead of 0.3, what
would the steering have been on this exact run?"* No field model, no LiDAR model,
no vehicle model needed. Also doubles as a regression test for the fix plan.

Note: replay is only exact for one step — the moment commands differ, the robot
would have been somewhere else and the logged geometry no longer applies. So Tier
A is honest for **one-step command comparison**, not for trajectory prediction.
Say this in the UI or it will be misread.

**Tier B — closed-loop simulation.** Needs §3.3 (kinematics) + §3.4 (a synthetic
LiDAR against a field model) + §3.5 (latency) + the identified powertrain lag.
This is where the real cost is, and it is mostly in the LiDAR model — the field
is a rectangle-in-rectangle, so a ray-cast against 8 line segments is genuinely
tractable, but wall-detection noise/dropout is what makes the sim behave like
reality, and that is hard to fake convincingly.

### 4.3 — Identify hidden parameters from ~20 runs

**Feasible, but only for a specific short list.** Your instinct that "the sensor
data isn't that good" is correct, so scope this to parameters where the signal is
strong and the estimator is simple. Realistic targets:

| Parameter | How | Signal quality |
|---|---|---|
| **servo pulse → actual wheel angle** | `δ = atan(L · ψ̇ / v)` from OTOS heading rate and speed; bin by commanded pulse to build the real curve | **Strong.** Highest value — it directly tests the linearity assumption in §3.2 |
| **actuation latency** | cross-correlate commanded steering against measured yaw rate; the lag is the peak | Good, needs steering changes with sharp edges |
| **powertrain lag τ** | fit first-order response of measured speed to commanded speed steps | Good, but needs §2 fix 3 (signed velocity) |
| **max usable lateral accel** | `a_lat = v · ψ̇`; take a high percentile across corners | Moderate — noisy, but a percentile is robust |

Not realistic: identifying everything at once, or anything requiring accurate
sideslip, until §2 fixes 3 and 4 land.

The honest framing: this is **parameter fitting on four specific quantities**, not
general system identification. Framed that way it is a few hundred lines of
numpy and it will work. Framed as "make the sim match reality" it will not
converge.

---

## 5. What to decide before building

1. **Does the workbench bind to the real C++ controller (pybind11), or
   re-implement in Python?** Recommendation: bind. This decision is hard to
   reverse later.
2. **Tier A replay only, or commit to Tier B closed-loop sim?** Recommendation:
   Tier A first, decide on B after seeing how much it is used.
3. **Raw LiDAR capture: yes or no?** ~272 KB/s in binary at 20 Hz. Without it,
   `LidarProcessor` thresholds can never be re-tuned offline. With it, storage
   and write bandwidth on the Pi need checking. This is the one real
   cost/benefit tradeoff in the whole plan.
4. **Web page or desktop?** Everything above is UI-agnostic. The data model
   matters more than the frontend, and picking the frontend first usually
   distorts the data model.

## 6. Suggested order

```
config.json per run  +  §2 fixes 3, 4, 6      ← small, unblocks everything
        ↓
4.1 log browser + config diff                 ← immediate daily value
        ↓
pybind11 binding to NavigationController      ← the architectural commitment
        ↓
4.2 Tier A replay                             ← answers most tuning questions
        ↓
4.3 four-parameter identification             ← feeds Tier B
        ↓
4.2 Tier B closed-loop sim                    ← only if A proved insufficient
```

Everything above the pybind11 line is worth doing regardless of whether the
simulator is ever built.
