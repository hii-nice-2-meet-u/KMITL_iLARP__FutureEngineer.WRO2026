# Motion — Core Equations & Patch Plan

> **สรุปไทย** — เอกสารนี้คือ *คู่มือลงมือ* อ่านแล้วทำได้เลยโดยไม่ต้องย้อนอ่านอย่างอื่น
> ส่วนเหตุผล/หลักฐาน/การวิเคราะห์อยู่ใน `MOTION_MODEL_UNIFIED_PLAN.md`
>
> **แกนเดียวของทั้งแพช:** ให้ **κ (ความโค้ง)** เป็นสกุลเงินกลางของทุกชั้น
> แล้วแปลงเป็น δ (มุมพวงมาลัย) **ครั้งเดียว** ที่ขอบ actuator
> เพราะ κ บวกกันเชิงเส้นได้ ส่วน δ บวกกันไม่ได้
>
> **สถานะ: ยังไม่แก้โค้ด**

**Analysis / evidence:** `MOTION_MODEL_UNIFIED_PLAN.md`
**Theory:** `VEHICLE_MECHANICS_STUDY_TH.txt` · **Measured corrections:** `VEHICLE_MECHANICS_REVIEW.md`
**Baseline:** `8392d74`, 94-column telemetry, `code/tests/` tracked

---

# PART 1 — The spine

## 1.1 The one sentence

```
κ is the currency.  δ = atan(L·κ) happens exactly once, at layer 6.
```

Everything else in this document follows from that. The reason it matters:

```
two curvatures add:        κ_total = κ_wall + κ_avoid          ✓ exact
two steering angles do not: δ_total ≠ δ_wall + δ_avoid          ✗ 4.3% error at 20°
```

That single fact dissolves `F-05`/`F-06` (the "replace or blend the steering?" debate in
the obstacle work) — in curvature, it adds.

## 1.2 The six layers

```
  L0  scan → geometry      per-point time → deskew → breakpoint → robust fit → walls
  L1  state                signed v, ω, δ_meas
  L2  kinematics           κ ↔ δ ↔ pulse            ← the hub, one file
  L3  reference            clothoid κ(s)
  L4  control              every controller returns κ; they SUM
  L5  speed                v coupled to κ
  L6  actuator boundary    κ rate-limited, then κ → δ → pulse
```

## 1.3 The equations, by layer

### L0 — sensor → geometry

```
0.1  per-point time      t_i = t_N − (1 − φ_i)·T
                         ** φ_i captured in ACQUISITION order, before ascendScanData() **

0.2  deskew (SE(2))      Δt  = t_N − t_i
                         p_N = R(−ω·Δt) · (p_i − d)

                               1  ⎡ sin(ωΔt)      −(1−cos(ωΔt)) ⎤
                         d =  ───  ⎢                             ⎥ · v
                               ω  ⎣ (1−cos(ωΔt))    sin(ωΔt)     ⎦

                         d → v·Δt as ω → 0 ; use that below |ω| < 1e-3
                         approximation error ≤ ½·|ω|·‖v‖·Δt²  = 0.53 mm @ (0.42, 1.0, 0.05)

0.3  breakpoint          D_max = r·sin(Δφ)/sin(λ − Δφ) + 3σ_r          λ = 10°, σ_r from M-11

0.4  robust weighted fit min Σ wᵢ (n̂·pᵢ + c)²
                         σ_⊥² = σ_r²(û·n̂)² + r²σ_φ²(1 − (û·n̂)²)
                         w = w_geom · w_huber
                         w_geom  = 1/σ_⊥²
                         w_huber = 1 if |e|/ŝ ≤ c else c·ŝ/|e|   (c = 1.345, ŝ = 1.4826·MAD)
```

### L1 — state

```
1.1  signed speed        v = vx·cos ψ + vy·sin ψ        (NOT hypot — it loses the sign)
1.2  yaw rate            ω = velocity.h                 (already logged: otos_yaw_rate_rps)
1.3  measured steering   δ_meas = atan(L·ω / v)         |v| > 0.05 required
```

`1.3` is the steering feedback the system has never had, and the observable that
identifies the steering map.

### L2 — kinematics (the hub)

```
2.1  κ = tan( δ_eff(u) ) / L                u = pulse_us − centre_us
2.2  ψ̇ = v · κ
2.3  δ_eff = SteeringMap(u)                 monotone, odd, invertible
```

**Rule:** every kinematic `tan`/`atan`/`atan2` in the control path lives here and nowhere
else. See §3.1 for the one existing violation.

### L3 — reference (clothoid)

```
3.1  κ(s)     = κ_p · s / s_r                    on the ramp
3.2  θ(s)     = κ_p · s² / (2·s_r)
3.3  θ_ramp   = κ_p · s_r / 2
3.4  s_r      = v · ( δ_eff(κ_p) / δ̇_max ) + m_trigger
3.5  s_a      = Θ/κ_p − s_r                      feasibility: s_r ≤ Θ/κ_p
3.6  degenerate: if s_r > Θ/κ_p  →  reduce κ_p to Θ/s_r.  NEVER shrink s_r.
                 (s_r is a hardware limit; κ_p is a preference. Log it when it fires.)
```

Do **not** implement Fresnel integrals. Only `κ(s)` and `θ(s)` are needed — both elementary.

### L4 — control (everything returns κ, and they SUM)

```
4.1  straight   κ = tan(δ_stanley)/L      δ_stanley = PID(−e_ψ) + atan2(k·e_ct, v + v_soft)
4.2  corner     κ = κ_ref(s) + Kp·e_track + Ki·∫e_track
4.3  obstacle   κ += clamp( κ_avoid, ±κ_avoid_max )
4.4  total      κ_cmd = clamp( κ, ±κ_max )
```

In `4.2` the feed-forward **is** `κ_ref` — no `atan2(L,R)`, no blend weights.

### L5 — speed, coupled to curvature

```
5.1  v_curve  = sqrt( a_lat_max / |κ_filt| )     κ_filt, NEVER the raw command
5.2  v_target = min( v_mode, v_curve, v_approach )
5.3  v_cmd[k] = v_cmd[k−1] + clamp( v_target − v_cmd[k−1], ±a_max·dt )
```

`a_lat = v²κ` is physics, not a tuned constant, so the slowdown applies in **every** mode.
Today an aggressive Stanley correction on a straight produces no slowdown at all.

### L6 — the single boundary

```
6.1  dδ/dκ    = L / (1 + (L·κ)²)

6.2  κ̇_max(κ) = (du/dt)_max / [ (du/dδ)(δ) · (dδ/dκ)(κ) ]
                ** the hardware limit is maximum_servo_step_us — a PULSE rate **

6.3  δ_cmd = atan( L · κ_cmd )
6.4  u     = SteeringMap⁻¹( δ_cmd )
6.5  rpm   = v_cmd · 60 / (π·D) · motor_rpm_command_scale
```

Four smoothing stages collapse into one:

| today | after |
|---|---|
| smoothstep feed-forward blend | deleted — the clothoid ramps instead |
| steering low-pass `τ = 0.035 s` | deleted |
| steering slew clamp `3.0 rad/s` | becomes the **source** of `κ̇_max` |
| servo pulse step limit `500 µs` | **kept** — a hardware guard, not a filter |

## 1.4 The constraint that governs the whole programme

```
              g · s_v
Λ  =      ─────────────           rank 1
              s_ω · L
```

`g`, `s_v/s_ω`, `L` enter the likelihood only through this product. **No log-only fit can
separate them.** `M-4` (a floor-measured radius) is the only observation that does not
pass through the odometry, and therefore the only one that adds an independent row.

---

# PART 2 — Current → target

| Symbol | Today | After | Task |
|---|---|---|---|
| command type | `NavigationCommand{speed, steering_rad, accel}` | `NavigationCommand{speed, curvature_1pm}` + `ActuatorCommand{speed, steering_rad}` | P-02, P-03 |
| `target_acceleration_mps2` | computed, logged, **reaches no actuator** | deleted | P-03 |
| `speed_pid` | tuned in `tune.txt`, inert | deleted, docs corrected | P-03 |
| Stanley output | `δ` | `κ` | P-04 |
| corner feed-forward | `atan2(L,R)` × smoothstep blend | `κ_ref(s)` from the clothoid | P-05, P-10 |
| `curvature_gain` | scalar, unmeasured, 1.0 | `SteeringMap` measured by M-4 | P-06 |
| steering clamp source | `stanley.max_steering_rad` (F-11) | `NavigationConfig::max_steering_rad` | P-21b |
| rate limiting | 4 stages, δ-domain | 1 stage, κ-domain, pulse-derived | P-11 |
| speed vs steering | independent | `v_curve = √(a_lat/κ)` | P-12 |
| `a_lat_max` | 0.50, exceeded by 274% | p90 = 1.097, measured | P-13 |
| obstacle avoidance | replaces `steering_rad` | `κ += κ_avoid` | P-14 |
| `wall_correction_rad` | controller heading injected into the processor | deleted; deskew fixes the cause | P-22 |
| `max_point_gap_m` | 0.12 flat | adaptive `D_max(r)` | P-23 |
| line fit | unweighted TLS, RMS gate drops whole segments | Huber, then geometric weights | P-25, P-26 |

---

# PART 3 — Patch recipes

Each entry is: **files · change · how to prove it**. One task, one commit.

## P-00 — make the sim test emit servo pulses  ⚠️ PREREQUISITE

**This blocks all of Track A and was missed by both inherited documents.**
`code/tests/navigation_corner_sim_test.cpp` does not touch `ActuatorOutput` or
`to_servo_pulse_us` — it never produces a pulse. The "pulse-identical" proof bar that
`MOTION_HANDOFF_ORDER.md` §5 and `MOTION_MODEL_UNIFIED_PLAN.md` §6 both specify **cannot
be run as written**.

- **Files:** `code/tests/navigation_corner_sim_test.cpp`, `navigation_heading_hold_test.cpp`
- **Change:** drive each simulated command through the same `to_servo_pulse_us()` +
  `limit_servo_pulse_step()` arithmetic the actuator uses, and print one
  `servo_pulse_us` per tick to stdout behind a `--dump-pulses` flag. Do not link the real
  `ActuatorOutput` (it opens SPI); extract or duplicate the pure conversion.
- **Prove:** run it, get a deterministic sequence, run it again, get the same bytes.
- **Behaviour:** none — test-only.

## Track A — κ plumbing (behaviour-neutral, provable)

### P-02 — add `curvature_1pm`, convert at the boundary
- **Files:** `navigation_state.hpp`, `navigation_controller.cpp`
- **Change:** add `curvature_1pm` to `NavigationCommand`. Controllers still compute δ
  internally; convert δ→κ where they return, and κ→δ at the top of `condition_command()`.
  The existing shaping runs **unchanged**.
- **Prove:** pulse-identical vs `git archive HEAD` baseline.

### P-03 — delete the dead speed path
- **Files:** `navigation_controller.{hpp,cpp}`, `navigation_state.hpp`,
  `log_types.{hpp,cpp}`, both `main.cpp`, `tune.txt`, `docs/control/README.md`
- **Change:** remove `speed_pid`, `speed_pid_`, `target_acceleration_mps2` everywhere;
  split `ActuatorCommand` out. Correct the two documents that instruct tuning a gain that
  reaches no actuator.
- **Prove:** pulse-identical **and** field count — `telemetry_csv_header()` vs
  `to_csv_row()`, state both numbers in the commit body.
- ⚠️ The only motion task that touches `log_types`.

### P-04 — Stanley returns κ
- **Files:** `stanley_controller.{hpp,cpp}`, `navigation_controller.cpp`
- **Change:** `update_normal` converts the Stanley δ to κ at the call boundary.
- **Prove:** pulse-identical.

### P-05 — corner feed-forward becomes κ_ref
- **Files:** `navigation_controller.cpp`
- **Change:** `update_turning` sums in curvature. The feed-forward term is `κ_ref`
  directly; `atan2(L,R)` moves into `kinematics.hpp`. Blend weights still applied — the
  clothoid replacing them is P-10, not this task.
- **Prove:** pulse-identical — `κ_ref` must reproduce today's blended feed-forward exactly.

### P-06 — `SteeringMap` replaces `curvature_gain`
- **Files:** `common/kinematics.hpp`, `code/tests/kinematics_test.cpp`
- **Change:** `δ_eff(u) = Σ aₖφₖ(u)`, monotone, odd unless M-4 shows asymmetry, invertible.
  Ship as the identity (`δ_eff(u) = u·45°/span`), which reproduces today bit-for-bit.
- ⚠️ `kinematics_test.cpp` currently asserts `curvature_gain` behaviour. **Rewrite it in
  the same commit** — do not delete it.
- **Prove:** pulse-identical while identity; property tests in PART 4.

### P-21b — `max_steering_rad` gets its own home (closes F-11)
- **Files:** `navigation_controller.{hpp,cpp}`
- **Change:** add `NavigationConfig::max_steering_rad`, initialised to the current
  effective value; `clamp_steering()` reads it instead of `stanley.max_steering_rad`.
- **Prove:** pulse-identical.

## Track B — perception

### P-20 — capture `scan_phase`
- **Files:** `lidar_module.cpp`, `lidar_struct.hpp`
- **Change:** record each point's fraction-of-revolution **before** `ascendScanData()`
  reorders by angle. Log the measured scan period.
- **Prove:** compiles; field is unread. Assert `scan_phase` is monotone pre-sort.

### P-21 — `deskew()` behind a flag, default off
- **Files:** `lidar_processor.{hpp,cpp}`
- **Change:** exact SE(2) form (0.2), `ω→0` limit guarded, `ScanMotion` passed as a
  parameter so the processor stays pure.
- **Prove:** unit tests — identity at `Δt=0`, identity at `ξ=0`, error ≤ ½|ω|‖v‖Δt².
- ⚠️ **Do not enable.** `P-22` is gated on `M-1`. Wrong sign ⇒ deskew **doubles** the error.

### P-25 / P-26 — robust fit, then geometric weights
- **Files:** `lidar_processor.cpp`
- **P-25:** Huber with MAD scale. MAD not RMS — RMS is what one specular return corrupts.
- **P-26:** add `w_geom = 1/σ_⊥²`. **Only if `M-11`'s incidence test justifies it** — if
  `σ_r(60°) > 3·σ_r(0°)` the photometric penalty cancels the geometric gain; drop P-26.
- **Prove:** state before implementing — `outer_wall_valid` duty cycle rises (P-25);
  `distance_error_m` noise falls at off-perpendicular wall angles (P-26).

### P-23 — adaptive breakpoint
- **Files:** `lidar_processor.cpp` · **Gate:** M-11
- **Prove:** segments-per-tick rises in `segments.csv`.

### P-22 — enable deskew, delete `wall_correction_rad`
- **Gate:** **M-1** · **Prove:** `wall_corner_stability_error_m` median falls from ~20 mm.
  If it does not fall, deskew is not the dominant error — say so and stop.

## Track C — offline tooling (zero robot risk, start any time)

| ID | File | What |
|---|---|---|
| P-30 | `tools/identify_steering.py` | Fit `δ_eff(u)` from **M-4 floor knots**. Log data is a cross-check only — §1.4 says it cannot be the source. Gate log samples on `\|Δpulse\|≈0` and `\|v\|>0.05`. |
| P-31 | `tools/identify_powertrain.py` | From **M-7**, fit `v_ss = g(u)·f(\|κ\|)`. Report whether the load term is needed. |
| P-32 | extend `tools/check_run.py` | Plausibility asserts: implied δ > `maximum_steering_command_deg`; `\|κ\| > max_curvature`; `a_lat > a_lat_max`. This single check is what found the curvature violation. |
| P-33 | `tools/identifiability_report.py` | Given which M-* have returned, print which parameters are determined / over-determined / still degenerate. |

## Track D — gated on measurement, do not begin

| ID | What | Gate |
|---|---|---|
| P-10 | Clothoid `CornerReference`; delete the smoothstep blend | M-4 |
| P-11 | Single κ rate limit in **pulse units**; delete low-pass + δ-slew | M-4 |
| P-12 | Speed–curvature coupling, filtered κ | M-4, M-7 |
| P-13 | Set `a_lat_max` from measurement (**p90 = 1.097**, not p99) | M-7 + sign-off |
| P-14 | Obstacle avoidance as κ superposition. **Supersedes T-11…T-13** | P-04, P-05 |

## Track E — documentation defects (independent, do now)

| ID | What |
|---|---|
| P-50 | `docs/mechanical/README.md` says one motor. There are **two, mirrored, driving a common input**, added for torque. The differential is downstream and works normally. |
| P-51 | The banner "M1 +RPM, M2 −RPM" describes **shaft rotation**; `write_motor_rpm()` sends one **command value** (inversion is downstream). Neither is wrong — reword so nobody reads a contradiction. |
| P-52 | Re-label `motor_rpm_command_scale = 0.571` as a provisional single-point correction, not an identified gain. |

---

# PART 4 — Verification

## Pulse-identical (every Track A task)

```
git archive <pre-change commit> | build   →  baseline
working tree                    | build   →  candidate
run both tests with --dump-pulses on each
require: byte-identical
```

Rationale: the servo quantises to integer µs (~1.5e-3 rad/µs), four orders coarser than
the κ→δ→κ float round-trip (~1e-7 rad). Identical **pulses** is correct and achievable;
identical **floats** is neither. Requires **P-00**.

## Field count (any `log_types` edit)

`telemetry_csv_header()` and `to_csv_row()` are two hand-maintained parallel lists.
Count both. State both in the commit body. A mismatch silently shifts every later column.

## Behaviour change

Name the telemetry column, direction, and rough magnitude **before** implementing.
A prediction made afterwards is not evidence.

## Property tests (`code/tests/`, registered in its `CMakeLists.txt`)

| Unit | Must hold |
|---|---|
| `SteeringMap` | monotone; odd unless M-4 shows otherwise; `u(δ_eff(u)) ≈ u` to 1e-6; invertible across the full commandable range |
| `BicycleModel` | κ↔δ round-trip to 1e-6 over ±45°; both odd and strictly monotone; `speed_for_lateral_limit(0,·)` finite |
| clothoid | `θ(s_r) = κ_p·s_r/2` exactly; `Θ` reproduced to 1e-6; degenerate branch reduces `κ_p`, never `s_r`; `κ(s)` continuous at both joins |
| deskew | identity at `Δt=0`; identity at `ξ=0`; error ≤ ½|ω|‖v‖Δt²; no singularity as ω→0 |
| weighted TLS | exact on a noiseless line; one gross outlier moves the fit < 1 mm; `σ_⊥` matches Monte-Carlo |

---

# PART 5 — Order, gates, and how to resume cold

## Order

```
P-50 P-51 P-52          docs, independent, do now
P-00                    make the tests emit pulses      ⚠️ blocks all of Track A
  │
  ├─ P-02 → P-03 → P-04 → P-05 → P-06 → P-21b           Track A, pulse-identical
  ├─ P-20 → P-21(off)                                   Track B
  ├─ M-11 → P-25 → P-26 → P-23
  └─ P-30 P-31 P-32 P-33                                Track C, offline
                                                        
[M-1] → P-22 deskew ON
[M-4] → P-10 → P-11 ──┐
[M-7] ────────────────┴→ P-12 → P-13 → P-14
```

## The measurement programme (~90 min, no code, gates everything on the right)

| ID | Experiment | Gives |
|---|---|---|
| M-5 | Wheelbase with a ruler | `L` |
| M-3a | Push 2.00 m, motors off, read OTOS (`O-12` tool exists) | `s_v` |
| M-3c | Jack the wheels; command 100/200/400 RPM; count revolutions over 30 s | STM32 RPM scale — leading suspect for the 1.75× error |
| M-6 | Rotate exactly 360° by hand, read OTOS heading | `s_ω` |
| M-4 | Steady circle at ≥3 fixed pulses; measure floor diameter | `δ_eff(u)` — **the only external observation** |
| M-7 | Speed sweep ≥5 setpoints **× ≥3 fixed steering angles** | powertrain structure, load term |
| M-8 | Coast-down, motors disabled | real deceleration (config says 5.0; measured p01 is −0.48) |
| M-11 | LiDAR `σ_r` at 0.5/1.0/2.0 m **and at 0°/30°/60° wall yaw** | `σ_r(r, incidence)` for P-23/P-26 |
| M-1, M-2 | `docs/audit/HARDWARE_CHECKS.md` Checks 1 and 2 | LiDAR zero axis; OTOS world axis |

## Cold-start checklist

1. `git log --oneline -5` — confirm the baseline moved or not.
2. Which `M-*` have returned? Nothing in Track D may start without its gate.
3. Is `P-00` done? If not, no Track A task can be proven and none should be committed.
4. Read `MOTION_MODEL_UNIFIED_PLAN.md` §1 before proposing any new mechanism — a
   four-for-four hypothesis was already wrong once.

## Two things that are true and easy to forget

**The model is still contradicted by measurement.** The vehicle drives `R = 0.155 m`
where geometry permits `0.210 m` at the 38° clamp and `0.164 m` even at the 45° actuator
limit. Every number derived from κ in PART 1 inherits that. It is why Track D is gated,
and it is why `M-4` is the highest-value fifteen minutes available.

**`tan`/`atan` are not currently in the control path at all.** The invariant "kinematic
conversions live only in `kinematics.hpp`" is about to become *harder* to hold, not
easier: the one existing violation is `std::atan2(wheelbase_m, radius_m)` inside
`update_turning`, which is a kinematic conversion wearing `atan2` clothing. `P-05` moves
it. Stanley's `atan2(k·e_ct, v + v_soft)` is **not** a kinematic conversion — it is a
control law and it stays.
