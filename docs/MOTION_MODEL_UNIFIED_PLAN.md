# Motion Model — Unified Plan

> **สรุปไทย** — รวมสามเอกสาร (`VEHICLE_MECHANICS_STUDY_TH.txt`, `VEHICLE_MECHANICS_REVIEW.md`,
> `MOTION_MODEL_IMPLEMENTATION_PLAN.md`, `MOTION_HANDOFF_ORDER.md`) เข้าเป็นแผนเดียว
> พร้อมแก้ข้อผิดพลาดที่พบและเพิ่มการวิเคราะห์ใหม่
>
> **สถานะของสมมติฐานหลัก:** ผมเคยเสนอ H-1 (เฟืองท้ายถูกล็อกโดยกฎการควบคุม) เพื่ออธิบาย
> ความผิดปกติสองเรื่องด้วยกลไกเดียว — **ทีมยืนยันว่าผิด และถอนแล้ว** ดู §1
> ความผิดปกติทั้งสองจึงยัง **ไม่มีคำอธิบาย** และน่าจะไม่เกี่ยวกัน
>
> **สถานะ: แผน ยังไม่แก้โค้ด**

**Supersedes:** `MOTION_MODEL_IMPLEMENTATION_PLAN.md`, `MOTION_HANDOFF_ORDER.md`
**Companion (do not edit):** `VEHICLE_MECHANICS_STUDY_TH.txt`, `VEHICLE_MECHANICS_REVIEW.md`
**Written against:** `8392d74`, working tree clean, 94-column telemetry
**ID scheme:** `M-nn` measurement · `P-nn` implementation (continues the existing numbering) · `H-nn` hypothesis

---

## 0. Status reconciliation

The four inherited documents were written at different times and contradict each other.
Verified against the repository at `8392d74`:

| Claim | Source | Actual | Verdict |
|---|---|---|---|
| `code/tests/` is gitignored, blocks all verification (`O-18`) | plan §7.3 | 5 test files tracked since `7cdc910` | **stale — unblocked** |
| Telemetry is 83 columns | plan header | **94** | stale |
| Telemetry is 94 columns | handoff | 94 | current |
| `P-01` kinematics done | handoff | `e8c5b27`, `kinematics.hpp` present, nothing consumes it | current |
| `motor_rpm_command_scale = 0.571` | handoff | set in **both** `main.cpp` (`df7e406`) | current |
| OTOS linear scalar measured −8.7 % | handoff | tool exists (`7a235f3`, `O-12`) | current |
| `segments.csv` not yet written | plan §9 | landed (`O-06`, `82bf71b`) | stale |
| Robot has a **single** DC gear-motor | `docs/mechanical/README.md` §2 | **two motors, mirrored, driving a common input; plus the differential** (team) | **doc is wrong — fix it (P-50)** |
| Drive is "M1 +RPM, M2 −RPM" | `open/main.cpp:71` banner | `write_motor_rpm()` sends the same **command** to both; the motors are mounted mirrored so their **shafts** turn opposite ways, and the firmware handles the inversion (`M_ENC_INVERTED` exists) | **not a bug — the banner mixes shaft rotation with command value. Clarify the wording (P-51)** |
| `curvature_gain` unmeasured, stays 1.0 | both | 1.0 | current |

**Two documentation defects fall out of this table and should be fixed regardless of
anything else in this plan:** the mechanical README's motor count, and the launch banner
whose wording conflates shaft rotation with command value.

---

## 1. H-1 — proposed, and withdrawn

> **สรุปไทย** — บันทึกสมมติฐานที่ตกไปแล้ว เพื่อไม่ให้มีใครเสนอซ้ำ และเพื่อเตือนวิธีคิดที่พลาด

### What was proposed

**H-1: dual independent speed control locks the rear axle.** `write_motor_rpm()` issues
the closed-loop `M1_SPD`/`M2_SPD` with the same value to both motors. If each motor drove
one rear wheel, two speed controllers each holding its own wheel to the same setpoint
would oppose the differential's only function, producing a locked axle, scrub, and yaw
beyond Ackermann.

### Why it is wrong

The team confirmed the actual topology: **the two motors are mounted mirrored and drive a
common input**, added to supply torque the earlier single-motor design could not. They are
not one-per-wheel. The differential therefore still splits speed between the rear wheels
exactly as intended, and the vehicle drives normally.

**There is no scrub mechanism. H-1 is withdrawn.**

This also resolves the banner: the motors' *shafts* turn in opposite directions because
they are physically mirrored, while the *command* is the same value for both, with the
inversion handled downstream. The banner and the code were describing two different things.

### The methodological lesson, recorded deliberately

H-1 produced four predictions and all four matched the logged aggregates. That felt like
strong evidence and it was not. The four "confirmations" were:

- two of them (yaw excess, and its growth with curvature) are consequences of **any**
  mechanism that inflates measured curvature, including a mis-calibrated steering map;
- one (lower speed ratio while turning) follows from **any** steering-angle-dependent drag,
  of which rear scrub is only one source;
- one (multiplicative rather than additive) is a property of nearly every scale error.

None of them discriminated. A hypothesis that explains everything and forbids nothing has
not been tested by the data it "explains". **The falsifying observation was a fact about
the assembly that no amount of log analysis would have produced.**

Practical consequence for the rest of this plan: §2's identifiability argument and the
measurement programme matter *more*, not less, now that the tidy single-cause story is
gone.

### What survives

| Idea | Status |
|---|---|
| A steering-angle-dependent load term `f(\|κ\|)` in the powertrain | **Survives**, with a different cause: front-wheel cornering drag, which needs no locked axle. `M-7`'s design (sweep speed × fixed steering angles) is unchanged |
| `P-40` torque-mode drive to "restore" differential action | **Withdrawn** — the differential was never defeated |
| `M-9` differential free/locked test | **Withdrawn** — answered by the team |
| The curvature violation | **Re-opened.** See §2.3, now the primary unexplained result |
| The 1.75× speed error | **Re-opened.** Still unexplained; `motor_rpm_command_scale = 0.571` remains an empirical single-point patch (P-52) |

### One latent concern, noted but not blocking

Two speed controllers acting on a **rigidly coupled** shaft is a redundant-actuator
arrangement: both regulate the same mechanical degree of freedom. If the two encoders or
PID tunings differ even slightly, the loops fight — each pushing against the other while
the shaft sits at some intermediate speed. Symptoms would be elevated current draw and
heat rather than a motion defect, so it is consistent with "drives fine".

Worth watching, not worth acting on now. It becomes relevant only if `M-7` finds the
command→speed relation is unstable or hysteretic rather than merely mis-scaled.

---

## 2. Identifiability — why log-only fitting cannot work

> **สรุปไทย** — คณิตศาสตร์ที่อธิบายว่าทำไมต้องวัดด้วยตลับเมตร ไม่ใช่ fit จาก log

### 2.1 The degeneracy

Curvature is inferred from odometry as `κ̂ = ω̂ / v̂`. Both come from the same sensor with
unknown scale factors `s_ω`, `s_v`:

```
ω_true = s_ω · ω̂        v_true = s_v · v̂

              ω_true     s_ω    ω̂       s_ω
κ_true  =    -------- =  --- · ---  =  ---- · κ̂
              v_true     s_v    v̂       s_v
```

The vehicle model contributes another factor `g` (`curvature_gain`):

```
κ_true = g · tan(δ) / L
```

The **observable** is `κ̂` as a function of the commanded `δ`. Rearranging,

```
                s_v     g
κ̂(δ)  =       ----- · ---  · tan(δ)
                s_ω     L
```

so what a log can constrain is the single lumped product

```
                g · s_v
Λ  =        ------------          [one number]
                s_ω · L
```

against three unknowns `g`, `s_v/s_ω`, `L`. In log-parameters the likelihood gradient
with respect to `(log g, log(s_v/s_ω), −log L)` points the same direction for all three,
so the Fisher information matrix has **rank 1**. No amount of driving, no richness of
excitation, and no choice of estimator separates them, because they enter the likelihood
only through their product.

> **This is why `M-4` must be a tape measure.** A floor-measured radius is an observation
> of `κ_true` that does not pass through the odometry at all, which is the only way to add
> an independent row to the information matrix.

### 2.2 What each experiment adds

Each row below adds one independent constraint. This is the minimal set that makes the
system identifiable.

| ID | Experiment | Constrains | Cost |
|---|---|---|---|
| **M-5** | Measure the wheelbase with a ruler, rear-axle centre to front-axle centre | `L` directly | 1 min |
| **M-6** | Rotate the robot through exactly 360° by hand against a floor mark; read OTOS heading change | `s_ω` | 5 min |
| **M-3a** | Push the robot 2.00 m along a tape, motors off; read OTOS displacement (`O-12` tool exists) | `s_v` | 5 min |
| **M-4** | Steady circle at fixed pulse; measure the floor diameter | `g · tan(δ(u)) / L`, external | 15 min |
| **M-7** | Speed sweep, ≥ 5 setpoints spanning the range, steady state only | powertrain gain / deadband / affine structure | 20 min |
| **M-8** | Coast-down from steady speed, motors disabled | friction, real deceleration | 10 min |
| **M-3c** | Jack the drive wheels clear; command 100 / 200 / 400 RPM; count wheel revolutions over 30 s | STM32 RPM scale — the leading suspect for the 1.75× error | 15 min |
| **M-11** | LiDAR range noise `σ_r` at 0.5 / 1.0 / 2.0 m, 200 scans (= old `P-24`) | `σ_r` for the breakpoint | 20 min |

With `L` (M-5), `s_ω` (M-6), `s_v` (M-3a), and an external `κ_true` (M-4), the gain `g`
is over-determined and can be **cross-checked** rather than assumed — which is what turns
this from curve-fitting into measurement.

**Total: about 80 minutes and no code.** It remains the highest-leverage time available
to the project, and the argument for it is now a rank argument rather than an intuition.

### 2.3 The curvature violation — the open candidates

With H-1 withdrawn (§1) this is the primary unexplained result in the motion model:
the vehicle drives `R_min = 0.155 m` where the geometry permits only `0.210 m` at the
38° clamp and `0.164 m` even at the 45° actuator limit.

Ordered by cost to settle, not by prior probability — the cheap exclusions come first
regardless of which is favoured.

| # | Candidate | Cheap test | Arithmetic |
|---|---|---|---|
| 1 | `wheelbase_m` is wrong | M-5 | `R = 0.155` at the 38° clamp implies **L = 121 mm**; at 45°, **L = 155 mm**. Recorded: 163.75 mm. A 121 mm wheelbase would be a gross recording error, so this is unlikely to be the *whole* story but is 1 minute to exclude. |
| 2 | OTOS angular scale | M-6 | Correcting only the known −8.7 % linear scalar moves `κ_max` from 6.44 to 5.88 /m, i.e. `R` from 0.155 → **0.170 m**. Still inside the 0.210 m limit, so **the linear scalar alone does not explain it.** An angular scalar of the same magnitude and opposite sense would be needed on top. |
| 3 | pulse → δ map is nonlinear | M-4 | `to_servo_pulse_us()` assumes `pulse = centre + (δ/45°)·span`, i.e. perfectly linear. An Ackermann linkage is not. The implied `δ` at `R = 0.155` is **46.6°** — just past the 45° actuator limit, exactly the signature of a map that delivers more angle than commanded near the extremes. |

The three are **not mutually exclusive** and may all contribute; `R = 0.155` needs a
combined factor of about 1.35 on `tan(δ)/L`, which several small errors could supply
together. That is exactly why §2.2 exists: the experiment set is designed to separate
them rather than to confirm a favourite.

**Current standing.** Candidate 3 is the most likely single contributor — the implied
46.6° sits just past the 45° actuator limit, which is the characteristic signature of a
map delivering more angle than commanded near the extremes, and `to_servo_pulse_us()`
assumes exact linearity through a linkage that has no reason to be linear. But §1 is a
recent reminder that "most likely" is not evidence. `M-4` produces the discriminating
observation, and it produces it for all three at once.

---

## 3. Corrections to the inherited documents

> **สรุปไทย** — ของเดิมผิดตรงไหนบ้าง พร้อมหลักฐาน

### 3.1 The review over-reaches on the powertrain structure

`VEHICLE_MECHANICS_REVIEW.md` §4 concludes from two per-mode medians that the
command→speed relation "has at least two free parameters". Fitting an affine model
through exactly those two points:

```
NORMAL   v_cmd = 0.233  ->  v_meas = 0.425
TURNING  v_cmd = 0.299  ->  v_meas = 0.452

v_meas = 0.409 · v_cmd + 0.330
```

The intercept says the robot moves at **0.330 m/s when commanded to stop**, which is
false. The affine model is therefore not merely under-determined — it is refuted by a
constraint the fit ignored (`v_cmd = 0 ⇒ v_meas = 0`).

**Correct conclusion: two aggregate medians support no model at all.** The mode
difference is confounded with speed, with steering angle, and with the transient/steady
mix inside each mode. `M-7` (a genuine sweep) is required before any structure is
claimed. This also means `motor_rpm_command_scale = 0.571` is a **provisional single-point
correction**, not an identified gain, and should be labelled as such in the code comment.

Under **H-1** the mode difference has a physical explanation that is neither gain nor
deadband: it is steering-angle-dependent scrub loss. If so the right model is

```
v_meas = g(u) · f(|κ|)
```

with `f` decreasing in `|κ|` — a *multiplicative load term*, which no amount of fitting a
gain-plus-deadband will capture. `M-7` should therefore sweep speed **at several fixed
steering angles**, not only in a straight line. This is a change to the review's
experiment design.

### 3.2 The study's §13.2 is wrong; the review's correction is right and worth keeping

"จุดปลาย segment เก่ากว่าจุดกลางเสมอ" is false — a sample's time within a scan is a
function of its **bearing**, not its range. The review's observation that this makes the
consequence *worse* (the corner point's timing offset varies as its bearing sweeps, so
skew enters as a drifting bias rather than a constant one) is correct and is the
sharper statement. Carried forward.

### 3.3 The plan's clothoid degeneracy rule is the wrong branch

`MOTION_MODEL_IMPLEMENTATION_PLAN.md` §4.3 says that when the corner is too tight for
two full ramps, "shorten both ramps proportionally". That preserves the path but demands
a steering rate the servo cannot deliver, so the controller silently falls behind its own
reference.

`s_ramp` is set by hardware; `κ_max` is a design choice. **Reduce `κ_max`, not
`s_ramp`.** Derivation in §4.3. In practice the branch is unreachable at current values
(`s_ramp = 0.109 m` against a limit of `Θ/κ_max = 0.707 m`, a 6.5× margin) so this is a
guard rather than a live concern — but it should be a guard that logs, not one that
silently degrades.

### 3.4 `curvature_gain` is a one-parameter approximation to something measurable

See §4.1. Recommend replacing it with a calibrated map before it acquires users.

---

## 4. The mathematical core

> **สรุปไทย** — คณิตศาสตร์ที่ต้องถูกต้องก่อนเขียนโค้ด แต่ละหัวข้อจบในตัวเอง

### 4.1 The steering map — replace the gain with a function

`kinematics.hpp` currently models the vehicle as

```
κ = g · tan(δ) / L
```

with `g` a scalar to be measured. But `M-4` does not measure `g` — it measures pairs
`(u_i, R_i)` of servo pulse and floor radius. From each pair, the **effective bicycle
steering angle** follows exactly:

```
δ_eff(u_i) = atan( L / R_i )
```

No gain appears. The gain was only ever a device for expressing "the map is not what
`to_servo_pulse_us()` assumes". Once the map is measured, the gain is redundant — and
worse, it is a rank-1 approximation to a function that is known to be nonlinear (§2.3
candidate 3).

**Proposal `P-06`: replace `curvature_gain` with a calibrated `SteeringMap`.**

```
δ_eff(u) = Σ_k a_k φ_k(u)          u = pulse_us − centre_us
```

with `φ_k` a monotone cubic (PCHIP) through the measured knots, or a low-order odd
polynomial `a₁u + a₃u³` if the M-4 table is sparse. Requirements:

- **odd**: `δ_eff(−u) = −δ_eff(u)` unless M-4 shows genuine asymmetry, in which case fit
  the two sides independently and say so
- **monotone**: enforced, so the inverse `u(δ)` exists and is single-valued
- **invertible in closed form or by bisection**: the actuator needs `u(δ)`

Then

```
κ(u) = tan(δ_eff(u)) / L
```

is exact by construction, `curvature_gain` disappears, and the model carries its own
calibration data rather than a fudge factor. Until M-4 runs, ship the map as the identity
(`δ_eff(u) = u · 45° / span`), which reproduces today's behaviour bit-for-bit.

> Keeping `curvature_gain` **and** adding a map would be two competing corrections for
> the same physical effect. Pick one. The map subsumes the gain.

### 4.2 Rate limiting in the unit the hardware actually enforces

The plan derives, from `δ = atan(Lκ/g)`,

```
dδ/dκ = (L/g) / (1 + (Lκ/g)²)      ⇒      κ̇_max(κ) = δ̇_max · (1 + (Lκ/g)²) · g/L
```

which is correct but incomplete: the hardware limit is not `δ̇_max` in rad/s. It is
`maximum_servo_step_us` per tick — a limit on **pulse rate**. The full chain is
`κ → δ → u`, so

```
                du_max/dt                    du_max/dt
κ̇_max(κ)  =  ------------  =  -----------------------------------
              du/dκ            (du/dδ)(δ) · (dδ/dκ)(κ)
```

With a linear pulse map `du/dδ` is constant and the two formulations agree up to a
constant. With the calibrated map of §4.1 they do not, and only this form is right.

**Consequence for `P-11`:** the single κ rate limiter must be derived from
`maximum_servo_step_us / dt`, not from `max_steering_rate_rad_s`. The latter is itself a
derived quantity whose provenance nobody has recorded. Deriving from the pulse step
removes one hand-tuned constant and grounds the limit in a datasheet number.

### 4.3 Clothoid corner reference — derivation and the degenerate branch

A clothoid has curvature linear in arc length. For the entry ramp on `s ∈ [0, s_r]`:

```
κ(s)  = κ_p · s / s_r
θ(s)  = ∫₀ˢ κ dσ = κ_p s² / (2 s_r)
θ_ramp = θ(s_r) = κ_p s_r / 2
```

The corner is ramp–arc–ramp with total heading `Θ` (π/2 for a WRO corner):

```
Θ = 2 θ_ramp + κ_p s_a = κ_p s_r + κ_p s_a
⇒  s_a = Θ/κ_p − s_r
```

Feasibility requires `s_a ≥ 0`, i.e.

```
s_r ≤ Θ / κ_p                                                    (feasibility)
```

`s_r` is set by hardware and the trigger margin:

```
s_r = v · ( δ_eff(κ_p) / δ̇_max ) + m_trigger
```

**Degenerate branch (§3.3).** If `s_r > Θ/κ_p`, the two ramps alone over-rotate. Two
repairs exist:

- (a) **hold `s_r`, reduce the peak.** With `s_a = 0`, `Θ = κ_p' s_r ⇒ κ_p' = Θ / s_r`.
  The servo constraint is respected; the path widens.
- (b) **hold `κ_p`, shrink `s_r` to `Θ/κ_p`.** The path is preserved; the required
  steering rate now exceeds the servo and the controller lags its reference.

Take **(a)**. `s_r` encodes a hardware limit and a stated trigger uncertainty; `κ_p`
encodes a preference. Never let a preference override a limit. Log
`clothoid_peak_reduced` when the branch fires so the widening is visible rather than
silent.

**Note on Fresnel integrals.** A clothoid's *position* requires Fresnel integrals
`C(t), S(t)`, and implementations often drag them in. This controller needs only `κ(s)`
and `θ(s)`, both elementary above. **Do not implement Fresnel integrals** — the
reference is a curvature schedule, not a geometric curve to be drawn.

**Worked example** at `v = 0.42`, `R = 0.45` (`κ_p = 2.2222 /m`), `δ̇ = 3.0 rad/s`,
`m = 0.06 m`:

```
δ_eff  = atan(0.16375 × 2.2222) = 0.3491 rad = 20.0°
s_r    = 0.42 × (0.3491/3.0) + 0.06 = 0.0489 + 0.06 = 0.1089 m
θ_ramp = 2.2222 × 0.1089 / 2 = 0.1210 rad = 6.93°
2θ_ramp = 13.9°     s_a = 1.5708/2.2222 − 0.1089 = 0.5980 m     arc = 76.1°
feasibility: s_r = 0.109 ≤ Θ/κ_p = 0.707     (6.5× margin)
```

Against today's `22.5° + 32° = 54.5°` of blending: **the ramps shrink from 61 % of the
corner to 15 %**, and the number is derived rather than tuned.

### 4.4 Weighted robust line fitting — and a result that inverts the intuition

The current fit is unweighted TLS with no outlier rejection. Both defects are worth
fixing, but the weighting has a consequence nobody in the three documents noticed.

A LiDAR return at range `r`, bearing `φ`, with independent range and bearing noise
`σ_r, σ_φ`, has Cartesian covariance

```
Σ = J · diag(σ_r², σ_φ²) · Jᵀ ,        J = ∂(x,y)/∂(r,φ)
```

whose principal axes are **along the beam** (variance `σ_r²`) and **across the beam**
(variance `r² σ_φ²`). For a line with unit normal `n̂` and beam direction `û`, the
variance that actually matters — perpendicular to the line — is

```
σ_⊥²  =  σ_r² (û·n̂)²  +  r² σ_φ² (1 − (û·n̂)²)
```

Evaluate at the two extremes, with `σ_r ≈ 10 mm` (to be confirmed by `M-11`) and
`σ_φ ≈ Δφ/√12 = 0.225°/√12 ≈ 1.1 mrad`:

| incidence | `û·n̂` | `σ_⊥` at r = 1 m |
|---|---|---|
| normal (beam ⊥ wall) | 1 | **10 mm** |
| grazing (beam ∥ wall) | 0 | **1.1 mm** |

> **Under a constant-`σ_r` model, grazing returns are ~9× more informative about a wall's
> perpendicular distance than head-on returns**, because at grazing incidence the range
> error slides *along* the wall and barely displaces it.

This inverts the usual intuition ("grazing returns are unreliable"), which is true for
*angle* but false for *perpendicular offset*.

**Caveat, and it matters.** `σ_r` is **not** constant with incidence. At grazing angles
the beam footprint elongates and the return mixes ranges across it, so `σ_r` grows —
possibly by enough to cancel the 9×. The honest form of the claim is therefore:

- the *geometry* term strongly favours grazing returns, and
- the *photometry* term opposes it by an unmeasured amount.

`M-11` should therefore measure `σ_r` **as a function of incidence angle**, not only of
range, by repeating the wall capture at 0°, 30° and 60° of wall yaw. If `σ_r(60°)` is
more than ~3× `σ_r(0°)`, the geometric weighting is not worth the complexity and `P-26`
should be dropped in favour of `P-25` alone. **State this test's outcome before
implementing `P-26`.**

**Estimator.** `P-25` ships the Huber half alone; `P-26` adds the geometric half only if
the `M-11` incidence test justifies it. Together they are IRLS with a product weight:

```
w_i = w_geom,i · w_huber,i

w_geom,i  = 1 / σ_⊥,i²                                    (from the model above)
w_huber,i = 1                if |e_i|/ŝ ≤ c
          = c ŝ / |e_i|      otherwise                    (c = 1.345, ŝ = 1.4826·MAD)
```

`w_geom` depends on `n̂`, so iterate: fit unweighted → compute `n̂` → weights → refit.
Two iterations suffice; the normal equations stay 2×2 and closed-form, so cost is
negligible. MAD rather than RMS for the scale estimate, because RMS is exactly what a
single specular return corrupts.

**Expected effect, stated before implementation:** `outer_wall_valid` duty cycle rises,
and `distance_error_m` noise falls at wall angles far from perpendicular. If neither
moves, the weighting model is wrong and should be reverted rather than tuned.

### 4.5 Deskew as an SE(2) action, with the approximation error bounded

A point measured at time `t_i` lives in the sensor frame `S(t_i)`. To express it in
`S(t_N)` (scan end), apply the inverse of the body motion over `Δt_i = t_N − t_i`. For a
constant body twist `ξ = (v_x, v_y, ω)`:

```
p_N  =  R(−ω Δt) · ( p_i − d(Δt) )

              1  ⎡  sin(ωΔt)      −(1 − cos(ωΔt)) ⎤
d(Δt)  =     ───  ⎢                                ⎥ · v
              ω  ⎣ (1 − cos(ωΔt))     sin(ωΔt)     ⎦
```

`d` is the exact SE(2) displacement; the common shortcut is `d ≈ v Δt`, whose leading
error is

```
‖d − vΔt‖  ≈  ½ |ω| ‖v‖ Δt²
```

At `v = 0.42 m/s`, `ω = 1.0 rad/s`, `Δt = 0.05 s`: **0.53 mm**, against a 40 mm stability
gate. The shortcut is therefore justified **at these speeds** — but the bound is stated
so that the justification is checkable rather than assumed, and it must be revisited if
`ω` or the scan period grows.

`ω → 0` makes the closed form singular; use the `vΔt` limit below `|ω| < 10⁻³ rad/s`.

**The sign warning stands and is the single most dangerous line in this plan.** The
translation is subtracted along the LiDAR frame's forward axis. If `M-1`
(`HARDWARE_CHECKS` Check 1) shows that axis is 180° from what the code assumes, deskew
**doubles** the error. `P-21` ships behind a flag defaulting off; `P-22` is gated.

### 4.6 Speed–curvature coupling must use a filtered curvature

```
a_lat = v² κ      ⇒      v_curve(κ) = sqrt( a_lat_max / |κ| )
```

Two failure modes, both avoidable:

1. **Braking on noise.** `κ` from the live Stanley output carries LiDAR jitter; each
   spike becomes a speed cut. Use the *reference* curvature, or a low-passed `κ`. The
   review raises this; it is a correctness requirement, not a refinement.
2. **`κ → 0` gives `v → ∞`.** Guarded in `kinematics.hpp` already
   (`MINIMUM_CURVATURE_1PM = 1e-3`), which caps the implied speed at
   `sqrt(a_lat_max/10⁻³) = 37 m/s` — effectively `min()` with the mode speed. Correct as
   written; noted so nobody "fixes" it.

`max_lateral_acceleration_mps2` becomes the most consequential number in the system the
moment this lands. It is 0.50 today, **exceeded by 274 %** in normal operation, i.e. it
does nothing. Measured p90 is 1.097, p99 1.371. Use **p90**, not p99: the coupling should
bind before the limit is reached, not at the worst sample ever observed. `P-13`,
separate commit, explicit sign-off.

---

## 5. Task list

> **สรุปไทย** — เรียงตาม gate ไม่ใช่ตามความสำคัญ ทุกอย่างซ้ายมือของ gate เริ่มได้ทันที

### Track M — measurement (no code, ~80 min total)

| ID | What | Unlocks |
|---|---|---|
| **M-5** | Wheelbase with a ruler | §2.3 candidate 1 |
| **M-3c** | STM32 RPM scale, wheels jacked | the 1.75× error |
| **M-6** | OTOS angular scalar, 360° | §2.3 candidate 2 |
| **M-3a** | OTOS linear scalar (tool exists, `O-12`) | `s_v` |
| **M-4** | Steady-circle floor radii, ≥ 3 pulses | `δ_eff` map (P-06), `curvature_gain` |
| **M-7** | Speed sweep ≥ 5 setpoints **× ≥ 3 fixed steering angles** | powertrain structure (§3.1) |
| **M-8** | Coast-down | real deceleration |
| **M-11** | LiDAR `σ_r` | P-23, P-26 |
| **M-1, M-2** | `HARDWARE_CHECKS` Checks 1 and 2 | P-22, F-07/F-08 |

### Track A — control plumbing (behaviour-neutral, provable, start now)

| ID | Task | Proof bar |
|---|---|---|
| **P-02** | Add `curvature_1pm` to `NavigationCommand`; controllers still compute δ and convert at the boundary; `condition_command()` converts back, shaping unchanged | pulse-identical |
| **P-03** | Delete `speed_pid`, `target_acceleration_mps2`; split `ActuatorCommand`. Update `tune.txt` + `docs/control/README.md` | pulse-identical + field count |
| **P-04** | Stanley returns `κ` at its boundary | pulse-identical |
| **P-05** | `update_turning`: feed-forward **is** `κ_ref`; drop `atan2(L,R)` + blend from the sum | pulse-identical |
| **P-06** | Replace `curvature_gain` with `SteeringMap` (§4.1); ship as identity. **`kinematics.hpp` is already committed with `kinematics_test.cpp` (`e8c5b27`) — that test asserts the gain's behaviour and must be rewritten in the same commit**, not left asserting a removed field | pulse-identical while identity; test rewritten, not deleted |

### Track B — perception

| ID | Task | Gate | Proof bar |
|---|---|---|---|
| **P-20** | `scan_phase` per point, **in acquisition order, before `ascendScanData()`**; log measured scan period | — | compiles; field unread |
| **P-21** | `deskew()` behind a flag, default **off**; exact SE(2) form (§4.5) | — | unit-test the transform; assert the ½ωvΔt² bound |
| **P-25** | Robust line fit: MAD-scaled Huber trim, no geometric weighting. Attacks the root cause of F-14 | — | `outer_wall_valid` duty cycle rises |
| **P-26** | Add geometric weighting `1/σ_⊥²` on top of P-25 (§4.4) | **M-11 incidence test** | `distance_error_m` noise falls at off-perpendicular wall angles |
| **P-23** | Adaptive breakpoint replacing `max_point_gap_m` | M-11 | segments/tick rises |
| **P-22** | Enable deskew; delete `wall_correction_rad` | **M-1** | `wall_corner_stability_error_m` median falls |

### Track C — identification tooling (offline, zero robot risk)

| ID | Task |
|---|---|
| **P-30** | `tools/identify_steering.py` — fit `δ_eff(u)` from M-4 knots **and** cross-check against `atan(L·ω/v)` from logs, gated on `\|Δpulse\| ≈ 0` and `\|v\| > 0.05`. The floor data is the source; the log is the check, never the reverse (§2.1) |
| **P-31** | `tools/identify_powertrain.py` — from M-7, fit `v_ss = g(u)·f(\|κ\|)`; report whether a load term is needed (§3.1) |
| **P-32** | Extend `check_run.py` with physical-plausibility asserts: implied δ > `maximum_steering_command_deg`, `\|κ\| > max_curvature`, `a_lat > max_lateral_acceleration_mps2` |
| **P-33** | `tools/identifiability_report.py` — given which of the M-series have returned, print which parameters are determined, over-determined, or still degenerate (§2.1) |

### Track D — gated on measurement (do not begin)

| ID | Task | Gate |
|---|---|---|
| **P-10** | Clothoid reference (§4.3); delete the smoothstep blend | M-4 |
| **P-11** | Single κ rate limit in pulse units (§4.2); delete the low-pass and δ-slew | M-4 |
| **P-12** | Speed–curvature coupling, filtered κ (§4.6) | M-4, M-7 |
| **P-13** | Set `max_lateral_acceleration_mps2` from measurement | M-7 + sign-off |
| **P-14** | Obstacle avoidance as κ superposition. **Supersedes `T-11`…`T-13`** | P-04, P-05 |

### Track E — documentation defects (do now, independent of everything)

| ID | Task |
|---|---|
| **P-50** | `docs/mechanical/README.md` says one motor; there are two, mirrored, driving a common input, added for torque. Correct it and record the topology |
| **P-51** | The launch banner says "M1 +RPM, M2 −RPM" (shaft rotation) while `write_motor_rpm()` sends one command value (inversion is downstream). Neither is wrong; the wording invites the reader to think they disagree. Reword to state both facts |
| **P-52** | Re-label `motor_rpm_command_scale = 0.571` as a provisional single-point correction, not an identified gain (§3.1) |

---

## 6. Verification

**Pulse-identical (Track A).** The servo is quantised to integer µs (~1.5 × 10⁻³ rad/µs),
four orders coarser than the κ→δ→κ float round-trip error (~10⁻⁷ rad). Identical *pulses*
is the correct and achievable bar; identical *floats* is neither.

```
git archive the pre-change commit  -> build -> baseline
working tree                       -> build -> candidate
run navigation_corner_sim_test and navigation_heading_hold_test on both,
dump the full servo_pulse_us sequence, require byte-identical
```

`code/tests/` is tracked (`7cdc910`), so this is reproducible by anyone.

> ⚠️ **`P-00` is a prerequisite and was missed by both inherited documents.**
> `navigation_corner_sim_test.cpp` does not touch `ActuatorOutput` or
> `to_servo_pulse_us` — it never produces a servo pulse, so this bar cannot be run as
> written. The tests must first be taught to emit a pulse sequence. Recipe in
> `MOTION_CORE_AND_PATCH_PLAN.md` PART 3. **No Track A task can be proven, and therefore
> none should be committed, until P-00 lands.**

**Field count (any `log_types` edit).** `telemetry_csv_header()` and `to_csv_row()` are
two hand-maintained parallel lists. Count both, state both in the commit body. A mismatch
silently shifts every later column.

**Behaviour change.** Name the telemetry column, its direction, and rough magnitude
**before** implementing; then show it in a real or replayed log. A prediction made
afterwards is not evidence.

**Property tests** for every new math unit, registered in `code/tests/CMakeLists.txt`:

| Unit | Properties |
|---|---|
| `SteeringMap` | monotone; odd (unless M-4 shows asymmetry); `u(δ_eff(u)) ≈ u` to 1e-6; inverse exists on the whole commandable range |
| `BicycleModel` | round-trip κ↔δ to 1e-6 over ±45°; both conversions odd and strictly monotone; `speed_for_lateral_limit(0,·)` finite |
| clothoid | `θ(s_r) = κ_p s_r / 2` exactly; `Θ` reproduced to 1e-6; degenerate branch reduces `κ_p` and never `s_r`; `κ(s)` continuous at both joins |
| deskew | identity at `Δt = 0`; identity at `ξ = 0`; exact-vs-approx error ≤ ½‖ω‖‖v‖Δt²; no singularity as ω→0 |
| weighted TLS | exact recovery of a noiseless line; a single gross outlier moves the fit by < 1 mm; `σ_⊥` model matches a Monte-Carlo draw |

---

## 7. Risk register

| # | Risk | Mitigation |
|---|---|---|
| 1 | **A replacement single-cause story is adopted as readily as H-1 was.** H-1 matched four logged aggregates and was still wrong (§1). | No mechanism enters the vehicle model without an observation that *discriminates* it from the alternatives in §2.3 — not merely one it is consistent with. |
| 2 | **The κ migration is blamed for the curvature violation.** The violation exists today, in δ. | Land P-02 with proven-identical pulses first. The violation must remain visible and unchanged across the refactor. |
| 3 | **`δ_eff` is fitted from logs instead of the floor.** §2.1 shows the log cannot separate the factors; fitting from it folds sensor error into the vehicle model permanently. | P-30 takes M-4 as source and the log as cross-check only. Enforce in review. |
| 4 | **Deskew enabled before M-1 and doubles the error.** | P-21 flag-off; P-22 gated. |
| 5 | **P-13 lands with `a_lat_max = 0.50`** (robot crawls at 0.474 m/s) or with p99 (robot slides). | Use p90 = 1.097. Separate commit, measurement attached, sign-off. |
| 6 | **Two speed loops on one rigidly coupled shaft fight each other** (§1). Not a motion defect, but it would show as current draw and heat. | Out of scope here. Revisit only if `M-7` finds the command→speed relation hysteretic rather than merely mis-scaled. |
| 7 | **Three plans still overlap** (`T-nn` fix, `O-nn` observability, `P-nn` motion). | This document supersedes the two motion documents. P-14 supersedes T-11…T-13; P-26 precedes F-14. Update `FIX_LOG.md` when P-14 lands. |
| 8 | **The clothoid arrives before the trigger is accurate enough.** | `m_trigger` exists for this, sized from the measured 20 mm stability floor. Do not tune it to zero because the servo arithmetic says it is unnecessary. |

---

## 8. Order

```
P-50/51/52 doc defects        (independent, do now)
M-5 M-3a M-3c M-6 M-4 M-7 M-8 M-11 M-1 M-2   (~90 min, no code)
   │
   ├─► P-02 ─► P-03 ─► P-04 ─► P-05 ─► P-06        (Track A, pulse-identical)
   │                                    │
   ├─► P-20 ─► P-21(off)                │
   ├─► M-11 ─► P-26 ─► P-23             │
   ├─► P-30 P-31 P-32 P-33              │
   │                                    │
   [M-1] ─► P-22 deskew ON              │
   [M-4] ─► P-10 ─► P-11 ────────────► P-12 ─► P-13 ─► P-14
   [M-7] ────────────────────────────────┘
```

Everything left of a gate starts today. Everything right of one waits on about eighty
minutes with a tape measure.

---

## 9. Non-goals

- **No simulator yet.** Layer 2 is contradicted by measurement (§1, §2.3). A simulator
  built on it would be confidently wrong. Revisit after M-4.
- **Do not implement `T-11`…`T-13`.** P-14 replaces them.
- **Do not act on F-07 / F-08** before `HARDWARE_CHECKS` returns.
- **No tuning changes** except `P-13`, which exists solely for that and requires sign-off.
- **No raw LiDAR scan capture.** `segments.csv` (`O-06`) covers the debugging need at
  ~5 % of the storage cost.
- **Do not edit `VEHICLE_MECHANICS_STUDY_TH.txt`.** It is a study, correctly labelled.
  This document carries the corrections.
- **No Fresnel integrals** (§4.3).
- **No pure-pursuit hybrid.** The NORMAL/TURNING split already is one; a second blending
  mechanism before the kinematic model is trusted would confound both.

---

## 10. Open questions for the lead

1. **The curvature violation is now the primary open result** (§2.3). Of the three
   candidates, the pulse→δ map is the cheapest to settle and `M-4` settles it. Is there
   any known reason to expect the linkage to be linear?
2. **The 1.75× speed error is also re-opened.** `M-3c` (jack the wheels, command a known
   RPM, count revolutions) isolates the STM32's RPM scale in 15 minutes and is the
   leading suspect. Has anyone checked what CPR / gear ratio the firmware assumes?
3. **`corner_radius_m = 0.45`** was chosen against a *geometric* minimum of 0.210 m. If
   the vehicle can actually drive 0.155 m, the choice deserves revisiting — but not until
   it is understood *why* it can.
