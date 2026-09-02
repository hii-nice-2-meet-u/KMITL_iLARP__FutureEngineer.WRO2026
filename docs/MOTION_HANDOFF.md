# Motion — Session Handoff

> **สรุปไทย** — ใบส่งงาน motion สำหรับเซสชันถัดไป อ่านไฟล์นี้ไฟล์เดียวก็ทำงานต่อได้
> ทุกอย่างที่ landed พิสูจน์แล้ว, ทุกอย่างที่เหลือมี gate หรือเหตุผลชัดว่าทำไมยังไม่ทำ

**Repo:** `KMITL_iLARP__FutureEngineer.WRO2026`
**HEAD at handoff:** `cdca3b2` (branch `main`, working tree clean)
**Session baseline:** `8392d74` — everything below `61de22a` is this session's work.
**Pulse fingerprint reference commit:** `d00520a`.

---

## 1. Read these first, in order

| File | What it is |
|---|---|
| `docs/MOTION_MODEL_UNIFIED_PLAN.md` | **Why** — analysis, identifiability rank argument, the withdrawn H-1 (read §1), the open curvature violation (§2.3), corrections to the study |
| `docs/MOTION_CORE_AND_PATCH_PLAN.md` | **How** — the six-layer equation chain, per-task recipes, measurement programme |
| `docs/MOTION_PROGRESS.md` | running task ledger + how to run the pulse harness |
| `docs/TUNING_VARIABLES.md` | every tuning var and where it lands in `run_meta.json` |
| `docs/VEHICLE_MECHANICS_STUDY_TH.txt` / `VEHICLE_MECHANICS_REVIEW.md` | theory / measured corrections (do not edit) |

Task IDs: `P-nn` implementation, `M-nn` measurement, `H-nn` hypothesis. This
continues the numbering already in the repo (`P-01` = `e8c5b27`, `O-nn` =
observability/logging track).

---

## 2. The one idea the whole plan rests on

**κ (curvature) is the control stack's currency; `δ = atan(L·κ)` happens exactly
once, at the actuator boundary.** Reason: two curvatures add linearly
(`κ = κ_wall + κ_avoid`), two steering angles do not. That single fact dissolves
the obstacle-avoidance "replace or blend?" debate (F-05/F-06) — in curvature it
just adds.

**But the vehicle model is still contradicted by measurement:** the robot drives
`R = 0.155 m` where geometry permits `0.210 m` at the 38° clamp. Every κ-derived
number inherits this. That is why the physics-changing tasks (Track D) are gated
on measurement.

---

## 3. What landed this session (all verified)

| Task | Commit | Change | Proof |
|---|---|---|---|
| P-50/51/52 | `53bd3e1`,`e197244` | doc fixes: two motors; drive-banner wording; RPM-scale relabelled provisional | build |
| P-00 | `d00520a` | `common/servo_pulse.hpp` (extracted pulse math) + golden test; sim tests gain `--dump-pulses` | golden + determinism |
| **P-02** | `8c2366b` | `curvature_1pm` in `NavigationCommand`; δ→κ at the update() boundary, κ→δ in `condition_command()` | **pulse-identical** |
| **P-03** | `9e015fe` | delete `speed_pid` / `target_acceleration_mps2` (F-09) end to end | pulse-identical + field 94→93 |
| **P-21b** | `70e46f3` | `NavigationConfig::max_steering_rad` — clamp no longer reads Stanley cfg (F-11) | pulse-identical |
| P-20 | `a3bf503` | per-point `scan_phase` + `scan_period_us` on `LidarPoint`/`TimedLidarData` | build; field unread |
| (log) | `f11b581` | record the 6 `LidarProcessor::process` thresholds in `run_meta.json` | build; values unchanged |
| **P-21** | `b18201b` | `LidarProcessor::deskew()` + `ScanMotion`, **flag-off** | 6-property unit test + pulse-identical |

Plus `docs/TUNING_VARIABLES.md` (`cfab2c1`): confirmed **every** config group is
recorded per run; the only gap (LiDAR process args) was closed in `f11b581`.

**Net state of the control stack:** internal language is curvature at the
command boundary, the dead speed-PID path is gone, the steering clamp is
mode-independent, and the robot drives **byte-for-byte identically** — proven,
not asserted.

---

## 4. The verification harness (you MUST use this for Track A)

Track A tasks change the *representation*, not the behaviour. The bar is
**identical servo pulses**, not identical floats (a κ→δ→κ round trip differs by
~1e-7 rad, four orders below the servo's integer-µs quantisation).

```bash
B=<build dir>              # cmake -S code -B $B ; native build is fine
BL=<baseline dir>          # holds corner.pulses + heading.pulses from d00520a

# regenerate the baseline once, from a clean checkout of d00520a, if you don't have it:
#   git worktree add /tmp/base d00520a ; build there ; run the two dumps into $BL

cmake --build $B --target navigation_corner_sim_test navigation_heading_hold_test
$B/tests/navigation_corner_sim_test  --dump-pulses > /tmp/c.txt
$B/tests/navigation_heading_hold_test --dump-pulses > /tmp/h.txt
diff $BL/corner.pulses  /tmp/c.txt     # MUST be empty
diff $BL/heading.pulses /tmp/h.txt     # MUST be empty
```

Reference SHAs of the baseline dumps: corner `86fb8c61`, heading `5fc18ce4`
(first 8 of `sha1sum`). Corner = 170 pulses, heading = 4.

**If a task legitimately changes a sim test's config** (P-21b did — it changed
the clamp source), update the test config to preserve intent; do **not** move
the baseline. Only a real, reviewed behaviour change moves it.

Field-count check for any `log_types.cpp` edit: count `telemetry_csv_header()`
vs the fields `to_csv_row()` emits; state both in the commit.

---

## 5. Build notes / gotchas

- **Native build works** for everything except the camera stack. `perception_fusion_test`
  and the obstacle app fail to *link* against this host's **libcamera 0.2.0**
  (`controls::AeState` was removed from the vendored LCCV). Pre-existing,
  unrelated to motion. Build motion targets directly:
  `cmake --build $B --target navigation_corner_sim_test navigation_heading_hold_test lidar_deskew_test open_challenge_main`.
- To **syntax-check** `obstacle/main.cpp` on this host, use the LCCV shim at
  `<scratch>/shim/lccv.hpp` (a stub `lccv::PiCamera`) on the include path with
  `-fsyntax-only`. See how P-21/P-09 were checked.
- `code/tests/` is tracked; add a test per math/behaviour change and register it
  in `code/tests/CMakeLists.txt`. Non-camera tests: navigation_corner_sim,
  navigation_heading_hold, track_map, kinematics, servo_pulse, lidar_deskew.
- **Two `main.cpp` files** get every app-level change (open + obstacle). The
  banner/config live in `open_challenge_common.hpp`, shared by both.

---

## 6. Next tasks — status and the decision they need

| Task | What | Blocker | Can land now? |
|---|---|---|---|
| **P-22** | enable deskew; drop `wall_correction_rad` | **M-1** | No — wrong LiDAR-zero sign doubles the error |
| **P-25** | robust Huber line fit (F-14 root cause) | none, but it's a **geometry behaviour change** | Code + unit test yes; acceptance ("`outer_wall_valid` duty cycle rises") needs a real/replayed log |
| **P-23 / P-26** | adaptive breakpoint / geometric weighting | **M-11** (`σ_r` vs incidence) | No |
| **P-32 / P-33** | offline Python: plausibility asserts in `check_run.py`; identifiability report | none | **Yes — zero robot risk** |
| **P-04 / P-05** | Stanley / turn feed-forward return κ natively | — | **Deferred** (see §7) |
| **P-10…P-14** | clothoid, κ rate limit, speed coupling, `a_lat_max`, obstacle-as-κ | **M-4** (+ M-7 for P-12/13) | No |
| **P-40** | torque-mode drive | **withdrawn** (H-1 was wrong; the differential works) | — |

**The single highest-leverage next action is the ~90-minute measurement
programme**, especially **M-4** (steady-circle floor radius, 15 min): it is the
*only* external observation that breaks the rank-1 identifiability degeneracy
(`Λ = g·s_v/(s_ω·L)`, one number, three unknowns) and it unlocks all of Track D.

Recommended order for the next session:
1. If hardware is available: run the measurement programme (§8), starting M-4.
2. If not: do **P-32/P-33** (offline, useful immediately for analysing any log).
3. Land **P-25** only when a run exists to confirm the duty-cycle improvement.

---

## 7. Deferred, with reason (do not "finish" these blindly)

- **P-04 / P-05** — in the current identity-gain regime they relocate the δ→κ
  conversion into the handlers with **byte-identical pulses and no structural
  benefit** until the handlers do native curvature math (P-10, M-4-gated).
  Fold them into P-10/P-14 when the handlers go curvature-native. Doing them now
  is pure churn.
- **ActuatorCommand type split** (was in P-03's description) — deferred to the
  curvature-native boundary (P-11), to avoid churning the actuator interface
  twice.

---

## 8. The measurement programme (no code, ~90 min) — gates everything on the right

Full procedures in `MOTION_CORE_AND_PATCH_PLAN.md` PART 5 and
`docs/audit/HARDWARE_CHECKS.md`.

| ID | Experiment | Gives | Gate for |
|---|---|---|---|
| **M-4** | steady circle at ≥3 fixed pulses; tape-measure the floor diameter | `δ_eff(u)` map, `curvature_gain` | P-10…P-13, the sim |
| **M-1** | point the LiDAR raw-zero at a known wall; is it +Y or −Y in `test_lidar`? | LiDAR forward axis sign | **P-22** (deskew enable) |
| **M-2** | push straight, heading held ~0; which world axis does OTOS advance? | OTOS world frame (F-08) | nav world-transform fix |
| **M-11** | wall at 0.5/1.0/2.0 m **× 0°/30°/60° yaw**, 200 scans, per-range/incidence stdev | `σ_r(r, incidence)` | P-23, P-26 |
| **M-5** | wheelbase with a ruler | `L` | identifiability |
| **M-3a** | push 2.00 m motors off; OTOS displacement (`O-12` tool exists) | `s_v` | identifiability |
| **M-3c** | jack the wheels; command 100/200/400 RPM; count revs | STM32 RPM scale (the 1.75×) | powertrain |
| **M-6** | rotate exactly 360° by hand; OTOS heading change | `s_ω` | identifiability |
| **M-7** | speed sweep ≥5 setpoints **× ≥3 fixed steering angles** | powertrain `v = g(u)·f(\|κ\|)` | P-12/P-13 |
| **M-8** | coast-down, motors off | real deceleration (cfg says 5.0, real ~0.48) | ramp limits |

When results come back, record them and update `curvature_gain` / `a_lat_max`
**from the measurement, never from the log** (the log cannot separate the
factors — that is the whole point of §2.1).

---

## 9. Two facts that are easy to forget

1. **The model is contradicted by measurement** (curvature violation, §2). Do
   not build a simulator or trust any κ-derived tuning number until M-4.
2. **A four-for-four hypothesis (H-1) was already wrong once** — it matched four
   logged aggregates and was still false, because none of them *discriminated*
   it. Before proposing any new single-cause mechanism, read
   `MOTION_MODEL_UNIFIED_PLAN.md` §1. Require an observation that discriminates,
   not one the mechanism is merely consistent with.

## 10. Cold-start checklist

1. `git log --oneline` — has HEAD moved past `cdca3b2`?
2. Which `M-*` have returned? No Track D / P-22 task starts without its gate.
3. Rebuild the two sim tests; confirm they still match the `d00520a` fingerprint
   before starting a new Track A task.
4. Read `MOTION_MODEL_UNIFIED_PLAN.md` §1 before proposing any new mechanism.
