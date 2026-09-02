# Motion — Progress Log

> Companion to `MOTION_CORE_AND_PATCH_PLAN.md`. Records what has landed so a
> resume is cold-start clean. Update it when a task lands.

**Baseline:** `8392d74`. **Pulse fingerprint reference:** commit `d00520a`
(corner 170 pulses, heading 4 pulses; regenerate with `--dump-pulses`).

## Landed

| Task | Commit | What | Verified |
|---|---|---|---|
| P-50 | `53bd3e1` | mechanical README: two motors, not one | — |
| P-51/52 | `e197244` | drive banner wording; RPM-scale relabelled provisional | build |
| P-00 | `d00520a` | `common/servo_pulse.hpp` + golden test; sim tests `--dump-pulses` | golden + determinism |
| P-02 | `8c2366b` | `curvature_1pm` in the command; convert at the boundary | **pulse-identical** |
| P-03 | `9e015fe` | delete `speed_pid` / `target_acceleration_mps2` (F-09) | pulse-identical + field count 93=93 |
| P-21b | `70e46f3` | `NavigationConfig::max_steering_rad` (F-11) | pulse-identical |
| P-20 | `a3bf503` | per-point `scan_phase` + `scan_period_us` | build; field unread |
| (log) | `f11b581` | record the 6 `LidarProcessor::process` thresholds in `run_meta.json` | build; values unchanged |
| P-21 | `b18201b` | motion `deskew()` transform + `ScanMotion`, **flag-off** | 6-property unit test; pulse-identical |

**Full config surface is now recorded per run** — see `docs/TUNING_VARIABLES.md`.
Every tuning group appears in `run_meta.json`.

## The verification harness (how to prove pulse-identity)

```
B=<build dir>
BL=<baseline dir>   # holds corner.pulses, heading.pulses from d00520a
cmake --build $B --target navigation_corner_sim_test navigation_heading_hold_test
$B/tests/navigation_corner_sim_test  --dump-pulses > /tmp/c.txt
$B/tests/navigation_heading_hold_test --dump-pulses > /tmp/h.txt
diff $BL/corner.pulses /tmp/c.txt      # must be empty
diff $BL/heading.pulses /tmp/h.txt      # must be empty
```
If a task legitimately changes a sim test's config (e.g. P-21b changed the
clamp source), update the test config to preserve intent — do not move the
baseline.

## Deferred, with reason

- **P-04 / P-05** (Stanley/turn feed-forward return κ natively). In the
  identity-gain regime these relocate the δ→κ conversion into the handlers with
  byte-identical pulses and no structural benefit until the handlers do native
  curvature math (P-10, gated on M-4). Doing them now is churn. Fold them into
  P-10/P-14 when the handlers go curvature-native.
- **ActuatorCommand type split** (was part of P-03's description). Deferred to
  the curvature-native boundary (P-11) to avoid churning the actuator interface
  twice.

## Next, and why it stops here

P-21 is landed (deskew code, flag-off, unit-tested). What remains splits into
three, and none is a clean "land it now with proof":

- **P-22** (enable deskew, drop `wall_correction_rad`) — gated on **M-1**.
  Wrong LiDAR-zero sign ⇒ deskew doubles the error. Cannot enable without it.
- **P-25** (robust Huber line fit, F-14 root cause) — *not* gated, but it is a
  **LiDAR-geometry behaviour change**: its acceptance evidence is
  "`outer_wall_valid` duty cycle rises" on a real/replayed log, which needs a
  run. Implementable + unit-testable now; not field-validatable here.
- **P-23 / P-26** (adaptive breakpoint / geometric weighting) — gated on
  **M-11** (`σ_r`, incidence-dependent).
- **P-32 / P-33** (offline Python: plausibility asserts, identifiability
  report) — zero robot risk, unblocked, a context switch into tooling.
- **P-04 / P-05 / Track D (P-10…P-14)** — deferred / M-4-gated (see below).

So the decision at this point is: land P-25 unvalidated, do the offline tooling,
or pause for the measurement programme.

Everything further is gated on the ~90-minute measurement programme
(`MOTION_MODEL_UNIFIED_PLAN.md` §2.2 / `MOTION_CORE_AND_PATCH_PLAN.md` PART 5):
M-5, M-3a, M-3c, M-6, M-4, M-7, M-8, M-11, M-1, M-2. None needs code.

**The single highest-value item is M-4** (steady-circle floor radius): it is the
only external observation that breaks the rank-1 identifiability degeneracy
(§2.1) and it unlocks the entire physics-changing Track D (P-10…P-14).

## Cold-start checklist

1. `git log --oneline` — has the baseline moved past `a3bf503`?
2. Which M-* have returned? No Track D task starts without its gate.
3. Rebuild the two sim tests; regenerate the baseline fingerprint only if a
   deliberate, reviewed change moved it.
4. Read `MOTION_MODEL_UNIFIED_PLAN.md` §1 before proposing any new mechanism —
   a four-for-four hypothesis (H-1) was already wrong once.
