# Fix log — drive/steering control path & tuning telemetry

Tracks execution of `docs/FIX_PLAN_DRIVE_AND_LOGGING.md`.
Baseline: commit `f2fba69`, tagged **`pre-audit-fixes`**.

**Current position: Stage 0, 1 and 2 complete. Stage 3 not started.**
Stage 3 is gated on a field test — see "Next" at the bottom.

---

## Task status

| ID | Task | Status | Commit | Verified by |
|---|---|---|---|---|
| T-00 | Baseline commit | DONE (by the team) | `f2fba69` | Obstacle app and 8 modified files are tracked |
| T-01 | Tag baseline | DONE | tag `pre-audit-fixes` | `git tag` lists it |
| T-02 | `exit_acceleration_blend_rad` unit bug (F-01) | DONE | `8525b4c` | Weight recomputed at 5 heading errors |
| T-03 | `corner_radius_m` → 0.45 (F-04) | DONE | `347d5a2` | All three consumers recomputed; min-turn-radius check |
| T-04 | `turn_heading_pid` output limits (F-16) | DONE | `c945ada` | 20° + 15° = 35° < 38° clamp |
| T-05 | `plot_run.py` column names | DONE (not in repo) | — | Ran against a 75-column synthetic log; PNG renders |
| T-06 | PID / Stanley accessors | DONE | `a172af0` | `control` builds; pure addition |
| T-07 | PID internals → CSV | DONE | `3da6f88` | Header/value pairing, 75/75 aligned |
| T-08 | Turn-trigger + timing columns | DONE | `3da6f88` | same |
| T-09 | Obstacle + fusion columns | DONE | `3da6f88` | same; `obstacle/main.cpp` syntax-clean |
| T-10 | `row_index` + drop reporting | DONE | `3da6f88` | same |
| T-11…T-14 | Obstacle architecture | **NOT STARTED** | — | Gated on the Stage 1 field test |
| T-15…T-20 | Controller lifecycle | NOT STARTED | — | — |
| T-21…T-25 | Decoupling | NOT STARTED | — | — |

---

## Tuning values that moved

| Parameter | Old | New | Why |
|---|---|---|---|
| `exit_acceleration_blend_rad` | 20.0 rad (1145.9°) | 0.3491 rad (20°) | F-01 — bare literal missing the `* PI / 180` every neighbour has |
| `corner_radius_m` | 0.12 m | 0.45 m | F-04 / D2 — 0.12 m is below the vehicle's minimum turning radius (0.210 m at the 38° clamp), so the commanded arc was undrivable |
| `turn_heading_pid.min_output` / `.max_output` | ∓0.785398 (∓45°) | ∓0.261799 (∓15°) | F-16 — the per-PID limit exceeded the 38° composite clamp, so it was never the binding constraint |

### Derived quantities, before → after

| | Before | After |
|---|---|---|
| Corner feed-forward `atan2(0.16375, R)` | 53.8° (saturated the 38° clamp) | 20.0° |
| Headroom for `turn_heading_pid` | 0° | 18° |
| Lateral-accel speed limit `sqrt(0.5·R)` | 0.245 m/s | 0.474 m/s |
| Effective corner speed | 0.245 m/s | 0.280 m/s (now bound by `turning_speed_mps`) |
| Geometric turn trigger @0.28 m/s | 0.185 m | 0.498 m |
| Exit-acceleration weight at 90° heading error | 0.982 | 0.000 |

---

## Re-tuning checklist — values now stale

Every one of these was tuned against a corner that ran at ~0.44 m/s with a
permanently saturated feed-forward and a turn trigger firing 0.185 m from the
corner. All three of those facts have changed. **Do not trust these until they
have been re-checked on the track:**

| Parameter | Current | Why it is suspect |
|---|---|---|
| `turn_entry_blend_rad` | 22.5° | Tuned to soften entry into a saturated feed-forward. With 20° of feed-forward and 18° of headroom, entry is far gentler already — this may now be too slow. |
| `turn_exit_blend_rad` | 32° | Same reasoning at corner exit. |
| `heading_tolerance_rad` | 16.5° | Loosened to let turns complete despite a saturated, poorly-tracking corner. With the PID able to act, this can likely tighten. |
| `heading_confirm_frames` | 2 | Coupled to the tolerance above. |
| `approach_distance_m` | 0.90 m | Now only 0.39 m ahead of the turn trigger (was 0.72 m). The approach ramp has far less room. **Raise this before reverting `corner_radius_m` if the robot arrives too fast.** |
| `turn_heading_pid.kp` | 0.30 | Tuned when the PID's output was being discarded by the clamp for most of every corner. It now has real authority; 0.30 is likely too low. |
| `turning_speed_mps` | 0.28 | Not stale, but see headroom below. |

### Available headroom (do not apply blind)

`max_lateral_acceleration_mps2 = 0.50` is no longer the binding constraint on
corner speed — `turning_speed_mps = 0.28` is. If grip allows, `turning_speed_mps`
can rise toward **~0.47 m/s** before the lateral limit binds again. This is a
future tuning session, not part of this plan, and it needs tyre-grip testing on
the real competition surface.

---

## Telemetry schema change

`telemetry.csv` grew from 50 to **75 columns**. New columns, by the blind spot
they close:

- **Controller internals:** `stanley_cross_track_term_rad`,
  `stanley_heading_term_rad`, `stanley_heading_integral`,
  `turn_heading_pid_output_rad`, `turn_heading_pid_integral`
- **Turn trigger:** `turn_trigger_source` (0 none / 1 inner-corner /
  2 front-fallback / 3 legacy-front), `turn_trigger_frames`, `turn_armed`,
  `replay_gate_suppressed`
- **Timing:** `raw_update_dt_s` (pre-clamp)
- **Obstacle avoidance:** `obstacle_active`, `obstacle_color`,
  `obstacle_pass_side`, `obstacle_forward_m`, `obstacle_right_m`,
  `obstacle_target_right_m`, `obstacle_steering_rad`, `obstacle_confidence`,
  `obstacle_world_x_m`, `obstacle_world_y_m`
- **Fusion:** `lidar_valid_count`, `camera_valid_count`, `matched_count`,
  `frame_confirmed_count`, `camera_time_synchronized`
- **Integrity:** `row_index`

Deliberately **not** added: `speed_pid_*`, because D5/T-16 deletes that
controller. Deliberately **deferred**: `segments.csv` and raw-scan capture —
raw scan is ~272 KB/s even in binary and needs a storage-bandwidth decision.

---

## Deviations from the plan

1. **Stage 2 was reordered.** The plan put T-05 (`plot_run.py`) first; it was
   done last instead, so the plotter could be fixed once against the final
   75-column header rather than twice.
2. **T-09 added world coordinates** (`obstacle_world_x_m/y_m`) beyond the
   audit's robot-relative proposal, so `plot_run.py`'s existing obstacle
   scatter can be driven from `telemetry.csv` directly. This removes the need
   for the `obstacles.csv` that the plotter expected and the robot never wrote.

## Known gaps

- **`plot_run.py` is not version-controlled.** It lives at
  `/home/jukkruw/iLARP/plot_run.py`, outside the repository, so the T-05 fix is
  not captured in any commit and is not reproducible for anyone else. Consider
  moving it (and `gen_sample.py`) to `tools/` inside the repo — this is also
  worth points under rubric criterion 5.
- **`gen_sample.py` was not updated.** It still generates the old
  `pos_x`/`pos_y`/`speed_mps` schema plus a fabricated `obstacles.csv`, so it no
  longer matches what `plot_run.py` reads. Either update it to emit the real
  75-column header or delete it.
- **Nothing here has run on hardware.** All verification was arithmetic,
  compilation, and a synthetic-log round-trip.

---

## Next

1. **Run one Open Challenge session** with the Stage 1 changes. The single most
   important thing to watch: **is the inner-wall corner reliably detected at
   ~0.50 m?** The turn now triggers 2.7× further out than before, and that is a
   detection question the source cannot answer. Check `turn_trigger_source` and
   `wall_corner_forward_m` in the new telemetry.
2. **Run the two checks in `HARDWARE_CHECKS.md`** (~10 minutes). Check 2 becomes
   load-bearing during Stage 3.
3. Then start Stage 3 (T-11 → T-14), the obstacle-avoidance restructure.
