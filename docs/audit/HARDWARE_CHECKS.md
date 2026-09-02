# Hardware checks — frame conventions and mount results

The audit found two disagreements between code and documentation. Check 1 is
now resolved from the assembled robot and datasheet convention; Check 2 still
needs a short physical test before changing any world-frame transform.

Check 1 (M-1) has now been resolved from the assembled-robot observation below.
Check 2 remains open until the OTOS world-axis push test is performed.

---

## Check 1 — LiDAR raw angle zero (F-07)

**The disagreement.** `LidarProcessor::polar2cartesian`
(`code/modules/lidar/lidar_processor.cpp:135-143`) computes:

```cpp
result.x_m = point.distance_m * -std::sin(rad);
result.y_m = point.distance_m * -std::cos(rad);
```

Both `mark.txt` and `docs/lidar/README.md` specify the same formula **without
the minus signs**. The code is therefore a 180° rotation of the documented
mapping — at `angle_deg = 0` the documented formula puts a point at `y = +d`
(ahead) and the code puts it at `y = −d` (behind).

Either the sensor is mounted with its raw-zero axis pointing backwards and the
negation is a correct, undocumented compensation, or it is a genuine bug. The
source cannot tell which.

### Procedure

1. Put the robot on the floor with a flat wall roughly **1 m directly in front**
   of it, and nothing within 2 m on the other three sides.
2. Find the RPLIDAR's own raw-angle-zero reference (the silkscreen mark or the
   cable exit — check the S3 datasheet for which one marks 0°) and note which
   way it points relative to the robot's forward direction.
3. Run the LiDAR viewer:
   ```bash
   ./test_lidar
   ```
4. On the rendered view, find the drawn robot origin and the fitted wall.

### Result (M-1 — recorded)

The raw-zero arrow points to the **rear of the robot**. Therefore the physical
front is raw 180°. With the current code's two minus signs, raw 180° maps to
`(x, y) = (0, +distance)`, so the robot frame remains **+Y forward**. The
forward translation term in deskew (`dy = v * dt`) has the correct sign.

**Result:** PASS — P-22 may enable deskew; do not change `polar2cartesian()`.
The minus signs are the documented 180° mount compensation.
**Date / who ran it:** recorded from team/datasheet confirmation, 2026-09-02

---

## Check 2 — OTOS world axis at heading 0 (F-08)

**The disagreement.** Two robot→world transforms read the same OTOS pose in the
same process and differ by a constant 90°:

| Implementation | At heading 0 |
|---|---|
| `perception::robot_to_world` (`perception.cpp:231-246`) | forward → **+world X**, right → −world Y |
| `NavigationController::update_wall_corner_landmark` (`navigation_controller.cpp:~734`) | forward → **+world Y**, right → +world X |

`obstacle_controller.hpp::to_robot` is the exact algebraic inverse of the
`perception` version, so two of three already agree and the navigation one is
the outlier. That is strong evidence but not proof — this check establishes
which convention the physical sensor actually reports.

### Procedure

1. Place the robot on the floor pointing along whichever direction the team
   calls "forward" on the field diagram. Note that direction.
2. Power up and let `calibrate_otos()` finish (keep the robot **completely
   still** during calibration — it calls `calibrateImu(255, true)` then
   `resetTracking()`, so this defines the zero).
3. Run any app that prints the pose — `./test_otos` is simplest.
4. Confirm the reported heading is near **0 rad**. If it is not, the tracking
   reset did not take; restart before continuing.
5. **Push the robot straight forward by hand** about 1 m along the direction
   from step 1. Do not let it rotate.
6. Read the reported `x` and `y`.

### What to record

- [ ] `x` grew by ~1.0 m and `y` stayed near 0 → **forward is +world X.**
      `perception::robot_to_world` is correct; the navigation wall-corner
      transform is the one to fix.
- [ ] `y` grew by ~1.0 m and `x` stayed near 0 → **forward is +world Y.**
      The navigation transform is correct; `perception::robot_to_world` and
      `obstacle_controller::to_robot` both need fixing — note that this is the
      more expensive outcome, since it means the obstacle world positions
      (including the new `obstacle_world_x_m`/`obstacle_world_y_m` telemetry
      columns) are currently 90° wrong.
- [ ] Neither — both changed, or the magnitude is far off 1 m → the OTOS
      mounting offset or scaling is wrong, which is a separate problem. Record
      the raw numbers and stop.

Measured `x` after the 1 m push: `__________`
Measured `y` after the 1 m push: `__________`
Measured heading during the push: `__________`

**Result:** _______________________________________________
**Date / who ran it:** ____________________________________

---

## Why this matters even though nothing is visibly broken

`wall_corner_filtered_world_` is currently only converted back to robot frame by
the same function's own inverse, so the 90° error cancels and today's turn
trigger works. The moment that world position is cross-referenced against
anything built on the `perception` convention — `TrackMap`, the traffic
landmarks, `ObstacleController`'s landmark matching — the error becomes live and
the landmark lands 90° away from the real corner.

Stage 3 of the fix plan brings the obstacle controller and the navigation
controller into the same loop. That is the point at which this stops being
theoretical.
