# Control

Low-level controllers used by the navigation layer: a general PID and a Stanley
lateral controller built on top of it.

**Source code:** [`code/control`](../../../../code/control)
· PID: [`pid.cpp`](../../../../code/control/pid.cpp) / [`pid.hpp`](../../../../code/control/pid.hpp)
· Stanley: [`stanley_controller.cpp`](../../../../code/control/stanley_controller.cpp) / [`.hpp`](../../../../code/control/stanley_controller.hpp)

---

## 1. PID (`control::PID`)

A reusable PID with the safety features a real control loop needs:

- **Output clamping** — `min_output` / `max_output`.
- **Integral anti-windup** — the integral term is clamped to
  `min_integral` / `max_integral`, so it cannot wind up while the output is
  saturated.
- **dt handling** — `calculate(setpoint, current)` measures `dt` from a
  `steady_clock`; an explicit `calculate(setpoint, current, dt_s)` overload is
  used when the caller already has a timestep. `dt` is guarded by `max_dt_s`
  (default 0.10 s) so a long gap between calls cannot cause a derivative spike.
- **`reset()`** clears the integral and derivative history between runs.

`PIDConfig` fields: `kp, ki, kd, min_output, max_output, min_integral,
max_integral, max_dt_s`.

The navigation layer instantiates PID three times, with different gains, for:
heading hold during a corner (`turn_heading_pid`), speed→acceleration
(`speed_pid`), and inside the Stanley controller (`heading_pid`).

---

## 2. Stanley controller (`control::StanleyController`)

Standard Stanley steering law for the Ackermann front axle:

```
delta = heading_error
      + atan2( k · cross_track_error , speed + softening_speed )
```

- **`k`** (cross-track gain, default 1.0; Open Challenge uses 0.85) — how hard to
  correct lateral error.
- **`softening_speed_mps`** (0.20) — prevents divide-by-near-zero and overly
  aggressive steering at low speed.
- **`max_steering_rad`** (default 30°; Open Challenge widens to 45°) — physical
  steering limit; the output is clamped to it.
- The **heading term** is stabilised by an internal PID (`heading_pid`) rather
  than used raw, which damps oscillation.

Sign convention: **negative = LEFT, positive = RIGHT**, matching the navigation
and actuator layers.

---

## 3. Decision log (rubric criterion 4)

Visible in the code:

- **Stanley over pure-pursuit** — Stanley references the front axle and handles
  both cross-track and heading error directly, which suits a wall-following car
  with a LiDAR-measured wall line.
- **PID as a shared primitive** — one well-tested implementation (with
  anti-windup and dt guarding) is reused for heading, speed, and the Stanley
  heading term, instead of three ad-hoc loops.
- **Speed-softened cross-track term** — keeps steering sane at the low speeds
  used in SEARCH_DIRECTION and corner exits.

The source tree records the active gains and logs both raw and shaped steering,
but it does not include step-test results, a before/after comparison, or plots.
Those records should be added only from actual track runs.
