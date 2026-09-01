#!/usr/bin/env python3
"""plot_run.py -- offline analyzer for a robot run.

Reads one telemetry.csv (the only file it needs) and renders:

  <out>.png          top-down map: trajectory coloured by speed, TURNING
                     markers, obstacle pillars, per-lap overlay
  <out>_signals.png  validity-gated time series of the wall-following and
                     command signals (only written if those columns exist)

Usage:  python3 plot_run.py <telemetry.csv> [out_basename]

Two rules make this robust against schema drift, which is the whole reason it
was moved into the repo:

  1. Column names are discovered from the file header, not hard-coded. A log
     that lacks a column degrades to an empty/omitted panel with a printed
     note -- never a KeyError. It reads both the current schema and older ones.

  2. Every numeric series is gated on its *_valid flag before plotting (see
     the missing-value convention in log_types.hpp): a 0 that means "not
     measured" is drawn as a gap, not as a real data point. This is what stops
     wall distances from diving to zero through every corner.
"""
import sys

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

FIELD_HALF_M = 1.6  # half of a ~3.2 m WRO field; only affects the dashed box

# value column -> the boolean column that says whether it was measured. A value
# whose flag is 0 (or NaN) is blanked so a gap is drawn instead of a false zero.
VALIDITY = {
    "outer_distance_m": "outer_wall_valid",
    "inner_distance_m": "inner_wall_valid",
    "distance_error_m": "wall_following_active",
    "angle_error_rad": "wall_following_active",
    "wall_corner_forward_m": "turn_trigger_evaluated",
    "effective_turn_trigger_m": "turn_trigger_evaluated",
    "battery_voltage_v": "battery_valid",
    "camera_process_us": "camera_process_valid",
}


def gated(t, value_col):
    """Series with NaN where the value was not actually measured this tick."""
    values = pd.to_numeric(t[value_col], errors="coerce").astype(float)
    flag_col = VALIDITY.get(value_col)
    if flag_col is None or flag_col not in t.columns:
        if flag_col is not None:
            print(f"note: no '{flag_col}' column; plotting '{value_col}' "
                  f"ungated (cannot tell measured 0 from missing)")
        return values
    valid = pd.to_numeric(t[flag_col], errors="coerce").fillna(0) != 0
    return values.where(valid)


def require(t, cols, panel):
    missing = [c for c in cols if c not in t.columns]
    if missing:
        print(f"note: skipping {panel}: missing {', '.join(missing)}")
        return False
    return True


def speed_coloured_path(ax, x, y, speed):
    pts = np.array([x, y]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    finite = speed[np.isfinite(speed)]
    lo, hi = (float(finite.min()), float(finite.max())) if len(finite) else (0.0, 1.0)
    lc = LineCollection(segs, cmap="viridis", norm=plt.Normalize(lo, max(hi, lo + 1e-6)))
    lc.set_array(speed[:-1])
    lc.set_linewidth(2.5)
    ax.add_collection(lc)
    return lc


def obstacle_points(t):
    """Deduplicated obstacle pillars from the inline telemetry columns."""
    cols = ["obstacle_active", "obstacle_world_x_m", "obstacle_world_y_m",
            "obstacle_color"]
    if not all(c in t.columns for c in cols):
        return pd.DataFrame(columns=["x", "y", "color"])
    active = t[pd.to_numeric(t.obstacle_active, errors="coerce").fillna(0) != 0]
    if not len(active):
        return pd.DataFrame(columns=["x", "y", "color"])
    o = pd.DataFrame({
        "x": pd.to_numeric(active.obstacle_world_x_m, errors="coerce"),
        "y": pd.to_numeric(active.obstacle_world_y_m, errors="coerce"),
        # logged as 0 none / 1 RED / 2 GREEN
        "color": pd.to_numeric(active.obstacle_color, errors="coerce")
        .map({1: "RED", 2: "GREEN"}),
    }).dropna(subset=["x", "y", "color"])
    o["kx"] = o.x.round(1)
    o["ky"] = o.y.round(1)
    return o.groupby(["kx", "ky", "color"], as_index=False).first()


def draw_field(ax):
    ax.add_patch(plt.Rectangle((-FIELD_HALF_M, -FIELD_HALF_M), 2 * FIELD_HALF_M,
                 2 * FIELD_HALF_M, fill=False, ls="--", ec="grey"))
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def draw_map(t, out_png):
    if not require(t, ["pos_x_m", "pos_y_m", "measured_speed_mps", "mode", "lap"],
                   "map"):
        return
    x = pd.to_numeric(t.pos_x_m, errors="coerce").to_numpy()
    y = pd.to_numeric(t.pos_y_m, errors="coerce").to_numpy()
    speed = pd.to_numeric(t.measured_speed_mps, errors="coerce").to_numpy()
    obs = obstacle_points(t)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7.2))

    lc = speed_coloured_path(ax1, x, y, speed)
    fig.colorbar(lc, ax=ax1, label="speed (m/s)", fraction=0.046)
    turning = t[t["mode"] == "TURNING"]
    if len(turning):
        ax1.scatter(pd.to_numeric(turning.pos_x_m, errors="coerce"),
                    pd.to_numeric(turning.pos_y_m, errors="coerce"),
                    s=8, c="orange", alpha=0.35, label="TURNING", zorder=2)
    for _, r in obs.iterrows():
        ax1.scatter(r.x, r.y, s=220, marker="s", edgecolors="black", zorder=5,
                    c=("red" if r.color == "RED" else "green"))
    ax1.scatter([x[0]], [y[0]], s=120, c="black", marker="*", zorder=6,
                label="start")
    draw_field(ax1)
    ax1.set_title("Trajectory (speed) + obstacles")
    ax1.legend(loc="upper right", fontsize=8)

    for lap, g in t.groupby("lap"):
        ax2.plot(pd.to_numeric(g.pos_x_m, errors="coerce"),
                 pd.to_numeric(g.pos_y_m, errors="coerce"),
                 lw=1.8, alpha=0.8, label=f"lap {lap}")
    for _, r in obs.iterrows():
        ax2.scatter(r.x, r.y, s=120, marker="s", edgecolors="black", zorder=5,
                    c=("red" if r.color == "RED" else "green"))
    draw_field(ax2)
    ax2.set_title("Per-lap overlay (repeatability)")
    ax2.legend(loc="upper right", fontsize=8)

    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print("wrote", out_png)


def draw_signals(t, out_png):
    """Validity-gated time series. This is the panel that exposes the
    zero-fill artefact: gated wall distances show gaps through corners."""
    if "timestamp_us" not in t.columns:
        print("note: skipping signals: missing timestamp_us")
        return
    ts = pd.to_numeric(t.timestamp_us, errors="coerce")
    time_s = (ts - ts.iloc[0]) / 1e6

    panels = []
    if require(t, ["outer_distance_m", "inner_distance_m"], "wall-distance panel"):
        panels.append(("wall distance (m)", [
            ("outer_distance_m", "outer"), ("inner_distance_m", "inner")]))
    if "raw_steering_rad" in t.columns and "steering_rad" in t.columns:
        panels.append(("steering (rad)", [
            ("raw_steering_rad", "raw"), ("steering_rad", "shaped")]))
    if "target_speed_mps" in t.columns and "measured_speed_mps" in t.columns:
        panels.append(("speed (m/s)", [
            ("target_speed_mps", "target"), ("measured_speed_mps", "measured")]))
    if not panels:
        print("note: skipping signals: no plottable columns")
        return

    fig, axes = plt.subplots(len(panels), 1, figsize=(13, 3.0 * len(panels)),
                             sharex=True, squeeze=False)
    for ax, (ylabel, series) in zip(axes[:, 0], panels):
        for col, label in series:
            ax.plot(time_s, gated(t, col), lw=1.2, label=label)
        ax.set_ylabel(ylabel)
        ax.legend(loc="upper right", fontsize=8)
        ax.grid(alpha=0.25)
    axes[-1, 0].set_xlabel("time (s)")
    axes[0, 0].set_title("Validity-gated signals (gaps = not measured)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print("wrote", out_png)


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: python3 plot_run.py <telemetry.csv> [out_basename]")
    tele_path = sys.argv[1]
    out_base = sys.argv[2] if len(sys.argv) > 2 else "run"
    if out_base.endswith(".png"):
        out_base = out_base[:-4]

    t = pd.read_csv(tele_path)
    print(f"loaded {len(t)} rows, {len(t.columns)} columns from {tele_path}")
    draw_map(t, out_base + ".png")
    draw_signals(t, out_base + "_signals.png")


if __name__ == "__main__":
    main()
