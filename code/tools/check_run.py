#!/usr/bin/env python3
"""check_run.py -- automated per-run QA for a robot log.

Reads a run directory (or a telemetry.csv directly) and reports data-integrity
and plausibility problems, so a bad run is caught by a script instead of by
eye. Every defect this was built against was originally found by hand.

Usage:  python3 check_run.py <run_dir | telemetry.csv>

Exit status is non-zero if any check FAILs (SKIP and WARN do not fail), so it
can gate a pipeline.

Checks:
  A  a column that should carry live geometry is ~0 for >95% of a mode's rows
  B  gaps in row_index (a silently dropped telemetry row)
  C  non-zero writer drop counts recorded in run_meta.json
  D  raw_update_dt_s samples above max_update_period_s (a stalled loop)
  E  median(measured_speed_mps / target_speed_mps) outside [0.85, 1.15]
  F  telemetry header field count disagreeing with run_meta.json
  G  a *_valid flag contradicting its value (valid but zero / invalid but set)
"""
import json
import os
import sys

import numpy as np
import pandas as pd

EPS = 1e-9
SPEED_RATIO_LO, SPEED_RATIO_HI = 0.85, 1.15
ZERO_FILL_FRACTION = 0.95
MIN_MODE_ROWS = 10
DEFAULT_MAX_DT_S = 0.12

# Columns that a healthy run keeps varying in the listed modes. A whole mode
# reading exactly zero means either the pre-O-01 zero-fill bug or that the
# geometry was never seen -- both are worth a human's attention.
EXPECTED_LIVE = {
    "outer_distance_m": ("NORMAL", "TURNING"),
    "inner_distance_m": ("NORMAL", "TURNING"),
    "wall_angle_rad": ("NORMAL", "TURNING"),
    "distance_error_m": ("NORMAL",),
    "angle_error_rad": ("NORMAL",),
}

# value column -> validity flag, for the contradiction check.
VALIDITY = {
    "outer_distance_m": "outer_wall_valid",
    "inner_distance_m": "inner_wall_valid",
    "distance_error_m": "wall_following_active",
    "battery_voltage_v": "battery_valid",
}


class Report:
    def __init__(self):
        self.failed = False

    def _line(self, tag, msg):
        print(f"[{tag}] {msg}")

    def ok(self, msg):
        self._line("PASS", msg)

    def skip(self, msg):
        self._line("SKIP", msg)

    def warn(self, msg):
        self._line("WARN", msg)

    def fail(self, msg):
        self._line("FAIL", msg)
        self.failed = True


def num(df, col):
    return pd.to_numeric(df[col], errors="coerce")


def check_zero_fill(t, r):
    any_checked = False
    for col, modes in EXPECTED_LIVE.items():
        if col not in t.columns or "mode" not in t.columns:
            continue
        for mode in modes:
            rows = t[t["mode"] == mode]
            if len(rows) < MIN_MODE_ROWS:
                continue
            any_checked = True
            zero_frac = (num(rows, col).abs() < EPS).mean()
            where = f"{col} in {mode} ({len(rows)} rows)"
            if zero_frac > ZERO_FILL_FRACTION:
                r.fail(f"A: {where} is exactly 0 in "
                       f"{zero_frac * 100:.0f}% of rows -- not measured, or "
                       f"logged as a false zero")
            else:
                r.ok(f"A: {where} varies ({zero_frac * 100:.0f}% zero)")
    if not any_checked:
        r.skip("A: no expected-live columns/modes present")


def check_row_index(t, r):
    if "row_index" not in t.columns:
        r.skip("B: no row_index column (pre-Stage-A log)")
        return
    idx = num(t, "row_index").dropna().astype("int64").to_numpy()
    if len(idx) < 2:
        r.skip("B: too few rows")
        return
    diffs = np.diff(idx)
    gaps = int((diffs > 1).sum())
    dropped = int((diffs[diffs > 1] - 1).sum())
    if gaps:
        r.fail(f"B: {gaps} gap(s) in row_index, {dropped} row(s) missing "
               f"(telemetry queue overflow)")
    else:
        r.ok("B: row_index is contiguous")


def check_drop_counts(meta, r):
    logging = (meta or {}).get("logging")
    if not logging:
        r.skip("C: no logging drop counts in run_meta.json")
        return
    nonzero = {k: v for k, v in logging.items()
               if k.endswith("_dropped_rows") and v}
    if nonzero:
        r.fail("C: writer dropped rows: " +
               ", ".join(f"{k}={v}" for k, v in nonzero.items()))
    else:
        r.ok("C: no dropped rows recorded")


def check_dt(t, meta, r):
    if "raw_update_dt_s" not in t.columns:
        r.skip("D: no raw_update_dt_s column")
        return
    max_dt = DEFAULT_MAX_DT_S
    nav = (meta or {}).get("navigation_config", {})
    if isinstance(nav, dict) and "max_update_period_s" in nav:
        max_dt = float(nav["max_update_period_s"])
    dt = num(t, "raw_update_dt_s").dropna()
    over = int((dt > max_dt + EPS).sum())
    if over:
        r.fail(f"D: {over} tick(s) with raw_update_dt_s > {max_dt:.3f}s "
               f"(max {dt.max():.3f}s) -- loop stalled")
    else:
        r.ok(f"D: loop period within {max_dt:.3f}s (max {dt.max():.3f}s)")


def check_speed_ratio(t, r):
    if "measured_speed_mps" not in t.columns or "target_speed_mps" not in t.columns:
        r.skip("E: speed columns absent")
        return
    target = num(t, "target_speed_mps")
    measured = num(t, "measured_speed_mps")
    moving = target > 0.05
    if moving.sum() < MIN_MODE_ROWS:
        r.skip("E: too few moving rows")
        return
    ratio = (measured[moving] / target[moving]).replace([np.inf, -np.inf], np.nan)
    med = float(ratio.median())
    if not (SPEED_RATIO_LO <= med <= SPEED_RATIO_HI):
        r.fail(f"E: median measured/target speed = {med:.2f}, outside "
               f"[{SPEED_RATIO_LO}, {SPEED_RATIO_HI}] -- drivetrain scale error")
    else:
        r.ok(f"E: median measured/target speed = {med:.2f}")


def check_field_count(t, meta, header_len, r):
    schema = (meta or {}).get("schema", {})
    expected = schema.get("telemetry_field_count") if isinstance(schema, dict) else None
    if expected is None:
        r.skip("F: no schema.telemetry_field_count in run_meta.json")
        return
    if int(expected) != header_len:
        r.fail(f"F: telemetry header has {header_len} fields, run_meta.json "
               f"says {expected} -- schema/binary mismatch")
    else:
        r.ok(f"F: header field count matches run_meta.json ({header_len})")


def check_valid_contradiction(t, r):
    checked = False
    for col, flag in VALIDITY.items():
        if col not in t.columns or flag not in t.columns:
            continue
        checked = True
        value = num(t, col)
        valid = num(t, flag).fillna(0) != 0
        valid_zero = int((valid & (value.abs() < EPS)).sum())
        invalid_set = int((~valid & (value.abs() >= EPS)).sum())
        if valid_zero or invalid_set:
            r.fail(f"G: {col}/{flag} contradiction -- {valid_zero} valid-but-0, "
                   f"{invalid_set} invalid-but-nonzero")
        else:
            r.ok(f"G: {col}/{flag} consistent")
    if not checked:
        r.skip("G: no value/flag pairs present")


def main():
    if len(sys.argv) < 2:
        sys.exit("usage: python3 check_run.py <run_dir | telemetry.csv>")
    arg = sys.argv[1]
    if os.path.isdir(arg):
        tele_path = os.path.join(arg, "telemetry.csv")
        meta_path = os.path.join(arg, "run_meta.json")
    else:
        tele_path = arg
        meta_path = os.path.join(os.path.dirname(arg) or ".", "run_meta.json")

    if not os.path.exists(tele_path):
        sys.exit(f"no telemetry.csv at {tele_path}")

    with open(tele_path) as f:
        header_len = len(f.readline().rstrip("\n").split(","))
    t = pd.read_csv(tele_path)
    meta = None
    if os.path.exists(meta_path):
        try:
            with open(meta_path) as f:
                meta = json.load(f)
        except (OSError, json.JSONDecodeError) as exc:
            print(f"note: could not read {meta_path}: {exc}")

    print(f"checking {tele_path}: {len(t)} rows, {header_len} columns"
          + (", run_meta.json present" if meta else ", no run_meta.json"))
    r = Report()
    check_zero_fill(t, r)
    check_row_index(t, r)
    check_drop_counts(meta, r)
    check_dt(t, meta, r)
    check_speed_ratio(t, r)
    check_field_count(t, meta, header_len, r)
    check_valid_contradiction(t, r)

    print("RESULT:", "FAIL" if r.failed else "PASS")
    sys.exit(1 if r.failed else 0)


if __name__ == "__main__":
    main()
