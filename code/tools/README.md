# Offline tools

Python utilities for analysing a run's logs off the robot. They read the CSVs
written into each run directory (`telemetry.csv`, `walls.csv`, `corners.csv`,
`segments.csv`) plus `run_meta.json`.

## `plot_run.py`

```bash
python3 code/tools/plot_run.py <telemetry.csv> [out_basename]
```

Writes `<out_basename>.png` (top-down map: speed-coloured trajectory, TURNING
markers, obstacle pillars, per-lap overlay) and, when the columns exist,
`<out_basename>_signals.png` (validity-gated time series).

Two properties keep it honest as the schema evolves:

- **Columns are read from the file header**, not hard-coded. A log missing a
  column skips that panel with a note instead of crashing, so it renders both
  the current schema and older logs.
- **Series are gated on their `*_valid` flags** before plotting. A value whose
  flag is 0 means "not measured" (see the missing-value convention in
  `code/modules/logging/log_types.hpp`) and is drawn as a gap, never as a
  false zero. This is what stops wall distances from diving to zero through
  every corner.

Requires `numpy`, `pandas`, `matplotlib`.

> The old `gen_sample.py` synthetic-log generator was removed: it emitted a
> stale schema and a fabricated `obstacles.csv` the robot never writes, and real
> run logs now exist. Validate against a real run directory instead.
