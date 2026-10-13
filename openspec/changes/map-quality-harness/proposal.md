# Proposal

## Why

A code review on 2026-10-13 found about 26 defects (`docs/code-review-tracker.md`) after two days of symptom measurements on recording 44 had not converged. The symptoms: the odometry's heading drifts ≈ −4° between the start room and the gallery, final maps show doubled walls and a smeared railing, and near-identical inputs (s0/s1/s2) give very different maps.

Before anything is fixed, every fix needs the same yardstick: the three map-quality measures of equal weight (global geometry, consistency = no doubled walls, sharpness), the run-to-run noise floor, and log lines for the silent events the review found (steps skipped, windows dropped, map rebuilt). This change builds that ground work. The fixes follow in changes B–I (plan of 2026-10-13).

## What Changes

- **Diagnostics (bit-identical):**
  - with `log_window_diagnostics`, one line per window with the heading change the LiDAR applies against the IMU prediction, and the window's yaw stiffness;
  - always, a log line and a per-run counter for each silent event: the sliding-map rebuild firing, a window dropped for too few points, a solver step skipped, a "big jump" partial update.
- **Harness `tools/map_quality.py`** (pure Python + numpy):
  - one command per result folder, or per set s0/s1/s2, prints one table: façade angle start room vs gallery, doubled walls at the revisits of the final map, railing and stairwell checks, pose jitter, and the s0/s1/s2 spread;
  - writes a plan-overlay PNG;
  - the analysis scripts it needs (façade lines, revisit registration, plan anchoring) move from the job folder into `tools/`, with README entries.
- **Noise floor and two config-only experiments**, recorded in this change: the current master on s0/s1/s2 (full and odometry-only), a `/fp:fast` probe, runs with consistency off or fewer passes, and runs with the map rebuild off.
- **Tracker** `docs/code-review-tracker.md`: one row per finding with its status, updated by every following change.

## Capabilities

### New Capabilities

- `map-quality-evaluation`: how a result is judged (the three measures, the s0/s1/s2 spread, the noise floor), and the harness that computes them.

### Modified Capabilities

- `run-progress`: silent events in the odometry are reported as log lines and counted per run.

## Impact

- **Code:**
  - `src/application/run_odometry.cpp`: diagnostic and event log lines, counters;
  - `src/domain/scan_to_map.cpp`: report a skipped step or a big-jump partial update;
  - `src/application/lidar_odometry.cpp`: dropped windows.
- **Tools:** `tools/map_quality.py` and its helpers; `tools/README.md`.
- **Results:** bit-identical (`44_short` against `tools/baseline/44_short.manifest.json`); only the console log changes.
- **Docs:** `docs/code-review-tracker.md`; CLAUDE.md's testing section names the harness.
