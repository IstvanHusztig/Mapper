# Proposal

## Why

The doubled stairwell railings of recording 44 are drift between floors. That is measured, with `tools/stairwell_check.py` and the building plan (`robust-loop-closure`, X-series, 2026-10-10). The cross-floor loop edges of `robust-loop-closure` (X3) tie the floors to 1–14 cm on all perturbed inputs (shipped: 18–69 cm). But `tools/facade_check.py` shows they distort the long gallery on floor 0. Its façade, straight on the plan, lies on average +49 / −64 / +103 cm off in X3 against +29 / −52 / +39 cm shipped on `44_s0/s1/s2`. So X3 can't ship, and the shipped map is already 30–50 cm off along the gallery.

Three questions are open, and today's guess-and-run approach is too slow and too error-prone for them:
1. **Why do the cross-floor edges move floor 0's gallery?** Measured so far:
   - floor 0's own gallery loops take the tension (residuals 6–8 cm against under 5 cm shipped);
   - the cross-floor edges attaching to gallery segments are well-conditioned (information ratio 0.20–0.57), so "façade-only matches" is refuted (X4).
2. **Why does the gallery itself drift?** The out-and-back loops along it need 1.7–1.9 m of correction, and the shipped result still bends it by 30–50 cm.
3. **Why does the odometry turn 2–6° on the straight stairs?** It's the root cause of the floor offsets, always in −x.
   - Not the accelerometer term: `imu_always_on = false` made it worse.
   - Not a degenerate stretch: the information ratio is 0.25–0.89 there.
   - Not the IMU timestamps: measured −5 to −11 ms on 44 and −1 to −7.5 ms on 45f, against 79 / 52 ms before the 2026-10-09 correction.

Each loop-closure experiment costs a full 25–35 min run today, although the odometry is bit-identical between them. And variants were judged on metrics that missed the problem (`ghost.py`, single inputs).

## What Changes

- **Reprocessing from an odometry snapshot.**
  - A run can save its finished odometry: the windows and the run state the later stages use.
  - A reprocessing entry point loads the recording's windows and points, restores the snapshot instead of running the odometry, then runs loop closure, consistency and saving.
  - The result is bit-identical to a full run with the same settings.
  - A loop-closure experiment then takes minutes, not half an hour.
- **A fixed evaluation suite** (`tools/evaluate_floors.py`), the one judge for every variant in this change:
  - runs `stairwell_check`, `facade_check`, `spot_check` (railing spot) and the loop summary on `44_s0`, `44_s1`, `44_s2` and `45f_p`;
  - prints one table against the shipped references;
  - `ghost.py` is not used for decisions.
- **Diagnosis 1, the graph:** for X3, per-segment corrections against shipped; leave-one-out and group experiments on the cross-floor edges (shaft pairs, gallery pairs, end-of-recording pairs). It finds which edges move floor 0's gallery, and why the graph puts the floors' correction there. A fix is proposed from the measurements, not chosen in advance.
- **Diagnosis 2, the gallery:** where along the gallery the drift accumulates (the façade per slice and per time), and whether the corridor is degenerate along its axis. The odometry's per-window diagnostics are compared with the façade metric.
- **Diagnosis 3, the stairs:** the heading per window over both trips down (LiDAR result against integrated gyro), the window where the 2–6° appear, and three cheap tests: an IMU shift of −6 ms, the gyro-bias handling during the descent, and the rotation prior's weighting in turns.
- **Each diagnosis ends in a written finding** in this change's `findings.md` (measurement, interpretation, confidence). A fix becomes its own change.
- `tools/trajectory_seams.py`: check whether it compares trajectories without the chunk poses (it reported 0.49 m where the world comparison shows 0.94 m), and fix or document it.

## Capabilities

### New Capabilities

### Modified Capabilities
- `processing-pipeline`: a run can save its odometry as a snapshot, and a reprocessing entry point runs the post-processing and saving from it with results identical to a full run.

## Impact

- `src/application/odometry_pipeline.{h,cpp}` (snapshot save and restore), a snapshot reader and writer in `src/infrastructure`, a small reprocessing executable (as `tools/cut_recording`) or a CLI flag, `src/domain/odometry_config.h` / `toml_io.h` (the snapshot setting) + golden TOML files.
- `tests/`: a round-trip test of the snapshot (save, restore, post-process gives the same result as without the detour), on the synthetic loop recording.
- `tools/evaluate_floors.py`, `tools/facade_check.py` (already written in `robust-loop-closure`), `tools/README.md`.
- No change to shipped results: the snapshot is off by default, and the diagnoses only read results.
- Depends on the uncommitted work of `robust-loop-closure` (cross-floor code, tools). That work is committed first, as it stands, with everything off by default.
