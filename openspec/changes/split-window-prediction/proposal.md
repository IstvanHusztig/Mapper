# Proposal

## Why

Stage 6 of `docs/code-split-and-region-agents-plan.md` asks for no function above about 80 lines in the window loop and
its terms (tier 1). A measurement of every function in `src/` leaves one there: `predict_window_poses` (84 lines). It
holds a copy of `place_window_after`'s chaining, and the zero-velocity test it runs is written out twice more in
`run_odometry.cpp`, each with its own copy of the 0.025 s smoothing constant. The other long functions belong to the
front-end, map and back-end regions and are split in each region's first job.

## What Changes

- **`predict_window_poses` in four steps** (pure refactor): the mean shift between the two previous windows, the chaining
  onto the previous window's end through `place_window_after` (the copy goes), the rest test, and the translation guess
  (accelerometer integration or rest, constant velocity, mean shift). An empty window keeps today's behaviour by a guard.
- **One rest test** (`window_rests`): the acceleration on a 50 ms average and the stationarity test, with the smoothing
  half width as one named constant, used by the prediction, `window_at_rest` and `log_accelerometer_fit`. The size checks
  and the gyro threshold's conversion stay at the callers.
- **`tools/function_lengths.py`**: lists every function above a line limit in C++ files or folders, for the verification
  region and the region contracts of stage 7.
- **The plan's table** of the long functions, measured, with the region and the stage each is split in.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `unit-tests`: the window prediction's steps are tested on their own.

## Impact

- **Code:** `src/domain/window_steps.{h,cpp}`, `src/application/run_odometry.cpp`; `tests/window_steps_test.cpp`;
  `tools/function_lengths.py`, `tools/README.md`.
- **Results:** none. Every commit is bit-identical (characterization guard, loop guard, replays, 44_short, the per-term
  table); the full 44 and Livox 3 are IDENTICAL with the current manifests (no setting changes).
- **Not in this change:** the front-end, map and back-end functions (split in their region's first job), the I/O and GUI
  functions (when next changed); no Debug build (a pure split).
