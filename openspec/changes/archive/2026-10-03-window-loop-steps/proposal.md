# Proposal

## Why

`compute_step_2` (`app/run_odometry.cpp`) is a single 640-line function.
- **Before the loop:** about 80 lines of setup.
- **Per window:** a loop body of about 530 lines doing about fifteen separate things: velocity estimate, initial guess, gyro-bias prior, turn detection, accelerometer gating, robust mode, the LM iterations, bias and offset updates, the tail hand-off, the preview, the NaN check, re-chaining the later windows, and the map update.
- **Carried state:** about ten local variables hold what passes from one window to the next.

None of these steps can be tested or changed alone. The saved "longer velocity span" change, for example, has to edit the middle of this loop.

This is roadmap step 6b, the second of the three changes the user split step 6 into on 2026-10-03.

## What Changes

- **The loop body becomes a sequence of named steps.** Each step is a function with explicit inputs and outputs. The code inside each step is moved, not rewritten, and keeps the same order of floating-point operations.
- **Pure steps go to `app/window_steps.{h,cpp}`.** They depend only on `Window`, Eigen and `imu_kinematics`:
  - `place_window_after`: chain a window's poses from a start pose. It replaces the three copies of this loop: the first window and the re-chaining of later windows.
  - `estimate_previous_velocity`: the velocity prior of a window, its age, and whether the `max_prior_velocity_mps` limit dropped it.
  - `predict_window_poses`: the initial guess. It chains from the previous window's end, then uses IMU integration, rest detection, constant velocity or the mean shift.
  - `apply_gyro_bias_prior`: the bias prior grown by the random walk, removed from the guess's rotations.
  - `is_turn_window`: the yaw rate of the guess against the threshold.
  - `repropagate_window_tail`: the hand-off of the last nodes.
- **Steps that use `OdometryState`, the map or the logger stay in `app/run_odometry.cpp`, as file-local functions:**
  - run setup: `OdometryRunSetup`, built once by `prepare_run`;
  - accelerometer-term gating and its update after the window;
  - robust mode;
  - the LM iteration loop;
  - the gyro-bias update and its log;
  - the NaN check;
  - the reference-map update;
  - the end-of-run summary.
- **State carried between windows** is one struct, `WindowLoopCarry`: carried bias, gravity vector, acceleration-offset covariance, degeneracy state, `acc_distance`, the map points and the counters. It replaces the scattered locals.
- **`compute_step_2` is renamed `optimize_windows`**, the domain name ARCHITECTURE.md asks for, in code and docs. The `OdometryPipeline` stage keeps its name `run_odometry`.
- **Tests on synthetic windows** (`tests/window_steps_test.cpp`) cover each pure step.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None. This is a pure refactor (`skip_specs: true`): console messages and results stay the same.

## Impact

- **Code:**
  - `app/run_odometry.{h,cpp}`: the loop drops from about 640 to about 200 lines.
  - New `app/window_steps.{h,cpp}`.
  - `app/odometry_pipeline.cpp`: the call site.
  - `CMakeLists.txt`.
- **Tests:** new `tests/window_steps_test.cpp`. The tests also compile `app/window_steps.cpp`, but still don't link `core`.
- **Results:**
  - `44_short` must stay bit-identical on every commit.
  - The full 44 must be identical with `tools/baseline/44.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_baseline`. Both were refreshed on 2026-10-03.
- **Docs:** `CLAUDE.md`, `docs/PIPELINE.md`, `docs/index.html` (the `where` lines) and `docs/ARCHITECTURE.md` (6b done).
