## Why

Between windows the odometry hands on three inertial states: the velocity, the acceleration offset (folded into the
gravity vector) and the gyro bias. On the Livox recordings the velocity is handed on as known to 5–11 mm/s whatever
happened before: after 5 s without the LiDAR the believed sigma is 10.7 mm/s, and the first windows that see again
change the speed by 0.3–0.65 m/s. After such a corridor the window resists the correction and the run stays slow for
seconds: the 1–3 m scatter between repeated Livox 4 runs (stage 8, section 5 of `livox4-passage-and-flyaway`).
Raising one noise setting (`imu_accel_floor_sigma_mps2`) recovers both corridors in every replay only at 0.5, is
decided by 0.1 % changes at 0.2–0.3, and leaves the speed loss inside the blind windows (90 replays,
`experiments/stage8/stalls/floor_calib_summary.md`). The code shows why a number cannot fix it: the motion edges state
the carried velocity a second time in every window, and the link between velocity and offset is dropped at every
hand-off. The target use (caves, long passages with nothing to see along the axis) needs an uncertainty that grows
while the LiDAR is blind and gives way when it sees again.

## What Changes

Two corrections of the model's logic and one extension that waits for a measurement, each behind a setting that
defaults to today's behaviour, each tested by replay before the next:

- **Layer 1: motion edges carry no velocity information.** Today the position rows of the edges tie every window to a
  guess that was integrated from the carried velocity and gravity: a second copy of the prior, with a fixed weight.
  With `imu_edges_follow_state = true` the edges' position measurement follows the window's start velocity and
  acceleration offset, so the edges keep the shape of the accelerometer's path and say nothing about the two states.
- **Layer 2: velocity and acceleration offset carried together.** Today each is handed on with its own 3×3
  covariance; the link between them (an offset error becomes a velocity error that grows with time) is dropped at
  every window. With `imu_carry_joint_covariance = true` the next window's prior is the joint 6×6 covariance.
- **Layer 3, only if measured: an acceleration error common to a window.** With `imu_accel_window_sigma_mps2 > 0` the
  window's offset gets that much extra prior uncertainty, which is not carried: only the slow part of the estimated
  offset is folded into gravity. Whether the accelerometer has such an error, and how large, is measured first; the
  user decides on the layer with that number.
- **A reading that says whether the carried states are honest** (`tools/carry_honesty.py`, from `window_features.csv`):
  prior sigma against the correction the next window makes, by information-ratio band; per blind stretch the growth of
  the sigma, the sign and size of the first correction after it, and how long the speed takes to settle.
- After the user's decision on the numbers and the clouds: values in the sensor files, new baselines, and the switches
  of accepted layers removed (rejected layers removed with their code).
- **Not in this change, measured by it:** the LiDAR's pull against travel in the first weak windows after a blind
  stretch, and what happens to the position there (the window's first pose is fixed). Design, D6 and Open Questions.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `lidar-odometry-motion-model`: new requirements on what the motion edges may assert, on how velocity and
  acceleration offset are carried, on the acceleration error common to a window, on the growth of the carried
  uncertainty through blind windows, and on the honesty reading. The existing requirements stay; the defaults keep
  today's results bit for bit.

## Impact

- Code: `src/domain/motion_model_terms.cpp` (edges), `accelerometer_term.cpp` (joint prior), `window_noise_model.{h,cpp}`
  (joint prior information), `window_problem.{h,cpp}` (joint covariance out of the solve and in its saved state),
  `src/application/run_odometry.cpp` (`prepare_acc_term`, `prepare_velocity_state`, `update_acc_term_after_window`),
  `window_loop.h` (`WindowLoopCarry`), `src/infrastructure/window_capture_file.cpp` (format version, old captures still
  read), `odometry_snapshot_file.cpp` (the settings hash's `added_later` list), `toml_io.h` and `odometry_config.h`
  (three settings, golden TOML files).
- Regions: inertial, noise-budget, solver, verification; the orchestrator for settings, carry and capture format.
- Results: unchanged with the defaults (44_short, the guards and the capture replays stay bit-identical). With the
  layers on, every Livox and L2 result changes; nothing goes into `settings/` before the user's decision.
- Time: a few more 3×3 blocks per window in the equations; no measurable cost expected, measured in the step checks.
