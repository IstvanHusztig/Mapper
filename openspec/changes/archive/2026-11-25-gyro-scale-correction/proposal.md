# Proposal

## Why

Stage 8 (`livox4-passage-and-flyaway`, tasks 3.5 and 3.11) found that the Livox gyro reads its yaw rate about 0.4 %
too low: the estimated yaw bias follows the turn rate, and on a copy of Livox 4 with the gyroZ column multiplied by
1.004 (`raw data/livox/4_gz0040`) that dependence is gone. The candidate settings with `ahrs_gain = 0` on that copy
gave the map the user called perfect (2026-11-19, `experiments/stage8/full_sheets/4_gain0_yaw_scaled.png`; wall
pieces seen twice disagree by 0.020 m p50 against 0.039 m, plan fit 79 % within 10 cm against 73 %). Editing a
recording is not a method: dMapper needs the scale as a setting so that the original recording gives the same map.

## What Changes

- New settings `gyro_scale_x`, `gyro_scale_y`, `gyro_scale_z` (new TOML section `[sensor_calibration]`, default 1.0):
  each raw gyro sample is multiplied per axis right after the IMU is loaded, before the orientation filter, the
  start-up bias and the windows see it. At 1.0 nothing is touched, so every existing result stays bit-identical.
- The run logs one line with the scale when any axis differs from 1.
- Snapshots and window captures count the three keys only when set (as the other settings added later).
- After confirmation runs and the user's look: `settings/livox-vertical.toml` gets the stage 8 set (exact NDT cell
  statistics, acceleration offset random walk 0.01, degeneracy projection 0.02 / 0.04, `ahrs_gain = 0`,
  `gyro_scale_z = 1.004`), with the golden TOML regenerated and the Livox baselines renewed. The L2 file is unchanged.
- Not in this change: a tool that fits the scale from a recording once and writes it as a calculated field.

## Capabilities

### New Capabilities

(none)

### Modified Capabilities

- `sensor-settings`: the gyro scale becomes a sensor setting applied to the raw samples; the Livox settings file
  carries the measured scale.

## Impact

- Code: `src/domain/odometry_config.h` (three members), `src/infrastructure/toml_io.h` (keys and the new section),
  `src/domain/imu_sample.h` (a function that scales the gyro of an `Imu`), `src/application/lidar_odometry.cpp`
  (`load_data` applies it), `src/infrastructure/odometry_snapshot_file.cpp` (keys counted only when set).
- Tests: a unit test of the scaling; the golden TOML files (new section in every saved file).
- Settings: `settings/livox-vertical.toml` only after the user's decision on the runs.
- Regions: front-end (loading) and inertial (what the scale means for the bias); knowledge files updated.
- Docs: the guide's orientation section and the settings reference.
