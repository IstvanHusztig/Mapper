# Design

## Context

The IMU CSV is read by `load_imu` (`src/infrastructure/recording_folder_source.cpp`) through the port
`RecordingSource::read_imu(chunk, logger)`; each value is parsed as a double and stored in `ImuSample::gyro`, a
float vector. `load_data` (`src/application/lidar_odometry.cpp`) merges the chunks; the orientation filter
(`calculate_trajectory`), the start-up bias and `compute_step_1` (window `imu_angles`, the bias, the accelerometer
term) all read that vector. The copy `raw data/livox/4_gz0040` was made in Python: the gyroZ text parsed as a double,
multiplied by 1.004, written back with full precision (repr).

The points already have a load-time correction of the same kind: `read_points(chunk, filter, logger)` takes a
`PointFilter` that carries the lever arm.

## Goals / Non-Goals

**Goals:**
- One place where the gyro scale is applied, before anything reads the gyro.
- A run of `raw data/livox/4` with `gyro_scale_z = 1.004` sees the gyro the copy `4_gz0040` gave, up to a rare
  one-float-step difference (see decision 1).
- Defaults bit-identical for every recording and settings file.

**Non-Goals:**
- Estimating the scale during the run (a state in the window problem) or the fitting tool (a later change).
- A scale for the accelerometer, or cross-axis terms (misalignment).
- Re-scaling in `--replay`: a window capture holds the windows' IMU data as loaded; a replay with a different scale
  does not re-scale them (the capture refuses nothing new; the knowledge file says so).

## Decisions

**1. Scale while parsing, in double precision, through the port.** `read_imu` gets a second argument, an
`ImuCalibration` (domain type in `imu_sample.h`: `Eigen::Vector3d gyro_scale`, default ones), next to
`read_points`' `PointFilter`. `load_imu` multiplies the parsed double before it is stored as float, on both the
current and the legacy CSV path, and only for an axis whose factor is not 1.0.
- Why: this is the arithmetic of the copy (double times factor, then one rounding to float). Scaling the stored float
  in `load_data` would round twice.
- Limit (design review 2026-11-19): the CSV library parses decimals with its own routine (`csv.hpp` `data_type`,
  digit by digit with repeated 0.1 products), which is not correctly rounded. Unscaled reads are unaffected after the
  float narrowing, but the copy's text and the original's text times 1.004 then differ by one float step in about
  179 of 183137 Livox 4 samples (an emulation of the parser; the run decides). So the setting gives nearly, not
  exactly, the copy's input; the runs are sensitive enough (near-identical runs end 1-1.6 m apart) that the
  confirmation run is effectively one more repeat member, judged with the others.
- Alternative: scale in `load_data` after the merge (application layer only, no port change). Rejected for the
  double rounding above. Alternative: scale the window rotations after the filter, as the 2026-11-18 experiment
  branch (`gyro_scale_correction_*`, commit 5869043) did. Rejected: the orientation filter and the start-up bias would
  still see the unscaled gyro, which is not what the map was made with.

**2. Factors, not corrections.** `gyro_scale_z = 1.004` (a factor, default 1.0) rather than the experiment's
`gyro_scale_correction_z = 0.004`. Reads as what the code does, and a fitted factor can be written as is.

**3. New section `[sensor_calibration]`.** The keys go into their own TOML section, which the later fitting tool
will write to (with its source line). Every saved TOML gains the section, so all golden TOML files are regenerated in
their own commit.

**4. Snapshots and captures.** The three keys join `added_later` in `odometry_snapshot_file.cpp` (counted in the
settings hash only when not at their default), so existing snapshots and captures stay valid.

**5. Who implements.** The changed code is the front-end region's (`lidar_odometry.cpp`, `imu_sample.h`,
`recording_folder_source.cpp`) plus the orchestrator's shared files (config, TOML, snapshot, port). It is small and
mechanical; the orchestrator implements it on a branch and records it in the front-end and inertial knowledge files,
instead of a region job.

**6. Shipping the Livox values only after confirmation.** The Livox file gets the whole stage 8 set at once, after:
the Livox 4 run with the setting compared with `candidate_gain0_livox4_gz0040` (no window at the time limit
`real_time_threshold_seconds`, else the comparison is timing-dependent); Livox 3 with the same set (its own scale measured
first from the bias-against-rate slope, `experiments/stage8/tools/s8_bias_compare.py`); repeat runs with tiny changes
on Livox 4 (`verify-robustness`); the user's look at both recordings.

## Risks / Trade-offs

- [The factor was fitted on Livox 4 and checked on the same recording] → Livox 3 is measured and run before shipping;
  if its slope does not point to about 1.004 the shipped value waits for the user's decision.
- [A slow IMU clock looks the same as a gyro scale] → named in the knowledge file; the clock check
  (`tools/imu_time_offset.py`, rate) stays an open question. The factor corrects the data either way.
- [The good map is one run] → repeat runs before shipping; the user decides on their spread.
- [`ahrs_gain = 0` alone made the end stretch run away in an earlier experiment build (stage 8 task 3.4)] → the Livox
  3 run and the repeats read the end stretch's largest speed.

## Migration Plan

Defaults change nothing; no migration. Old TOML files load (missing keys take the defaults). Rollback: remove the
keys from the Livox file.

## Open Questions

- Whether the L2 has a gyro scale error too: not measured; the L2 file keeps 1.0.
