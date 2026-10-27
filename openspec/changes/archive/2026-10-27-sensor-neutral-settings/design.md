# Design

## Context

The input format (`timestamp` ns, gyro rad/s, acc g, LAZ `gps_time` s) is already generic. A survey of the code found one sensor dependence that no TOML can change, and that is the start-up gyro bias in `calculate_trajectory` (`src/application/lidar_odometry.cpp:123-176`):
- it uses the first `1000` IMU samples;
- it takes the median at index `500`;
- it applies the variance gate `5.0` deg²/s².

At the L2's ~1 kHz that window is ~1 s; at a 200 Hz IMU it would be 5 s.

The other sample-counted values are already settings: `threshold_nr_poses` and `handoff_tail_nodes`. The L2 noise values and lever arm are code defaults, with L2 comments, and both shipped TOMLs set them. The AHRS first step `1/200` and the `dt` fallback `0.002` are generic and stay.

## Goals / Non-Goals

**Goals:**
- the start-up bias is configurable, with results bit-identical on the existing L2 data;
- sensor files are named after the sensor;
- a provisional Mid-360S file exists and loads.

**Non-Goals:**
- **Tuning the Mid-360S.** The IMU rate, noise and lever arm are measured from the first recordings in a later change.
- **Generalizing the developer markdown** (`CLAUDE.md`, `tools/README.md`, `docs/*.md`), tool docstrings and `openspec/changes/**`. The user decided this comes later, once the Mid-360S has been tested.
- **Changing the window or tail settings to seconds.**
- **Recording-side concerns** such as axis remapping and timestamps, which belong to the recorder.

## Decisions

- **The start-up window is counted in samples, not seconds.** `startup_gyro_bias_nr_samples` (int, default 1000) is counted like `threshold_nr_poses`. A setting in seconds would be more uniform across sensors, but recording 44 runs at ~890 Hz, so 1000 samples isn't a whole number of seconds there. Seconds would change the 44 baseline, and the change is meant to be bit-identical.
- **The median stays at index `n / 2`**, which is 500 for 1000, so the result is the same element as before.
- **The variance gate `startup_gyro_bias_max_variance_deg2_s2`** (double, default 5.0) keeps the existing formula: the sum of the per-axis variances, divided by n. Its value depends on the gyro noise, so it belongs in the TOML.
- **Plumbing:** `calculate_trajectory` gets two more parameters next to `use_remove_imu_bias_from_first_stationary_scan`, as its signature already passes each setting separately. Its callers are `odometry_pipeline.cpp:62` and `tests/orientation_test.cpp:66`. Alternatives considered: passing `OdometryConfig`, which is a larger change of the signature that the test fakes rely on, or a struct, which would be a new pattern for one call. Both were rejected.
- **Section:** both keys go in `[madgwick_filter]`, next to the switch they belong to (`toml_io.h:177`).
- **The code defaults stay at the L2 values.** Changing them would alter any run whose TOML lacks a key. Only their comments change, to "the sensor's IMU, set in its TOML". The L2 provenance moves into `l2-vertical.toml` comments: 43/static, the data sheet `acc_cov`, and the lever arm in the native frame.
- **Renames use `git mv`** for history: `lidar-vertical.toml` → `l2-vertical.toml`, `lidar_vertical.golden.toml` → `l2_vertical.golden.toml`, and `lidar-hardware-specs.md` → `l2-hardware-specs.md`.
- **The existing main specs** (`gui-workflow`, `processing-pipeline`, `run-progress`, `regression-baseline`, `recording-cut`, `lidar-odometry-motion-model`) name `settings/lidar-vertical.toml` only as the file their scenarios run with. The path is updated in place, without a delta, because no requirement changes. Their use of 44 as the regression dataset is developer procedure and stays.
- **`livox-vertical.toml`** is a copy of `l2-vertical.toml` with a "provisional" header and these values:
  - `filter_threshold_xy_outer = 40` (spec: 40 m at 10 %);
  - `filter_threshold_xy_inner = 0.35` (the operator's body);
  - IMU-rate values for the 200 Hz IMU (Mid-360S user manual §3.5): `threshold_nr_poses = 72`, `handoff_tail_nodes = 6`, `startup_gyro_bias_nr_samples = 200`;
  - the lever arm from the manual's IMU position, x = 11.0, y = 23.29, z = −44.12 mm in the point frame (IMU axes = point axes). It is given in dMapper's output axes with the vertical mount's axis mapping, using the same convention as the L2 entry. Its sign is checked on the first recording, because the offset is about 3 times the L2's;
  - TODO markers on the gyro ARW, the accel noise density and floor, `decimation`, and the variance gate, all of which are measured on the first static recording.

  The Mid-360S outputs a point without a return as (0, 0, 0). The inner xy filter already removes such points, which a unit test of `load_point_cloud`'s filter, or a comment, records.

  It gets its own golden TOML and test case.
- **User docs** (`README.md`, `docs/*.html`, `docs/hu/*.html`, code and test comments):
  - remove L2 and recording-44 history;
  - replace "L2" with "the LiDAR" or "the IMU";
  - name a settings file as "the sensor's TOML (`l2-vertical.toml` / `livox-vertical.toml`)".

  The English and Hungarian pages change in step.

## Risks / Trade-offs

- **Old commands and scripts with `settings/lidar-vertical.toml` break.** That includes the user's GUI habit and scratch scripts. → Mention the rename in README and in the result message. There is no compatibility copy, since a duplicate file would drift.
- **Odometry snapshots saved before the change may be refused,** because the two keys are new odometry settings. → Snapshots are temporary; rerun the odometry. This is noted in the task result.
- **The provisional Livox values may be wrong** (the IMU rate isn't in the spec sheet). → The header says so, and the TODOs name what to measure. Its first real use is a separate tuning change.
- **The doc rewrite could drop information a developer still needs.** → Developer markdown and tool docstrings are out of scope, and the facts remain in `l2-vertical.toml`, `l2-hardware-specs.md` and git history.

## Migration Plan

1. Rename `settings/lidar-vertical.toml` to `l2-vertical.toml` in commands, GUI loads and scripts.
2. Rollback is a plain revert. No data or result format changes.

## Open Questions

- **The vertical mount's axis mapping for the Mid-360S** (which native axis becomes dMapper's x, y and z). The user gives it. It only turns the manual's IMU position into the TOML's lever arm.
- **The Mid-360S noise values** come from the first static recording. They fill the TODOs in `livox-vertical.toml`, without changing this design.
