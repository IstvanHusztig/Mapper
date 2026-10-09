# lidar-odometry-motion-model Specification

## Purpose

Defines how LiDAR odometry uses the IMU to constrain consecutive trajectory nodes within each optimization window: the rotation prior, online gyro bias estimation carried across windows, the hand-off between windows, the limit on the velocity carried from one window to the next, and how bias estimates are reported.

## Requirements

### Requirement: Time-scaled rotation prior
When time-scaled rotation sigma is enabled, the rotation uncertainty allowed between two consecutive trajectory nodes SHALL grow with the square root of their time separation. Per axis, it SHALL be `max(k * ARW_axis * sqrt(dt), sigma_floor)`, where `ARW_axis`, `k` and `sigma_floor` are configurable. When it is disabled, the constant per-node rotation sigmas SHALL be used as before.

#### Scenario: Short node gap gets less rotation freedom
- **WHEN** two consecutive nodes are 0.2 ms apart and another pair is 2 ms apart, with time-scaled sigma enabled
- **THEN** the allowed rotation sigma for the 0.2 ms pair is smaller than for the 2 ms pair, in the ratio sqrt(0.2/2), unless limited by `sigma_floor`

#### Scenario: Floor protects near-zero gaps
- **WHEN** two consecutive nodes have a time separation of 0 or near 0
- **THEN** the rotation sigma equals `sigma_floor` and the optimization completes without numerical failure

#### Scenario: Map quality at least matches the tuned constant sigma
- **WHEN** datasets 43 and 42 (repaired IMU timestamps) are processed with `settings/lidar-vertical.toml` (default time-scaled sigma, k = 4)
- **THEN** on each dataset the median node-to-node yaw change is at most 0.03°, no window has an end-of-window yaw spike above 3°, and the occupied 3 cm cells in the Z section [-0.3, 0.7] m do not exceed the constant-sigma-0.02 baseline (43: 15,945, 42: 6,872)

### Requirement: Online gyro bias estimation
When gyro bias estimation is enabled, the system SHALL estimate a 3-axis gyro bias for every optimization window after the first while the sensor is moving, with no stationary period required. It SHALL remove the estimated bias from the IMU rotation increments used in that window. Bias estimation SHALL be enabled by default.

#### Scenario: Bias matches a static recording
- **WHEN** a recording of at least 15 s with the sensor lying still (`raw data/43/static`) is processed with default parameters
- **THEN** each axis of the estimated bias, averaged over the windows after the first 5 s, is within ±0.5 °/s of that axis' gyro mean over the recording (43/static: x +0.03, y −0.01, z −0.61 °/s)

#### Scenario: Bias on handheld data
- **WHEN** datasets 42 and 43 are processed with default parameters
- **THEN** on 43 the z-axis bias averaged over the windows after the first 5 s is within ±0.5 °/s of the static-recording z bias (−0.61 °/s, same unit), on 42 it is negative, and no bias clamp warning is logged on either dataset

#### Scenario: Map quality with bias estimation
- **WHEN** datasets 42 and 43 are processed with default parameters (bias estimation enabled)
- **THEN** the occupied 3 cm cells exceed those of the same run with bias estimation disabled by at most 5 %

#### Scenario: Bias estimate is stable
- **WHEN** the static recording is processed with default parameters
- **THEN** the median window-to-window change of the z-axis bias after the first 5 s is at most 0.3 °/s

#### Scenario: No stationary start required
- **WHEN** a recording starts with the operator already moving, so the startup stationary-bias check reports excessive movement
- **THEN** bias estimation still runs: the first window keeps the prior, and every later window produces a bias estimate

#### Scenario: Disabled reproduces previous behaviour
- **WHEN** gyro bias estimation, time-scaled sigma and tail re-propagation are all disabled
- **THEN** the trajectory and map quality metrics are within the run-to-run spread of the pre-change code for the same input and parameters (the pre-change code is not reproducible run to run, so a byte comparison against it is not possible)

### Requirement: Reproducible results
Processing the same input with the same parameters twice SHALL produce byte-identical trajectory and point cloud outputs. The window optimization SHALL compute in parallel, and its result SHALL be byte-identical to a sequential computation of the same window. Parallel computation is built in and is not a setting.

#### Scenario: Two runs are identical
- **WHEN** a dataset is processed twice with identical parameters and no window stopped by `real_time_threshold_seconds`
- **THEN** the trajectory and point cloud outputs of both runs are byte-identical

#### Scenario: Parallel equals sequential
- **WHEN** a window is optimized once with the parallel node loop and once with the sequential one
- **THEN** the resulting poses, gyro bias and translation information are bit-identical

#### Scenario: No multithreading setting
- **WHEN** parameters are loaded from a TOML file that still contains `useMultithread`, or saved to one
- **THEN** the key is ignored on loading and not written on saving

### Requirement: Bias continuity across windows
The bias estimate and its uncertainty SHALL be carried from each window to the next as a prior, and the uncertainty SHALL grow with elapsed time according to a configurable bias random-walk. The bias SHALL be limited to a configurable maximum magnitude per axis.

#### Scenario: Feature-poor window does not reset the bias
- **WHEN** a window contains too few LiDAR constraints to determine the bias on its own
- **THEN** the bias estimate for that window stays within the propagated prior uncertainty of the previous window's estimate

#### Scenario: Bias clamp
- **WHEN** the estimated bias on any axis would exceed the configured maximum
- **THEN** it is limited to the maximum and a warning naming the window is logged

### Requirement: Robust window hand-off
The pose that starts a window SHALL NOT be taken from a trajectory node whose orientation is determined by fewer LiDAR constraints than the configured tail length implies. The trailing nodes of each window SHALL be re-derived from the last well-constrained node using the bias-corrected IMU rotation, and the start of the next window SHALL be derived from them.

#### Scenario: No heading jump at window boundaries
- **WHEN** dataset 43 is processed with default settings
- **THEN** in no window does the yaw of the last node differ by more than 3° from the median yaw of the nodes 30 to 60 before it

#### Scenario: Tail re-propagation can be disabled
- **WHEN** the tail length is set to 0
- **THEN** the next window starts from the optimized last node, as before this change

### Requirement: Bias reporting
The processing results SHALL include, for every window, the window start time, the estimated 3-axis gyro bias in °/s, and its 1-sigma uncertainty in °/s.

#### Scenario: Results file contains bias track
- **WHEN** a run completes with bias estimation enabled
- **THEN** the processing-results JSON in the output folder contains one bias entry per processed window, in time order

### Requirement: Configurable through TOML
All parameters introduced by this capability SHALL be loadable from and saved to the parameters TOML file. A TOML file that lacks them SHALL load with the documented defaults.

#### Scenario: Old TOML still loads
- **WHEN** a TOML file written before this change is loaded
- **THEN** loading succeeds and the new parameters take their default values

#### Scenario: Saved parameters round-trip
- **WHEN** a run saves its parameters TOML and that file is loaded for a new run
- **THEN** all new parameters have the same values as in the first run

### Requirement: Velocity prior limit
For each window after the first, the odometry SHALL estimate the previous window's final velocity. It SHALL use that velocity as a prior for the window only when its magnitude is at most `max_prior_velocity_mps`. The setting is in the `[motion_model_uncertainty]` section of the parameters TOML, and its built-in default is 10.0 m/s. A faster velocity SHALL NOT be used as a prior: the window SHALL then be processed as if no previous velocity were available, and a console message SHALL report the speed and the limit. The setting SHALL be loaded from and saved to the parameters TOML like the other settings of this capability.

#### Scenario: Velocity within the limit is used
- **WHEN** the previous window ends at 3 m/s and `max_prior_velocity_mps` is 5.0
- **THEN** the window uses the 3 m/s velocity as its prior

#### Scenario: Velocity above the limit is dropped
- **WHEN** the previous window ends at 7 m/s and `max_prior_velocity_mps` is 5.0
- **THEN** the window uses no velocity prior, and the console shows the 7 m/s speed and the 5 m/s limit

#### Scenario: TOML without the key keeps the old limit
- **WHEN** a parameters TOML that has no `max_prior_velocity_mps` key is loaded
- **THEN** the limit is 10.0 m/s, as it was before this setting existed

### Requirement: Degeneracy-aware NDT terms
The scan-to-map optimization SHALL decompose each window's geometric translation information, the sum of n·nᵀ over the surface normals its points are matched to. For every eigen-direction it SHALL compute the direction's share of the information: its eigenvalue divided by the largest eigenvalue. Along a direction whose share is below `degenerate_projection_ratio`, the NDT point terms SHALL carry no translation information, so that the motion model alone places the window along that direction. Between `degenerate_projection_ratio` and `degenerate_projection_ratio_full` the NDT translation information along the direction SHALL be scaled linearly from zero to its full value. Rotations and all other terms SHALL be unchanged. The settings are in the `[imu_translation]` section of the parameters TOML. Their built-in defaults are 0.0, and `degenerate_projection_ratio_full = 0.0` SHALL leave the NDT terms exactly as before. The result SHALL NOT depend on the number of threads.

#### Scenario: Blind direction left to the motion model
- **WHEN** a window in a corridor that is open along x is optimized with `degenerate_projection_ratio = 0.1` and `degenerate_projection_ratio_full = 0.2`, and its guess is shifted along x and across it
- **THEN** the shift along x is kept and the error across x is corrected

#### Scenario: Constrained window unchanged
- **WHEN** a window in a closed room, whose smallest share is about 0.48, is optimized with `degenerate_projection_ratio = 0.1` and `degenerate_projection_ratio_full = 0.2`
- **THEN** the result is bit-identical to the result with the setting off

#### Scenario: Off by default
- **WHEN** a parameters TOML without the two keys is loaded
- **THEN** both are 0.0 and the odometry gives the same results as before the setting existed

### Requirement: Loop closure cannot wreck a run
The loop closure SHALL solve its segment pose graph with steps that do not raise the weighted squared error of the graph. The angle residuals SHALL be wrapped into (-π, π]. Before applying the result, it SHALL check that every corrected segment pose is finite and that no segment moved further than `loop_closure_max_correction_m` plus the largest accepted loop correction. A result that fails the check SHALL be discarded with a warning, and the trajectory SHALL stay as the odometry left it.

#### Scenario: Consistent loop applied
- **WHEN** a loop of segments drifts 5 cm per segment and one exact loop edge closes it
- **THEN** the drift is removed and the result passes the check

#### Scenario: Runaway result refused
- **WHEN** the pose graph result moves a segment by 2.2·10⁶ m, or contains a non-finite pose
- **THEN** the result is discarded, a warning is logged, and the trajectory is unchanged

### Requirement: Accelerometer always in the solution
With `imu_always_on = true` (in the `[imu_translation]` section, default false), every window after the first SHALL include the accelerometer as a soft measurement of the change of world-frame velocity between nodes:
- **Noise from the sensor:** the term SHALL be weighted by the accelerometer noise, computed from `imu_accel_noise_density_mps2_sqrt_hz` over the term's span, plus `imu_accel_floor_sigma_mps2`.
- **Rotations included:** the term SHALL depend on the node rotations through the gravity-compensated acceleration, so that the optimization can correct a tilt in the attitude.
- **Velocity as a carried state:** the window's start velocity SHALL be an unknown with a prior: the previous window's final velocity, and its covariance grown by the acceleration noise over the time between them. The first window's prior is `imu_velocity_initial_sigma_mps`.
- **Bias as a carried state:** the accelerometer bias SHALL be an unknown carried from window to window with its covariance, grown by `imu_acceleration_offset_rw_mps2_sqrt_s`.
- **Timestamps:** the span and noise of each term SHALL come from the sample timestamps, not from a nominal IMU rate. A term whose span is stretched by a gap to more than twice `imu_acceleration_span_s` SHALL be dropped.
- **Zero velocity at rest:** a window detected at rest SHALL get a zero-velocity measurement on its velocity unknowns with `imu_zero_velocity_sigma_mps`. If the first window is at rest, its velocity prior SHALL be zero with that sigma.
- **No gate:** the degenerate-stretch gate of the accelerometer term SHALL NOT be used.

With `imu_always_on = false` the odometry SHALL give the same results as before the setting existed. The settings SHALL be loaded from and saved to the parameters TOML.

#### Scenario: A blind corridor keeps its speed
- **WHEN** a window walks at 1 m/s along a corridor that is open along its axis, with the accelerometer showing no acceleration and the previous velocity at 1 m/s
- **THEN** the window moves about 0.4 m along the axis in its 0.4 s, instead of stalling

#### Scenario: A tilt is corrected in the attitude
- **WHEN** a stationary window starts with its attitude tilted by 1 deg and the LiDAR geometry is well constrained
- **THEN** the tilt is reduced and the positions do not drift by more than a few millimetres

#### Scenario: A blind window at rest stays put
- **WHEN** a window in an open corridor is at rest, with the accelerometer showing only gravity and a previous velocity of 0.5 m/s along the axis
- **THEN** the window is detected at rest and its positions move by no more than a few millimetres

#### Scenario: An IMU gap drops the stretched terms
- **WHEN** the IMU samples of a window have a gap longer than twice `imu_acceleration_span_s`
- **THEN** the accelerometer terms spanning the gap are not used and the window is still solved

#### Scenario: Off keeps the old behaviour
- **WHEN** a parameters TOML without `imu_always_on` is loaded
- **THEN** the setting is false and the results are bit-identical to the results before it existed
