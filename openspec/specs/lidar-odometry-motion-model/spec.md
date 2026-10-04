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
With multithreading disabled, processing the same input with the same parameters twice SHALL produce byte-identical trajectory and point cloud outputs. Multithreading SHALL be disabled by default.

#### Scenario: Two single-threaded runs are identical
- **WHEN** dataset 43 is processed twice with `useMultithread = false` and otherwise identical parameters
- **THEN** `trajectory_lio_0.csv` and `scan_lio_0.laz` of both runs are byte-identical

#### Scenario: Default is single-threaded
- **WHEN** parameters are loaded from `settings/default_settings.toml` or `settings/lidar-vertical.toml`, or no TOML is given
- **THEN** `useMultithread` is false

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
