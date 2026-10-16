## MODIFIED Requirements

### Requirement: Time-scaled rotation prior
When time-scaled rotation sigma is enabled, the rotation uncertainty allowed between two consecutive trajectory nodes SHALL grow with the square root of their time separation. Per axis, it SHALL be `max(k * ARW_axis * sqrt(dt), sigma_floor)`, where `ARW_axis`, `k` and `sigma_floor` are configurable. When it is disabled, the constant per-node rotation sigmas SHALL be used as before. The weight of each rotation term SHALL be the inverse square of its sigma, with no further factor.

#### Scenario: Short node gap gets less rotation freedom
- **WHEN** two consecutive nodes are 0.2 ms apart and another pair is 2 ms apart, with time-scaled sigma enabled
- **THEN** the allowed rotation sigma for the 0.2 ms pair is smaller than for the 2 ms pair, in the ratio sqrt(0.2/2), unless limited by `sigma_floor`

#### Scenario: Floor protects near-zero gaps
- **WHEN** two consecutive nodes have a time separation of 0 or near 0
- **THEN** the rotation sigma equals `sigma_floor` and the optimization completes without numerical failure

#### Scenario: Map quality at least matches the tuned constant sigma
- **WHEN** datasets 43 and 42 (repaired IMU timestamps) are processed with `settings/lidar-vertical.toml` (default time-scaled sigma, k = 4 before the √π rescale, 7.0898 after it)
- **THEN** on each dataset the median node-to-node yaw change is at most 0.03°, no window has an end-of-window yaw spike above 3°, and the occupied 3 cm cells in the Z section [-0.3, 0.7] m do not exceed the constant-sigma-0.02 baseline (43: 15,945, 42: 6,872)

#### Scenario: Weight is the configured sigma's
- **WHEN** a rotation term's sigma is σ
- **THEN** its weight in the normal equations is 1 / σ² (the earlier Cauchy-density factor 1 / (π (1 + x²)) is gone; the shipped sigmas were scaled by √π, k = 7.0898 and `sigma_floor` = 0.003545°, so the effective prior is unchanged)

### Requirement: Online gyro bias estimation
When gyro bias estimation is enabled, the system SHALL estimate a 3-axis gyro bias for every optimization window after the first while the sensor is moving, with no stationary period required. It SHALL remove the estimated bias from the IMU rotation increments used in that window exactly once: the increments of the window's motion model already have the carried bias removed, so the motion-model measurement SHALL subtract only the difference between the window's estimate and the carried bias. Bias estimation SHALL be enabled by default.

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

#### Scenario: Disabled reproduces previous behaviour
- **WHEN** gyro bias estimation, time-scaled sigma and tail re-propagation are all disabled
- **THEN** the trajectory and map quality metrics are within the run-to-run spread of the pre-change code for the same input and parameters (the pre-change code is not reproducible run to run, so a byte comparison against it is not possible)

#### Scenario: Bias is removed once
- **WHEN** at least three chained synthetic windows are optimized with a known gyro bias and the bias carried from window to window
- **THEN** the estimate of the last window is within 5 % of the known bias on every axis (before the fix it settled near half of it)

#### Scenario: Bias on recording 44
- **WHEN** the 95 s cuts of recording 44 are processed with `settings/lidar-vertical.toml` (odometry only)
- **THEN** the z-axis bias estimate averaged over 30–85 s is within 0.15 °/s of the bias fitted from the gyro against the trajectory (−0.82 °/s)

#### Scenario: No stationary start required
- **WHEN** a recording starts with the operator already moving, so the startup stationary-bias check reports excessive movement
- **THEN** bias estimation still runs: the first window keeps the prior, and every later window produces a bias estimate

### Requirement: Robust window hand-off
The pose that starts a window SHALL NOT be taken from a trajectory node whose orientation is determined by fewer LiDAR constraints than the configured tail length implies. The trailing nodes of each window SHALL be re-derived from the last well-constrained node using the bias-corrected IMU rotation, and the start of the next window SHALL be derived from them. The first node of the next window SHALL be placed at its own time: the previous window's last node advanced by the IMU motion between the two samples. The points between the two samples SHALL belong to the previous window's last node.

#### Scenario: No heading jump at window boundaries
- **WHEN** recording 44 is processed with default settings
- **THEN** in no window does the yaw of the last node differ by more than 3° from the median yaw of the nodes 30 to 60 before it

#### Scenario: No motion lost at the hand-off
- **WHEN** chained synthetic windows are placed under a constant yaw rate
- **THEN** the yaw of the last window's last node equals the integrated rate to within 1e-6 rad (before the fix, 1/N of the rotation was lost per window of N nodes)

#### Scenario: No points lost between windows
- **WHEN** a recording is cut into windows
- **THEN** every point between the first and the last node of the recording that lies within 10 ms of a node belongs to exactly one window

#### Scenario: Tail re-propagation can be disabled
- **WHEN** the tail length is set to 0
- **THEN** the next window starts from the optimized last node, advanced by the IMU motion to its own time

## ADDED Requirements

### Requirement: Orientation estimate from the start
The IMU orientation estimate SHALL integrate the gyro's heading changes from the first sample on. The orientation filter's initialisation period (fast gravity alignment) SHALL NOT reset the heading.

#### Scenario: Heading during the initialisation
- **WHEN** the IMU turns at a constant yaw rate during the filter's first 3 s
- **THEN** the estimated heading change over that time equals the integrated rate (not 0)
