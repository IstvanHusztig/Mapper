## ADDED Requirements

### Requirement: Motion edges assert no velocity
With `imu_edges_follow_state = true` (in the `[imu_translation]` section, default false) and the accelerometer in the
solution, the position part of the motion edges SHALL follow the window's start velocity and acceleration offset in
every window whose guess was integrated from the accelerometer: a window whose nodes move as the accelerometer's path
does for another start velocity or another acceleration offset SHALL cost the edges nothing. The rotation part of the
edges SHALL be unchanged, and so SHALL the edges of a window at rest. With the setting false the results SHALL be
bit-identical to the results before the setting existed, and a window capture or odometry snapshot written before
the setting existed SHALL still be accepted as made with the same settings.

#### Scenario: Another start velocity costs the edges nothing
- **WHEN** a window at its guess gets a start velocity 0.5 m/s larger and its nodes are moved along the path
  integrated from that velocity
- **THEN** the cost and the right-hand side of the motion edges are the same as at the guess

#### Scenario: Another acceleration offset costs the edges nothing
- **WHEN** a window at its guess gets an acceleration offset of 0.1 m/s² and its nodes are moved along the path
  integrated with that offset
- **THEN** the cost and the right-hand side of the motion edges are the same as at the guess

#### Scenario: Off keeps the results
- **WHEN** a parameters TOML without `imu_edges_follow_state` is loaded
- **THEN** the setting is false and the results are bit-identical to the results before it existed

### Requirement: Velocity and acceleration offset are carried together
With `imu_carry_joint_covariance = true` (default false) the next window's prior on its start velocity and its
acceleration offset SHALL be their joint distribution from the previous window, including the covariance between the
two, propagated over the time between the windows (the velocity changes by the offset times that time) and grown by
the noise as before. Where the velocity prior does not come from the previous window (the first window, a carried
speed above `max_prior_velocity_mps`) the covariance between the two SHALL be zero. With the setting false the two
SHALL be carried with separate covariances as before, with bit-identical results.

#### Scenario: An offset error adds up over blind windows
- **WHEN** N consecutive windows have no LiDAR information along an axis, the acceleration offset along it is
  uncertain by sigma and the other noises are small against it
- **THEN** the carried velocity sigma along that axis after the N windows equals the model's closed form (sigma × N ×
  the window duration for the offset's part, plus the white noise's part) within 1 %

#### Scenario: A rejected step keeps the covariance of the accepted one
- **WHEN** the solver rejects a step and restores the last accepted state of a window
- **THEN** the covariance between end velocity and offset that is carried on is the accepted state's

#### Scenario: A capture written before the setting existed still replays
- **WHEN** a window capture written by a build without the joint covariance is replayed with its own settings
- **THEN** every window comes out bit for bit as captured

### Requirement: An acceleration error common to a window
`imu_accel_window_sigma_mps2` (default 0) SHALL be the sigma of an accelerometer error that is constant over a window
and independent from window to window. With a value above zero:
- the window's acceleration offset SHALL have that much more prior uncertainty;
- after the window only the part of the estimated offset that the carried offset's own uncertainty accounts for SHALL
  be folded into the gravity vector, and the carried offset covariance and its covariance with the velocity SHALL be
  those of that part;
- the error SHALL NOT be carried to the next window.

With the value 0 the results SHALL be bit-identical to the results before the setting existed. (This requirement is
built only after the measurement of task 5.1 and the user's decision; if the layer is not built, the requirement is
dropped from the change before it is archived.)

#### Scenario: A blind window's velocity becomes less certain by the window error
- **WHEN** a window has no LiDAR information along an axis and `imu_accel_window_sigma_mps2` is s
- **THEN** its end-velocity sigma along that axis is larger than its start-velocity sigma by s × the window duration,
  added in quadrature

#### Scenario: The carried offset takes its share
- **WHEN** a window estimates an acceleration offset and the carried offset's prior variance equals the window
  error's variance
- **THEN** half of the estimated offset is folded into the gravity vector

### Requirement: The carried uncertainty grows while the LiDAR is blind
With `imu_edges_follow_state` and `imu_carry_joint_covariance` in use, the start-velocity prior's sigma along an axis
without LiDAR information SHALL grow from window to window for as long as the axis stays without information, with
no upper limit set by the motion edges' weight.

#### Scenario: Fifteen blind windows
- **WHEN** fifteen consecutive windows of 0.36 s have no LiDAR information along the direction of travel and the
  acceleration offset's prior sigma is 0.02 m/s²
- **THEN** the start-velocity prior's sigma along it is larger in each window than in the one before, and after the
  fifteenth it is at least 0.1 m/s

### Requirement: The honesty of the carried states can be read
A tool SHALL print, from the per-window table a run or replay writes (`window_features.csv`), for windows grouped by
their information ratio: the start-velocity prior's sigma, the correction the window made to that prior, and their
ratio, and the velocity chi² (median and 95th percentile); the estimated acceleration offset against its prior sigma;
and for every stretch of consecutive windows without LiDAR information along travel the prior sigma at its first and
last window, the size and the direction (along or against travel) of the first correction after it with the
information ratio there, and the number of windows until the speed settles. A reading the table does not hold SHALL
be printed as NOT MEASURED.

#### Scenario: A run with a blind stretch
- **WHEN** the tool reads the table of a run with a stretch of ten windows below the projection ratio
- **THEN** it prints one line for that stretch with the prior sigma at entry and exit and the correction after it

#### Scenario: A table without the noise columns
- **WHEN** the table was written without the noise model's columns
- **THEN** the sigma readings are NOT MEASURED and the other readings are still printed
