## ADDED Requirements

### Requirement: Accelerometer always in the solution
With `imu_always_on = true` (in the `[imu_translation]` section, default false), every window after the first SHALL include the accelerometer as a soft measurement of the change of world-frame velocity between nodes:
- **Noise from the sensor:** the term SHALL be weighted by the accelerometer noise, computed from `imu_accel_noise_density_mps2_sqrt_hz` over the term's span, plus `imu_accel_floor_sigma_mps2`.
- **Rotations included:** the term SHALL depend on the node rotations through the gravity-compensated acceleration, so that the optimization can correct a tilt in the attitude.
- **Velocity as a carried state:** the window's start velocity SHALL be an unknown with a prior: the previous window's final velocity, and its covariance grown by the acceleration noise over the time between them. The first window's prior is `imu_velocity_initial_sigma_mps`.
- **Bias as a carried state:** the accelerometer bias SHALL be an unknown carried from window to window with its covariance, grown by `imu_acceleration_offset_rw_mps2_sqrt_s`.
- **No gate:** the degenerate-stretch gate of the accelerometer term SHALL NOT be used.

With `imu_always_on = false` the odometry SHALL give the same results as before the setting existed. The settings SHALL be loaded from and saved to the parameters TOML.

#### Scenario: A blind corridor keeps its speed
- **WHEN** a window walks at 1 m/s along a corridor that is open along its axis, with the accelerometer showing no acceleration and the previous velocity at 1 m/s
- **THEN** the window moves about 0.4 m along the axis in its 0.4 s, instead of stalling

#### Scenario: A tilt is corrected in the attitude
- **WHEN** a stationary window starts with its attitude tilted by 1 deg and the LiDAR geometry is well constrained
- **THEN** the tilt is reduced and the positions do not drift by more than a few millimetres

#### Scenario: Off keeps the old behaviour
- **WHEN** a parameters TOML without `imu_always_on` is loaded
- **THEN** the setting is false and the results are bit-identical to the results before it existed
