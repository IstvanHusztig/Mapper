# Spec Delta

## ADDED Requirements

### Requirement: Start-up gyro bias from a hand-held stand-still
With `use_remove_imu_bias_from_first_stationary_scan = true`, the system SHALL estimate the gyro bias from the first
`imu_startup_bias_window_s` seconds of IMU data as the per-axis median, and accept it when that stretch shows no real
motion: the medians of its four quarters agree per axis within `imu_startup_bias_max_spread_deg_s`, the gravity
direction (mean accelerometer of the first and last quarter) turns by at most `imu_startup_bias_max_tilt_deg`, and the
per-axis rate scatter stays within `imu_startup_bias_max_noise_ratio` times the sensor's noise
(`gyro_arw_axis × √(IMU rate)`). An accepted bias SHALL be removed from every gyro sample of the recording, the first
seconds included, before the orientation is estimated. A rejected one SHALL be 0, as today. The run SHALL log the
estimate and, for a rejection, the test that failed with its value. When the start-up bias was accepted and gyro bias
estimation is on, the online estimate SHALL start with sigma `gyro_bias_startup_sigma_deg_s` instead of
`gyro_bias_initial_sigma_deg_s`.

#### Scenario: Sensor lying still
- **WHEN** the gyro samples of the static recording (`unsynced_archive/43/static`, gyro only) go through the start-up test
- **THEN** the bias is accepted and lies within 0.2 °/s of the recording's median per axis (z about −0.6 °/s)

#### Scenario: Hand-held stand-still
- **WHEN** recording 44 is processed (first-second total variance 12.1 (°/s)², rejected by the previous fixed limit of 5.0)
- **THEN** the start-up bias is accepted, its z value lies within 0.5 °/s of the odometry's settled estimate (about −1.0 °/s), and the first windows' online estimate doesn't overshoot it by more than 0.5 °/s

#### Scenario: Moving start
- **WHEN** the first stretch contains a turn or a tilt (synthetic samples with a rate step, or a gravity direction that turns)
- **THEN** the bias is rejected (0) and the log names the failed test

#### Scenario: Turned start room
- **WHEN** recording 44 and its repeat runs s1, s2 are processed with the new start-up bias
- **THEN** the start room's rotation against the whole floor (`plan_fit.py`) is smaller in absolute value than with the shipped code on each run
