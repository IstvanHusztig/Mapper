# sensor-settings Specification

## Purpose
Makes dMapper independent of one LiDAR model: every value that differs between sensors comes from the settings file, and the repository ships one tuned settings file per supported sensor.

## Requirements

### Requirement: Sensor values come from the settings file
Every value that depends on the sensor SHALL be a setting that the TOML loads and saves. The processing code SHALL NOT contain a number that only fits one sensor's IMU rate, noise or geometry. That covers:
- the IMU noise values;
- the LiDAR–IMU lever arm;
- the point filter's range;
- the window length in IMU samples;
- the tail re-propagation length;
- the start-up gyro bias window and gate.

When a setting is missing from the TOML, its built-in default SHALL apply as before.

#### Scenario: Two sensors without a rebuild
- **WHEN** one dMapper build processes a recording of each supported sensor, each with its own settings file
- **THEN** both runs complete with that sensor's values, and no code change or rebuild is needed between them

### Requirement: Start-up gyro bias window and gate are settings
When the start-up stationary bias is enabled (`use_remove_imu_bias_from_first_stationary_scan`), it SHALL be estimated from the first `startup_gyro_bias_nr_samples` IMU samples, as the per-axis median. It SHALL be rejected as movement, which means a bias of zero, when the summed per-axis variance of those samples exceeds `startup_gyro_bias_max_variance_deg2_s2`. The defaults SHALL be 1000 samples and 5.0 deg²/s². When the recording has no more IMU samples than the window, the bias SHALL be zero.

#### Scenario: Defaults reproduce the previous behaviour
- **WHEN** a recording is processed with settings that don't set the two keys, or that set them to 1000 and 5.0
- **THEN** the results are bit-identical with the results from before the settings existed

#### Scenario: Window sized for a slower IMU
- **WHEN** the settings set `startup_gyro_bias_nr_samples = 200` for a 200 Hz IMU
- **THEN** the start-up bias is the median of the first 200 samples, which is 1 s of data

### Requirement: One shipped settings file per supported sensor
The repository SHALL ship `settings/l2-vertical.toml` for the Unitree L2 and `settings/livox-vertical.toml` for the Livox Mid-360S, both for the vertical handheld mount. Each SHALL load without error, and SHALL save back to the same values (golden TOML test). A file whose values are not yet measured SHALL say so in its header.

#### Scenario: Both sensor files load
- **WHEN** the unit tests load `settings/l2-vertical.toml` and `settings/livox-vertical.toml` and save them again
- **THEN** each saved file equals its golden TOML

#### Scenario: Renamed L2 file keeps its values
- **WHEN** the CLI processes `44_short` with `settings/l2-vertical.toml`
- **THEN** the result is bit-identical with the `44_short` manifest made with the former `settings/lidar-vertical.toml`

### Requirement: User documentation is sensor-neutral
The user documentation SHALL describe the sensor in general terms (the LiDAR, the IMU). It SHALL NOT contain results or defects of one sensor or one recording. That covers `README.md`, the HTML docs in `docs/` and `docs/hu/`, and the code comments. Sensor-specific facts and their sources SHALL live only in that sensor's settings file and hardware sheet in `settings/`.

#### Scenario: No L2 history in the user docs
- **WHEN** the user documentation and code comments are searched for "Unitree", "L2" and "recording 44"
- **THEN** the only hits are names of settings files or hardware sheets
