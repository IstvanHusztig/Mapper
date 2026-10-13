## ADDED Requirements

### Requirement: The shipped settings file lists every setting
`settings/default_settings.toml` SHALL contain every setting a parameter file can set, each under
its table, so a user sees every value that can be changed without reading the code.

#### Scenario: A loadable setting is missing from the shipped file
- **WHEN** a setting can be loaded from a parameter file but `settings/default_settings.toml` does not contain it
- **THEN** the unit tests fail and name the missing setting

#### Scenario: The shipped file equals the built-in defaults
- **WHEN** the shipped `settings/default_settings.toml` is loaded
- **THEN** every setting has exactly its built-in default, so the CLI with a parameter file that omits a key uses the same value as the GUI started with the shipped file

### Requirement: Unknown keys in a parameter file are reported
When a parameter file contains a key that is not a setting, loading SHALL log a warning that names
the key and its table, and SHALL go on with the other settings.

#### Scenario: A misspelled or removed key
- **WHEN** the CLI processes a recording with a parameter file that contains `[lidar_odometry_control] ndt_map_horizon = 15.0` (not a setting)
- **THEN** a warning names `ndt_map_horizon` in `[lidar_odometry_control]`, and the run goes on with the built-in value of `ndt_map_horizon_m`

### Requirement: Values that depend on the sensor or the site are settings
A value the processing uses that depends on the sensor or on the site and the walk (ranges,
noise, search radii and steps, time constants of the sensor's behaviour) SHALL be a setting with a
built-in default. A numerical detail of a method that does not depend on them (finite-difference
steps, convergence thresholds, batch sizes) SHALL be a named constant in the code, not a setting.

#### Scenario: Loop closure on a longer route
- **WHEN** a user needs loop-closure registration to search a wider radius than the built-in one
- **THEN** they set it in the parameter file, without a rebuild

#### Scenario: Former constants keep their values
- **WHEN** `44` and `44_short` are processed with `settings/lidar-vertical.toml` after the hard-coded values became settings
- **THEN** the point clouds and trajectories are bit-identical with the previous baselines
