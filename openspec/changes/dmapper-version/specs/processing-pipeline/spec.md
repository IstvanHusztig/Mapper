## ADDED Requirements

### Requirement: Results carry dMapper's version
A run's output folder SHALL contain its parameters as `dMapper_params_<version>_<date-time>.toml` and its processing report as `dMapper_results_<version>_<date-time>.json`, where `<version>` is the version dMapper was built with (major.minor.patch, set in the top-level `CMakeLists.txt`). Both files SHALL record that version (`software_version`; the report's `lidar_odometry_version` as "v" + version). The version stamp in a loaded parameter file SHALL NOT change the version a run records; it is only compared with the build's version, and a different major or minor version gives a warning.

#### Scenario: A run with the shipped settings
- **WHEN** the CLI processes `44_short` with `settings/lidar-vertical.toml`
- **THEN** the output folder contains `dMapper_params_0.1.0_<date-time>.toml` and `dMapper_results_0.1.0_<date-time>.json`, and both record version 0.1.0

#### Scenario: A parameter file from another version
- **WHEN** a parameter file with `software_version = "0.95.0"` in `[version_info]` is loaded
- **THEN** a version-mismatch warning is printed, and the run still records 0.1.0
