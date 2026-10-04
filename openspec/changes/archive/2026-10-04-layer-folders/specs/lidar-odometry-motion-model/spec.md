## MODIFIED Requirements

### Requirement: Reproducible results
With multithreading disabled, processing the same input with the same parameters twice SHALL produce byte-identical trajectory and point cloud outputs. Multithreading SHALL be disabled by default.

#### Scenario: Two single-threaded runs are identical
- **WHEN** dataset 43 is processed twice with `useMultithread = false` and otherwise identical parameters
- **THEN** `trajectory_lio_0.csv` and `scan_lio_0.laz` of both runs are byte-identical

#### Scenario: Default is single-threaded
- **WHEN** parameters are loaded from `settings/default_settings.toml` or `settings/lidar-vertical.toml`, or no TOML is given
- **THEN** `useMultithread` is false
