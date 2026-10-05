## ADDED Requirements

### Requirement: Reproducible results with or without multithreading
Processing the same input with the same parameters twice SHALL produce byte-identical trajectory and point cloud outputs, with multithreading enabled or disabled. A multithreaded run SHALL produce byte-identical trajectory and point cloud outputs to a single-threaded run of the same input and parameters. Multithreading SHALL be enabled by default.

#### Scenario: Two single-threaded runs are identical
- **WHEN** dataset 43 is processed twice with `useMultithread = false` and otherwise identical parameters
- **THEN** `trajectory_lio_0.csv` and `scan_lio_0.laz` of both runs are byte-identical

#### Scenario: Two multithreaded runs are identical
- **WHEN** a dataset is processed twice with `useMultithread = true` and otherwise identical parameters
- **THEN** the trajectory and point cloud outputs of both runs are byte-identical

#### Scenario: Multithreaded equals single-threaded
- **WHEN** a dataset is processed once with `useMultithread = true` and once with `useMultithread = false`, with otherwise identical parameters and no window stopped by `real_time_threshold_seconds`
- **THEN** the trajectory and point cloud outputs of both runs are byte-identical

#### Scenario: Default is multithreaded
- **WHEN** parameters are loaded from `settings/default_settings.toml` or `settings/lidar-vertical.toml`, or no TOML is given
- **THEN** `useMultithread` is true

## REMOVED Requirements

### Requirement: Reproducible results
**Reason**: Multithreaded runs are now deterministic and bit-identical with single-threaded ones, so reproducibility no longer depends on multithreading being off, and the default becomes multithreaded.
**Migration**: Replaced by "Reproducible results with or without multithreading". Its single-threaded scenario is kept there unchanged; "Default is single-threaded" becomes "Default is multithreaded". Set `useMultithread = false` to run on one core.
