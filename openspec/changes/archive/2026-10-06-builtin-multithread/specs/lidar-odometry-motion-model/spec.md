## ADDED Requirements

### Requirement: Reproducible results
Processing the same input with the same parameters twice SHALL produce byte-identical trajectory and point cloud outputs. The window optimization SHALL compute in parallel, and its result SHALL be byte-identical to a sequential computation of the same window. Parallel computation is built in and is not a setting.

#### Scenario: Two runs are identical
- **WHEN** a dataset is processed twice with identical parameters and no window stopped by `real_time_threshold_seconds`
- **THEN** the trajectory and point cloud outputs of both runs are byte-identical

#### Scenario: Parallel equals sequential
- **WHEN** a window is optimized once with the parallel node loop and once with the sequential one
- **THEN** the resulting poses, gyro bias and translation information are bit-identical

#### Scenario: No multithreading setting
- **WHEN** parameters are loaded from a TOML file that still contains `useMultithread`, or saved to one
- **THEN** the key is ignored on loading and not written on saving

## REMOVED Requirements

### Requirement: Reproducible results with or without multithreading
**Reason**: Multithreading no longer changes results and is always on, so the setting and the requirement about its two values go.
**Migration**: Replaced by "Reproducible results". Remove `useMultithread` from your own TOML files if you like; it is ignored.
