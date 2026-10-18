# Spec Delta

## ADDED Requirements

### Requirement: Gyro rotation term in the consistency pass
With `consistency_gyro_rotation_term = true` (section `[consistency]`, default false), the consistency pass SHALL add,
between every two consecutive trajectory nodes of the same window, a term that ties their relative rotation to the
relative rotation of the IMU orientation at those nodes, corrected by that window's estimated gyro bias over the time
between them. Its sigma per axis SHALL be `consistency_gyro_sigma_scale` (default 3.0) times the odometry's
time-scaled rotation sigma for the same time step, `max(rotation_sigma_inflation_k × gyro_arw_axis × √dt,
rotation_sigma_floor_deg)`. A window whose IMU rotation over the window differs from its odometry result by more than
`consistency_gyro_gate_deg` (default 3.0) SHALL get no such terms, and the run SHALL log how many windows were left
out. The term SHALL apply to the consistency pass with `use_mutliple_gaussian = false`; with the setting false, every
result SHALL be bit-identical to a run without this feature.

#### Scenario: Setting off
- **WHEN** a recording is processed with `consistency_gyro_rotation_term = false`
- **THEN** every output file equals the current baselines bit for bit (apart from the parameters file, which lists the new settings)

#### Scenario: Node rotation follows the gyro
- **WHEN** the synthetic loop recording is processed with the consistency pass and the term on
- **THEN** the node-to-node rotation of the result differs from the true one less than without the term

#### Scenario: A window whose gyro disagrees with the LiDAR
- **WHEN** a window's IMU rotation differs from its odometry result by more than `consistency_gyro_gate_deg`
- **THEN** that window gets no gyro terms, its nodes are held by the point and smoothness terms only, and the log counts it

#### Scenario: Reprocessing from a snapshot
- **WHEN** an odometry snapshot is reprocessed with the term switched on or off
- **THEN** the snapshot is accepted (the settings are in `[consistency]`) and the result equals a full run with the same settings

### Requirement: Hold-out evaluation mode
With `holdout_block_s` > 0 (section `[evaluation]`, default 0 = off), the points whose timestamps fall into odd blocks
of `holdout_block_s` seconds (block index = floor(t / holdout_block_s)) SHALL be left out of every map and every point
term of the odometry, the loop closure and the consistency pass. They SHALL still be placed with the final trajectory
and written to every output point cloud like all other points. The setting SHALL count as an odometry setting for
reprocessing from a snapshot. With `holdout_block_s = 0`, every result SHALL be bit-identical to a run without this
feature.

#### Scenario: Held-out points are saved
- **WHEN** a recording is processed with `holdout_block_s = 0.01`
- **THEN** the merged point cloud contains the points of the odd 10 ms blocks, placed with the final trajectory, although no stage fitted them

#### Scenario: Snapshot with another hold-out
- **WHEN** a snapshot made with one `holdout_block_s` is reprocessed with another
- **THEN** the reprocessing is refused with a message naming `holdout_block_s`

#### Scenario: Mode off
- **WHEN** `holdout_block_s = 0`
- **THEN** every output file equals the current baselines bit for bit (apart from the parameters file)
