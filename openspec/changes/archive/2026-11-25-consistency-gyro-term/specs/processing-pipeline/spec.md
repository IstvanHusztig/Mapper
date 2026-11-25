# Spec Delta

## ADDED Requirements

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
- **THEN** the reprocessing is refused as made with other odometry settings, and a snapshot made before the mode existed is still accepted with the mode off

#### Scenario: Mode off
- **WHEN** `holdout_block_s = 0`
- **THEN** every output file equals the current baselines bit for bit (apart from the parameters file)
