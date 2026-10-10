# Spec Delta

## ADDED Requirements

### Requirement: Reprocessing from an odometry snapshot
With `save_odometry_snapshot = true`, a run SHALL write, after the odometry and before the post-processing, a snapshot of everything the later stages read from the odometry: every window's poses, motion-model poses, timestamps, IMU angles and accelerations, gyro bias and velocity state, and the run state the post-processing and saving use. A reprocessing entry point SHALL take a recording folder, a parameters file, an output folder and a snapshot. It SHALL load the recording and cut its windows as a run does, restore the windows and run state from the snapshot instead of running the odometry, then run the post-processing (loop closure, consistency, each as enabled in the parameters) and save the result. Its result SHALL be bit-identical to a full run with the same recording and parameters. It SHALL refuse a snapshot whose recording (window count and window timestamps) or odometry settings differ from the given ones, with a message naming the difference. The setting SHALL default to false, and a run without it SHALL give results bit-identical to before.

#### Scenario: Same result as a full run
- **WHEN** a recording is processed with `save_odometry_snapshot = true`, and then reprocessed from that snapshot with the same parameters
- **THEN** every output file except the parameters TOML and the date-stamped names is bit-identical to the full run's

#### Scenario: Different loop-closure settings
- **WHEN** a snapshot is reprocessed with different loop-closure settings
- **THEN** the result equals a full run with those settings, without the odometry being run again

#### Scenario: A snapshot from another recording or other odometry settings
- **WHEN** the snapshot's window count, window timestamps or odometry settings differ from the given recording and parameters
- **THEN** the reprocessing stops with a message naming what differs and writes no result
