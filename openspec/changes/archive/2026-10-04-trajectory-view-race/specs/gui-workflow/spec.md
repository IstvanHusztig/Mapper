## ADDED Requirements

### Requirement: Trajectory shown during a run
While a run's odometry is going, the GUI SHALL draw the trajectory of the windows the odometry has finished only, with the cross at the last finished pose. During loop closure, consistency and saving it SHALL keep showing that trajectory. When the run has finished, the GUI SHALL show the final, post-processed trajectory. The GUI SHALL NOT read a window's poses while the run may still change them, and drawing SHALL NOT change the processing or its results.

#### Scenario: Trajectory grows with the processing
- **WHEN** the user processes `44_short` in the GUI
- **THEN** the drawn trajectory grows window by window with the odometry, and the cross is at the end of the processed part, not at the end of the recording

#### Scenario: Final trajectory after post-processing
- **WHEN** a run with loop closure has finished
- **THEN** the drawn trajectory is the corrected one, as saved

#### Scenario: Results unaffected
- **WHEN** the GUI processes `44_short` with `settings/lidar-vertical.toml`
- **THEN** every output file compares identical with the `44_short` manifest
