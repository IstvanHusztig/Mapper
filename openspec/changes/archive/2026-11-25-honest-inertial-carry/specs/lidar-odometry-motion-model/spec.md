## ADDED Requirements

### Requirement: Motion edges assert no velocity
With `imu_edges_follow_state = true` (in the `[imu_translation]` section, default false) and the accelerometer in the
solution, the position part of the motion edges SHALL follow the window's start velocity and acceleration offset in
every window whose guess was integrated from the accelerometer: a window whose nodes move as the accelerometer's path
does for another start velocity or another acceleration offset SHALL cost the edges nothing. The rotation part of the
edges SHALL be unchanged, and so SHALL the edges of a window at rest. With the setting false the results SHALL be
bit-identical to the results before the setting existed, and a window capture or odometry snapshot written before
the setting existed SHALL still be accepted as made with the same settings.

#### Scenario: Another start velocity costs the edges nothing
- **WHEN** a window at its guess gets a start velocity 0.5 m/s larger and its nodes are moved along the path
  integrated from that velocity
- **THEN** the cost and the right-hand side of the motion edges are the same as at the guess

#### Scenario: Another acceleration offset costs the edges nothing
- **WHEN** a window at its guess gets an acceleration offset of 0.1 m/s² and its nodes are moved along the path
  integrated with that offset
- **THEN** the cost and the right-hand side of the motion edges are the same as at the guess

#### Scenario: Off keeps the results
- **WHEN** a parameters TOML without `imu_edges_follow_state` is loaded
- **THEN** the setting is false and the results are bit-identical to the results before it existed

