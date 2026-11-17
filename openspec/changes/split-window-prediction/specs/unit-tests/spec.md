## ADDED Requirements

### Requirement: Window prediction steps tested one by one
The unit tests SHALL test each step of a window's initial guess on its own, on synthetic windows without recorded data:
the mean shift between the two previous windows, the chaining onto the previous window's end, the rest test, and the
translation guess.

#### Scenario: Mean shift and its limit
- **WHEN** the two previous windows' last nodes are 0.36 m apart over 360 nodes, and in a second case 500 m apart
- **THEN** the mean shift is 0.001 m per node in the first case, and zero with the drop reported in the second

#### Scenario: Chaining equals placing
- **WHEN** a window with turning relative poses is chained onto a previous window's end
- **THEN** its poses are bit-identical with `place_window_after` on the same window and end

#### Scenario: Rest test
- **WHEN** the rest test runs on a window whose accelerometer reads gravity only and whose rotation does not change, and on the same window with a walking acceleration
- **THEN** the first window rests and the second does not

#### Scenario: Translation guess branches
- **WHEN** the translation guess runs with accelerometer integration, at rest, with constant velocity and with the mean shift only
- **THEN** the positions follow p = v t + a t² / 2, stay at the start, follow the previous velocity over the node times plus the gap, and follow the mean shift per node, respectively
