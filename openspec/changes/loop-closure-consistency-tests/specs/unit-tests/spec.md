## ADDED Requirements

### Requirement: Loop closure tested on synthetic data
The unit-test executable SHALL contain loop-closure scenarios that run the loop closure on a synthetic recording with a known true trajectory that revisits a place, without recorded data, and check the corrected trajectory against that truth. The scenarios SHALL set the loop-closure options themselves. They SHALL be skipped in the regular test run, because each takes seconds in Release and many minutes in Debug, and SHALL run when requested explicitly, in either build, so that the loop closure can be debugged on them.

#### Scenario: The regular run skips them
- **WHEN** `ctest` or the test executable runs without options
- **THEN** the loop-closure scenarios are skipped, and they run when requested with `--no-skip`

#### Scenario: Drift on the second lap
- **WHEN** the trajectory's second lap drifts away from the truth and its points were seen from the true poses
- **THEN** at least one revisit is accepted, and the position error at the end of the second lap is reduced to at most half of the drift

#### Scenario: A trajectory that is already right
- **WHEN** the trajectory equals the truth
- **THEN** every node stays within 5 mm and 0.1° of the truth after the loop closure; as long as the loop closure doesn't meet this, the scenario SHALL be marked as an expected failure that names the cause

#### Scenario: No revisit
- **WHEN** the recording contains no pair of segments far enough apart in time and close enough in space
- **THEN** the loop closure returns 0 and every pose is unchanged

### Requirement: Consistency pass tested on synthetic data
The unit tests SHALL run the consistency pass, in both of its variants (a single Gaussian per cell, and several per cell over time), on a synthetic recording with a known true trajectory, without recorded data, and SHALL check the result against that truth. The tests SHALL set the consistency options themselves.

#### Scenario: Offsets between windows
- **WHEN** each window's poses are moved by a small rigid offset, as the seams between odometry windows leave them
- **THEN** the pass reduces the mean pose error against the truth and the number of occupied 3 cm cells of the map

#### Scenario: A trajectory that is already right
- **WHEN** the trajectory equals the truth
- **THEN** every node stays within 5 mm and 0.1° of the truth after the pass

#### Scenario: A missed expectation
- **WHEN** a variant does not meet one of these expectations
- **THEN** that case is marked as an expected failure that names the cause, so that it is reported once it starts to pass
