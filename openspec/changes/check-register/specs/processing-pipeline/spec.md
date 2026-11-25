## MODIFIED Requirements

### Requirement: Settings files are read strictly
A settings file SHALL be refused, with a list of its problems, when it holds a key the program would not read: an
unknown section or key, a key in another section than its own, or a value of the wrong type (a double accepts an
integer). A setting removed from dMapper SHALL be accepted with a message that it is ignored when its value asks for
the behaviour that remains (a removed switch set to false or 0; a parameter of a removed feature at any value), and the
file SHALL be refused when the value asks for the removed behaviour. A removed setting whose value entered the odometry
settings hash at every value SHALL be accepted only at the value the hash still counts for it, so that a file the
loader accepts never fails later against a snapshot or capture made with that same file.

#### Scenario: A misplaced key
- **WHEN** a settings file has `ndt_huber_cutoff = 3.0` in `[lidar_odometry_control]` instead of `[imu_translation]`
- **THEN** the file is refused and the message names the key and the section it belongs in

#### Scenario: An older file with a removed setting at the remaining behaviour
- **WHEN** a settings file written before the removal holds `use_mutliple_gaussian = false` and a `[robust_lidar_odometry]` section with `use_robust_and_accurate_lidar_odometry = false`
- **THEN** the file loads, and a message says that each of these settings was removed and is ignored

#### Scenario: A removed setting asks for removed behaviour
- **WHEN** a settings file holds `use_mutliple_gaussian = true`
- **THEN** the file is refused and the message names the setting and the value that is accepted

#### Scenario: A removed parameter that the hash counted
- **WHEN** a settings file holds `turn_rate_threshold_deg_s = 45.0`
- **THEN** the file is refused and the message names the setting and the value 30 that is accepted
