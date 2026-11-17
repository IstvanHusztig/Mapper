## ADDED Requirements

### Requirement: Settings files are read strictly
A settings file SHALL be refused, with a list of its problems, when it holds a key the program would not read: an
unknown section or key, a key in another section than its own, or a value of the wrong type (a double accepts an
integer). A setting removed from dMapper SHALL be accepted with a message that it is ignored when its value asks for
the behaviour that remains (a removed switch set to false or 0; a parameter of a removed feature at any value), and the
file SHALL be refused when the value asks for the removed behaviour.

#### Scenario: A misplaced key
- **WHEN** a settings file has `ndt_huber_cutoff = 3.0` in `[lidar_odometry_control]` instead of `[imu_translation]`
- **THEN** the file is refused and the message names the key and the section it belongs in

#### Scenario: An older file with a removed setting at the remaining behaviour
- **WHEN** a settings file written before the removal holds `use_mutliple_gaussian = false` and a `[robust_lidar_odometry]` section with `use_robust_and_accurate_lidar_odometry = false`
- **THEN** the file loads, and a message says that each of these settings was removed and is ignored

#### Scenario: A removed setting asks for removed behaviour
- **WHEN** a settings file holds `use_mutliple_gaussian = true`
- **THEN** the file is refused and the message names the setting and the value that is accepted

### Requirement: Results record the build
The processing report (`dMapper_results_*.json`) SHALL hold a `build_info` block with the git commit the program was
built from, whether the source had uncommitted changes of tracked files at build time, the build type, the compiler, the
compiler flags and the compile definitions. The values SHALL be taken at build time, not at configure time.

#### Scenario: A build after a commit
- **WHEN** the program is built again after a commit, without configuring again, and processes a recording
- **THEN** the report's `build_info.git_commit` is the new commit
