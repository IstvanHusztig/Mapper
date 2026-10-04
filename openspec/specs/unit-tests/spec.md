# unit-tests Specification

## Purpose

Provides a unit-test executable for the project so domain and application code can be tested in
isolation, without recorded data, as the Clean Architecture refactor proceeds.

## Requirements

### Requirement: Test executable builds with both presets
The project SHALL provide a unit-test executable that builds with both the `x64-Release` and the
`x64-Debug` presets, without network access and without any dependency outside `3rdparty/` and
the system OpenGL.

#### Scenario: Build the tests
- **WHEN** `cmake --build --preset x64-Release --target lidar_odometry_tests` is run on a configured tree
- **THEN** the build succeeds and produces the test executable

### Requirement: Tests run without data files
The unit tests SHALL run without any recorded dataset, GUI or display, and SHALL report success
or failure through the process exit code. They SHALL be runnable through CTest.

#### Scenario: Run the smoke test
- **WHEN** `ctest --test-dir out/build/x64-Release` is run after building the tests
- **THEN** the smoke test passes and the command exits with status 0

#### Scenario: A failing test
- **WHEN** a test assertion fails
- **THEN** the test executable exits non-zero and names the failing test

### Requirement: Scan-to-map optimizer tested on synthetic data
The unit tests SHALL run the scan-to-map optimizer of one window on a synthetic scene with a known true trajectory, without recorded data, and SHALL check its result against that truth. The tests SHALL set the optimizer's options themselves, so that changing the shipped settings files does not change the test outcome.

#### Scenario: A perfect initial guess
- **WHEN** the window's initial guess equals the true trajectory in a closed room
- **THEN** the optimized poses stay within 3 mm and 0.1° of the truth

#### Scenario: A rotation drift in the initial guess
- **WHEN** the initial guess turns away from the truth at a constant yaw rate, as a gyro bias makes it do, and the bias is not estimated
- **THEN** the heading error at the end of the window is reduced to at most 10 % of the guess's, and the positions stay within 3 mm of the truth

#### Scenario: Gyro bias estimation
- **WHEN** the same drifting guess is optimized with gyro-bias estimation on
- **THEN** the estimated bias about the vertical axis matches the injected yaw-rate error within 20 %

#### Scenario: Iterations needed
- **WHEN** a drifting guess is optimized for the full number of iterations of a real run
- **THEN** the test reports the iteration after which the poses no longer change by more than 0.1 mm and 0.001°, and the iteration at which the update first falls below 1e-2, together with the pose error against the truth at that iteration

#### Scenario: Degenerate scene
- **WHEN** the window moves along a corridor with no end walls
- **THEN** the test expects the translation information reported for the window to be at least 100 times weaker along the corridor than across it, while in a closed room no direction is that weak; as long as the map's surface normals don't meet this, the corridor case SHALL be marked as an expected failure that names the cause, so that it is reported once it starts to pass

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
