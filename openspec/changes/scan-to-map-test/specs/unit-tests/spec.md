## ADDED Requirements

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
