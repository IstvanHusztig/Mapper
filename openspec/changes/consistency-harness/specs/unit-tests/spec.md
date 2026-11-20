# Spec Delta

## ADDED Requirements

### Requirement: Synthetic recordings with inertial data and a known truth
The unit tests SHALL provide synthetic recordings that hold what a recorder produces (IMU samples with timestamps,
gyro and accelerometer values, and LiDAR points with timestamps) together with the truth at every instant: pose,
velocity, gyro bias and acceleration offset. A synthetic recording SHALL be served to the processing stages the way a
recording folder is, so that the orientation estimate, the window cutting and the window loop run unchanged on it,
without recorded data.

- **The model world:** with its default parameters a recording SHALL obey the estimator's own noise claims: each
  sensor error is drawn from the distribution the corresponding weight states, the sensors share one clock, one
  position and one set of axes, and the test states every setting it changes to make a claim true.
- **Named deviations:** each of these SHALL be switchable on its own: a lever arm of the accelerometer, a time offset
  between LiDAR and IMU, a rotation between their axes, a body-fixed accelerometer bias, a gyro scale error,
  quantised gyro values, and LiDAR points stamped between the trajectory nodes.
- **The generator is tested:** integrating a recording's noise-free IMU samples independently SHALL reproduce its
  true trajectory within a stated tolerance, the statistics of its noise SHALL match its parameters, and its points
  SHALL lie on the scene.

#### Scenario: The generator reproduces its own truth
- **WHEN** the noise-free IMU samples of a synthetic walking recording are integrated by a strap-down integration that shares no code with the estimator
- **THEN** the integrated poses and velocities equal the recording's truth within the stated tolerance

#### Scenario: The noise is what the parameters say
- **WHEN** a synthetic recording at rest is generated with a given accelerometer noise density, gyro noise density and random walks
- **THEN** the sample statistics of its IMU values match those parameters within their sampling error

#### Scenario: A recording goes through the real stages
- **WHEN** a synthetic recording is processed through loading, orientation estimate, window cutting and the window loop
- **THEN** the run completes without recorded data, and each window's result can be compared with the truth at its timestamps

### Requirement: Noise-free recordings are reproduced
The unit tests SHALL run the window loop on noise-free model-world recordings (rest, constant velocity, constant
acceleration, a constant turn, walking) and SHALL check, per window, that the state handed to the next window
(velocity, gravity vector, gyro bias, pose) equals the truth and that no carried state is moved away from its prior,
each within a tolerance derived from the rest and constant-velocity cases and stated in the test. A motion for which
the present code is known to deviate SHALL be a case marked as expected to fail, with the finding it shows and its
cause; where the size of today's deviation was predicted, a second case SHALL assert that size.

The tests SHALL also compare two consecutive windows solved in sequence with the same nodes solved as one window, on
a noise-free recording with the random walks of the carried states set to zero: the end state and its covariance
SHALL agree within a stated tolerance, or the case SHALL be marked as expected to fail with its cause.

#### Scenario: Rest and constant velocity
- **WHEN** a noise-free recording at rest, and one at constant velocity, is processed
- **THEN** every window's handed-on state equals the truth and no carried state moves from its prior, within the stated tolerance

#### Scenario: A known deviation is recorded, not hidden
- **WHEN** a noise-free recording with a constant acceleration is processed by code that has the hand-off finding F2
- **THEN** the case that states the honest outcome is marked as expected to fail with that finding, and the case that asserts the predicted size of the deviation passes

#### Scenario: A fix removes the marker
- **WHEN** a case marked as expected to fail starts to pass
- **THEN** the test run reports it, so that the marker is removed with the fix

### Requirement: Claimed uncertainty is tested against actual error
The unit tests SHALL process repeated model-world recordings that differ only in their noise, and SHALL compute, for
the same window of the route in every run, the normalised error of each state handed to the next window (velocity;
gravity vector against true gravity plus the true offset; both jointly; gyro bias) against its claimed covariance.
Summed over the runs, the value SHALL be compared with the chi-square interval for that number of runs.

- **Where the model world is exactly true:** in windows without LiDAR information along travel, for the components
  along travel of velocity and offset, the value SHALL lie inside the interval with the settings under test.
- **Controls:** the same check SHALL fail when the recordings' accelerometer noise is generated three times stronger
  or three times weaker than claimed, and SHALL fail with `imu_edges_follow_state` and `imu_carry_joint_covariance`
  off. The statistic itself SHALL be tested on a linear problem that is honest by construction.
- **Known shortfalls:** window classes and states for which the present code is known not to meet the interval (the
  first windows after a stretch without LiDAR information, windows the LiDAR dominates, the gyro bias) SHALL be cases
  marked as expected to fail, with the finding and the measured factor.
- **Size:** a small set of runs SHALL be part of every test run within a stated time budget; the full set SHALL be
  runnable on demand with one command.

#### Scenario: The inertial carry is honest where its model holds
- **WHEN** the full set of model-world recordings of the out-and-back corridor route is processed with both honesty switches on
- **THEN** for the blind windows the summed normalised error of the along-travel velocity and offset lies inside the chi-square interval

#### Scenario: The test can fail
- **WHEN** the same recordings are generated with three times the claimed accelerometer noise, or processed with both honesty switches off
- **THEN** the same check lies outside the interval

#### Scenario: A known shortfall is recorded
- **WHEN** the first windows after a blind stretch are checked on code that holds each window's first pose exact
- **THEN** the case is marked as expected to fail with finding F1 and reports the mean normalised error

### Requirement: Audit findings are pinned by known-answer cases
For every finding of the 2026-11-20 audit that was checked in the code and has a synthetic set-up with a known
answer, the unit tests SHALL hold a case that states the honest outcome. While the defect exists the case SHALL be
marked as expected to fail, with the finding's identifier and cause; a case for newly added code SHALL pass. The
cases SHALL cover at least: a displaced first pose of a window (F1), the velocity prior over the gap between windows
(F2a), the re-propagated tail under acceleration (F2b), the rest test with a tilted orientation estimate (F3), the
normalised move against the previous velocity statistic (F6), the step of a fractional degeneracy projection (C16),
the LiDAR term's claimed against its actual scatter (F4), and the rotation dependence of the cell test.

#### Scenario: A displaced first pose
- **WHEN** a window in a fully seen room is solved with a velocity prior at the truth and only its first node displaced by 10 mm along travel
- **THEN** the honest outcome (the end velocity within its claimed uncertainty of the truth) is asserted, and on code with finding F1 the case is marked as expected to fail

#### Scenario: The normalised move has an expected value of one
- **WHEN** a scalar state with a Gaussian prior is updated by one measurement of the same variance, over many draws
- **THEN** the normalised move averages 1 and the previous statistic averages 0.5
