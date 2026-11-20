# Spec Delta

## ADDED Requirements

### Requirement: Synthetic recordings with inertial data and a known truth
The unit tests SHALL provide synthetic recordings that hold what a recorder produces (IMU samples with timestamps,
gyro and accelerometer values, and LiDAR points with timestamps) together with the truth at every instant: pose,
velocity, gyro bias and acceleration offset. The recordings SHALL be cut into windows by the same stages that cut a
recorded folder, without recorded data.

- **The model world:** with its default parameters a recording SHALL draw each sensor error as the estimator's
  corresponding weight states, and SHALL follow the estimator's time conventions for gyro samples, accelerometer
  samples and the assignment of points to trajectory nodes. The tests SHALL state, per claim of the estimator,
  whether the recording makes it true, conditions on it, or leaves it approximate.
- **The generator is tested:** integrating a recording's noise-free IMU samples independently SHALL reproduce its
  true trajectory within a stated tolerance; the statistics of its noise SHALL match its parameters; its points
  SHALL be the true surface hits plus the specified noise; and every generated point SHALL be kept with the node it
  was generated for, including at window boundaries.

#### Scenario: The generator reproduces its own truth
- **WHEN** the noise-free IMU samples of a synthetic walking recording are integrated by a strap-down integration that shares no code with the estimator
- **THEN** the integrated poses and velocities equal the recording's truth within the stated tolerance

#### Scenario: The noise is what the parameters say
- **WHEN** a synthetic recording at rest is generated with given noise densities and random walks
- **THEN** the sample statistics of its IMU values match those parameters within their sampling error

#### Scenario: No point is lost or moved to another node
- **WHEN** a synthetic recording is cut into windows by the real stages
- **THEN** every generated point is in the window and at the node it was generated for, including points of a window's first and last node

### Requirement: The statistics are tested on a problem that is honest by construction
Before a statistic is used on the estimator, the unit tests SHALL apply it to a small linear-Gaussian problem written
in the test, whose claimed covariance is correct by construction.

- The normalised-error test SHALL accept the honest problem and SHALL reject it when the reported covariance is
  scaled down or up, in a case where the scaled quantity dominates the tested state.
- Solving the problem as one batch and by exact sequential elimination of the same factors SHALL give the same mean
  and covariance to rounding.
- The normalised move of a prior to its posterior SHALL average 1, with the truth drawn from the prior and an
  independent measurement error.

#### Scenario: The test can fail in both directions
- **WHEN** the honest problem's reported covariance is scaled by a factor below 1, and by a factor above 1
- **THEN** the normalised-error test rejects on the high side and on the low side respectively

#### Scenario: The normalised move has an expected value of one
- **WHEN** a scalar state with a Gaussian prior is updated by one measurement of the same variance, over many draws
- **THEN** the normalised move averages 1, and the move divided by the prior variance alone averages 0.5

### Requirement: Each term is tested against the truth
The unit tests SHALL evaluate each term of the window optimization on windows cut from synthetic recordings, at the
true state.

- **Noise-free:** the term's residual SHALL be zero within a bound derived for that term and motion and stated in
  the test (rest, constant velocity, constant acceleration, turning in place, a turn with centripetal acceleration,
  walking).
- **With model-world noise:** where the test makes the term's noise claim true, the whitened residuals SHALL have
  unit variance and no covariance between terms, within their sampling error; where the claim is only approximate,
  the measured variance and covariance SHALL be reported as the size of the approximation.

#### Scenario: A term agrees with the generator
- **WHEN** the accelerometer terms of a window cut from a noise-free recording with constant acceleration are evaluated at the truth
- **THEN** every residual is zero within the stated bound

#### Scenario: A term's noise is what its weight says
- **WHEN** the accelerometer terms are evaluated at the truth over many model-world recordings whose term boundaries fall on sample boundaries
- **THEN** each whitened residual has unit variance and neighbouring terms are uncorrelated, within the sampling error

### Requirement: A window's claimed covariance is tested against its actual error
The unit tests SHALL solve windows whose prior error is drawn from the prior's claimed covariance, with model-world
inertial data and with the LiDAR's information set and verified by the test, and SHALL compare the error of the state
the window hands on with the covariance it claims.

- The instant, the axis or state, the primary comparisons and the decision rule over them SHALL be fixed before the
  runs. Every modelled random quantity SHALL be redrawn in every run.
- For a state of r tested dimensions over M runs, the sum of the normalised errors SHALL be compared with the
  chi-square interval for M r degrees of freedom. Windows of one run SHALL NOT be pooled as independent draws.
- A test that asserts honesty SHALL state the size of deviation it is able to detect; a set too small for that
  SHALL assert only that the runs complete and their preconditions hold.
- Missing windows and the coverage of every reported class SHALL be part of the result.

#### Scenario: The inertial terms and the carry where their model holds
- **WHEN** windows without LiDAR information along an axis are solved over the full set of runs, with that absence verified by the test
- **THEN** the summed normalised error of the handed-on velocity and offset along that axis is compared with its chi-square interval, and the result is reported with the detectable deviation

#### Scenario: A small set claims nothing it cannot show
- **WHEN** the small set that is part of every test run has finished
- **THEN** it has asserted that the runs complete with finite values and valid preconditions, and no honesty result

### Requirement: The hand-off is tested from a known boundary state
The unit tests SHALL run the loop's own steps from a carried state and a reference map that the test sets, and SHALL
check what the next window is actually given: its first pose, the mean and covariance of its priors after their
growth, and the points' poses that enter the map.

#### Scenario: The prior over the gap
- **WHEN** a window with a known exact end state is followed by the next window after a gap, under a constant acceleration
- **THEN** the mean of the next window's start-velocity prior is compared with the true start velocity

#### Scenario: A displaced reference map
- **WHEN** a window whose inertial data and relative motions are exactly consistent is solved against a map displaced along travel by a known amount, and again without the displacement
- **THEN** the change of its start and end velocity and its claimed covariance are reported for both

### Requirement: Known discrepancies are recorded, not hidden
A discrepancy between the estimator and the truth that was reproduced and isolated in a fixture SHALL be held by a
case that asserts the one honest outcome and is marked as expected to fail, with the finding's identifier and its
cause. The fixture's own validity SHALL be asserted in cases that pass. A discrepancy that was only measured, or only
predicted, SHALL be reported and SHALL NOT be marked. When a marked case starts to pass, the test run SHALL report
it.

#### Scenario: A reproduced discrepancy
- **WHEN** a fixture shows that the next window's prior mean differs from the true start velocity by the acceleration times the gap
- **THEN** a case that asserts equality is marked as expected to fail with that finding, and the fixture's validity cases pass

#### Scenario: A fix removes the marker
- **WHEN** a case marked as expected to fail starts to pass
- **THEN** the test run reports it, so that the marker is removed with the fix

### Requirement: The window loop is measured on synthetic recordings
The unit tests SHALL be able to run the complete front end and window loop on model-world recordings and report, per
class of window (at rest, seen by the LiDAR, without LiDAR information along travel, after such a stretch, turning),
the errors against the truth, the normalised errors and the per-window readings. This is a measurement: it SHALL NOT
by itself mark anything as an error of the code.

- **Preconditions:** a run SHALL verify and report that the stretch meant to be without LiDAR information has
  near-zero translation information along the true axis and keeps the information across it, that the structure seen
  after it was mapped before, and that the reference map was not rebuilt. A run whose precondition fails SHALL be
  reported as such and SHALL NOT be used.
- **Comparisons** between settings SHALL use the same recordings and seeds.
- A short run SHALL be part of every test run: it completes, its preconditions hold, and it repeats bit for bit.

#### Scenario: A run that is not what it was meant to be
- **WHEN** the corridor of a synthetic recording turns out to give LiDAR information along its axis
- **THEN** the run reports the failed precondition and its numbers are not used

#### Scenario: Two settings on the same world
- **WHEN** the loop is run with `imu_edges_follow_state` and `imu_carry_joint_covariance` off and on
- **THEN** both runs use the same recordings and seeds, and the report shows them side by side per class of window
