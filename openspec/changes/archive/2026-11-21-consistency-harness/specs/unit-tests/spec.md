# Spec Delta

## ADDED Requirements

### Requirement: Synthetic recordings with inertial data and a known truth
The unit tests SHALL provide synthetic recordings that hold what a recorder produces (IMU samples with timestamps,
gyro and accelerometer values, and LiDAR points with timestamps) together with the truth at every instant: pose,
velocity, gyro bias and acceleration offset. The recordings SHALL be read and cut into windows by the same stages
that read and cut a recorded folder, with the same filtering and calibration of the samples, without recorded data.

- **The model world:** with its default parameters a recording SHALL draw each sensor error as the estimator's
  corresponding weight states, and SHALL follow the estimator's time conventions for gyro samples, accelerometer
  samples and the assignment of points to trajectory nodes. The tests SHALL state, per claim of the estimator, of
  which kind it is: an error the recording draws, a statement that is exact under a condition, a covariance derived
  from earlier claims, or a weight with no matching data error.
- **The generator is tested:** integrating a recording's noise-free IMU samples independently SHALL reproduce its
  true trajectory within a stated tolerance; the statistics of its noise SHALL match its parameters; its points
  SHALL be the true surface hits plus the specified noise.
- **The cut is checked:** every generated point that the cut is meant to keep SHALL be found in its window's
  original point set at the node it was generated for, including at window boundaries; the points kept after
  decimation and their coverage of the nodes SHALL be checked separately.

#### Scenario: The generator reproduces its own truth
- **WHEN** the noise-free IMU samples of a synthetic walking recording are integrated by a strap-down integration that shares no code with the estimator
- **THEN** the integrated poses and velocities equal the recording's truth within the stated tolerance

#### Scenario: The noise is what the parameters say
- **WHEN** a synthetic recording at rest is generated with given noise densities and random walks
- **THEN** the sample statistics of its IMU values match those parameters within their sampling error

#### Scenario: Points end up where they were generated for
- **WHEN** a synthetic recording with a stationary beginning and a sample beyond its last window is cut into windows by the real stages
- **THEN** every point the cut is meant to keep is in its window's original point set at the node it was generated for, and the decimated sets cover every node

### Requirement: The statistics are tested on a problem that is honest by construction
Before a statistic is used on the estimator, the unit tests SHALL apply it to a small linear-Gaussian problem written
in the test, whose claimed covariance is correct by construction.

- The statistic and its quantiles SHALL be checked on inputs with known values.
- The normalised-error test SHALL accept the honest problem, and SHALL reject it when the reported covariance is
  multiplied by a factor fixed beforehand and by its reciprocal, in a case where the scaled quantity dominates the
  tested state, with the number of runs chosen to meet a stated power in both directions.
- Solving the problem as one batch and by exact sequential elimination of the same factors SHALL give the same mean
  and covariance to rounding.
- The normalised move of a prior to its posterior SHALL average 1, with the truth drawn from the prior and an
  independent measurement error.

#### Scenario: The test can fail in both directions
- **WHEN** the honest problem's reported covariance is multiplied by the prespecified factor, and by its reciprocal, over the number of runs the power target requires
- **THEN** the normalised-error test rejects on the low side and on the high side respectively

#### Scenario: The normalised move has an expected value of one
- **WHEN** a scalar state with a Gaussian prior is updated by one measurement of the same variance, over many draws
- **THEN** the normalised move averages 1, and the move divided by the prior variance alone averages 0.5

### Requirement: Terms are tested against the truth
The unit tests SHALL evaluate, on windows cut from synthetic recordings and at the true state, each term of the
window optimization whose claim the model world can make true.

- **Noise-free:** the term's residual SHALL be zero within a bound derived for that term and motion and stated in
  the test (rest, constant acceleration, turning in place, a turn with centripetal acceleration, walking).
- **With model-world noise:** the signed, whitened residuals SHALL have unit variance, and terms that share no
  sample SHALL be uncorrelated, within their sampling error; where a claim holds only approximately for a sensor's
  timing, the measured variance and covariance SHALL be reported as the size of the approximation.
- **Terms that cannot be made true** (the LiDAR point terms, whose points differ from their cells' means even at the
  truth, and the position rows that restate the window's guess) SHALL be reported descriptively and SHALL NOT be
  asserted to be zero or honest.

#### Scenario: A term agrees with the generator
- **WHEN** the accelerometer terms of a window cut from a noise-free recording with constant acceleration are evaluated at the truth
- **THEN** every residual is zero within the stated bound

#### Scenario: A term's noise is what its weight says
- **WHEN** the accelerometer terms are evaluated at the truth over many model-world recordings whose term boundaries fall on sample boundaries
- **THEN** each whitened residual has unit variance and terms that share no sample are uncorrelated, within the sampling error

### Requirement: A window's claimed covariance is tested against its actual error
The unit tests SHALL solve windows whose prior error is drawn from the prior's claimed covariance, with model-world
inertial data and with the LiDAR's information set and verified by the test, and SHALL compare the error of the state
the window hands on with the covariance it claims.

- The test SHALL list the factors that are active in its set-up. Where a factor without a matching data error
  remains active, the scatter SHALL be reported descriptively and nothing SHALL be asserted.
- The instant, the axis or state, the primary comparisons, the decision rule over them and the numerical tolerances
  SHALL be fixed before the runs. Every modelled random quantity SHALL be redrawn in every run.
- For a state of r tested dimensions over M runs, the sum of the normalised errors SHALL be compared with the
  chi-square interval for M r degrees of freedom. Windows of one run SHALL NOT be pooled as independent draws.
- A test that asserts honesty SHALL state the deviation it is able to detect; a set too small for that SHALL assert
  only that the runs complete and their preconditions hold.
- Missing windows and the coverage of every reported class SHALL be part of the result.

#### Scenario: The inertial terms and the carry where their model holds
- **WHEN** windows without LiDAR information along an axis are solved over the full set of runs, with that absence verified by the test
- **THEN** the summed normalised error of the handed-on velocity and offset along that axis is compared with its chi-square interval, and the result is reported with the detectable deviation

#### Scenario: A small set claims nothing it cannot show
- **WHEN** the small set that is part of every test run has finished
- **THEN** it has asserted that the runs complete with finite values and valid preconditions, and no honesty result

### Requirement: The hand-off is tested from a known boundary state
The unit tests SHALL run the loop's own steps from a carried state, a preceding window and a reference map that the
test sets, and SHALL check what the next window is actually given: its first pose as prepared, the mean and
covariance of its priors after their growth, and the poses with which points enter the map. A boundary error that a
test injects SHALL be verified to be present after the window was prepared, before any result is read.

#### Scenario: The prior over the gap
- **WHEN** a window with a known exact end state is followed by the next window after a gap, under a constant acceleration
- **THEN** the mean of the next window's start-velocity prior is compared with the true start velocity

#### Scenario: A displaced reference map
- **WHEN** a window whose inertial data and relative motions are exactly consistent is solved against a map displaced along travel by a known amount, and again without the displacement
- **THEN** the change of its start and end velocity and its claimed covariance are reported for both, as a sensitivity

### Requirement: Known discrepancies are recorded, not hidden
A discrepancy between the estimator and the truth SHALL be held by a case marked as expected to fail only when it was
reproduced and isolated in a fixture and the honest outcome is stated by an oracle that was verified independently.
The case SHALL assert that one outcome and SHALL name the finding and its cause. The fixture's own validity SHALL be
asserted in cases that pass. A discrepancy that was only measured, or only predicted, SHALL be reported and SHALL NOT
be marked. When a marked case starts to pass, the test run SHALL report it.

#### Scenario: A reproduced discrepancy with an oracle
- **WHEN** a fixture shows that the next window's prior mean differs from the true start velocity by the acceleration times the gap
- **THEN** a case that asserts equality with the true start velocity is marked as expected to fail with that finding, and the fixture's validity cases pass

#### Scenario: A sensitivity without an oracle
- **WHEN** a fixture measures how a window's velocity responds to a displaced map
- **THEN** the response is reported beside its control and no case is marked

#### Scenario: A fix removes the marker
- **WHEN** a case marked as expected to fail starts to pass
- **THEN** the test run reports it, so that the marker is removed with the fix

### Requirement: The window loop is measured on synthetic recordings
The unit tests SHALL be able to run the complete front end and window loop on model-world recordings and report, per
class of window (at rest, seen by the LiDAR, without LiDAR information along travel, after such a stretch, turning),
the errors against the truth, the normalised errors and the per-window readings. This is a measurement: it SHALL NOT
by itself mark anything as an error of the code.

- **Preconditions:** for a route with a stretch meant to be without LiDAR information, a run SHALL verify, against
  tolerances fixed beforehand, that the translation information along the true axis is near zero there and the
  information across it is kept, that the matches after the stretch are to structure mapped on the earlier pass and
  identified as such by the test, and that the reference map was not rebuilt. A run whose precondition fails SHALL
  be reported as such and SHALL NOT be used.
- **Comparisons** between settings SHALL use the recordings and seeds that are valid under both settings, and SHALL
  report what was left out for each.
- A short run SHALL be part of every test run: it completes, its preconditions hold, its declared deterministic
  values repeat, and the readings it could not measure are counted.

#### Scenario: A run that is not what it was meant to be
- **WHEN** the corridor of a synthetic recording turns out to give LiDAR information along its axis
- **THEN** the run reports the failed precondition and its numbers are not used

#### Scenario: Two settings on the same world
- **WHEN** the loop is run with `imu_edges_follow_state` and `imu_carry_joint_covariance` off and on
- **THEN** the report shows both side by side per class of window, from the seeds valid under both, with the exclusions per setting
