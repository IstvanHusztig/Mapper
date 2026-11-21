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
The unit tests SHALL run the consistency pass on a synthetic recording with a known true trajectory, without recorded data, and SHALL check the result against that truth. The tests SHALL set the consistency options themselves.

#### Scenario: Offsets between windows
- **WHEN** each window's poses are moved by a small rigid offset, as the seams between odometry windows leave them
- **THEN** the pass reduces the mean pose error against the truth and the number of occupied 3 cm cells of the map

#### Scenario: A trajectory that is already right
- **WHEN** the trajectory equals the truth
- **THEN** every node stays within 5 mm and 0.1° of the truth after the pass

#### Scenario: A missed expectation
- **WHEN** the pass does not meet one of these expectations
- **THEN** that case is marked as an expected failure that names the cause, so that it is reported once it starts to pass

### Requirement: Window capture tested on synthetic data
The unit tests SHALL run the window loop on the synthetic loop recording with a capture, without recorded data, and SHALL
check that the capture leaves the run unchanged, that the capture file gives the capture back, and that a replay with the
same settings repeats every captured window bit for bit.

#### Scenario: Capture, file and replay
- **WHEN** one lap of the loop recording is run with a capture of 5–12 s, the capture is written to a file, read back and replayed with the same settings
- **THEN** the poses of the run equal those of a run without capture, and every replayed window equals the captured one

#### Scenario: The range of a capture
- **WHEN** windows start at 0, 2, 4, 6 and 8 s and the capture range is 1–5 s
- **THEN** the captured windows are those starting at 2 and 4 s, and a range in which no window starts captures nothing

### Requirement: Window optimization terms tested one by one
The unit tests SHALL test each term of the window optimization on its own, on a synthetic window, without recorded data:
- **The cost relation:** for every term with a cost, the right-hand side the term adds SHALL equal minus half the
  numeric gradient of its cost, over every unknown the term touches. For a term whose residual is linear in the
  unknowns, the matrix diagonal SHALL equal half the numeric second derivative.
- **The NDT point terms:** checked with the degeneracy projection off, with and without the Huber kernel and the
  information scale.
- **The degeneracy projection:** it SHALL pass the node blocks unchanged when every direction is strong, remove a weak
  translation direction from them, and add nothing when it is off.
- **The origin fix:** it SHALL weigh only the first node's six parameters.

#### Scenario: A term's gradient
- **WHEN** the motion-model edges are built on a turning window whose poses differ from the motion model, with bias estimation on
- **THEN** each pose parameter's and each bias component's right-hand side equals minus half the central-difference gradient of the edges' cost

#### Scenario: The degeneracy projection
- **WHEN** the window's normal information has a share of 0.05 along z, below `degenerate_projection_ratio` 0.1
- **THEN** each node block B enters the matrix as M B M and its right-hand side g as M g, with M removing the z translation

### Requirement: Per-term window table tested on synthetic windows
The unit tests SHALL test the per-term window table without recorded data:
- **Completeness:** the weights and pulls of the rows, summed, SHALL equal those of the window's complete normal
  equations, within a tolerance.
- **Each motion against a known answer:**
  - a guess moved sideways by a known amount: the NDT shift pull sideways points back, between 75 % and 105 % of the
    amount at 1 cm (one linear step: points that change their cell make NDT's cost non-quadratic);
  - a guess stretched along travel: the NDT stretch pull points back, within the same bounds;
  - a trajectory with a constant acceleration against a guess without it: the accelerometer's curve pull is the
    missing end displacement;
  - poses turned against the motion model by a heading ramp: the edge rotations' turn pull about vertical is minus the
    ramp.
- **Directions:** in a corridor, the NDT weight along the corridor SHALL be far below the weight across it.
- **No effect:** the window loop on the synthetic recording SHALL give the same poses, bit for bit, with the table on
  and off.
- **Files:** the CSV output SHALL hold the expected columns and one row per window, state and row.

#### Scenario: The rows add up
- **WHEN** the table is computed on a synthetic window with the accelerometer term, bias estimation and the degeneracy projection on
- **THEN** each motion's summed weight and weighted pull over the rows equal those reduced from the complete normal equations

#### Scenario: A known sideways offset
- **WHEN** the guess of a room window is moved 1 cm sideways from the true poses
- **THEN** the NDT shift pull sideways, minus its value at the true poses, is between −1.05 cm and −0.75 cm

### Requirement: Solver tested without a map
The unit tests SHALL test the Levenberg-Marquardt solver on a small problem that has no window, map or term:
- one iteration SHALL linearize, hand the covariance of the undamped unknowns (from the undamped matrix) to the
  problem, and apply a step; without a step it SHALL end after the covariance and leave the state unchanged;
- the damping factor SHALL grow tenfold after iterations 10, 20, …, and at once after a step the problem refuses;
- the iterations SHALL end when the update falls below the convergence threshold, at the iteration limit, at a forced
  last iteration, or when the time callback says so, and the result SHALL name which;
- with the step check, a step that raises the cost SHALL be taken back, the damping raised, and the rejection counted;
  without it no state SHALL be saved.

#### Scenario: A refused step
- **WHEN** the small problem refuses the step of iteration 2
- **THEN** iteration 3 runs with ten times the damping, and the state is the one before the refused step

#### Scenario: Why it stopped
- **WHEN** the small problem converges before the iteration limit
- **THEN** the result says converged and gives the iteration it ended after

### Requirement: Window verdict tested on made-up evidence
The unit tests SHALL test the window verdict without recorded data:
- a window SHALL have a flag only above its limit (bent above 3°, bent severely above 10°, fast above 2.5 m/s, bias
  high above 0.5 °/s), and none at the limit;
- a reading that is missing SHALL give `NOT MEASURED`;
- the run summary SHALL give the counts, shares, first and worst window, coverage and triggers of a made-up list of
  windows (exactly 1 % of the windows bent is clear, more is a trigger), and the chi² statistics over the windows with
  an information ratio below 0.45 and over all;
- the window loop on the synthetic recording SHALL give the same poses, bit for bit, with `log_window_verdict` on and
  off.

#### Scenario: At the limit
- **WHEN** a window's heading bend is exactly 3.0° and another's is 3.01°
- **THEN** only the second is bent

#### Scenario: Missing evidence
- **WHEN** a window has no end velocity
- **THEN** its speed is `NOT MEASURED`, and the summary counts it as not measured, not as clear

### Requirement: Window noise model tested on synthetic windows
The unit tests SHALL build the window noise model on synthetic windows with known timestamps and settings, without
recorded data, and check each weight against its formula and against what the window's terms used before the terms
took the model.

#### Scenario: Each weight equals its formula
- **WHEN** the model is built for a synthetic window with known node timestamps and settings
- **THEN** the time-scaled rotation sigma of each edge equals max(k·ARW·√dt, floor), with the turn windows' k; the translation weights carry the position information scale only with the accelerometer term; each accelerometer weight equals 1/((density²/τ + floor²)·τ²) for its span; the priors' covariances equal the carried ones grown by their random walk over the window or the gap

#### Scenario: Conditions as in the terms
- **WHEN** the model is built for the first window, for a window at rest, and for a window without the accelerometer term
- **THEN** the first window has no gyro-bias, accelerometer or velocity-prior weights; the window at rest has the zero-velocity weight; the velocity-continuity weights exist only without the accelerometer term

#### Scenario: Same weights as the terms computed
- **WHEN** the model is built for a synthetic window before the terms are switched to it
- **THEN** each of its weights is bit-identical with the weight the corresponding term computes for that window

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
