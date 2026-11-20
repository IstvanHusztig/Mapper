# Design: consistency-harness

Revised twice on 2026-11-20 after the independent reviews `review-codex.md` and `review-codex-2.md` (both "not
ready"); the last section maps every review item to what changed.

## Context

See `proposal.md` for the motivation and `research.md` for the evidence (findings F1-F8, their limits, the lineage
table). This file says how the question "is what the estimator claims as good as it says?" is turned into tests and
readings.

What exists and is reused:
- `OdometryPipeline` takes its recording through the `RecordingSource` port and hands each window's record to a
  `WindowTermSink`. The front-end functions (`load_data`, `calculate_trajectory`, `compute_step_1`) and the loop
  (`start_window_loop`, `process_windows`, `optimize_windows`) are public, so a test can cut real windows from
  synthetic samples and run the real loop steps from a state it sets itself, as a replay does from a capture.
- `tests/synthetic_recording.{h,cpp}`: scenes of plane faces, their sampling, `loop_recording` (no IMU data).
  `tests/scan_to_map_test.cpp`: one window against a map of true points, a corridor, the blind-chain cases of
  `honest-inertial-carry`, and the measured cost of a plane in its cell (a perfect guess moves 0.34 mm at best,
  2.26 mm with faces on cell boundaries). `src/application/memory_point_store.h`, `tests/serial_executor.h`.
- The repository's convention for a known shortcoming: a doctest case marked `should_fail` with the cause in a
  comment; when it starts to pass, doctest reports it and the marker goes.

Constraints: no result changes; tests need no recorded data; the verification region never edits `src/`; nothing
goes into a settings file; the velocity chi2, its limits and its reporting (contract of 2026-11-12) are not touched.

**Words.** The Consistency pass (`use_consistency`) already owns the word "consistency" here, so:
- *honest:* a claimed covariance matches the scatter of the actual error.
- *model world:* synthetic data in which every sensor error is drawn as the estimator's weights assume, in the
  estimator's own time conventions.
- *normalised error* of a state with claimed covariance P: e^T P^-1 e for the error e (estimate minus truth).
- *normalised move:* the same for the move from a prior to the window's result, scaled to an expected value of 1.
- *fixture:* a test set-up on the production code in which the inputs, the boundary state and what the LiDAR can
  observe are set by the test and checked.
- *oracle:* an independent, verified statement of what the honest outcome is.

## Goals / Non-Goals

**Goals:**
- A failure is attributed to the code or the scheme only where that attribution is valid (D1); every finding of
  `research.md` that was checked in code gets a fixture that measures it and, where an oracle exists, pins it.
- The audit's predictions are tested, not trusted.
- The instruments are tested before they are used (D2).
- The same definitions on a real run and on a synthetic run.

**Non-Goals:**
- Fixing any finding; changing a limit, a setting file or a baseline.
- Runs with real-world deviations, the lever arm on a recording, regressions on raw data (see Follow-ups).
- Truth tests of loop closure and the consistency pass; a realistic LiDAR simulator.
- A verdict on the map.

## Decisions

### D1. Attribution only where the observation model is known

On the full loop a failure cannot be sorted "by construction": a noise-free point on a wall still has a residual to
its cell's mean (0.34-2.26 mm of motion for a perfect guess, `scan_to_map_test.cpp:272-292`); a window's LiDAR
measurement and its prior share the errors of the windows before it; removing the LiDAR's translation information
along an axis leaves the attitude coupling and the errors the stretch was entered with; and a pass can hide errors
that cancel.

So there are four objects, and what each may conclude is fixed in advance:

| Object | What it is | May conclude |
|---|---|---|
| Exact control (D2) | a linear-Gaussian problem, honest by construction, no production code | that the instruments work: test direction, power, the reading |
| Fixtures (D5) | production terms, window problem and loop steps with inputs, boundary state and observable directions set and verified by the test | "the code or the scheme is wrong under its own model", only for a quantity the fixture isolates **and** an oracle states |
| The loop on synthetic recordings (D8) | the real front end and loop on a model-world recording | measurements with their preconditions; a paired comparison between settings; no attribution |
| Real recordings (D10, D11) | readings and the report tool | measurements; no attribution |

### D2. The exact control: testing the instruments

A small chain of linear-Gaussian factors written in the test (increments with known noise, absolute measurements with
known covariance, a prior), solved as one batch and by exact sequential elimination.

- Batch and sequential elimination of the **same** factors agree to rounding in mean and covariance. (Two cuts of a
  production window are not the same factors; that comparison is a sensitivity and a follow-up.)
- The normalised-error statistic and its quantiles are verified deterministically (known inputs, known values).
- The test accepts the honest chain, and rejects it when the reported covariance is multiplied by a **prespecified**
  factor and by its reciprocal, on a state that the scaled quantity dominates, with the number of runs chosen as the
  smallest that meets the power target in both directions (D6). A factor arbitrarily close to 1 cannot be detected,
  and tripling one noise source does not guarantee rejection for every state; the control is built where it does.
- The normalised move of D10 averages 1 there, and the old velocity statistic's form averages 0.5, with the truth
  drawn from the prior and an independent measurement error of equal variance.

### D3. The synthetic recording, in the estimator's conventions

The generator produces what a recorder produces (IMU samples, timed LiDAR points) plus the truth (pose, velocity,
gyro bias, acceleration offset at any instant). The real `calculate_trajectory` and `compute_step_1` cut it into
windows, so no test keeps its own copy of what a window looks like.

- **Motion:** analytic and twice differentiable, from segments joined smoothly: rest, speeding up, constant
  velocity, constant acceleration, turning in place, a turn with centripetal acceleration, walking (the measured
  Livox 3 dynamics: about 0.5 / 0.5 / 0.9 m/s2 rms per body axis, 11 / 11 / 32 deg/s rms, 2 Hz steps).
- **The estimator's time conventions, generated as they are used** (checked in code):
  - the orientation filter uses gyro sample k for the interval *ending* at node k, and its first update uses a 5 ms
    step; the model-world gyro value is the mean body rate over that interval. Where the rotation axis changes,
    integrating a mean rate as one rotation has an error, which gets a derived bound;
  - the accelerometer term holds sample k over the interval *starting* at node k, rotated with node k's attitude;
    the model-world value is defined for that use, and under rotation the difference between "interval mean of the
    body force rotated at the left end" and "mean world force", like the position quadrature, is a discretisation
    term with a derived bound, not a zero;
  - a point is assigned to the last node *before* its timestamp (`lidar_odometry.cpp:381-385`): a point stamped
    exactly at node k goes to node k-1, and at a window's first node it is dropped. Model-world points for node k
    are generated at pose k and stamped strictly inside node k's interval (the next representable time after it).
- **The source behaves as the production source does:** it applies the gyro calibration, rejects zero-time points,
  applies the XY filter to the raw point and then the lever-arm translation.
- **Which points are checked.** The first 60 001 accepted points are the map seed and set the first eligible IMU
  time; a slice excludes its upper bound, so the recording has a stationary seed prefix and one IMU sample beyond the
  last tested full window. The slice gate (more than 1 000 assigned points) acts on the original points, before the
  3 cm decimation. So: every **eligible** tagged hit is checked in the windows' original point sets for the node it
  was generated for; the retained representatives and their coverage are checked separately in the decimated sets.
- **What else the front end needs** (verified by the generator's tests): a complete recording, point store and
  preview, an orientation estimator and an executor; IMU timestamps increasing, in seconds, gyro in rad/s,
  acceleration in the sensor's g; the first window at rest; the tested motion after the filter's 3 s ramp; the
  expected window count and timestamps.
- **Sensor profile:** Livox timing by default (500 Hz, 180-node windows), because several settings count samples.
  Fewer points than the real sensor, with the emitted range limited explicitly. Where a fixture needs the terms'
  boundaries on sample boundaries (D4), it uses a node interval chosen for that and says so.
- **The generator is tested itself:** an independent strap-down integration of its noise-free samples reproduces its
  truth; its noise statistics match its parameters; its points are the latent hits on the scene plus the specified
  jitter.

Switches for real-world deviations (lever arm, time offset, points stamped where they were taken, ...) are designed
as single parameters but built in the follow-up.

### D4. The model world: four kinds of claim

A claim is "made true" only if a test verifies it. Each row is checked before any honesty test relies on it.

- *generated raw noise:* the recording draws the error the weight states;
- *conditional observation:* the term states something that is exactly true under a condition, with no random
  observation behind it;
- *propagated claim:* a covariance the code derives from earlier claims;
- *approximate factor:* no data error corresponds to the term's weight.

| Claim of the estimator | In the model world | Kind |
|---|---|---|
| Accelerometer term: variance (density^2 / tau + floor^2) tau^2, independent between terms | white sample noise at D = sqrt(density^2 + floor^2 tau0) (0.01057 m/s2/sqrt(Hz) at tau0 = 11 ms), settings: that density, floor 0. This matches the shipped weight at tau0 only. With samples of variance D^2 / dt_i a term's mean has variance D^2 sum(l_i^2 / dt_i) / tau^2 (l_i its overlap with sample i); a split sample feeds both neighbours; half spans, clipped end spans and the gap weigh differently | generated raw noise. The terms' noise contributions are independent only where their sample supports are disjoint (verified), and only conditional on the true rotations, gravity, offset and the fixed unit factor; for the Livox timing the variance and cross-covariance of the actual residuals are measured |
| Velocity prior over the gap: covariance D^2 gap + floor^2 gap^2; joint prior F P F^T; random walks over the window's duration | the offset and the gyro bias are constant inside a window and step between windows by their walk | propagated claim: the covariance algebra is tested on its own; the mean (which copies the old velocity, F2a) separately |
| Rotation edges: max(k ARW sqrt(dt), floor), independent | gyro white noise at ARW / sqrt(dt) in body rate, k = 1, floor 0, `ahrs_gain = 0` | generated raw noise, after the check that this gives the assumed edge variance behind the filter's ramp and the conversion to the edges' angles |
| Zero velocity in a window at rest (sigma 0.01 m/s at both ends) | at true rest the two observations are exact: their residuals at the truth have no scatter although the weight is finite | conditional observation. A noise-free constant-velocity window also passes the rest test, with `imu_always_on` even when `use_zero_velocity_detection` is off (`run_odometry.cpp:1040-1047`, `imu_kinematics.cpp:86-108`): a characterised case (A2) |
| Edge position rows (sigma 1e-4 m, scale beta = 0.1): the integrated guess restated; with layer 1 its shape | they reuse the window's IMU data | approximate factor (C34) |
| Origin fix (1e12 on node 0, a curvature on the current step) | the boundary pose is exact in the fixtures that say so | conditional observation |
| First window: gravity vector from its mean reading, start-up gyro bias, initial priors | estimates from noisy data | conditioned on: the first retained frame is taken as given, and a comparison that depends on it is reported as conditional |
| NDT term: a point is an independent Gaussian draw from an exact cell, in two grids, Huber 3, scale 0.1 | a point at the truth generally differs from its cell's mean; the map comes from the run's own windows | approximate factor; measured by K8 and in D8 |
| Sensors on one clock, one position, one set of axes | so | generated |

### D5. Fixtures on the production code

Each fixture names the quantity it isolates, lists every factor that is active in it, and asserts its own validity
separately from its result. Where an approximate factor remains active, the fixture's scatter is reported
descriptively and nothing is asserted.

- **F-a, a term at the truth (noise-free).** For the terms whose claim can be made true (accelerometer terms with
  start and end, rotation edges, the priors): the residual at the true state is zero within a bound derived for that
  term and motion. Not the NDT term, and not the position rows (approximate factors).
- **F-b, a term's noise.** Signed, whitened accelerometer residuals at the truth, with the true rotations, gravity,
  offset and unit factor, over model-world recordings: unit variance, and no covariance between terms with disjoint
  sample supports. For the Livox timing the measured variance and covariance are the size of the approximation.
- **How a production residual is observed.** The accelerometer writer only accumulates equations; its residual is
  local. A fixture passes one production span at a time and recovers the signed residual from the offset's
  right-hand side (-rhs_offset / (weight x sign x tau)); edges are checked through paired or isolated blocks. When
  the evaluation poses are set to the truth, the window's measured motion-model poses are kept as they are.
- **F-c, a window's claimed covariance.** The window problem with a prior whose error is drawn from its claimed
  covariance and model-true inertial data, the LiDAR's information controlled: none, or a corridor with exact cell
  statistics in which the test verifies near-zero projected translation information along the true axis and kept
  transverse information. The scatter of the handed-out end state against `take_covariance`'s blocks.
- **F-d and F-e, the hand-off and short chains through the real loop steps.** The boundary pose lives in the
  preceding window, not in the carried state, and the predictor rebuilds the current first pose from that window,
  its finite-difference velocity and the gap. So a fixture: initialises the loop's setup from a valid stationary
  window; starts at a window index above zero with window 0 and the predecessor history the predictor reads; stores
  indexed intermediate points; sets the carried state consistently (the velocity-valid flag, the end time, means and
  every covariance block); seeds the enabled grids with `update_rgd` from true points; and injects a boundary error
  in the **predecessor's** frame, verifying that it is still there after the window was prepared before any result
  is read. A LiDAR-free step stores an explicitly empty point set (the store distinguishes empty from absent). The
  map is exact at entry only: every finished window's solved points enter it.

**Error vectors** (they decide the degrees of freedom, so they are fixed here):
- The truth is aligned once, at the first retained node, which is conditioned on; world positions, velocities,
  gravity plus offset, the corridor direction and every covariance or projected axis use that one frame.
- *Velocity:* handed-on velocity minus the true velocity at the last node's time.
- *Gravity and offset:* in the raw accelerometer units. The carried vector starts as the first window's mean rotated
  reading and gains (offset estimate) / s per window, with s = 9.80665 / |g| recomputed from the vector before the
  fold. The error is s_before (g_after - g_true,raw), the truth (gravity plus the true offset) in the same raw units
  and frame, against the offset covariance (SI). Multiplying the vector by its own recomputed factor would normalise
  the magnitude error away.
- *Gyro bias,* in rad/s and sensor axes: e_b = estimated online bias - (generated body bias - the start-up bias
  actually subtracted).
- *Joint:* estimate minus truth for both parts, with the cross block.
- Controls for the definitions: a sensor scale other than 1 g, a rotated start, and a case whose cross covariance
  has a known sign.

### D6. Statistics

- **Law.** At one preselected instant, over M independent runs in which every modelled random quantity is redrawn
  (the prior's error included), the sum of the normalised errors is chi-square with M r degrees of freedom. This
  exact law is applied only to errors that are independent, centred and conditionally Gaussian after whitening;
  elsewhere the finite ensemble's outcome is reported. For a fixed truth axis u the error is projected first
  (A = u^T, r = 1; velocity and offset together r = 2) and so is the covariance (A P A^T).
- **Direction.** A claimed sigma that is too small (a ceiling against a growing error) drives the value **up**.
- **Alternative and size.** A prespecified variance multiplier (1.5) and its reciprocal; tail probabilities corrected
  for the number of primary comparisons; the smallest M that meets a 90 % power target in both directions. For one
  two-sided test at 1 %, 200 degrees of freedom give about 94 % power for 1.5 and 92 % for its reciprocal. With 6
  runs the scalar interval per degree of freedom is [0.11, 3.09]: a smoke check, not an honesty test.
- **Before sampling:** the instant, the axis, the primary comparisons, the decision rule, the numerical tolerances
  and the individual term comparisons are written down. Successive windows of one run are not pooled as independent
  draws; no window is chosen by its measured error; missing windows and the coverage of each class are reported.
- **No prospective markers.** The effect of the switches, the exit windows and the gyro bias are measured; a marker
  is set only by the rule of D7.

### D7. Cases, and "measure, then pin"

A discrepancy becomes a case marked as expected to fail only when it was reproduced and isolated in a fixture **and
an oracle that was verified separately states the honest outcome**. The marker sits on a case that asserts that one
outcome; the fixture's validity assertions are in passing cases beside it. Everything else is reported as a
measurement, or as a paired sensitivity (with and without one change). The audit's numbers are hypotheses.

| Case | Finding | Set-up (fixture) | What is measured; the oracle where there is one | Audit's hypothesis |
|---|---|---|---|---|
| A1 | - | rest (F-a, F-e) | states that are active after initialisation stay at the truth | passes |
| A2 | rest test | noise-free constant velocity through the loop steps | whether the window is taken as at rest and what the zero-velocity term then does; the constant-velocity reference itself in the window fixture with rest false | taken as rest |
| A3 | F2a, F2b | constant acceleration (F-a, F-d) | term residuals zero (oracle: the model itself); the boundary in K3 and K4 | |
| A4a / A4b | - | turning in place / a turn with centripetal acceleration (F-a) | term residuals within the derived discretisation bounds | unknown |
| A5 | - | walking (F-a, then F-e) | the terms' quadrature error and the map error, and the total; a measurement, the remainder is not attributed (the effects interact) | unknown |
| K1 | F1 | F-c in a seen room against a map displaced along travel by a known amount, paired with the zero-shift control; map, convergence, duration, prior and cross block specified | the signed sensitivity of start and end velocity per mm, and the claimed covariance beside it. The displaced map breaks the exact-map premise and a conditional covariance need not grow with a deterministic shift, so no outcome is asserted | [acc] 1.5-4 per T, [lidar] 1-1.5 per T |
| K2 | F1 | F-e: the exit of a blind stretch with a boundary error of known size injected in the predecessor's frame and verified retained | the velocity after the exit against its claimed sigma, window by window; paired with the zero-error control | a jump near error / T, then ringing (from one excitation) |
| K3 | F2a | F-d: a known exact preceding end state, a 2 ms gap, 2 m/s2 | the prepared prior mean; oracle: the true start velocity | 4 mm/s short |
| K4 | F2b | the tail re-propagation alone on p = a t^2 / 2 at 500 Hz, tail 15, span 10; and the next window's actual prediction | the last node; the next first pose as actually predicted; oracle: the truth | -0.75 mm at 1 m/s2; -1.66 mm at the next node at 2 m/s2 |
| - | F2 | F-e under constant acceleration, tail 15 and tail 0 | the start-velocity move, paired (tail 0 also changes the map) | +17-19 mm/s is a fitted hypothesis |
| K5 | F6 | the exact control (D2) | the normalised move averages 1 | must pass |
| K6 | F3 | level true poses, a constant tilted filter pitch, the true force, threshold 0.3 m/s2 | recognised as rest; oracle: the device rests | not recognised from about 1.75 deg |
| K7 | C16 | the isolated weak-axis block, s = 0.5 | the block's own step; oracle: at most g / k | 2 g / k |
| K8 | F4 | a scalar rigid shift with the other modes held (or the marginal variance of the shift actually solved); independent scan draws against a fixed true map, then with an earlier noisy map redrawn per trial; fine grid alone and both grids; a flat wall along travel | scatter / claimed sigma, with grid duplication, map noise and the plane's pull separated; descriptive (an approximate factor) | 1.4 and 1.8 under independent 1 cm noise; the in-plane response was measured non-zero |
| K9 | cell test | a vertical plane with stated tangent variances, its normal turned 45 deg in XY, the inverse refreshed | usable in both orientations or in neither (oracle: rotation invariance) | usable only when turned |
| K10 | C37 | the origin fix's rotation weight | oracle: 1 / sigma^2 with sigma in radians | 1e12 where 3.3e15 was meant; harmless |
| K11 | F5 | F-e with a perfect gyro and the boundary attitude turned by 0.05 deg in the predecessor's frame, verified retained, paired with the unturned control | the bias estimate's response against its claimed sigma (the boundary-tilt mechanism, as distinct from a general bias statistic) | leaves zero by 3-5 claimed sigmas |

### D8. The loop on synthetic recordings: an integration measurement

The real front end and `optimize_windows` on model-world recordings. Nothing here attributes; it shows what the whole
scheme does to an honest world, with the same definitions as on a real run.

- **Routes:** R1, the loop with pillars (seen everywhere); R2, out and back through a plain corridor (a first pass
  through unmapped space has nothing to be corrected against; the way back gives the re-observation, and it is what
  a dead-end cave passage is); R3, rest, walk, rest. The emitted range is limited so that R2's blind middle fits a
  short route (with the shipped 40 m range it would need more than 80 m of corridor).
- **Settings:** the stage 8 candidate set with exact cell statistics. The shipped Livox file uses the previous
  statistics, for which a plain corridor's information was 925 along against 155 across
  (`scan_to_map_test.cpp:522-525`), so a run with the shipped file is a second, separately reported measurement.
- **Preconditions of R2, predeclared and verified per run:** minimum counts of matched side and floor normals in the
  blind windows; a tolerance for the projected translation information along the true corridor axis (near zero) and
  for the transverse information (kept); at the nominated exit, full support along the axis and accepted matches to
  a set of **outbound landmark cells**, designated beforehand by their cell keys and disjoint from what the return
  leg could have mapped itself; no map rebuild; the time spent in each regime. The matches are computed by the test
  from the window's points and the grids, because the loop's translation matrix and match counts carry no identity
  of the visit.
- **Pairing:** a comparison between settings uses the intersection of the seeds that are valid under both, and
  reports the exclusions per setting.
- **Reported:** per class of window (rest, seen, blind by position, exit by position, turn) the errors, the
  normalised errors of D5 and the readings of D10; with both layer switches off and on.
- **In every `ctest`:** a short smoke run: it completes, its preconditions hold, declared deterministic fields
  repeat, unavailable readings are counted; a run that reaches the solver's wall-clock limit invalidates its timing.

### D9. The record, completed after the hand-off

Today the per-window record is built and written in the report step, before the offset is folded into gravity,
before the tail is re-propagated and before the cancellation check (`run_odometry.cpp:1172-1189`, then `:1201-1235`).

- The table and all existing columns stay as they are, taken at the same moment as today.
- Recorded right after the window was prepared, on **this window's** row: its first pose as prepared, and the prior
  means and covariances it was actually prepared with (after growth and fall-backs). The fixtures join adjacent rows
  for the hand-off; outside the processed range there is no successor measurement, and a replay may end before one.
- Appended after the hand-off: the carry actually handed on (velocity, gravity vector with the unit factor before
  the fold, offset covariance, cross covariance, gyro bias, validity of each); the last node's pose and time as
  solved and as handed on; the window's mean rotation rate, defined from its attitude increments (which a capture
  holds; raw gyro values are not available in a replay), and its world-frame acceleration at its start.
- In memory the record holds the full covariance matrices (velocity, bias, offset and the cross block); the CSV
  holds means and sigmas.
- Written once per window, at the end of the hand-off; a window whose hand-off is cancelled writes its record with
  the handed-on part marked absent.
- A record exists only with `log_window_terms`; the fixtures switch it on with their keeping sink.
- The window problem's diagnostic snapshot for D10 is taken out of the local problem before it goes out of scope.

Nothing feeds back into the odometry; `Window`, the capture file and the settings hash do not change.

### D10. Honesty readings on real runs

For a carried state with prior N(m, P), the window's result x and posterior covariance P_post, the move d = x - m
has covariance P - P_post in a linear-Gaussian update with a fixed observation model, so
E[d^T P^-1 d] = n - tr(P^-1 P_post), and

    q = d^T P^-1 d / (n - tr(P^-1 P_post))      has expected value 1.

Its distribution is a weighted sum of squares, not chi-square, and it is descriptive: P_post is conditional on the
fixed first node, the weights are re-weighted (Huber) and the problem is linearised.

- **Which covariance.** The solve computes the covariance before each step (`lm_solver.cpp:84-100`), so the one it
  ends with, the one the estimator hands on, belongs to the state one step before the final one; a final
  re-linearisation happens only with the step check, which is off by default. The reading uses that same covariance:
  it describes the claim that is actually carried. It is measured when the last accepted step moved the tested state
  by less than 1 % of its prior sigma in every component, which is the ordinary case after the damping schedule.
  *Alternative considered:* a fresh covariance on a diagnostic copy after the timed solve; it would add a second
  linearisation path to production code for a difference below that tolerance.
- **States:** the start velocity (n = 3), always; the gyro bias (n = 3); and, when the joint prior is used, start
  velocity and offset together (n = 6, d = [v_start - m_v, e - m_e], the full joint start prior and posterior with
  both cross blocks). The marginal velocity reading stays in joint mode too, so runs with and without the switch
  compare.
- **Computation:** whitened through a scale-aware Cholesky solve of P; the eigenvalues of the whitened posterior must
  lie in [0, 1] to numerical tolerance.
- **`NOT MEASURED`:** the term absent; the covariance absent, non-finite or ill-conditioned (a failed
  factorisation); a covariance "reduction" that is not physical; the last step above the tolerance (a window stopped
  early); a clamped bias; a trace reduction n - tr(P^-1 P_post) below 0.5. The validity follows a step that is taken
  back.
- **Tested** on a window with a non-zero last step and the step check off, where a finite reading is required, also
  in joint mode; a failed factorisation, a step taken back and a clamped bias each on their own. A summary with zero
  coverage does not pass that test.
- **Reporting:** largest, median, 95th percentile and coverage (also when it is zero) in the verdict summary; all
  readings on a flagged window's line; columns of `window_features.csv`. No limit, flag or trigger.
- **Unchanged:** the velocity chi2, its limits 5 / 20 on the weak windows (information ratio below 0.45) and its
  all-window statistics.

### D11. The tool, and evidence by replay

`tools/honesty_report.py` (pure Python, tests in `tests/test_verification_tools.py`): one table per
`window_features.csv`, side by side for several: the start-velocity move (median of the norm, per axis, by quarter of
the rotation rate and of the start acceleration, lag-1 autocorrelation, at rest) and the readings of D10, each under
one stated mask and definition, from the new columns. Where an input is missing it prints "unavailable"; it does not
rebuild a quantity another way. It gives no verdict.

The existing runs' tables do not have the new columns, and the audit's figures used other masks, raw gyro over two
windows and trajectories. So the evidence of this change is **replays of the existing captures with the new build**
(Livox 3, the stage 8 Livox 4 captures, 44_short), each labelled as conditional on its captured map and carried
state and as a forced replay; a capture holds no stop reason, so windows that stopped at the time limit are taken
from the original run's log. The regressions (lever arm, time offset) need time-resolved data and are a follow-up.

### D12. Who builds what, and the order of the gates

| Part | Region | Model / effort |
|---|---|---|
| Exact control, generator, fixtures, loop measurement, the tool | verification (never edits `src/`) | Opus / medium |
| Handed-on state in the record and the CSV | noise-budget | Opus / medium |
| Covariance blocks kept, readings in the verdict | solver | Opus / medium |
| `owners.toml`, `tests/CMakeLists.txt`, the shared loop steps (prepare, report, finish), documentation from the hand-backs, merges, references, replays, summaries | orchestrator | Opus / medium; reading the results Fable / high |

- Ownership and CMake entries land in the same commit as every new tracked file they name, in every group.
- The step checks compare whole CSV files against reference files that live outside the repository
  (`<guards>/refs`, with the commit they were recorded from). At the integration of the new columns: both tables'
  old-column prefixes are compared token by token with their references and every other reference is checked
  unchanged; only then are the references recorded again from the integrated commit, and that provenance (commit,
  date, what was verified) is written down. From there on "ALL HOLD" is required.
- "Results unchanged" means: the existing normalised manifest comparison and the exact deterministic outputs
  (replays with forced iteration counts, the term table); a comparison that depends on the time limit is flagged.
- New findings get their identifiers before a case is marked with them; their documentation is completed at the end.

## Risks / Trade-offs

- [The synthetic LiDAR is kinder than a real one] -> the LiDAR term's conclusions rest on K8 (with its assumptions
  stated) and on the readings of real replays; nothing is called a lower bound.
- [A generator error is read as an estimator error] -> the generator's own tests and F-a: a term's residual at the
  truth is checked before any honesty test relies on that term.
- [The model world's settings differ from the shipped ones] -> D4 says per claim what kind it is; the loop is
  measured with the candidate set and, separately, the shipped file.
- [Evidence is thin: one recording, one blind stretch, candidate settings] -> the design asserts no outcome from it;
  a result is worded "did not reproduce at this size, route, seed and settings", never "ruled out".
- [Markers hide a second defect] -> one assertion per marked case, an oracle behind it, validity in separate cases.
- [Monte Carlo time] -> the large sets are at term and window level, where a trial is one or two windows.
- [The readings invite a verdict] -> no limits; the summary states what they do not say.
- [The harness grows] -> the gates of D12 and the rule of D7; the follow-ups are named and left out.

## Migration Plan

No migration: defaults and results are unchanged. Order: the exact control; the record and readings (references
recorded again once); the generator with its tests; the fixtures, measured then pinned; the loop measurement; the
tool and the replays; the summary for the user. Rolling back is removing tests, columns and the tool.

## Follow-ups (named, not in this change)

- One real-world deviation at a time in the generator (lever arm, LiDAR-IMU time offset, a rotation between the
  axes, a body-fixed accelerometer bias, a gyro scale error, quantised gyro values, points stamped where they were
  taken, vibration, a range offset per surface), each compared from paired seeds with the same baseline settings.
- The Mid-360S lever arm: the mount's axis map from the recorder's source, and a run with it set.
- The regressions of the audit (hand-off, lever arm, time offset) with raw IMU data and trajectories.
- Sensitivity to the window length (two real cuts of the same recording).
- A reference marginal of the boundary state over the production terms of two windows, to size what the hand-off
  loses, and with it an oracle for K1 and K2.
- What happens to the layers of `honest-inertial-carry`.

## Open Questions

- Whether the on-demand sets become part of `tools/end_runs.py`: after their first timings.
- The final names of the new columns and summary lines: with the implementation, in the knowledge files.

## The reviews of 2026-11-20 and what changed

Both reviews are by an independent reviewer, read-only. Their claims about the code were checked by the orchestrator
before each revision; all that were checked read as the reviews say.

**First review (`review-codex.md`)**

| Item | Resolution |
|---|---|
| Blocking 1: attribution on the loop is not valid | D1; the loop is a measurement; tolerances from derived bounds |
| Blocking 2: the proposed timestamps create a systematic error | D3: the three time conventions; points stamped strictly inside their node's interval; F-a |
| Blocking 3: one density does not make term noise independent | D4 row 1; F-b |
| Blocking 4: other information sources, false rest | D4; A2 is a characterisation |
| Blocking 5: the error vector | D5 "Error vectors" with three controls |
| Blocking 6: direction, dimension, power | D6; no prospective markers; controls in D2 |
| Blocking 7: blindness and re-observation must be verified | D8 preconditions |
| Blocking 8: a different cut is a different likelihood | D2 (same factors only); the two-cut comparison is a follow-up |
| Blocking 9: the record precedes the hand-off | D9 |
| Should change: reading, cases, tool, gates, scope | D10, D7, D11, D12, proposal and Follow-ups |

**Second review (`review-codex-2.md`)**

| Item | Resolution |
|---|---|
| 1. The right population of points; the source's filter and calibration; the first gyro update; the rotation integration bound | D3 "The source behaves as the production source does", "Which points are checked", and the first time convention |
| 2. The exact experiment per factor (zero velocity has no scatter, the gap's mean, position rows, conditional independence) | D4's four kinds and its rows; D5 F-a and F-b limited to the terms whose claim can be made true; F-c lists its factors and is descriptive where an approximate factor remains |
| 3. Observing production residuals; the gyro error | D5 "How a production residual is observed" and the gyro bullet |
| 4. Hand-off fixtures need the predecessor's history | D5 F-d and F-e; the LiDAR-free step with an empty point set (the former open question) |
| 5. Demonstrate the observation claimed (K1, A5, R2) | D7: an oracle is required for a marker, K1 and K2 are paired sensitivities, A5 attributes nothing; D8: predeclared tolerances, outbound landmark cells, paired intersection of seeds |
| 6. The alternative and the calibration | D2 and D6 |
| 7. The prepared pose on its own row; full matrices in memory | D9 |
| 8. The reading must exist on its target path | D10 "Which covariance" and "Tested"; the rotation feature from attitude increments (D9); forced replays labelled (D11) |
| 9. Three dependency errors in the tasks | tasks 3.3, 4.3, 5.7 and 5.8; the same-commit rule in D12 |

Not taken over as written:
- First review: one "production-term fixture" became five (F-a to F-e), because term, window and hand-off isolate
  different quantities.
- Second review, item 8: a fresh covariance on a diagnostic copy after the timed solve. The reading uses the
  covariance the solve ends with, because that is the claim the estimator carries, with a tolerance on the last
  step; the test the review asks for (a finite reading with a non-zero last step and the step check off) is kept.
