# Design: consistency-harness

Revised 2026-11-20 after the independent review (`review-codex.md`, verdict "not ready"); the last section says what
changed and why.

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

## Goals / Non-Goals

**Goals:**
- A failure is attributed to the code or the scheme only where that attribution is valid (D1), and every finding of
  `research.md` that was checked in code gets a fixture that measures it and, once reproduced, pins it.
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

The first draft promised that every failure is sorted "by construction". The review showed why that cannot hold on
the full loop: a noise-free point on a wall still has a residual to its cell's mean (0.34-2.26 mm of motion for a
perfect guess, `scan_to_map_test.cpp:272-292`); a window's LiDAR measurement and its prior share the errors of the
windows before it; removing the LiDAR's translation information along an axis leaves the attitude coupling and the
errors the stretch was entered with; and a pass can hide errors that cancel.

So there are four objects, and what each may conclude is fixed in advance:

| Object | What it is | May conclude |
|---|---|---|
| Exact control (D2) | a linear-Gaussian problem, honest by construction, no production code | that the instruments work: test direction, power, the reading |
| Fixtures (D5) | production terms, window problem and loop steps with inputs, boundary state and observable directions set and verified by the test | "the code or the scheme is wrong under its own model", for the quantity the fixture isolates |
| The loop on synthetic recordings (D8) | the real front end and loop on a model-world recording | measurements with their preconditions; a comparison between settings from the same seeds; no attribution |
| Real recordings (D10, D11) | readings and the report tool | measurements; no attribution |

*Alternatives considered:* one Monte Carlo over the full loop with a pass/fail number (attributes nothing); tolerances
taken from "gentle" loop cases (they contain the LiDAR floor and the rest term, so they cannot set a tolerance).

### D2. The exact control: testing the instruments

A small chain of linear-Gaussian factors written in the test (increments with known noise, absolute measurements with
known covariance, a prior), solved as one batch and by exact sequential elimination.

- Batch and sequential elimination of the **same** factors agree to rounding in mean and covariance. (Two different
  cuts of a production window are not the same factors: anchors, end terms, origin fix, tail, rest decision, the
  layout of bias and offset and the time the first half enters the map all change. That comparison is a sensitivity,
  not a test, and is a follow-up.)
- The normalised-error test accepts the honest chain and rejects the same chain with the reported covariance scaled
  down and up, on a problem where the scaled quantity dominates the tested state. A control that cannot fail proves
  nothing; tripling one noise source does not guarantee rejection for every state, so the control is built where it
  does.
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
  - the orientation filter uses gyro sample k for the interval *ending* at node k; the model-world gyro value is the
    mean body rate over that interval;
  - the accelerometer term holds sample k over the interval *starting* at node k, rotated with node k's attitude; the
    model-world value is defined for that use, and where the body rotates, the difference between "interval mean of
    the body force rotated at the left end" and "mean world force" is a discretisation term with a derived bound,
    not a zero;
  - a point is assigned to the last node *before* its timestamp (`lidar_odometry.cpp:381-385`): a point stamped
    exactly at node k goes to node k-1, and at a window's first node it is dropped. Model-world points for node k
    are generated at pose k and stamped strictly inside node k's interval (the next representable time after it).
- **What the front end needs** (verified by the generator's own tests, not assumed): a complete recording, point
  store and preview, an orientation estimator and an executor; IMU timestamps increasing, in seconds, gyro in rad/s,
  acceleration in the sensor's g; all 60 001 seed points and the first window at rest; the tested motion after the
  filter's 3 s ramp; more than 1 000 assigned points per window slice after the 3 cm decimation; the expected
  window count and timestamps.
- **Sensor profile:** Livox timing by default (500 Hz, 180-node windows), because several settings count samples.
  Fewer points than the real sensor, with the emitted range limited explicitly. Where a fixture needs the terms'
  boundaries on sample boundaries (D4), it uses a node interval chosen for that and says so.
- **The generator is tested itself:** an independent strap-down integration of its noise-free samples reproduces its
  truth; its noise statistics match its parameters; its points are the latent hits on the scene plus the specified
  jitter; every generated point is retained with the node index it was generated for, including at window
  boundaries.

Switches for real-world deviations (lever arm, time offset, points stamped where they were taken, ...) are designed
as single parameters but built in the follow-up (Follow-ups).

### D4. The model world: what is generated, what is conditioned on, what stays approximate

A claim is "made true" only if the fixture verifies it. Each row is checked by a test before any honesty test uses it.

| Claim of the estimator | In the model world | Status |
|---|---|---|
| Accelerometer term: variance (density^2 / tau + floor^2) tau^2, independent between terms | white sample noise at D = sqrt(density^2 + floor^2 tau0) (0.01057 m/s2/sqrt(Hz) at tau0 = 11 ms), settings: that density, floor 0. This matches the shipped weight at tau0 only. With samples of variance D^2 / dt_i a term's mean has variance D^2 sum(l_i^2 / dt_i) / tau^2 (l_i its overlap with sample i); a split sample feeds both neighbours; half spans, clipped end spans and the gap weigh differently | **generated**, exact only where term boundaries fall on sample boundaries (the exact fixture chooses its node interval so); for the Livox timing the variance and cross-covariance of the actual weighted residuals are measured and reported |
| Velocity prior over the gap: D^2 gap + floor^2 gap^2; joint prior F P F^T; random walks over the window's duration | the same formulas; the offset and the gyro bias are constant inside a window and step between windows by their walk | **generated** |
| Rotation edges: max(k ARW sqrt(dt), floor), independent | gyro white noise at ARW / sqrt(dt) in body rate, k = 1, floor 0, `ahrs_gain = 0`; that this gives the assumed edge variance after the filter's ramp and the conversion to the edges' angles is checked | **generated**, after the check |
| Edge position rows (sigma 1e-4 m, scale beta = 0.1): the integrated guess restated; with layer 1 its shape | no data error corresponds to it | **stays approximate**; its effect is a measured quantity (C34) |
| Zero velocity at rest (sigma 0.01 m/s at both ends) | true at rest. A noise-free constant-velocity window also passes the rest test, with `imu_always_on` even when `use_zero_velocity_detection` is off (`run_odometry.cpp:1040-1047`, `imu_kinematics.cpp:86-108`) | **generated** at rest; uniform motion is a characterised case (A2), and the constant-velocity reference lives in a window fixture with rest set false |
| Origin fix (1e12 on node 0, a curvature on the current step) | the boundary pose is exact in the fixtures that say so | **conditioned on** |
| First window: gravity vector from its mean reading, start-up gyro bias, initial priors | drawn noise makes these estimates | **conditioned on**, or their noise accounted for; a comparison that depends on them is reported as conditional |
| NDT term: a point is an independent Gaussian draw from an exact cell, in two grids, Huber 3, scale 0.1 | cannot be made true on a plane (in-plane part), nor for a map built from the run's own windows | **stays approximate**; measured by K8 and in D8 |
| Sensors on one clock, one position, one set of axes | so | **generated** |

### D5. Fixtures on the production code

Each fixture names the quantity it isolates and asserts its own validity separately from its result.

- **F-a, a term at the truth (noise-free).** For each production term (accelerometer terms with start and end,
  rotation edges, position rows, the priors), on windows cut by the real front end: the residual at the true state is
  zero within a bound derived for that term and motion (zero for constant acceleration; the stated discretisation
  term under rotation). A time, frame or unit convention that two parts of the code read differently shows here,
  per term.
- **F-b, a term's noise (Monte Carlo at term level).** Whitened residuals at the truth have unit variance and no
  cross-covariance between terms, in the sample-aligned fixture; for the Livox timing the measured variance and
  cross-covariance are the size of the approximation.
- **F-c, a window's claimed covariance.** The window problem with a prior whose error is drawn from its claimed
  covariance, model-true inertial data, and the LiDAR's information controlled: none (the existing no-point set-up),
  or a corridor with exact cell statistics in which the test verifies that the projected translation information
  along the true axis is near zero and the transverse information is kept. The scatter of the handed-out end state
  against `take_covariance`'s blocks.
- **F-d, the hand-off from a known boundary state.** The prior mean and covariance the next window is actually
  prepared with, after growth and fall-backs; the re-propagated tail; the next window's first pose.
- **F-e, short chains through the real loop steps.** `process_windows` from a carried state and a map of true points
  that the test sets, as a replay continues from a capture: the blind stretch (growth of error against growth of
  claimed sigma, both switches off and on), and the exit with an entry error of known size.

**Error vectors** (they decide the degrees of freedom, so they are fixed here):
- The truth is aligned once, at the first retained node; world positions, velocities, gravity plus offset and the
  corridor direction are transformed consistently.
- *Velocity:* handed-on velocity minus the true velocity at the last node's time.
- *Gravity and offset:* in the raw accelerometer units and the estimator's frame. The carried vector starts as the
  first window's mean rotated reading and gains (offset estimate) / s per window, with s = 9.80665 / |g| recomputed
  from the vector before the fold. The error is s_before (g_after - g_true,raw), with the truth (gravity plus the
  true offset) expressed in the same raw units and frame, against the offset covariance (SI). Multiplying the vector
  by its own recomputed factor would normalise the magnitude error away.
- *Gyro bias:* in sensor axes, the generated bias minus the start-up estimate actually subtracted.
- *Joint:* estimate minus truth for both parts, with the cross block.
- Controls for the definitions: a sensor scale other than 1 g, a rotated start, and a case whose cross covariance
  has a known sign.

### D6. Statistics

- **Law.** At one preselected instant, over M independent runs in which every modelled random quantity is redrawn
  (the prior's error included), the sum of the normalised errors is chi-square with M r degrees of freedom if the
  claim is honest and the errors Gaussian. For a fixed truth axis u the error is projected first (A = u^T, r = 1; for
  velocity and offset together r = 2) and so is the covariance (A P A^T).
- **Direction.** A claimed sigma that is too small (a ceiling against a growing error) drives the value **up**.
- **Size.** The on-demand sets are sized to detect a variance ratio of 1.5 (a sigma 22 % off) with 90 % power at the
  1 % level: about 200 degrees of freedom. With 6 runs the scalar interval per degree of freedom is [0.11, 3.09]
  (with 40 runs [0.52, 1.67]); 6 runs are a smoke check that things run, not an honesty test.
- **Rules.** The instant, the axis, the primary comparisons and the decision rule over them (a family-wise level) are
  written down before the run. Successive windows of one run are not pooled as independent draws, and no window is
  chosen by its measured error. Missing windows and the coverage of each class are reported.
- **No prospective markers.** The effect of the switches, the exit windows and the gyro bias are measured; an
  "expected to fail" marker is set only by the rule of D7.

### D7. Cases, and "measure, then pin"

A discrepancy becomes a case marked as expected to fail only when it was **reproduced and isolated in a fixture**.
The marker sits on a case that asserts the one honest outcome; the fixture's validity assertions are in passing
cases beside it. Until then the case reports its measurement. The audit's numbers are hypotheses to compare with.

| Case | Finding | Set-up (fixture) | What is measured / the honest outcome | Audit's hypothesis |
|---|---|---|---|---|
| A1 | - | rest (F-a, F-e) | states that are active after initialisation stay at the truth | passes |
| A2 | rest test | noise-free constant velocity through the loop steps | whether the window is taken as at rest and what the zero-velocity term then does; the constant-velocity reference itself in the window fixture with rest false | taken as rest |
| A3 | F2a, F2b | constant acceleration (F-a, F-d) | term residuals zero; see K3 and K4 for the boundary | |
| A4a / A4b | - | turning in place / a turn with centripetal acceleration (F-a) | term residuals within the derived discretisation bound | unknown |
| A5 | - | walking (F-a, then F-e) | the terms' quadrature error and the map error first; only what remains is attributed | unknown |
| K1 | F1 | F-c in a seen room with the **map** displaced along travel by a known amount, so that the window's inertial data and relative motions stay exactly consistent; map, convergence, duration, prior and cross block specified; paired with the zero-shift control | the signed response of start and end velocity per mm, and the claimed covariance beside it; honest: the response stays within the claimed uncertainty, or the claim grows with it | [acc] 1.5-4 per T, [lidar] 1-1.5 per T, covariance unchanged |
| K2 | F1 | F-e: the exit of a blind stretch with an entry error of known, injected size | the velocity after the exit against its claimed sigma, window by window | a jump near error / T, then ringing (from one excitation) |
| K3 | F2a | F-d: a known exact preceding end state, a 2 ms gap, 2 m/s2 | the prepared prior mean against the true start velocity | 4 mm/s short |
| K4 | F2b | the tail re-propagation alone on p = a t^2 / 2 at 500 Hz, tail 15, span 10 | the last node, and the next first node, against the truth | -0.75 mm at 1 m/s2; -1.66 mm at the next node at 2 m/s2 |
| - | F2 | F-e under constant acceleration, tail 15 and tail 0 | the start-velocity move (tail 0 also changes the map) | +17-19 mm/s is a fitted hypothesis |
| K5 | F6 | the exact control (D2) | the normalised move averages 1 | must pass |
| K6 | F3 | level true poses, a constant tilted filter pitch, the true force, threshold 0.3 m/s2 | recognised as rest | not recognised from about 1.75 deg |
| K7 | C16 | the isolated weak-axis block, s = 0.5 | the block's own step at most g / k | 2 g / k |
| K8 | F4 | a scalar rigid shift with the other modes held (or the marginal variance of the shift actually solved); independent scan draws against a fixed true map, then with an earlier noisy map redrawn per trial; fine grid alone and both grids; a flat wall along travel | scatter / claimed sigma, with grid duplication, map noise and the plane's pull separated | 1.4 and 1.8 under independent 1 cm noise; the in-plane response was measured non-zero |
| K9 | cell test | a vertical plane with stated tangent variances, its normal turned 45 deg in XY, the inverse refreshed | usable in both orientations or in neither | usable only when turned |
| K10 | C37 | the origin fix's rotation weight | 1 / sigma^2 with sigma in radians | 1e12 where 3.3e15 was meant; harmless |
| K11 | F5 | F-e with a perfect gyro and the boundary attitude turned by 0.05 deg about a horizontal axis, paired with the unturned control | the bias estimate's response against its claimed sigma (the boundary-tilt mechanism, as distinct from a general bias statistic) | leaves zero by 3-5 claimed sigmas |

### D8. The loop on synthetic recordings: an integration measurement

The real front end and `optimize_windows` on model-world recordings. Nothing here attributes; it shows what the whole
scheme does to an honest world, with the same definitions as on a real run.

- **Routes:** R1, the loop with pillars (seen everywhere); R2, out and back through a plain corridor (a first pass
  through unmapped space has nothing to be corrected against; the way back gives the re-observation, and it is what
  a dead-end cave passage is); R3, rest, walk, rest. The emitted range is limited so that R2's blind middle fits a
  short route (with the shipped 40 m range it would need more than 80 m of corridor).
- **Settings:** the stage 8 candidate set with exact cell statistics. The shipped Livox file uses the previous
  statistics, for which a plain corridor's information was 925 along against 155 across
  (`scan_to_map_test.cpp:522-525`): there even the weakest direction can point the wrong way, so a run with the
  shipped file is a second, separately reported measurement.
- **Preconditions, verified and reported per run:** in the blind windows, enough matched side and floor normals, a
  projected translation information along the true corridor axis near zero, and the transverse information kept; at
  the nominated exit, full support along the axis and matches to structure that was mapped before and retained; no
  map rebuild; the time spent in each regime. A run whose precondition fails is reported as such and not used.
- **Reported:** per class of window (rest, seen, blind by position, exit by position, turn) the errors, the
  normalised errors of D5 and the readings of D10; with both layer switches off and on from the same seeds.
- **In every `ctest`:** a short smoke run: it completes, its preconditions hold, values are finite and repeat.

### D9. The record, completed after the hand-off

Today the per-window record is built and written in the report step, before the offset is folded into gravity,
before the tail is re-propagated and before the cancellation check (`run_odometry.cpp:1172-1189`, then `:1201-1235`).

- The table and all existing columns stay as they are, taken at the same moment as today.
- Appended after the hand-off: the carry actually handed on (velocity, gravity vector with the unit factor before
  the fold, offset covariance, cross covariance, gyro bias, validity of each); the last node's pose and time as
  solved and as handed on; the prior means and covariances this window was actually prepared with (after growth and
  fall-backs); the next window's prepared first pose where there is one; the window's mean rotation rate and its
  world-frame acceleration at its start (their definitions stated with the columns).
- Written once per window, at the end of the hand-off; a window whose hand-off is cancelled writes its record with
  the handed-on part marked absent.
- A record exists only with `log_window_terms`; the fixtures switch it on with their keeping sink.
- The window problem's diagnostic snapshot for D10 (start-velocity blocks, their validity) is taken out of the local
  problem before it goes out of scope.

Nothing feeds back into the odometry; `Window`, the capture file and the settings hash do not change.

### D10. Honesty readings on real runs

For a carried state with prior N(m, P), the window's result x and posterior covariance P_post, the move d = x - m
has covariance P - P_post in a linear-Gaussian update with a fixed observation model, so
E[d^T P^-1 d] = n - tr(P^-1 P_post), and

    q = d^T P^-1 d / (n - tr(P^-1 P_post))      has expected value 1.

Its distribution is a weighted sum of squares, not chi-square, and it is descriptive: P_post is conditional on the
fixed first node, the bias can be clamped, the weights are re-weighted (Huber) and the problem is linearised.

- **States:** the start velocity (n = 3), always; the gyro bias (n = 3); and, when the joint prior is used, start
  velocity and offset together (n = 6, d = [v_start - m_v, e - m_e], the full joint start prior and posterior with
  both cross blocks). The marginal velocity reading stays in joint mode too, so runs with and without the switch
  compare.
- **Computation:** whitened through a scale-aware Cholesky solve of P; the eigenvalues of the whitened posterior must
  lie in [0, 1] to numerical tolerance.
- **`NOT MEASURED`:** the term absent; the covariance absent, stale (not from the accepted final state), non-finite
  or ill-conditioned; a covariance "reduction" that is not physical; a clamped bias; a trace reduction
  n - tr(P^-1 P_post) below 0.5. The validity follows a step that is taken back.
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
state, with windows that stopped at the time limit flagged. Comparing the tool's new figures with the audit's is
done figure by figure with the differences in definition named; the regressions (lever arm, time offset) need
time-resolved data and are a follow-up.

### D12. Who builds what, and the order of the gates

| Part | Region | Model / effort |
|---|---|---|
| Exact control, generator, fixtures, loop measurement, the tool | verification (never edits `src/`) | Opus / medium |
| Handed-on state in the record and the CSV | noise-budget | Opus / medium |
| Covariance blocks kept, readings in the verdict | solver | Opus / medium |
| `owners.toml`, `tests/CMakeLists.txt`, the shared loop steps (report, finish), documentation from the hand-backs, merges, references, replays, summaries | orchestrator | Opus / medium; reading the results Fable / high |

- Ownership entries land in the same commit as the new tracked files (the ownership check reads tracked files).
- The step checks compare whole CSV files. At the commit that adds columns: the old columns' raw tokens are compared
  with the reference and every other reference is checked unchanged; only then is the changed table reference
  refreshed, and from there on "ALL HOLD" is required.
- "Results unchanged" means: the existing normalised manifest comparison and the exact deterministic outputs
  (replays with forced iteration counts, the term table); a comparison that depends on the time limit is flagged.
- Fixture checks that depend on the record run after the record is integrated.

## Risks / Trade-offs

- [The synthetic LiDAR is kinder than a real one] -> the LiDAR term's conclusions rest on K8 (with its assumptions
  stated) and on the readings of real replays; nothing is called a lower bound.
- [A generator error is read as an estimator error] -> the generator's own tests and F-a: a term's residual at the
  truth is checked before any honesty test uses that term.
- [The model world's settings differ from the shipped ones] -> D4 says per claim what is generated, conditioned on
  or approximate; the loop is measured with the candidate set and, separately, the shipped file.
- [Evidence is thin: one recording, one blind stretch, candidate settings] -> the design asserts no outcome from it;
  the audit's numbers are hypotheses (D7), and a result is worded "did not reproduce at this size, route, seed and
  settings", never "ruled out".
- [Markers hide a second defect] -> one assertion per marked case; validity in separate passing cases.
- [Monte Carlo time] -> the large sets are at term and window level, where a trial is one or two windows; the loop
  runs are few.
- [The readings invite a verdict] -> no limits; the summary states what they do not say.
- [The harness grows] -> the gates of D12 and the measure-then-pin rule; the follow-ups are named and left out.

## Migration Plan

No migration: defaults and results are unchanged. Order: the exact control; the record and readings (table reference
refreshed once); the generator with its tests; the fixtures, measured then pinned; the loop measurement; the tool
and the replays; the summary for the user. Rolling back is removing tests, columns and the tool.

## Follow-ups (named, not in this change)

- One real-world deviation at a time in the generator (lever arm, LiDAR-IMU time offset, a rotation between the
  axes, a body-fixed accelerometer bias, a gyro scale error, quantised gyro values, points stamped where they were
  taken, vibration, a range offset per surface), each compared from paired seeds with the same baseline settings.
- The Mid-360S lever arm: the mount's axis map from the recorder's source, and a run with it set.
- The regressions of the audit (hand-off, lever arm, time offset) with raw IMU data and trajectories.
- Sensitivity to the window length (two real cuts of the same recording).
- A reference marginal of the boundary state over the production terms of two windows, to size what the hand-off
  loses.
- What happens to the layers of `honest-inertial-carry`.

## Open Questions

- Whether the on-demand sets become part of `tools/end_runs.py`: after their first timings.
- The final names of the new columns and summary lines: with the implementation, in the knowledge files.
- Whether the loop steps accept a window without points (for a LiDAR-free chain through `process_windows`) or
  that chain stays at window level, where it exists today.

## The review of 2026-11-20 and what changed

`review-codex.md` (an independent reviewer, read-only). Its claims about the code were checked by the orchestrator
before this revision: the point assignment, the rest test and the zero-velocity term under `imu_always_on`, the
moment the record is written, the cost of a plane in its cell, the previous cell statistics in a corridor, and the
shipped Livox thresholds all read as the review says.

| Review item | Resolution |
|---|---|
| Blocking 1: attribution on the loop is not valid | D1: four objects with fixed rights to conclude; the loop is a measurement; A4 split, A5 measures quadrature and map error first; tolerances from derived bounds |
| Blocking 2: the proposed timestamps create a systematic error | D3: the three time conventions generated as the code uses them; points stamped strictly inside their node's interval; F-a checks each term at the truth |
| Blocking 3: one density does not make term noise independent | D4 row 1 (0.01057, exact only sample-aligned, otherwise measured); F-b |
| Blocking 4: other information sources, false rest | D4 rows for position rows, zero velocity, origin fix, first window, NDT; A2 is a characterisation; A1 only for active states |
| Blocking 5: the error vector | D5 "Error vectors", with the three controls |
| Blocking 6: direction, dimension, power | D6; no prospective markers; controls in D2 |
| Blocking 7: blindness and re-observation must be verified | D8 preconditions, candidate statistics, explicit range, rebuilds recorded |
| Blocking 8: a different cut is a different likelihood | D2 (same factors only); the two-cut comparison is a follow-up |
| Blocking 9: the record precedes the hand-off | D9 |
| Reading (should change) | D10 as proposed by the review |
| Cases (should change) | D7 table; K10 for C37; K11 for F5's boundary-tilt mechanism |
| Tool and historical tables (should change) | D11: new columns, one definition, "unavailable", replays with the new build |
| Order of gates (should change) | D12 |
| Scope and wording (should change) | proposal and Follow-ups; specs reduced to behaviours |
| Notes on the pipeline's inputs | D3 "What the front end needs"; the memory point store's path |

Not taken over as written: the review's suggestion of a "production-term fixture" is implemented as five fixtures
(F-a to F-e) because term, window and hand-off isolate different quantities.
