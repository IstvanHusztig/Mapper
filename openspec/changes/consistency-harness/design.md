# Design: consistency-harness

## Context

See `proposal.md` for the motivation and `research.md` for the evidence (findings F1-F8, their limits, the lineage
table). This file says how the question "is what the estimator claims as good as it says?" is turned into tests and
readings.

What exists and is reused:
- `OdometryPipeline` takes its recording through the `RecordingSource` port and its services through `RunServices`, and
  hands each window's record to a `WindowTermSink` (`set_window_terms`). A synthetic recording can therefore go through
  the real `load` (reading, orientation filter, window cutting) and the real window loop without touching them.
- `tests/synthetic_recording.{h,cpp}`: scenes of plane faces, their sampling, `loop_recording` (no IMU data, truth as
  the guess). `tests/scan_to_map_test.cpp`: one window against a map built from true points, including a corridor and
  the blind-chain cases of `honest-inertial-carry`. `tests/memory_point_store.h`, `tests/serial_executor.h`.
- The convention of this repository for a known shortcoming: a doctest case marked `should_fail` with the cause in a
  comment; when it starts to pass, doctest reports it and the marker goes.

Constraints: no result changes (step checks identical, 44_short bit-identical); tests need no recorded data; the
verification region never edits `src/`; nothing goes into a settings file; the agreed limits of the velocity chi2
(contract of 2026-11-12) are not touched.

**Words.** The Consistency pass (`use_consistency`) already owns the word "consistency" here, so the tests are called
*truth tests* and the statistics as follows:
- *honest:* a claimed covariance matches the scatter of the actual error.
- *model world:* a synthetic recording in which every sensor error is drawn exactly as the estimator's weights assume.
- *normalised error* of a carried state x with claimed covariance P: (x_est - x_true)^T P^-1 (x_est - x_true); its
  expected value under an honest claim is the state's dimension n (NEES in the literature).
- *normalised move:* the same for the move from a prior to the window's result, against the covariance the move has
  under the model, scaled to an expected value of 1 (D9).
- *seen / blind / exit window:* the LiDAR holds the direction of travel / it does not (information ratio below the
  projection's lower threshold) / the first windows after a blind stretch in which it does again.

## Goals / Non-Goals

**Goals:**
- A method that sorts every failure into one of two bins by construction: the code or the scheme fails on data that
  obeys its own model (tiers A and B), or the model is too simple for reality (tier C), with a size for each.
- Every finding of `research.md` that was checked in code is pinned by a test that a fix must turn green.
- The predictions of the audits ("expected today") are tested, not trusted.
- The same statistics on a real run and on a synthetic run, so that tier C's worlds can be compared with the
  recordings like with like.

**Non-Goals:**
- Fixing any finding; changing a limit, a setting file or a baseline.
- Truth tests of loop closure and the consistency pass.
- A realistic LiDAR simulator (occlusion, beam pattern, incidence effects): the synthetic LiDAR stays simple, and its
  limits are stated (Risks).
- A verdict on the map: none of this replaces the user's look at a cloud.

## Decisions

### D1. Three tiers, one question each

| Tier | World | Question | Kind | A failure means |
|---|---|---|---|---|
| A | noise-free model world | truth in, truth out: are the parts of the code consistent with each other? | deterministic, one run per case | two parts of the code disagree about the same quantity (time, frame, state) |
| B | model world with noise | is each claimed covariance as large as the actual error? | Monte Carlo over seeds | the scheme double counts, drops or misweights something under its own model |
| C | model world plus one deviation | how large is each real-world effect, and does it look like what the recordings show? | experiment, report | a limit of the model or a missing calibration, with its size |

Tier A comes first because it needs no statistics: in a noise-free model world the truth makes every residual zero, so
the truth is the solution, and any estimate off the truth, or any carried state moved away from its prior, is an
inconsistency. `research.md`'s F2 predicts exactly such a deviation (a start-velocity move of 17-19 mm/s under a
constant 2 m/s2). Tier B needs the model world's noise and is where "information counted twice" and "a covariance
dropped" show. Tier C is the route to the unexplained 75-80 % of symptom 1.

*Alternative considered:* one large Monte Carlo over realistic data with a single pass/fail number. Rejected: it fails
today for many reasons at once and attributes nothing.

### D2. The synthetic world is a recording

The generator produces what a recorder produces: IMU samples (timestamp, gyro, accelerometer) and LiDAR points with
timestamps, plus the truth (pose, velocity, gyro bias, acceleration offset at any instant). It is served by an
in-memory `RecordingSource`, so the tests run `OdometryPipeline::load` and `run_odometry` unchanged: the start-up gyro
bias, the orientation filter with its ramp, the window cutting, the map seed from the first points, the rest test and
the hand-off are all the real code. A window-level fixture built by hand would not exercise the front end, and F3 and
the start-up findings of [gyro] live there.

- **Motion:** an analytic trajectory, twice differentiable, from segments joined smoothly: rest (the recording starts
  with at least 2 s of it, as the pipeline expects), speeding up, walking, a turn, slowing down. "Walking" has the
  measured dynamics of Livox 3: acceleration about 0.5 / 0.5 / 0.9 m/s2 rms per body axis and rotation rates about
  11 / 11 / 32 deg/s rms, at a 2 Hz step rate, so that effects which scale with acceleration and rotation rate appear
  at their real size.
- **Scenes and routes** (from `box_scene` / `loop_scene`): (R1) the existing loop with pillars, seen everywhere;
  (R2) *out and back through a plain corridor*: a room with structure, a corridor longer than twice the points' range
  so that its middle is blind along travel, a small end room, a turn, and the way back. A first pass through unmapped
  space has nothing to be corrected against; the exit correction of F1 needs structure that was mapped before, which
  the way back gives. It is also what a dead-end cave passage is. (R3) rest, walk, rest.
- **Sensor profile:** the Livox one by default (500 Hz IMU, the settings of `settings/livox-vertical.toml`), because
  several settings count samples (`handoff_tail_nodes`, the velocity spans). Fewer points per node than the real
  sensor, to keep the runs short. An L2 profile is a parameter set, not new code.
- **Named deviations, one switch each** (tier C): lever arm of the accelerometer; time offset between LiDAR and IMU
  stamps; a rotation between the IMU's and the LiDAR's axes; a body-fixed accelerometer bias; a gyro scale error;
  gyro values quantised at 0.061 deg/s; LiDAR points stamped where they were taken, between nodes; a band-limited
  accelerometer error (vibration); a range offset per surface; IMU samples taken at an instant, not as the mean of
  their interval.
- **The generator is tested itself:** an independent strap-down integration of its noise-free IMU samples reproduces
  its true trajectory; the sample statistics of its noise match its parameters; its points lie on the scene's faces.
  Without this a generator error would be read as an estimator error.

*Alternative considered:* extend `loop_recording` with per-node `imu_acc` / `imu_angles` (windows built by the test).
Kept only for single-window cases (D7); as the main route it would skip the front end and need its own copy of what
`compute_step_1` produces.

### D3. The model world: claims made true, and what cannot be

In the model world the data is generated from the same numbers the weights are built from, so that "honest" is
decidable. The run uses the shipped settings with these substitutions, each written next to its reason in the test:

| Claim of the estimator | Model world |
|---|---|
| Accelerometer error per term: (density^2 / tau + floor^2), independent between terms | white noise only, at the density that gives the shipped per-term sigma at the typical term length (0.1 m/s2 at 11 ms -> about 0.0105 m/s2/sqrt(Hz)); settings: that density, floor 0. The claimed information is the shipped one, and the data obeys it |
| Acceleration offset: a random walk in the world frame at `imu_acceleration_offset_rw_mps2_sqrt_s` | exactly that (the world frame is the model's choice, C17; a body-fixed bias is a deviation of tier C) |
| Rotation edges: max(k x ARW x sqrt(dt), floor) per edge, independent | gyro white noise at the ARW, k = 1, floor 0, `ahrs_gain = 0` so the filter is a pure integration after its ramp |
| Gyro bias: a random walk at `gyro_bias_rw_deg_s_sqrt_s` | exactly that |
| Samples held over their interval | each sample is the mean of the true signal over its interval |
| Points placed with the node before them | points stamped at node times |
| Accelerometer at the pose origin; sensors on one clock and one set of axes | so |

Two claims cannot be made true by construction, and the harness measures them instead of assuming them:
- **The NDT term on a plane.** It treats a point as a draw from its cell's Gaussian in all three directions; on a flat
  surface larger than the cell the in-plane part claims a hold it does not have (F4).
- **The map as exact.** The loop builds the map from its own earlier windows.

So a state that the LiDAR dominates cannot be expected to be honest even in the model world, and tier B is laid out by
what dominates a state (D6): the blind stretch isolates the inertial terms and the carry, where the model world is
exactly true; a single-window Monte Carlo against a map of true points measures the NDT term's own claim (D7, K8);
the seen windows of a run measure the total.

### D4. What the loop reports about a window

The truth tests need, per window, the state that is handed to the next window, with its covariances. Today the
velocity and the gyro bias are in `Window`, but the gravity vector, the offset's covariance and the pose the next
window starts from are inside the loop.

- `WindowTermRecord` gains the **handed-on state**: end velocity and covariance; the carried gravity vector and its
  unit factor; the offset covariance; the velocity-offset cross covariance (zero unless carried); gyro bias and
  covariance; the last node's pose as solved and as handed on (after the tail is re-propagated), and its time. Full
  matrices in memory; means and sigmas as columns of `window_features.csv`.
- The window problem keeps two more blocks of the covariance columns it already receives (start velocity, and start
  velocity with offset) for the reading of D9. They are not stored in `Window`.
- Also as features, because the signature table of D8 needs them and the audits had to rebuild them from raw files:
  the window's mean rotation rate and its world-frame acceleration over the first 30 ms.

Nothing here feeds back into the odometry; `Window`, the capture file and the settings hash do not change. The
record is produced where a sink is set, as today.

*Alternatives considered:* (a) stepping `process_windows` from the test and reading `WindowLoop::carry`: the map seed
and the run's first steps are file-local, and the front end would be bypassed. (b) a field of `WindowOutcome`: that
type is what a replay compares; a diagnostic record is the right home, and real runs get the columns for free.

### D5. Tier A: truth in, truth out

Noise-free model world, fixed seeds for the LiDAR's small jitter (plane cells need a thickness). One run per case:

| Case | Motion (route R1 or R3) | Expected of today's code |
|---|---|---|
| A1 | rest | passes; sets the floor |
| A2 | constant velocity | passes; sets the floor |
| A3 | constant acceleration, 2 m/s2 | fails: F2a and F2b (predicted: first node of each window 1.66 mm behind, prior mean 4 mm/s low, start-velocity move 17-19 mm/s) |
| A4 | constant yaw rate at constant speed | to be seen |
| A5 | walking | fails by F2; the rest is unknown and is the point |
| A6 | A3 and A5 with `handoff_tail_nodes = 0` | separates F2b from F2a |

Checked per window: the handed-on velocity, gravity vector and gyro bias against the truth; the handed-on pose
against the truth; and each carried state's move from its prior. Tolerances come from the floor that A1 and A2
measure (the LiDAR jitter's effect and the discretisation), times a stated factor, and are written in the test.
The truth is brought into the estimate's frame by aligning the first node.

**The hand-off against one long window.** On a noise-free recording, two consecutive windows solved in sequence must
give the same end state and nearly the same end covariance as the same nodes solved as one window (offset and bias
random walks set to zero for this case, so that both forms have the same model). The difference is what the hand-off
loses or distorts, with no statistics needed; under acceleration it shows F2, and the covariances show what
conditioning on the first pose costs. It runs through the real loop (two recordings cut with different window
lengths), not through a re-implemented hand-off.

Each failing case is entered as "expected to fail" with the finding it shows. For A3 the predicted sizes are also
asserted in a second, passing case (a characterisation): if today's code does not show what the audit predicted, the
mechanism is not understood, and that is a result. A fix removes both.

### D6. Tier B: claimed uncertainty against actual error

Model world with noise, route R2 (out and back through the corridor) plus R1, M runs with different seeds, both
layer switches on unless said otherwise.

- **Statistic.** For window k of the route (the same instant in every run) and state x (handed-on velocity, n = 3;
  gravity vector against true gravity plus the true offset, n = 3; both jointly with the cross block, n = 6; gyro
  bias, n = 3): the sum over the M runs of the normalised error is chi-square with M n degrees of freedom if the
  claim is honest. The band is the two-sided 99 % interval.
- **By class.** Each window of the route has a class fixed by the route: rest, seen, blind (by position in the
  stretch), exit (first, second, third window after a stretch), turn. Reported per class: the mean normalised error
  per degree of freedom (1 if honest) and the share of windows inside their band.
- **What is asserted.**
  - *Blind windows, the component along the corridor, of velocity, offset and both jointly:* inside the band. This
    is where the model world is exactly true (D3), so a failure here is an error of the inertial terms or the carry
    under their own model. Expected to pass with both layers on; if it does not, that is a new finding.
  - *The same with both switches off:* outside the band, far below honest (the 11 mm/s ceiling against a growing
    error). This is **positive control 1**: the harness reproduces the defect that started `honest-inertial-carry`.
  - *Exit windows:* expected to fail (F1), entered as such, with the mean and the size of the velocity error
    recorded.
  - *Seen windows, and the gyro bias everywhere:* expected to fail (F4, F5, F2), entered as such with their factors.
- **Positive control 2:** the same recordings with the accelerometer noise generated three times stronger than
  claimed must leave the band on the high side in the blind windows, and three times weaker on the low side.
  A test that cannot fail proves nothing.
- **The statistic itself** is tested on a linear chain with a textbook Kalman filter: honest by construction, inside
  the band; with a doubled measurement weight, outside.
- **Size and time.** Full set on demand: M = 40 (band for n = 3: mean within about +-0.3 of 1 per degree of freedom).
  In every `ctest` run: M = 6 on a shortened R2, asserting only positive control 1 and the blind-window check with
  its wider band. Budget for all truth cases in `ctest`: one minute in Release; in Debug the loop-level cases are
  skipped, as the loop-closure cases are. Seeds are fixed, so a result is reproducible.

### D7. Known-answer cases for the audit's findings

One small case per finding that was checked in code. Each states the honest outcome and is marked expected to fail
with the finding's id while the defect exists; K5 tests new code and must pass. Single-window cases use the fixtures
of `scan_to_map_test.cpp` (a window against a map of true points); the others the generator.

| Case | Finding | Set-up | Honest outcome | Predicted today |
|---|---|---|---|---|
| K1 | F1 | a seen room, true motion 1 m/s, velocity prior at the truth (sigma 5 mm/s); the first node alone displaced 10 mm along travel | the end velocity's normalised error below the 99 % bound (11.3) | both velocities change by 28-42 mm/s, the covariance unchanged: 6 or more sigmas |
| K2 | F1 | tier B's route R2, the exit windows | as K1, over runs | a velocity jump near (position error) / T, the opposite sign in the next window, sigma below 10 mm/s |
| K3 | F2a | two windows at constant 2 m/s2 | the second window's prior mean equals the true start velocity | 4 mm/s low |
| K4 | F2b | `repropagate_window_tail` on p = a t^2 / 2, a = 1 m/s2, tail 15, span 10 | the last node on the truth | 0.75 mm behind |
| K5 | F6 | scalar: prior N(0, P), one measurement with variance P, many draws | the normalised move of D9 averages 1 | (the old statistic averages 0.5: asserted as a fact about it) |
| K6 | F3 | a window at perfect rest whose filter angles are tilted by 2 deg | recognised as rest | not recognised; at 1.5 deg both agree |
| K7 | C16 | one node, block diag(k), gradient g along the weak axis, s = 0.5 | the projected block's own step at most g / k | 2 g / k |
| K8 | F4 | Monte Carlo of one window against a map of true points and against a map made of one earlier noisy window, fine grid alone and both grids | scatter of the solved shift / claimed sigma = 1 | about 1.4 and about 1.8; a flat wall's in-plane part claims a hold and pulls nothing |
| K9 | cell test | a planar cell with a normal sigma of 3 mm, axis-aligned and turned 45 deg | both usable or both not | usable only when turned |

The cases of the audits that are about reality, not about the code under its own model (a body-fixed bias after a
turn, a lever arm, an axis rotation, a carried bias error in a blind stretch, the start-up bias median), are tier C
experiments (D8), not pass/fail cases.

### D8. Tier C: one deviation at a time, against the recordings

For the model world and for each deviation of D2 at the size the audits fitted (lever arm 44 mm, time offset 6 ms,
axis rotation 0.2 deg, body-fixed bias 0.05 m/s2, gyro scale 0.4 %, ...), one run of R1 with walking, and the
**signature table** that `tools/honesty_report.py` also makes from a real run:

- the start-velocity move: median of the norm, median per axis, by quarter of the rotation rate, by quarter of the
  acceleration at the window start, lag-1 autocorrelation, the value at rest;
- the normalised moves of D9 (median, 95th percentile);
- the regressions of the audits: move against boundary acceleration (ms), the lever-arm fit, the time-offset fit.

The real rows (Livox 3: 23-24 mm/s, growing 16.8 -> 28.6 with rotation rate and 19.8 -> 29.0 with boundary
acceleration, autocorrelation -0.12 to -0.35, isotropic, 3.6 mm/s at rest) are the target. A deviation whose row has
that shape is a candidate for the unexplained part; one that does not is ruled out at that size. Combinations are run
only for candidates. Run on demand, written to a folder outside the repository given by an environment variable; the
numbers go to the user without a verdict.

Also here, as readings with their predicted sizes: a carried gyro-bias error of 0.3 deg/s through a 4 s blind stretch
(0.41 m/s if the tilt is not held, 0.08-0.2 if the LiDAR holds it); a body-fixed bias through a 180 deg turn and a
blind stretch (F7); the start-up bias from quantised samples.

### D9. Honesty readings on real runs

For a carried state x with prior N(m, P) and the window's posterior covariance P_post, the move d = x_result - m has
covariance P - P_post under a correct linear-Gaussian model, so E[d^T P^-1 d] = n - tr(P^-1 P_post). The reading is

    q = d^T P^-1 d / (n - tr(P^-1 P_post)),   expected value 1.

- **States:** the start velocity (n = 3); with `imu_carry_joint_covariance` the start velocity and offset jointly
  (n = 6, the joint prior); the gyro bias (n = 3).
- **NOT MEASURED** where the term is absent, and where the denominator is below 0.5: the window then adds so little
  to the state that the ratio is noise.
- **Where:** computed in every run with the verdict's other readings; median and 95th percentile in the verdict
  summary; columns of `window_features.csv`. No limit, no flag, no trigger.
- **The existing velocity chi2 is not changed**, nor its limits 5 / 20: they are part of the contract agreed on
  2026-11-12. With both statistics side by side on real runs, the user can decide later whether to re-base it.
- **What the reading does not say:** P_post is conditional on the exact first node (F1) and the problem is robust and
  nonlinear, so 1 is a model expectation. A reading near 1 does not prove honesty; a reading far above 1 shows a
  claim that the window's own data contradicts.

### D10. The tool

`tools/honesty_report.py` (pure Python, with tests in `tests/test_verification_tools.py`): reads one or more
`window_features.csv`, prints the signature table of D8 per run and side by side, and writes it as CSV. The audits'
throw-away scripts (`experiments/consistency/scripts/`) are its starting point; the regressions state their
regressors and standard errors. It gives no verdict.

### D11. Evidence on recordings with existing settings

No code; each is a number for the user, none a verdict:
- the tool on the existing runs with tables (Livox 3: base, layer 1, layers 1+2, the gyro-noise run; Livox 4
  candidates; the 44_short table of the step checks);
- replays of the Livox 3 and Livox 4 captures with `handoff_tail_nodes = 0`: does the part of the move that follows
  the boundary acceleration go away (F2b), and what else moves;
- one full Livox 3 run with the lever arm set. The Mid-360S mount's axis map is read from the recorder's source
  (read only); the L2's vertical remap would give (-0.044, 0.023, -0.011) m, which agrees with the fitted x in sign
  and roughly in size.

Full runs are sequential (`real_time_threshold_seconds` makes parallel runs timing-dependent).

### D12. Who builds what

| Part | Region | Model / effort |
|---|---|---|
| Generator, in-memory source, truth statistics, tiers A-C, known-answer cases, the tool | verification (never edits `src/`) | Opus / medium |
| Handed-on state in the record and the CSV | noise-budget | Opus / medium |
| Covariance blocks kept, readings in the verdict | solver | Opus / medium |
| `owners.toml`, merges, step checks, references, runs, summaries | orchestrator | Opus / medium; reading the results Fable / high |

At most two editing agents at once: the record and readings (noise-budget, then solver) run beside the generator
(verification); the truth cases need both.

## Risks / Trade-offs

- [The synthetic LiDAR is kinder than a real one: no occlusion, perfect planes, even sampling] -> its results are
  lower bounds for F4; the conclusions about the LiDAR term rest on K8, on the readings of real runs and on tier C's
  comparison with the recordings, and are worded as such.
- [A generator error is read as an estimator error] -> the generator's own tests (D2); A1 and A2 as the floor; every
  tier A failure is cross-checked by switching one ingredient of the motion.
- [The model world's settings differ from the shipped ones (floor 0, k = 1)] -> same code paths, the same claimed
  information where it matters (D3); tier C and the real readings use the shipped settings.
- [Cases expected to fail can hide a second defect inside the first] -> each asserts one quantity; the main ones have
  a characterisation of today's size beside them (D5), so a change of size is reported.
- [Monte Carlo time] -> a small set in `ctest` within a one-minute budget, the full set on demand with its time
  reported.
- [The readings invite a verdict] -> they have no limits; the summary says what they do not say (D9).
- [`window_features.csv` gains columns] -> existing columns keep their names, order and bits; the step checks' table
  references are re-recorded in their own commit, after the old columns were shown identical.
- [The harness finds more than can be fixed soon] -> that is its job; the expected-to-fail cases are the list, and
  the user decides the order.

## Migration Plan

No migration: defaults and results are unchanged. Order of landing: the record and readings (results identical,
references re-recorded), the generator with its own tests, tier A and the known-answer cases, tier B, the tool,
tier C and the evidence runs, the summary for the user. Rolling back is removing tests, columns and the tool.

## Open Questions

- Whether the on-demand Monte Carlo and tier C become part of `tools/end_runs.py`: after their first timings.
- The final names of the new columns and summary lines: with the implementation, in the knowledge files.
- Whether the L2 profile gets its own tier B run in this change or with the first fix that touches the L2 settings.
