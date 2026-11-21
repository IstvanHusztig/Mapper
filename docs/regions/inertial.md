# Region: inertial

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

The inertial region makes a window's first guess from the previous window and the IMU, and writes the IMU's terms into
the window's equations: the motion edges between consecutive nodes (gyro and the guess's shape), the velocity
continuity, the accelerometer terms, and the priors of the states carried from window to window (gyro bias, velocity,
acceleration offset). After the window it hands those states on and re-propagates the window's tail. It owns the
residuals and Jacobians of these terms; their weights come from the noise model (region noise-budget).

## Contract

**Takes**
- From the front-end, each `Window` (`src/domain/window.h`): `poses` (the AHRS orientation per IMU sample, one node per
  sample), `timestamps`, `imu_acc` (raw accelerometer, body frame, in g), `imu_angles`, `imu_motion_from_previous` (the
  IMU's motion from the previous window's last node to this window's first).
- From the window loop (orchestrator, `run_odometry.cpp`): `OdometryRunSetup` (`gravity_norm`, `prediction` options,
  rotation options) and `WindowLoopCarry` (`bias`, `velocity_mean` / `velocity_cov` / `velocity_time_s`, `acc_gravity`,
  `acc_offset_cov`, `first_window_at_rest`).
- From the noise-budget, `WindowNoiseModel` (every weight, and the layout `unknowns` built by the solver's
  `layout_window_unknowns`). The terms read weights from it and never compute one.

**Gives**
- The guess: `load_and_predict` → `predict_window_poses` (chaining through `place_window_after`, the rest test
  `window_rests`, `guess_window_translations`); then in `prepare_window` `apply_gyro_bias_prior` and, with
  `imu_always_on`, `prepare_velocity_state` and `integrate_window_translations`. `window.motion_model_poses` is set to the
  guess; it is the measurement of the motion edges.
- The terms, called by the solver's `WindowProblem::linearize` in this fixed order after the NDT blocks:
  `add_motion_edge_terms`, `add_gyro_bias_prior`, `add_velocity_continuity`, `add_acceleration_terms`,
  `add_velocity_priors` (start-velocity prior, zero velocity, weak end tie), `add_acceleration_offset_prior`
  (`window_problem.cpp:41-48`). Each adds to a `WindowEquations` (matrix, rhs, `motion_cost`, `residuals`).
- After the window (`finish_window`, orchestrator): `update_gyro_bias_after_window`, `update_acc_term_after_window`, then
  `repropagate_window_tail`; then `place_window_after` places the next window after this one's re-propagated last node,
  and after that the map adds the window (map region; so the window's points enter the map with the re-propagated
  tail, see C32).

**Invariants others rely on**
- Units and frames: gyro bias in rad/s, body frame (`Window::gyro_bias`); logged in deg/s. Velocities in m/s, world frame
  (`Window::velocity.start/end`). The acceleration offset is in m/s², world frame (see C17); after the window it is
  folded into `carry.acc_gravity` (accelerometer units: `acc_gravity += acc_offset / to_mps2`, `to_mps2 = 9.80665 /
  |acc_gravity|`).
- The carried bias is removed once, from the guess (`apply_gyro_bias_prior` → `remove_gyro_bias_from_rotations`); the
  edges subtract only the update `(b − b_prior)·dt` (`motion_model_terms.cpp::edge_measurement`). The tail re-propagation
  gets the same update (`finish_window`).
- The bias and the accelerometer term start in the second window (`prepare_window`: `estimate_bias && i > 0`;
  `prepare_acc_term(..., i > 0, ...)`). The first window is registered against the seed map, built from points not yet motion-compensated.
- Each term can be called alone on an empty `WindowEquations` with const inputs: the per-term table
  (`window_term_table.cpp::term_rows`) relies on it. The edges add their cost only when `equations.with_cost` is set.
- Only the next window is placed after a window is done (change `chain-next-window-only`); on a run stopped by a non-finite
  window distance `place_windows_after` places the rest once (the other cancelled exits do not call it).
- The rest test is one function, `window_rests` (50 ms average, `rest_smoothing_half_width_s = 0.025`), used by the
  prediction, `window_at_rest` and `log_accelerometer_fit`.
- The weights' formulas this region's code calls are the noise-budget's: `time_scaled_rotation_sigma` (through the
  edges' weights), `random_walk_prior_cov` (`window_steps.cpp::apply_gyro_bias_prior`, `prepare_acc_term`) and
  `velocity_prior_cov_over_gap` (`prepare_velocity_state`). A change of one of them changes what this region carries.
- C34 (noise-budget's entry) has its other half here: `integrate_window_translations` in `prepare_window` sets the
  guess that `motion_model_poses` holds and the edges tie to.
- The carried end velocity is the solve's (`window.velocity.end` and its marginal covariance from the solver's
  `take_covariance`), read before the tail is re-propagated.

## Owns

- `src/domain/motion_model_terms.{h,cpp}`: the motion edges (`add_motion_edges`, `add_motion_edge_terms`; edge measurement
  with the frame correction and the bias update, the 12×12 block, the bias coupling), `add_gyro_bias_prior`,
  `add_velocity_continuity` (only without the accelerometer term, with its first-step prior).
- `src/domain/accelerometer_term.{h,cpp}`: `tait_bryan_rotation_jacobian` (numeric, h = 1e-5), the accelerometer terms
  over the noise model's spans (anchor triples, start velocity, end velocity), `add_start_velocity_prior`,
  `add_zero_velocity`, `add_weak_end_velocity_tie`, `add_velocity_priors`, `add_acceleration_offset_prior`.
- `src/domain/window_steps.{h,cpp}`: `place_window_after`, `place_windows_after`, `estimate_previous_velocity`,
  `window_rests`, `mean_shift_of`, `guess_window_translations`, `predict_window_poses`, `apply_gyro_bias_prior`,
  `is_turn_window`, `integrate_window_translations`, `repropagate_window_tail`, and `convergence_of` (a solver
  diagnostic that lives here).
- In `src/application/run_odometry.cpp`: `load_and_predict` (previous velocity, the guess), `prepare_acc_term` (offset
  prior grown by its random walk, the term's options), `window_at_rest`, `prepare_velocity_state` (start-velocity prior:
  carried end velocity grown over the gap, else the first-window rest or `imu_velocity_initial_sigma_mps`),
  `update_acc_term_after_window` (velocity and offset carried, `acceleration offset` log line),
  `update_gyro_bias_after_window` (`gyro bias` log line, clamp warning, bias carried).
  Also `log_accelerometer_fit` (behind `log_window_diagnostics`): the accelerometer term's fit to the result, this
  region's own diagnostic.
- Tests: `tests/window_steps_test.cpp`; the motion and accelerometer cases of `tests/window_terms_test.cpp` and the gyro
  bias and `imu always on` cases of `tests/scan_to_map_test.cpp` (shared files).

Not owned but close: the bias clamp itself (`window_solver.cpp`, solver), the velocity covariance (`take_covariance`,
solver), `is_window_stationary` / `world_acceleration` / `remove_gyro_bias_from_rotations` (`imu_kinematics`,
front-end), the weights (`window_noise_model`, noise-budget), the turn decision and its k (`prepare_window`,
orchestrator).

## How to check your work

- Unit tests (Release): `out/build/x64-Release/bin/lidar_odometry_tests.exe -tc="place_window*,place_windows*,estimate_previous*,predict_window_poses*,apply_gyro_bias*,is_turn_window*,repropagate*,integrate_window*,window_rests*,convergence_of*"`
  for `window_steps_test.cpp` (chaining keeps relative poses; previous velocity and its limit; the mean shift and its
  1 m drop; the chained guess equals `place_window_after`; the four translation branches; bias prior growth and removal;
  turn threshold; tail re-propagation; constant acceleration integrated exactly with uneven spacing; rest still vs
  walking). Terms: `-tc="window terms: the gyro-bias*,window terms: the velocity continuity*,window terms: the motion-model*,window terms: the acceleration*,window terms: the velocity priors*,window terms: each term counts*"`
  (right-hand side = minus half the cost's gradient, residual counts). Solves: `-tc="scan-to-map: a yaw drift*,scan-to-map: the gyro bias*,scan-to-map: a gyro bias carried*,scan-to-map? imu always on*"`
  (bias estimated and removed once; blind corridor follows the accelerometer; rest stays put; tilt goes into the
  attitude; an IMU gap drops the stretched terms).
- Replays: `python tools/run_replay.py <capture> <settings>` (captures in `tools/local_paths.toml`: 44_short 20–40 s,
  Livox 3 100–130 s). Then `python tools/step_checks.py` (about 7 min) before handing back.
- Readings that are this region's own:
  - per-term table (`log_window_terms`, `python tools/window_terms.py <csv> [--window N]`): the residual side (cost per
    residual, pull) of the rows `edge_position`, `edge_rotation`, `gyro_bias_prior`, `velocity_continuity`,
    `accelerometer`, `velocity_prior`, `zero_velocity`, `end_velocity_tie`, `offset_prior` (their hold/weight is the
    noise-budget's reading); `window_features.csv` columns velocity start/end, gyro bias, acceleration offset;
  - window verdict: heading/roll/pitch bend against the gyro, speed and "fast", bias and "bias high", velocity chi²;
  - log lines: `gyro bias [deg/s] window`, `WARNING: gyro bias clamped`, `acceleration offset [m/s^2] window`,
    `previous window velocity ... not used as prior`, `!!!mean_shift.norm() > 1.0!!!`; with `log_window_diagnostics`
    the `window diagnostics N: velocity start/end, end sigma` line, `accelerometer fit` (this region's own diagnostic); the run summary counts (zero-velocity, accelerometer term active, windows at rest, turn
    windows).
- Tools: `tools/eval_odometry.py` (window yaw against the gyro), `tools/trajectory_seams.py` (position stalls between two
  results). Skills: `verify-run` (bends, slides, bias estimate), `verify-recording` (gyro held offsets, rest bias; the
  user's L2 has a faulty gyro).

## Known flaws

- **C17** Acceleration offset modelled in the world frame; an accelerometer bias is body-frame. Today:
  `accelerometer_term.cpp::add_acceleration_term` (`mean_acc - acc_offset`) and the fold into `carry.acc_gravity` in
  `run_odometry.cpp::update_acc_term_after_window`. Reported; possible cause of a ~1° tilt. Its size by the audit of
  2026-11-20 (F7, `experiments/consistency/research.md`, Livox 3 and 4 runs; not measured by the harness): a 0.05 m/s²
  body-fixed misfit on both Livox recordings is 0.07-0.10 m/s² wrong after a 90-180° turn and 0.3-0.4 m/s after 4 s
  blind; the accelerometer innovations show about 2 % of it (the two audits sit uneasily together).
- **F2a** (verified: red unit test; marker K3): `prepare_velocity_state` grows the start-velocity prior's covariance over
  the gap between windows but takes its mean as the carried end velocity, not advanced by the acceleration. Measured
  (`tests/unit_cases_inertial_test.cpp`, 2 ms gap): the prior mean is -2.000 / -4.000 mm/s off the true start velocity
  at 1 / 2 m/s², 0 at rest (the covariance algebra F P F^T + Q is exact to 1e-16). Pinned by `unit case K3 marker: the
  start velocity prior's mean equals the true start velocity` (`doctest::should_fail`, oracle the true start velocity
  within 0.1 mm/s). The audit also found the last node's accelerometer sample used by no term (`accelerometer_term.cpp`),
  not measured on its own. Order: step 3 of the decision in `openspec/changes/consistency-harness/tasks.md` 8.2.
- **F2b** (verified: red unit test; marker K4): `repropagate_window_tail` replaces the tail by a constant-velocity line
  from a finite difference, which lags under acceleration; the next window's fixed first pose comes from it (C32 is the
  rotation half). Measured on p = a t²/2 at 500 Hz, tail 15 / span 10: last node -0.750 mm at 1 m/s², next first pose
  -1.664 mm at 2 m/s²; with the shipped L2 tail 30 / span 20 at 2 m/s²: -6.000 / -6.324 mm. Pinned by `unit case K4
  marker: the tail re-propagation's last node and the next first pose equal the truth` (oracle 0.05 mm). Order: step 4,
  on its own (the tail also moves the points that enter the map).
- **F3** (verified: red unit test; marker K6): the rest test removes gravity with the orientation filter's absolute tilt,
  which with `ahrs_gain = 0` is an open-loop gyro integration. Measured with level true poses and a constant tilted
  filter pitch, threshold 0.3 m/s²: rest is recognised up to 1.75° and lost from 1.7528° = 2 asin(0.3 / 2g). Pinned by
  `unit case K6 marker: the rest test recognises rest at a filter tilt of 2 and 3 degrees`. Code: `window_steps.cpp::
  window_rests` (here) on `imu_kinematics.cpp::is_window_stationary` (front-end). The audit: 21 rest windows missed at
  245-252 s of one Livox 3 run at gain 0; on Livox 4 the test still finds 98 of 100 (research 4b.4); the Livox 4
  reference run of 2026-11-20 (gain 0) takes 98 windows as at rest, none next to a window faster than 0.3 m/s. Bears on
  F12 (the filter is open loop only after its start-up ramp). Order: step 5 (stops and slow walking labelled
  independently first, then the fix).
- **C26** The gyro bias can absorb a constant LiDAR yaw drift (observability). Bias carry: `apply_gyro_bias_prior`,
  `update_gyro_bias_after_window`; the estimate is solved in the window problem. Reported, by theory; crosses to the
  solver. Linked to C38 (front-end: 44's gyro at 253–259.7 s, bias swings −0.86 → +0.48 °/s).
- **C32** The tail is re-propagated from the gyro (`window_steps.cpp::repropagate_window_tail`, called in
  `finish_window`) before the window's points enter the map (`add_window_to_map`), and the next window starts from it:
  each hand-off carries (bias error) × ~33 ms of yaw (30 nodes on the L2, 15 at 500 Hz on the Livox). Reported; the
  test `handoff_tail_nodes = 0` was not run on a recording (on synthetic chains under constant acceleration, tail 15
  against 0: see F2b and the 2026-11-20 fixtures entry). Crosses to the map.
- Fixed in the code (tracker updated 2026-11-18): **C5** (the Cauchy factor 1/π on the rotation prior; removed by D4,
  519b3e8, sigmas ×√π; no Cauchy factor left in the window problem), **C6** (bias subtracted twice; D3, 48734fc: only the update is
  subtracted, test "a gyro bias carried from window to window is removed once"), **C15** (hand-off off-by-one and
  silently dropped windows; D5, a203f1c and 76e2922: `imu_motion_from_previous`, points up to the next window's first
  node, a dropped slice is logged as a warning in `compute_step_1`). **C29** is fixed (76e2922).

## To split in the first job

None: no function of this region is above 80 lines (`python tools/function_sizes.py src --min 80`, 2026-11-18;
`predict_window_poses` was split in stage 6).

## Experiments

- **2026-11-21, consistency harness: the loop and the recordings, this region's readings** (`consistency-harness` 6.1-7.3;
  `experiments/consistency/loop.md`, `replays.md`, `summary.md`; reviewed, shown without a verdict). *Synthetic loop*
  (candidate settings, `ahrs_gain` 0.05, world drawn from the settings' own noise claims, 1,800 points per window, 20
  seeds; R2 10 valid seeds): with both `honest-inertial-carry` switches off / on, R1 13 of 20 runs more than 1 m off
  (8 over 15 m) / 0 of 20 (largest 0.35 m), end-velocity NEES per dof median 59 / 4.5; R3 largest error 0.14 / 0.034 m;
  R2 blind windows NEES 200 / 1.67, RMS position 1.55 / 1.80 m, end error median 2.97 / 0.59 m. With the switches on
  the NEES while moving on R1 is still 4.1 (seen) and 6.6 (turns). A moving window is taken for rest in every R2 run
  (37 windows per setting) and in 19-20 of 20 R3 runs (the tail of the deceleration): not explained (F10 below is the
  noise-free case). *Orientation filter feedback* (noise-free IMU, one seed each; code in the front-end, but the
  rotation edges measure the filter's output, see the review of 2026-11-19): in the first fixture `ahrs_gain` 0.05
  tilted the attitude 0.3-1.2° while the body accelerated and R1 ended 34.2 m off (gain 0: 0.048 m; gain 0.05 with
  bias estimation off: 1.7 m, Codex); in the reworked fixture 1.51 m against 0.014 m. With gain 0 and the matched noise
  the errors were larger (R2 48 m, first fixture). *Recordings* (`tools/honesty_report.py`; forced replays and the
  Livox 4 reference run of 2026-11-20 with gain 0): start-velocity move median 23 (Livox 3, 30 s), 123 / 185 (Livox 4
  passage / flyaway, older settings), 37 (44_short), 26 mm/s (Livox 4 full; at rest 4.5 mm/s); over the 917 moving
  windows of the full run it grows by quarter of the rotation rate 24.0 / 26.1 / 28.0 / 34.7 mm/s, on the 30 s Livox 3
  replay and 44_short it does not. The move is there in the gain-0 Livox 4 run, i.e. without the filter's feedback. Window 373 (133.9 s, corridor): start velocity moved 1.114 m/s from a prior sigma of 5.0 mm/s per axis, at an
  information ratio of 0.051 and a heading bend of 0.15°; which velocity was right is not known (order step 1 looks at
  windows 372-374). Not concluded: which fix is right or what one does to a map; the switches were not run on the
  recordings; the loop's gain 0.05 against the reference run's 0 is noted, not re-run.
- **2026-11-20, consistency harness: the fixtures on this region's code** (`consistency-harness` 5.1-5.8; tests
  `tests/unit_cases_inertial_test.cpp`, `tests/loop_chains_test.cpp`, `tests/window_covariance_test.cpp`,
  `tests/terms_at_truth_test.cpp`; `experiments/consistency/fixtures.md`, logs in `fixtures/`; all on synthetic data
  with the stage 8 candidate settings). Accelerometer terms at the true state within their derived bounds in every
  motion (largest ratio 0.58; exactly 0 at rest and at constant acceleration). K3 / F2a, K4 / F2b, K6 / F3: reproduced
  exactly and pinned (Known flaws). F2 chain at 2 m/s² (paired, one seed): the first moving window's start move +15.1
  mm/s with tail 15, +11.2 with tail 0 (the tail about 4 of 15; audit's fitted +17-19), prior mean error -5.3 mm/s in
  both. **F10** (A2, new; a characterisation, not pinned): a noise-free window at 2 m/s without acceleration passes the
  rest test, and with `imu_always_on` the zero-velocity term solves it as at rest (start 0.646, end 0.010 m/s, move
  0.035 m of 0.716 m; with the rest test forced off it keeps 2 m/s; one window, empty map); not pinned by the user's
  decision (handheld: a walking person never moves without acceleration; a rest test on inertial data alone cannot tell
  constant velocity from rest). **F5** (K11, paired): boundary attitude turned 0.05° with a perfect gyro: the bias leaves
  its control by at most 1.34 claimed sigmas (0.04 °/s; audit 3-5) and stays above 0.8 over windows 5-9; the pitch error
  0.050 -> -0.019° (overshoots to about 40 % of the turn): mechanism yes, size no. **C34** (no LiDAR, shipped position-row
  scale 0.1, 100 runs): switches off, velocity along travel 4.0 per dof (claimed sigma about half the scatter), offset
  1.45; on 1.07-1.12; near the 100-iteration cap. Window algebra (F-c, primary rows, 277 runs per set, this fixture's own
  world: accelerometer floor 0, a = 0.8 m/s², priors drawn from their claim so F2a is left out): handed-on velocity,
  offset, their joint and the gyro bias without LiDAR are inside their 1 % intervals, 0.88-1.06 per dof. **F12** (new
  observation; the filter is the front-end's): the orientation filter's start-up ramp levels on the accelerometer for 3 s
  even with `ahrs_gain = 0`. Blind stretch (20 seeds, last blind window): end-velocity normalised error 19.5 per run with
  the switches off, 1.85 on (interval [0.34, 2.12]; 20 runs cannot show honesty). Not concluded: anything on a recording.
- 2026-11-19 Layer 1 of `honest-inertial-carry` (`imu_edges_follow_state`, fb48605, merged 2b41eaa, off by default): the
  edges' position measurement follows the start velocity and offset. Replays on the five stage 8 Livox 4 captures
  (families of seven on lower, upper, end; `experiments/regions/inertial/2026-11-19-edges-follow-state/summary.md`,
  reviewed): the prior sigma now grows through blind stretches (lower 6.2 -> 36.7 mm/s, today's ceiling 10.7; like
  sqrt(N) without layer 2); the speed two windows after the exit 1.09-1.24 m/s in every member (today 0.16-0.54); path
  in the slow stretch lower 8.50-8.55 m, upper 8.12-8.13 m (today 5.9-6.4, 5.8-7.5). Misses against the design's
  targets: a one-window speed spike at the lower exit (+462 mm, 1.51-1.52 m/s; the decay inside the blind windows stays,
  0.81 -> 0.34 m/s), and end stretch / passage end positions 0.69 / 0.30 m from today (today's spread 0.02). Full Livox 3
  run: within 0.13 m of today (only after its blind stretch at 214 s), no flags, chi2 median 26 (25).
  `experiments/stage8/carry/livox3_efs/`.
- 2026-11-20 Layer 2 of `honest-inertial-carry` (`imu_carry_joint_covariance`, 3d6c03f + 85057e4, merged 164e9c8, off:
  identical): (end velocity, offset) carried with their cross covariance into one 6x6 prior. Tests: a blind chain's
  sigma follows sigma_e x elapsed time within 1 %. Replays with layers 1+2 (`experiments/regions/inertial/
  2026-11-19-joint-carry/summary.md`, reviewed): blind sigma grows ~linearly (lower 5.9 -> 90.6 mm/s, p 1.03); the exit
  correction stays ~1.2 m/s (12.3 sigma, today 6.6); the window after it 87.5 sigma (sigma back to 5.5 mm/s after one
  seen window); lower speed up to 1.86 m/s; end stretch 1.64-2.38 m and passage 0.06 m from today (spread 0.02). Open:
  the sigma collapse after the exit; the end-stretch shift.
- 2026-11-19 Code review of the region (the owner agent, reading only; stage 8). Findings, none in the tracker yet:
  (1) the rotation edges measure the orientation filter's output, not the gyro: `compute_step_1`
  (`lidar_odometry.cpp:334-341`) makes the nodes and `motion_model_poses` from Fusion's orientation and
  `edge_measurement` (`motion_model_terms.cpp:89-108`) takes their relative rotation; Fusion adds gain · (â × ĝ) to the
  gyro (`ahrs_gain` 0.05 in both sensor files, rejection off), by the review's estimate about 0.4 °/s rms while walking
  and 0.06° per window, 5–11 times the edges' sigma with the measured gyro noise (0.006 / 0.013 / 0.009° per window);
  the accelerometer is so used twice. (2) No code path loses rotation in proportion to the turn rate (integration,
  units, bias removal, timing checked); a gyro scale is not modelled and k = 1 leaves no room for one (0.4 % at 30 °/s
  is 5 σ per window); an IMU clock 0.4 % slow against the LiDAR would look the same (not checked). (3) The hard-fixed
  node 0 turns an attitude error ε0 into a bias demand of 1.5 ε0 / T = 4.2 (°/s)/° at T = 0.36 s. (4) `to_mps2 =
  9.80665 / |acc_gravity|` is recomputed after each fold of the offset (`run_odometry.cpp:108`, `:172`): a vertical
  offset of 0.1 m/s² rescales every acceleration by 1 %. (5) The gap between windows gets no bias correction
  (`place_window_after`, `window_steps.cpp:10`; about 0.05° per 30 s at 0.3 °/s). (6) The hand-off extrapolates with
  the finite-difference velocity while the carried velocity is the solve's (mm level). Checked from existing runs: the
  bias estimate swings on all three axes while walking (Livox 4, measured gyro noise: x −0.43…+0.20, y −0.58…+0.36,
  z −0.37…+0.39 °/s; Livox 3 within ±0.36). Not run: `ahrs_gain = 0`; the clock check.
- 2026-11-18 The acceleration offset's random walk (`imu_acceleration_offset_rw_mps2_sqrt_s`, 0.0001 in both sensor
  files, built-in 0.05; the offset is folded into the gravity vector, so it is the state that can follow a tilt of the
  attitude): its prior sigma on the Livox 4 stairs is 0.002 m/s² and the accelerometer's steady misfit against the path
  is up to 1.0° as a tilt (stairs), 0.7° (end stretch), 0.1° (passage, Livox 3). At 0.01 the misfit and the velocity
  chi² fall (stairs 267 → 37; partly by construction: both are measured with the gravity vector the offset adjusts), but
  with the default cell statistics the passage then runs to 11.2 and 20.9 m at up to 3.7 m/s. Full runs with 0.01,
  the exact cell statistics and the projection at 0.02 / 0.04 (the stage 8 candidate): Livox 4 passage 5.94 → 9.53 m,
  no flags, largest bias 0.58 → 0.49 °/s, start-to-end 3.73 → 5.03 m; Livox 3 unchanged (0.06 m). The user's look at
  Livox 4: fewer double walls around the start rooms than before, double walls remain. `openspec/changes/
  livox4-passage-and-flyaway/tasks.md` 3.11, 3.12; `experiments/regions/lidar-matching/2026-11-18-cell-statistics/`.
- 2026-11-18 Stage 6 split of `predict_window_poses` into four steps and one rest test; pure refactor. Full 44 and
  Livox 3 IDENTICAL (35 files each). `openspec/changes/archive/2026-11-18-split-window-prediction/`.
- 2026-11-17 First per-term table, Livox 3 100–130 s (84 windows): start velocity ends a median 2.9 σ (rms) from its
  prior, above 1 σ in 81 of 84; accelerometer cost per residual 359 at the guess, 0.114 at the result; guess cost /
  (velocity change)² median 1.05e6 (corr. 0.69), consistent with `prepare_velocity_state` setting end = start velocity
  (not shown). `docs/window-terms.html` points 4 and 8.
- 2026-11-15 Livox 4 passage (100–110 s): the carried speed decays 1.15 → 0.13 m/s, a 10 m walk is measured as 5.3 m.
  On the 125 s prefix exact cell statistics, projection 0.3/0.5, accelerometer floor 0.02 and fix (a) don't fix passage
  and gallery together. `docs/agents/roadmap.md`, "Flyaway and passage-stall findings".
- 2026-11-12 Gated accelerometer mode and its older fixed-rotation term removed (ten settings); 44_short 10 of 11, full
  44 34 of 35 files identical (only the params TOML). `docs/agents/removal-validation.md`.
- 2026-11-10 livox/2 with k = 1: at position slides (20–22 s, 320–323 s, 3–4.5 m/s while the accelerometer shows
  < 0.5 m/s²) the bias estimate jumps 0.08 → −1.6 °/s (z, claimed σ 0.015) and decays over ~25 s; true bias ≤ 0.35 °/s.
  Memory note `fast-turn-fragility-state.md`.
- 2026-10-17 D5 (C15) and its completion C29: hand-off loss in a test 0.005 rad / 0.86 mm per hand-off → 1e-16 / 0;
  Debug ctest passed. Tracker C29; memory note `fix-plan-state.md`.
- 2026-10-17 D3 (C6): test chain 0.99 → 1.96 of a true 2.0; on the six 95 s cuts of 44 the z bias −0.43 → −0.81 °/s,
  gallery heading unchanged (paired gal−A +0.13°, sd 0.41). Memory note `fix-plan-state.md`.
- 2026-10-16 Hand-eye fit on 44: bias +0.61/+0.46/−0.82 °/s, yaw scale −0.58 ± 0.25 %; the 1/360 hand-off loss predicts
  only ±0.3° of heading. Memory note `fix-plan-state.md`.
- 2026-10-09 `imu_always_on` on the corrected 44 (three runs, IMU +0/+1/+2 ms): stalls in blind passages 2 of 3 runs →
  none; 19/20/19 loops, largest correction ≤ 3.2 m; odometry ~1.85× slower; without the rotation coupling one run
  collapsed (4 loops, 691 m route). `/fp:fast` loses window 213 (stairwell, t ≈ 86 s): 6.1 m apart.
  `openspec/changes/archive/2026-10-09-imu-always-on/tasks.md` 3.1.
- 2026-10-09 Accelerometer fit on 44 with the term off (580 windows): 10 s misfit means mostly < 0.1 m/s² (tilt ≤ 0.6°);
  the rest 1.37 m/s² rms at 50 ms, the ~2 Hz walking surge a constant-velocity window misses; no window at rest. Same
  file, task 1.1.
- 2026-10-08 Ghost wall on 44 from a stall at 210 s in the `/fp:precise` build: the gated accelerometer term opened only
  at window 584, after the stall. `openspec/changes/archive/2026-10-08-odometry-stall-fix/`.
- 2026-10-03 `max_prior_velocity_mps` as a setting (L2 5 m/s): 0 drops on 44_short.
  `openspec/changes/archive/2026-10-03-max-prior-velocity/`.
- 2026-10-03 Only the next window is placed: full 44 within 1e-6; 44 odometry 1567 s vs 1651 s.
  `openspec/changes/archive/2026-10-03-chain-next-window-only/`.
- 2026-09-30 Gyro bias estimation, the time-scaled rotation sigma and the tail hand-off introduced (dataset 43,
  unsynchronized IMU). `openspec/changes/archive/2026-09-30-gyro-bias-estimation/`.

## Ideas rejected

- **Gated accelerometer term** (only in degenerate windows, ratio gate 0.3): opened too late for the 44 stall
  (2026-10-08), replaced by `imu_always_on` (2026-10-09), code removed 2026-11-12 (`docs/agents/removal-validation.md`).
- **Rescues for the `/fp:fast` stairwell window 213** (not handing on the end velocity and offset of a window that
  misfits the accelerometer by > 1 m/s²; re-solving it with inflation k 1, with the bias held, or with a 0.02 m/s²
  floor): no help in the fast build, no effect in the precise one; removed (`2026-10-09-imu-always-on/tasks.md` 3.1).
- **`imu_always_on` without the rotation coupling**: one 44 run collapsed; kept (same source).
- **Honest carried covariance, fix (d)** (`carried_covariance_normal_information_only`, the carried velocity covariance
  from the normal information only): written, never switched on, removed 2026-11-16 in stage 0; to be rebuilt only
  after an IMU-carry design (roadmap step 3). Not rejected on evidence.

## Open questions

- Do the rotation edges have to carry the orientation filter's accelerometer feedback (review 2026-11-19, finding 1)?
  Runs 2026-11-19 (`experiments/stage8/candidate_runs_summary.md`, one run each, no verdict on cause): `ahrs_gain = 0`
  alone did not clearly change the bias swings (rms x/y/z 0.100/0.111/0.114 -> 0.102/0.089/0.100 deg/s) nor the walls;
  the scale alone removed the bias's dependence on the yaw rate (slope -0.22 % -> -0.01 %; the factor was fitted on
  the same recording) but not the swings; both together gave the user's "perfect" map (visit pair difference p50
  0.039 -> 0.020 m). Being confirmed by change `gyro-scale-correction` (repeats, Livox 3).
- The gyro scale: a setting now (`gyro_scale_x/y/z`, `[sensor_calibration]`, change `gyro-scale-correction`), applied
  to the raw gyro as read. Later: measured once by a tool and stored as a calculated value; an estimated state after
  that (the user, 2026-11-19). Gyro sensitivity or a slow IMU clock: not told apart (`tools/imu_time_offset.py`, rate).
  Livox 3 shows the same yaw signature (bias against rate slope -0.26 / -0.29 %, correlation -0.70 / -0.78).
- Is the velocity chi² high on all three recordings (Livox 3 weak windows median 35, judged good) because the prior is
  too tight, or because process noise between windows is missing? (Plan progress, "Verdict limits decided"; shared with
  noise-budget.)
- Does `prepare_velocity_state` starting the end velocity at the start velocity cause the large accelerometer cost at
  the guess (Livox 3, point 8)? Consistent, not shown.
- Livox 4 passage: does a wrong carried speed get baked into a window (roadmap D1: motion-model sigma 1e-4 m per edge,
  C34) and handed on as known to ~5 mm/s (P2)?
- Above `max_prior_velocity_mps` the carried velocity becomes zero with σ `imu_velocity_initial_sigma_mps` (roadmap M5:
  a jolt to standstill): a speed plausibility check instead (roadmap step 4)?
- IMU carry design (c): estimate accelerometer bias and tilt over many seconds while the LiDAR sees, carry them with a
  growing uncertainty through blind stretches (roadmap step 2); C17 belongs to it.
- C26 / C38: a robust bias update when gyro and LiDAR disagree by degrees per window.
- C32: what does `handoff_tail_nodes = 0` do to the heading?
- The `/fp:fast` stairwell window 213 of 44 (fragile decision, open since 2026-10-09).
- The user's L2 has a faulty gyro (held offsets of 22–32 °/s): which L2 recordings can tune the bias model at all?

## See also

- [Regions](README.md)
- [Poses, windows and window chaining](../window-chaining.html)
- [The per-term table](../window-terms.html)
- [Degeneracy information](../degeneracy-information.html)
- [Pipeline](../PIPELINE.md)
- [Code-review tracker](../code-review-tracker.md)
- [The code-split plan](../code-split-and-region-agents-plan.md)
- [All documents](../README.md)
