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
- Only the next window is placed after a window is done (change `chain-next-window-only`); on a cancelled run
  `place_windows_after` places the rest once.
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
  `run_odometry.cpp::update_acc_term_after_window`. Reported; possible cause of a ~1° tilt.
- **C26** The gyro bias can absorb a constant LiDAR yaw drift (observability). Bias carry: `apply_gyro_bias_prior`,
  `update_gyro_bias_after_window`; the estimate is solved in the window problem. Reported, by theory; crosses to the
  solver. Linked to C38 (front-end: 44's gyro at 253–259.7 s, bias swings −0.86 → +0.48 °/s).
- **C32** The tail is re-propagated from the gyro (`window_steps.cpp::repropagate_window_tail`, called in
  `finish_window`) before the window's points enter the map (`add_window_to_map`), and the next window starts from it:
  each hand-off carries (bias error) × ~33 ms of yaw (30 nodes on the L2, 15 at 500 Hz on the Livox). Reported; the
  test `handoff_tail_nodes = 0` was not run. Crosses to the map.
- Fixed in the code (tracker updated 2026-11-18): **C5** (the Cauchy factor 1/π on the rotation prior; removed by D4,
  519b3e8, sigmas ×√π; no Cauchy factor left in the window problem), **C6** (bias subtracted twice; D3, 48734fc: only the update is
  subtracted, test "a gyro bias carried from window to window is removed once"), **C15** (hand-off off-by-one and
  silently dropped windows; D5, a203f1c and 76e2922: `imu_motion_from_previous`, points up to the next window's first
  node, a dropped slice is logged as a warning in `compute_step_1`). **C29** is fixed (76e2922).

## To split in the first job

None: no function of this region is above 80 lines (`python tools/function_sizes.py src --min 80`, 2026-11-18;
`predict_window_poses` was split in stage 6).

## Experiments

- 2026-11-18 Stage 6 split of `predict_window_poses` into four steps and one rest test; pure refactor. Full 44 and
  Livox 3 IDENTICAL (35 files each). `openspec/changes/archive/2026-11-18-split-window-prediction/`.
- 2026-11-17 First per-term table, Livox 3 100–130 s (84 windows): start velocity ends a median 2.9 σ (rms) from its
  prior, above 1 σ in 81 of 84; accelerometer cost per residual 359 at the guess, 0.114 at the result; guess cost /
  (velocity change)² median 1.05e6 (corr. 0.69), consistent with `prepare_velocity_state` setting end = start velocity
  (not shown). `docs/window-terms-livox3.html` points 4 and 8.
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
- [Per-term table: Livox 3, 100–130 s](../window-terms-livox3.html)
- [Degeneracy information](../degeneracy-information.html)
- [Pipeline](../PIPELINE.md)
- [Code-review tracker](../code-review-tracker.md)
- [The code-split plan](../code-split-and-region-agents-plan.md)
- [All documents](../README.md)
