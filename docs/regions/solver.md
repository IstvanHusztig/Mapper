# Region: solver

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

The part that turns the terms' equations into a window's result: the banded normal matrix the terms add to, the
Levenberg-Marquardt iterations with their damping and stop rules, the step with its checks, and after the iterations the
window verdict. It knows no term's maths; it owns the order in which a window's equations are built and solved.

## Contract

**Takes**
- From the window loop (`process_windows`, orchestrator): `solve_window(LoopContext&, WindowWork&)` with the window, its
  points, the maps, `OdometryRunSetup`, the accelerometer term (`AccTerm`), the window's rotation options and its noise
  model (`WindowNoiseModel`, noise-budget), and the run's `ScanToMapWorkspace`.
- From the term regions, each through `WindowEquations` (`window_equations.h`), called by `WindowProblem::linearize` in
  this fixed order: the NDT point terms (`sum_ndt_point_terms`, then `add_projected_ndt_blocks`; lidar-matching), the
  motion edges, gyro-bias prior and velocity continuity (`motion_model_terms`; inertial), the acceleration terms, velocity
  priors and acceleration-offset prior (`accelerometer_term`; inertial), then `fix_origin` (`window_solver.cpp`).
- Settings: `nr_iter` (100), `lm_factor_initial` (5), `convergence_delta_threshold` (1e-12), `lm_step_check` (off in both
  shipped settings), `real_time_threshold_seconds` (10 s per window).

**Gives**
- The window's poses, velocities (`Window::velocity`), gyro bias (`Window::gyro_bias`) and acceleration offset, changed in
  place, and the covariances of the extra unknowns (`take_covariance`: the bias and end-velocity covariances into
  `Window`, the end velocity's covariance with the offset into `Window::velocity.end_offset_covariance` (rows end velocity,
  columns offset; with the two others the 6×6 joint block), the offset covariance into `WindowProblem::acc_offset_cov_`)
  that the inertial region carries to the next window.
- `WindowOptimization` (an `LmResult`: last iteration, delta, damping, rejected steps, `LmStop`), the counts of failed
  solves and big-jump nodes (warnings in `solve_window`; `silent events` at the end of the run).
- `WindowVerdict` (`judge_window`), pushed to `WindowLoopCarry::verdicts`; `summarize_run` and `log_run_verdict` at the
  end of a run, replay or cancelled run.

**What the others rely on**
- Layout of the unknowns (`layout_window_unknowns`): `[pose_0 .. pose_n-1 | gyro bias (3) | acceleration offset (3) |
  v_start (3) | v_end (3)]`, 6 Tait-Bryan parameters per node; the extras exist only with bias estimation / the
  accelerometer term. The pose band is 17 parameters, wider with the accelerometer anchors; a write outside the band is
  dropped and counted, and `linearize` asserts there is none.
- `BandedNormalMatrix` keeps every element in the order it is given, so its sums are the bits of a dense matrix; its
  sparse pattern is the set of elements written, not the nonzero ones, so `PatternCachedCholesky` reuses the symbolic
  analysis from one iteration to the next. Changing what a term writes changes the pattern, never the result.
- One iteration (`lm_iteration`): linearize; covariance of the extras from the undamped matrix; with the step check a
  snapshot; damping `lm_factor * I` on the pose parameters only (`damped_step`); sparse Cholesky solve
  (`SimplicialCholesky`); `apply`.
- A step (`apply_window_step`): rejected whole when the solve failed or any node would end 1 m or more from its guess
  (`stepped_poses` against `Window::motion_model_poses`, "big jump"); otherwise poses, offset, velocities and bias
  (clamped at `gyro_bias_max_deg_s`) are updated. `delta` is the sum of `|h_x|` over all unknowns (1e6 when not applied).
- Damping (`next_lm_factor`): x10 after iterations 10, 20, ...; x10 at once after a step not applied. With
  `lm_step_check` a step that raised the cost is taken back (`restore_accepted`) and the damping raised x10; that
  iteration skips the convergence test, the scheduled update and the time check; a final cost-only call keeps the
  lowest-cost state.
- Stop (`lm_solve`): converged (`delta` below the threshold on an accepted iteration), iteration limit, time limit
  (`out_of_time`, asked after each counted iteration), or forced (a replay's captured iteration count; no time limit).
  The clock is in the application (`run_lm_iterations`), not in the domain.
- The time limit's clock is `std::chrono::system_clock` (`run_odometry.cpp::run_lm_iterations`), which is not
  monotonic: a clock adjustment during a window can end it early or late.
- A new unknown of a term (inertial) means a change here too: `WindowUnknowns` / `layout_window_unknowns`,
  `take_covariance`, and the state `WindowProblem` saves and restores for the step check.
- The verdict reads the window after the iterations and before the tail is re-propagated (`report_window`, before
  `finish_window`). It only observes: no window is processed differently because of it. A reading a window doesn't have
  is `NOT MEASURED`. Limits (`VerdictLimits`, kept by the user 2026-11-17): bent > 3°, bent severely > 10°, fast >
  2.5 m/s, bias high > 0.5 °/s per axis; run triggers: more than 1 % of measured windows bent, any other flag, velocity
  chi² of the windows with information ratio < 0.45 median > 5 or p95 > 20.

## Owns

- `src/domain/lm_solver.{h,cpp}`: `LmProblem`, `lm_iteration`, `lm_solve`, `next_lm_factor`, `LmStop`.
- `src/domain/window_problem.{h,cpp}`: `WindowProblem`, the window as an `LmProblem` (the term order, snapshots).
- `src/domain/window_solver.{h,cpp}`: `fix_origin`, `apply_window_step`.
- `src/domain/window_equations.{h,cpp}`: `WindowUnknowns`, `layout_window_unknowns`, `WindowEquations`.
- `src/domain/normal_equations.{h,cpp}`: `BandedNormalMatrix`, `PatternCachedCholesky`, `solution_by_row`, `SumBlocks`.
- `src/domain/scan_to_map.{h,cpp}`: `optimize_lidar_odometry` (one iteration, used by the tests and the guard),
  `stepped_poses`, `ndt_window_cost` (diagnostics; repeats the NDT point rules of lidar-matching), `ScanToMapWorkspace`,
  `ScanToMapCost`.
- `src/domain/window_verdict.{h,cpp}`: `judge_window`, `summarize_run`, `log_run_verdict`, `log_window_verdict`.
- In `src/application/run_odometry.cpp`: `run_lm_iterations` (builds the problem and the options, the clock),
  `solve_window`, `window_evidence`.
- Tests: `tests/lm_solver_test.cpp`, `tests/normal_equations_test.cpp`, `tests/window_verdict_test.cpp`; its cases in the
  shared `tests/scan_to_map_test.cpp` and `tests/window_terms_test.cpp`.

## How to check your work

- **Unit tests** (`out/build/x64-Release/bin/lidar_odometry_tests.exe`):
  - `lm_solver_test.cpp` (a two-unknown problem, no map): one iteration and the covariance from the undamped matrix;
    the damping schedule; the four stops; the step check taking a step back.
  - `normal_equations_test.cpp`: banded vs dense sums bit for bit, explicit zeros in the pattern, dropped writes, the
    Cholesky analysis kept and redone, `solution_by_row`, `SumBlocks`.
  - `window_verdict_test.cpp`: flags only above their limits, none at them, `NOT MEASURED`, the run summary, chi² percentiles.
  - shared: "scan-to-map: a node 1 m or more from its guess is a big jump", "the damping grows every 10 iterations",
    "iterations needed", "reusing the Cholesky analyses changes nothing"; "window terms: the layout of the unknowns",
    "the origin fix", "each term counts the scalar residuals".
  - Run only these: `lidar_odometry_tests -tc="lm solver*,BandedNormalMatrix*,PatternCachedCholesky*,solution_by_row*,pose_band_to_sparse*,SumBlocks*,window verdict*,scan-to-map: a node*,scan-to-map: the damping*,scan-to-map: iterations*,scan-to-map: reusing*,window terms: the layout*,window terms: the origin*,window terms: each term*"`.
  - The loop guard ("window loop guard", `tests/window_capture_test.cpp`, skipped: the loop with the step check off and
    on, early ends, rejected steps) and the characterization guard run inside `python tools/step_checks.py`.
- **Replays:** `python tools/run_replay.py <capture> <settings>`; it prints how many windows stopped at the time limit
  (above 0: the result depended on the machine's load, run it again). `python tools/step_checks.py` for "changes no number".
- **Readings that are this region's:**
  - the verdict: `window verdict summary` lines (every run), `window verdict <n>:` per flagged window
    (`log_window_verdict`), the verdict columns of `window_features.csv` (`log_window_terms`): stop cause, iterations,
    damping, rejected steps, steps not applied, big-jump nodes, remaining step.
  - per-term table: the `total` row (remaining step: near 0 when the terms balance, large when the damping froze the
    window) and the `damping` row.
  - log lines: `finished at iteration N/100 ... delta` per window; the warnings "solver step(s) not applied" and "big
    jump"; `silent events` at the end; `convergence window N` with `log_iteration_convergence`; the solver phases
    (covariance solve, solve, update, Cholesky analyses) with `log_stage_timing`.
- **Tools:** `tools/convergence_summary.py` (iterations needed, from a `log_iteration_convergence` log),
  `tools/run_health.py` (the same flags from the log), `tools/window_terms.py`. Verify skills: `verify-run` on every log,
  `verify-robustness` when a stop rule or damping change is said to beat another.

## Known flaws

- **C13** (verified): LM never checks the cost unless `lm_step_check` is on (off in both shipped settings); the damping
  grows x10 every 10 iterations whatever happens; the threshold 1e-12 is never reached, so every window runs all 100
  iterations; the result depends on wall-clock time (`real_time_threshold_seconds`). Today: `lm_solver.cpp::lm_solve`,
  `next_lm_factor`, `damped_step` (absolute damping on the poses; its comment calls a factor grown to ~1e10 negligible), `window_solver.cpp::apply_window_step`
  (`delta` sums units of m, rad, rad/s, m/s and m/s²), `run_odometry.cpp::run_lm_iterations` (`out_of_time`).
  Low priority in the tracker; the 2026-10-18 twin test traced the λ0 dependence to chain amplification.
- **C26** (reported, crosses from inertial): the gyro bias, estimated in the same solve, can absorb a constant LiDAR yaw
  drift (observability, expected by theory). The bias prior is the inertial region's; the joint solve is this region's.
- Fixed, for reference: **C7** (a step skipped when an entry of the sparse solution was exactly zero; now
  `normal_equations.cpp::solution_by_row` keeps zeros in place, and `steps_skipped` counts only failed solves) and **C8**
  (a big-jump node was left out of a step that was otherwise applied; now `apply_window_step` rejects the whole step and
  `next_lm_factor` raises the damping at once).

- **C41** (reported): `SumBlocks` / `Blocks` (`normal_equations.*`) are used only by tests; the comments in
  `phase_timing.h` (`pose_band_to_sparse`) and on `ScanToMapWorkspace` (`scan_to_map.h:23-24`: `steps_skipped` from before the C7
  fix, and big-jump nodes "not moved" from before C8: the whole step is rejected now) are stale. No result affected.

## To split in the first job

None (`python tools/function_sizes.py src --min 80`, 2026-11-18: no function of the solver's files, nor
`run_lm_iterations`, `solve_window` or `window_evidence`, is above 80 lines).

## Experiments

- **2026-11-19** Joint covariance of end velocity and offset (change `honest-inertial-carry`, job J3, design D2):
  `take_covariance` also keeps cov(end velocity, offset) in `VelocityState::end_offset_covariance` (rows `ie`, columns
  `io`); the saved State restores it with `window_.velocity`. It matches a dense inverse within 1e-6 of the block's largest
  entry on the synthetic room, a step taken back restores it bit for bit, step checks identical (nobody reads it yet).
  Not in the window records of `binary_io.h`; the carry's copy is in the capture format 2. Merged 820ee9e.
- **2026-11-17** Verdict readings on full runs of 44, Livox 3 and Livox 4; all limits kept by the user. Bent and fast
  mark Livox 4's flyaway (76 of 85 bent, all 124 fast windows in 200-280 s), not its passage; bias high in 634 of 689
  windows of 44 (L2 gyro); velocity chi² triggers on all three (Livox 3 weak windows median 35).
  `docs/code-split-and-region-agents-plan.md` (Progress).
- **2026-11-17** Solver unit and window verdict (stage 4): every commit identical on the guards, replays and 44_short.
  `openspec/changes/archive/2026-11-17-solver-and-window-verdict/`.
- **2026-11-17** Livox 3 100-130 s, per-term table: all 84 windows ran 100 iterations, damping ends at 5e9; remaining step
  0.0000 m in all twelve motions (largest 0.009 mm). `docs/window-terms.html`.
- **2026-10-18** C13 twin test on 44_short: each window solved twice from the same state with λ0 5 and 0.5 differs p50
  0.0006 mm, max 0.026 mm; chained runs differ at the windows' first node p50 0.65 -> 4.2 mm. A stop rule of 1e-5 m /
  1e-6 rad cuts the iterations about 45 % and does not remove the difference. Tracker C13.
- **2026-10-15** Under memory pressure one window of the full 44 took 10.02 s (limit 10 s); the run diverged from window
  142 on. Tracker C13.
- **2026-10-14** Damping raised at once after a step not applied (`next_lm_factor`): 0 rejections on 44, bit-identical.
  Tracker C8.
- **2026-10-13** C13 probe, 50 more iterations at the starting damping: 44_short yaw sum 0.002°; full s1 windows 630-636
  moved up to 2.2° yaw, map not better (façade -4.55 / -4.81°). Counters: 0 skipped steps, 0 big jumps on 44 s0/s1/s2.
  Tracker C7, C8, C13.
- **2026-10-13** Exact cell statistics: 240-2220 big-jump rejections per run on 44 (before the C8 follow-up). Tracker C20.
- **2026-10-08** Fixed sparse pattern: 6 analyses in 6,900 covariance solves; covariance solve 2.30 -> 0.67 ms and damped
  solve 2.78 -> 1.18 ms per iteration; 44_short odometry 68.8 -> 52.8 s. `/fp:precise` everywhere: the full 44 differs
  from the `/fp:fast` build by up to 1.03 m (ghost wall).
  `openspec/changes/archive/2026-10-08-fixed-pattern-precise-fp/`, `2026-10-08-odometry-stall-fix/`.
- **2026-10-08** Banded normal matrix: the symbolic analysis was 30 % of the odometry on 44_short; a pattern cache on
  the nonzero pattern never hit (6,900 analyses in 6,900 solves). `openspec/changes/archive/2026-10-08-banded-normal-matrix/`.
- **2026-10-04** Iterations needed on 44 (`log_iteration_convergence`): poses settled after median 51, p90 79, max 91 of
  100; delta < 1e-2 after median 27, with a pose error then of median 0.016 mm, max 1.48 mm.
  `openspec/changes/archive/2026-10-04-iteration-convergence-log/`.
- **Before 2026-10-04** `convergence_delta_threshold = 1e-2`: 3-4x faster on recordings 42 / 43, map 0.5-1.5 % less
  sharp. Same change's proposal.

## Ideas rejected

- **A stop rule in physical units** (1e-5 m / 1e-6 rad, 2026-10-18): cuts iterations about 45 % but leaves the chained
  λ0 dependence. Not adopted. Tracker C13.
- **A faster convergence threshold as default** (1e-2): left opt-in; neither `nr_iter` nor the threshold changed
  (2026-10-04). `openspec/changes/archive/2026-10-04-iteration-convergence-log/`, `docs/PERFORMANCE.md`.
- **`/fp:fast`** as the release mode: replaced by `/fp:precise` (2026-10-08); going back would hide fragile windows, not
  fix them. `openspec/changes/archive/2026-10-08-odometry-stall-fix/proposal.md`.
- **Robust mode** (`optimize_sf2`, `optimize_rigid_sf`, the `[robust_lidar_odometry]` section): off in both sensor
  settings, removed 2026-11-16 (stage 0). `openspec/changes/archive/2026-11-16-code-split-preparation/`.

## Open questions

- Should the iterations end on a cost-checked or physically scaled rule by default? `lm_step_check` exists but is off,
  and its effect on full runs is not in the tracker (the roadmap says it stopped the `/fp:fast` runaway on 44 in October).
- What would a convergence threshold mean while `delta` adds metres, radians and m/s together?
- Does the damping (absolute, up to 5e9 on the poses) leave directions outside the table's twelve motions unconverged?
- The only step guard is 1 m from the guess; there is no speed plausibility check (roadmap M4), and a flagged window
  still enters the map (M6). Acting on the verdict was left out of stage 4.
- The velocity chi² triggers even on the good Livox 3: a noise-model question (noise-budget), read here.
- The bias-high flag says almost nothing on the L2; a run trigger on the bias trend is an alternative for later.
- `SumBlocks` / `Blocks` are used only by their test.

## See also

- [Regions](README.md)
- [Code-split plan](../code-split-and-region-agents-plan.md) (stage 4: the solver unit and the window verdict)
- [The per-term table](../window-terms.html)
- [Performance](../PERFORMANCE.md)
- [Code-review tracker](../code-review-tracker.md)
- [All documents](../README.md)
