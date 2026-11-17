# Proposal

## Why

The iteration loop of a window (damping, convergence, step check) is tied to the window in every line, so it can't be
tested without a map and its known flaw (every window runs all its iterations) has no home. And nothing judges a
window when it is done: bends, speeds and bias are only found afterwards by parsing the log
(`tools/run_health.py`), and why the iterations stopped is not recorded at all. Stage 4 of
`docs/code-split-and-region-agents-plan.md`.

## What Changes

- **A loop guard first** (a skipped doctest case): the synthetic loop recording through the window loop with
  `lm_step_check` off and on, every window's state as exact hexadecimal doubles. No existing check covers the loop
  with the step check on.
- **The solver unit** (`src/domain/lm_solver.{h,cpp}`, pure refactor): Levenberg-Marquardt over a problem that can
  linearize itself, take the covariance of its undamped unknowns, apply a step, and save and restore its state.
  `lm_iteration` is one iteration; `lm_solve` is today's loop of `run_lm_iterations`, statement for statement, and it
  also returns why it stopped. The time limit and the per-iteration record become callbacks, so the domain reads no
  clock.
- **`WindowProblem`** (`src/domain/window_problem.{h,cpp}`): the window's side of that interface, built from the
  term functions and `window_solver`. `optimize_lidar_odometry` stays as a thin single-iteration call (the
  characterization guard and the term tests use it).
- **`process_windows` in four named steps** (pure refactor): `prepare_window`, `solve_window`, `report_window`,
  `finish_window`, each under 80 lines.
- **The window verdict** (`src/domain/window_verdict.{h,cpp}`): after each window a pure function turns what is known
  into checks, each `CLEAR`, `TRIGGER` or `NOT MEASURED`, following the agreed evaluation contract.
  - Checks with the agreed limits of `tools/run_health.py`: heading bend against the gyro (3°, 10°), end speed
    (2.5 m/s), residual gyro bias (0.5 °/s).
  - Readings without a limit: why the iterations stopped and the solver's events, the start velocity's distance from
    its prior for every window, information ratio, tilt bend, and with `log_window_terms` the per-term costs, the
    remaining step and the LiDAR's weight share.
  - A run summary in every run; `log_window_verdict` (`[lidar_odometry_debug_info]`, off) adds a line per triggered
    window; with `log_window_terms` the checks and readings are columns of `window_features.csv`.
  - It only observes: no window is treated differently.
- **A readings table** of 44, Livox 3 and Livox 4 for the user, who decides which new reading gets a limit.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `processing-pipeline`: every run reports a window verdict summary; a debug setting adds the per-window lines.
- `unit-tests`: the solver is tested on a small problem without a map; the window verdict is tested on made-up evidence.

## Impact

- **Code:** new `src/domain/lm_solver`, `window_problem`, `window_verdict`; `src/domain/scan_to_map.cpp`,
  `window_solver`; `src/application/run_odometry.cpp`, `window_loop.h`, `window_term_sink.h`;
  `src/infrastructure/window_term_csv`, `toml_io`; `CMakeLists.txt`; tests.
- **Results:** none. Every commit is bit-identical (characterization guard, loop guard, replays, 44_short).
- **Logs:** every run gains the verdict summary lines.
- **Files that change with the setting:** the three golden TOMLs and the params TOML entry of the manifests (own
  commits); `window_features.csv` gains columns.
- **Run time for the readings table:** one full run each of 44, Livox 3 and Livox 4 with the table on.
- **Not in this change:** acting on the verdict, changing when the iterations end, limits for the new readings.
