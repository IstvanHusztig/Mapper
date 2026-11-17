# Proposal

## Why

The weights of a window's least-squares problem are computed inside the terms, from options, timestamps and carried
state, on every iteration: the motion edges' sigmas (constant or time-scaled), the accelerometer noise formula, the
velocity, bias and offset priors, the two 0.1 scales, the origin fix, the NDT range weight, Huber weight and scale.
Nobody can see a window's noise model as a whole, so the balance between the terms (the question the per-term table of
stage 3 raised, and the velocity chi² of stage 4 that triggers even on a good map) has no single place to be read or
changed. Stage 5 of `docs/code-split-and-region-agents-plan.md`.

## What Changes

- **`WindowNoiseModel`** (`src/domain/window_noise_model.{h,cpp}`, pure refactor): every weight of one window, built
  once per window by `build_window_noise_model` in `prepare_window`, with today's expressions, operand order and
  conditions, together with the window's layout of unknowns.
  - Values fixed for the window: motion-edge weights per edge, velocity-continuity weights, the accelerometer terms'
    weights with their spans, the gyro-bias, start-velocity and acceleration-offset priors (mean, covariance,
    information), zero velocity, the weak end-velocity tie (its 1.0 becomes a named constant), the origin fix.
  - Rules applied per point and per iteration stay in the NDT term with their parameters in the model: planarity, the
    range weight and its cap per grid, the Huber cutoff, `ndt_information_scale`, the projection's ratios.
- **The terms take the model** instead of computing weights; `WindowProblem`, the per-term table and the weak-axis
  diagnostic `ndt_window_cost` read the same model. Consistency's time-scaled rotation sigma calls the model's sigma
  function.
- **The log:** `log_window_noise` (`[lidar_odometry_debug_info]`, off) adds a `window noise <n>:` line per window with
  the model as sigmas; with `log_window_terms` the same values are columns of `window_features.csv`; every run logs one
  `noise budget` line with the settings-derived constants.
- **Clean-ups found by the inventory:** the no-op reset of the motion model's x/y/z sigmas in `prepare_acc_term` is
  removed and `CLAUDE.md` loses the degenerate-window sigma raise that no longer exists; `ndt_window_cost` follows the
  term's per-grid range weight and cap (it applied them to both grids); the origin fix's rotation sigma used in degrees
  as if radians is kept bit-identical, commented, and recorded in the code-review tracker.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `processing-pipeline`: every run logs the noise budget; a debug setting adds each window's noise model.
- `unit-tests`: the noise model's builder is tested on synthetic windows against its formulas and today's terms.

## Impact

- **Code:** new `src/domain/window_noise_model`; `motion_model_terms`, `accelerometer_term`, `ndt_term`,
  `window_solver`, `window_equations`, `window_problem`, `window_term_table`, `scan_to_map` (`ndt_window_cost`);
  `src/application/run_odometry.cpp`, `consistency.cpp`, `window_term_sink.h`; `src/infrastructure/window_term_csv`,
  `toml_io`; `CMakeLists.txt`; tests.
- **Results:** none. Every commit is bit-identical (characterization guard, loop guard, replays, 44_short). Only the
  weak-axis diagnostic's log numbers change (`log_weak_axis_diagnostics`), on purpose.
- **Logs:** every run gains the `noise budget` line.
- **Files that change with the setting:** the three golden TOMLs and the params TOML entry of the manifests (own
  commits); `window_features.csv` gains columns.
- **Run time:** a little less (inverses and per-edge weights computed once per window instead of every iteration);
  measured, not a goal.
- **Not in this change:** any change of a weight's value, the velocity prior's covariance (the stage 4 chi² question),
  the origin-fix quirk, loop closure's and consistency's own weights.
