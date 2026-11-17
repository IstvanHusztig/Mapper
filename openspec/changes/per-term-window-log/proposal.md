# Proposal

## Why

Livox 4's two faults sit inside the per-window optimizer (the passage stall at 100–125 s, the flyaway at 200–280 s).
After stage 2 the terms are separate functions, but the log still can't say which term holds a window back or pushes
it away. Stage 3 of `docs/code-split-and-region-agents-plan.md`: a per-window, per-term table of cost, weight and pull,
so a replayed stretch names the term (and later its region) in seconds.

## What Changes

- **Splits first (pure refactors, bit-identical):**
  - `sum_ndt_point_terms` (251 lines): `NdtGrid` and a point-term function (today's local `Grid` and `point_term`), the
    points-by-node index, one grid's loop over the nodes, the conductor. The maps become `const`. A grid can be summed
    alone into node blocks, whatever the projection setting.
  - `add_projected_ndt_blocks`: the projection matrix as its own function.
  - `add_motion_edge_terms` (216 lines): `MotionEdgeWeights` built by its own function; the edge's measurement, block,
    cost and bias coupling as functions. With the other weights zero the same function gives the position or the
    rotation rows alone.
  - `add_acceleration_terms` (116 lines): the per-node samples, one term, the three callers.
  - `add_velocity_priors`: start prior, zero velocity, weak end tie as three functions in today's order.
  - `WindowEquations` gets a residual counter.
- **`window_term_table`** (`src/domain/window_term_table.{h,cpp}`): from the optimizer's inputs, all `const`, the table
  of one window state. Rows: NDT indoor, NDT outdoor, NDT before the projection, edge positions, edge rotations, gyro-bias
  prior, velocity continuity, accelerometer, velocity prior, zero velocity, weak end tie, offset prior, origin fix,
  total, damping. Per row: cost, residuals, and weight and pull for four motions (shift, stretch, curve, turn) along
  three directions (along travel, sideways, vertical).
- **The window loop** evaluates it at the guess and at the result, outside the iterations and the time limit's clock.
  A failure of the table (a non-finite NDT distance) is a warning, never a stopped run.
- **Setting** `[lidar_odometry_debug_info] log_window_terms` (default off; outside the odometry settings hash, so
  captures and snapshots stay valid).
- **Output:** one log line per window; `window_terms.csv` (every row) and `window_features.csv` (one row per window,
  for the future learned tuner) through a new port `WindowTermSink` and a CSV adapter, written by a CLI run to its
  output folder and by `dMapper --replay` next to the capture. `tools/window_terms.py` prints them.
- **Tests:** the rows sum to the real assembly; synthetic cases for each motion and term; results identical with the
  setting on and off.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `processing-pipeline`: a new debug setting writes the per-term window table, without changing results.
- `unit-tests`: the per-term table is tested on synthetic windows.

## Impact

- **Code:** `src/domain/ndt_term`, `motion_model_terms`, `accelerometer_term`, `window_equations.h` (splits); new
  `src/domain/window_term_table.{h,cpp}`; `src/application/run_odometry.cpp`, `window_loop.h`, a new port header,
  `window_replay.cpp`; `src/infrastructure` (CSV adapter, `toml_io`); `src/presentation/cli`; `tools/window_terms.py`;
  `CMakeLists.txt`; tests.
- **Results:** none. Every commit is bit-identical (guard, replays, 44_short); with the setting on the poses are the
  same bits.
- **Files that change with the setting:** the three golden TOMLs and the params TOML entry of the manifests (own
  commits).
- **Time:** with the setting on, two extra assemblies per window (expected a few percent of the odometry); measured.
