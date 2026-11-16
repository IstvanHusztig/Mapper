# Proposal

## Why

`optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`) is 1064 lines, in which nine kinds of terms add into one
matrix. No term can be tested alone, and the log can't say which term holds a window back or pushes it away. Stage 2
of `docs/code-split-and-region-agents-plan.md`: split it into one function per term, bit-identical, so that stage 3 can
log each term and the region agents of stage 7 can own them.

## What Changes

- **Shared types** (`src/domain/window_equations.{h,cpp}`):
  - `WindowUnknowns` and `layout_window_unknowns`: the unknowns, their columns, the accelerometer anchors, the band;
  - `WindowEquations`: the banded matrix, the right-hand side, the running motion cost, and whether the cost is wanted.
- **The terms**, moved unchanged and called in their old order:
  - `ndt_term.{h,cpp}`:
    - `sum_ndt_point_terms`: one point term for both grids, with a per-grid policy;
    - `add_projected_ndt_blocks`;
    - `window_translation_information`;
    - `ndt_huber`.
  - `motion_model_terms.{h,cpp}`:
    - `add_motion_edge_terms`: rotation, position and bias coupling, in one loop as before;
    - `add_gyro_bias_prior`;
    - `add_velocity_continuity`.
  - `accelerometer_term.{h,cpp}`:
    - `add_acceleration_terms`;
    - `add_velocity_priors`;
    - `add_acceleration_offset_prior`.
  - `window_solver.{h,cpp}`:
    - `fix_origin`;
    - `extra_unknowns_covariance`;
    - `solve_window_step`;
    - `apply_window_step`.
- **`optimize_lidar_odometry` becomes a conductor** of about 80 lines: the calls in the old order, the cost block, the
  return for the cost-only call, the phase timers.
- **A characterization guard** (a skipped doctest case) writes every output of the optimizer as exact hexadecimal
  doubles. It covers 23 option cases, three calls each, including the branches no shipped run reaches. Every commit of
  stages 2 to 5 must reproduce it byte for byte.
- **A test per term** (`tests/window_terms_test.cpp`, plus the NDT case in `tests/scan_to_map_test.cpp`): the
  right-hand side is minus half the numeric gradient of the term's cost. The projection, the origin fix and the layout
  have their own checks.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `unit-tests`: the terms of the window optimization are tested one by one.

## Impact

- **Code:** `src/domain/scan_to_map.cpp` (1210 → 187 lines); new files `window_equations`, `ndt_term`,
  `motion_model_terms`, `accelerometer_term`, `window_solver`; `CMakeLists.txt`.
- **Results:** none. Every commit stays bit-identical: the guard, the replays of the 44_short and Livox 3 captures, and
  44_short against its manifest.
- **Left above the 80-line target:** `sum_ndt_point_terms` (251 lines) and `add_motion_edge_terms` (216). They are split
  further at the start of stage 3.
