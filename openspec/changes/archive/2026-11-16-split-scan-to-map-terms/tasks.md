# Tasks

**Step checks, on every commit** (`stage2_check.py`):
- the Release build;
- the unit tests;
- the characterization guard, byte-identical with the reference recorded at 0710807;
- the replays of the 44_short capture (20–40 s, 21 windows) and the Livox 3 capture (100–130 s, 84 windows), every
  window identical;
- 44_short against its manifest.

## 1. Design

- [x] 1.1 Write the design into the plan and have it reviewed against the code. Result: f326a1e.

## 2. Guard

- [x] 2.1 Add the characterization case to `tests/scan_to_map_test.cpp` (skipped; `--no-skip`, file
  `LO_CHARACTERIZATION_OUT`): 23 option cases × 3 calls. Record the reference twice; the two files must be identical.
  The big-jump case must count big jumps. Result: 0710807; repeatable; the big jump counted 50 nodes per call.

## 3. Moves

- [x] 3.1 `WindowUnknowns`, `layout_window_unknowns`, `WindowEquations`; the function uses them, nothing moves. Result:
  962b17c; all checks identical.
- [x] 3.2 `fix_origin`, `extra_unknowns_covariance`, `solve_window_step`, `apply_window_step` in `window_solver`. Add
  `tests/window_terms_test.cpp` with the layout and the origin fix. Result: 184bc97; all checks identical.
- [x] 3.3 `add_gyro_bias_prior`, `add_velocity_continuity` in `motion_model_terms`, with their gradient tests. Result:
  7d76248; all checks identical.
- [x] 3.4 `add_motion_edge_terms`, with `WindowEquations::with_cost`. Gradient test over every pose parameter and the
  bias, with and without time-scaled sigmas and bias estimation. Result: 0b26a7e; all checks identical.
- [x] 3.5 The three accelerometer functions in `accelerometer_term`, with their helpers. Gradient tests over poses,
  offset and velocities; the weak end tie for the end velocity only. Result: 008b8c6; all checks identical.
- [x] 3.6 `sum_ndt_point_terms`, `add_projected_ndt_blocks`, `window_translation_information` in `ndt_term`;
  `optimize_lidar_odometry` becomes the conductor. Tests:
  - the NDT gradient, with and without Huber and the scale;
  - the projection: strong directions pass, weak ones are removed, nothing is added when it's off.

  Result: f5f919d; all checks identical; `scan_to_map.cpp` 1210 → 187 lines.
- [x] 3.7 One NDT point term for both grids with a per-grid policy. Result: 6b83387; all checks identical, including
  the guard's planarity and range-weight cases.

## 4. End of stage

- [x] 4.1 Full 44 and Livox 3 against their manifests. Result (6b83387): both IDENTICAL, 35 of 35 files.
- [x] 4.2 Debug build and `ctest`. Result: pass (10 min).
