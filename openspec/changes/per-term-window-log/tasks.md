# Tasks

**Step checks, on every commit** (as stage 2, `stage2_check.py`):
- the Release build;
- the unit tests;
- the characterization guard, byte-identical with the reference recorded at `addd8e4`;
- the replays of the 44_short capture (20–40 s, 21 windows) and the Livox 3 capture (100–130 s, 84 windows), every
  window identical;
- 44_short against its manifest.

## 1. Design

- [x] 1.1 Write the design into the plan and have it reviewed against the code; the user's decisions. Result: plan
  section "Stage 3: per-term log", 2026-11-17, all five decisions as recommended.

## 2. Guard

- [ ] 2.1 Record the characterization reference again from `addd8e4` (the file is not in git), twice; the two files
  must be identical. Verify: `stage2_check.py` passes on `addd8e4`.

## 3. Splits (pure refactors)

- [ ] 3.1 `sum_ndt_point_terms`: `NdtGrid` and the point-term function, the points-by-node index, one grid's loop over
  the nodes, the conductor; the maps `const`; a grid can be summed alone into node blocks. Projection matrix out of
  `add_projected_ndt_blocks`. Verify: step checks identical; existing NDT gradient and projection tests pass;
  `tools/function_sizes.py` shows no NDT function above 80 lines.
- [ ] 3.2 `add_motion_edge_terms`: `MotionEdgeWeights` and its builder; the edge's measurement, block, cost and bias
  coupling as functions. Add a test: the edge block with only position weights plus the one with only rotation weights
  equals the full block. Verify: step checks identical; edge gradient tests pass; no function above 80 lines.
- [ ] 3.3 `add_acceleration_terms` (per-node samples, one term, three callers) and `add_velocity_priors` (start prior,
  zero velocity, weak end tie, in today's order). Verify: step checks identical; accelerometer gradient tests pass; no
  function above 80 lines.
- [ ] 3.4 Residual counter in `WindowEquations`, incremented by every term. Add a test of the counts on a synthetic
  window. Verify: step checks identical.

## 4. The table

- [ ] 4.1 `src/domain/window_term_table.{h,cpp}`: the motion vectors (shift, stretch, curve, turn × along, sideways,
  vertical; velocities as the motion's derivative; turn through the angle Jacobian, empty within 3° of fi = ±90°); the
  rows of the plan, including NDT before the projection, origin fix (rad), total and damping; weight, pull, cost and
  residuals per row. Tests (`tests/window_term_table_test.cpp`): rows sum to the complete equations within a tolerance;
  sideways and stretched NDT guess; accelerometer curve; edge-rotation turn; corridor weights. Verify: the new tests
  pass, step checks identical.

## 5. Window loop, setting, log

- [ ] 5.1 Setting `log_window_terms` in `OdometryConfig`, `TomlIO::POINTERS` and the `[lidar_odometry_debug_info]`
  `CATEGORIES` entry; golden TOMLs regenerated in their own commit. Verify: unit tests pass; a capture made at
  `addd8e4` replays as "same settings".
- [ ] 5.2 Port `WindowTermSink`; the window loop calls the table at the guess (after `motion_model_poses` is set) and at
  the result (directly after `run_lm_iterations`), with the window's own options, no workspace, outside the timed
  spans; exceptions caught as a warning; one log line per window. Test: the synthetic recording through the window loop
  with the setting on and off gives the same poses bit for bit. Verify: that test, step checks identical with the
  setting off, the Livox 3 replay identical with it on.

## 6. Files and tool

- [ ] 6.1 CSV adapter (`window_terms.csv`, `window_features.csv`) in the infrastructure; CLI run writes to the output
  folder, `dMapper --replay` next to the capture. Test of the adapter's columns and rows. Verify: the test passes; a
  44_short run with the setting on writes both files with one table per window.
- [ ] 6.2 `tools/window_terms.py` (pure Python): one window's table, or a per-term summary of a range; documented in
  `tools/README.md`. Verify: it prints the table of a window from the 44_short run of 6.1.
- [ ] 6.3 Manifests: params TOML entries refreshed in their own commit. Verify: 44_short and Livox 3 compare IDENTICAL.
- [ ] 6.4 Docs: `CLAUDE.md` (setting, files, tool), the plan's progress. Verify: the text matches the code.

## 7. End of stage

- [ ] 7.1 Full 44 and Livox 3 against their manifests with the setting off. Verify: both IDENTICAL.
- [ ] 7.2 44_short and the Livox 3 replay with the setting on: poses identical; the odometry time with and without the
  setting reported. Verify: replay exits 0; times written into this file.
- [ ] 7.3 Debug build and `ctest` (the NDT split touches the parallel loop). Verify: pass.
