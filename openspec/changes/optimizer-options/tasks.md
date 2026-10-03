# Tasks

Every code commit: both presets build `dMapper` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`. (The GUI must be closed while building.)

## 1. Rename tool

- [x] 1.1 `tools/rename_identifiers.py`: `--lines A-B` (single file only; usage error otherwise) and a dotted NEW for `-r` (design decision 5). Update the docstring and `tools/README.md`.

  Verify on a sample file in the scratchpad: two functions use the same parameter name, and a rename with `--lines` of the first changes only the first; a dotted NEW works for `-r`. Commit.

## 2. Option structs

- [x] 2.1 Add `app/optimizer_options.{h,cpp}` with `ScanToMapOptions`, `MotionModelOptions`, `VelocityPrior`, `get_scan_to_map_options` and `get_motion_model_options` (design decision 1). Include the header from `app/lidar_odometry_utils.h`, and add both to `SOURCES`. Verify both presets build.
- [x] 2.2 Add `tests/optimizer_options_test.cpp` (design decision 6) and compile `app/optimizer_options.cpp` into the tests. Verify `ctest` passes, and that the test fails when two builder lines are swapped (try it once, then revert). Run the step checks. Commit.

## 3. New signature

- [ ] 3.1 Change `optimize_lidar_odometry`'s declaration and definition to the signature of design decision 2. Rename its body with `tools/rename_identifiers.py --lines <function>` (design decision 3): dry run first, review the hit list, then apply.

  In `compute_step_2`:
  - build `ScanToMapOptions` and `MotionModelOptions` once per run, and set the window's `x/y/z_sigma_m` where the `sigma_*_m` locals are set today;
  - build a `VelocityPrior` from `use_prev_velocity`/`prev_velocity`;
  - pass `windows[i]`.

  Verify:
  - `optimize_lidar_odometry` has 15 parameters;
  - a grep for `lidar_odometry_motion_model_|ablation_study_|rgd_params_` in its body finds nothing.

  Run the step checks. Commit.
- [ ] 3.2 Rename the parameters of `optimize_sf2` and `optimize_rigid_sf` (design decision 4), each with `--lines`, dry run first. Verify that a grep for `intermediate_trajectory` in `app/` finds nothing. Run the step checks. Commit.

## 4. Integration

- [ ] 4.1 Full 44 (background): verify IDENTICAL with `tools/baseline/44.manifest.json` (refreshed 2026-10-03), or else that `compare_baseline.py --tolerance 1e-6` against `raw data/44_baseline` passes. Report which.
- [ ] 4.2 Docs:
  - `CLAUDE.md` (Architecture: the option structs and `optimize_lidar_odometry`'s parameters; Testing: the new test source);
  - `docs/ARCHITECTURE.md` (roadmap 5b done; §4, the "about 40 positional arguments" item solved).

  Commit.
- [ ] 4.3 Run `openspec validate optimizer-options --strict` and confirm it passes.
