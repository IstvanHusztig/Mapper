# Proposal

## Why

Change B of the code-review fix plan (2026-10-13, `docs/code-review-tracker.md`). These findings are plain defects with an obvious correct form. Most of them never fire on recording 44 (counted in change A: 0 skipped steps, 0 big jumps, 0 dropped windows), so fixing them keeps the results bit-identical. The others are measured. They come first because later changes must not be judged on code that drops work silently, or that accepts loops depending on the distance from the start point (critical for caves hundreds of metres long).

## What Changes

- **C7 — steps never dropped silently:** the solution of a sparse solve is read by row, so an entry that is exactly zero keeps its place. Both the window optimizer and consistency use one shared helper. Neither drops a whole step any more because one unknown's step is 0.
- **C8 — "big jump" rejects the whole step:** when a node's step exceeds 1 m, the optimizer keeps the whole window as it was for that iteration. Today it skips that node but applies the step to the others and to the extra unknowns.
- **C9 — loop closure independent of the world origin:**
  - the correction's "shift" is measured as the movement of the segment's centre, not the translation of a world-frame transform;
  - the plausibility check uses each node's position change;
  - the hard-coded 1.0 m cross-floor limit becomes a setting with today's value.
- **C10 — `ndt_cell_index` uses `floor`:** the cells through the map origin get their correct width. Changes numbers; measured.
- **C11 — `.mrp` / `.mri` written with full precision** (`setprecision(17)`).
- **C12 — output naming:** `point_sizes_per_chunk.json` documented as per window; the CSV `om/fi/ka` columns documented as the raw AHRS angles. No format change.
- **C27 — compiler warnings for missing returns:** MSVC C4715 enabled as an error.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `processing-pipeline`: optimizer steps are applied completely or not at all, never silently in part.
- `loop-closure`: acceptance and plausibility do not depend on where the loop lies relative to the world origin.

## Impact

- **Code:** `src/domain/normal_equations.{h,cpp}` (helper), `src/domain/scan_to_map.cpp`, `src/application/consistency.cpp`, `src/application/loop_closure.cpp`, `src/domain/odometry_config.h` + `src/infrastructure/toml_io.h` (one new setting), `src/domain/ndt_map.cpp`, `src/infrastructure/folder_result_sink.cpp`, `CMakeLists.txt`.
- **Tests:** helper unit test; big-jump test on a synthetic window; loop closure offset invariance (synthetic loop shifted by +200 m gives the same edges); `ndt_cell_index` at negative coordinates.
- **Results:**
  - C7, C8, C11 (data files), C12 and C27: bit-identical on 44 / 44_short, except the `.mrp` / `.mri` text;
  - C9 and C10 change numbers; they are measured with the harness and get new baselines in their own commit.
