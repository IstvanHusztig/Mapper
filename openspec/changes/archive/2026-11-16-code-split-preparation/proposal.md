# Proposal

## Why

Livox 4 has two faults left, both inside the per-window optimizer: a stall in a blind passage and a flyaway. The optimizer
is one function of 1064 lines (`optimize_lidar_odometry`), so no term can be looked at or tested alone.
`docs/code-split-and-region-agents-plan.md` (agreed 2026-11-16) splits it in stages. This change is stage 0, the
preparation:

- **Code that no sensor setting reaches.** Several switches and two whole features are off in both shipped settings files:
  - `degenerate_projection_all_point_normals`, `carried_covariance_normal_information_only` and `ndt_unmatched_point_cost`;
  - robust mode (`optimize_sf2`, `optimize_rigid_sf`, the `[robust_lidar_odometry]` section);
  - `Consistency2` (`use_mutliple_gaussian`).

  They add branches to the code the split must keep bit-identical. The user decided to remove them, and to keep
  `lm_step_check` with its cost code, because the per-term log and the window verdict of later stages need a cost.
- **Results can't be traced to their code.** The good Livox 3 map of 2026-11-12 came from a side build with hidden
  compile-time switches; its output folder recorded neither the commit nor the build flags.
- **No measure of "huge functions".** Stage 6 needs a list of the long functions that can be tracked.

## What Changes

- **Three switches removed:**
  - `degenerate_projection_all_point_normals` and `carried_covariance_normal_information_only`, with the `all_points_*`
    statistics of `NdtBucket` and their map-update code;
  - `ndt_unmatched_point_cost`. The cost still counts unmatched points.
- **Robust mode removed:** `optimize_sf2`, `optimize_rigid_sf`, `update_rgd_spherical_coordinates`,
  `src/domain/robust_lidar_odometry.{h,cpp}` and the 16 settings of `[robust_lidar_odometry]`.
- **`Consistency2` removed**, with `use_mutliple_gaussian`, `consistency_min_distance` and the `NdtBucket2` types.
- **Retired settings still load.** The strict TOML loader keeps a list of removed settings. An older file loads when a
  removed setting asks for the behaviour that remains (it is ignored with a message), and is refused when it asks for
  the removed one.
- **The results record the build.** `cmake/build_info.cmake` writes `build_info.h` at every build. The results JSON gets
  `build_info` with:
  - the git commit, and whether the source had uncommitted changes;
  - the build type, compiler, flags and compile definitions.
- **`tools/function_sizes.py`** lists the functions above a length limit.
- **`compare_baseline.py`:**
  - it ignores the commit and the uncommitted-changes flag;
  - it skips the `.asc` files that the analysis tools export into result folders.
- **Shipped settings files, golden TOMLs and docs** without the removed settings.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `processing-pipeline`: settings files are read strictly, with retired settings; the results record the build.
- `regression-baseline`: the commit and the uncommitted-changes flag are normalized; analysis exports are not compared.
- `unit-tests`: the consistency pass is tested in its one remaining variant.

## Impact

- **Code:**
  - removed: `src/domain/robust_lidar_odometry.{h,cpp}`, `Consistency2` in `src/application/consistency.cpp`;
  - changed: `scan_to_map.cpp`, `ndt_map.cpp`, `ndt_types.h`, `odometry_config.h`, `optimizer_options.*`, `toml_io.*`,
    `odometry_snapshot_file.cpp`, `run_odometry.cpp`, `lidar_odometry.cpp`, `folder_result_sink.cpp`;
  - new: `cmake/build_info.cmake`, `tools/function_sizes.py`.
- **Results:** bit-identical. The only differences are the params TOML (fewer keys) and the results JSON (`build_info`).
- **Snapshots:** odometry snapshots written before this change are refused, because the robust settings were part of
  the odometry settings hash.
- **Baselines:** the manifests are refreshed for the params TOML and the results JSON only.
