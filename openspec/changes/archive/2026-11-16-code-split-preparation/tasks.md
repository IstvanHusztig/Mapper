# Tasks

**Step checks, on every code commit:**
- the Release build of `dMapper` and `lidar_odometry_tests`;
- `ctest` passes;
- a Release CLI run on `44_short` matches `tools/baseline/44_short.manifest.json`. Only the params TOML may differ, and
  only by the removed keys.

## 1. Plan

- [x] 1.1 Record the user's decisions in `docs/code-split-and-region-agents-plan.md` and commit it. Result: 9d7a882.

## 2. Removals

- [x] 2.1 Remove `degenerate_projection_all_point_normals`, `carried_covariance_normal_information_only` and
  `ndt_unmatched_point_cost`, with the `NdtBucket::all_points_*` statistics and their code in `ndt_map.cpp`. Update
  `scan_to_map_test.cpp` and `ndt_map_test.cpp`, and the snapshot hash's `added_later` list. Run the step checks.
  Result: 43748c7; 44_short identical except the params TOML (3 keys fewer).
- [x] 2.2 Regenerate the golden TOMLs in their own commit. Result: 30c87de.
- [x] 2.3 Update `docs/agents/roadmap.md` (M2, P1, P2, next steps). Result: f6b45f1.
- [x] 2.4 Remove robust mode and `Consistency2` with their settings, `update_rgd_spherical_coordinates` and the
  `NdtBucket2` types. Take them out of the shipped settings files. Reduce `consistency_test.cpp` to `Consistency`.
  Update `CLAUDE.md`, `docs/ARCHITECTURE.md` and `docs/code-review-tracker.md` (C36 moot). Run the step checks.
  Result: 3362a2a; 44_short identical except the params TOML.
- [x] 2.5 Add the retired-settings list to `toml_io.cpp`, with a test in `toml_golden_test.cpp` (accepted at the
  remaining behaviour, refused otherwise, a retired section accepts only its retired keys). Result: in 3362a2a.
- [x] 2.6 Regenerate the golden TOMLs. Result: cc0f561.

## 3. Build identity and function sizes

- [x] 3.1 Add `cmake/build_info.cmake` and the `build_info` target. `FolderResultSink` writes `build_info` into the
  results JSON, and `compare_baseline.py` normalizes `git_commit` and `git_uncommitted_changes`. Run the step checks.
  Result: 194a591; the results JSON differs only by the new block.
- [x] 3.2 Add `tools/function_sizes.py` and its section in `tools/README.md`. Result: in 194a591.

## 4. End of stage

- [x] 4.1 Release build at the committed HEAD, then the full 44 and Livox 3 against their manifests. Result 194a591:
  all files dMapper writes are identical except the params TOML and the results JSON. The 44 manifest of f241eb7 listed
  14 `scan_lio_*.asc` files, exported by `map_quality.py`.
- [x] 4.2 Debug build and `ctest` (the cell struct lost fields). The Debug build tree was stale (cache from the old repo
  path) and was recreated. Result: pass.
- [x] 4.3 `compare_baseline.py` skips `.asc` files. Rewrite the three manifests from the stage-end runs. Result: c365fdd;
  only the params and results hashes changed, and the `.asc` entries are gone.
- [x] 4.4 Document the changes in the user guide (`docs/index.html`). Result: 249acac.
