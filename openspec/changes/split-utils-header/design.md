# Design

## Context

What each file uses from `lidar_odometry_utils.h` today (found by grepping the code, 2026-10-04):

| Group | Files | Uses |
|---|---|---|
| Optimizers | `scan_to_map`, `robust_lidar_odometry`, `consistency`, `loop_closure` | NDT map, `Point3Di`, transformations, jacobians (scan-to-map also `cauchy.h`), `std::execution` |
| Window loop | `run_odometry`, `lidar_odometry` (`save_result`), `odometry_pipeline` | config, state, NDT map; `lidar_odometry.cpp` also Fusion and transformations |
| Settings | `optimizer_options`, `toml_io`, `toml_config_repository` | config; `NDT::GridParameters` (TOML reads and writes all its fields) |
| Adapters | `folder_result_sink` (LASzip, json, config, state), `recording_folder_source` (LASzip, Fusion), `disk_point_store`, `folder_preview_sink` | mostly their own includes already |
| GUI | `lidar_odometry_gui.cpp` | config, `fs`, iostream |

The `core` headers involved:
- `structures.h` (`Point3Di`, `TaitBryanPose`) and `transformations.h` include only Eigen and the standard library;
- `ndt.h` includes `point_cloud.h` (which includes `GL/freeglut.h` and `observation_picking.h`) and `session.h` (point clouds, manual pose-graph loop closure, control points).

The app only uses four plain structs nested in class `NDT`: `GridParameters`, `Bucket`, `BucketCoef` and `Bucket2`. It also uses `PointCloud::PointBucketIndexPair` once, in `decimate`.

## Goals / Non-Goals

**Goals:**
- No app header includes LASzip, nlohmann json, GL or `session.h` unless it declares something that needs them.
- The domain-side files compile without GL, LASzip, json or Fusion anywhere in their include tree: `scan_to_map`, `normal_equations`, `imu_kinematics`, `window_steps`, `ndt_map`, `optimizer_options` and `robust_lidar_odometry`.
- Results unchanged (precise build bit-identical).

**Non-Goals:**
- Moving files into folders or new CMake targets (step 7b).
- Fusion in `input_ports.h` (step 9), and splitting `core` (step 11).
- Renaming anything beyond the four NDT types and the deleted header.

## Decisions

1. **Three headers, cut by who uses what.**
   - **`ndt_map.h`** includes `ndt_types.h`, `structures.h`, Eigen and `<unordered_map>`. `ndt_map.cpp` is `lidar_odometry_utils.cpp`, renamed with `git mv` so its history follows.
   - **`odometry_config.h`** includes `ndt_types.h` (for `NdtGridParameters`), `structures.h` (for `TaitBryanPose`), Eigen and `<string>`.
   - **`odometry_state.h`** includes `ndt_map.h` and `odometry_config.h` (for `RunPaths`).

   Alternative: one `odometry_types.h` holding everything. Rejected: config users (TOML, GUI) would get the NDT map again, and 7b puts config and state in different layers.

2. **The NDT value types move to `core/include/ndt_types.h`, and `NDT` keeps aliases.**
   - The structs are moved to namespace scope as `NdtGridParameters`, `NdtBucket`, `NdtBucketCoef` and `NdtBucket2`, with identical fields, order and default member values.
   - In class `NDT`: `using GridParameters = NdtGridParameters;` and the same for the others.
   - The app and the tests switch to the new names (`NDT::Bucket` → `NdtBucket` …), mechanically, in their own commit.

   Alternatives:
   - Forward-declaring nested types isn't possible in C++.
   - An app-owned copy of the structs would give two definitions to keep in sync, and calls into `core`'s NDT code would need conversions.
   - Including `ndt.h` and accepting GL defeats the goal.

   `core` is changed only by moving the definitions. This is allowed by the roadmap's rule ("pieces move out of it only when a roadmap step touches them").

3. **`decimate` gets its own index-pair struct.** A file-local `struct PointBucketIndex { int index_of_point = 0; unsigned long long index_of_bucket = 0; };` has the same layout and defaults as `PointCloud::PointBucketIndexPair`. `std::sort` with the same comparator on the same data gives the same order, so the decimated cloud stays identical.

4. **Include what you use, found by the compiler.**
   - After the split, the old header is deleted. Every file that then fails to compile gets exactly the includes it needs: the app headers, the `core` headers, and the third-party headers.
   - `namespace fs = std::filesystem;` goes into each `.cpp` that uses `fs::`. No header defines it.
   - The diff may contain only include lines, the moved blocks, the type renames of decision 2, and the `fs` aliases.

5. **Commit order:**
   1. `ndt_types.h`, the aliases, and the renames in the app and tests (the old header still includes everything);
   2. the three new headers and the `.cpp` rename; `lidar_odometry_utils.h` keeps only includes of the new headers plus its old third-party includes, so every file still compiles;
   3. the old header deleted, with includes fixed file by file;
   4. the include check (decision 6) and the docs.

6. **The include check uses `/showIncludes`.** A scratch script compiles each domain-side file (listed in Goals) with `cl /showIncludes /Zs`, using the dMapper target's include paths and defines (from `compile_commands.json` or the ninja rule). It fails if any included path contains `laszip`, `nlohmann`, `freeglut`, `GL/`, `session.h`, `point_cloud.h` or `Fusion`. The result goes into the tasks, and the script stays in the scratchpad: 7b's CMake targets replace it with link-time enforcement.

7. **Checks** (as in `window-loop-steps`):
   - each code commit: both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
   - the precise build's `44_short` is IDENTICAL with the precise manifest from master;
   - the fast build's `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_short_baseline` (report the largest difference; that folder's params file is older, so that one file is expected to differ);
   - once per change, the full 44: IDENTICAL with `tools/baseline/44.manifest.json`, or within 1e-6 of `raw data/44_baseline`.

## Risks / Trade-offs

- **[Changed include order moves `/fp:fast` last bits]** Different headers can change what the compiler inlines. → The precise build is the bit-identity check, and the fast build only has to be within 1e-6 (decision 7).
- **[A transitive include was supplying something no one names]** Examples: `<execution>`, `<iostream>` for `std::cout` in a test, or `<map>`. → The compiler finds these, and the fix is an include. If a file compiles only because of an accidental include, that's harmless for this change; 7b's targets would expose it.
- **[`core` file touched]** `ndt.h` changes. → Only definitions move, behind aliases, so all `core` code compiles unchanged. `CORE_SOURCE_FILES` and the link order are not touched.
- **[Hidden GL users]** `lidar_odometry_gui.cpp` or `lidar_odometry.h` may have relied on `ndt.h` for GL or `session.h`. → They include what they use (decision 4), and the GUI already includes freeglut itself.
