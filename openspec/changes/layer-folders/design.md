# Design

## Context

After step 7a (`split-utils-header`), these are the include facts the layer split depends on (2026-10-04):
- The domain-side files reach no LASzip, json, GL, `session.h` or Fusion (checked with `/showIncludes`). Their only app-level dependencies are on each other, plus `run_reporting.h` (`scan_to_map` takes a `Logger&`).
- The use-case files still include `core`'s `export_laz.h`, which pulls in `session.h`, `point_cloud.h`, GL and LASzip. The only reason is the inline `points_to_vector`. `lidar_odometry.cpp` also includes `session.h` and `Version.hpp`, but uses neither.
- `lidar_odometry.cpp` uses Fusion (AHRS) and TBB (`parallel_for`), and `input_ports.h` exposes Fusion types (`Imu`). Both are roadmap step 9.
- The app's `get_index` / `get_rgd_index` (`ndt_map.cpp`) have the same signatures and bodies as `core/src/hash_utils.cpp`. Today the exe's own objects win at link time. `CLAUDE.md` calls this "static-archive semantics".
- `main()` sits in `lidar_odometry_gui.cpp`:
  - the CLI branch (`argc == 4`) uses only locals and `formatTime`, which is defined in the GUI file;
  - the GUI branch uses the file-scope globals;
  - a `try`/`catch` around both calls `mandeye::fd::OutOfMemMessage()`.
- `default_settings.toml` lives in `app/`. CMake copies it next to the exe (two places in `CMakeLists.txt`), and one main-spec scenario names its path.

## Goals / Non-Goals

**Goals:**
- The layout of `docs/ARCHITECTURE.md` §5, with each file in its layer.
- Each layer is a CMake library whose include paths admit only what the layer may use. A violation is a compile error, not a review finding.
- Results unchanged (precise build bit-identical).

**Non-Goals:**
- Fusion and TBB behind ports (step 9); until then, they are documented exceptions of `lo_application`.
- Removing the GUI globals (step 10), and sorting `core` (step 11).
- Any renaming beyond decision 3, and any change to `CORE_SOURCE_FILES`.

## Decisions

1. **Layer assignment.** The borderline cases:

   | File | Layer | Why |
   |---|---|---|
   | `run_reporting.h` (`Logger`, `ProgressReporter`) | domain | The optimizers report through `Logger`, and the header has no dependencies. A domain-owned interface that the outer layers implement is still dependency-rule clean. |
   | `odometry_config.h`, `odometry_state.h` | domain | Plain data, used by the domain's option builders (`optimizer_options`) and by the NDT maps. They depend only on `ndt_types.h` and `structures.h`. |
   | `loop_closure`, `consistency` | application | They take the `PointStore` port and run over all windows, so they are the use cases `CloseLoops` / `RefineConsistency`. Their math could move into the domain later (step 8 adds their tests). |
   | `lidar_odometry` (`load_data`, `calculate_trajectory`, `compute_step_1`, `run_consistency`, `save_result`) | application | Use cases, which talk to the ports. |
   | `recording_files` (pairing rules) | infrastructure | Rules about file names and extensions, used only by the folder adapter. |
   | `formatTime` | presentation | Moves to `src/presentation/format_time.h` (inline), shared by the CLI and the GUI. |

2. **Enforcement through include paths, not a script.** Every library gets only the include directories its layer may use. Quoted includes between app files resolve through the `PUBLIC` include directories of the linked libraries, so the moved files need no edits.
   - **`lo_domain`:** `src/domain`, `core/include`, Eigen, manif, `observation_equations/codes`. It gets **no** `3rdparty` root, LASzip, json, freeglut, ImGui, Fusion or TBB directories. Including `session.h`, `point_cloud.h` or `export_laz.h` from `core/include` then fails on `GL/freeglut.h` or `laszip`, so the "math kernel" stays usable while its I/O parts don't.
   - **`lo_application`:** `src/application`, and links `lo_domain` (`PUBLIC`). The exceptions until step 9, marked in `CMakeLists.txt`: Fusion `PUBLIC` (it is part of the `Imu` port type) and TBB `PRIVATE`.
   - **`lo_infrastructure`:** `src/infrastructure`, and links `lo_application`, plus LASzip, json, toml, `core/include` and the dirs `export_laz.h` needs.
   - **The exe** `dMapper`: `src/main.cpp`, `src/presentation/**`, `core/src/utils.cpp`. It links `lo_infrastructure`, `core`, Fusion, the GUI libraries and GL, as today.
   - **The check:** temporarily add `#include <laszip/laszip_api.h>` to a domain file and `#include <nlohmann/json.hpp>` to an application file. The build must fail for both; then revert. The `/showIncludes` script of 7a is retired.

3. **Rename the app's cell-index functions (preparation a).** `get_index` → `ndt_cell_key` and `get_rgd_index` → `ndt_cell_index`, in code only (`tools/rename_identifiers.py`, dry run first). The app files then stop including `<hash_utils.h>`.
   - Once `ndt_map.cpp` sits in `lo_domain.lib`, `core.lib`'s `hash_utils.obj` defines the same symbols. Which one links, or whether linking fails with LNK2005, would then depend on the library search order.
   - Distinct names make the app call its own functions by construction. `core` keeps its own for its own code.

   Alternative: delete the app's copies and call `core`'s. Rejected: `lo_domain` would need `core.lib` to link, which the tests avoid today, and `core`'s copy is compiled with `core`'s flags.

4. **`points_to_vector` to `core/include/point_placement.h` (preparation b).**
   - It is moved verbatim (inline, including Eigen, `structures.h` and `<vector>`), and `export_laz.h` includes the new header, so every `core` user still sees it.
   - The use cases include `point_placement.h` instead of `export_laz.h`. The unused `session.h` / `Version.hpp` includes go (preparation c).

5. **`main()` splits into the composition root and two front ends.**
   - `src/main.cpp` keeps today's `try`/`catch` and dispatches: `argc == 4` → `run_cli(argv)` (`src/presentation/cli/cli.{h,cpp}`, today's CLI branch verbatim), otherwise `run_gui(argc, argv)` (the GUI branch, still in `lidar_odometry_gui.cpp` with its globals).
   - This happens in `app/` first (task 1.2), so the later moves stay pure renames.
   - Alternative: leave `main()` in the GUI file. Rejected: then the CLI would be part of the GUI layer, the opposite of "one pipeline, two front ends".

6. **`default_settings.toml` to `settings/`.** It sits next to `lidar-vertical.toml`, the other shipped settings file. The two CMake copy commands change their source path, and the exe still finds it next to itself. The main spec's scenario is updated by this change's delta.

7. **Commit order:**
   1. preparations a–c;
   2. the `main()` split in `app/`;
   3. the moves and the libraries, together, because the build must stay green. The diff holds only renames, CMake, and the `default_settings.toml` path;
   4. the enforcement check and the full 44;
   5. the docs.

8. **Checks** (as in 7a): every code commit with the step checks (both presets, `ctest`, precise `44_short` IDENTICAL with the precise manifest, fast `44_short` IDENTICAL or within 1e-6), and the full 44 once.

## Risks / Trade-offs

- **[Library boundaries change what `/GL` /LTCG inlines]** Moving code into static libraries can change the `/fp:fast` build's last bits. → The precise build decides bit-identity, and the fast build only has to be within 1e-6.
- **[A missed include directory]** Some file needs a directory its layer doesn't get. → That is the point of the change: if it is a real layer violation, it is reported and pauses implementation; if the file is in the wrong layer, the assignment changes, recorded in the tasks.
- **[Tests and link order]** The tests link `lo_infrastructure` / `lo_application` / `lo_domain` without `core`. If some library object needs a `core` symbol, the test link fails. → The tests then link `core` too, if it links without GL (to be found out); otherwise the dependency is reported.
- **[Docs drift]** About 60 path mentions in the docs. → A scripted `app/<file>` → new-path rewrite over `CLAUDE.md`, `docs/` and `tools/README.md`, then a grep for `app/` that must come back empty outside the archive.
