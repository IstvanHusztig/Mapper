# Tasks

**Checks on every code commit** (design decision 8), run by the scratchpad's `step_check.sh`:
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- the precise build's `44_short` is IDENTICAL with the precise manifest from master;
- the fast build's `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_short_baseline` (report the largest difference).

Close the GUI before building. After task 2.1, the precise build tree must be reconfigured (the source list changes).

## 1. Preparations in `app/`

- [x] 1.1 Preparations a–c (decisions 3 and 4):
  - rename `get_index` → `ndt_cell_key` and `get_rgd_index` → `ndt_cell_index` in `app/` and `tests/` (`tools/rename_identifiers.py`, `--dry-run` first), and drop the then unused `<hash_utils.h>` includes;
  - move `points_to_vector` verbatim into `core/include/point_placement.h`, with `export_laz.h` including it;
  - `run_odometry.cpp` and `lidar_odometry.cpp` include `point_placement.h` instead of `export_laz.h`, and the unused `session.h` / `Version.hpp` includes go.

  Verify: the step checks; `grep -rn "get_rgd_index\|get_index\|hash_utils\|export_laz\|session.h" app tests` lists only adapters (`folder_*`, `recording_folder_source`) and the GUI. Commit. Result: 18 renames, all in `app/` and `tests/`. The grep now finds `export_laz.h` only in `folder_preview_sink.cpp`, `folder_result_sink.cpp` and the GUI. Removing `export_laz.h` also removed a `using namespace std` that two spots relied on, so `ofstream` (`lidar_odometry.cpp`) and `fixed` (`run_odometry.cpp`) are now `std::`-qualified. Both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files).
- [x] 1.2 Split `main()` (decision 5) in `app/`:
  - `app/main.cpp` with the `try`/`catch` and the dispatch;
  - `app/cli.{h,cpp}` with `run_cli`, today's CLI branch verbatim;
  - `run_gui` in `lidar_odometry_gui.cpp`;
  - `formatTime` in `app/format_time.h`.

  Verify: the step checks (the CLI runs through `main.cpp`); start the GUI once with no arguments, and check that it opens and loads `default_settings.toml` (title "dMapper"); an unreadable TOML in CLI mode still exits 1. Commit. Result: both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files) through `main.cpp`. The GUI started with no arguments opened a window titled "dMapper" and logged "Default settings loaded from: '…in\default_settings.toml'". `dMapper a b c` printed "Cannot load parameters from: 'b'" and exited 1.

## 2. Folders and libraries

- [x] 2.1 `git mv` every file of `app/` into `src/domain`, `src/application`, `src/infrastructure`, `src/presentation/cli`, `src/presentation/gui` and `src/main.cpp` (decision 1). Move `default_settings.toml` to `settings/` (decision 6). Define `lo_domain`, `lo_application` and `lo_infrastructure` with the include directories of decision 2, link `dMapper` from them, and make `tests/CMakeLists.txt` link the libraries instead of compiling app sources. Verify:
  - `app/` no longer exists;
  - `git diff -M --stat` shows the sources as renames with no content change;
  - the step checks pass;
  - `default_settings.toml` is still copied next to both exes.

  Record any file whose layer had to change, and why. Commit. Result: the design's layer assignment held. All 59 files moved as renames with no content change; the build failed on nothing on the first attempt. `default_settings.toml` is copied next to the exe and is byte-identical with `settings/default_settings.toml`. Both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files); the precise tree was reconfigured with the libraries.
- [x] 2.2 The enforcement check (decision 2): temporarily add `#include <laszip/laszip_api.h>` to `src/domain/scan_to_map.cpp` and `#include <nlohmann/json.hpp>` to `src/application/run_odometry.cpp`. Verify: both fail to compile, with the error text recorded here. Revert, and check that the tree builds again. No commit (nothing changes); record the result.

  Result (both reverted, the tree builds again):
  - **Fail as intended:** `scan_to_map.cpp` + `<laszip/laszip_api.h>` → `fatal error C1083 … "laszip/laszip_api.h": No such file or directory`; `run_odometry.cpp` + `<nlohmann/json.hpp>` → the same for `nlohmann/json.hpp`. `core`'s `export_laz.h` in the domain also fails, on its LASzip include.
  - **Finding:** `core`'s `session.h` **compiles** in the domain. Its GL and ImGui includes (`point_cloud.h`, `control_points.h` …) sit behind `#if WITH_GUI == 1`, and the domain doesn't define `WITH_GUI`, so the session and point-cloud types are plain data there. Design decision 2 claimed they would fail. The user decided (2026-10-04) to document this gap instead of adding a guard: they stay a review rule until roadmap step 11 sorts `core`. Design decision 2 and `CLAUDE.md` are corrected.
- [x] 2.3 Run the full 44 once (background or subagent). Verify: IDENTICAL with `tools/baseline/44.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_baseline` (report the largest difference). Record the result here. Result (2026-10-04): IDENTICAL with `tools/baseline/44.manifest.json` (35 files), 2597 s wall time.

## 3. Docs

- [x] 3.1 Rewrite every `app/<file>` path in `CLAUDE.md`, `docs/*.md`, `docs/*.html` and `tools/README.md` to its new location (scripted), and update:
  - the `CLAUDE.md` sections Build (output, the `default_settings.toml` copy), Testing (the tests link the libraries), and Architecture (the layer folders and libraries, the Fusion/TBB exceptions, the cell-index rename in the link-order paragraph);
  - `docs/ARCHITECTURE.md`: step 7b done, §4/§5 as built.

  Verify: `grep -rn "app/" CLAUDE.md docs tools/README.md` finds only history (list each one), and `openspec validate layer-folders --strict` passes. Commit.

  Result: 80 paths rewritten by script, every one mapped:

  | File | Paths |
  |---|---|
  | `CLAUDE.md` | 38 |
  | `ARCHITECTURE.md` | 11 |
  | `index.html` | 11 |
  | `PIPELINE.md` | 7 |
  | `degeneracy-information.html` | 5 |
  | `window-chaining.html` | 5 |
  | `PERFORMANCE.md` | 3 |

  Reworded by hand:
  - `CLAUDE.md`: a Layers paragraph, the tests link the libraries, the link-order paragraph with the rename, `run_cli`;
  - `PIPELINE.md`: "the code is in `src/`";
  - `ARCHITECTURE.md`: 7b done, §5 as built.

  The `app/` mentions left are history: `ARCHITECTURE.md` lines 59 and 155 (`lidar_odometry_utils.h`), and the 7b entry itself. `openspec validate --strict` passes.
