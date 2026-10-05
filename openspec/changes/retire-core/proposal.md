# Proposal

## Why

Roadmap step 11, the last one. `core/` is HDMapping's shared library: about 66 files and 48,000 lines (sessions, point clouds, NDT, ICP, pose graphs, GNSS, ground control points, the viewer). dMapper is built against all of it, yet it uses very little (measured 2026-10-05):
- **the exe links `core.lib` for three functions:** linked without `core`, it misses only `mandeye::fd::OpenFileDialog`, `SelectFolder` and `OutOfMemMessage` (`core/src/pfd_wrapper.cpp`, 90 lines);
- **the domain uses four header-only files:** `structures.h` (`Point3Di`, `TaitBryanPose`, `LaserBeam` …), `transformations.h`, `ndt_types.h`, `point_placement.h`;
- **infrastructure uses `exportLaz`** from `export_laz.h`. It needs only LASzip and Eigen, but the header includes `session.h`, which drags 13 headers and GL into the adapters. That is why `lo_infrastructure` needs `WITH_GUI=1` and the `FREEGLUT_LIB_PRAGMAS=0` workaround;
- **the GUI uses the viewer** in `core/src/utils.cpp` (camera, `initGL`, menus, compass), compiled into the exe; its session-picking functions, which dMapper doesn't call, are what make it include `session.h`;
- **infrastructure uses `csv.hpp`,** a vendored third-party CSV parser.

Everything else costs build time and carries the `CORE_SOURCE_FILES` link-order rule, the `hash_utils`/`surface.cpp` symbol traps, and a review rule ("don't use session types in the domain") that the compiler can't enforce. The user decided (2026-10-05) to retire `core/`: dMapper owns what it uses, and the rest goes.

## What Changes

- **The domain's headers move into `src/domain`,** verbatim (`structures.h`, `transformations.h`, `ndt_types.h`, `point_placement.h`), so their includes stay as written.
- **`exportLaz` moves into infrastructure** (`src/infrastructure/laz_export.{h,cpp}`), unchanged, and the sinks include it instead of `export_laz.h`. `csv.hpp` moves to `3rdparty/csv-parser/`, where vendored libraries live.
- **The GUI gets its own viewer and dialogs** in `src/presentation/gui`: `viewer.{h,cpp}` with the parts of `utils.{hpp,cpp}` the GUI uses (globals, camera, `initGL`, mouse and keyboard, menus, axes, compass, `setNewRotationCenter`), and `file_dialogs.{h,cpp}` with the three dialog functions (from `pfd_wrapper`), using `3rdparty/portable-file-dialogs` (`core`'s copy is byte-identical).
- **`core` is no longer built or linked,** and `core/` is deleted (git history keeps it). `CORE_SOURCE_FILES` and the `core` target leave `CMakeLists.txt`. `lo_infrastructure` loses `WITH_GUI=1` and `FREEGLUT_LIB_PRAGMAS=0`.
- **Vendored libraries nothing builds against any more** (expected: glew, implot, ImGuizmo, glm) leave the build, each confirmed by a green build without it.
- **No behaviour change:** results stay bit-identical (precise build), the GUI looks and works the same.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None: a pure refactor (`skip_specs`). No main spec names `core`.

## Impact

- **Code:** moves into `src/domain`, `src/infrastructure`, `src/presentation/gui` and `3rdparty/`; `core/` deleted; `CMakeLists.txt` (no `core` target, the layer include directories, the exe's links), `tests/CMakeLists.txt` (include directories).
- **Docs:** `CLAUDE.md` (the `core/` library and link-order sections go; layers, the viewer, the dialogs), `docs/ARCHITECTURE.md` (step 11 done; the decision "core is wrapped and moved on touch" closes), `README`-type mentions of `core` in `docs/` and `tools/`.
- **Not in this change:** trimming the moved headers to what dMapper uses (`structures.h` still holds HDMapping structs like `WorkerData`), renaming the `mandeye::fd` namespace, and changing the `HDMapping_*` result file names.
