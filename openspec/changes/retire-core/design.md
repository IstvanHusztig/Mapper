# Design

## Context

Measured on 2026-10-05 (master 2587512):
- **Linking `dMapper` without `core`** fails on exactly three unresolved symbols: `mandeye::fd::OutOfMemMessage` (`main.cpp`), `OpenFileDialog` and `SelectFolder` (`lidar_odometry_gui.cpp`). All three are in `core/src/pfd_wrapper.cpp` (90 lines, which includes `portable-file-dialogs.h`). Nothing else in `core.lib` is reached.
- **`core` headers reached from `src/`,** transitively, 19 of 36:
  - the domain and application: `structures.h`, `transformations.h`, `ndt_types.h`, `point_placement.h`;
  - infrastructure: `csv.hpp` (a third-party CSV parser); `export_laz.h`, for `exportLaz` only (`folder_preview_sink.cpp`, `folder_result_sink.cpp`). `exportLaz` uses only LASzip and Eigen, but the header includes `session.h` → `point_cloud.h` → `GL/freeglut.h` … (13 headers);
  - the GUI: `utils.hpp` (the viewer; its `.cpp` is compiled into the exe), `pfd_wrapper.hpp`, `portable-file-dialogs.h`, `export_laz.h` (included, unused).
- **`utils.hpp` includes `session.h` and `registration_plane_feature.h`** for its last five functions (`rayIntersection`, `GetLaserBeam`, `distance_point_to_line`, `getClosestTrajectoryPoint`, `getClosestTrajectoriesPoint`). The GUI uses only `setNewRotationCenter` of that group, which calls `GetLaserBeam` and `rayIntersection` with a `RegistrationPlaneFeature::Plane`. `structures.h` has a `Plane` with the same fields and constructors.
- **`core/include/portable-file-dialogs.h`** is byte-identical to `3rdparty/portable-file-dialogs/portable-file-dialogs.h`.
- **No other target uses `core`:** `cut_recording` uses LASzip only, and the tests don't link `core`.
- **Vendored libraries not referenced from `src/` or `tests/`:** glew, implot, ImGuizmo (the exe links `imguizmo` and `implot`, and `glm` is on its include path).

## Goals / Non-Goals

**Goals:**
- dMapper builds without `core/`: every file it needs lives in a layer folder or in `3rdparty/`.
- The layers only see what they use: infrastructure no longer reaches `session.h` or GL, and the domain can't include HDMapping's session types at all.
- Results bit-identical, the GUI unchanged.

**Non-Goals:**
- Trimming the moved headers (`structures.h` keeps HDMapping structs like `WorkerData`, `Job`, `LAZSector`; `transformations.h` keeps every pose conversion). A later cleanup, now easy because the compiler sees all users.
- Renaming `mandeye::fd`, changing the GUI's look, or the `HDMapping_*` result file names.

## Decisions

1. **Verbatim moves, include spellings unchanged.** `git mv` keeps the history, and the include directories change instead of the `#include` lines:
   - `core/include/{structures.h, transformations.h, ndt_types.h, point_placement.h}` → `src/domain/` (the domain already exports `src/domain` publicly, so `<structures.h>` resolves for every layer);
   - `core/include/csv.hpp` → `3rdparty/csv-parser/csv.hpp`, on `lo_infrastructure`'s private include path.

2. **`exportLaz` into `src/infrastructure/laz_export.{h,cpp}`.** The function body is copied unchanged; `inline` becomes an ordinary definition in the `.cpp`. The two sinks include `laz_export.h`. The output files are written by the same code, so they stay identical.

3. **The GUI's viewer: `src/presentation/gui/viewer.{h,cpp}`,** from `core/include/utils.hpp` and `core/src/utils.cpp`:
   - **kept, verbatim:** the globals and constants, `truncPath`, `wheel`, `reshape`, `motion`, the keyboard handlers, `ShowMainDockSpace`, `initGL`, `draw_ellipse`, `showAxes`, the camera transition and presets, `camMenu`, `view_kbd_shortcuts`, `cor_window`, `ImGuiHyperlink`, `ShowShortcutsTable`, `info_window`, `drawMiniCompassWithRuler`, `distanceToPlane`, `rayIntersection`, `GetLaserBeam`, `setNewRotationCenter`;
   - **changed:** `distanceToPlane`, `rayIntersection` and `setNewRotationCenter` take `structures.h`'s `Plane` instead of `RegistrationPlaneFeature::Plane` (same fields, same arithmetic); `viewer.h` includes `structures.h` and `imgui.h` instead of `registration_plane_feature.h` and `session.h`;
   - **dropped:** `distance_point_to_line`, `getClosestTrajectoryPoint`, `getClosestTrajectoriesPoint` (session picking, unused by dMapper);
   - whatever the move shows to be unused besides (a function the GUI never reaches) stays, unless it is what pulls in a removed header; the task lists what was dropped.

4. **The dialogs: `src/presentation/gui/file_dialogs.{h,cpp}`,** a verbatim copy of `pfd_wrapper.{hpp,cpp}` (namespace `mandeye::fd` kept, so the callers don't change), built into the exe with `3rdparty/portable-file-dialogs` on its include path. `main.cpp` and the GUI include `file_dialogs.h`.

5. **Retiring `core`:** once nothing includes `core/include` (checked with a grep and a build without that include directory), the `core` target, `CORE_SOURCE_FILES`, `core/src/utils.cpp` in the exe's sources, and every `core/include` include directory leave the CMake files, and `core/` is deleted with `git rm`. `lo_infrastructure` then drops `WITH_GUI=1` and `FREEGLUT_LIB_PRAGMAS=0` (they only existed for `core`'s headers).

6. **Unused vendored libraries:** after step 5, remove from the build each of glew, implot, ImGuizmo and the glm include path, one at a time, keeping the removal only if both presets still build. The `3rdparty/` folders of removed libraries are deleted too (git history keeps them). Any library that turns out to be needed stays, with the reason recorded.

7. **Commit order,** each one green:
   1. domain headers and `csv.hpp` moved (include directories adjusted, `core` still built);
   2. `laz_export` in infrastructure;
   3. `viewer` and `file_dialogs` in the GUI; the exe no longer compiles `core/src/utils.cpp`;
   4. `core` target removed, `core/` deleted, infrastructure without `WITH_GUI`;
   5. unused vendored libraries;
   6. the full 44, then the docs.

8. **Checks:**
   - every code commit: the step checks (both presets build `dMapper` and `lidar_odometry_tests`, `ctest`, precise `44_short` IDENTICAL with the precise manifest from master, fast `44_short` IDENTICAL or within `--tolerance 1e-6`). The precise build tree must be reconfigured after the CMake changes;
   - the full 44 once (IDENTICAL with `tools/baseline/44.manifest.json`, or within 1e-6 of `raw data/44_baseline`);
   - a GUI run by the user after commit 3 (and once more after 5 if libraries left): start-up, both menus, Camera presets, Ctrl + right-click rotation centre, compass, Load parameters, a `44_short` run, its result IDENTICAL with the manifest.

## Risks / Trade-offs

- **[Header order and macros]** `utils.hpp` defined `DEG_TO_RAD` / `RAD_TO_DEG` as floats, and the domain's `angle_units.h` has its own. Moving `utils.hpp` content into `viewer.h` keeps the GUI's float versions in the GUI only. → The GUI file includes both today; the move doesn't change that.
- **[A hidden user of a deleted header]** Something includes a `core` header only through another one (e.g. `<Eigen/...>` or `<filesystem>` through `structures.h`). → The build fails at commit 4 and the missing include is added where it's used, recorded in the task.
- **[`/fp:fast` last bits]** `exportLaz` leaving an inline header, and `WITH_GUI` leaving infrastructure, can change what `/GL` inlines. → The precise build decides; the fast build only has to be within 1e-6.
- **[A large delete]** About 60 files go. → It is a `git rm` in its own commit; everything is in history, and `3rdparty` libraries are removed one at a time with a build in between.
