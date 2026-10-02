# Design

## Context

See proposal.md for the motivation. Where the old names live today:

- **Executable and target:** `CMakeLists.txt`:
  - `project(lidar_odometry_step_1)`;
  - `add_executable(lidar_odometry_step_1 ...)`, plus its compile definitions, include directories, link libraries, the two post-build copies (runtime DLLs and `default_settings.toml`), `/bigobj`, and `install`.

  `tests/CMakeLists.txt` defines its own target `lidar_odometry_tests` and doesn't reference the app target.
- **Window title:** `winTitle` (`app/lidar_odometry_gui.cpp:39`) = "Step 1 (Lidar odometry) " + version. `select_input` changes it to `winTitle + " - ..\<folder>"` with `glutSetWindowTitle`.
- **Info text:** `infoLines` in `app/lidar_odometry_gui.cpp`. Its second-to-last line is overwritten in `display()` with "It saves session file in <working_directory>\lio_result_*", so the vector needs at least two lines.
- **Version resource:** `app/resource.rc`: CompanyName "Mandeye", FileDescription "HDMapping Step 1", InternalName "Odometry", OriginalFilename `lidar_odometry_step_1.exe`, ProductName "HDMapping", ProgramID and LegalCopyright naming `github.com/MapsHD/HDMapping`.
- **Mandeye in the app's own sources:**
  - the `lidar_odometry_gui.cpp` header comment, Info text and folder dialog title;
  - `MandeyeFolderSource` in `app/mandeye_recording_source.{h,cpp}` (used by `app/folder_inputs.cpp`);
  - comments in `recording_files.h` and `folder_inputs.h`;
  - `tools/cut_recording/cut_recording.cpp`;
  - `CLAUDE.md`, `docs/ARCHITECTURE.md`, the `recording-cut` spec Purpose.

  The `mandeye::fd` namespace is defined in `core` (`pfd_wrapper.hpp`).
- **Command name in docs and tools:** `README.md`, `CLAUDE.md`, `tools/README.md`, `docs/ARCHITECTURE.md`, `docs/index.html`, the docstrings of `tools/compare_baseline.py` and `tools/eval_odometry.py`, `tools/merge_chunks.sh`, and the `regression-baseline` spec.

## Goals / Non-Goals

**Goals:**
- The app is dMapper everywhere a user sees a name: the exe, the window title, the file properties, the Info window, the dialogs and the docs.
- No Mandeye references in the app's own code and docs.
- Outputs unchanged, bit for bit.

**Non-Goals:**
- Output naming and contents (`HDMapping_*` files, `session.mjs` keys, `lidar_odometry_version`, `software_version`).
- `core/` and `3rdparty/` (including `mandeye::fd` and `HDMAPPING_VERSION_*`).
- Renaming the `app/lidar_odometry*.{h,cpp}` sources, the test target, or the repository/GitHub name.
- A new icon.

## Decisions

1. **One CMake variable for the name.** `set(APP_NAME dMapper)` is defined once, followed by `project(${APP_NAME})` and `add_executable(${APP_NAME} ...)`, and every target reference uses `${APP_NAME}`. A future rename is then one line.
   - *Alternative:* keep the target name and set the `OUTPUT_NAME` property to `dMapper`. Rejected: `cmake --build --target` would still need the old name, which keeps it alive in docs and scripts.
2. **Window title:** `winTitle = "dMapper"`. `select_input` no longer changes the title, so the `glutSetWindowTitle` call goes.
3. **Info text:**
   1. "dMapper computes the trajectory and point cloud of a LiDAR + IMU recording (lidar*.laz + imu*.csv)."
   2. ""
   3. "Start -> 1. Load parameters, then Start -> 2. Load data and process."
   4. "" (overwritten with the "It saves session file in …" line)
   5. "Each result folder holds session.mjs, the trajectories (trajectory_lio_*.csv) and the point clouds (scan_lio_*.laz, scan_lio_merged.laz)."

   This keeps the second-to-last slot that `display()` overwrites.
4. **Resource strings:**
   - CompanyName "dMapper";
   - FileDescription "dMapper - LiDAR + IMU mapping";
   - InternalName and ProductName "dMapper";
   - OriginalFilename `dMapper.exe`;
   - ProgramID "dMapper";
   - LegalCopyright "dMapper; based on HDMapping (c) 2025 github.com/MapsHD/HDMapping" (attribution kept, since the code derives from HDMapping);
   - versions unchanged.
5. **Adapter rename:** `git mv` `app/mandeye_recording_source.{h,cpp}` → `app/recording_folder_source.{h,cpp}`, and the class `MandeyeFolderSource` → `RecordingFolderSource`. The `SOURCES` list and `#include`s follow. It's a pure rename; the `44_short` check proves it.
6. **Scripts outside the repository** (my check scripts in the session scratchpad): updated to `dMapper.exe` in the same step as the CMake rename, so every later check uses the new exe. The old exe in `out/build/*/bin` is deleted, so a stale binary can't be run by mistake.

## Risks / Trade-offs

- **[A stale old exe is run]:** after the rename, `lidar_odometry_step_1.exe` stays in `bin/` with old code. → Delete it in the CMake task, and say so in the summary for the user's own shortcuts.
- **[Visual Studio keeps the old startup item]** (a target that no longer exists). → The user re-selects `dMapper.exe`; this is mentioned in the summary.
- **[Attribution]:** removing HDMapping from the visible name must not remove the credit. → LegalCopyright and `README.md` keep the HDMapping attribution.
- **[Archived OpenSpec changes still say `lidar_odometry_step_1`]:** they are history and aren't edited.

## Migration Plan

Commits:
1. CMake rename with `APP_NAME`, the scripts updated and the old exe deleted (check `44_short` through `dMapper.exe`);
2. window title, Info text, folder dialog, resource strings;
3. the adapter rename and Mandeye comments;
4. the docs.

No full 44 run is needed: no computation code changes, so `44_short` identical proves the outputs. Rollback: revert, and rebuild the old target.
