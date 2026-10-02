# Tasks

Every code commit: both presets build `dMapper` (before task 1.1: `lidar_odometry_step_1`) and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`. (The GUI must be closed while building.)

## 1. Executable name

- [x] 1.1 `CMakeLists.txt`: add `set(APP_NAME dMapper)`; use `project(${APP_NAME})`, `add_executable(${APP_NAME} ...)` and `${APP_NAME}` in every target reference (compile definitions, includes, links, both post-build copies, `/bigobj`, install).

  Update my check scripts to `out/build/<preset>/bin/dMapper.exe`, and delete the old `lidar_odometry_step_1.exe` (and its `.pdb`) from both build trees.

  Verify:
  - `dMapper.exe`, the runtime DLLs and `default_settings.toml` are in `out/build/x64-Release/bin`;
  - a grep for `lidar_odometry_step_1` in `CMakeLists.txt` finds nothing.

  Run the step checks through `dMapper.exe`. Commit.

## 2. Window title, Info, dialog, file properties

- [x] 2.1 `app/lidar_odometry_gui.cpp`:
  - `winTitle = "dMapper"`; remove the `glutSetWindowTitle` call in `select_input`;
  - the Info text of design decision 3;
  - folder dialog title "Select recording folder (lidar*.laz + imu*.csv)";
  - header comment rewritten without Mandeye or `multi_view_tls_registration`.

  Verify both presets build.
- [x] 2.2 `app/resource.rc`: the strings of design decision 4. Verify both presets build, and that the file properties of `dMapper.exe` (PowerShell `(Get-Item ...).VersionInfo`) show ProductName dMapper and OriginalFilename `dMapper.exe`. Run the step checks. Commit.

## 3. Mandeye references in the code

- [ ] 3.1 `git mv` `app/mandeye_recording_source.{h,cpp}` → `app/recording_folder_source.{h,cpp}`, and rename `MandeyeFolderSource` → `RecordingFolderSource` (`SOURCES` in `CMakeLists.txt`, `#include`s, `app/folder_inputs.{h,cpp}`).

  Rewrite the comments in `recording_files.h`, `folder_inputs.h`, `recording_folder_source.h` and `tools/cut_recording/cut_recording.cpp` neutrally.

  Verify that a case-insensitive grep for `mandeye` in `app/`, `tools/`, `tests/` and `CMakeLists.txt` finds only `mandeye::fd::` calls (the `core` namespace). Run the step checks. Commit.

## 4. Docs

- [ ] 4.1 Use the name dMapper and the command `dMapper <input_folder> <params.toml> <output_folder>` in:
  - `README.md` (title and build/run lines; the HDMapping attribution stays);
  - `CLAUDE.md` (What this is, Build, Run, Architecture: `RecordingFolderSource`, no "Mandeye-style");
  - `tools/README.md`;
  - `docs/ARCHITECTURE.md`;
  - `docs/index.html`;
  - the docstrings of `tools/compare_baseline.py` and `tools/eval_odometry.py`, and `tools/merge_chunks.sh`.

  Edit the Purpose of `openspec/specs/recording-cut/spec.md` to say "recording folder (lidar*.laz + imu*.csv)".

  Verify that a grep for `lidar_odometry_step_1` outside `openspec/changes/archive/` finds nothing. Commit.

## 5. Integration

- [ ] 5.1 GUI check, **done by the user**: start `out\build\x64-Release\bin\dMapper.exe`; the title reads "dMapper" before and during a `44_short` run (Start → 1 → `lidar-vertical.toml`, Start → 2 → `44_short`); Info shows the new text; the folder dialog shows the new title. Then I verify that the result is IDENTICAL with the `44_short` manifest.
- [ ] 5.2 Run `openspec validate rename-to-dmapper --strict` and confirm it passes.
