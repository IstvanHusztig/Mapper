# Proposal

## Why

The app is no longer HDMapping's "Step 1" or a Mandeye tool. It is the user's own LiDAR + IMU mapping application, used with their dLidar recordings. Its name still says otherwise:
- **window title:** "Step 1 (Lidar odometry) v0.95.0";
- **executable:** `lidar_odometry_step_1.exe`;
- **Windows file properties:** "Mandeye" / "HDMapping";
- **Info text:** "first step in MANDEYE process";
- **folder dialog:** "Select Mandeye data folder";
- **code:** the input adapter is named `MandeyeFolderSource`.

The user decided (2026-10-02):
- the application is called **dMapper**;
- the window title is exactly "dMapper";
- references to Mandeye are removed from the app's own code and docs.

## What Changes

- **BREAKING (command name):** the executable and CMake target become `dMapper` (`dMapper.exe`), and the CMake project is `dMapper`. Scripts and docs that call `lidar_odometry_step_1` change to `dMapper`.
- **Window title:** exactly "dMapper", with no version and no folder name. The folder is already shown in the progress window. The version stays in the Windows file properties and the outputs.
- **Info window text:** rewritten for dMapper: what it does, and where results go (`lio_result_*` with `session.mjs`, trajectories and LAZ files). No Mandeye and no `multi_view_tls_registration` workflow text.
- **Folder dialog title:** "Select recording folder (lidar*.laz + imu*.csv)".
- **Windows version resource** (`app/resource.rc`):
  - ProductName and InternalName "dMapper", OriginalFilename `dMapper.exe`;
  - FileDescription "dMapper - LiDAR + IMU mapping", CompanyName "dMapper";
  - LegalCopyright keeps the attribution to HDMapping, which the code is derived from.
- **Mandeye references in the app's own code and docs:**
  - `MandeyeFolderSource` → `RecordingFolderSource`, and `app/mandeye_recording_source.{h,cpp}` → `app/recording_folder_source.{h,cpp}`;
  - comments in `recording_files.h`, `folder_inputs.h` and `tools/cut_recording/cut_recording.cpp` describe the format neutrally ("recording folder: lidar*.laz + imu*.csv");
  - the header comment of `lidar_odometry_gui.cpp`;
  - `CLAUDE.md`, `docs/ARCHITECTURE.md`, and the Purpose of the `recording-cut` spec.
- **Docs:** `README.md`, `CLAUDE.md`, `tools/README.md`, `docs/ARCHITECTURE.md` and `docs/index.html` use the name dMapper and the new command. The Python tools' docstrings say "dMapper result folder".
- **Unchanged:**
  - every output file: `HDMapping_params_*`, `HDMapping_results_*`, `session.mjs` and its keys, the version strings. Results stay bit-identical, and HDMapping tools can still read them;
  - `core/` (including the `mandeye::fd` file-dialog namespace, which is part of `core`);
  - the source file names `app/lidar_odometry*.{h,cpp}` (their roles change in later roadmap steps);
  - the test target name `lidar_odometry_tests`.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- `gui-workflow`: adds the application name and the exact window title.
- `regression-baseline`: "Deterministic output for identical runs" names the CLI command, which becomes `dMapper`.

## Impact

- **Build:**
  - `CMakeLists.txt` (project and target name, post-build copies, install);
  - the build trees: the old `lidar_odometry_step_1.exe` stays in `out/build/*/bin` until deleted. The rename doesn't need a fresh configure, but Visual Studio's startup item must be re-selected.
- **Code:** `app/lidar_odometry_gui.cpp`, `app/resource.rc`, the renamed adapter files, `app/folder_inputs.{h,cpp}`, `app/recording_files.h`, `tools/cut_recording/cut_recording.cpp`.
- **Docs and tools:** `README.md`, `CLAUDE.md`, `tools/README.md`, `docs/ARCHITECTURE.md`, `docs/index.html`, the docstrings of `tools/*.py` and `tools/merge_chunks.sh`, `openspec/specs/recording-cut/spec.md` (Purpose).
- **Outside the repository:** the user's shortcuts or scripts that call `lidar_odometry_step_1.exe`, and my local check scripts.
- **Validation:**
  - `44_short` must be identical through the renamed CLI;
  - the user checks the GUI: the window title, the Info text and the folder dialog.
