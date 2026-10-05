# Tasks

## 1. Title

- [x] 1.1 Set `winTitle` in `src/presentation/gui/lidar_odometry_gui.cpp` to "dMapper v 0.1", and update the title in `CLAUDE.md`. Verify: both presets build `dMapper` and `lidar_odometry_tests`, `ctest` passes; the GUI started with no arguments has the window title "dMapper v 0.1" (read with `(Get-Process dMapper).MainWindowTitle`, then closed). No `44_short` / 44 runs: no processing code changes. Commit. Result (2026-10-05): both presets build and `ctest` passes; the GUI started with no arguments reported `MainWindowTitle` "dMapper v 0.1".
