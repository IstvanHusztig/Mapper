# Tasks

Every code commit: both presets build `dMapper` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`. (The GUI must be closed while building.)

## 1. Redraw timer

- [x] 1.1 In `app/lidar_odometry_gui.cpp`, replace the `glutPostRedisplay()` at the end of `display()` with `schedule_redraw()` (design decision 1): a `glutTimerFunc` of 100 ms while `loRunning`, 16 ms otherwise, guarded by `redraw_scheduled`.

  Verify that a grep for `glutPostRedisplay` in `app/lidar_odometry_gui.cpp` finds only the one in `redraw_timer`. Run the step checks. Commit.

## 2. Integration

- [x] 2.1 GUI check and measurement, **done by the user**, on an otherwise idle computer:
  1. start `dMapper.exe`;
  2. Start → 1 → `settings/lidar-vertical.toml`, Start → 2 → `44_short`;
  3. check that rotating and zooming the view and the Camera menu views still feel smooth when no run is going.

  Then I verify:
  - the `elapsed_time_seconds` of the new `lio_result_*` is at most 1.3× the CLI's (about 197 s);
  - the result is IDENTICAL with the `44_short` manifest.

  If it is still slower, I report the numbers and stop for profiling, without changing the cap.

  Result: `44_short/lio_result_8` from the GUI took 194.9 s, against about 197 s for the CLI (0.99×; before the cap 510–570 s). It is IDENTICAL with the manifest (11 files). The user reported it much faster.
- [x] 2.2 Docs: `CLAUDE.md` (Architecture, GUI: the redraw is timer-driven, 10/60 fps). Commit.
- [x] 2.3 Run `openspec validate gui-frame-rate-cap --strict` and confirm it passes.
