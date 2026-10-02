# Tasks

Every code commit: both presets build `dMapper` and `lidar_odometry_tests`; `ctest` passes; a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`. (The GUI must be closed while building.)

## 1. Preview during the odometry

- [x] 1.1 `OdometryPipeline::load_window_points(window, points) const` (forwards to the run's `PointStore`; false without inputs). `GuiProgress::windows_done()`: the `done` count reported during the Odometry stage, reset at each run. Verify both presets build.
- [x] 1.2 `PreviewCloud` in `app/lidar_odometry_gui.cpp` (design decision 2): xyz and rgb float arrays; the voxel set (0.10 m); `add_window(i)` (load, transform with `trajectory[index_pose]`, deduplicate by voxel); colours by height, recomputed on frames that added points; the 3-million-point cap with one log line. Verify both presets build.
- [x] 1.3 Drawing and lifecycle (design decisions 3 and 4):
  - `select_input` clears the preview;
  - during the Odometry stage, `display()` adds up to 4 finished windows per frame;
  - the cloud is drawn with client arrays before the trajectory, when `show_point_cloud` is on.

  Verify both presets build. Run the step checks. Commit.

## 2. Final rebuild and view option

(Implemented together with group 1 in one commit: the end-of-run rebuild and the View item belong to the same small state machine, `update_preview()`.)

- [x] 2.1 When `loRunning` goes from true to false, clear and rebuild from all windows (4 per frame). Add View → Show point cloud (default on), first in the View menu. Verify both presets build. Run the step checks. Commit.

## 3. Integration

- [x] 3.1 GUI check, **done by the user**:
  1. start `dMapper.exe`; Start → 1 → `settings/lidar-vertical.toml`, Start → 2 → `44_short`;
  2. watch the coloured cloud grow along the trajectory, and rebuild after the run;
  3. View → Show point cloud toggles it;
  4. rotating and zooming stays smooth.

  Then I verify:
  - the run time (results JSON) is at most 1.3× the CLI's;
  - the result is IDENTICAL with the `44_short` manifest.

  Result: the user ran the GUI with the preview and reported it done (`lio_result_9`). The run took 198.6 s against about 197 s for the CLI (1.0×; 194.9 s without the preview), and the result is IDENTICAL with the manifest (11 files).
- [ ] 3.2 Optional, if the user wants it: a GUI run of the full `44`, checking smoothness and the memory of `dMapper.exe` (Task Manager) during and after the run. The result must be identical or within `--tolerance 1e-6` of `raw data/44_baseline`.
- [x] 3.3 Docs: `CLAUDE.md` (GUI bullet: the live preview, how it's built, View → Show point cloud). Commit.
- [x] 3.4 Run `openspec validate live-point-cloud-preview --strict` and confirm it passes.
