# Tasks

**Checks on every code commit** (design decision 5), run by the scratchpad's `step_check_gvm.sh`: both presets build `dMapper` and `lidar_odometry_tests`, `ctest` passes; precise and fast `44_short` IDENTICAL.

Close the GUI before building.

## 1. Report a window when it is final

- [x] 1.1 In `optimize_windows`, move `progress.progress(i + 1, windows.size())` after the `add_window_distance` check (decision 1). Verify: the step checks; the CLI log of `44_short` is the same as before apart from timings (diff the logs with numbers in `[s]` masked). Commit. Result: both presets build, `ctest` passes, precise and fast `44_short` are IDENTICAL (11 files). The CLI log (332 lines) differs from the one before only in the end-of-run time line ("finished computation at …", "Elapsed time"), after masking the per-window seconds.

## 2. The view

- [x] 2.1 `GuiProgress::read_finished_windows`, with `stage_started` taking the same lock (decision 2), and the tests of decision 4 in `tests/gui_view_model_test.cpp`. Verify: the step checks. Commit. Result: `read_finished_windows` and a second mutex in `GuiProgress`; `stage_started` switches the stage under it. New test case: no read while loading, a read with `windows_done` 5 during the odometry, a `stage_started(LoopClosure)` from another thread held back while a read is in progress (the stage stays Odometry until the read returns), and no read after it. All 6 GUI test cases pass (42 assertions). Step checks: see 2.2.
- [x] 2.2 `TrajectoryCopy` in `lidar_odometry_gui.cpp` (decision 3): copied inside `read_finished_windows` during the run, rebuilt with the preview when the run ends, cleared on a new run; `draw_run_data` draws it. The preview's reads move inside `read_finished_windows`. Verify: the step checks; `grep -n "run().windows" lidar_odometry_gui.cpp` lists only reads inside `read_finished_windows` or after the run (list them here). Commit. Result: the reads of `run.windows` in `lidar_odometry_gui.cpp` are now `PreviewCloud::add_windows` / `add_window` and `TrajectoryCopy::add_windows`, called from `update_preview` only (a) inside `read_finished_windows`, (b) at the end-of-run edge (running false: the worker only shows its message), and (c) for the preview's final fill after the run; `draw_run_data` draws the copy. Both presets build, `ctest` passes, precise and fast `44_short` are IDENTICAL (11 files), checked on 2.1 and 2.2 together.
- [ ] 2.3 Run the full 44 once (background or subagent). Verify: IDENTICAL with `tools/baseline/44.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_baseline`. Record the result here.
- [ ] 2.4 GUI run by the user (decision 5): `lidar-vertical.toml`, `44_short`; the trajectory grows with the odometry and the cross follows its end; after the run, the trajectory and the point cloud are the final ones. Verify: `lio_result_<n>` IDENTICAL with the `44_short` manifest; record what the user saw.

## 3. Docs

- [x] 3.1 Update `CLAUDE.md` (the GUI's run view: trajectory and preview from finished windows, the lock in `GuiProgress`) and `docs/ARCHITECTURE.md` (10b done). Verify: `openspec validate trajectory-view-race --strict` passes. Commit. Result: `CLAUDE.md` describes the trajectory copy and the rule for reading a run's windows; `docs/ARCHITECTURE.md` marks 10b done.
