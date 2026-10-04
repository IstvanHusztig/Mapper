# Design

## Context

After step 10 (2026-10-04):
- **The trajectory:** `draw_run_data` (`lidar_odometry_gui.cpp`) walks `view_model->run().windows` every frame, except during Loading, and draws every node's position plus a cross at the very last node's pose.
- **The worker writes poses:**
  - in `optimize_windows` (`src/application/run_odometry.cpp`): window `i` while it is optimized; then `progress.progress(i + 1, n)`; then `add_window_distance`, which on failure restores window `i`'s poses (and `place_windows_after` places the rest) and ends the odometry; then `place_window_after(windows[i + 1], …)`. `finish_run` writes only `motion_model_poses`;
  - in `post_process`: `run_loop_closure` and `run_consistency` write every window's poses, each after `stage_started(LoopClosure)` / `stage_started(Consistency)`;
  - `save` announces Saving and reads only.
- **The point cloud preview** reads windows below `GuiProgress::windows_done()` while `stage() == Odometry`, and rebuilds from all windows once the run has ended (`running()` false; the worker then only shows its message box).
- **`GuiProgress`** keeps the stage and the window count in atomics; nothing orders the view's "stage is Odometry, read window k" against the worker's "announce loop closure, write window k".
- **The console** prints no per-window line from `progress` in the odometry stage (`ConsoleReporter::progress` returns unless the stage is Loading).

## Goals / Non-Goals

**Goals:**
- No reading of a pose the run may still change, for the trajectory and the preview alike.
- The trajectory shows what is processed: finished windows during the odometry, the final trajectory after the run.
- Results and the CLI's output unchanged.

**Non-Goals:**
- Showing loop closure's or consistency's corrections live (they write all windows at once; the final trajectory appears when the run ends).
- Changing the preview's thinning, colours or limits.

## Decisions

1. **Report a window done only when it is final.** In `optimize_windows`, `progress.progress(i + 1, …)` moves after the `add_window_distance` check. On success nothing writes window `i` again during the odometry (`place_window_after` writes `i + 1`, `finish_run` only `motion_model_poses`). On failure the odometry ends without reporting window `i`, and the view gets it with the rebuild after the run. The processing doesn't read the reporter, so results stay bit-identical, and the console prints nothing per window here.

2. **A lock in `GuiProgress`, shared by the stage change and the view's reads:**
   ```cpp
   // calls read(windows_done) under the lock if the odometry is the current stage; a stage change waits for it
   template <class Read> void read_finished_windows(Read read);
   ```
   - `stage_started` takes the same lock while it switches the stage. The worker announces loop closure (or consistency, or saving) before any write, so once `stage_started` has returned, the view can't be inside a read, and a later read sees the new stage and reads nothing.
   - The view's reads are short: the trajectory copy of a few windows' node positions, and the preview's at most `windows_per_frame` (4) windows a frame, loaded from the point store. The worker waits at most that long, once per stage change.
   - `progress` and `windows_done` stay atomics; the count only grows.

   Alternative: copy the poses on the worker thread at each `progress` call. Rejected: it would put view data into the reporter and copy every pose of a long run twice.

3. **`TrajectoryCopy` in the view** (next to `PreviewCloud`, main thread only): the node positions of the windows copied so far, and the last copied pose for the cross.
   - During the run: inside `read_finished_windows`, append windows `copied … windows_done - 1`.
   - When the run ends (the same edge that triggers the preview's final rebuild): clear and copy all windows of the final trajectory.
   - On a new run (`start_run` returned true): clear, as the preview is cleared.
   - `draw_run_data` draws the copy; it no longer touches `run().windows`.

   The preview's reads move inside `read_finished_windows` too.

4. **Tests:** `GuiProgress::read_finished_windows` reads while the stage is Odometry and passes `windows_done`; it doesn't read in another stage; and a `stage_started` from another thread waits until a read in progress has returned (a read that blocks on a latch, a thread that changes the stage, check that the stage is still Odometry until the latch opens).

5. **Checks:**
   - every code commit: both presets build `dMapper` and `lidar_odometry_tests`, `ctest`, precise and fast `44_short` IDENTICAL (the `run_odometry.cpp` move must not change a result);
   - the full 44 once, because `src/application` changes (IDENTICAL with `tools/baseline/44.manifest.json`, or within 1e-6 of `raw data/44_baseline`);
   - a GUI run by the user: the trajectory grows with the processing and the cross follows it; after the run the trajectory and the preview are the final ones; `lio_result_<n>` IDENTICAL with the `44_short` manifest.

## Risks / Trade-offs

- **[A deadlock]** The view holds the lock while it loads up to 4 windows' points from the point store, which takes the store's own lock; the worker holds no store lock when it calls `stage_started`. → Only these two places take the new lock, and neither calls the other under it.
- **[The trajectory looks different during a run]** Users saw the whole IMU track at once; now it grows. → That is the intended change (spec requirement "Trajectory shown during a run").
- **[A failed odometry]** The last window isn't reported, so the view's trajectory ends one window early until the run ends and the copy is rebuilt. → Acceptable; the run stops there anyway.
