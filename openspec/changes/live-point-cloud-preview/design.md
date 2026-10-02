# Design

## Context

- **`compute_step_2`** handles windows in order. For window `i` it loads the window's intermediate points (`PointStore`, `WindowIntermediate`), optimizes `worker_data[i].intermediate_trajectory`, applies the hand-off correction to that window's last nodes, and only then reports `progress.progress(i + 1, n)` (`app/lidar_odometry_utils_optimizers.cpp`, around line 2021). So once window `i` is reported done, its trajectory doesn't change again during the odometry.
- **Loop closure and consistency** (`post_process`) later change the trajectories of many windows. `save` uses the final ones.
- **Global points:** each intermediate point carries `index_pose`. Its world position is `worker_data[i].intermediate_trajectory[p.index_pose] * p.point`, the same expression `compute_step_2` uses to build `points_global`.
- **GUI:**
  - `display()` already draws `pipeline.worker_data[*].intermediate_trajectory` every frame while the worker thread runs. `worker_data` isn't resized during step 2, only its contents change.
  - The redraw is timer-driven (about 10 fps during a run, 60 otherwise).
  - `GuiProgress::progress(done, total)` stores the fraction for the progress window.
  - `loRunning` is set by `openData` and cleared by the worker thread after saving.
- **`DiskPointStore::load`** keeps no state between calls (it opens the file for that window), so reading a finished window's file from the main thread while the worker reads others is safe.

## Goals / Non-Goals

**Goals:**
- A cloud that grows window by window during the odometry, and a rebuild from the final trajectory after the run.
- No change to processing, results or run time (within the 1.3× of `gui-workflow`).
- Bounded memory and drawing cost on the full 44 recording.

**Non-Goals:**
- Showing the cloud during loading, or live updates during loop closure and consistency (only the final rebuild).
- Colour by intensity or other colour modes, and point size settings.
- A preview in the CLI.
- Using the NDT bucket map. It is a sliding local map (cleared as the trajectory advances), so it can't show the whole run.

## Decisions

1. **The preview is built on the GUI's main thread from data the pipeline already keeps.**
   - `OdometryPipeline` gets `bool load_window_points(std::size_t window, std::vector<Point3Di> &points) const`, which forwards to its `PointStore`. It is the only pipeline change, and it adds no computation to the run.
   - `GuiProgress` records the number of finished windows during the Odometry stage (`windows_done()`, atomic).
   - *Alternative rejected:* a preview port called from `compute_step_2`. It would add code to the optimizer (with `/fp:fast` a code-generation risk for the 44 baseline) and couple processing to display.
2. **`PreviewCloud`** (GUI file, main thread only):
   - Members: `std::vector<float>` xyz and rgb arrays, an `std::unordered_set<uint64_t>` of occupied voxel keys (0.10 m display voxel), the next window to add, and the z range.
   - `add_window(i)`: load the window's points, transform each with `trajectory[p.index_pose]`, and keep a point only if its voxel isn't occupied yet. This deduplicates across overlapping windows, so the preview grows with the mapped volume, not the run length.
   - Per frame, at most 4 windows are added, so a frame stays short. The colours (height ramp blue → red across the current z range) are recomputed only on frames that added points.
3. **Drawing:**
   - `glVertexPointer`/`glColorPointer` + `glDrawArrays(GL_POINTS)` (OpenGL 2 client arrays), with point size 2. One call per frame instead of a `glVertex` call per point.
   - It is drawn before the trajectory, so the trajectory stays visible on top.
   - View → **Show point cloud** (`show_point_cloud`, on by default).
4. **Lifecycle:**
   - `select_input` (a new run) clears the preview.
   - During the run, after the Odometry stage starts, windows `next_window .. windows_done()-1` are added.
   - When `loRunning` changes from true to false, the run has saved. The preview is cleared and rebuilt from all windows with the final trajectories, 4 per frame, which takes about 3 s for 690 windows at 60 fps.
   - If the run failed (no windows), nothing is drawn.
5. **Limits:**
   - A 0.10 m voxel bounds the preview by the mapped volume. The full 44 cave run is expected at a few hundred thousand points: about 7 MB of floats, drawn in about 1 ms with client arrays. The measurement on 44 confirms this.
   - If the count exceeds 3 million, the builder stops adding and logs one line, a safety cap.

## Risks / Trade-offs

- **[Reading `worker_data[i]` while the worker writes other windows]:** formally a data race on the vector contents. Only finished windows (`i < windows_done()`) are read, and nothing writes them until post-processing; the trajectory drawing already does the same.
  → During post-processing (loop closure, consistency) the builder adds no windows; it waits for the end-of-run rebuild.
- **[Main-thread file reads stall the UI]:** about 4 small files per frame. A stall shows as lag, not a wrong result. → Lower the per-frame count if the user notices it.
- **[Run time]:** the main thread does more work at 10 fps. → The GUI check compares the run time with the CLI's (within 1.3×) and the result with the baseline.
- **[Height colours on very flat scans]:** a zero z range is clamped to 1 m.

## Migration Plan

Commits:
1. the accessor, `windows_done()`, `PreviewCloud` and the drawing;
2. the end-of-run rebuild and the View item.

Each is built in both presets, `ctest`, and checked with the CLI on `44_short`. Then the user's GUI run of `44_short`: growth, final rebuild, run time and identical result. Optionally a GUI run of the full 44, to check memory and smoothness. Rollback: revert.
