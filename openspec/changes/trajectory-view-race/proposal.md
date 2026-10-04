# Proposal

## Why

Roadmap item 10b, found while proposing step 10. During a run the GUI draws the trajectory from every window's poses while the worker thread writes them, in the odometry and again in loop closure and consistency. That is a data race: the view can show half-updated poses, and in C++ it is undefined behaviour. It also shows something misleading: before the odometry reaches a window, that window's poses are only the IMU placement, so the drawn trajectory runs ahead of the processing and the red cross sits at the end of the IMU track, not where the odometry is.

The point cloud preview already reads only windows the odometry reported done, but it has two small gaps of its own: the odometry can still change the window it just reported when the run fails there, and the view checks "the stage is Odometry" before reading, while the worker may switch to loop closure in between.

## What Changes

- **The trajectory is drawn from a copy, as the preview is.** During the odometry the view copies the node positions of each window reported done, and draws the copy: the trajectory grows with the processing, and the cross marks the last finished pose. During loop closure, consistency and saving the copy stays as the odometry left it. When the run has finished, the copy is rebuilt once from the final trajectory, together with the preview.
- **Reading finished windows is synchronised with the stage change.** `GuiProgress` holds a lock that the view takes while it reads finished windows (trajectory and preview), and that a stage change takes too. The worker announces loop closure before it writes any pose, so once the view sees a stage other than Odometry under the lock, it reads no more.
- **A window is reported done only when the odometry won't change it again.** `optimize_windows` reports progress after the failure check (which can restore the window's poses), instead of before. The console prints nothing per window in the odometry stage, so the CLI's output doesn't change; results don't change either.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `gui-workflow`: a new requirement for the trajectory shown during and after a run (only finished windows, the cross at the last finished pose, the final trajectory after the run, no reading of poses the run may still change).

## Impact

- **Code:** `src/presentation/gui/gui_progress.h` (the lock), `src/presentation/gui/lidar_odometry_gui.cpp` (the trajectory copy, the preview reading under the lock), `src/application/run_odometry.cpp` (the progress report moves by a few lines); `tests/gui_view_model_test.cpp` (the lock against a stage change).
- **Docs:** `CLAUDE.md` (the GUI description), `docs/ARCHITECTURE.md` (10b done).
- **Results:** unchanged; the processing doesn't read the progress reporter.
