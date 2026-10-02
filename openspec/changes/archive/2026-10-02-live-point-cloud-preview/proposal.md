# Proposal

## Why

During a run the GUI shows only the trajectory growing. The old green "initial points" cloud was removed in the GUI cleanup; it only ever showed the first 1–2 seconds of the scan. The user wants to see the point cloud of the whole run building up while it is processed, and the final map when the run ends, without opening the result in another program (decided 2026-10-02).

## What Changes

- **New live point cloud preview in the 3D view:**
  - **During the odometry stage,** every window the odometry has finished is added to the preview. Its intermediate (already decimated) points come from the point store, are placed with that window's optimized trajectory, and are thinned once more to a display voxel (0.10 m).
  - **When the run has finished,** after loop closure and consistency have corrected the trajectory, the preview is rebuilt once from the final trajectory, so it shows the map that was saved.
  - **Colour:** points are coloured by height (blue low → red high, across the preview's height range), which reads well in caves with several levels.
- **View → Show point cloud** (on by default) switches the preview on and off.
- **The preview is built and drawn only by the GUI.** The processing is not changed: it reads data the pipeline already keeps (the window's points in the point store, `worker_data[i].intermediate_trajectory`) for windows the progress reporter has reported as done. `OdometryPipeline` gets a read-only accessor for a window's stored points.
- **Memory and speed are bounded:**
  - at most a few windows are added per frame;
  - the display voxel keeps the preview to roughly a few hundred thousand points even on the full 44 recording;
  - drawing happens at the capped redraw rate (10 frames per second during a run).
- **Unchanged:** processing, outputs and the CLI. A GUI run stays as fast as the CLI and identical to the baseline.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- `gui-workflow`:
  - adds "Live point cloud preview";
  - "Remaining view options" gains Show point cloud in the View menu.

## Impact

- **Code:**
  - `app/lidar_odometry_gui.cpp`: the preview builder and drawing, View item, and `GuiProgress` remembering how many windows are done;
  - `app/odometry_pipeline.{h,cpp}`: a const accessor `load_window_points(i, points)` through the run's `PointStore`.
- **Not touched:** the processing functions, `core/`, the output files, the TOML format.
- **Validation:**
  - the `44_short` CLI check stays identical;
  - the user runs `44_short` in the GUI: the cloud grows during the run, is rebuilt at the end, and the result is identical with a run time like the CLI's;
  - optionally a full 44 GUI run, to check memory and smoothness on the long recording.
