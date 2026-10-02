# Proposal

## Why

A GUI run of `44_short` takes 510–570 s, while the CLI takes about 197 s with the same `dMapper.exe` and settings. The GUI window redraws without pause: `display()` ends with `glutPostRedisplay()`, so the next frame is requested at once, hundreds of times per second. Every frame redraws the whole trajectory and the ImGui windows. This competes with the processing thread.

The slowdown also changes results. Each window's optimisation stops after `real_time_threshold_seconds = 10` s, so a slower GUI run stops some windows earlier than the CLI:
- `lio_result_5` differed from the CLI by 1–2 mm;
- `lio_result_7` differed by 9e-9 m.

That breaks the `processing-pipeline` requirement "GUI and CLI results are identical" whenever the GUI is slow. The user asked for a frame-rate cap during runs (2026-10-02).

## What Changes

- **The GUI redraw rate is limited:**
  - about **10 frames per second while a run is going** (the progress window and the growing trajectory don't need more);
  - about **60 frames per second otherwise**, so camera moves and keyboard shortcuts stay smooth;
  - mouse dragging still redraws on each mouse move (`core` already requests that).
- **Implementation:** one scheduled redraw through `glutTimerFunc` replaces the unconditional `glutPostRedisplay()` at the end of `display()`.
- **Measured before and after** on `44_short` by the user (GUI time against CLI time), and the GUI result compared with the baseline.
- **Unchanged:** processing, outputs, the CLI, the menus.

## Capabilities

### New Capabilities
- None.

### Modified Capabilities
- `gui-workflow`: adds a requirement that the GUI limits its redraw rate, so a GUI run takes about as long as the CLI and gives the same result.

## Impact

- **Code:** `app/lidar_odometry_gui.cpp` (end of `display()`, a small timer callback). Nothing in `core/`.
- **Validation:**
  - `44_short` CLI identical (the processing is untouched);
  - the user runs `44_short` in the GUI; I compare the elapsed time (from the results JSON) with the CLI's, and the result with the baseline.
- **Risk:** a little more input lag during a run (up to about 0.1 s for clicks and keys). Accepted.
