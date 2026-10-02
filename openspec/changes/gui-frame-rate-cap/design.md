# Design

## Context

- **`display()`** (`app/lidar_odometry_gui.cpp`) draws the 3D view (the trajectory of every window, the compass) and the ImGui windows (menu bar, progress window, `cor_window`). It ends with `glutSwapBuffers(); glutPostRedisplay();`, so GLUT calls it again at once. There is no idle or timer callback.
- **Input:** `core`'s `motion()` calls `glutPostRedisplay()` on mouse moves. Clicks, the wheel and keyboard shortcuts rely on the continuous loop to become visible. Camera transitions (Camera menu views) animate frame by frame.
- **Processing** runs on the worker thread started by `openData` (`loRunning` is set during the run). With `useMultithread = false` it uses one core. The redraw loop takes the main thread and the GL driver.
- **Measured:** GUI `44_short` runs took 510–570 s against 197 s for the CLI (`elapsed_time_seconds` in the results JSON of `lio_result_5`–`7` and the baseline).

## Goals / Non-Goals

**Goals:**
- GUI runs about as fast as the CLI (target: at most 1.3× on `44_short`), so the GUI result matches the CLI.
- No visible change to how the GUI looks or works, apart from a lower redraw rate during runs.

**Non-Goals:**
- Changing `real_time_threshold_seconds` or the time limit itself: a separate decision, noted.
- Faster drawing of the trajectory (e.g. vertex buffers): not needed at 10 frames per second.
- Changes in `core` (the `motion()` redraw stays).

## Decisions

1. **The next frame is scheduled with a timer instead of being requested at once.** `display()` ends with `schedule_redraw()`:
   ```cpp
   bool redraw_scheduled = false;
   void redraw_timer(int) { redraw_scheduled = false; glutPostRedisplay(); }
   void schedule_redraw()
   {
       if (redraw_scheduled)
           return;
       redraw_scheduled = true;
       glutTimerFunc(loRunning ? 100 : 16, redraw_timer, 0); // ~10 fps during a run, ~60 fps otherwise
   }
   ```
   The flag keeps the timers from piling up when a mouse move has already requested a frame. GLUT callbacks all run on the main thread, so the flag needs no lock.
   - *Alternative:* `glutIdleFunc` with a sleep. Rejected: it blocks the main loop, including input handling, during the sleep.
   - *Alternative:* vsync (`wglSwapIntervalEXT`). Rejected: it depends on the driver and the monitor, and still redraws at 60 Hz or more during a run.
2. **The rates are constants** in the GUI file (100 ms during runs, 16 ms otherwise), not TOML settings. The user decided that TOML holds processing settings; the redraw rate is not one, and changing these constants needs no user choice.
3. **Verification:**
   - the CLI `44_short` check proves processing is untouched;
   - the GUI timing and the identical result can only be checked by the user running the GUI. I read the elapsed time from the results JSON and compare the files.

## Risks / Trade-offs

- **[Input lag during runs]:** up to about 100 ms before a click or key shows. → Accepted. The progress window's Pause/Resume reacts within one frame.
- **[The GUI is still slower for another reason]**, for example the GL driver or the trajectory drawing. → The measurement shows it. If the GUI is still more than 1.3× slower, the next step is profiling, not raising the cap further.
- **[Load from other programs]:** the 10 s time limit still makes a run on a busy computer differ. That is outside this change and was recorded as a finding.

## Migration Plan

One commit for the timer, built in both presets with `ctest`, the `44_short` CLI check, then the user's GUI measurement. Rollback: revert.
