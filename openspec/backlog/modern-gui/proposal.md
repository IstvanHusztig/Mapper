# Proposal

## Why

The GUI works, but it looks and feels like a debugging tool, and that makes results harder to judge than they need to
be:
- Window and input come from freeglut: there is no "is this key held" state, Ctrl+letter needs a workaround in
  `keyboardDown`, and high-DPI screens are not handled.
- The view draws with fixed-function OpenGL: the whole preview is compiled into a display list, points are square
  pixels with no depth cue, and the trajectory, compass and axes are drawn point by point with `glBegin`.
- Cave walls coloured by height alone look flat; ledges, passage shapes and doubled walls are hard to see.
- Progress is a floating window that says "Also check console for progress": the log is only in the console.
- The Camera menu mixes the fixed views with raw debug numbers (rotation, translation, mouse sensitivity).
- The camera and view state are loose globals in `viewer.h`, taken over from `core`.

A clean, modern and efficient viewer is also the base for later work: a window inspector, a swim mode, and showing
the settings a model chooses for each part of the trajectory.

## What Changes

- **Window and input on GLFW** instead of freeglut, with ImGui's GLFW backend (already in
  `3rdparty/imgui/backends`). freeglut leaves `3rdparty/`. The redraw throttle (about 10 frames per second during a
  run, 60 otherwise) is kept exactly.
- **A camera module**: orbit, orthographic top-down and the smooth transitions move out of the `viewer.{h,cpp}`
  globals into a camera with no OpenGL, unit-tested without a display. It can find the trajectory node nearest to a
  click.
- **A shader renderer**: a vendored OpenGL loader (glad) and ImGui's OpenGL 3 backend. Points, trajectory, grid,
  axes and compass are drawn from GPU buffers by small GLSL programs:
  - colour by height with a scale that reads on dark and light backgrounds;
  - round points sized by distance;
  - the preview grows window by window instead of being rebuilt as a whole;
  - the trajectory carries one value per node that sets its colour (a hook for a later window inspector).
- **Edge shading** (eye-dome lighting) as a screen pass after the points, on by default, with a switch and a
  strength slider.
- **One fixed layout, no menu bar** (every action is in the Run panel or the toolbar; the keys stay):
  - a Run panel: Parameters and Recording, each with "Load…", then Start processing (picking a folder no longer starts
    the run), Pause/Resume, and the list of stages with progress and estimate;
  - the 3-D viewport with a toolbar: fixed views, orthographic, show cloud/trajectory/compass, edge shading;
  - a Log panel with the run's messages, filtered by level (the console still gets every line);
  - a status bar: points shown, frame rate, camera position, picked node, parameter file, version.

  The menu bar, the floating Progress window and the Camera menu's debug numbers go, and so do the back, left,
  right and bottom views and Lock Z, with their keys.
- **The approved target layout:** `mockup/target-layout.html` (2026-11-17) shows the layout, both themes and the
  colours; `design.md` → "Target Layout" maps every element onto ImGui, GLFW and OpenGL 3.3.
- **Dark and light themes**, an embedded UI font with icons, and scaling for high-DPI screens. Theme, panel sizes and view
  options are remembered between sessions in the GUI's own preferences file, not in the run TOML.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `gui-workflow`: no menu bar: the Start actions move to the Run panel and the view controls to the toolbar (both
  requirements renamed and rewritten); the background colour and active-file requirements are rewritten; new
  requirements for the layout, themes, point and trajectory drawing, edge shading,
  picking a node, the viewer's speed and remembered view options.
- `run-progress`: the stages are shown in the Run panel instead of a progress window; the GUI shows the run's log in a
  panel, still without writing a log file.
- `unit-tests`: the camera is tested without a display.

## Impact

- **Code:** `src/presentation/gui/` (the viewer split into camera, renderer, layout and theme;
  `lidar_odometry_gui.cpp`); a presentation-layer `Logger` adapter that forwards to `ConsoleReporter` and keeps the
  lines for the Log panel; `CMakeLists.txt`, `cmake/imgui.cmake`; camera tests. `GuiViewModel` gains no graphics,
  windowing or ImGui code. No file outside `src/presentation/`, `3rdparty/`, the build files, tests and docs changes.
- **3rdparty:** add GLFW, glad (generated for OpenGL 3.3), a UI font and an icon font with their licences; remove
  freeglut. GLU is no longer needed.
- **Results:** none. Processing code is untouched; GUI runs of `44_short` stay identical with the manifest and within
  1.3× the CLI's time.
- **Settings:** none. `[misc] clear_color` is still read and written (no golden TOML change), but no longer sets the
  background.
- **Docs:** the GUI section of `docs/index.html`, the `clear_color` row of its settings table, CLAUDE.md's GUI and
  layer notes; the Hungarian pages at the end.
- **Not in this change:** settings widgets, the window inspector, swim mode, settings per part of the trajectory (a
  domain change; see design, "Later work").
