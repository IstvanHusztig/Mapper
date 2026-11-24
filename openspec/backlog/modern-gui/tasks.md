# Tasks

**Step checks, on every commit:**
- the `x64-Release` build;
- the unit tests (`ctest`);
- for phases 1, 3 and 5 also the `x64-Debug` build and `ctest` (build-system, threading and lifetime changes);
- no file in `src/domain`, `src/application` or `src/infrastructure` changed (`git diff --stat`).

**GUI run check** (end of phases 1, 3 and 5, and the checkpoint after phase 2): load `settings/l2-vertical.toml`,
process `44_short` in the GUI on an otherwise idle computer; every output file identical with the `44_short` manifest
(`tools/compare_baseline.py`), and the elapsed time in the results JSON at most 1.3 × the CLI's for the same run.

## 0. Before starting

- [ ] 0.1 `git mv openspec/backlog/modern-gui openspec/changes/modern-gui`; remove its line from
  `openspec/backlog/README.md`; `openspec validate modern-gui`. Verify: valid, or the CLI's findings fixed (the files
  were written without the CLI).
- [ ] 0.2 Answer the open questions at the end of `design.md` with the user; write the answers there. Re-read
  `viewer.cpp`, `lidar_odometry_gui.cpp` and `gui_view_model.h` for changes since 2026-11-17.
- [ ] 0.3 Measure today's viewer as the reference: the frame rate with the full preview of `44` at the isometric view,
  idle and while rotating; the GUI vs CLI time of `44_short` (three runs each). Verify: the numbers are in this file.
- [ ] 0.4 Record the view and projection matrices of every fixed view and of three orbit/pan/zoom states from the old
  viewer (a temporary print). Verify: the file is in `tests/data/` for task 2.1.

## 1. Window and input on GLFW

- [ ] 1.1 Vendor GLFW 3.4 as `3rdparty/glfw` (docs, tests, examples off); add it to `CMakeLists.txt`. Verify: builds
  in both presets.
- [ ] 1.2 Replace `initGL` and the GLUT callbacks with GLFW: window with an OpenGL 3.3 compatibility context, mouse,
  wheel, keys and resizing; ImGui's GLFW and OpenGL 2 backends in `cmake/imgui.cmake`. Remove the Ctrl+letter
  workaround. Verify: every key works as before (B, L, R, U and Shift+Z go in task 2.2); Ctrl+O opens the folder dialog.
- [ ] 1.3 The main loop with the throttle of design decision 4. Verify: idle frame rate about 60 and about 10 during a
  run (frame counter in the log at debug level, removed afterwards); no CPU core busy while idle.
- [ ] 1.4 Remove `3rdparty/freeglut` and every reference to it. Verify: `grep -ri glut src cmake CMakeLists.txt` finds
  nothing.
- [ ] 1.5 GUI run check.

## 2. Camera module

- [ ] 2.1 `src/presentation/gui/camera.{h,cpp}` as `lo_gui_camera` (Eigen only): orbit, pan, zoom, the orthographic
  view, the fixed views, the transition; `tests/camera_test.cpp`. Verify: the matrices of task 0.4 match to 1e-6.
- [ ] 2.2 The viewer uses the camera; the camera globals in `viewer.h` go; the back, left, right and bottom views and
  Lock Z are removed with their keys and menu entries. Verify: `grep` finds no `rotate_x`,
  `translate_z` or `camera_ortho_` outside `camera.*`; the view behaves as before (manual check of every remaining fixed
  view, drag, pan, Shift + wheel); B, L, R, U and Shift+Z do nothing.
- [ ] 2.3 Picking: nearest trajectory node within 8 px, a marker, its time and window number on the status line
  (a plain ImGui text line until phase 5). Test on a synthetic trajectory. Verify: the test passes; clicking on
  `44_short`'s trajectory shows plausible times.
- [ ] 2.4 **Checkpoint:** GUI run check; report the state to the user and wait before phase 3.

## 3. Shader renderer

- [ ] 3.1 Vendor glad (OpenGL 3.3, compatibility) as source; switch ImGui to its OpenGL 3 backend. Check the OpenGL
  version at start-up and refuse with a clear message below 3.3. Verify: builds; the GUI looks as before.
- [ ] 3.2 Point buffer for the preview with room for the 3 M-point cap, appended per window, refilled once after the
  run; shaders for colour by height (min/max uniform), round points, size by distance. Remove the display list.
  Verify: the preview grows during a run of `44_short`; the time to add a window does not grow with the cloud (debug
  timing over the run).
- [ ] 3.3 Trajectory as a line strip with a per-node value and a colour mode (single colour for now); grid, axes,
  compass and ruler in buffers with one line shader. Remove the remaining `glBegin` code and GLU. Verify:
  `grep -E "glBegin|glu[A-Z]" src` finds nothing.
- [ ] 3.4 Speed: frame rate with the full preview of `44`, idle and while rotating. Verify: at the idle cap, and not
  below the numbers of task 0.3.
- [ ] 3.5 GUI run check.

## 4. Edge shading

- [ ] 4.1 Framebuffer with colour and depth textures; the eye-dome lighting pass (design decision 7); strength and
  radius uniforms; the result shown with `ImGui::Image` in the central dock node, mouse input only while it is hovered. Verify: on and off look as specified on a passage of `44`.
- [ ] 4.2 Orthographic view with linear depth. Verify: the top view of `44` shows the walls' edges.
- [ ] 4.3 Toolbar switch and strength slider (a temporary menu entry until phase 5); frame rate with it on.
  Verify: still at the idle cap with the full preview of `44`.
- [ ] 4.4 Screenshots of three places in `44` with it on and off, for the user (into `reports/modern-gui/`).

## 5. Layout and look

- [ ] 5.1 `theme.{h,cpp}`: dark and light palettes for ImGui, background, trajectory, grid and compass, from the table
  in `design.md` → "Target Layout"; switch in the toolbar; `clear_color` no longer applied. Verify: switching changes everything at once; a loaded TOML's
  `clear_color` changes nothing; the golden TOML tests pass unchanged.
- [ ] 5.2 Inter and Lucide embedded as compressed arrays, licences in `3rdparty/fonts/`; scaling from the window's
  content scale, also when the window moves to another screen. Verify: sharp at 100 % and 150 %.
- [ ] 5.3 Colour scale (design decision 8): a small check script for the contrast against both backgrounds, then
  screenshots of `44` in both themes. Verify: the choice and the contrast numbers are in `design.md`.
- [ ] 5.4 `GuiLog` (design decision 12) with a unit test (order, levels, ring-buffer wrap, console output unchanged);
  `GuiViewModel` hands it to the pipeline. Verify: tests pass; the console output of a GUI run is byte-identical with
  that before the change (timestamps apart).
- [ ] 5.4a `GuiViewModel`: `select_recording(folder)` and `selected_recording()`; `start_run()` runs the selected
  recording; Ctrl+O picks the recording only. Tests in `tests/gui_view_model_test.cpp`: no run without a selection,
  the selection kept after a run, a second run with other parameters. Verify: tests pass.
- [ ] 5.5a `widgets.{h,cpp}`: toggle button, pill, field with icon, stage row (design → "Can ImGui, GLFW and OpenGL
  3.3 do it?"). Verify: side by side with `mockup/target-layout.html` in both themes.
- [ ] 5.5 Layout as `mockup/target-layout.html`: the menu bar removed, the locked DockBuilder layout (design
  decision 13), Run panel (Parameters and Recording, each with "Load…", then Start processing; Pause/Resume
  with the warning as tooltip, stage list), Log panel (filter, copy, clipper), toolbar, status bar; the keys in the toolbar tooltips;
  remove the Progress window. Verify: the scenarios of
  `gui-workflow` and `run-progress` by hand; screenshots of the GUI in both themes next to the mockup's, into
  `reports/modern-gui/`.
- [ ] 5.6 Preferences in `gui.ini` in the user's settings folder (design decision 11). Verify: the `View options are
  remembered` scenarios; deleting the file restores the defaults.
- [ ] 5.7 GUI run check, with the light theme and the Log panel open.

## 6. Docs and cleanup

- [ ] 6.1 Remaining view globals into one `ViewerState`; `viewer.{h,cpp}` split so no file is over about 500 lines.
  Verify: builds; nothing behaves differently.
- [ ] 6.2 `docs/index.html`: the GUI section (layout, themes, edge shading, picking, preferences file) with
  screenshots; the `clear_color` row says it no longer sets the background. CLAUDE.md: GUI section, layer notes
  (`lo_gui_camera`, `GuiLog`), the vendored libraries list. Verify: the docs' top line, terms line and "See also" are
  intact.
- [ ] 6.3 The Hungarian pages (`docs/hu/`), at the end of the work.
- [ ] 6.4 Sync the specs (`gui-workflow`, `run-progress`, `unit-tests`) and archive the change.
