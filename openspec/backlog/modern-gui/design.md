# Design

## Context

Planned on 2026-11-17 against the code at that date and parked in `openspec/backlog/`. The user's decisions of
2026-11-17:
- no settings widgets: a model may choose the settings for each part of the trajectory next year, so a flat settings
  panel would be the wrong shape; instead the trajectory gets the hooks a later window inspector needs;
- edge shading in;
- dark and light themes.

The GUI today (`src/presentation/gui/`):
- `viewer.{h,cpp}` (1059 lines, from `core`'s `utils`): freeglut window and callbacks, the camera as globals
  (`rotate_x/y`, `translate_x/y/z`, the orthographic zoom and shift, the transition targets), the Camera menu with its
  debug numbers, axes and compass drawn with `glBegin`, `gluPerspective`/`glOrtho`.
- `lidar_odometry_gui.cpp` (688 lines): the menus, the floating Progress window, the preview cloud (`PreviewCloud`,
  thinned to 0.10 m voxels, at most 3 M points, compiled into a display list), the trajectory copy, the redraw timer
  (`schedule_redraw`: `glutTimerFunc` with 100 ms during a run, 16 ms otherwise), and `run_gui`.
- ImGui 1.92.5 with docking (`MainDockSpace`), `StyleColorsDark`, the default font, the GLUT and OpenGL 2 backends.
- `GuiViewModel` owns a `ConsoleReporter` by value and hands it to the pipeline as the `Logger`.

## Goals / Non-Goals

**Goals:**
- One clean window: Run panel, viewport with toolbar, Log panel, status bar; no floating Progress window.
- Point clouds that are easy to read, caves especially: round points, a good colour scale, edge shading.
- Efficient drawing: GPU buffers, the preview growing per window, the idle cap reached with the full preview of `44`.
- Dark and light themes, a proper font, high-DPI.
- Viewer code that is testable and replaceable: a camera with no OpenGL, a renderer behind a small interface, no
  camera globals.
- Results unchanged, and GUI runs as fast as today.

**Non-Goals:**
- Settings widgets or presets; the `Settings only from TOML files` requirement stays.
- The window inspector, swim mode, measurements, editing, georeferencing.
- Settings per part of the trajectory (see "Later work").
- Any change outside `src/presentation/`, `3rdparty/`, the build files, tests and docs.

## Target Layout

Approved by the user on 2026-11-17, without a menu bar (same day: the Start, View and Camera menus only repeated the
Run panel and the toolbar; the back, left, right and bottom views, Lock Z, hiding panels and Reset layout were
dropped): `mockup/target-layout.html` (opens in any browser; theme, edge shading and its
strength work; the cloud is made up and the numbers are examples). Canvas version:
https://claude.ai/artifact/4CifpNooZMKzEuWEPqoCr5. Its colours are the starting palettes of decision 9:

| Token | Dark | Light | Used for |
|---|---|---|---|
| bg / chrome / panel / field | `#0f1216` / `#0b0e12` / `#151a20` / `#1b2129` | `#eef0f3` / `#e6e9ed` / `#ffffff` / `#f6f7f9` | window, status bar, panels, input boxes and buttons |
| border | `#262e38` | `#d6dbe1` | panel edges, separators |
| text / muted | `#e6e9ee` / `#9aa4b2` | `#1b1f24` / `#5b6472` | text, labels and secondary text |
| accent / on accent / accent soft | `#4c8dff` / `#08111f` / `#16294a` | `#1f5fd6` / `#ffffff` / `#e3ecfc` | primary button, progress, pressed toggles |
| viewport / trajectory | `#090b0e` / `#ff8a3d` | `#f7f8fa` / `#c2410c` | 3-D background, trajectory and picked node |
| ok / warn / error | `#4cc38a` / `#f2b84b` / `#ff6b6b` | `#1e7b4f` / `#9a6400` / `#c62828` | stage done, log levels |
| height scale | viridis, full range | viridis, top 20 % trimmed | point colours (decision 8 still checks it) |

Sizes: toolbar buttons 32 px, Run panel buttons 40 px, Run panel 300 px wide, Log panel 190 px high,
status bar 26 px; corner radius 6 px (toolbar) and 8 px (panel fields and buttons); Inter 13 px for text, 11 px
upper case for panel captions, JetBrains Mono 12 px for the log and the numbers.

### Can ImGui, GLFW and OpenGL 3.3 do it?

Yes: every element maps onto ImGui 1.92 (docking), the GLFW backend and the shader renderer of this change. None
needs another library. What it takes, element by element:

| Element in the mockup | How | Work |
|---|---|---|
| Fixed panels with resizable edges | a dock space built once with DockBuilder and locked (`ImGuiDockNodeFlags_NoUndocking`, no tab bars, no close buttons); the splitters still resize (decision 13) | standard |
| Toolbar and status bar fixed above and below | `BeginViewportSideBar` (ImGui's side bars around the dock space) | standard |
| Rounded buttons and fields, borders, colours per theme | `ImGuiStyle` (`FrameRounding`, `FrameBorderSize`, colours from `theme.cpp`) | standard |
| Pressed toggles (Ortho, Cloud …) and log filter pills | `Button` with pushed colours by state; pills with a larger `FrameRounding` | small helper |
| Fields with an icon and a "Load…" link | `BeginChild` with border, icon glyph from the merged icon font, `SameLine` | small helper |
| Stage list with coloured dots and a thin progress bar | draw list `AddCircleFilled`; `ProgressBar` with height 6 and rounding | small helper |
| Pause tooltip, keys in the toolbar tooltips | `SetItemTooltip` | standard |
| Strength slider | `SliderInt` | standard |
| Log with time, level and message columns, filter, copy | `BeginTable` + `ImGuiListClipper`; `SetClipboardText` | standard |
| Fonts: Inter, JetBrains Mono, icons | three embedded fonts, icons merged into Inter (decision 10) | standard |
| Upper-case captions with letter spacing | upper-case text; ImGui has no letter spacing, so captions are a little tighter than in the mockup | — |
| 3-D view inside the layout | the scene is drawn into the edge-shading framebuffer (decision 7) and shown with `ImGui::Image` in the central dock node; mouse input is taken only while that image is hovered | part of phase 4 |
| Round points, height colours, edge shading, trajectory | shaders (decisions 6 and 7) | phases 3–4 |
| Picked node marker and its label | marker in the trajectory shader; label as an ImGui window placed at the node's screen position | small |
| Compass, scale bar, colour scale over the view | `GetWindowDrawList` over the viewport image; the colour scale as `AddRectFilledMultiColor` segments | small |
| Toolbar wrapping on a narrow window | manual: before each item, compare the remaining width and start a new row | small |

Not possible as in the browser, and not needed: CSS transitions and soft shadows. The app looks flatter than the
mockup in those details only.

The helpers (toggle button, pill, icon field, stage row) go into `widgets.{h,cpp}` next to `theme.{h,cpp}`; they add
about 1–2 days to phase 5.

## Decisions

1. **Stay native: C++, ImGui, OpenGL.** No game engine, no web viewer. The GUI runs the pipeline in its own process
   and must give the CLI's result; ImGui already carries the menus, docking and dialogs. *Rejected:* a web viewer
   (Potree, three.js) needs a separate process, an export step and a server; an engine (Magnum, bgfx, Filament) is a
   large dependency for drawing points and lines.

2. **GLFW for window and input.** ImGui's main backend; key-held state, raw mouse movement, display scaling
   (`glfwGetWindowContentScale`), clean shortcut handling. Vendored as `3rdparty/glfw` (3.4) through
   `add_subdirectory` with docs, tests and examples off. *Rejected:* SDL3 (larger, brings audio and more we don't
   need); keeping freeglut (no key state, the Ctrl+letter workaround, no DPI).

3. **OpenGL 3.3 compatibility context and glad.** Asking for a compatibility context keeps the old drawing calls
   working while the renderer is replaced step by step, so phase 1 changes only the window and input, with ImGui's
   OpenGL 2 backend. Phase 3 switches to ImGui's OpenGL 3 backend and glad, generated for OpenGL 3.3 and vendored as
   source. Once the old calls are gone, nothing depends on GLU. *Rejected:* a core profile from the start (one big
   change instead of steps); Vulkan or Direct3D (much more code, no gain for points and lines).

4. **The main loop keeps the throttle exactly.** `glfwWaitEventsTimeout` until the next frame's deadline: 100 ms
   after the last frame during a run, 16 ms otherwise, as `schedule_redraw` does now. Input events may wake the loop
   early while the mouse is dragged (the spec allows it). Swap interval 1. Rationale: a slower run changes results
   through `real_time_threshold_seconds`; a busy loop would slow the run. Checked by a GUI run of `44_short` against
   the manifest and against the CLI's time (≤ 1.3×) in phases 1, 3 and 4.

5. **The camera as its own unit.** `camera.{h,cpp}` with Eigen only, built as `lo_gui_camera` (like
   `lo_gui_view_model`, no ImGui, GLFW or GL), linked by the exe and the tests. It holds the orbit state, the
   orthographic state and the transition, takes input as plain values (drag delta, wheel, keys, frame time) and
   gives the view and projection matrices. Picking: the node of the trajectory copy whose screen position is
   nearest to the click, within 8 px. The fixed views are checked against the old viewer's matrices (1e-6) before the
   old code goes. Swim mode later is one more camera.

6. **Renderer: one buffer per thing, grown not rebuilt.**
   - The preview's points are one vertex buffer of `xyz` floats with room for the 3 M-point cap (36 MB). Each window
     the preview takes in is appended with `glBufferSubData`; the rebuild after the run refills it once. No display
     list.
   - Colour by height is computed in the shader from `z` and a min/max uniform, so changing the colour scale or the
     theme uploads nothing.
   - Points are round (discard outside the circle in the fragment shader) with a size from distance
     (`gl_PointSize`), clamped; in orthographic view a constant size.
   - The trajectory is a line strip with one float per node that a colour mode maps to a colour; in this change the
     mode is "single colour per theme". The selected node gets a marker.
   - Grid, axes, compass and ruler move to small buffers drawn with one line shader.

7. **Edge shading: eye-dome lighting as a screen pass.** The scene is drawn into a framebuffer with a colour and a
   depth texture; a full-screen pass compares each pixel's log depth with 8 neighbours at a radius of one to two
   pixels and darkens it by the summed positive differences (Boucheny 2009; as in CloudCompare and Potree). In
   orthographic view the linear depth is used. Uniforms: strength and radius. Background pixels stay untouched. The
   cost depends on the screen size, not on the number of points. ImGui is drawn after the pass, unshaded.

8. **Colour scale.** One perceptually even scale for both themes (candidates: viridis, cividis), with the top end
   trimmed on the light theme so the highest points keep a contrast of at least 3:1 against the background (WCAG,
   non-text). The choice is made in phase 5 with a small check script and screenshots of `44` in both themes, and
   recorded here.

9. **Themes.** `theme.{h,cpp}` holds two palettes: ImGui style colours, the viewport background, the trajectory,
   grid and compass colours. Switching sets everything at once. `[misc] clear_color` stays in `GuiPreferences` and
   the TOML (read and saved as now, so the golden TOMLs don't change) but no longer sets the background; removing it
   is a later settings change.

10. **Font and icons, embedded.** Inter (SIL OFL 1.1) for text and Lucide (ISC) for icons, merged into one ImGui
    font and compiled into the exe as compressed arrays (`binary_to_compressed_c` from `imgui/misc/fonts`), so no
    font files ship. ImGui 1.92's dynamic font sizes plus the window's content scale give sharp text at any
    display scaling. Licences go into `3rdparty/fonts/`.

11. **Preferences in the user's settings folder.** A `gui.ini` in `%APPDATA%\dMapper\` (the matching folder on other
    systems), written through ImGui's ini file plus a settings handler `[dMapper][View]` for theme, edge shading and
    its strength, the toolbar's show switches and the two panel sizes. Not the exe folder, which may be read-only; not the working directory, which
    is where ImGui puts `imgui.ini` today.

12. **The Log panel through a tee in the view model.** A `GuiLog` (in `lo_gui_view_model`, no ImGui) implements
    `Logger`: each message goes to the `ConsoleReporter` unchanged, then into a ring buffer of 20 000 entries (level
    and text) under a mutex. `GuiViewModel` hands `GuiLog` to the pipeline instead of the `ConsoleReporter`. The
    panel copies new entries once per frame and draws them with `ImGuiListClipper`, so it costs the same with 100 or
    20 000 lines. Console output stays byte-identical. `GuiLog` gets its own unit test.

13. **A fixed layout built in code.** The arrangement is built at every start with ImGui's DockBuilder as in the
    target layout: Run panel left (300 px), Log panel below the viewport (190 px), the viewport in the central node;
    the nodes are locked (no undocking, tab bars or close buttons). Toolbar and status bar are viewport side bars.
    Only the two panel sizes are saved (decision 11). No menu bar: every action is in the Run panel or the toolbar,
    and the keys stay. *Rejected:* a free dock layout with Reset layout and hidable panels (more states to test, no
    use for one window with four parts).

14. **Run panel instead of the Progress window.** From top to bottom: Parameters, Recording (each a field with
    "Load…"), Start processing. Picking the recording only selects it (`GuiViewModel::select_recording`); Start
    processing runs it, so a second run with other parameters needs no new folder dialog. Today's "Load data and
    process…" did both in one step (user, 2026-11-17). The panel reads the same `GuiProgress` as today; it adds the stage list
    with done/running/to come and keeps it after the run with the total time. The pause warning ("You can't resume if
    you close the program") becomes a tooltip on Pause.

## Risks / Trade-offs

- **A different main loop changes run time, and through it results** → decision 4; the GUI-versus-CLI check on
  `44_short` in phases 1, 3 and 4, and a stop after phase 2 before the renderer is touched.
- **Remote desktop and old drivers** → over some remote-desktop sessions Windows gives only OpenGL 1.1 in software;
  today's fixed-function viewer may run there, the new one won't. → the GUI checks the OpenGL version at start-up and
  says plainly what is missing; the CLI is unaffected. To confirm with the user whether remote use matters.
- **GPU memory** → 36 MB for 3 M points plus the framebuffer textures; fine on any GPU with OpenGL 3.3.
- **Threads** → GLFW and OpenGL calls stay on the main thread; the worker never touches them (true today, kept).
- **Build system and lifetime changes** → phases 1 and 3 also need the `x64-Debug` build and `ctest`.
- **Size of the step** → five phases, each merged on its own; after phase 2 the GUI is already better (input, tested
  camera) even if the rest waits.

## Migration Plan

| Phase | Content | Effort | Ends with |
|---|---|---|---|
| 0 | Before starting: move out of the backlog, validate, confirm the open questions, measure today's viewer | ½ day | reference numbers |
| 1 | GLFW, ImGui GLFW + OpenGL 2 backends, the throttled loop, freeglut removed | 2–3 days | GUI vs CLI check |
| 2 | Camera module and tests, picking, the camera globals removed | 1–2 days | **checkpoint: stop and report** |
| 3 | glad, ImGui OpenGL 3 backend, buffers and shaders, display list and GLU removed | 4–6 days | speed and GUI vs CLI check |
| 4 | Edge shading | 1–2 days | screenshots of `44` |
| 5 | Theme, font, widgets, layout, Run panel, Log panel, toolbar, status bar, preferences, colour scale (as `mockup/target-layout.html`) | 5–7 days | GUI vs CLI check |
| 6 | Docs, CLAUDE.md, cleanup, Hungarian pages, sync specs, archive | 1–2 days | |

About 3.5–4 weeks in all. Rollback: each phase is its own set of commits; reverting a phase restores the previous
viewer.

## Later Work

- **Window inspector** (about 3–5 days): click a node, see its window's motion (acceleration, turn rate), the window
  verdict's flags and readings, and later the settings a model chose; colour the trajectory by any per-window value.
  Uses the picking (decision 5) and the per-node value (decision 6). Plots along the trajectory would need implot
  vendored again (it went with `core`).
- **Settings per part of the trajectory** is not a GUI change: `OdometryConfig` is one set of settings per run, and
  the snapshot check, the settings hash and the golden TOMLs rely on that. It needs its own change in the domain.
- **Swim mode:** a third camera in `lo_gui_camera` plus fog in the point shader.

## Open Questions

To answer when the change leaves the backlog:
1. Does the GUI have to work over remote desktop? (Risk above.)
2. Preferences in `%APPDATA%\dMapper\gui.ini`: fine?
3. Colour scale: decided in phase 5 (decision 8); any preference beforehand?
