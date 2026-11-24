## RENAMED Requirements

- FROM: `### Requirement: Start menu workflow`
- TO: `### Requirement: Start actions
The Run panel SHALL offer, from top to bottom:
1. **Parameters** with "Load…", which loads a TOML parameter file;
2. **Recording** with "Load…", which picks a recording folder (lidar*.laz + imu*.csv) and shows it; it is also
   available with Ctrl+O. Picking a folder SHALL only select it: nothing is read or processed yet;
3. **Start processing**, which loads the selected recording, processes it and saves the result in one go.

Start processing SHALL be disabled until a recording is selected, and while a run is going; both "Load…" actions SHALL
be disabled while a run is going. Processing SHALL be possible without loading parameters first, using the start-up
settings. After a run the recording SHALL stay selected, so the same recording can be processed again, for example
with other parameters; each run loads it from the folder again. The GUI SHALL NOT offer another way to start
processing.

#### Scenario: Parameters, recording, start
- **WHEN** the user loads `settings/l2-vertical.toml` with the Parameters "Load…", picks `44_short` with the Recording "Load…", then presses Start processing
- **THEN** the result folder is identical to the CLI's result for `44_short` with that file

#### Scenario: Picking a folder does not process it
- **WHEN** the user picks a recording folder
- **THEN** the Run panel shows the folder, the stage list stays empty, and nothing starts until Start processing is pressed

#### Scenario: Same recording, other parameters
- **WHEN** a run of `44_short` has finished and the user loads another parameter file and presses Start processing
- **THEN** a second run of `44_short` starts with the new parameters and saves its own result folder

#### Scenario: No step-by-step processing
- **WHEN** the user looks for a way to load data without processing it, or to process data loaded by an earlier run again
- **THEN** there is none: each run loads and processes together

### Requirement: Active parameter file is shown
The GUI SHALL show which parameter file the current settings came from, in the Run panel and in the status bar. It SHALL show the start-up `default_settings.toml` if no other file was loaded, the last loaded file after "Load…", or "built-in defaults" if no file was loaded at all. While a run is going, both SHALL show the file the run was started with.

#### Scenario: After loading a file
- **WHEN** the user loads `l2-vertical.toml` and then starts processing
- **THEN** the Run panel and the status bar both name `l2-vertical.toml`

### Requirement: View controls
The GUI SHALL have no menu bar. Its view controls SHALL be in the toolbar above the 3-D view:
- the fixed views Top, Front and Isometric, and Reset;
- the orthographic projection;
- Show point cloud, Show trajectory and Show compass/ruler;
- Edge shading and its strength;
- the theme (Dark, Light).

The back, left, right and bottom views and Lock Z SHALL be removed, with their keys (B, L, R, U, Shift+Z). Every other key that works today SHALL keep working, among them T, F, I, Z and C; Ctrl+O SHALL pick the recording (see `Start actions`). A toolbar button that has a key SHALL name it in its tooltip.

The background of the 3-D view SHALL come from the theme. The `[misc] clear_color` setting SHALL still be read and saved, but SHALL NOT change the background.

#### Scenario: No menu bar
- **WHEN** the GUI has started
- **THEN** there is no menu bar: the Run panel, the toolbar and the 3-D view start at the top of the window

#### Scenario: Key named in the tooltip
- **WHEN** the user rests the mouse on the toolbar's Top button
- **THEN** the tooltip names the key T

#### Scenario: Background follows the theme
- **WHEN** the user switches from the dark to the light theme
- **THEN** the 3-D view's background changes from dark to light at once, and loading a TOML with another `clear_color` does not change it

## ADDED Requirements

### Requirement: Window layout
The GUI SHALL show one window with a Run panel on the left, the 3-D view with a toolbar above it, a Log panel below it, and a status bar at the bottom, as in `mockup/target-layout.html` of the `modern-gui` change.
- **Run panel:** the Start actions (see `Start actions`), Pause/Resume, and the run's stages (see `run-progress`).
- **Toolbar:** see `View controls`.
- **Log panel:** see `run-progress`.
- **Status bar:** the number of points shown, the frame rate, the camera position, the picked trajectory node, the active parameter file and dMapper's version.

The panels SHALL always be shown and SHALL NOT be moved, undocked or closed; the user MAY resize the Run panel's width and the Log panel's height by dragging their edges. The GUI SHALL NOT open floating windows during a normal run, apart from file dialogs and message boxes.

#### Scenario: Start
- **WHEN** the GUI starts
- **THEN** the Run panel is on the left, the Log panel below the 3-D view, the toolbar above it and the status bar at the bottom

#### Scenario: Resizing the Log panel
- **WHEN** the user drags the top edge of the Log panel upwards
- **THEN** the Log panel grows and the 3-D view shrinks; nothing else moves

### Requirement: Dark and light themes
The GUI SHALL offer a dark and a light theme, switched in the toolbar without a restart. A theme SHALL set the colours of all panels, the toolbar, the status bar, the icons and the 3-D view's background. Text, icons and the trajectory SHALL be readable in both themes. The GUI SHALL use an embedded font with icons, and SHALL scale text and panels to the screen's display scaling.

#### Scenario: Switching theme during a run
- **WHEN** the user switches to the light theme while the odometry runs
- **THEN** the whole window changes at once, the run continues, and its result is identical with the `44_short` manifest

#### Scenario: High-DPI screen
- **WHEN** the GUI starts on a screen with 150 % display scaling
- **THEN** text and panels are drawn at 150 % size and are sharp

### Requirement: Point and trajectory drawing
The 3-D view SHALL draw points as round dots whose size shrinks with distance, coloured by height with a colour scale that reads on both the dark and the light background. While a run's odometry is going, adding a finished window to the preview SHALL NOT take longer as the cloud grows. The trajectory SHALL be drawn as a line coloured by one value per node; in this change that value is constant, so the line has one colour per theme.

#### Scenario: Height colours on a light background
- **WHEN** the full preview of `44` is shown in the light theme
- **THEN** the highest points are still clearly visible against the background

### Requirement: Edge shading
The 3-D view SHALL offer edge shading (eye-dome lighting): edges and depth steps in the point cloud are darkened so that surfaces and their shapes are visible without normals. It SHALL be on by default, SHALL work in the perspective and the orthographic view, and SHALL have a switch and a strength setting in the toolbar. Switching it SHALL NOT change the processing or its results.

#### Scenario: Cave passage
- **WHEN** the user looks at a passage of `44` with edge shading on and then off
- **THEN** with it on, wall edges, ledges and the step between floor and wall are visibly darker; with it off, the view looks as without this capability

### Requirement: Picking a trajectory node
Clicking in the 3-D view near the trajectory SHALL select the nearest node within a few pixels of the click. The status bar SHALL show the selected node's time from the first window's start and its window number. Clicking away from the trajectory SHALL clear the selection. Picking SHALL NOT change the camera, the processing or its results.

#### Scenario: Click on the trajectory
- **WHEN** the user clicks on the trajectory of a finished run
- **THEN** the status bar shows the time and window number of the node under the cursor, and a marker shows it in the view

### Requirement: Viewer speed
With the full preview of `44` shown and edge shading on, the GUI SHALL redraw at its idle cap (about 60 frames per second) on the development computer, and SHALL NOT be slower than the viewer it replaces at the same view.

#### Scenario: Full preview, idle
- **WHEN** a run of `44` has finished and the user rotates the view with edge shading on
- **THEN** the status bar shows about 60 frames per second

### Requirement: View options are remembered
The GUI SHALL remember, between sessions, the theme, edge shading and its strength, the toolbar's show switches, and the sizes of the Run and Log panels. It SHALL keep them in its own preferences file in the user's settings folder, never in a parameter TOML or in a result folder. Without the file, the GUI SHALL start with the defaults.

#### Scenario: Restart
- **WHEN** the user picks the light theme, turns edge shading off, closes the GUI and starts it again
- **THEN** the GUI starts in the light theme with edge shading off

#### Scenario: Results do not carry view options
- **WHEN** the user processes `44_short` after changing the theme and the panel sizes
- **THEN** the result folder holds the same files with the same contents as before the change
