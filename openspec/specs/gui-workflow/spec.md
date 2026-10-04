# gui-workflow Specification

## Purpose
Defines what the GUI offers for processing a recording: a short Start workflow, settings that come only from TOML files, and a view of the trajectory. The app concentrates on mapping; georeferencing and editing are left to other tools.

## Requirements

### Requirement: Start menu workflow
The GUI SHALL offer a Start menu with two actions, in this order:
1. "Load parameters…", which loads a TOML parameter file;
2. "Load data and process…", which picks a recording folder, then loads, processes and saves the result in one go. It is also available with Ctrl+O.

Processing SHALL be possible without loading parameters first, using the start-up settings. The GUI SHALL NOT offer another way to start processing.

#### Scenario: Parameters, then data
- **WHEN** the user loads `settings/lidar-vertical.toml` through Start → Load parameters, then processes `44_short` through Start → Load data and process
- **THEN** the result folder is identical to the CLI's result for `44_short` with that file

#### Scenario: No step-by-step processing
- **WHEN** the user looks for a way to load data without processing it, or to process loaded data again
- **THEN** there is none: loading and processing happen together

### Requirement: Settings only from TOML files
The GUI SHALL NOT offer widgets, menus or presets that change processing settings. The settings SHALL be the built-in defaults, overlaid by `default_settings.toml` from the exe folder at start-up, and then by each TOML file the user loads.

#### Scenario: Convention flags are kept
- **WHEN** a loaded TOML sets `fusionConventionEnu = true`
- **THEN** that value is used for processing, and no window resets it

### Requirement: Active parameter file is shown
The GUI SHALL show which parameter file the current settings came from, in the Start menu and in the progress window while a run is going. It SHALL show the start-up `default_settings.toml` if no other file was loaded, the last loaded file after Load parameters, or "built-in defaults" if no file was loaded at all.

#### Scenario: After loading a file
- **WHEN** the user loads `lidar-vertical.toml` and then starts processing
- **THEN** the Start menu and the progress window both name `lidar-vertical.toml`

### Requirement: No georeferencing in the GUI
The GUI SHALL NOT offer loading reference point clouds, placing the start pose by hand, or aligning the recording to a reference. Georeferencing is left to a separate post-processing step on result folders.

#### Scenario: Menus and windows
- **WHEN** the user opens every menu and window of the GUI
- **THEN** there is no reference-cloud loading and no "Align to reference" action

### Requirement: No result editing or partial export in the GUI
The GUI SHALL NOT offer selecting ranges of windows, exporting parts of a result, writing RESSO pose files, or moving parts of the trajectory by hand. The result folder written by a run is the GUI's only output.

#### Scenario: After a run
- **WHEN** a run has finished
- **THEN** the GUI shows the trajectory, and there is no export or editing action

### Requirement: Remaining view options
The menu bar SHALL contain Start, View and Camera; there is no Info button or Info window. The View menu SHALL offer Show point cloud, Show trajectory, Show compass/ruler and Lock Z. The Camera menu (fixed views and orthographic projection, from `core`) stays as it is. The progress window with Pause and Resume SHALL remain.

The background colour SHALL come from the `[misc] clear_color` setting of the loaded parameters, with no colour picker in the GUI.

#### Scenario: Menu bar
- **WHEN** the GUI has started
- **THEN** the menu bar shows Start, View and Camera, and nothing else

#### Scenario: Background colour from the file
- **WHEN** the user loads a TOML whose `[misc] clear_color` differs from the current one
- **THEN** the view background changes to that colour

### Requirement: Application name and window title
The application SHALL be named dMapper. The executable SHALL be `dMapper` (`dMapper.exe` on Windows), and the GUI window title SHALL be exactly "dMapper", without a version number or folder name. The GUI's texts (dialogs, tooltips) SHALL NOT refer to other products' workflows, such as Mandeye or HDMapping's registration steps.

#### Scenario: Window title during a run
- **WHEN** the GUI has started and a recording folder has been picked and is being processed
- **THEN** the window title still reads exactly "dMapper"

#### Scenario: Folder dialog
- **WHEN** the user chooses Start → Load data and process
- **THEN** the folder dialog asks for a recording folder (lidar*.laz + imu*.csv) and does not mention Mandeye

### Requirement: Limited redraw rate
The GUI SHALL limit how often it redraws the window: about 10 times per second while a run is going, and about 60 times per second otherwise. Mouse dragging MAY redraw more often. Processing in the GUI SHALL therefore take about as long as the same run through the CLI, and its result SHALL be identical to the CLI's.

#### Scenario: GUI run of the short reference dataset
- **WHEN** the GUI processes `44_short` with `settings/lidar-vertical.toml` loaded, on an otherwise idle computer
- **THEN** the elapsed time recorded in the results JSON is at most 1.3 times the CLI's elapsed time for the same run, and every output file compares identical with the `44_short` manifest

#### Scenario: Idle window
- **WHEN** no run is going and the user doesn't interact with the window
- **THEN** the GUI redraws at most about 60 times per second

### Requirement: Live point cloud preview
While a run's odometry is going, the GUI SHALL show a thinned point cloud of every window the odometry has finished, placed with that window's optimized trajectory and coloured by height. When the run has finished, the GUI SHALL rebuild the preview once from the final, post-processed trajectory, so it matches the saved result. Building and drawing the preview SHALL NOT change the processing or its results, and SHALL NOT noticeably slow the run.

#### Scenario: Cloud grows during a run
- **WHEN** the user processes `44_short` in the GUI
- **THEN** the view shows the point cloud growing along the trajectory while the odometry runs

#### Scenario: Final map after post-processing
- **WHEN** a run with loop closure has finished
- **THEN** the preview is rebuilt from the corrected trajectory

#### Scenario: Results unaffected
- **WHEN** the GUI processes `44_short` with `settings/lidar-vertical.toml` with the preview shown
- **THEN** every output file compares identical with the `44_short` manifest, and the run time stays within 1.3 times the CLI's

### Requirement: Trajectory shown during a run
While a run's odometry is going, the GUI SHALL draw the trajectory of the windows the odometry has finished only, with the cross at the last finished pose. During loop closure, consistency and saving it SHALL keep showing that trajectory. When the run has finished, the GUI SHALL show the final, post-processed trajectory. The GUI SHALL NOT read a window's poses while the run may still change them, and drawing SHALL NOT change the processing or its results.

#### Scenario: Trajectory grows with the processing
- **WHEN** the user processes `44_short` in the GUI
- **THEN** the drawn trajectory grows window by window with the odometry, and the cross is at the end of the processed part, not at the end of the recording

#### Scenario: Final trajectory after post-processing
- **WHEN** a run with loop closure has finished
- **THEN** the drawn trajectory is the corrected one, as saved

#### Scenario: Results unaffected
- **WHEN** the GUI processes `44_short` with `settings/lidar-vertical.toml`
- **THEN** every output file compares identical with the `44_short` manifest
