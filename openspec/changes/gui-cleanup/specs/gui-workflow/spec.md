# Spec Delta

## Purpose

Defines what the GUI offers for processing a recording: a short Start workflow, settings that come only from TOML files, and a view of the trajectory. The app concentrates on mapping; georeferencing and editing are left to other tools.

## ADDED Requirements

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
The menu bar SHALL contain Start, View, Camera and Info. The View menu SHALL offer Show trajectory, Show compass/ruler and Lock Z. The Camera menu (fixed views and orthographic projection, from `core`) stays as it is. The progress window with Pause and Resume SHALL remain.

The background colour SHALL come from the `[misc] clear_color` setting of the loaded parameters, with no colour picker in the GUI.

#### Scenario: Menu bar
- **WHEN** the GUI has started
- **THEN** the menu bar shows Start, View, Camera and Info, and nothing else

#### Scenario: Background colour from the file
- **WHEN** the user loads a TOML whose `[misc] clear_color` differs from the current one
- **THEN** the view background changes to that colour
