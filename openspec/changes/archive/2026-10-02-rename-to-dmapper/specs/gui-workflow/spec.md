# Spec Delta

## ADDED Requirements

### Requirement: Application name and window title
The application SHALL be named dMapper. The executable SHALL be `dMapper` (`dMapper.exe` on Windows), and the GUI window title SHALL be exactly "dMapper", without a version number or folder name. The GUI's texts (dialogs, tooltips) SHALL NOT refer to other products' workflows, such as Mandeye or HDMapping's registration steps.

#### Scenario: Window title during a run
- **WHEN** the GUI has started and a recording folder has been picked and is being processed
- **THEN** the window title still reads exactly "dMapper"

#### Scenario: Folder dialog
- **WHEN** the user chooses Start → Load data and process
- **THEN** the folder dialog asks for a recording folder (lidar*.laz + imu*.csv) and does not mention Mandeye

## MODIFIED Requirements

### Requirement: Remaining view options
The menu bar SHALL contain Start, View and Camera; there is no Info button or Info window. The View menu SHALL offer Show trajectory, Show compass/ruler and Lock Z. The Camera menu (fixed views and orthographic projection, from `core`) stays as it is. The progress window with Pause and Resume SHALL remain.

The background colour SHALL come from the `[misc] clear_color` setting of the loaded parameters, with no colour picker in the GUI.

#### Scenario: Menu bar
- **WHEN** the GUI has started
- **THEN** the menu bar shows Start, View and Camera, and nothing else

#### Scenario: Background colour from the file
- **WHEN** the user loads a TOML whose `[misc] clear_color` differs from the current one
- **THEN** the view background changes to that colour
