# Spec Delta

## ADDED Requirements

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

## MODIFIED Requirements

### Requirement: Remaining view options
The menu bar SHALL contain Start, View and Camera; there is no Info button or Info window. The View menu SHALL offer Show point cloud, Show trajectory, Show compass/ruler and Lock Z. The Camera menu (fixed views and orthographic projection, from `core`) stays as it is. The progress window with Pause and Resume SHALL remain.

The background colour SHALL come from the `[misc] clear_color` setting of the loaded parameters, with no colour picker in the GUI.

#### Scenario: Menu bar
- **WHEN** the GUI has started
- **THEN** the menu bar shows Start, View and Camera, and nothing else

#### Scenario: Background colour from the file
- **WHEN** the user loads a TOML whose `[misc] clear_color` differs from the current one
- **THEN** the view background changes to that colour
