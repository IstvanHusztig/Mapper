# Spec Delta

## ADDED Requirements

### Requirement: Limited redraw rate
The GUI SHALL limit how often it redraws the window: about 10 times per second while a run is going, and about 60 times per second otherwise. Mouse dragging MAY redraw more often. Processing in the GUI SHALL therefore take about as long as the same run through the CLI, and its result SHALL be identical to the CLI's.

#### Scenario: GUI run of the short reference dataset
- **WHEN** the GUI processes `44_short` with `settings/lidar-vertical.toml` loaded, on an otherwise idle computer
- **THEN** the elapsed time recorded in the results JSON is at most 1.3 times the CLI's elapsed time for the same run, and every output file compares identical with the `44_short` manifest

#### Scenario: Idle window
- **WHEN** no run is going and the user doesn't interact with the window
- **THEN** the GUI redraws at most about 60 times per second
