# Spec Delta

## Purpose

Defines how a processing run reports what it is doing: the stage and progress shown in the GUI,
pausing and resuming, and the messages written to the console. Reporting never changes the result.

## ADDED Requirements

### Requirement: GUI shows the current stage
While the GUI processes a recording, its progress window SHALL show which stage is running: Loading, Odometry, Loop closure, Consistency or Saving.
- During the odometry stage it SHALL show the percentage of windows done, the elapsed time and the estimated finish time.
- For the other stages it SHALL show the stage name and the elapsed time, but no percentage or estimate.
- A stage that is switched off in the settings SHALL NOT be shown.

#### Scenario: One-click run with loop closure
- **WHEN** the user processes a folder with the simple one-click flow and `use_loop_closure = true`
- **THEN** the progress window shows "Loading", then "Odometry" with a rising percentage, then "Loop closure", then "Saving", in that order

#### Scenario: Loading is visible
- **WHEN** the user has picked a folder in the one-click flow and the data is still loading
- **THEN** the progress window is open and shows "Loading", and the GUI keeps redrawing

#### Scenario: Consistency switched off
- **WHEN** `use_consistency = false`
- **THEN** the progress window never shows "Consistency"

### Requirement: Pause and resume
While a GUI run is in the loading or odometry stage, the user SHALL be able to pause it and resume it.
- A paused run SHALL stop at the next window boundary and stay stopped until resumed.
- After resuming, the run SHALL continue where it stopped.
- The saved result SHALL be identical to that of a run that was never paused.

#### Scenario: Pause during odometry
- **WHEN** the user presses Pause during the odometry stage, waits, and presses Resume
- **THEN** the percentage stops rising while paused, the run then finishes, and the saved result is identical to an unpaused run

### Requirement: Whole console messages
Each console message from a run SHALL be written as one complete line, even when several threads report at the same time.
- Informational and debug messages SHALL go to standard output.
- Warnings and errors SHALL go to standard error.

#### Scenario: Parallel loading
- **WHEN** a recording with many LAZ files is loaded with parallel loading
- **THEN** every per-file line (`total - filter = kept`) appears whole on its own line, and no two lines are mixed together

### Requirement: Logging stays on the console
A run SHALL NOT write its log messages to a file, and SHALL NOT add any file to the output folder because of logging or progress reporting.

#### Scenario: Output folder contents
- **WHEN** the CLI processes a recording
- **THEN** the output folder holds the same set of files as before this capability existed

### Requirement: Reporting does not change results
Progress reporting, pausing and log output SHALL NOT change what a run computes. A refactor that changes only how a run reports SHALL reproduce the `44_short` baseline bit for bit, and the full `44` baseline within the tolerance of `regression-baseline`.

#### Scenario: Baseline after the reporting change
- **WHEN** the CLI processes `44_short` and then `44` with `settings/lidar-vertical.toml`
- **THEN** `tools/compare_baseline.py` reports `44_short` identical with its manifest, and `44` identical or within tolerance 1e-6 of the baseline folder
