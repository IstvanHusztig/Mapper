## MODIFIED Requirements

### Requirement: GUI shows the current stage
While the GUI processes a recording, its Run panel SHALL list the run's stages in order (Loading, Odometry, Loop closure, Consistency, Saving), marking each as done, running or still to come.
- During the odometry stage it SHALL show the percentage of windows done, the elapsed time and the estimated finish time.
- For the other stages it SHALL show the stage name and the elapsed time, but no percentage or estimate.
- A stage that is switched off in the settings SHALL NOT be listed.
- When the run has finished, the list SHALL stay, with the total time, until the next run starts.

#### Scenario: One-click run with loop closure
- **WHEN** the user processes a folder with `use_loop_closure = true`
- **THEN** the Run panel marks "Loading" as running, then "Odometry" with a rising percentage, then "Loop closure", then "Saving", each marked done when the next starts

#### Scenario: Loading is visible
- **WHEN** the user has picked a folder and the data is still loading
- **THEN** the Run panel marks "Loading" as running, and the GUI keeps redrawing

#### Scenario: Consistency switched off
- **WHEN** `use_consistency = false`
- **THEN** the Run panel never lists "Consistency"

## ADDED Requirements

### Requirement: Log panel in the GUI
The GUI SHALL show the messages of the current run in its Log panel, in the order they were written, each with its level (debug, info, warning, error). The user SHALL be able to filter the panel by level and to copy its text. Every message SHALL still go to the console exactly as without the panel. The panel SHALL keep at least the last 20 000 messages and SHALL stay responsive however many messages arrive. Its content SHALL NOT be written to a file.

#### Scenario: A warning during a run
- **WHEN** a GUI run writes a warning
- **THEN** the warning appears in the Log panel, marked as a warning, and on standard error as before

#### Scenario: Long run
- **WHEN** a GUI run of `44` writes its messages
- **THEN** the panel scrolls smoothly, and the run's time stays within 1.3 times the CLI's
