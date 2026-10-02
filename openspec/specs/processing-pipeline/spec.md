# processing-pipeline Specification

## Purpose

Defines the one sequence of processing stages that turns a recording into a trajectory and point
clouds, where its settings come from, and that the command line and the GUI produce identical
results for the same input and settings.

## Requirements

### Requirement: One processing sequence
A full run SHALL execute these stages in this order:
1. load the recording;
2. estimate the IMU orientation;
3. split into windows;
4. run the LiDAR odometry;
5. loop closure, only when `use_loop_closure` is true;
6. the consistency pass, only when `use_consistency` is true and `num_constistency_iter` > 0;
7. save the results.

The command line and every GUI path that produces a result SHALL use this same sequence.

#### Scenario: Loop closure enabled
- **WHEN** a recording is processed with `use_loop_closure = true`
- **THEN** loop closure runs after the odometry and before the consistency pass, in the CLI and in the GUI alike

#### Scenario: Post-processing disabled
- **WHEN** `use_loop_closure = false` and `use_consistency = false`
- **THEN** the results are saved directly after the odometry

### Requirement: GUI and CLI results are identical
For the same input folder and the same settings, a result saved by the GUI's "Load data and process" SHALL be identical to the CLI's output. The comparison ignores only the run-dependent fields listed in `regression-baseline`.

#### Scenario: GUI run of the short reference dataset
- **WHEN** the GUI processes `44_short` with `settings/lidar-vertical.toml` loaded and saves the result
- **THEN** every output file compares identical with `44_short_baseline`

### Requirement: CLI settings come only from the given file
In command-line mode, the settings SHALL be the built-in defaults overlaid with the given
parameter file and nothing else, so the result does not depend on the current working directory.
If the given file cannot be read or parsed, the CLI SHALL exit with a non-zero status without
processing.

#### Scenario: Started from another directory
- **WHEN** the CLI processes `44_short` with `settings/lidar-vertical.toml` while the working directory contains a different `default_settings.toml`
- **THEN** the output is identical to `44_short_baseline`

#### Scenario: Unreadable parameter file
- **WHEN** the parameter file does not exist or is not valid TOML
- **THEN** the CLI prints an error and exits non-zero without writing an output folder

### Requirement: GUI start-up defaults come from the exe folder
On start-up the GUI SHALL load `default_settings.toml` from the folder of the executable, if it
exists there, independent of the current working directory.

#### Scenario: Started from Visual Studio or a shortcut
- **WHEN** the GUI is started with a working directory that is not the exe folder
- **THEN** it still loads the `default_settings.toml` that sits next to the exe

### Requirement: Consistency only inside the processing sequence
The GUI SHALL NOT offer an action that runs the consistency pass on its own. Consistency runs only as part of the processing sequence, controlled by the settings (`use_consistency`).

#### Scenario: After a run
- **WHEN** a run started with "Load data and process" has finished in the GUI
- **THEN** the available actions are the Start menu and the view options, with no consistency button
