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
- **WHEN** the GUI processes `44_short` with `settings/l2-vertical.toml` loaded and saves the result
- **THEN** every output file compares identical with `44_short_baseline`

### Requirement: CLI settings come only from the given file
In command-line mode, the settings SHALL be the built-in defaults overlaid with the given
parameter file and nothing else, so the result does not depend on the current working directory.
If the given file cannot be read or parsed, the CLI SHALL exit with a non-zero status without
processing.

#### Scenario: Started from another directory
- **WHEN** the CLI processes `44_short` with `settings/l2-vertical.toml` while the working directory contains a different `default_settings.toml`
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

### Requirement: Results carry dMapper's version
A run's output folder SHALL contain its parameters as `dMapper_params_<version>_<date-time>.toml` and its processing report as `dMapper_results_<version>_<date-time>.json`, where `<version>` is the version dMapper was built with (major.minor.patch, set in the top-level `CMakeLists.txt`). Both files SHALL record that version (`software_version`; the report's `lidar_odometry_version` as "v" + version). The version stamp in a loaded parameter file SHALL NOT change the version a run records; it is only compared with the build's version, and a different major or minor version gives a warning.

#### Scenario: A run with the shipped settings
- **WHEN** the CLI processes `44_short` with `settings/l2-vertical.toml`
- **THEN** the output folder contains `dMapper_params_0.1.0_<date-time>.toml` and `dMapper_results_0.1.0_<date-time>.json`, and both record version 0.1.0

#### Scenario: A parameter file from another version
- **WHEN** a parameter file with `software_version = "0.95.0"` in `[version_info]` is loaded
- **THEN** a version-mismatch warning is printed, and the run still records 0.1.0

### Requirement: Settings files are read strictly
A settings file SHALL be refused, with a list of its problems, when it holds a key the program would not read: an
unknown section or key, a key in another section than its own, or a value of the wrong type (a double accepts an
integer). A setting removed from dMapper SHALL be accepted with a message that it is ignored when its value asks for
the behaviour that remains (a removed switch set to false or 0; a parameter of a removed feature at any value), and the
file SHALL be refused when the value asks for the removed behaviour.

#### Scenario: A misplaced key
- **WHEN** a settings file has `ndt_huber_cutoff = 3.0` in `[lidar_odometry_control]` instead of `[imu_translation]`
- **THEN** the file is refused and the message names the key and the section it belongs in

#### Scenario: An older file with a removed setting at the remaining behaviour
- **WHEN** a settings file written before the removal holds `use_mutliple_gaussian = false` and a `[robust_lidar_odometry]` section with `use_robust_and_accurate_lidar_odometry = false`
- **THEN** the file loads, and a message says that each of these settings was removed and is ignored

#### Scenario: A removed setting asks for removed behaviour
- **WHEN** a settings file holds `use_mutliple_gaussian = true`
- **THEN** the file is refused and the message names the setting and the value that is accepted

### Requirement: Results record the build
The processing report (`dMapper_results_*.json`) SHALL hold a `build_info` block with the git commit the program was
built from, whether the source had uncommitted changes of tracked files at build time, the build type, the compiler, the
compiler flags and the compile definitions. The values SHALL be taken at build time, not at configure time.

#### Scenario: A build after a commit
- **WHEN** the program is built again after a commit, without configuring again, and processes a recording
- **THEN** the report's `build_info.git_commit` is the new commit

### Requirement: Window capture and replay
With `capture_windows_from_s` and `capture_windows_to_s` set (`[lidar_odometry_debug_info]`; off while `to < from` or
`from < 0`), a command-line run SHALL write `window_capture_<from>-<to>s.bin` to its output folder. It SHALL hold:
- the window loop's state before the first window that starts at or after `from` seconds (counted from the first
  window's start);
- the windows up to the last one that starts at or before `to`;
- their points;
- each window's last iteration and final poses.

The capture SHALL NOT change the run's results.

`dMapper --replay <capture> <params.toml>` SHALL run only the captured windows again:
- **With the odometry settings the capture was made with:** each window SHALL end after the iteration it ended after in
  the run, without the time limit, and every window SHALL come out bit for bit as captured; otherwise the replay SHALL
  exit non-zero.
- **With other odometry settings:** the replay SHALL report how far the windows move from the captured ones.

#### Scenario: A capture doesn't change the run
- **WHEN** 44_short is processed with a capture of 20–40 s
- **THEN** every result file matches the run without the capture, apart from the params TOML, and the output folder also holds the capture

#### Scenario: A replay with the same settings
- **WHEN** a capture of Livox 3, 100–130 s, is replayed with the settings of its run
- **THEN** all 84 windows are identical with the capture, and the replay exits 0

#### Scenario: A replay with other settings
- **WHEN** a capture is replayed with fewer iterations than its run
- **THEN** the replay reports the windows that differ and their largest position difference

### Requirement: Per-term window table
With `log_window_terms = true` (`[lidar_odometry_debug_info]`, default `false`) the odometry SHALL record, for every
window, a table of what each term of the window's optimization does, at two states: the guess (before the first
iteration) and the result (after the last).

- **Rows:** NDT indoor grid, NDT outdoor grid, NDT before the degeneracy projection, edge positions, edge rotations,
  gyro-bias prior, velocity continuity, accelerometer, velocity prior, zero velocity, weak end-velocity tie,
  acceleration-offset prior, origin fix, their total, and the damping of the iterations. A row whose term is not active
  in the window SHALL be absent.
- **Per row:** the cost, the number of residuals, and for each of four motions of the window (shift, stretch, curve,
  turn) along each of three directions (along travel, sideways, vertical) the weight and the pull. The pull is in
  metres (degrees for turn) at the window's last node; it is empty where the weight is zero.
- **Output:** one log line per window; `window_terms.csv` with every row and `window_features.csv` with one row per
  window. A command-line run SHALL write both files to its output folder; `dMapper --replay` SHALL write them next to
  the capture file.
- **No effect on results:** every pose, velocity, bias, map and output file other than the params TOML and the two CSV
  files SHALL be the same bits with the setting on and off. A failure while computing the table SHALL be logged as a
  warning and leave that window's rows empty; it SHALL NOT stop the run.
- **Compatibility:** the setting SHALL NOT be part of the odometry settings that window captures and odometry snapshots
  are checked against.

#### Scenario: The table doesn't change the run
- **WHEN** 44_short is processed with `log_window_terms = true`
- **THEN** every result file matches the manifest except the params TOML, and the output folder also holds `window_terms.csv` and `window_features.csv` with one table per window

#### Scenario: A replay with the table
- **WHEN** the Livox 3 capture (100–130 s) is replayed with its run's settings plus `log_window_terms = true`
- **THEN** all 84 windows are identical with the capture, the replay exits 0, and `window_terms.csv` holds the 84 windows' tables

#### Scenario: Old captures stay valid
- **WHEN** a capture made before this setting existed is replayed with its run's settings
- **THEN** the replay accepts it as made with the same odometry settings
