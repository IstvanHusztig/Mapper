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

### Requirement: Window verdict
After each window's optimization the odometry SHALL record what the run already knows about the window as readings
and flags, and at the end of the odometry it SHALL report the run's triggers, each `CLEAR`, `TRIGGER` or
`NOT MEASURED`. The verdict is an investigation trigger (agreed evaluation contract of 2026-11-12), not a statement
about the map's accuracy. A reading that a window does not have SHALL be `NOT MEASURED`, never clear.

- **Flags of a window, and the run's triggers:**
  - bent / bent severely: the vertical component of the rotation from the window's guess to its result at the last
    node is above 3° / above 10°. Run triggers: more than 1 % of the windows bent; any window bent severely.
  - fast: the norm of the window's end velocity is above 2.5 m/s. Run trigger: any fast window.
  - bias high: the estimated gyro bias of an axis is above 0.5 °/s. Run trigger: any such window.
- **Readings without a limit** (recorded, never a flag): why the iterations stopped (converged, iteration limit, time
  limit, forced by a replay), the iterations run, the last damping factor, steps not applied, big-jump nodes and
  rejected steps; the start velocity's distance from its prior (chi²); the information ratio; the tilt bend against
  the gyro; and, with `log_window_terms`, each term's cost per residual, the largest remaining step and the LiDAR's
  share of the weight along travel.
- **Windows without a reading:** a window without the accelerometer term has no speed and no velocity chi²; a run
  without gyro-bias estimation has no bias reading. These SHALL be `NOT MEASURED` and SHALL show in the coverage.
- **Run summary:** at the end of the odometry every run SHALL log, per flag, the number and share of windows that have
  it, the first and the worst with its time, and the coverage (windows measured of all), and the run's triggers; per
  reading its median, 95th percentile and largest value; and for the velocity chi² the median and 95th percentile
  (trigger above 5 or above 20) over the windows with an information ratio below 0.45, with the same statistics over
  all measured windows beside them. A replay SHALL log the summary of the windows it replayed; a cancelled run SHALL
  log the summary of the windows it finished and say that it is incomplete.
- **Per-window output:** with `log_window_verdict = true` (`[lidar_odometry_debug_info]`, default `false`) the run SHALL
  log one line, starting with `window verdict <n>:`, for every window that has a flag, with all its readings. With
  `log_window_terms` the flags and readings SHALL also be columns of `window_features.csv`.
- **No effect on results:** no window SHALL be processed differently because of its verdict. Every output file other
  than the params TOML and `window_features.csv` SHALL be the same bits with `log_window_verdict` on and off, and the
  setting SHALL NOT be part of the odometry settings that window captures and odometry snapshots are checked against.

#### Scenario: A run reports its windows
- **WHEN** 44_short is processed with the shipped settings
- **THEN** the log holds the verdict summary with each flag's count, share and coverage and the run's triggers, and every result file matches the manifest

#### Scenario: The summary agrees with the log tool
- **WHEN** 44_short is processed with `log_window_diagnostics` and `log_window_verdict` on
- **THEN** the summary's counts of windows bent more than 3° and 10°, faster than 2.5 m/s and with a bias above 0.5 °/s equal those of `tools/run_health.py` on the same log, except for windows whose reading lies within the log's printed precision of a limit and windows the two treat differently on purpose, and both kinds are listed

#### Scenario: A reading that doesn't exist
- **WHEN** a window has no accelerometer term (the first window, or `imu_always_on` off)
- **THEN** its speed and its velocity chi² are `NOT MEASURED`, and the summary's coverage shows it

### Requirement: Window noise model
Every weight of a window's least-squares problem SHALL come from one noise model per window, built before the window's
first iteration from the settings, the window's timestamps and the state carried from the previous window; the terms
SHALL receive their weights from it and SHALL NOT compute them. Weights that depend on the current poses or the map
(the NDT cell information, planarity, range weight and cap, Huber weight, information scale and degeneracy projection)
SHALL be applied per point by the NDT term with their parameters taken from the model. The model SHALL change no
result.

- **Contents:** the motion edges' translation and rotation weights per edge (with the position information scale and
  the turn windows' inflation), the velocity-continuity weights, the accelerometer terms' weights with their spans, the
  gyro-bias, start-velocity and acceleration-offset priors (mean, covariance, information), the zero-velocity weight,
  the weak end-velocity tie, the origin fix's weights, and the NDT term's parameters per grid.
- **Same model everywhere:** the window's problem, the per-term window table and the weak-axis diagnostic SHALL read the
  same model; the diagnostic SHALL apply the range weight and its cap per grid as the NDT term does.
- **Noise budget:** every run SHALL log one `noise budget` line at the start of the odometry with the constants derived
  from the settings (sigmas before time scaling, accelerometer noise density and floor, random walks, scales).
- **Per-window output:** with `log_window_noise = true` (`[lidar_odometry_debug_info]`, default `false`) the run SHALL
  log one line per window, starting with `window noise <n>:`, giving the model as sigmas (1/√weight): the motion edges'
  translation and rotation sigmas (smallest, median and largest over the edges, per axis), the velocity-continuity
  sigma, the accelerometer sigmas (smallest and largest over its terms), the start-velocity prior's sigmas (square
  roots of its covariance's eigenvalues), the bias and offset priors' sigmas, the zero-velocity sigma when the window is
  at rest, and the NDT scale and Huber cutoff. With `log_window_terms` the same values SHALL be columns of
  `window_features.csv`.
- **No effect on results:** every output file other than the params TOML and `window_features.csv` SHALL be the same
  bits with `log_window_noise` on and off, and with this change and without it; the setting SHALL NOT be part of the
  odometry settings that window captures and odometry snapshots are checked against.

#### Scenario: The refactor changes no result
- **WHEN** 44_short, the full 44 and Livox 3 are processed with the shipped settings
- **THEN** every result file matches its manifest except the params TOML entry, which gains `log_window_noise`

#### Scenario: A run reports its noise budget
- **WHEN** any recording is processed
- **THEN** the log holds one `noise budget` line before the first window

#### Scenario: Per-window noise lines
- **WHEN** 44_short is processed with `log_window_noise` on
- **THEN** the log holds one `window noise <n>:` line per window, and every result file except the params TOML is the same bits as with the setting off

#### Scenario: Captures replay unchanged
- **WHEN** the 44_short (20–40 s) and Livox 3 (100–130 s) captures made before this change are replayed with their runs' settings
- **THEN** every window is identical with the capture

### Requirement: Handed-on state and honesty readings of a window
The odometry SHALL record, per window, how far the window moved each carried state from its prior, measured against
the covariance that move has under the window's own model, and, with the per-window table, the state the window was
prepared with and the state it actually hands to the next one. These are readings for investigation; they are
descriptive, and they are not statements about the map's accuracy.

- **Normalised move:** for a carried state with prior mean m and prior covariance P, the window's result x and the
  covariance P_post the window hands on for it, the reading SHALL be
  (x - m)^T P^-1 (x - m) / (n - tr(P^-1 P_post)), with n the state's dimension. Its expected value in a
  linear-Gaussian update is 1.
- **States:** the start velocity, in every window that has the accelerometer term; the gyro bias; and, with
  `imu_carry_joint_covariance`, the start velocity and the acceleration offset jointly, with their joint prior and
  posterior. The start velocity's own reading SHALL also be given in that case.
- **`NOT MEASURED`:** a reading SHALL be `NOT MEASURED`, never a number, when the window has no such term; when the
  covariance it needs is absent, not finite, ill-conditioned, or would mean a negative reduction of uncertainty;
  when the window's last step still moved the state by more than a stated small share of its prior sigma (a window
  that stopped early); when the gyro bias was limited by its clamp; and when n - tr(P^-1 P_post) is below 0.5.
- **Present where it should be:** an ordinary window that adds information to a carried state and ends with a small
  last step SHALL have a finite reading for it, in both the separate and the joint form of the prior.
- **Run summary:** the verdict summary of every run, replay and cancelled run SHALL list, per reading, the largest
  value, the median, the 95th percentile and the coverage (windows measured of all, also when none was). A window
  that has a flag SHALL show its readings on its line. The readings SHALL have no limit, no flag and no run trigger.
- **The velocity chi² is unchanged:** its formula, its limits (5 and 20) over the windows with an information ratio
  below 0.45, and its statistics over all measured windows SHALL stay as they are.
- **Prepared and handed-on state:** with `log_window_terms` the window's record SHALL hold, on the window's own row:
  its first position as prepared, and the means and sigmas of the priors it was prepared with; and, taken after the
  hand-off: the end velocity, the carried gravity vector, the gyro bias, each with its claimed sigmas and whether it
  is valid; the velocity-offset covariance where it is carried; the last node's position as solved and as handed on;
  the window's mean rotation rate from its attitude increments and its acceleration at its start. These SHALL be
  columns of `window_features.csv` after the columns that file had before, which SHALL keep their names, order and
  values. A record SHALL be written once per window; a window whose hand-off is cancelled SHALL have its handed-on
  values marked absent.
- **No effect on results:** no window SHALL be processed differently because of these readings. Every output file
  other than `window_features.csv` SHALL stay the same under the existing result comparison, and nothing here SHALL
  be part of the odometry settings that window captures and odometry snapshots are checked against, or of a capture
  file.

#### Scenario: A run reports the readings
- **WHEN** 44_short is processed with the shipped settings
- **THEN** the verdict summary lists the largest value, median, 95th percentile and coverage of each normalised move, and every result file matches the manifest

#### Scenario: An ordinary window has a reading
- **WHEN** a window that the LiDAR sees well finishes its iterations with a small non-zero last step
- **THEN** the normalised move of its start velocity is a finite number, with the separate and with the joint prior

#### Scenario: A reading that doesn't exist
- **WHEN** a window has no accelerometer term, or adds almost nothing to a carried state, or stopped while its steps were still large
- **THEN** that state's normalised move is `NOT MEASURED` and the summary's coverage shows it

#### Scenario: The table gains columns and keeps the old ones
- **WHEN** a capture is replayed with `log_window_terms` before and after this change
- **THEN** `window_terms.csv` is the same bits, and `window_features.csv` has the same values in the columns it had before, followed by the new columns

#### Scenario: A cancelled hand-off
- **WHEN** a run is cancelled at a window's hand-off
- **THEN** that window's record is written once, with its table as before and its handed-on values marked absent

#### Scenario: A missing input is named, not replaced
- **WHEN** `tools/honesty_report.py` is given a `window_features.csv` without the columns a figure needs
- **THEN** it prints that figure as unavailable and computes the others under their stated definitions

### Requirement: Optimizer steps are applied completely or not at all
An iteration of the window optimizer or of consistency SHALL either apply its whole step to every unknown, or apply nothing and report it. A step SHALL NOT be dropped because one unknown's step is exactly zero. A step SHALL NOT be applied to only part of the nodes.

#### Scenario: An unknown with a zero step
- **WHEN** the solution of an iteration has an entry that is exactly zero (for example an extra unknown that nothing constrains)
- **THEN** the other unknowns' steps are applied, and no step is reported as skipped

#### Scenario: A node step larger than 1 m
- **WHEN** one node's step in an iteration exceeds 1 m
- **THEN** no node, bias, velocity or offset of that window changes in that iteration, and the event is counted as a big jump

### Requirement: Output poses keep full precision
The chunk poses in `session_poses.mrp` and `.mri` SHALL be written with enough digits to round-trip a double.

#### Scenario: Far from the origin
- **WHEN** a chunk pose has a translation of 1234.56789 m
- **THEN** the value read back from the file equals the one written
