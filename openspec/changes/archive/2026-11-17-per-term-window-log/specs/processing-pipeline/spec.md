## ADDED Requirements

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
