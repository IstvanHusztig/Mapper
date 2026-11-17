## ADDED Requirements

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
