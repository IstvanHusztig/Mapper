## ADDED Requirements

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
