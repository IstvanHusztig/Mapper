## ADDED Requirements

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
