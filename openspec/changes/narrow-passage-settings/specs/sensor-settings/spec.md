## MODIFIED Requirements

### Requirement: One shipped settings file per supported sensor
The repository SHALL ship `settings/l2-vertical.toml` for the Unitree L2 and `settings/livox-vertical.toml` for the Livox Mid-360S, both for the vertical handheld mount. Each SHALL load without error, and SHALL save back to the same values (golden TOML test). A file whose values are not yet measured SHALL say so in its header. The Livox file's gyro noise (`gyro_arw_x/y/z_deg_sqrt_s`) SHALL be the values measured on the Livox gyro at rest, without inflation (`rotation_sigma_inflation_k` = 1), and its comment SHALL name the recording and time span they were measured on. A sensor MAY have further shipped settings files for a kind of place; the repository SHALL ship `settings/livox-narrow.toml` for the Livox Mid-360S in a passage where the scans do not see along the direction of travel. Such a file SHALL name in its header the recording it was tuned on, what it was judged by and what it cannot repair, and SHALL load and save back like the others (golden TOML test). The file for the vertical handheld mount SHALL NOT change with it.

#### Scenario: Both sensor files load
- **WHEN** the unit tests load `settings/l2-vertical.toml` and `settings/livox-vertical.toml` and save them again
- **THEN** each saved file equals its golden TOML

#### Scenario: Renamed L2 file keeps its values
- **WHEN** the CLI processes `44_short` with `settings/l2-vertical.toml`
- **THEN** the result is bit-identical with the `44_short` manifest made with the former `settings/lidar-vertical.toml`

#### Scenario: The Livox file carries the Livox gyro's measured noise
- **WHEN** `settings/livox-vertical.toml` is read
- **THEN** its gyro ARW values are 0.0094 / 0.0224 / 0.0144 °/√s with inflation 1 and floor 0.0002°, and its comment names `raw data/livox/4`, 2–20 s, as their source

#### Scenario: The narrow-passage file loads
- **WHEN** the unit tests load `settings/livox-narrow.toml` and save it again
- **THEN** the saved file equals its golden TOML, and its header names the recording it was tuned on

#### Scenario: The vertical file is untouched
- **WHEN** the narrow-passage file is added
- **THEN** a run with `settings/livox-vertical.toml` is bit-identical with its manifest
