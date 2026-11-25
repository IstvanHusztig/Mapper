## MODIFIED Requirements

### Requirement: One shipped settings file per supported sensor
The repository SHALL ship `settings/l2-vertical.toml` for the Unitree L2 and `settings/livox-vertical.toml` for the Livox Mid-360S, both for the vertical handheld mount. Each SHALL load without error, and SHALL save back to the same values (golden TOML test). A file whose values are not yet measured SHALL say so in its header. The Livox file's gyro noise (`gyro_arw_x/y/z_deg_sqrt_s`) SHALL be the values measured on the Livox gyro at rest, without inflation (`rotation_sigma_inflation_k` = 1), and its comment SHALL name the recording and time span they were measured on.

#### Scenario: Both sensor files load
- **WHEN** the unit tests load `settings/l2-vertical.toml` and `settings/livox-vertical.toml` and save them again
- **THEN** each saved file equals its golden TOML

#### Scenario: Renamed L2 file keeps its values
- **WHEN** the CLI processes `44_short` with `settings/l2-vertical.toml`
- **THEN** the result is bit-identical with the `44_short` manifest made with the former `settings/lidar-vertical.toml`

#### Scenario: The Livox file carries the Livox gyro's measured noise
- **WHEN** `settings/livox-vertical.toml` is read
- **THEN** its gyro ARW values are 0.0094 / 0.0224 / 0.0144 °/√s with inflation 1 and floor 0.0002°, and its comment names `raw data/livox/4`, 2–20 s, as their source
