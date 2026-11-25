# Spec Delta

## ADDED Requirements

### Requirement: Gyro scale is a sensor setting
The settings `gyro_scale_x`, `gyro_scale_y` and `gyro_scale_z` SHALL multiply the gyro rate of every IMU sample on
that sensor axis when the recording is loaded, before any part of the processing reads the gyro (orientation filter,
start-up bias, windows). The defaults SHALL be 1.0, and at 1.0 an axis SHALL be left untouched. When any axis differs
from 1.0 the run SHALL log the three factors once.

#### Scenario: Defaults change nothing
- **WHEN** a recording is processed with settings that don't set the three keys, or set them to 1.0
- **THEN** the results are bit-identical with the results from before the settings existed

#### Scenario: Setting replaces an edited recording
- **WHEN** a recording is processed with `gyro_scale_z = 1.004`
- **THEN** each gyro z rate the processing sees is the parsed value times 1.004, rounded once to float; it equals
  that of a copy of the recording whose gyro z column was multiplied by 1.004, except for a value now and then that
  is one float step off because the CSV reader does not round decimals exactly

#### Scenario: Snapshots made before the setting
- **WHEN** an odometry snapshot or window capture written before the setting existed is used with settings that
  leave the three keys at 1.0
- **THEN** it is accepted as before

### Requirement: The Livox settings file carries the measured gyro scale
`settings/livox-vertical.toml` SHALL set the gyro scale measured on the Livox Mid-360S, and its header SHALL say
where the value comes from. The L2 settings file SHALL keep the defaults until a scale is measured for it.

#### Scenario: Livox file after the change
- **WHEN** the unit tests load `settings/livox-vertical.toml`
- **THEN** `gyro_scale_z` is the measured value and the saved file equals its golden TOML
