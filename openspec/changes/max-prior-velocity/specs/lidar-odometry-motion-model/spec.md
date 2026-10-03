## ADDED Requirements

### Requirement: Velocity prior limit
For each window after the first, the odometry SHALL estimate the previous window's final velocity. It SHALL use that velocity as a prior for the window only when its magnitude is at most `max_prior_velocity_mps`. The setting is in the `[motion_model_uncertainty]` section of the parameters TOML, and its built-in default is 10.0 m/s. A faster velocity SHALL NOT be used as a prior: the window SHALL then be processed as if no previous velocity were available, and a console message SHALL report the speed and the limit. The setting SHALL be loaded from and saved to the parameters TOML like the other settings of this capability.

#### Scenario: Velocity within the limit is used
- **WHEN** the previous window ends at 3 m/s and `max_prior_velocity_mps` is 5.0
- **THEN** the window uses the 3 m/s velocity as its prior

#### Scenario: Velocity above the limit is dropped
- **WHEN** the previous window ends at 7 m/s and `max_prior_velocity_mps` is 5.0
- **THEN** the window uses no velocity prior, and the console shows the 7 m/s speed and the 5 m/s limit

#### Scenario: TOML without the key keeps the old limit
- **WHEN** a parameters TOML that has no `max_prior_velocity_mps` key is loaded
- **THEN** the limit is 10.0 m/s, as it was before this setting existed
