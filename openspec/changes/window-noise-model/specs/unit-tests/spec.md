## ADDED Requirements

### Requirement: Window noise model tested on synthetic windows
The unit tests SHALL build the window noise model on synthetic windows with known timestamps and settings, without
recorded data, and check each weight against its formula and against what the window's terms used before the terms
took the model.

#### Scenario: Each weight equals its formula
- **WHEN** the model is built for a synthetic window with known node timestamps and settings
- **THEN** the time-scaled rotation sigma of each edge equals max(k·ARW·√dt, floor), with the turn windows' k; the translation weights carry the position information scale only with the accelerometer term; each accelerometer weight equals 1/((density²/τ + floor²)·τ²) for its span; the priors' covariances equal the carried ones grown by their random walk over the window or the gap

#### Scenario: Conditions as in the terms
- **WHEN** the model is built for the first window, for a window at rest, and for a window without the accelerometer term
- **THEN** the first window has no gyro-bias, accelerometer or velocity-prior weights; the window at rest has the zero-velocity weight; the velocity-continuity weights exist only without the accelerometer term

#### Scenario: Same weights as the terms computed
- **WHEN** the model is built for a synthetic window before the terms are switched to it
- **THEN** each of its weights is bit-identical with the weight the corresponding term computes for that window
