> **Withdrawn delta, 2026-11-13.** User rejected the residual-floor experiment
> and its setting was removed. Retained for evidence only; do not sync these
> requirements into main specs or reimplement them. See ../../rejection.md.

## ADDED Requirements

### Requirement: Residual-noise floor is an explicit recorded setting
The system SHALL load and save `ndt_residual_noise_floor_sigma_m` in `[lidar_odometry_control]` as a finite nonnegative standard deviation in metres. The omitted-key default SHALL be zero. Invalid negative or nonfinite settings, and positive values whose square is nonfinite or rounds to zero, SHALL be rejected before processing. Saved run parameters and startup diagnostics SHALL identify enabled regularization and its odometry-only scope. No positive default SHALL be introduced by this experiment.

#### Scenario: Old settings file loads
- **WHEN** a parameters file omits the residual-noise floor
- **THEN** the floor is disabled and the previous processing behavior is preserved

#### Scenario: Explicit setting round trip
- **WHEN** a valid positive residual-noise floor is loaded and saved
- **THEN** the same value and units are preserved and the run identifies activation

#### Scenario: Invalid floor rejected
- **WHEN** a negative, nonfinite or unrepresentable-squared residual-noise floor is supplied
- **THEN** processing is refused with an actionable setting error
