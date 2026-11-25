> **Withdrawn delta, 2026-11-13.** User rejected the residual-floor experiment
> and its implementation was removed. Retained for evidence only; do not sync
> these requirements into main specs or reimplement them. See ../../rejection.md.

## ADDED Requirements

### Requirement: Optional residual confidence floor preserves map statistics
Odometry SHALL support a nonnegative residual-noise standard-deviation floor. When positive, accepted cells SHALL contribute information computed from their covariance with eigenvalues bounded below by the squared configured floor. Stored cell means, covariance, normals and populations SHALL remain unchanged. Both matching grids, robust residual weights and reported cost SHALL use the same effective information. This option SHALL NOT directly regularize loop-closure or consistency information or change their models/configuration; downstream information and outputs MAY change because odometry inputs change.

#### Scenario: Thin accepted cell with floor enabled
- **WHEN** an eligible positive-definite cell has an eigenvalue below the configured variance floor
- **THEN** its residual information is limited in that direction, with matching cost and solve weighting, and its stored statistics are unchanged

#### Scenario: Floor disabled
- **WHEN** the option is absent or zero
- **THEN** existing cell eligibility, arithmetic and output behavior are preserved

#### Scenario: Invalid enabled covariance
- **WHEN** enabled regularization encounters nonfinite or nonpositive covariance eigenvalues or a failed decomposition
- **THEN** the cell does not contribute invalid information and the rejection is identifiable in diagnostic evidence

#### Scenario: Regularization does not broaden legacy support
- **WHEN** a cell would be ineligible under the frozen baseline's eligibility checks
- **THEN** enabling the floor does not silently admit that cell

#### Scenario: Mixed-axis Huber reweighting
- **WHEN** flooring one covariance direction decreases a correspondence's Mahalanobis residual and increases its Huber weight
- **THEN** all axes use the consistently recomputed weight and verification includes possible increased influence on unfloored axes rather than asserting monotonic robust weakening

#### Scenario: Rotated fixed eligible covariance
- **WHEN** the same eligible covariance and residual are rigidly rotated
- **THEN** the floor operation transforms equivariantly within numerical tolerance, without promising rotation invariance of the separate legacy eligibility gate
