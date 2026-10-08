## MODIFIED Requirements

### Requirement: Degeneracy-aware NDT terms
The scan-to-map optimization SHALL decide, per window, which directions of the window's common motion the NDT terms constrain, and SHALL scale the NDT information in the other directions down, so that the motion model places the window there. Two detectors are available, and at most one SHALL be active:

- **Geometric ratio** (`degenerate_projection_ratio`, `degenerate_projection_ratio_full`, in the `[imu_translation]` section): this detector looks only at translation. It decomposes the sum of n·nᵀ over the matched surface normals.
  - Along a direction whose share (eigenvalue / largest eigenvalue) is below `degenerate_projection_ratio`, the NDT terms SHALL carry no translation information.
  - Between that value and `degenerate_projection_ratio_full`, the information SHALL be scaled linearly.
- **Signal-to-noise test** (`degenerate_snr_detection`, `degenerate_snr_factor`, `degenerate_point_sigma_m`, in the same section): this detector looks at both translation and rotation.
  - It forms the information that the summed NDT node terms give to a rigid motion of the whole window about the window's mean node position.
  - It separates translation from rotation by Schur complements and decomposes each.
  - Each eigen-direction's NDT information SHALL be scaled by the probability that its information exceeds `degenerate_snr_factor` times the information that noise alone would produce. The noise model takes the LiDAR's point noise `degenerate_point_sigma_m` and each cell's point count and spread.

Rotations SHALL be scaled only by the signal-to-noise test. All other terms SHALL be unchanged. Built-in defaults:
- `degenerate_projection_ratio` and `degenerate_projection_ratio_full`: 0.0;
- `degenerate_snr_detection`: false;
- `degenerate_snr_factor`: 10;
- `degenerate_point_sigma_m`: 0.02 m.

With both detectors off the NDT terms SHALL be exactly as before either existed. If a parameters TOML switches both detectors on, the signal-to-noise test SHALL be used and a warning SHALL be logged. The result SHALL NOT depend on the number of threads.

#### Scenario: Blind direction left to the motion model
- **WHEN** a window in a corridor that is open along x is optimized with either detector on, and its guess is shifted along x and across it
- **THEN** the shift along x is kept and the error across x is corrected

#### Scenario: Constrained window unchanged
- **WHEN** a window in a closed room is optimized with either detector on
- **THEN** no direction is scaled, and the result is bit-identical to the result with both detectors off

#### Scenario: Rotation about a tunnel's axis
- **WHEN** a window in a straight tunnel with a round cross-section is optimized with the signal-to-noise test on
- **THEN** the NDT information about rotation around the tunnel's axis is scaled down, and the rotations about the other axes are not

#### Scenario: Noise does not fake a constraint
- **WHEN** the cells of a corridor map get noisy normals and covariances, and a window is optimized with the signal-to-noise test on
- **THEN** the translation along the corridor is still scaled down

#### Scenario: Off by default
- **WHEN** a parameters TOML without the five keys is loaded
- **THEN** both detectors are off and the odometry gives the same results as before either detector existed
