## ADDED Requirements

### Requirement: Degeneracy-aware NDT terms
The scan-to-map optimization SHALL decompose each window's geometric translation information, the sum of n·nᵀ over the surface normals its points are matched to. For every eigen-direction it SHALL compute the direction's share of the information: its eigenvalue divided by the largest eigenvalue. Along a direction whose share is below `degenerate_projection_ratio`, the NDT point terms SHALL carry no translation information, so that the motion model alone places the window along that direction. Between `degenerate_projection_ratio` and `degenerate_projection_ratio_full` the NDT translation information along the direction SHALL be scaled linearly from zero to its full value. Rotations and all other terms SHALL be unchanged. The settings are in the `[imu_translation]` section of the parameters TOML. Their built-in defaults are 0.0, and `degenerate_projection_ratio_full = 0.0` SHALL leave the NDT terms exactly as before. The result SHALL NOT depend on the number of threads.

#### Scenario: Blind direction left to the motion model
- **WHEN** a window in a corridor that is open along x is optimized with `degenerate_projection_ratio = 0.1` and `degenerate_projection_ratio_full = 0.2`, and its guess is shifted along x and across it
- **THEN** the shift along x is kept and the error across x is corrected

#### Scenario: Constrained window unchanged
- **WHEN** a window in a closed room, whose smallest share is about 0.48, is optimized with `degenerate_projection_ratio = 0.1` and `degenerate_projection_ratio_full = 0.2`
- **THEN** the result is bit-identical to the result with the setting off

#### Scenario: Off by default
- **WHEN** a parameters TOML without the two keys is loaded
- **THEN** both are 0.0 and the odometry gives the same results as before the setting existed

### Requirement: Loop closure cannot wreck a run
The loop closure SHALL solve its segment pose graph with steps that do not raise the weighted squared error of the graph. The angle residuals SHALL be wrapped into (-π, π]. Before applying the result, it SHALL check that every corrected segment pose is finite and that no segment moved further than `loop_closure_max_correction_m` plus the largest accepted loop correction. A result that fails the check SHALL be discarded with a warning, and the trajectory SHALL stay as the odometry left it.

#### Scenario: Consistent loop applied
- **WHEN** a loop of segments drifts 5 cm per segment and one exact loop edge closes it
- **THEN** the drift is removed and the result passes the check

#### Scenario: Runaway result refused
- **WHEN** the pose graph result moves a segment by 2.2·10⁶ m, or contains a non-finite pose
- **THEN** the result is discarded, a warning is logged, and the trajectory is unchanged
