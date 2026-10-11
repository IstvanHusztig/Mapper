# Spec Delta

## ADDED Requirements

### Requirement: Directional release of the in-plane NDT pull in weak windows
With `ndt_in_plane_release_ratio` > 0, the scan-to-map optimization of a window whose previous window's LiDAR translation information ratio is below `ndt_in_plane_release_ratio` SHALL reduce, for every point matched to a planar cell, the cell's stiffness along the in-plane part of the previous window's weakest translation direction. The reduction SHALL be full at or below `ndt_in_plane_release_ratio_full` and fall linearly to none at `ndt_in_plane_release_ratio`. The stiffness along the cell's normal and the in-plane stiffness across that direction SHALL stay unchanged. Cells that are not planar SHALL keep the point-to-distribution term. The result SHALL NOT depend on the number of threads. With `ndt_in_plane_release_ratio` = 0, which is the built-in default, the results SHALL be bit-identical to those without the setting. Both settings SHALL be loadable and savable through the parameters TOML.

#### Scenario: A corridor's cell means moved along the axis
- **WHEN** a corridor window is optimized with full release along the corridor axis, and the planar cells' means are moved along the axis
- **THEN** the result moves by less than 1 % of what it moves with the release off (the remainder is the cells' normals being off the axes by tenths of a degree)

#### Scenario: Cell means moved across the weak direction
- **WHEN** the same window is optimized with full release along the axis, and the planar cells' means are moved within their planes across the axis (vertically on the walls)
- **THEN** the result still moves (those anchors are kept), clearly more than for the means moved along the axis

#### Scenario: A room keeps its accuracy
- **WHEN** the synthetic room's yaw-drift case is optimized with the release on (any direction)
- **THEN** the trajectory matches the truth as closely as without it

#### Scenario: Off by default
- **WHEN** a parameters TOML without the keys is loaded
- **THEN** both settings are 0 and the odometry gives bit-identical results
