# Spec Delta

## ADDED Requirements

### Requirement: Point-to-plane NDT terms in planar cells
With `ndt_point_to_plane = true`, the scan-to-map optimization SHALL use, for each point matched to a planar cell, only the point's distance from the cell's plane along the cell's normal, weighted by the inverse of the cell's variance along that normal (floored at the sensor noise). A point's position within the plane SHALL NOT enter its term. A cell SHALL count as planar when the spread of its points across the plane is clearly smaller than within it. Points matched to cells that are not planar (edges, poles, round railings, clutter) SHALL keep the point-to-distribution term. With `ndt_point_to_plane = false`, which is the built-in default, the results SHALL be bit-identical to those without the setting. The setting SHALL be loadable and savable through the parameters TOML. The result SHALL NOT depend on the number of threads.

#### Scenario: A wall cell's mean elsewhere on the wall
- **WHEN** the mean of a planar cell is moved within its plane (along the wall) and the window is optimized again
- **THEN** with point-to-plane on the result is unchanged within rounding, and with it off the result moves

#### Scenario: Room accuracy kept
- **WHEN** the synthetic room of the scan-to-map tests is optimized with point-to-plane on
- **THEN** the trajectory matches the truth as closely as with point-to-distribution

#### Scenario: Off by default
- **WHEN** a parameters TOML without the key is loaded
- **THEN** `ndt_point_to_plane` is false and the odometry gives bit-identical results
