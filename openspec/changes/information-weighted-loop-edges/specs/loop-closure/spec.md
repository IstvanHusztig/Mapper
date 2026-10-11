# Spec Delta

## ADDED Requirements

### Requirement: Loop edges weighted by their registration's information
Every accepted cross-floor loop edge SHALL be weighted in the pose graph by an information matrix over its full relative-pose residual, derived from its registration's information at the converged pose rather than from fixed sigmas. Directions that the shared geometry determines well SHALL be tied, and directions it leaves undetermined SHALL stay (nearly) free. The matrix SHALL be scaled so that the best-determined translation direction has the sigma `loop_closure_edge_sigma_m`. The world-vertical move of the shared region SHALL carry no information (the height of a cross-floor edge comes from the odometry). An edge whose registration information can't be used SHALL NOT be added (the next candidate is tried). Every such edge SHALL be logged with the sigmas of its principal directions at its shared region. The robust kernel SHALL act on the edge's residual in these units. Same-floor edges SHALL keep the fixed sigmas. With `loop_closure_cross_floor_max_vertical_m` = 0 there are no such edges, and the result SHALL be that of same-floor loop closure alone.

#### Scenario: No cross-floor loops
- **WHEN** `loop_closure_cross_floor_max_vertical_m` = 0
- **THEN** the result equals that of same-floor loop closure bit for bit

#### Scenario: Equivalent to the fixed sigmas for an isotropic registration
- **WHEN** an edge's information matrix equals the diagonal of the fixed loop sigmas
- **THEN** the pose-graph solution equals the one with the fixed-sigma edge within numerical precision

#### Scenario: A shaft leaves its axis free
- **WHEN** a cross-floor registration's shared region is a vertical cylinder (a round shaft)
- **THEN** the edge corrects the horizontal offset between the floors and their tilt, but neither their height nor their relative turn about the shaft's axis (both stay as the odometry has them, within a few millimetres and hundredths of a degree)

#### Scenario: A box-shaped region ties the rotation
- **WHEN** a cross-floor registration's shared region holds walls in two horizontal directions (a lift shaft, a room corner)
- **THEN** the edge also corrects the relative turn between the floors

### Requirement: Information-weighted edges are dropped by their weighted residual
For an information-weighted loop edge, the false-edge test SHALL use the edge's residual weighted by its information matrix, expressed in metres of `loop_closure_edge_sigma_m` (the square root of the weighted squared residual, times that sigma), against `loop_closure_max_residual_m`. A residual in a direction the edge leaves free SHALL NOT count. The log of kept and dropped edges SHALL show this residual for such edges.

#### Scenario: A residual along the free direction
- **WHEN** an information-weighted cross-floor edge disagrees with the solved graph only in height, or in the turn about a round shaft's axis
- **THEN** it is not dropped

#### Scenario: A residual along a tied direction
- **WHEN** an information-weighted edge disagrees with the solved graph by more than `loop_closure_max_residual_m` in a well-determined horizontal direction
- **THEN** it is dropped as a false edge, as a fixed-sigma edge would be
