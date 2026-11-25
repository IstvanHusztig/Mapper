# Spec Delta

## ADDED Requirements

### Requirement: Cross-floor revisits
With `loop_closure_cross_floor_max_vertical_m` greater than `loop_closure_max_vertical_offset_m`, loop closure SHALL also try, for each segment, earlier segments whose centre lies between those two heights above or below it, at least `loop_closure_cross_floor_min_time_gap_s` apart in time and within the horizontal search radius, nearest first, up to `loop_closure_candidates` registrations. It SHALL add at most one such edge per segment, in addition to any same-floor edge. Such a pair SHALL be registered only on the horizontal footprint both segments cover, with the search limited to `loop_closure_cross_floor_search_m`. The edge SHALL be accepted only with an overlap of at least `loop_closure_cross_floor_min_fitness`, no worse than before registration, a translation correction of at most 1 m and a rotation correction of at most `loop_closure_cross_floor_max_angle_deg`. It SHALL constrain the relative pose as weighted by its registration's information (change `information-weighted-loop-edges`). Each tried cross-floor registration SHALL be logged as such, with a count of tries and accepted edges. With `loop_closure_cross_floor_max_vertical_m` = 0 no cross-floor candidate SHALL be tried.

#### Scenario: Two floors linked through a stairwell
- **WHEN** a segment on one floor and an earlier segment on the floor below both see the same stairwell shaft, and their registration passes the cross-floor acceptance test
- **THEN** a cross-floor loop edge links them, and the stairwell's railing seen from both floors is drawn once (offset between the floors within a few centimetres)

#### Scenario: The floor just above, a few seconds earlier
- **WHEN** a segment on the lowest floor sees the shaft 10–20 s after a segment on the floor above saw it
- **THEN** that pair is a cross-floor candidate, although it is closer in time than `loop_closure_min_time_gap_s`

#### Scenario: Off by default
- **WHEN** `loop_closure_cross_floor_max_vertical_m` = 0
- **THEN** the loop edges and the result are those of same-floor loop closure alone, bit for bit

### Requirement: Loop edges are weighted by a robust kernel
With `loop_closure_kernel_scale` > 0, the pose graph SHALL weight every loop edge by a robust kernel of its residual (in units of the edge's sigmas). An edge that agrees with the solution SHALL keep its full weight, and the weight SHALL fall smoothly as the disagreement grows. The disagreement SHALL be judged from the plain least-squares solution, not from the odometry's poses, so that loops correcting the drift are not taken for outliers. Odometry edges SHALL keep their full weight. With `loop_closure_kernel_scale` = 0 the pose graph SHALL be plain weighted least squares.

#### Scenario: A slightly wrong edge among consistent ones
- **WHEN** a stretch is covered by several loop edges, one of them off by 15 cm (below the drop limit) while the others agree with each other
- **THEN** with the kernel that edge moves the solution by clearly less than it does without the kernel, and the solution is within a few millimetres of the one with that edge right

#### Scenario: Consistent edges
- **WHEN** all loop edges agree with each other
- **THEN** the solution with the kernel equals the solution without it within millimetres

### Requirement: Angle residuals are wrapped in the pose-graph solve
The pose-graph solve SHALL wrap every edge's angle residuals into (−180°, 180°], as its cost does.

#### Scenario: A revisit walked in the opposite direction
- **WHEN** a loop edge's relative yaw is near ±180°
- **THEN** the solve converges as for any other edge (no residual of 360° after a step)

## MODIFIED Requirements

### Requirement: False loop edges are dropped
After the pose graph is solved, loop closure SHALL compute, for every loop edge, how far the solved relative pose of its two segments is from the edge's measurement (translation and rotation). While the largest of these, relative to `loop_closure_max_residual_m` and `loop_closure_max_residual_deg`, exceeds the limit, it SHALL drop that one edge and solve the graph again. For an information-weighted (cross-floor) edge its weighted residual SHALL count instead (change `information-weighted-loop-edges`). It SHALL log each dropped edge with its residual and the residuals of the edges kept, marking cross-floor edges. If no edge is left, the trajectory SHALL stay unchanged.

#### Scenario: One false edge among consistent ones
- **WHEN** one accepted registration is off by a metre while the other loop edges agree with each other
- **THEN** that edge has the largest residual, is dropped, and the graph solved without it agrees with the remaining edges within centimetres

#### Scenario: Consistent edges
- **WHEN** all loop edges agree with the solved graph within the limits
- **THEN** no edge is dropped, and the result is that of the graph with all accepted edges
