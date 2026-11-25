# loop-closure Specification

## Purpose

Loop closure after the odometry: which revisits of a place become constraints of the pose graph that corrects the trajectory's drift, and when an accepted constraint is dropped again as a false match.

## Requirements

### Requirement: Further partners for a rejected revisit
Loop closure SHALL group the windows into segments of `loop_closure_segment_windows` windows (6 by default, about 2.4 s). For each segment, loop closure SHALL try the earlier segments that qualify as revisits. On the same floor, an earlier segment qualifies when at least `loop_closure_min_path_m` was walked between the end of the earlier segment and the begin of the later one, and their centres lie within min(`loop_closure_search_radius_m`, `loop_closure_radius_per_path` × that path) horizontally and within the vertical limit. The path SHALL be summed over the first node of each window, counting a step only once the position has moved at least 0.1 m, so that standing still adds no path. Of the qualifying segments it SHALL register, distinct, the first `loop_closure_candidates` (4 by default) of: the nearest one, the one with the longest path between, the nearest at least 3 segments away from the nearest, and the longest-path one at least 3 segments away from the longest-path one. Of the accepted registrations it SHALL keep up to `loop_closure_edges_per_segment` (2 by default) as loop edges: with 2, the best near one (from the two nearest) and the best far one (from the two longest-path) by overlap after the refinement, both only if their earlier segments are at least 3 segments apart, otherwise the better one; with 1, the best one.

#### Scenario: The first partner is a different view
- **WHEN** the registration of a segment against one qualifying earlier segment is rejected, and a further tried segment shows the same place
- **THEN** that segment's registration can still become the loop edge if it passes the acceptance test

#### Scenario: Near and far revisits both kept
- **WHEN** a segment is accepted against a nearby earlier segment and against one far back along the path (the start, after a whole round), and the two earlier segments are at least 3 segments apart
- **THEN** both become loop edges of that segment

#### Scenario: Two tries of the same place
- **WHEN** the best near and the best far registration come from earlier segments less than 3 segments apart
- **THEN** only the one with the higher overlap becomes a loop edge

#### Scenario: A dead end walked out and back
- **WHEN** a segment on the way back from a dead end lies within a few metres of a segment of the way out, with more than `loop_closure_min_path_m` walked in between, however short the time
- **THEN** the earlier segment qualifies as a revisit

#### Scenario: Standing still or walking on
- **WHEN** the person stood still between two segments, or the earlier segment is the next-but-one on a straight walk (centre distance about the path between them)
- **THEN** the earlier segment does not qualify

### Requirement: False loop edges are dropped
Every loop edge, on the same floor and between floors, SHALL be weighted by its registration's information mapped into the edge's relative pose and scaled so that its best-determined translation has the sigma `loop_closure_edge_sigma_m` (between floors without the height); a same-floor edge whose registration information is unusable SHALL keep the fixed sigmas `loop_closure_edge_sigma_m` / `_deg`, and a cross-floor edge whose information is unusable SHALL NOT be used (the next candidate is tried). After the pose graph is solved, loop closure SHALL compute, for every loop edge, how far the solved relative pose of its two segments is from the edge's measurement (translation and rotation, and for an information-weighted edge its weighted residual). While any edge exceeds the limits `loop_closure_max_residual_m` / `loop_closure_max_residual_deg` (an information-weighted edge by its weighted residual), it SHALL drop the one edge among those over the limits whose removal lowers the cost of the re-solved graph the most, and solve the graph again. It SHALL log each dropped edge with its residual and the residuals of the edges kept. If no edge is left, the trajectory SHALL stay unchanged.

#### Scenario: One false edge among consistent ones
- **WHEN** one accepted registration is off by a metre while the other loop edges agree with each other
- **THEN** that edge is dropped, and the graph solved without it agrees with the remaining edges within centimetres

#### Scenario: A false edge strains a right one
- **WHEN** a false edge and a right edge it strains are both over the limits, the right one by more
- **THEN** the false edge is dropped, because removing it frees more of the graph's cost

#### Scenario: Consistent edges
- **WHEN** all loop edges agree with the solved graph within the limits
- **THEN** no edge is dropped, and the result is that of the graph with all accepted edges

### Requirement: Acceptance does not depend on the world origin
The size of a loop correction used to accept or reject a candidate SHALL be the movement of the segment it moves, measured at the segment's centre. The same holds for the plausibility check of the pose graph, measured at each node. Rotating a segment about a point far from the world origin SHALL NOT count as a large shift.

#### Scenario: The same loop far from the start
- **WHEN** the synthetic loop recording is processed once as recorded and once with every pose and point shifted by +200 m
- **THEN** the same loop edges are accepted and the corrections agree within numerical precision

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
