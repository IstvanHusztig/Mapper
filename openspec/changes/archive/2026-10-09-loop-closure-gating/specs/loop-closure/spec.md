# Spec Delta

## Purpose

Loop closure after the odometry: which revisits of a place become constraints of the pose graph that corrects the trajectory's drift, and when an accepted constraint is dropped again as a false match.

## ADDED Requirements

### Requirement: Further partners for a rejected revisit
For each segment, loop closure SHALL try the earlier segments that qualify as revisits (far enough apart in time, within the search radius horizontally and the vertical limit), nearest first. When a registration is rejected it SHALL try the next one, up to `loop_closure_candidates` registrations per segment, and SHALL stop at the first accepted one. With `loop_closure_candidates = 1` only the nearest partner is tried.

#### Scenario: The nearest partner is a different view
- **WHEN** the registration of a segment against its nearest earlier segment is rejected, and a further qualifying earlier segment shows the same place
- **THEN** that segment is registered as well, and its loop edge is accepted if the registration passes the acceptance test

#### Scenario: The nearest partner is accepted
- **WHEN** the registration against the nearest earlier segment is accepted
- **THEN** no further partner is tried for that segment

### Requirement: False loop edges are dropped
After the pose graph is solved, loop closure SHALL compute, for every loop edge, how far the solved relative pose of its two segments is from the edge's measurement (translation and rotation). While the largest of these, relative to `loop_closure_max_residual_m` and `loop_closure_max_residual_deg`, exceeds the limit, it SHALL drop that one edge and solve the graph again. It SHALL log each dropped edge with its residual and the residuals of the edges kept. If no edge is left, the trajectory SHALL stay unchanged.

#### Scenario: One false edge among consistent ones
- **WHEN** one accepted registration is off by a metre while the other loop edges agree with each other
- **THEN** that edge has the largest residual, is dropped, and the graph solved without it agrees with the remaining edges within centimetres

#### Scenario: Consistent edges
- **WHEN** all loop edges agree with the solved graph within the limits
- **THEN** no edge is dropped, and the result is that of the graph with all accepted edges
