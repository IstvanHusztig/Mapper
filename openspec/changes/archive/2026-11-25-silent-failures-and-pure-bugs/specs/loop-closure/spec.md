## ADDED Requirements

### Requirement: Acceptance does not depend on the world origin
The size of a loop correction used to accept or reject a candidate SHALL be the movement of the segment it moves, measured at the segment's centre. The same holds for the plausibility check of the pose graph, measured at each node. Rotating a segment about a point far from the world origin SHALL NOT count as a large shift.

#### Scenario: The same loop far from the start
- **WHEN** the synthetic loop recording is processed once as recorded and once with every pose and point shifted by +200 m
- **THEN** the same loop edges are accepted and the corrections agree within numerical precision
