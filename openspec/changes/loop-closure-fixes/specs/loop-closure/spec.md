## MODIFIED Requirements

### Requirement: Further partners for a rejected revisit
For each segment, loop closure SHALL try the earlier segments that qualify as revisits, nearest first. On the same floor, an earlier segment qualifies when at least `loop_closure_min_path_m` was walked between the end of the earlier segment and the begin of the later one, and their centres lie within min(`loop_closure_search_radius_m`, `loop_closure_radius_per_path` × that path) horizontally and within the vertical limit. The path SHALL be summed over the first node of each window, so that standing still adds no path. When a registration is rejected it SHALL try the next one, up to `loop_closure_candidates` registrations per segment, and SHALL stop at the first accepted one. With `loop_closure_candidates = 1` only the nearest partner is tried.

#### Scenario: The nearest partner is a different view
- **WHEN** the registration of a segment against its nearest earlier segment is rejected, and a further qualifying earlier segment shows the same place
- **THEN** that segment is registered as well, and its loop edge is accepted if the registration passes the acceptance test

#### Scenario: The nearest partner is accepted
- **WHEN** the registration against the nearest earlier segment is accepted
- **THEN** no further partner is tried for that segment

#### Scenario: A dead end walked out and back
- **WHEN** a segment on the way back from a dead end lies within a few metres of a segment of the way out, with more than `loop_closure_min_path_m` walked in between, however short the time
- **THEN** the earlier segment qualifies as a revisit

#### Scenario: Standing still or walking on
- **WHEN** the person stood still between two segments, or the earlier segment is the next-but-one on a straight walk (centre distance about the path between them)
- **THEN** the earlier segment does not qualify
