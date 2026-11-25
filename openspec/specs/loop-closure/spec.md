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

### Requirement: Odometry steps weighted from the odometry's readings
With `loop_closure_odometry_from_readings = true`, the loop closure pose graph SHALL take the uncertainty of each odometry step from the windows' readings: consecutive windows whose LiDAR translation information ratio is below `loop_closure_odometry_weak_ratio`, which are not at rest and whose weak axes agree within `loop_closure_odometry_axis_tolerance_deg` (one other window in between does not end the run) form a weak run; the run's sigma is the root sum of squares of `0.5 x loop_closure_odometry_accelerometer_error_mps2 x T^2` and of the sum over its windows of |placed speed along the axis - reference pace| x duration, the reference pace being the line from the pace before the run to the pace after it; that variance SHALL be shared among the odometry steps in proportion to the run's time in each and added along the run's axis to the step's translation covariance, everything else of the step staying at `loop_closure_odometry_sigma_m` / `_deg`. The same step weights SHALL be used by the solve, the cost and the pruning. Each run SHALL be logged with its time, duration, axis and sigma. With the setting false the graph SHALL be the one with fixed step sigmas, bit for bit.

#### Scenario: A blind stretch takes the correction
- **WHEN** the odometry placed 9 s of a corridor with almost no LiDAR information along it and far slower than the pace before and after, and loop edges at the corridor's mouth show metres of travel missing
- **THEN** the missing travel is placed in the steps of that stretch, and the segments before it move by centimetres

#### Scenario: No weak window
- **WHEN** no window of the recording is weak
- **THEN** every step has the fixed sigmas and the result equals the one with the setting false

#### Scenario: Readings missing
- **WHEN** the setting is true and the windows carry no readings (an odometry snapshot or loop capture written before the readings existed)
- **THEN** the loop closure does not run, an error names the cause, and a replay exits with 1

### Requirement: Match check by held-out innovation
With `loop_closure_match_check = true`, after the pruning the loop closure SHALL test every remaining loop edge by what adding it costs the graph solved without it and without the edges that reuse its scans (the same source segment, a target within 2 segments, or one edge's source within 1 segment of the other's target): the cost of that graph with the edge minus the cost without it. While the largest such value exceeds the chi-square quantile `loop_closure_match_check_quantile` for the rank of the edge's information, that edge SHALL be dropped, the pruning run again and the test repeated. Each drop SHALL be logged with its value, limit, the number of edges held out and the edge's innovation. The match check SHALL require `loop_closure_odometry_from_readings = true`; otherwise the loop closure does not run and an error names the cause.

#### Scenario: An influential false edge
- **WHEN** a false edge has bent the graph so that its residual after the solve is within the pruning limit, while the graph without it places the two segments 2 m elsewhere
- **THEN** the match check drops it

#### Scenario: True edges across a blind stretch
- **WHEN** several edges at a corridor's mouth agree that a blind stretch lost metres of travel
- **THEN** none of them is dropped, because the graph without them allows that travel in the stretch

#### Scenario: An edge no other loop reaches
- **WHEN** no independent loop edge constrains the two segments of an edge
- **THEN** the edge is kept, and its test value is small because the prediction without it is wide

### Requirement: The readings travel with the windows
Every solved window SHALL carry the readings the weighting needs (information ratio, weak axis, at rest, placed move, duration), with or without `log_window_terms`, and the odometry snapshot and the loop closure capture SHALL store them, so that a run from a snapshot and a loop closure replay give the result of the full run. Files written before the readings existed SHALL still load, with the readings absent.

#### Scenario: Replay with the rules on
- **WHEN** a loop closure capture written with readings is replayed with its own settings and both rules on
- **THEN** every node comes out as in the full run with those settings

#### Scenario: An older capture, rules off
- **WHEN** a capture written before the readings existed is replayed with its own settings
- **THEN** every node comes out bit for bit as before

### Requirement: Fine-time placement of a blind stretch
With `loop_closure_fine_time_placement = true`, after the pose graph's correction has been put onto the nodes, the loop closure SHALL, for every weak run of the odometry readings rule whose sigma is at least `loop_closure_fine_time_min_sigma_m`, compare the scans of each window of the run and of one segment's windows before and after it with the surfaces across the run's axis that windows at least 12 s outside that stretch saw, and SHALL move the nodes of the stretch along the run's axis by a profile of offsets that is zero at both ends of the stretch. Orientations, positions across the axis and every node outside the stretch SHALL stay as the pose graph left them. A window without such surfaces SHALL add no term to the profile and SHALL follow its neighbours along it; the log SHALL give how many such windows moved and their largest offset. With the setting false every result SHALL be bit-identical to the one without this step. The setting SHALL require `loop_closure_odometry_from_readings = true`; otherwise the loop closure does not run and an error names the cause.

#### Scenario: Travel placed at the wrong time inside a blind stretch
- **WHEN** the pose graph restored the travel a blind stretch had lost, but placed scans inside the stretch more than a metre along the passage from where another visit saw the same wall
- **THEN** those scans move along the passage onto that wall to within one step of the offset grid (0.10 m), and the nodes before and after the stretch do not move

#### Scenario: Nothing to see across the passage
- **WHEN** no window of an eligible run has a surface across its axis that another visit saw
- **THEN** no node moves, and the run is reported as not timed with its time, axis and sigma

#### Scenario: No eligible run
- **WHEN** no weak run reaches `loop_closure_fine_time_min_sigma_m`
- **THEN** the result equals the one with the setting false

#### Scenario: Readings rule off
- **WHEN** the setting is true and `loop_closure_odometry_from_readings` is false
- **THEN** the loop closure does not run, an error names the cause, and a replay exits with 1

### Requirement: A profile is applied only when it is supported
The profile of a run SHALL be applied only if at least one window of the run itself has surfaces to compare, the misfit of the scan points held out of the fit is lower with the profile than without it, and no loop edge is contradicted by it. Otherwise the run's nodes SHALL stay exactly as the pose graph left them and the cause SHALL be logged. The step SHALL never remove or reweight a loop edge. When the stretches of two eligible runs overlap, both SHALL be left as they are and reported.

#### Scenario: Held-out points do not improve
- **WHEN** a fitted profile does not lower the misfit of the held-out scan points
- **THEN** the run stays as the pose graph left it, and the log names the held-out check as the cause

#### Scenario: Repeating structure
- **WHEN** the passage has equal door frames every 2 m, the run's sigma lets the search reach more than one spacing, and the scans are in the right place
- **THEN** no window is moved by more than one step of the offset grid

### Requirement: Loop edges read at the times of their information
Every registration SHALL record how much of its translation information each window of its source segment gave. For a loop edge with a source window in an eligible run's stretch, the loop closure SHALL compute the edge's residual along the run's axis weighted over its source windows by that information (its time-support residual), before and after the profile, and SHALL class the edge: unconfirmed when less than half of its information along the axis comes from windows that are outside the stretch or have surfaces to compare; otherwise agreeing when the residual after the profile is within the pruning's limit read along the axis or no larger than before; otherwise contradicted. A contradicted edge SHALL stop the profile of that run. An edge whose target segment's first pose lies in the stretch SHALL be read there with the profile's offset and judged by the same limit. The residual at the first pose, the time-support residual before and after, the informed share and the class SHALL be logged for every such edge, also when the run is left as it is. A loop closure replay SHALL also write the time-support residual of every kept edge with a source window in any weak run, with the setting true or false, and with the setting true a table of every window of each stretch (time, offset, whether it had surfaces to compare, scores before and after).

#### Scenario: An edge whose information lies late in its segment
- **WHEN** the profile moves the first pose of an edge's source segment by a metre, and the windows that carry most of the edge's information along the passage are not moved
- **THEN** the edge agrees, the profile is applied, and the log shows both residuals

#### Scenario: An edge from scans that see nothing across the passage
- **WHEN** all source windows of an edge lie in the stretch and none has a surface across the axis
- **THEN** the edge is reported as unconfirmed and does not stop the profile

#### Scenario: Informative scans contradict the profile
- **WHEN** an edge with most of its information in windows that have surfaces to compare is further from its registration after the profile than the limit and than before
- **THEN** the run stays as the pose graph left it, and the log names the edge

### Requirement: The capture carries what a replay of the fine-time step needs
The loop closure capture SHALL store each registration's information per source window and, for every eligible run of a run made with the setting on, the score curves of its windows together with the pose graph result and the set of eligible runs they were computed for. A replay with the capture's own settings SHALL give every node of the run bit for bit, the fine-time move included. When the setting is true and the curves are missing or belong to another pose graph result or another set of eligible runs, the replay SHALL not apply the loop closure, SHALL name the cause and SHALL exit with 1. Captures written before this format SHALL still load and replay with the setting false as before.

#### Scenario: Replay with the capture's settings
- **WHEN** a capture written with the setting on is replayed with its own settings
- **THEN** every node comes out as in the run

#### Scenario: Another graph result
- **WHEN** a replay changes a loop closure setting so that the pose graph result differs from the captured one, with the fine-time setting true
- **THEN** nothing is written, the replay names the stale curves as the cause and exits with 1

#### Scenario: Another set of eligible runs
- **WHEN** a replay sets `loop_closure_fine_time_min_sigma_m` so that other runs are eligible than in the capturing run, with the fine-time setting true
- **THEN** nothing is written, the replay names the cause and exits with 1

#### Scenario: An older capture
- **WHEN** a capture of the earlier format is replayed with the fine-time setting false
- **THEN** every node comes out bit for bit as before

### Requirement: Loop closure capture and replay
With `capture_loop_closure = true` (`[lidar_odometry_debug_info]`, outside the settings hash, off by default) a
command-line run, also one made with `--from-snapshot`, SHALL write `loop_closure_capture.bin` to its output folder. It
SHALL hold the windows as the odometry left them, the segments (pose, times, centre, path), and for every registration
the loop closure tried: the two segments, whether it was a cross-floor try, the overlap before and after, the transform,
the registration's information matrix, the shift and angle of the correction, the information ratio where one was
computed, and the centre of the registered points. The capture SHALL NOT change the run's results.

`dMapper --replay-loop-closure <capture> <params.toml> <output_folder>` SHALL run the loop closure from the capture without
the recording and without registering anything: the choice of the tries, their acceptance, the choice of edges, the pose
graph, the pruning, and the correction of every node. It SHALL write the corrected trajectory in the result's trajectory
format and the loop closure's log lines.

- **With the settings the capture was made with:** every node pose SHALL come out bit for bit as the run's loop closure
  left it; otherwise the replay SHALL exit non-zero.
- **With other loop closure settings:** the replay SHALL use the captured registrations. If the changed settings ask for a
  registration the capture does not hold (another candidate, another search range, another segment length), the replay
  SHALL name the missing tries and exit non-zero without writing a trajectory.
- Settings that change the odometry or the registration itself (segment length, voxel sizes, the registration's
  statistics) SHALL be refused, with the setting named.

#### Scenario: Own settings reproduce the run
- **WHEN** a capture is replayed with the settings of the run that wrote it
- **THEN** the node poses equal those after the run's loop closure bit for bit and the exit code is 0

#### Scenario: A stricter acceptance in seconds
- **WHEN** a capture is replayed with a higher `loop_closure_min_fitness`
- **THEN** only the captured tries that still pass become edges, the graph is solved again, and the trajectory is written without the recording being read

#### Scenario: A try that was never registered
- **WHEN** changed settings select a candidate pair that the capturing run did not register
- **THEN** the replay names the pair, writes no trajectory and exits non-zero

#### Scenario: Normal runs are unchanged
- **WHEN** a run is made with `capture_loop_closure = false`
- **THEN** its results are bit-identical to those before this change
