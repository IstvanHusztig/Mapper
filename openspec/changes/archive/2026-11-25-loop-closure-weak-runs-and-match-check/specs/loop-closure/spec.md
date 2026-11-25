# Spec Delta

## ADDED Requirements

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
