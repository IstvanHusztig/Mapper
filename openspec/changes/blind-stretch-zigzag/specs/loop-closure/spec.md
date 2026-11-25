## MODIFIED Requirements

### Requirement: Fine-time placement of a blind stretch
With `loop_closure_fine_time_placement = true`, after the pose graph's correction has been put onto the nodes, the loop closure SHALL, for every weak run of the odometry readings rule whose sigma is at least `loop_closure_fine_time_min_sigma_m`, compare the scans of each window of the run and of one segment's windows before and after it with the surfaces across the run's axis that windows at least 12 s outside that stretch saw, and SHALL move the nodes of the stretch along the run's axis by a profile of offsets that is zero at both ends of the stretch. Orientations, positions across the axis and every node outside the stretch SHALL stay as the pose graph left them. A window without such surfaces SHALL add no term to the profile and SHALL follow its neighbours along it; the log SHALL give how many such windows moved and their largest offset. With the setting false every result SHALL be bit-identical to the one without this step. The setting SHALL require `loop_closure_odometry_from_readings = true`; otherwise the loop closure does not run and an error names the cause.

With `loop_closure_fine_time_straight_gaps = true` as well, the correction along the run's axis inside the stretch (the node's position after the step minus its odometry position, along the axis) SHALL be one line in time through the pose graph's correction at the two ends of the stretch and, at the middle of every window that has surfaces to compare, the pose graph's correction plus that window's offset; the offsets of those windows SHALL be the ones the profile gives without this setting. Between two such places the correction SHALL be straight, whatever the pose graph placed there. Whether the profile is applied SHALL be decided as without the setting, with the held-out misfit read at the place each window ends up. With `loop_closure_fine_time_straight_gaps = false` every result SHALL be bit-identical to the one before the setting existed. The setting SHALL require `loop_closure_fine_time_placement = true`; otherwise the loop closure does not run and an error names the cause.

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

#### Scenario: A correction that reverses between two windows with surfaces
- **WHEN** straight gaps are on, and between two windows with surfaces to compare the pose graph's correction along the axis rises by three metres and falls back by two
- **THEN** the correction of every node between them lies on the line between the two windows' corrections, and the two windows are where they are without the setting

#### Scenario: Straight gaps without the fine-time placement
- **WHEN** `loop_closure_fine_time_straight_gaps` is true and `loop_closure_fine_time_placement` is false
- **THEN** the loop closure does not run, an error names the cause, and a replay exits with 1

## ADDED Requirements

### Requirement: How far the correction goes back inside a weak run is reported
For every weak run of the odometry readings rule the loop closure SHALL log, and a loop closure replay SHALL write, the largest fall of the correction along the run's axis against the direction in which the correction changes across the run (zero for a correction that never reverses), as the pose graph left it and, for a run the fine-time step moved, after that step. The reading SHALL need no points and SHALL be given with the fine-time settings on or off.

#### Scenario: A correction that rises and falls
- **WHEN** the correction along a weak run's axis grows from 0 to 4.2 m, falls to 2.4 m and grows to 5.8 m across the run
- **THEN** the run's reading is 1.8 m

#### Scenario: A correction that only grows
- **WHEN** the correction along a weak run's axis never decreases across the run
- **THEN** the run's reading is 0
