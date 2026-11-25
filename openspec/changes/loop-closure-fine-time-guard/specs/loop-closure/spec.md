# Spec Delta

## ADDED Requirements

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
