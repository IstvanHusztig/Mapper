## ADDED Requirements

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
