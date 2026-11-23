## ADDED Requirements

### Requirement: Wall pairs measured from poses
A pure-Python tool SHALL measure how far apart two visits of the same wall lie, for any trajectory of a recording, on a set
of returns frozen beforehand:
- **Freeze:** from one result (its cloud and trajectory) the tool SHALL select wall pieces and their visits and store, for
  every selected return, its time, its trajectory node and its position relative to that node, together with the piece's
  position and normal, the visit it belongs to and the side it was seen from. The stored set SHALL name the result it was
  chosen on and how many pieces, visits and pairs it holds.
- **Measure:** for a trajectory with the same nodes (a result folder, an odometry snapshot, a loop closure replay) the tool
  SHALL place the stored returns with that trajectory, bring the trajectory into the frame of the earlier visit of each
  pair, and report the gap along the piece's normal for every same-side pair: median, 90th percentile and the number at or
  above a threshold, for all pairs and for stated time classes, and the pairs that cross the threshold between two
  trajectories.
- The tool SHALL refuse a trajectory whose node times differ from the stored set's, and SHALL state that its numbers are
  agreement between visits on a selected set, not accuracy.

#### Scenario: The frozen result reproduces its own cloud
- **WHEN** the set is measured with the trajectory of the result it was frozen on
- **THEN** every return lands where it lies in that result's cloud (within the cloud's stored resolution)

#### Scenario: Three stages of one run
- **WHEN** the set is measured with a run's odometry snapshot, its loop closure replay and its final trajectory
- **THEN** one table gives the gaps at the three stages for the same pairs, and no cloud is read

#### Scenario: Another recording's trajectory
- **WHEN** a trajectory with other node times is given
- **THEN** the tool refuses it with the reason

### Requirement: Short Livox reference dataset
The repository's baselines SHALL include a short Livox dataset: a cut of a Livox recording that contains revisits and
accepted loop closures, with its manifest under `tools/baseline/` and a run in `tools/end_runs.py`. Its comparison SHALL
follow the rules of the other manifests (bit-identical for a change that alters no number).

#### Scenario: A refactor is checked on Livox in minutes
- **WHEN** `tools/end_runs.py` runs after a change that alters no number
- **THEN** the short Livox dataset is run and reported IDENTICAL against its manifest

### Requirement: A run's check reports its timing and the time-limit count
Every run SHALL log the duration of each stage and the number of windows whose iterations stopped at the time limit.
`tools/run_health.py` SHALL report both; a count of zero SHALL be reported as "result independent of the machine's load",
any other count as dependent, and a log without the count as NOT MEASURED.

#### Scenario: A run that never reached the limit
- **WHEN** a run's log reports zero windows stopped at the time limit
- **THEN** `run_health.py` reports the result as independent of load and lists the stage durations

#### Scenario: An older log
- **WHEN** a log without the count is checked
- **THEN** the reading is NOT MEASURED
