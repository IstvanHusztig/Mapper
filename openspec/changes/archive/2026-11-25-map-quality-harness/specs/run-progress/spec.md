## ADDED Requirements

### Requirement: Silent events in the odometry are reported
When the odometry skips or changes something that the results don't show, it SHALL log a warning naming the window and the event. At the end of the run it SHALL log one summary line with a count for each kind. The events are:
- the sliding reference map is rebuilt;
- a window is dropped for too few points;
- a solver step is skipped;
- a step is applied to only part of a window's nodes ("big jump").

Reporting SHALL NOT change the results.

#### Scenario: The map rebuild fires on recording 44
- **WHEN** recording 44 is processed with `settings/l2-vertical.toml`
- **THEN** the log contains one map-rebuild line with its window number (window 586 at the time of writing), and the summary line counts one rebuild

#### Scenario: Results unchanged
- **WHEN** `44_short` is processed after the change
- **THEN** the point clouds and trajectories are bit-identical with `tools/baseline/44_short.manifest.json`
