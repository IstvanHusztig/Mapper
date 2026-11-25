## ADDED Requirements

### Requirement: A run states the limits it was judged with
Every run, replay and cancelled run SHALL log, once, the limits its window verdict used (bend, severe bend, speed, gyro
bias, share of bent windows, the chi-square median and 95th percentile, the information ratio that selects the weak
windows). The tool that judges a run from its log SHALL take its limits from that line and SHALL say in its output where
its limits came from: the run's log, its own defaults (a log without the line), or the command line. The tool's defaults
SHALL equal the program's, and a test SHALL fail when they differ or when the program's limits cannot be read.

#### Scenario: A log of a current build
- **WHEN** the tool judges a log that holds the limits line
- **THEN** it judges with those limits and its output says the limits are the run's own

#### Scenario: An older log
- **WHEN** the tool judges a log without the limits line
- **THEN** it judges with its defaults and its output says so

#### Scenario: The two sets drift apart
- **WHEN** a limit of the program's verdict is changed and the tool's default is not
- **THEN** the test that compares them fails and names the limit
