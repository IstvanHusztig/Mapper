## Purpose

How a processing result is judged before a change to the odometry, the map, loop closure or consistency is accepted. It uses three map-quality measures of equal weight, the spread between near-identical inputs, and the noise floor.

## ADDED Requirements

### Requirement: Three measures of equal weight
A result SHALL be judged on three measures, and no measure ranks above another:
- **global geometry:** on recording 44, the angle between the gallery façade seen from the start room and seen along the gallery; the building plan makes them parallel;
- **consistency:** walls seen on two passes land on each other, measured as the heading and offset at every revisit of the final map;
- **sharpness:** the railing nearest-neighbour check and the stairwell checks of `tools/evaluate_floors.py`, and the pose jitter.

#### Scenario: Known numbers are reproduced
- **WHEN** the harness is run on the results of 2026-10-11/12 (`44_wd_s0` final, `44_odo_h0_s0` odometry only)
- **THEN** it reports:
  - the façade angle: about −0.4° for the final map and about −3.9° for the odometry-only result;
  - the railing: 4.9 cm;
  - the jitter p99: 4.5° against 1.3°.

### Requirement: Spread and noise floor
A change SHALL be measured on the inputs s0, s1 and s2: recording 44 with 0, 1 and 2 leading IMU samples dropped. Its effect counts only where it exceeds the spread of the unchanged code on the same inputs, and a change SHALL NOT increase that spread.

#### Scenario: One table for a set of results
- **WHEN** the harness is given three result folders for s0, s1 and s2
- **THEN** it prints each measure per input, and the spread between them, in one table
