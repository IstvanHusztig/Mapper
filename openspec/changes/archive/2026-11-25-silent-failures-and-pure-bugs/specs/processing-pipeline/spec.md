## ADDED Requirements

### Requirement: Optimizer steps are applied completely or not at all
An iteration of the window optimizer or of consistency SHALL either apply its whole step to every unknown, or apply nothing and report it. A step SHALL NOT be dropped because one unknown's step is exactly zero. A step SHALL NOT be applied to only part of the nodes.

#### Scenario: An unknown with a zero step
- **WHEN** the solution of an iteration has an entry that is exactly zero (for example an extra unknown that nothing constrains)
- **THEN** the other unknowns' steps are applied, and no step is reported as skipped

#### Scenario: A node step larger than 1 m
- **WHEN** one node's step in an iteration exceeds 1 m
- **THEN** no node, bias, velocity or offset of that window changes in that iteration, and the event is counted as a big jump

### Requirement: Output poses keep full precision
The chunk poses in `session_poses.mrp` and `.mri` SHALL be written with enough digits to round-trip a double.

#### Scenario: Far from the origin
- **WHEN** a chunk pose has a translation of 1234.56789 m
- **THEN** the value read back from the file equals the one written
