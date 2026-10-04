## Purpose

How the cells of the NDT reference maps (odometry map, loop closure, consistency) are built from points: each cell's mean, covariance and surface normal, and the switch back to the previous statistics.

## ADDED Requirements

### Requirement: Cell statistics describe all of the cell's points
With exact cell statistics enabled, each reference-map cell SHALL hold the mean of all points added to it and their covariance about that mean, with a weak isotropic prior worth one point so that cells with one or two points stay usable. The cell's surface normal SHALL be the direction of the smallest spread of that covariance, recomputed whenever the cell receives points, and oriented towards the viewpoint of the points that built it. A cell whose two smallest spreads are too close to tell apart SHALL have no normal. The result SHALL NOT depend on the order in which a cell's points are added, beyond floating-point rounding.

#### Scenario: Points on a plane, inserted along lines
- **WHEN** points sampled on a flat surface are added to an empty map in grid order, line by line
- **THEN** every cell with at least 10 points has its mean within 1 mm of the centroid of its points, and, if its points span an area rather than lie on one line, its normal within 5° of the surface normal

#### Scenario: Points on one line
- **WHEN** a cell's points lie along one line, so that their spread is the same in both directions across it
- **THEN** the cell has no normal (a zero normal), while its mean and covariance are kept

#### Scenario: Insertion order
- **WHEN** the same points are added once in grid order and once in random order
- **THEN** each cell's mean, covariance and normal agree between the two maps within floating-point rounding

#### Scenario: A degenerate corridor
- **WHEN** a window is registered in a synthetic corridor with no end walls
- **THEN** the translation information reported by the scan-to-map optimizer is at least 100 times weaker along the corridor than across it

### Requirement: Points from the far side of a surface are kept apart
A point SHALL NOT be added to a cell whose current normal faces away from the point's viewpoint, so that the two sides of a thin wall don't mix in one cell.

#### Scenario: A thin wall seen from both sides
- **WHEN** a cell already holds points of a wall seen from one side, and points of the other side, seen from the other side, fall into the same cell
- **THEN** the cell's mean and normal stay those of the first side

### Requirement: Switch to the previous statistics
A setting SHALL select between the exact cell statistics and the previous ones (first point as mean, normal fixed from the first three points). With the previous statistics selected, the results SHALL be bit-identical to those of the version before this change.

#### Scenario: Previous statistics selected
- **WHEN** the reference recording is processed with the switch set to the previous statistics
- **THEN** the output is identical with the baseline made before this change
