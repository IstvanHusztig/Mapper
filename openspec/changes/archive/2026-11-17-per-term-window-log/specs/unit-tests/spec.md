## ADDED Requirements

### Requirement: Per-term window table tested on synthetic windows
The unit tests SHALL test the per-term window table without recorded data:
- **Completeness:** the weights and pulls of the rows, summed, SHALL equal those of the window's complete normal
  equations, within a tolerance.
- **Each motion against a known answer:**
  - a guess moved sideways by a known amount: the NDT shift pull sideways points back, between 75 % and 105 % of the
    amount at 1 cm (one linear step: points that change their cell make NDT's cost non-quadratic);
  - a guess stretched along travel: the NDT stretch pull points back, within the same bounds;
  - a trajectory with a constant acceleration against a guess without it: the accelerometer's curve pull is the
    missing end displacement;
  - poses turned against the motion model by a heading ramp: the edge rotations' turn pull about vertical is minus the
    ramp.
- **Directions:** in a corridor, the NDT weight along the corridor SHALL be far below the weight across it.
- **No effect:** the window loop on the synthetic recording SHALL give the same poses, bit for bit, with the table on
  and off.
- **Files:** the CSV output SHALL hold the expected columns and one row per window, state and row.

#### Scenario: The rows add up
- **WHEN** the table is computed on a synthetic window with the accelerometer term, bias estimation and the degeneracy projection on
- **THEN** each motion's summed weight and weighted pull over the rows equal those reduced from the complete normal equations

#### Scenario: A known sideways offset
- **WHEN** the guess of a room window is moved 1 cm sideways from the true poses
- **THEN** the NDT shift pull sideways, minus its value at the true poses, is between −1.05 cm and −0.75 cm
