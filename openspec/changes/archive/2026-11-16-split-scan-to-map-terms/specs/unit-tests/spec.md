## ADDED Requirements

### Requirement: Window optimization terms tested one by one
The unit tests SHALL test each term of the window optimization on its own, on a synthetic window, without recorded data:
- **The cost relation:** for every term with a cost, the right-hand side the term adds SHALL equal minus half the
  numeric gradient of its cost, over every unknown the term touches. For a term whose residual is linear in the
  unknowns, the matrix diagonal SHALL equal half the numeric second derivative.
- **The NDT point terms:** checked with the degeneracy projection off, with and without the Huber kernel and the
  information scale.
- **The degeneracy projection:** it SHALL pass the node blocks unchanged when every direction is strong, remove a weak
  translation direction from them, and add nothing when it is off.
- **The origin fix:** it SHALL weigh only the first node's six parameters.

#### Scenario: A term's gradient
- **WHEN** the motion-model edges are built on a turning window whose poses differ from the motion model, with bias estimation on
- **THEN** each pose parameter's and each bias component's right-hand side equals minus half the central-difference gradient of the edges' cost

#### Scenario: The degeneracy projection
- **WHEN** the window's normal information has a share of 0.05 along z, below `degenerate_projection_ratio` 0.1
- **THEN** each node block B enters the matrix as M B M and its right-hand side g as M g, with M removing the z translation
