## Purpose

> The residual-floor candidate was rejected on 2026-11-13. These evaluation
> principles are retained as historical requirements; they do not authorize
> more runs or imply successful validation. See ../../rejection.md.
Keep confidence-model experiments reproducible and protect successful indoor mapping while diagnosing open-space failures without unsupported acceptance claims.

## ADDED Requirements

### Requirement: Control evidence precedes candidate conclusions
Evaluation SHALL identify original inputs, executable/source, configuration, support and initialization, and SHALL require control reproduction before interpreting a restored-checkpoint intervention. Known causal results SHALL be reused. Missing later trace/state SHALL be explicitly reported. Each new processing/checkpoint run SHALL require user authorization; applying this change alone SHALL NOT grant it.

#### Scenario: Later failure lacks detailed trace
- **WHEN** a proposed causal explanation requires state not captured in existing evidence
- **THEN** evaluation reports the missing state and proposes a scoped capture without starting it automatically

#### Scenario: Checkpoint control differs
- **WHEN** restored control fails its declared reproduction gate
- **THEN** candidate results are not interpreted as an isolated causal intervention

### Requirement: Regularization evidence separates scaling from improvement
Evaluation SHALL justify the selected regularization scale using declared physical support, uncertainty limitations and relevance to the influential cells. It SHALL NOT treat stationary plane spread as a uniquely identified total residual noise model or select the scale by endpoint tuning. Fixed-state cost reductions across changed scales SHALL NOT establish improved geometry. Evaluation SHALL inspect mixed-axis robust influence and joint bias/velocity/offset estimates and covariance carry, preserving configured prior models/parameters and controlled initialization.

#### Scenario: Supported scale does not affect the proposed mechanism
- **WHEN** the defensible floor range leaves the implicated cell eigenvalues unchanged
- **THEN** evaluation records the incompatibility before implementation instead of enlarging the floor solely to improve the failing checkpoint

#### Scenario: Same prior configuration produces different carry
- **WHEN** changed NDT information changes the joint solve and outgoing uncertainty
- **THEN** evaluation checks units, finite/SPD status, innovations and subsequent carry and does not claim unchanged or calibrated posterior uncertainty

### Requirement: General acceptance protects both environment classes
Evaluation SHALL distinguish odometry and final outputs, retain coverage and failures, and measure trajectory behavior, geometry, revisit consistency, sharpness and robustness before recommending general acceptance. Indoor success alone SHALL NOT establish outdoor causality or cave suitability. Accepted general changes SHALL be documented in existing HTML guidance, with detailed recording evidence retained in OpenSpec.

#### Scenario: One local improvement
- **WHEN** a candidate reduces the known checkpoint bend but has not passed longer and full-route tests
- **THEN** it is reported as local evidence rather than a general correction

#### Scenario: Missing or regressed preservation evidence
- **WHEN** indoor geometry/coverage regresses or required perturbation evidence is missing
- **THEN** the evaluation does not declare general acceptance from a better endpoint alone
