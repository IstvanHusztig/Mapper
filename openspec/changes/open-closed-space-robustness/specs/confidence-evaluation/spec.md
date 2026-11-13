## Purpose
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

### Requirement: General acceptance protects both environment classes
Evaluation SHALL distinguish odometry and final outputs, retain coverage and failures, and measure trajectory behavior, geometry, revisit consistency, sharpness and robustness before recommending general acceptance. Indoor success alone SHALL NOT establish outdoor causality or cave suitability. Accepted general changes SHALL be documented in existing HTML guidance, with detailed recording evidence retained in OpenSpec.

#### Scenario: One local improvement
- **WHEN** a candidate reduces the known checkpoint bend but has not passed longer and full-route tests
- **THEN** it is reported as local evidence rather than a general correction

#### Scenario: Missing or regressed preservation evidence
- **WHEN** indoor geometry/coverage regresses or required perturbation evidence is missing
- **THEN** the evaluation does not declare general acceptance from a better endpoint alone
