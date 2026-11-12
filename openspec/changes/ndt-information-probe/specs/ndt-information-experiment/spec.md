## Purpose

Enable an isolated diagnostic experiment that changes scan-matching information
without silently changing inertial priors or shipped application settings.

## ADDED Requirements

### Requirement: Controlled information scaling
The scratch experiment SHALL scale NDT cost, Hessian and RHS consistently for
both grids while leaving inertial priors and geometric support unchanged.
The control factor1 SHALL preserve baseline arithmetic.

#### Scenario: Reduced information
- **WHEN** an approved factor below1 is selected
- **THEN** NDT contributions change by that factor and priors do not

### Requirement: Reproducible evidence
Each run SHALL identify its multiplier, executable, input and effective settings,
preserve fresh odometry and final artifacts, and expose missing evidence.

#### Scenario: Cross-multiplier snapshot reuse
- **WHEN** a run attempts to restore a snapshot under an experimental multiplier
- **THEN** restoration is refused rather than mixing experimental conditions

### Requirement: Bounded diagnostic scope
The experiment SHALL collect the approved nine input/factor combinations and
report observations separately from candidate acceptance.

#### Scenario: A promising isolated run
- **WHEN** a single intervention improves a measured property
- **THEN** the report retains robustness/map-coverage limitations and does not claim shipping readiness


### Requirement: Behavior-preserving diagnostic replay
Opt-in instrumentation SHALL preserve solver arithmetic and retain focused
iteration evidence separately from acceptance of any correction.

#### Scenario: Instrumented Livox1 reproduction
- **WHEN** alpha1 and alpha0.1 are replayed with tracing
- **THEN** exact stage-specific trajectories and cloud identity SHALL be checked against the retained corresponding runs before trace interpretation
- **AND** any mismatch SHALL be reported rather than silently treated as the same failure

#### Scenario: Focused full Livox2 reproduction
- **WHEN** the approved full alpha0.1/beta0.1 Livox2 replay enables focused diagnostic tracing
- **THEN** original full input and settings SHALL remain unchanged and no additional mapper run SHALL start automatically
- **AND** close-scene support, match turnover, prediction/carry state, factor contributions and accepted updates SHALL be distinguished in the report
- **AND** exact pre-loop/final trajectories and cloud identity SHALL be checked against the retained full run before treating the trace as its explanation

### Requirement: Isolated translation continuity intervention
The approved follow-up SHALL scale XYZ motion observation information consistently in Hessian, RHS and cost only when the acceleration factor is active, preserving other factors and beta1 behavior.

#### Scenario: Finite beta diagnostic
- **WHEN** beta0.1 is selected with alpha0.1 after synthetic and review gates
- **THEN** only XYZ observation contributions including their attitude coupling SHALL be reduced
- **AND** both approved recordings SHALL be compared to retained controls with provenance and no acceptance claim from single runs
