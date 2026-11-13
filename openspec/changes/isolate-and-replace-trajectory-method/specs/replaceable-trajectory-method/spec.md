# Spec Delta

## Purpose

Allow trajectory-estimation methods to be evaluated, selected and removed independently while preserving reproducible legacy processing and explicit state compatibility.

## ADDED Requirements

### Requirement: Stable per-run method selection
The system SHALL select one trajectory method through persisted settings shared by CLI and GUI processing. An absent selector SHALL select `legacy`. The method SHALL remain fixed throughout a run. Unknown or unavailable identifiers SHALL fail before processing with an actionable error.

#### Scenario: Existing settings remain usable
- **WHEN** existing settings omit the method selector
- **THEN** CLI and GUI select legacy and preserve its numerical behavior

#### Scenario: Explicit experimental selection
- **WHEN** either front end loads settings naming an available experimental method
- **THEN** the shared pipeline uses that method and records its identifier in the result

#### Scenario: Removed method requested
- **WHEN** settings name an implementation that is no longer available
- **THEN** processing does not start and the error identifies the unavailable method without silently substituting another

### Requirement: Explicit state and failure contract
Every method SHALL define its input/output units, time and coordinate conventions, state ownership, initialization/reset behavior and failure semantics. State needed across updates SHALL belong to that run and method. Failed updates SHALL identify whether any output or carried state was committed. The system SHALL NOT mix state from distinct methods or silently switch methods after failure.

#### Scenario: Fresh run after an earlier run
- **WHEN** a new run begins after a previous run completes or fails
- **THEN** its estimator state is initialized according to the selected method without inheriting the previous run's mutable state

#### Scenario: Update failure
- **WHEN** the selected method cannot produce a valid update
- **THEN** the result reports the failure and committed-state status according to its documented contract without an undisclosed fallback

### Requirement: Reproducible method identity and restart compatibility
Results and restart artifacts SHALL identify the method, implementation/configuration identity and state format needed to interpret or resume them. Restart validation SHALL reject incompatible method, settings, input or state versions before processing. Existing legacy snapshots SHALL have an explicitly tested compatibility path or a clear incompatibility error; their state SHALL NOT be reinterpreted as another method's state.

#### Scenario: Cross-method restart
- **WHEN** a restart artifact from one method is supplied for another method
- **THEN** the system refuses the restart and explains the method mismatch

#### Scenario: Old legacy artifact
- **WHEN** a historical legacy snapshot is loaded
- **THEN** the documented compatibility path validates it as legacy or rejects it explicitly before processing

### Requirement: Legacy extraction preserves numerical behavior
Introducing the interface SHALL preserve the legacy method's numerical results for identical inputs, numerical settings, build conditions and execution environment. Experimental controls and shipped-default configurations SHALL be identified and checked separately; introducing method selection SHALL NOT promote an experimental numerical setting to a default.

#### Scenario: Frozen control comparison
- **WHEN** the extracted legacy method processes a frozen control input under the same numerical configuration and environment
- **THEN** its numerical outputs match the pre-extraction control exactly, apart from explicitly added provenance metadata

### Requirement: Transparent evaluation evidence
An experimental method SHALL remain explicitly identified until accepted. Evaluation reports SHALL separately expose geometry, revisit consistency, sharpness, coverage, perturbation spread and odometry versus final-stage results under the agreed evaluation contract. Missing evidence SHALL be labeled NOT MEASURED. A closed endpoint or successful solver exit alone SHALL NOT be reported as proof of robust mapping.

#### Scenario: Local improvement with incomplete route evidence
- **WHEN** a candidate improves a local diagnostic but full-route or perturbation evidence is missing
- **THEN** the report identifies the measured improvement and missing evidence without declaring overall robustness or readiness to ship

### Requirement: Independent removal and explicit adoption
An experimental method SHALL be removable without changing the retained methods' numerical behavior or deleting historical evidence. Default changes and removal of legacy SHALL require an explicit adoption decision supported by the project's evaluation and visual-review gates. Accepted behavior and method-selection guidance SHALL be documented in the existing user documentation.

#### Scenario: Experimental method rejected
- **WHEN** the user decides to remove a rejected method
- **THEN** its implementation and exclusive dependencies can be removed while retained methods and historical evidence remain available

#### Scenario: Candidate accepted for general use
- **WHEN** the user accepts a validated candidate and explicitly approves its default status
- **THEN** the default and existing documentation are updated with method selection, behavior and validation limits
