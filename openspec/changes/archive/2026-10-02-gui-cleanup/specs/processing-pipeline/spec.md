# Spec Delta

## MODIFIED Requirements

### Requirement: GUI and CLI results are identical
For the same input folder and the same settings, a result saved by the GUI's "Load data and process" SHALL be identical to the CLI's output. The comparison ignores only the run-dependent fields listed in `regression-baseline`.

#### Scenario: GUI run of the short reference dataset
- **WHEN** the GUI processes `44_short` with `settings/lidar-vertical.toml` loaded and saves the result
- **THEN** every output file compares identical with `44_short_baseline`

## REMOVED Requirements

### Requirement: No separate consistency action in the GUI
**Reason**: Its scenario is tied to the "Compute all" button, which the GUI cleanup removes together with the whole step-by-step flow.
**Migration**: Replaced by "Consistency only inside the processing sequence" below, which states the same rule for the Start menu workflow.

## ADDED Requirements

### Requirement: Consistency only inside the processing sequence
The GUI SHALL NOT offer an action that runs the consistency pass on its own. Consistency runs only as part of the processing sequence, controlled by the settings (`use_consistency`).

#### Scenario: After a run
- **WHEN** a run started with "Load data and process" has finished in the GUI
- **THEN** the available actions are the Start menu and the view options, with no consistency button
