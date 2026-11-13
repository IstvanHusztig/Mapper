## Why

Livox2's full route still flies away while the new indoor multi-floor Livox3 map looks crisp. Existing evidence establishes excessive influence from thin NDT cells at W415, but not the complete later runaway mechanism; a local Cauchy improvement failed on the full route. We need a bounded confidence-model correction that preserves the indoor result, plus honest house-plan registration evidence.

## What Changes

- Reuse established failure evidence and bound the missing later-onset analysis; justify an empirical regularization range from existing data before selecting experiment values, without claiming stationary observations identify universal sensor noise.
- Add an experimental, disabled-by-default NDT residual covariance floor for odometry only, consistently used in both grids, cost and robust weights, without changing stored map statistics or other priors.
- Validate progressively against exact checkpoints and the outdoor/indoor references, including mixed-axis Huber reweighting and carried-state uncertainty, with per-run authorization, complete evidence and perturbation gates before adoption.
- Preserve the house-plan registration audit, fit/held-out regions, unresolved physical correspondences and fixed-scale overlays; never warp the map to the plan.
- Update existing HTML documentation when a general change is accepted; detailed recording evidence stays in OpenSpec rather than becoming sensor-specific product documentation.

## Capabilities

### New Capabilities
- `confidence-evaluation`: reproducible confidence-model experiments, failure/indoor-preservation gates and evidence-based promotion.
- `map-plan-registration`: fixed-scale registration evidence with held-out surfaces and explicit ambiguity.

### Modified Capabilities
- `ndt-reference-map`: introduce optional residual-information regularization without changing map cell statistics or the disabled path.
- `sensor-settings`: load/save a sensor-dependent residual-noise floor, defaulting to disabled, with rejected invalid settings.

## Impact

Likely code: domain optimizer options/scan-to-map cost and assembly; application option propagation; infrastructure TOML load/save; focused numerical/settings tests. Registration and measurement tools remain analysis-only. No new runtime dependency or sensor-axis/layout change. Work continues in the isolated experimental checkout, preserving frozen Huber references and failed Cauchy results. Production defaults, loop closure, consistency, gyro/velocity priors and map statistics are outside the first intervention. Approval of this proposal is not approval of any new processing run.

Evidence and research: ../ndt-information-probe/open-closed-improvement-plan.md, open-closed-terra-research.md, early-heading-cause-audit.md, cauchy-correction.md, livox3-indoor-plan.md, livox3-quality.md and houseplan-registration-audit.md. The active historical change is retained, not archived by this proposal.
