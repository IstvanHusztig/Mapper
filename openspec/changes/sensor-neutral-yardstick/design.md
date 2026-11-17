# Design

## Authorized tool implementation, November 12

User approved rejecting incomplete measurements, exact timestamp correspondence,
fingerprinting runs, preserving clouds and validating odometry/final stages.
Use explicit coverage statuses and nonzero exits for invalid/incomplete evidence;
diagnostic triggers remain distinct from map acceptance. Never resume legacy
outputs without recorded identity, overwrite unknown results, or delete clouds.
Use structured seam results, not stdout scraping. Preserve source timestamps
without float rounding. Read versioned snapshots with bounded lengths and reject
malformed/truncated data. Validate coordinate/time conventions against the C++
writer and real snapshot/log evidence. Application algorithms remain unchanged.
The earlier statements requiring a separate scope decision are satisfied by this
explicit approval; new scope outside these verification changes remains separate.

## Context

See proposal.md. The removal prerequisite is committed (e196692, e0b29ab,
8d230a1). Turn collection completed, but independent calibration is unresolved.
The user chose revised limits before comparisons. Older OpenSpec task counts
and the draft retune plan are not sufficient evidence of current completion.

## Goals / Non-Goals

Goals: reproducible inputs, auditable reference provenance, explicit coverage
and an agreed evaluation contract before comparisons. Non-goals are listed in
the proposal; specifically, an aggregate diagnostic PASS cannot establish cloud
accuracy or completed calibration.

## Decisions

- Use Livox/2 0-60 s, whole 95 s Livox/1 and 44_short, starting at rest. Reject
  the older proposed 295-335 s mid-walk cut because startup assumes rest.
- Preserve and inventory existing shipped/k=1/own-noise perturbation families
  before scheduling replacements. Historical k=1 also changed the rotation
  floor; own-noise also changed ARW. Record actual settings, not shorthand names.
- Treat output artifacts as evidence, with paths and source/build provenance
  limits. Do not infer today's executable identity from old result directories.
- Keep revised criteria as an explicit prerequisite task; no old threshold is
  silently accepted. Separate recording validity, local shape/revisits, drift
  before loop closure, final closure, building/rock sharpness and robustness.
- Exercise tools on known logs, report missing diagnostics as NOT MEASURED and
  log tool defects. Generic sweep reuse currently lacks fingerprints; seam
  alignment and failed-check exit status require resolution before trusting it.
  A separate implementation decision is needed for behavior changes.
- Independent research agents inspect measurement assumptions and evidence;
  the primary agent checks their results. No recursive review delegation.

## Risks / Trade-offs

- Stale caches/configuration names -> fresh output directories and explicit
  configuration inventory; no inferred equivalence or silent resumed experiments.
- Calibration/gyro circularity -> independent LiDAR evidence must be qualified;
  final optimized poses are behavior measurements, not calibration truth.
- Missing/approximate statistics -> record coverage, actual stop reasons and
  uncertainty-model limitations rather than treating omissions as passes.
- Plan accidentally authorizes numerical work -> phase 0b remains a separate
  scope; no candidates compared before the revised-limits decision.

## Current evidence

- Removal: docs/agents/removal-validation.md.
- Turn run/calibration limits: docs/agents/turn-calibration-measurements.md.
- Phase0a preparation outputs: out/phase0a-2026-11-12/ (local, not committed).
- Existing inventory and tool-review findings: evidence.md in this change.
