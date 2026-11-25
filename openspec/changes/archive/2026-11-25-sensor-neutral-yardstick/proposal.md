# Proposal

## Why

Phase 0a of the sensor-neutral retune needs a reproducible yardstick before any
candidate is judged. Prior experiments and newer user decisions are split across
Claude session logs, local outputs and a stale draft roadmap. The user explicitly
requires revised limits to be agreed before comparisons (2026-11-12).

## What Changes

- Track rest-start inputs, existing reference configurations and their provenance.
- Verify diagnostic coverage and tool behavior on known evidence without ranking
  candidates before the user agrees revised limits.
- Record completed prerequisites and calibration limits, with concrete evidence.
- Agree metric definitions and revised limits, then consolidate existing neutral
  perturbation families before deciding whether additional runs are necessary.
- Produce a phase-0a readiness record for a separately scoped phase-0b experiment.

## Capabilities

### New Capabilities

- `verification-evidence`: fail-closed diagnostic coverage, exact trajectory
  matching, identified resumable runs and separate odometry/final measurements.

### Modified Capabilities

None. User explicitly approved verification-tool implementation on 2026-11-12.
The original documentation-only restriction is superseded for these tools only.

## Impact

Planning, evidence, Python verification tools and their tests; output-local inputs/logs.
No new application dependencies, default settings, shipped calibration or numeric
odometry changes. `map-quality-harness` remains the existing recording-44 harness
change; this change links its evidence without marking its unfinished tasks done.
`synchronized-retune`, `imu-front-end` and `settings-audit` remain separate.

## Non-goals

Phase 0b, accepting physical weights, solving full calibration, new point filters,
committing/pushing unapproved work, or declaring a better final cloud. Phase 0a
prepares comparison; possible visible improvements are experiments in phase 0b
and corrective work in phase 3, subject to measurement and the user's visual check.
