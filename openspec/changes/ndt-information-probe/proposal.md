## Why

Phase0a established a current-build18.67m pre-loop displacement from a1mm
decimation change. We need a controlled intervention to distinguish how scan
matching weight affects actual velocity jumps versus claimed uncertainty.

## What Changes

- Scratch-only compile-time NDT information multiplier, recorded in logs/build identity.
- Consistent indoor/outdoor Hessian, RHS and diagnostic cost scaling; unchanged priors.
- Nine approved diagnostic runs with stage-specific measurements and preliminary views.

## Capabilities

### New Capabilities
- `ndt-information-experiment`: isolated, reproducible controlled weighting experiment.

### Modified Capabilities
None. No shipped setting or accepted algorithm change.

## Impact

Scratch checkout C:/Repo/dMapper/out/phase0b-worktree on
codex/phase0b-ndt-information, based on8d230a1. Root OpenSpec is the planning
source of truth. New dependencies none. Application changes stay in scratch.
User explicitly approved the nine-run plan2026-11-12; no additional confirmation
needed for its implementation/tests/runs. Candidate acceptance/push remain separate.

Follow-up explicitly authorized: diagnostic logging plus two full Livox1 replays
(alpha1/.1) to inspect the onset/runaway without changing settings or algorithm.
