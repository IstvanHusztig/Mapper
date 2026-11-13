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

User subsequently approved next-step-design.md: finite motion XYZ information beta0.1 with NDT alpha0.1 fixed, gated tests/review and two full-start Livox1/Livox2 first60s runs. No shipped defaults or acceptance changes.

On November13 the user approved the two isolated W754 checks described in
full-livox2-cell-attribution.md: exact control, then a three-outdoor-cell ablation
only if control reproduction succeeds. The latest instruction is to finish these
checks, save the state and stop for later continuation. No full replay or fix is
authorized by this step.
