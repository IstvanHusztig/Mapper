# Acceptance split, end-velocity seed and damping carry

2026-11-14: user authorized implementation and tests, keeping guards, independent
review and OpenSpec documentation. No new recording or captured-window replay.
Base scratch commit 16e4ae7; evidence in acceptance-108s-code-audit.md.

## Contract

- Separate a nonpublishing proposal result, immutable iteration context, private
  trial evaluation and one accepted-state commit. Covariance is staged before
  legacy mutation, and retains its base-linearization interpretation.
- Keep cell guards, between-iteration match refresh, fixed-support merit,
  1-degree step and 3-degree prediction bounds, and atomic rejection unchanged.
- Seed the free end velocity with the same integration that seeds positions.
  Only successful moving-window integration in the enabled acceptance path
  requests the output. Start velocity, physical priors and covariance are not
  replaced. Disabled processing retains its historical operation order.
- Return the damping used by the accepted proposal to the outer loop, then apply
  its existing scheduled multiplier once. Rejected/stationary updates publish
  no accepted damping; each new window still starts from configured initial LM.
- Revise the enabled snapshot policy identity to v3; inactive historical identity
  and shipped settings remain unchanged. Record effective damping and the node
  producing each maximum trial/prediction angle for future approved diagnostics.

This repairs two demonstrated local inconsistencies. It does not establish that
the physically wrong scan-matching preference or the W301 failure is solved.
It is not the broader replaceable-method interface; Charlie remains 3/19.

## Checks

- [x] Split proposal, trial and commit; preserve disabled path.
- [x] Correct enabled end seed and accepted damping carry; revise policy identity.
- [x] Test nonpublishing proposal/covariance, atomic rejection/stationarity,
  actual retry damping and its next-iteration handoff, irregular-time velocity
  integration and unchanged legacy seed, and settings/snapshot compatibility.
- [x] Build Release; run focused tests, full CTest and acceptance-log Python tests.
- [x] Independent code/evidence review; address findings and commit scoped work.

Recording equivalence, map quality and perturbation checks remain NOT MEASURED
until separately approved runs. No new cloud is promised by unit testing.

## Implementation and review in progress

`ScanToMapProposal` now carries the raw solve, staged covariance/information and
effective damping. Capture returns before the legacy update path and does not
write even the private physical state. `FrozenAcceptanceContext`,
`evaluate_trial` and `commit_trial` separate immutable inputs, trial construction
and publication. Accepted damping is an optional status value cleared on every
checked call, and the outer loop passes it to the existing scheduled LM update.

Independent Codex reviewer trace_review found no source blocker. Its requested
additional tests for reused failed-covariance status and initial IMU residual
consistency are added. Release build/testing is in progress; 9 Python acceptance
checker tests pass. No recording processing was started.

An accidentally overlapping incremental test build hit a locked object while
the first build linked. The first build completed successfully; the failed
incremental build is not validation evidence. A serialized fresh incremental
build now covers the final added tests. No recording executable was in use.

Release build3 completed. Focused tests: 14 cases / 997 assertions pass. Full
CTest: 151 cases pass, 3 skipped; the 3 pre-existing expected consistency
assertion failures remain expected (79725 assertions total). Full run 19.07 s.
These results precede the review's final logging-only correction: terminal
summary/stationary rows no longer label an untried LM value as effective damping.
The final Release rebuild/check will cover that correction too.

Debug is also checked because this split introduces an owned-sample context
with a borrowed pointer inside its immutable IMU options. The sandboxed Debug
build failed in MSVC's PDB manager (C1902), not a source test. A serialized
outside-sandbox build with two compiler workers is in progress. No preset,
compiler flags, installed tools or processing settings were changed.

Both build caches use alpha=1 and beta=1, consistent with the frozen refresh
control's recorded experiment_weights. Huber3 remains an explicitly selected
runtime experiment; no shipped-default or numerical weight change is included.

## Completed implementation and validation

Scratch commit `ba0268c315669dd748f7ba6b1c9116bbc94728ee`, branch
`codex/livox4-ahrs-gain-only`, clean worktree. Final Release executable SHA256:
`0bb0df9539cf92613bc89c9571d1e1fc324b51e134ae157a4c3d0b25757c547a`.

- Final Release build includes the diagnostic correction. Full CTest passes:
  151 cases, 3 skipped, 14.89 s. The three pre-existing expected consistency
  assertion failures remain expected; no failed test cases.
- Focused Release and Debug each pass 14 cases / 997 assertions. Debug's
  successful outside-sandbox two-worker build includes the final source.
- Acceptance-log Python checks: 9 passed. OpenSpec strict validation passes.
- Independent Codex review (trace_review, not Sonnet) found no remaining source
  blocker; both requested extra tests and the logging correction are included.

Evidence stays in scratch `out/seed-damping-*` logs, including
`seed-damping-final-build.log`, `seed-damping-final-ctest.log`,
`seed-damping-focused.log`, `seed-damping-debug-build2.log`,
`seed-damping-debug-focused.log` and `seed-damping-python.log`.

`effective_lm` is emitted only for actual proposal/trial/accepted rows.
`step_node` and `prediction_node` are zero-based window indices;
`prediction_time_s` is the node's recording timestamp, not elapsed log time.

No recording run, new map, default adoption or push. Map quality and resolution
of the 108-second failure remain NOT MEASURED. Ask before any replay. Medium
is sufficient for mechanical replay preparation/monitoring; use high to assess
new numerical behavior. Charlie remains 3/19, with this local checklist complete.
