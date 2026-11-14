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

## Authorized 125-second validation replay

The user approved the replay on 2026-11-14. Started at 11:18 CET, PID 35044.
Bundle: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-seed-damping-125s.
Verified the tested ba0268c executable, both input file hashes and identical v2
settings before launch. Huber3, cell guards, refreshed matches and acceptance
limits are active. Check the 31 s translation and 108 s rotation sections
against the refresh control and the v2 acceptance run.

Replay finished after 3111.23 s (51.85 min), native exit 1. Completed 300/346
windows. No merged LAZ or snapshot. Log SHA256:
64a5280564de54c209af18fb7311965e7821dbb66f2592e95287709c9eee2d5e.

W87 (30.96 s) matches v2 and the refresh control at logged precision:
result move [0.262,-0.199,-0.005] m, result-minus-guess [-0.008,0.011,0.001] m,
last-node yaw correction +0.353 deg. W300 (107.64 s) ends with a -2.505 deg
yaw correction versus -2.775 in v2 and -2.064 in the refresh control. These
are differences from each run's prediction; map accuracy is unmeasured.

W301 (108.00 s) accepted 51 local updates versus nine in v2. No W301 output
was committed to the map. Last accepted update (iteration 50) reached
2.9999999999999916 deg at node 152 (recording time 108.65889624 s),
with effective LM 5e18 and a merit decrease of about 2.9e-9. Iteration 51
exhausted retries with merit_not_decreased. At the last trial, effective LM
was about 5e24 and before/after merits were identical at logged precision.
The successful damping carry plus the retained multiplier merits analysis.
This run does not isolate the contribution of each fix.

Verify-run reports INCOMPLETE: no completed-window yaw bends over 3 deg, max
end speed 1.25 m/s, and velocity-jump chi-square median/p95 95/1172 across
104 reported windows. Wall, revisit and map geometry checks NOT MEASURED.
High effort recommended for analysis before proposing any further change.
