# NDT guarded correspondence refresh

## Authorized scope

After exact W87 reproduction (`ndt-guard-w87-trace.md`), the user authorized
refreshing valid matches between optimizer iterations and retaining cell guards.
Work remains in `out/ahrs-gain-worktree`; the shipped/default algorithm is unchanged.
Charlie remains 3/19. No recording/captured-window replay may start without new
explicit approval. Current high effort is sufficient.

## Implementation contract

- Before every guarded `run_lm_iterations` solve, rebuild associations using the
  current poses and the existing fine/coarse maps. This includes a retry after
  a rejected step; unchanged poses deterministically rebuild the same targets.
- Keep the newly owned targets immutable throughout that optimizer call and
  its cost/trace evaluations. Keep the last iteration's set for window diagnostics.
- Retain occupancy, finite/SPD/inverse consistency, range, viewpoint, compatibility
  and minimum unique-source support checks unchanged. No planarity requirement.
- Build the replacement before publishing it. Unsupported refresh throws with
  window/iteration context; never silently fall back to stale targets or legacy.
- Preserve the existing solver acceptance, damping, priors and Huber settings.
  This does not add nonlinear objective acceptance or compare matched-only costs
  across association sets. The existing acceptance weakness remains separate.
- Report iteration identity, admitted/rejected support and gained/lost/reassigned/
  retained fine/coarse targets. Counts represent sources, not surveyed geometry.
- Guard remains default-off. Frozen-control executable and external evidence are
  preserved. An enabled-only snapshot hash suffix `ndt_match_guard_policy:refresh_each_iteration_v1`
  rejects historical frozen-guard snapshots; disabled legacy hashes are unchanged.

## Checks

- [x] Move admission from whole-window setup to each iteration boundary.
- [x] Keep owned associations immutable within each solve and preserve validity checks.
- [x] Add fixture covering voxel-crossing refresh, immutable prior targets,
  newly invalid cells and failed refresh publication.
- [x] Release build and focused guard/Huber/trace/snapshot tests: 18 cases, 2436 assertions passed.
- [x] Independent implementation review (trace_review, Codex).
- [x] Candidate committed as `71e2fa5` in `codex/livox4-ahrs-gain-only`.

## Replay gate

No candidate map or accuracy result exists yet. Proposed next scope is one
50-second prefix using the corrected input and unchanged alpha1/beta1/Huber3,
compared with the frozen guard/control evidence around W87. That would evaluate
early translation behavior only; it cannot establish preservation of the later
107-second rotation improvement. A later approved 125-second comparison is needed
for that gate. Neither run is started or authorized here.

## Review corrections

Independent review caught historical enabled snapshot identity reuse and an
incorrect rollback implication in the old failure message. Both are fixed.
A failed refresh does not publish new matches or attempt that iteration, but
previous iterations have modified in-window state; this is not a rollback.
The exception aborts before publication/map insertion of the failed window.
Enabled snapshot roundtrip is tested alongside existing settings-mismatch checks.

## Final validation

Release `dMapper` and `lidar_odometry_tests` build succeeded. Focused tests:
`NDT match guard*,diagnostic Huber*,Livox1 trace*,odometry snapshot:*`.
18 test cases / 2436 assertions passed; 123 unrelated cases skipped.
Logs: scratch `out/refresh-final-build.log` and `out/refresh-tests.log`.
No Debug-only behavior was introduced; no Debug build or recording replay run.
Independent review has no remaining blocker after the two corrections above.
Code remains experimental/default-off in the scratch worktree, not adopted into
root shipped code. Neither quality nor improved trajectory has been demonstrated.
