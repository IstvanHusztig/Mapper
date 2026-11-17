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

## Approved refreshed-match prefix replay

User approved one short Livox4 replay to compare with the frozen guard. Launched
source `71e2fa5` in external `livox4-iteration-refresh-50s`, reusing the verified
corrected 50-second input under exclusive cache ownership. Settings are byte-identical
to `livox4-guard-w87-trace-corrected`; alpha1/beta1/Huber3 and both cell thresholds
remain unchanged. Input/settings/executable/DLL/reference identities are frozen in
`manifest.json`. Early trace is enabled; no additional recording run is authorized.
Validation compares equal timestamps at odometry and final stages, W87 correction,
and velocity/bend diagnostics. A full route and the 107-second rotation gate remain
outside this run. Current high effort is sufficient. Results pending below.

## Completed short replay: 2026-11-14

Candidate source `71e2fa5`, native exit 0, 137/137 windows, wall time 577.356 s
versus frozen 211.590 s (2.73x). Inputs/settings were verified unchanged after
processing. Equal timestamps/counts (24660 poses) were checked separately for
odometry and final comparisons. Artifacts are in external
`livox4-iteration-refresh-50s`: `comparison.json`, `comparison.png`, `run-health.json`,
`collection-review.json`, and `result/scan_lio_merged.laz`.

| Reading | Frozen guard | Refreshed guard |
|---|---:|---:|
| W87 endpoint correction norm | 8.18 mm | 13.49 mm |
| W87 outgoing velocity XYZ (m/s) | 0.653,-0.521,0.038 | 0.638,-0.512,0.035 |
| Maximum end speed | 2.992 m/s | 1.100 m/s |
| Windows above 2.5 m/s | 8 | 0 |
| Maximum heading bend against gyro | 1.783 deg | 1.488 deg |
| Odometry path from all snapshot poses | 27.634 m | 17.626 m |
| Max odometry disagreement with same-time unguarded reference | 11.912 m | 1.583 m |

The unguarded reference is the exact timestamp prefix of `livox4-ahrs-gain-trace`
(17.298 m path), not surveyed truth. Candidate/frozen maximum disagreement is
11.054 m in odometry and 11.877 m after post-processing. Final stage includes a
candidate loop correction up to 1.37 m, so it is not substituted for odometry.
The speed episode is absent in this prefix; the first W87 correction remains
modest, and both candidates still depart from the reference near 31 s. This is
local evidence supporting refresh, not proof of map accuracy or a complete fix.

### Diagnostic rounding disposition

The runner initially marked `needs_review` despite native completion because
`run_health` conservatively flagged W97's printed information ratio 0.450 without
chi-square. The full trace gives 0.4503025131938016 at final iteration 99.
`scan_to_map.cpp` uses the same sum of normal information for that trace and the
returned window diagnostic; `run_odometry.cpp` suppresses conditional chi-square
for ratio >= 0.45. W97 was therefore not eligible. Independent review verified
this. Preserve raw `run.json`/`run-health.json`; attach `collection-review.json`
rather than rewriting the warning or silently passing incomplete evidence.
Bend/speed/bias coverage is 137/137; 18 eligible conditional chi-square rows exist.
Their median/p95 are 169.95/1232.02 and remain diagnostic triggers; the frozen
34-row population differs, so do not rank their distributions directly.

Independent review: trace_review (Codex) confirms the bounded diagnostic result
and the rounding explanation. No robustness/noise-floor, matched-wall precision,
revisit accuracy, surveyed geometry or physical closure verdict is established.
The new cloud is available for user review, not an accepted final map.

No further recording run was started. In particular the 107 s rotation gate and
full route remain untested for this candidate. Current high effort suffices.
Charlie remains 3/19; retain both candidates pending the user's decision.

## Approved 125-second validation

User reports the refreshed 50-second cloud looks good and authorizes one
125-second Livox4 replay to check the rotation event near107s. This is visual
feedback on the short result, not whole-route adoption.

Run folder: external `livox4-iteration-refresh-125s`. Same frozen `71e2fa5`
executable/DLLs and byte-identical settings as the short replay and frozen guard;
reuse unchanged125sinput from `livox4-ahrs-gain-prefix/input`. Trace switches from
early W84–102 to `DMAPPER_TRACE_LIVOX4=1` (W297–300), with Huber3 unchanged.
Sanitize inherited diagnostic environment. Manifest hashes input, binaries,
settings and references. Check first130 snapshot windows against refreshed50s
before interpreting the extension. Compare all346windows to frozen guard and
W297–300 iterations to the unguarded rotation-failure reference. No other run is
authorized. Expected runtime is roughly20–25minutes from measured refresh cost;
actual runtime and completeness will be recorded. Current high effort sufficient.

## Completed 125-second replay: rotation gate not passed

Native exit 0; all346windows completed; wall1698.819s (28.31min). First130
odometry window payloads exactly reproduce refreshed50s. Source/settings/input
identities passed their frozen hashes. W297–300 analysis contains400iterations,
zero trace/decomposition/assembly errors. Native `run.json` status `collected`.

| Diagnostic | Unguarded reference | Frozen guard | Refreshed guard |
|---|---:|---:|---:|
| W298 /106.92s heading correction | -16.959 deg | -0.071 deg | -2.247 deg |
| W301 /108.00s | -3.488 deg | -0.528 deg | -8.600 deg |
| W302 /108.36s | -2.306 deg | -0.780 deg | -9.095 deg |
| W312 /111.96s | +1.510 deg | -0.001 deg | +5.874 deg |
| Maximum absolute heading correction | 16.959 deg | 1.783 deg | 9.095 deg |
| Windows with correction above3deg | 4 | 0 | 3 |
| Maximum estimated end speed | 1.260 m/s | 5.402 m/s | 2.404 m/s |
| Windows above2.5m/s | 0 | 116 | 0 |

Refreshing retains the short-run reduction in excessive speed across this prefix,
but loses part of the frozen guard's angular suppression. It is not a complete
fix: the original W298 spike is smaller while large nearby corrections persist.
Do not infer that exactly the same cause shifted later. Detailed factor capture
ends atW300, so the initiating factors forW301–302 andW312 are NOT MEASURED.
AtW298 the traced node164 final correction is -2.374deg and peak incremental
step -0.497deg; these are different metrics from the endpoint window diagnostic.
The trace shows no >3deg cumulative node164 correction during W297–300.

The candidate's speed rises toward2.404m/s at the prefix end; falling below the
2.5m/s diagnostic trigger is not proof of stability beyond125s. Post-processing
accepts9loops with up to6.16m segment correction. Keep this separate from odometry;
a plausible final cloud cannot certify the intervening trajectory.

W97 conditional-uncertainty warning remains in raw run-health output. The exact
first130-window identity, matching diagnostic lines, and prior short-run trace
ratio0.4503025131938016 resolve the same rounding ambiguity: it was not eligible
for the <0.45 conditional diagnostic. No missing bend/speed/bias windows. No
surveyed map accuracy, wall/revisit/robustness or full-route acceptance verdict.

### Artifacts and disposition

External `livox4-iteration-refresh-125s`: manifest/run/run-health, factor-analysis,
comparison.json and comparison.png, result/odometry_snapshot.bin and
result/scan_lio_merged.laz. Comparison JSON hashes log/snapshot/cloud/analysis/chart.
Log SHA256 `0ed0322483b3cfe132b2d4d7f3eddf128dac0b9be3070912067d09437ac2e89e`.
Snapshot SHA256 `d2ca43bfd01da979e1c427193da541fa49788bc5a7320a53ea12358faa391cf2`.
Controls: `livox4-ahrs-gain-trace` and `livox4-ndt-match-guard`.

Independent review (trace_review, Codex) verified the events and bounded conclusion:
angular problem unresolved, no claim of identical initiating cause. Final native
completion, prefix identity and400-iteration trace checks subsequently passed.
No automatic adoption/rejection or further replay. Next investigation should
focus onW301–302 (108s), using existing logs/state first; tracing their factor
updates would require a separately authorized capture. High effort is sufficient.
Charlie remains3/19. Preserve this candidate and both controls for user decision.
