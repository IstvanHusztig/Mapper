# Translation recovery after bounded-step exhaustion

User authorized implementation on 2026-11-14. Existing captured solver cases and
unit tests are the local validation; no new recording replay is authorized.
Checkpoint Charlie remains 3/19. This is a local acceptance detour.

The saved W301 iteration 51 still has feasible translation descent, despite
scalar LM damping freezing the joint direction. See solver-case-analysis.md.
Implement a conservative fallback after ordinary checked proposals exhaust:

- Solve the translation principal block of the complete frozen normal equations,
  holding rotations, bias, offset, velocities and their covariances exactly fixed.
- Preserve the ordinary successful path. Require a valid ordinary solve and
  covariance and identity weak-axis projection; do not bypass projected directions.
- Use diagonal equilibration and a separate bounded normalized damping sequence;
  never interpret it as the ordinary scalar LM parameter.
- Evaluate through the same full frozen merit, origin-increment penalty,
  translation/rotation limits and atomic publication. No forced success and no
  constrained-stationarity claim when recovery fails.
- Preserve incoming scalar LM carry on recovery (no accepted joint damping),
  including the existing periodic schedule. Recovery can recur; optimizing the
  global damping policy is outside this narrow correction.
- Record recovery method and preserved-covariance provenance, and change the
  enabled snapshot policy identity. Keep the feature opt-in.

Independent design review recommended exact matrix preservation instead of Euler
reconstruction and restricting recovery to identity projection. Both are included.

## Checks

- [x] Implement guarded translation recovery and explicit telemetry.
- [x] Test successful recovery, exact frozen state, rejection atomicity,
  nonidentity projection, damping carry and ordinary-path compatibility.
- [x] Replay existing captured i0/i5/i10/i51 calls; require native accepted merit
  decrease at i51 and unchanged ordinary outputs on the earlier calls.
- [x] Complete independent implementation review and relevant test suites.
- [x] Record artifacts, limitations and commits before requesting any new run.

This does not establish map improvement or explain the physical NDT mismatch.
Recovery fixes the demonstrated lost feasible translation, not all constrained
optimization cases. Full-route and perturbation validation remain unmeasured.

Implementation review found an incoming over-limit bias edge case: preserving
that bias would bypass the normal proposal clamp. Recovery now declines it.
Invalid reduced solves have their own rejection reason. Recovery-attempt failure
and exact preservation are covered by targeted fixtures (execution pending).

The reduced system is D H_tt D + lambda I, RHS D b_t, with
D_jj = 1/sqrt(H_jj). H includes the origin increment regularizer. The sequence
starts at lambda=0, then 1e-6, 1e-5, ... within the configured retry count;
each uses the existing backtracking budget. Positive finite diagonals, successful
LLT, finite output and a relative residual check are required. Frozen covariance
is labeled preserved_incoming, not recomputed or calibrated uncertainty.

## Existing-case result

Final Release binaries were frozen under
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-translation-recovery-cases/`.
Its manifest records input case hashes, binaries, native exit codes and log
hashes; `check_translation_recovery.py` records the validation procedure.

| Saved call | Result |
|---|---|
| W301 i0 | Exact reference decision, cost and output state |
| W301 i5 | Exact reference decision, cost and output state |
| W301 i10 | Exact reference decision, cost and output state |
| W301 i51 | Native exit0; accepted first translation recovery after 48 unchanged ordinary trials |

i51 merit falls from 67256.450383513758 to 67230.458720647759, including
0.0057465625997862634 origin-increment penalty. Rotation step is zero;
prediction-relative rotation remains 2.9999999999999916 degrees. Maximum actual
translation update is 0.0019097410968977157 m. Maximum displacement from motion
prediction is 0.0338418779 m, against the unchanged 1 m limit.

Independent Codex reviewer `/root/seed_damping_crosscheck` (not Sonnet) checked
all 180 rotation matrices and all 39 logged nuisance/covariance values remain
bit-for-bit identical, and recomputed the origin penalty. The final i51 log is
byte-for-byte identical to the independently audited preliminary result:
SHA256 `90913b0375010a23266c337af7614c1fc58af64cddde4f3554fce7669fc421c7`.

Frozen binaries:
- dMapper: `614765cc2b309db8498ec1f97a79bd3316c8ad138ba36aac988c934a2e542c0b`
- replay_solver_case: `6099472b7f8ed1462b121cc7d1faf69014fed1cc387910b9c20fdd7b78e28daf`

This proves recovery of feasible descent in the saved call. It does not prove
completion of W301 or a route: translation can later become stationary while
rotation remains constrained, and this deliberately conservative fallback then
still rejects. No convergence certificate or physical-map accuracy is claimed.

## Regression-test correction

The first full Release suite had one unexpected test failure: an older
cumulative-bound fixture required rejection within 80 calls. The candidate
instead continued with feasible translation while respecting the rotation cap.
Replace that behavioral assumption with per-call cost decrease and exact held
rotation on recovery, keeping the every-call cumulative-cap assertion and the
separate unresolved-decrease rejection test. Independent review agreed. This
changes the fixture's obsolete stall expectation, not the numerical algorithm.

## Next recording gate (not started)

A concrete candidate bundle is prepared in sibling directory
`livox4-translation-recovery-125s`, using these frozen binaries and the unchanged
125-second Livox4 prefix/settings. Manifest approval remains false. Huber3 is
enabled with `DMAPPER_DIAGNOSTIC_NDT_HUBER3=1`; no other diagnostic override.
Estimated 45-120 minutes, with recovery overhead not yet measured. Check the
31-second drift region, W301/108-second rotation region, completion and then
run/map checks. Ask the user before launch. No cloud exists for this candidate.
Map quality, full-route behavior, perturbations and historical golden recording
comparisons are NOT MEASURED; they require separately authorized recording work.

## Completed validation / checkpoint

Source commit: `d213b754f05892f0a7703b4d6c9afc81e67c93af` on scratch branch
`codex/livox4-ahrs-gain-only` in `out/ahrs-gain-worktree`; worktree clean.
No push or default adoption.

- Final full Release suite: 161 cases passed, 3 skipped, exit0. The three
  pre-existing expected-failure consistency assertions remain visible.
- Focused Debug: solver cases 9/216 assertions; acceptance 8/835 assertions;
  revised cumulative-bound fixture 1/242 assertions; all passed.
- Python: 10 acceptance-health and 2 solver-extraction tests passed.
- Four native saved-call checks passed; no new input processing/cache writes.
- Independent implementation and revised-test review completed; findings fixed.
- OpenSpec strict validation passed; diff whitespace checks passed.

Final logs are preserved with the external solver-case artifacts. Broader
Charlie remains 3/19. Medium effort is sufficient for preparation/monitoring;
request high before diagnosis of an unexpected new replay failure. A recording
replay remains approval-pending under the user's standing instruction.

User approved the 125-second replay on 2026-11-14. The first launch failed
before loading input with Windows exit 0xC0000135 because the prepared bundle
omitted freeglut.dll. Added the DLL from the same Release build and preserved
the loader-failure logs. PID 10864 started loading inputs at 15:50:26 CET.

The approved replay completed at 16:53 CET with exit 0 after about 63 minutes.
It passed W301 and completed all 346 windows. The result is
livox4-translation-recovery-125s/result/scan_lio_merged.laz (111480651 bytes).
Run-health recorded no bends above 3 degrees and a maximum speed of 1.30 m/s.
Velocity-jump chi-square triggered (median 153, p95 2374, 142 windows);
conditional uncertainty coverage remains incomplete. Map accuracy and visual
quality remain unassessed.
