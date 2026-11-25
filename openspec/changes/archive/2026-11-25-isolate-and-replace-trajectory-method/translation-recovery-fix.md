# Translation recovery after bounded-step exhaustion

## Proposed W214 extension (2026-11-14)

The user requested design on High with weak-direction and rotation guards
preserved, then authorized implementation, tests and independent review.
Implementation is complete in `out/ahrs-gain-worktree`. W214 diagnostics
are in `retained-tail-comparison-plan.md`. New recording and captured-case
replays require separate approval.

### Restricted fallback

After ordinary checked trials exhaust, an actively projected proposal may try
a translation fallback confined to directions the existing LiDAR information
test fully retains. Export the actual eigenbasis and scales from the proposal's
assembly. Let Q contain only columns with computed scale exactly 1. Directions
with scales below 1, including partially attenuated directions, receive zero
additional displacement in this fallback. This stricter fallback restriction
preserves the ordinary proposal's existing soft attenuation policy.

Use the already computed scales, including its configured threshold behavior.
Recomputing a cutoff from rounded eigenvalues could select a different subspace.
Freeze Q for the solver call and validate finite eigenvalues, eigensolver success
and orthonormal columns. Repeated eigenvectors may change sign or rotate within
an equal-eigenvalue group, so compare subspaces through Q Q^T.

For N pose translations and rank r, form B = I_N tensor Q. Extract the full
frozen-merit translation curvature H_tt and negative half-gradient b_t.
Build H_r = B^T H_tt B and b_r = B^T b_t. With diagonal scaling
D_jj = 1/sqrt(H_r[j,j]), solve

    (D H_r D + lambda I) z = D b_r
    delta_t = B D z

Use the existing normalized damping sequence and backtracking budget. Require
positive finite diagonals, successful LLT, finite steps and the existing relative
linear residual check. With accurate b_t and an SPD reduced system, its nonzero
solution is a descent direction for the frozen objective to first order.
Actual objective decrease remains the acceptance requirement.

Rank zero or invalid metadata produces an explicit recovery-unavailable reason.
Rank one/two uses this branch. Identity projection uses the existing recovery
path to preserve its validated outputs. This avoids extending the full-space
fallback into suppressed directions. Scaling a full-rank basis by attenuation
alone would allow the inverse solve to cancel that scaling.

### Match the derivative to the evaluated objective

Independent review found a concrete compatibility issue to address in the new
branch. `scan_to_map.cpp` forms NDT RHS with Euler-reconstructed poses, while
its checked NDT merit uses raw stored rotations. Even approximate orthogonal
rotations can make these residuals differ. For fixed raw rotation A, the NDT
residual is A*p+t-mu and its translation Jacobian is exactly I.

During lazy recovery-model assembly, supply a dedicated translation RHS built
from that raw residual, the same frozen target information and Huber weight
used by the merit. Retain the other objective terms' translation contributions
and the origin increment regularizer. Do not accidentally add projected NDT
RHS or count NDT twice. The existing translation curvature is an IRLS model,
not the exact robust Hessian. Expose this derivative only to the new branch,
leaving the ordinary and identity-recovery numerical paths intact.

Verify -2*b_t.dot(delta_t) against central finite differences of the native
frozen objective on synthetic fixtures, including approximate AHRS rotations
and active Huber weights. A derivative match does not certify stationarity.

### Publication and failure

Keep raw rotation matrices, velocity states, bias, offset and all covariance
blocks fixed during the restricted trial. Apply translations directly and use
the existing atomic trial/commit path. Require the existing full fixed-support
merit decrease, per-node translation bound and both rotation bounds. Check the
suppressed component (I-Q Q^T)*delta_t for the actual published displacement,
with a scale-aware floating-point tolerance. Do not relax a guard to make the
case complete. Preserve ordinary damping carry as in identity recovery.

If every restricted trial fails, report exhaustion. A zero restricted gradient
does not prove full constrained stationarity and must not silently complete the
window. W214 may still have no usable descent in the allowed subspace.

### Implementation and evidence checklist

- [x] Export validated projection metadata in the internal proposal and log
  fallback eligibility/rank/scales, including explicit skip reasons.
- [x] Add the lazy raw-residual translation derivative and restricted solve.
- [x] Test rotated rank-one/two spaces, excluded partial scales, rank zero,
  cutoff/repeated-eigenvalue behavior, invalid information, approximate rotations,
  Huber derivatives and an uphill ordinary proposal with feasible restricted
  descent. Verify published suppressed components and unchanged coupled state.
- [x] Test failure without descent and atomic rollback, existing angular and
  translation bounds, damping carry and identity-path regressions. Run relevant
  Release tests and independent implementation review before any replay.
- [ ] Prepare bounded W214 capture with actual projection metadata and frozen
  solver state so native directional checks can distinguish causes. The user
  must separately approve the concrete recording/captured-case replay scope.
  Retained-tail W214 was never captured, so older W301 cases cannot substitute.
- [ ] Compare the frozen failing call against the restricted fallback, including
  derivative agreement, actual decrease and weak-direction preservation. Any
  successful solver result still needs the earlier W94-100 trajectory divergence
  investigated before a retained-tail map-quality recommendation.

Reviewer `seed_damping_crosscheck` independently reviewed the subspace design
and identified the raw/Euler derivative issue incorporated here. It supports
testing the design. Whether this branch resolves the real W214 state remains
unmeasured. Charlie remains 3/19. High is appropriate for implementation and
numerical validation; medium suffices for later monitoring.

### Implemented and tested

Experimental commit `a27fa3344436af02bb7b318053a5af186bfb1ae1`, clean worktree.
The new branch uses symmetric cell information in its raw-residual translation
derivative and curvature. Independent review identified this requirement for
approximately symmetric admitted matrices, along with finite-value checks and
correct normal-based synthetic observability fixtures. Those findings were fixed.
The independent reviewer reported no remaining source blocker.

Full Release suite passed 164 cases, with 3 skipped cases and the 3 pre-existing
expected-failure assertions. Focused Debug passed 12 cases / 333 assertions.
The corrected uphill-proposal fixture exercises rejected ordinary steps followed
by restricted descent. The full Release suite includes the existing identity
recovery and acceptance guard tests. Saved real W301/W214 call replay remains
unmeasured in this change.

The existing active-projection rejection test uses zero normals and covers rank
zero without publication or recovery attempts. Existing cell-validity tests
cover indefinite/asymmetric covariance, stale inverse and invalid thresholds.
The exact new diagnostic wording is inspected in source rather than asserted.

Logs are under the experimental worktree's `out/` directory:
`w214-release-suite.log`, `w214-debug-focused.log`,
`w214-release-verified-build.log`, `w214-debug-build-retry.log`.
Earlier build overlap and sandbox PDB access failures were resolved through
serial builds and elevated compiler access. Their failed logs are preserved.

The enabled snapshot policy is now `fixed_merit_atomic_v5_restricted_recovery`.
Release executable SHA256 is
`0191C873A32629E474AC7DCCC98B8F1F8BAD1CD63E914482B21402277605B1CF`.
Actual W214 completion and map quality await separately authorized processing.
Medium is sufficient for preparing/monitoring a replay. High is appropriate for
interpreting new numerical failures. No recording or saved-case replay started.

## Existing implemented identity-projection recovery

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
