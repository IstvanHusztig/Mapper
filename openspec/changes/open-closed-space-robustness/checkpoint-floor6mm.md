# W415 residual-floor checkpoint

2026-11-13. User authorized the prepared control/candidate experiment with
"Astra high on, go". Approval is recorded in the frozen bundle manifest;
execution began at approximately 14:39 Europe/Berlin. No full-route replay is
authorized by this checkpoint approval.

Bundle: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/open-closed-floor6mm-w415/`.
Source: scratch commit `5be333f316c3a6b189603a51a9e6250fb57bd9e5`.
Approved manifest SHA256: `a09350385f9b52616fcdb11f8703e147eff228717f42e4d9c533af1046f56d77`.

Runner verified 426 pinned inputs before launching. It reserves run.json
exclusively, refuses existing result paths, and performs no automatic retry.
The native tool must reproduce all 100 Huber control iterations exactly before
running the single 100-iteration 0.006 m candidate. Alpha/beta remain 0.1,
Huber cutoff 3; original recordings, map statistics and configured priors stay
unchanged. Planning estimate was 5–15 minutes. Free C: space at launch: 93 GB.

Status: completed, native exit 0 after 124.49 seconds. All 426 pinned files
remained unchanged. Every emitted control record matched the frozen control
for all 100 iterations; the candidate also completed 100 iterations.

## Readings

| Measure | Huber control | 6 mm floor |
|---|---:|---:|
| Last-node rotation correction from identical seed | 9.142869 deg | 8.727052 deg |
| End-velocity magnitude | 0.502139 m/s | 0.501993 m/s |
| Residual gyro bias Z | 0.249715 deg/s | 0.244126 deg/s |
| Final indoor matched contributions | 8,891 | 8,850 |
| Final outdoor matched contributions | 7,085 | 7,053 |
| Start-velocity prior quadratic, model-relative | 886.357 | 823.010 |

The angular change is 0.416 deg (4.55%). Maximum same-node position difference
is 2.98 mm. These are local intervention readings, not surveyed errors or a
perturbation-resolved improvement. Both trials applied all 100 steps. All
recorded poses/states are finite; all 100 outgoing covariance blocks for each
of bias, velocity and acceleration offset are finite and positive definite.
Inverse-Hessian covariance is not calibrated uncertainty. No invalid-covariance
skips occurred. Legacy skips remain 77–79; the enabled counter's accepted
16,535–17,729 is information eligibility before viewpoint filtering, not final
matches. Initial poses, priors, source/association/target-geometry hashes agree.

## Fixed-state influence

At iteration 0 the floor affects 1,090 indoor contributions in 71 cells and
754 outdoor contributions in 12 cells. The same return can appear in both grids.
Only one of the three previously implicated outdoor cells is affected:
85899345919 (90 contributions). Its Huber sum weight increases 8.99236 ->
9.98506, while its signed world-Z rotation RHS changes -8540.59 -> -7469.55
rad^-1. The other two cells' influence is unchanged within numerical precision.
Whole outdoor world-Z RHS changes -36297.36 -> -34756.55 rad^-1. Thus the floor
activates as intended but makes only a limited change to this known mechanism.

Analysis uses alpha 0.1 and signed world translation/infinitesimal rotation
contributions before degeneracy projection and priors. These are not Euler
parameter torques or the joint solve. Native initial projected RHS is also
retained. Recomputed control weights agree with CSV exports and candidate
initial weight sums agree with native traces. Cross-sigma costs were not used
as geometry scores.

## Independent review and correction

Independent Codex reviewer checked all 100 control records against the original
evidence and confirmed the signs, units, support and narrow local interpretation.
Its separate startup-corrected raw-gyro calculation gives disagreement
9.188469 -> 8.773251 deg, consistent with the bend remaining; the primary
reproducible analysis reports seed-relative correction above.

An initial analysis compared end velocity against the carried *start*-velocity
prior. Source inspection and independent review identified this mismatch; the
final analysis uses velocity_start_after. The old 4949.73/4731.08 numbers are
not prior innovations. The first report is retained as superseded evidence;
v2 and final reports contain the corrected 886.357/823.010 values. No calibrated
chi-square interpretation is claimed. This correction does not change the
rotation or end-speed readings.

## Decision boundary and missing evidence

Recommendation: do not promote this candidate to a full-route replay or increase
sigma to chase the bend. It produced a small local response, not a flyaway fix.
This does not refute all residual-confidence mechanisms; only one implicated
cell was altered. Earlier Cauchy improved this checkpoint much more but worsened
the full route. The later W904–916 initiator remains unresolved.

Task 4.2 remains partially complete: this single-window checkpoint measures
outgoing covariance but not next-window incoming carry or subsequent motion.
Synthetic carry tests cannot substitute for that missing recording evidence.
No new cloud was generated by this local optimizer check.

Task 4.3 uses its explicit missing-evidence-report branch. Existing later logs
show prediction/carry growth, but snapshot v1 lacks pre-LM model poses,
gravity/conversion, offset prior covariance/history and complete incoming carry.
An exact sequential restart needs initial poses/model, bias/velocity/offset
means and covariances, gravity/to_mps2, rest and previous-use decisions,
velocity validity/time, future IMU increments, and map history/distance through
the W914 rebuild. See existing-data-checks.md. A cold-start segment is not an
equivalent control and has not been substituted.

Proposed next decision: retain the 6 mm result as insufficient and prepare a
scoped baseline state capture around the later runaway, rather than another
candidate full replay. Its capture implementation, frozen manifest and runtime
estimate must be reviewable before requesting run approval. No capture or
alternate algorithm change is authorized by the completed W415 experiment.
Tasks 4.4 and 5 remain blocked by missing longer-segment evidence and adoption
gates; this is not a completed OpenSpec change.

Reproducible analysis: tools/analyze_residual_floor_checkpoint.py; archived
with analysis-final.json in the experiment bundle. The file refuses existing
outputs and requires a completed exact-control gate. Analysis assertions verify
same initial state/priors/association, 100 iterations and independent weights.
No full-route or robustness replay was started. Default sigma stays zero.
