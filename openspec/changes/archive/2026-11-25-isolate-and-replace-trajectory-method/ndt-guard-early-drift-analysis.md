# Guard translation regression: existing-data diagnosis

User enabled high effort and requested existing-run investigation, OpenSpec
documentation and a stop before new runs. No algorithm changes, builds, optimizer
solves or recording replays were performed. Independent Codex review:
`trace_review` (not Sonnet).

## First material departure is around 31 seconds

The first 2.5 m/s crossing at W122 / 43.56 s is a late symptom. Comparing saved
odometry diagnostics (before loop closure), window-start position differences
are 3 mm at W86, 7 mm at W87, 99.85 mm at W88, 167 mm at W89 and 231 mm at W90.
They exceed 0.5 m at W102 / 36.36 s and 1 m at W107 / 38.16 s.
Rounded logs cannot establish bitwise equality of the earlier trajectories.

W87 starts at 30.96 s relative to the recording's first window:

| Logged quantity | Reference Huber3 | Guard + Huber3 |
|---|---|---|
| Predicted displacement, m | (0.269, -0.205, -0.007) | (0.271, -0.211, -0.004) |
| Result minus prediction, m | (-0.065, 0.066, -0.008) | (-0.004, 0.007, -0.002) |
| Outgoing velocity, m/s | (0.456, -0.353, 0.018) | (0.653, -0.521, 0.038) |
| Information ratio | 0.386 | 0.371 |

Thus nearly the same predicted displacement receives approximately 93 mm of
correction in the reference but only 8 mm in the guard. This is a correction
seen in the reference, not independently surveyed ground truth. The larger
guard velocity is carried into W88. Final snapshot endpoint separation at W87
is 99 mm; its outgoing velocity differs by (0.197, -0.168, 0.021) m/s.

There are **22,988 unique admitted sources** at W87 (17,640 fine / 19,335 coarse
terms). Invalid-cell rejections: 3,072 / 600; residual incompatibility: 173 / 1,061.
This is not a complete loss of correspondences. Fixed-set NDT stiffness along
the reported weak velocity direction is 2.57e6 versus velocity-prior 1.48e5 and
motion-model 1.45e5 cost/(m/s)^2. Those conditional cost curvatures are not
calibrated observability. The legacy rematched cost is nonsmooth and its
negative finite-difference stiffness must not be interpreted as negative
physical information or compared as the same objective.

## Code mechanism and what is actually demonstrated

Candidate commit 426b8ed builds correspondences in `run_lm_iterations` once,
from predicted poses. `ndt_match_guard.cpp` chooses each containing cell,
rejects initially incompatible sources and owns a copy of each admitted target.
`scan_to_map.cpp` then uses these same targets through all 100 iterations:
no reassignment and no reacquisition of initially excluded sources. The fixed
objective can therefore strongly constrain the result around targets selected
by an erroneous prediction. Valid individual cells do not ensure that this
fixed association set has sufficient capture range.

The observed W87 behavior is consistent with this design weakness: the guard
suppresses the correction seen in the reference and preserves more incoming
motion. This is the strongest supported explanation, **not an isolated causal
proof that freezing alone is responsible**. Cell occupancy filtering, initial
residual gating and freezing were enabled together. Their effects are confounded,
and both trajectories/maps already differ slightly before W87. No stale target,
wrong source index, sign error or shared-state lifetime bug was found in review.

Outgoing velocity is updated jointly in `scan_to_map.cpp`, copied to
`carry.velocity_mean` by `update_acc_term_after_window`, selected as the next
window's prior, and used by `integrate_window_translations`. The larger estimate
therefore feeds the next prediction and its next fixed association selection.
At W120 the guard predicts (-0.714, 0.120, -0.040) m and adjusts it by
(0.058, -0.008, -0.023) m: optimization opposes dominant negative-X motion but
does not undo it. It would be wrong to describe that window as the optimizer
simply adding the entire runaway displacement. W122 starts already 5.139 m
from the reference. The candidate's map rebuild is later (W333), not this trigger.

## Attitude/inertial coupling: measured contribution, unresolved net effect

Saved IMU samples and timestamps match exactly for the inspected windows.
Normalized final-pose rotations differ in tilt by 0.752 degrees at W87,
3.389 degrees at W120 and 3.342 degrees at W122. Mean
`9.80665 * (R_guard - R_reference) * a_body` is approximately
(-0.665, 0.135, -0.079) m/s² at W120 and (-0.593, 0.052, -0.004) at W122.
This computes the attitude contribution only, using saved final poses and a
nominal conversion. The integrator/factor subtract carried gravity, which can
compensate part of it. Pre-LM attitudes and exact carried gravity at W87 were
not captured. Therefore net gravity leakage or its amplification is **not
established**. Independent review specifically required this qualification.

## Evidence and next gate

External `livox4-ndt-match-guard/early-drift-existing-data.json` preserves W80–125
diagnostic excerpts and SHA256 hashes of both logs and odometry snapshots.
Reference: `livox4-ahrs-gain-trace`; candidate: `livox4-ndt-match-guard`, both under
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/`.
Detailed per-iteration factor/match traces cover W297–300 only. They cannot name
the first responsible W87 cell or recover its initial association set. Final
snapshot model poses are overwritten and are not an optimizer restart state.

Do not adopt this candidate or discard cell-validity checks on this evidence.
The next diagnostic target is W84–102, not W298. If further causal isolation is
requested, prepare bounded logging of incoming attitude/gravity/velocity and
initial-versus-reassociated matches/factor forces there, then request approval
for the smallest faithful prefix (about 40–50 s from the beginning). Separate
admission checks from correspondence refresh in the subsequent design; do not
silently launch an ablation or change the algorithm. A fixed set within one
linearization/trial is different from freezing it for the entire window.

Charlie remains 3/19. No additional run authorized or started. High remains
appropriate for the next numerical design/causal analysis.
