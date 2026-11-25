# Acceptance stop at W301: existing-data code audit

2026-11-14. User confirmed Astra high for analysis. Scope: inspect existing
corrected replay and code, recover a local visual comparison, assess a useful
code split. No new odometry replay or algorithm change.

Source: scratch `out/ahrs-gain-worktree` commit 16e4ae7. Experiment base:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/`.
Candidate: `livox4-step-acceptance-125s-v2`; control:
`livox4-iteration-refresh-125s`. See rotation-objective-acceptance-plan.md for
the completed test suite, approvals, run hashes and 300/346-window outcome.

## What enters the acceptance check

`run_odometry.cpp` prepares each window from the previous completed end pose,
IMU relative rotations, carried gyro bias, velocity and gravity/offset state.
It integrates translations when the always-on IMU term is active and the window
is moving. Thus `use_imu_translation_prediction=false` does not disable that
later always-on integration branch. The motion-model poses and the immutable
acceptance prediction are captured before the outer optimization loop.

W300 outgoing values, recorded immediately before W301:

- Velocity [-0.8022485971, -0.8275063000, 0.1134413135] m/s; norm 1.158118483
  m/s, below the configured 5 m/s carry guard.
- Velocity covariance diagonal [2.205348282e-5, 2.079310521e-5,
  1.558378503e-5] (m/s)^2, before the next gap-dependent growth.
- Gyro bias [-0.0852138214, -0.0732143765, 0.1002508137] deg/s.
- Carried gravity [0.0054622334, -0.0043673436, 1.0012050727] in accelerometer
  units; the next scale is 9.80665 divided by its norm.

These are real outgoing carry values, not a complete recorded W301 input state.
Detailed pose/sample traces cover W297-300 only. W301 has scalar acceptance,
factor-cost and match-count records, but no raw H/b, proposal vector, maximum-
angle node ID, complete input poses or individual target cells. Therefore the
exact node and correspondence that first pull the rotation remain unmeasured.

## Why it exceeds the limit

In `step_acceptance.cpp`, a proposal is computed from projected NDT normal
equations plus the other factors. Each trial scales the coupled update, builds
private poses/velocity/bias/offset, and checks every node's geodesic rotation
against both the current state and the immutable pre-optimization window prediction. Only an
admissible trial is scored using the unprojected fixed-support physical merit.
The caps are 1 degree per step and 3 degrees relative to that prediction.

The 3-degree cap is an acceptance veto; it is not a constraint in the linear
solve. Uniform backtracking and increased damping cannot guarantee a feasible
descent direction along its boundary. Failure here establishes exhaustion of
the attempted proposals, not absence of every feasible solution.

W301 accepted maximum correction progresses through 0.830208, 1.430011,
1.929388, 2.476036, 2.928153, 2.986063, 2.993877, 2.997768 and 2.999868 degrees.
Iteration 9's final/smallest logged trial would reach 3.000430104 degrees with
a 0.000645466-degree maximum step, so it is rejected before evaluating cost.
The maxima can refer to different nodes: they are not scalar additive yaw.
`after=inf` is the unevaluated-cost sentinel, not evidence of numeric overflow.

Iteration 0's merit improvement is mostly IMU cost reduction (151726.6), not
just NDT. During iterations 1-8, NDT is the only decreasing physical factor on
each accepted coupled update. At iteration 8 its 5.724754 reduction outweighs
motion +0.6159996, IMU +0.0271135, velocity +0.1034857 and smaller increases.
The total decrease is 4.977615. Therefore the objective still rewards the
outward updates; enforcing decrease alone does not establish physical alignment.
These are finite coupled cost changes, not isolated angular factor responses.

Unique admitted source points rise from 7890 at iteration 0 to 8699 at iteration
9; fine/coarse matches rise 4569/5653 to 4781/6464. Near the last boundary there
is little turnover (i9: fine gained1/reassigned1, coarse gained1). This is not
a wholesale association collapse at the final rejection. Valid cell statistics
and increasing support do not prove physical correctness. Costs from different
refreshed supports must not be compared as one fixed objective.

## Caller and initialization observations

1. The wrapper takes damping `lm` by value. W301 iteration 8 accepts attempt4
   (5 -> 50000); the caller retains 5, and `next_lm_factor(5,8,true)` stays 5.
   Iteration 9 starts again at 5. This loses successful retry damping between
   outer iterations. It is a continuity/efficiency weakness, not a demonstrated
   explanation for the outward direction: iteration 9 also tries 500000 and
   still exhausts its permitted scales. Any policy change needs its own test.
2. `prepare_velocity_state` seeds both velocities with the incoming mean.
   `integrate_window_translations` advances local velocity and positions but
   does not assign the integrated end velocity to `window.velocity.end`.
   Thus its initial end-velocity residual need not agree with its own integrated
   position seed. End velocity is a free optimized variable, not a forced
   physical prior. This can explain initial IMU cost cleanup, but cannot alone
   explain the later NDT-favored movement. Quantification must distinguish the
   fully traced neighboring windows from the missing W301 state.

The bounded residual reconstruction now quantifies observation 2. Using the
saved pre-integration poses, node samples, gravity and incoming velocity, the
Python calculation reproduces the initial IMU cost with normalized discrepancy
at most 7.7e-13. It follows Eigen's quaternion conversion and normalization,
the same integration, anchor intervals, residual signs and weights.

| Window | Logged initial IMU cost | Share from end-velocity term | Cost with only integrated end-velocity seed |
|---|---:|---:|---:|
| 297 | 42770.11946 | 99.9944% | 2.47350 |
| 298 | 10335.76827 | 99.9876% | 1.38978 |
| 299 | 39853.47972 | 99.9940% | 2.42775 |
| 300 | 81610.12989 | 99.9980% | 1.68876 |

This is initial-state residual evaluation, not another optimizer run. The last
column changes only the free end-velocity seed; poses, start velocity, attitude,
gravity and offset stay fixed. It establishes an avoidable initialization
inconsistency, not a flyaway cure. The later coupled solution may be unchanged;
the full effect needs a separately approved test. W301 attribution remains
unmeasured. Evidence: candidate `analysis/initial-end-velocity-analysis.json`
and `analysis/analyse_initial_end_velocity.py`.

## Useful split, proposed only

Separate the acceptance boundary into three small responsibilities:

1. `ProposalResult`: raw coupled step, solve status, effective damping and
   base-linearization covariance/information. Return before legacy mutation.
2. An immutable context plus a pure trial evaluator: frozen targets, base
   state, prediction and priors; return trial state, factor costs, rejection
   reason and maximum-angle node/timestamp.
3. One caller-owned commit of the exact accepted physical state and covariance.

This would make input/output capture and local unit tests easier without
splitting every residual into new files. The split alone is not a numerical
fix. Preserve the disabled path and checked arithmetic; changing damping carry,
end-velocity initialization or boundary handling must be separate, explicit
changes. Exact correspondence attribution additionally needs factor H/b and
target identities; an acceptance refactor cannot reconstruct missing evidence.

## Visual evidence without another replay

`local-comparison/` in the candidate bundle contains `control_W297-300.laz`,
`acceptance_v2_W297-300.laz`, `comparison.png`, the export Python helper and
`provenance.json`. Both clouds use identical original-cache points and the same
0.4 m output range filter. Candidate placement uses logged post-handoff poses;
control placement uses the odometry snapshot. No ICP, recentering, loop closure,
consistency pass or new optimization was performed.

Coverage: 157139 points over log-relative 106.560238-107.998988 s, approximately
1.44 seconds before W301. Exact node times, cache-source hashes, subset identity,
pose-index validity and LAZ readback are checked. Common-point disagreement is
median 1.49 mm, p95 17.49 mm, maximum 0.5203 m; this is not an accuracy metric.
The local plots look very similar. Wall sharpness, revisit agreement, global
geometry and robustness improvement remain NOT MEASURED. There is no finished
candidate map and no saved W301 cloud; this export cannot prove a better map.

The safety stop works, but recovery remains unresolved. Do not widen the cap or
declare the local candidate better based on this result. Charlie remains 3/19.

Independent Codex reviewer `trace_review` confirmed the boundary/caller findings
and performed the bounded residual reconstruction and visual export. The primary
agent inspected the scripts against the C++ formulas, snapshot layout and point
placement, verified output hashes/source gates, and viewed the plot. No Sonnet
review, new native run, refactor, default adoption or push occurred.
