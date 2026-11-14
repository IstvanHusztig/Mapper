# 108-second boundary stall: source and existing-data analysis

2026-11-14. User enabled high effort and authorized analysis, not another run.
Implementation examined: scratch ba0268c. Evidence: external
`livox4-seed-damping-125s/run.log`, settings and the preceding independent review.
Sol 6.1 agent `seed_damping_crosscheck` independently checked the claims below.
No algorithm edits, builds or recording replays were performed.

## What was overlooked

The acceptance wrapper is a veto around an unconstrained solve, not a solver
for the guarded problem. `step_acceptance.cpp:90-119` scales the entire proposed
update (and clamps bias), constructs poses, then rejects rotations outside the
limits. `scan_to_map.cpp:1410-1437` solves damped normal equations without those
rotation constraints. There is no constraint Jacobian, active-set multiplier or
tangential search. A feasible descent direction may exist without being on any
of the scaled proposed rays. Conversely, the current evidence cannot prove that
one exists at W301. The full-gradient stationarity check at
`step_acceptance.cpp:259-271` is unconstrained: a nonzero outward gradient at a
constrained optimum would not pass it.

This explains a missing recovery/convergence capability, not why the physical
NDT evidence first favors the outward rotation. Increasing the cap or treating
exhaustion as success would not establish a correct map.

### Damping carry retained a second, incompatible-looking policy

`scan_to_map.cpp:287-295` retains the legacy tenfold periodic increase after
accepting the retry damping. No branch decreases damping based on successful
physical objective reduction. W301 iteration 50 accepts at 5e18; the periodic
increase makes iteration 51 start at 5e19 and retries reach 5e24. The legacy
comment at line 1412 assumes roughly 1e10. Only pose parameters are damped;
bias, offset and endpoint velocities are not directly damped. Hence tiny logged
rotation updates do not establish whole-state stationarity.

Rotation-bound rejections start at iteration 5, before extreme damping. The
ratcheting policy is therefore not a sufficient explanation for the original
outward demand. It is part of the eventual stagnation chronology. Carrying the
successful value fixed a real state-transfer problem but did not make the
inherited damping schedule an adaptive trust-region method.

### Detailed tracing does not reach the checked solver

There are two independent blind spots:

1. `run_odometry.cpp:54` restricts the Livox4 detailed trace to W297-300.
2. More importantly, `step_acceptance.cpp:156` passes a null trace to the
   proposal assembly; line 227 also passes null for merit assembly. The outer
   trace receives only accepted updates and costs, not proposal H/RHS,
   factor gradients, actual projection or rejected raw steps.

Existing W300 log inventory confirms 100 accepted-update records and 900
acceptance-cost records, plus input/output states, but no inner factor matrices.
Merely extending the window filter to W301 would still miss the needed evidence.
The full unprojected RHS is already computed for the stationarity check but not
emitted. This is a concrete diagnostic omission, not missing sensor data.

## What the existing run now establishes

| W301 iteration | Accepted LM | Maximum correction, degrees | Fixed-support merit decrease |
|---|---:|---:|---:|
| 0 | 5 | 0.761765 | 2285.274 |
| 5 | 5 | 2.842104 | 433.628 |
| 7 | 5 | 2.993101 | 104.482 |
| 10 | 5e6 | 2.999994 | 0.439288 |
| 11 | 5e7 | 2.9999991 | 0.081474 |
| 20 | 5e14 | approximately 3 | 1.77e-6 |
| 50 | 5e18 | approximately 3 | 2.92e-9 |
| 51 | no accepted step | within cap in final trials | best 2.76e-10; threshold 9.56e-10 |

The limiting node is 152 from iteration 7. Target hash is unchanged from
iteration 11 through 51. The last membership change is at iteration 11;
iterations 12-51 report zero gained/lost/reassigned targets in both grids,
retaining 4915 fine and 6702 coarse matches. Late association turnover cannot
explain this tail of the stall. Residuals, robust weights and projection can
still vary with state even when membership does not.

The corrected seed changes the first-step explanation materially. W301 starts
with IMU cost 1.87418, not v2's 151737.009. Its first accepted step changes:

| Factor | Cost change, after minus before |
|---|---:|
| NDT | -2892.542677 |
| Motion | +534.810005 |
| IMU | +18.514091 |
| Carried velocity | +53.881191 |
| Bias | +0.018593 |
| Offset | +0.042354 |

The first coupled update is now paid for by NDT improvement, not repair of the
stale end-velocity seed. Scalar cost changes do not identify the angular gradient
or specific harmful matches. No new sign error was established in this review.

## Remaining proposal/objective question

Settings enable weak-axis projection (ratios 0.1/0.2). The proposal inherits it;
the merit explicitly disables it. `scan_to_map.cpp:814-855` transforms each
NDT block with M=diag(P,I): H becomes M H M and RHS becomes M b, while priors
remain unprojected. `step_acceptance.cpp:62` judges the full unprojected merit.
This is not automatically its Gauss-Newton model. Pose cross-coupling can change
the proposed angular direction even though only translation is projected.

Actual W301 projection scales are not recorded. It may be identity there;
the earlier control's projection values cannot be transferred to this state.
Thus this is a specific untested consistency question, not a proven cause.

## Recommended next step; not implemented or run

Correct the diagnostic path before another expensive replay: capture the actual
checked proposal and frozen inputs at the first boundary encounter (W301,
iterations 0-10), including factor RHS, projection, step blocks, full-merit
directional derivative and rotation-constraint slack/directional change.
Preserve a reusable frozen case so numerical investigations need not replay
125 seconds each time. Keep capture bounded and observation-only.

Then distinguish: (a) mismatched proposal/merit descent, (b) genuine constrained
stationarity, (c) an available feasible direction missed by scaling, and
(d) incorrect matching/weighting driving the objective. Any eventual damping
or constrained-step change needs a coherent convergence contract and tests,
not a relaxed guard or a smaller tolerance chosen to make this replay finish.

Root cause of the original physical flyaway remains unresolved. Cause of this
run's termination is established: boundary saturation followed by insufficient
merit decrease under escalating damping. No completed cloud, map-quality claim,
default adoption or push. New replay still requires approval; high remains useful
for the numerical work. Charlie remains 3/19.
