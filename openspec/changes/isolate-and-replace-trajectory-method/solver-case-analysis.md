# W301 captured-call analysis

2026-11-14. User enabled high and authorized analysis following capture.
Evidence is in the external `livox4-solver-capture-125s-retry/reproduction`
directory. No new recording processing or numerical implementation change.

## Reproduction

All four captured calls were run with the frozen `replay_solver_case.exe`.
Iterations 0, 5 and 10 returned native exit 0. Iteration 51 returned native
exit 1. Each passed exact reference comparison of decision/cost rows and
output poses, velocity, bias, offset and covariances. Four comparison files
retain the results. This validates reuse of these individual frozen calls.

## Findings

Independent reviewer `seed_damping_crosscheck` checked the native trace:
all 36 projection-axis rows across 12 proposal attempts have scale 1.
The smallest geometry ratio is 0.271531, above the full-retention threshold
0.2. Every proposal H/RHS payload equals the corresponding full-merit base
H/RHS payload exactly. Weak-axis projection and proposal-versus-merit
projection mismatch are inactive in these captured calls.

The offline tool `tools/analyse_solver_case_trace.py` solves the recorded
damped system with each factor RHS while keeping the same total Hessian.
Diagonal equilibration is used for the diagnostic solve. The summed solve
reproduces native proposals with relative error at most 9.7e-12 for iterations
0/5/10 and 2.6e-16 for iteration 51. Results and trace hashes are retained in
`reproduction/factor-analysis.json`.

At node 152 in iteration 5, the additive Euler angular component along the
native proposed angular direction decomposes as follows (degree-scaled
coordinates, not a geodesic angle or a factor-removal experiment):

| Contribution | Component |
|---|---:|
| All NDT | +0.668613 |
| Gyro motion rows, including bias coupling | -0.187131 |
| Motion translation rows | +0.002628 |
| Accelerometer residuals | -0.001814 |
| Carried velocity prior | -0.017258 |
| NDT range 0-2 m | +0.076606 |
| NDT range 2-5 m | +0.322930 |
| NDT range 5-15 m | +0.269073 |
| NDT range 15 m onward | +0.000003 |

Range rows partition NDT, and the two motion rows partition the motion
contribution. They must not be added a second time. This shows a coupled
NDT-driven proposal opposed by the gyro at the first captured boundary
encounter. At iteration 0, the NDT component is +0.729579 and the IMU component
-0.000013. The old stale end-velocity penalty is no longer the initiating
large residual. Actual wrong-cell geometry remains to be established.

At iteration 51, the full assembled gradient infinity norm is 148018.901,
versus the stationary threshold 1e-6, while LM starts at 5e19. The largest
gradient entry is node 0 world-y. It mixes coordinate units and is not an
angular attribution. At node 152 the Euler gradient components are roughly
[-967.388, -403.012, +444.780]. The native maximum pose-parameter increment is
1.48019e-15, and the maximum extra-state increment is 1.00597e-13 at attempt 0.
Thus this particular final call also has tiny nuisance-state steps, resolving
the earlier uncertainty left by angular-only logging.

The recorded terminal rejection is explained by insufficient objective decrease
after pose damping has grown enormously at the active rotation boundary.
Nonzero unconstrained gradient alone cannot distinguish constrained stationarity
from a missed feasible direction. The proposed-step direction at extreme damping
also differs from early iterations and must not be used to attribute the original
rotation demand.

## Next focused decision

### Existing-case feasible-descent check completed

The independent reviewer evaluated a pure translation at iteration 51 using
the saved target cells and Huber3 cost. Node 58 world-z increases by
3.802176549557161e-6 m, all rotations and other variables remain fixed.
The non-NDT residuals are affine in this coordinate, allowing their exact
quadratic cost change to be computed from full H/RHS minus NDT H/RHS.

- Exact saved-target Huber change is -0.05704982834965952.
- Other-factor change is +0.05411909114236948.
- Total frozen merit change is -0.002930737207290042, versus tolerance about
  9.5577e-10. Merit falls from 67256.45038351376 to 67256.44745277656.
- Maximum translation offset stays 0.0338259453 m against the 1 m guard.
  Rotations remain exactly fixed, preserving their limits.
- Reconstructed baseline NDT cost agrees with the native value within 2.26e-10.
  Reconstructed local NDT RHS/H agree within 4.0e-11/4.7e-9. Zero-step change
  is exactly zero, quarter-step change is -0.0012821726341, and reversed-step
  change is +0.00879194607184. No Huber regime boundary is crossed.

This resolves the earlier constrained-stationarity uncertainty for the frozen
objective. A feasible descent direction remains at the stalled state. The
current direction-scaling and pose-wide damping fail to find it. This check is
an offline evaluation, with native acceptance of the candidate still pending.
It does not determine why NDT favors the physically questionable rotation.

The next correction should allow a feasible coupled update at an active
rotation bound and adapt damping without freezing all pose coordinates.
Validate it first on these reproduced cases with actual nonlinear merit and
all guards, then request any necessary recording validation.

Inspect influential NDT targets and weighting, including both the
2-5 m and 5-15 m groups, before calling an acceptance change a physical flyaway
fix. No parameter change or completed-map improvement is established here.

The capture/reproduction gate is complete. Charlie remains 3/19. High remains
appropriate for the constrained numerical analysis.

Subsequent authorized fix: `translation-recovery-fix.md` records source d213b75
and native acceptance of feasible translation at i51, with rotations/nuisance
state held exactly fixed. Earlier i0/i5/i10 calls remain exact. This resolves
the saved-call stall only; route completion and physical-map improvement still
require the separately approved recording-validation gate.
