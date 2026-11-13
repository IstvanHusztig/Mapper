# Optimizer code audit and comparison with other methods

2026-11-13. Requested before proceeding with an optimizer bypass. Inspected the
executed gain-only source at fc73404, its caller and generated observation
equations, existing tests, prior findings and saved Livox4 states. A requested
Codex research agent independently reviewed gyro algebra and primary external
implementations. No production code/settings changed; no build, solver,
synthetic trajectory or recording replay was executed. Offline algebra checks
used the existing snapshots only.

## Result

No new sign/indexing error explaining the W298 rotation was established.
There is a definite angular-units defect, already recorded as C37, and an
additional acceleration-integration approximation. Neither currently explains
the large candidate/control difference. The most relevant unresolved weaknesses
are acceptance without an objective test, changing correspondence support, and
translation-only degeneracy handling. These are known findings, now connected
to the observed gyro-inconsistent rotation; they are not newly proven triggers.

## How this optimizer calculates an update

`src/domain/scan_to_map.cpp` in the executed worktree assembles:

1. Fine/coarse NDT point terms with Huber residual weights, per pose node.
2. Relative-pose motion terms, including gyro-bias cross terms.
3. Accelerometer terms coupling rotations, translations, start/end velocities
   and a world-frame acceleration offset, plus carried priors.
4. First-node increment regularization.

It forms normal equations `H dx = -J^T W r`, adds scheduled pose-only damping,
solves them, and adds pose/velocity/bias/offset increments. A successful solve
whose pose translations remain within 1 m of the prediction is accepted. Bias
and velocity covariance blocks are estimated from the undamped normal matrix.
The caller carries state, replaces the tail and inserts points into the map.
The function name/comment says LM, but no actual/predicted reduction ratio is
used to accept a step or adapt damping.

## Local findings and checks

| Area | Finding | Relevance to W298 |
|---|---|---|
| Gyro RHS/Jacobian signs | Generated residual is `delta=m-f`, Jacobian is `d(delta)/d(pose)`, and caller subtracts `J^T W delta`. Bias Jacobian `-dt I`, cross blocks and prior RHS have consistent signs. | No sign reversal found; independently checked. |
| Bias carry | Prediction removes carried bias; optimizer subtracts only `b-b_prior`. | Old C6 double subtraction is not present in executed source. |
| Acceleration angular Jacobian | For `r=-sign*tau*R*f+...`, source uses `+sign*tau*wk*s*[R*f]x*E`, where `E` maps Euler increments to world rotation. Independent finite differences at 128 saved attitudes agree to 3.04e-10 g/rad maximum entry error. | Formula's sign/convention checked. This is not a compiled full-assembly Jacobian test. |
| Matrix storage | Pose band covers adjacent edges, anchor triples and the extended end-velocity term. Extra-state blocks include both directions of coupling. | No out-of-band/indexing mechanism found by inspection; no new runtime assertion coverage claimed. |
| Origin angular units, C37 | `optimizer_options.cpp:35–37` preserves degrees; `scan_to_map.cpp:1209–1214` uses their inverse square directly on radian unknowns. Weight is 3282.806 times weaker than the documented degree sigma requires. | Definite existing defect. Current weight is still 1e12; first-node rotation is approximately unchanged at the event, so this does not explain the downstream 18-degree bend. |
| Acceptance/damping, C13 | `scan_to_map.cpp:214–248,1290–1328`: translation guard and solve status, no full-objective decrease or angular plausibility test. | Directly explains why the observed bend can be accepted, not which factor starts it. |
| Degeneracy, C31 | `scan_to_map.cpp:731–775`: normal-based 3x3 translation information produces `M=diag(P,I)`; rotation remains unprotected. | Structural limitation. Logged W298 final information ratio 0.272 exceeds the full-keep threshold 0.2, so projection is not established as the trigger; intermediate iterations were not logged. |
| Covariance/state commit | Covariance can change before a step is rejected. Tail poses and carried joint velocity are also distinct representations. | Rejection branch cannot explain this pair: all 34,600 steps were accepted. Existing tail discrepancy is documented separately. |
| NDT statistics/support, C20/C30 | Containing-voxel lookup, legacy first-point mean/fixed initial normal, and two grids; missing/invalid/back-facing matches disappear. Huber limits individual residual influence, not correspondence switching. | Plausible source of competing angular forces. Exact-statistics experiments previously failed validation; do not silently reactivate them. |
| Duplicated motion information, C34 | The tightly weighted translation motion model follows a prediction already derived from velocity/acceleration, while related inertial priors enter separately. Beta 0.1 reduces but does not remove that coupling. | A model-design concern, not a newly proven arithmetic bug. |

The small-angle Euler bias correction differs from exact SO(3) composition;
this approximation is not evidence of an 18-degree error at these sample rates.
Likewise, the end-velocity fallback treats start velocity as a frozen reference,
but its missing-end-term branch is not implicated by the supported local samples.

## Acceleration quadrature discrepancy

For piecewise-constant world acceleration, the exact acceleration contribution
to adjacent secant-velocity difference uses a triangular kernel:

`k(t)=(t-A)/(B-A)` on `[A,B]`, and `(C-t)/(C-B)` on `[B,C]`.

The implemented factor (`scan_to_map.cpp:1044–1200`) instead uses a uniform
average over the midpoint interval `[(A+B)/2,(B+C)/2]`, multiplied by its duration.
The start factor similarly substitutes a first-half box for the full-interval
linear kernel `(B-t)/(B-A)`; the end factor has the corresponding approximation.
Constant acceleration agrees. Changing acceleration generally does not. This
differs from the predictor's piecewise-constant position/velocity integration.

Offline kernel comparison used saved rotations/accelerations, without executing
an optimizer or generating a trajectory. Kernel areas agree within 1e-10 s,
so constant gravity cancels. Scale was 9.80665 m/s² per g; the exact carried-gravity
normalization was not recovered. Around W298, triple discrepancies are at most
0.00418 m/s; end discrepancies are 0.00355 control and 0.00357 m/s candidate
(about 1.37/1.38 nominal residual sigmas). Similar readings in both runs give
weak evidence for this approximation as the initiating jump cause. Any correction
must update temporal weights, Jacobians and noise treatment together.

## Primary-source comparison

| Implementation | Relevant behavior | What can transfer conceptually |
|---|---|---|
| [Ceres trust-region methods](https://ceres-solver.readthedocs.io/latest/nnls_solving.html#trust-region-methods) | Compares actual with predicted objective reduction and controls the trust region. | A real acceptance contract, rather than scheduled damping plus translation distance. This does not ensure the mapping objective itself is physically correct. |
| [FAST-LIO filter](https://raw.githubusercontent.com/hku-mars/FAST_LIO/main/include/IKFoM_toolkit/esekfom/esekfom.hpp) and [measurement model](https://raw.githubusercontent.com/hku-mars/FAST_LIO/main/src/laserMapping.cpp) | Iterated manifold state update with propagated covariance; nearest-neighbour plane validation and residual screening. | Consistent state/uncertainty treatment and explicit support validation. It is not a Ceres-style cost-rejection guarantee or proof it would solve this recording. |
| [LIO-SAM scan matching](https://raw.githubusercontent.com/TixiaoShan/LIO-SAM/master/src/mapOptmization.cpp) | Weak-direction projection uses the full 6D scan-matching Hessian. Its `LMOptimization` function solves normal equations by QR and applies additive updates, without a trust-region acceptance ratio. | Study rotational observability too; do not transplant its numerical eigenvalue thresholds or infer safety from the function name. |
| [LIO-SAM IMU factors](https://raw.githubusercontent.com/TixiaoShan/LIO-SAM/master/src/imuPreintegration.cpp), [GTSAM preintegration](https://raw.githubusercontent.com/borglab/gtsam/develop/gtsam/navigation/ManifoldPreintegration.cpp), [navigation-state integration](https://raw.githubusercontent.com/borglab/gtsam/develop/gtsam/navigation/NavState.cpp) | Pose/velocity/bias coupling uses per-sample inertial integration and associated Jacobians; position and velocity receive distinct time weights. | A reference for auditing dMapper's quadrature and world-frame offset approximation, not justification for replacing the whole pipeline now. |

Sources inspected 2026-11-13. These references are research comparisons, not
tested dMapper dependencies or a selected replacement algorithm.

## Decision and next step

Keep the proposed optimizer bypass parked while the equation/support audit is
resolved. Do not bundle the minor unit correction, integration changes, different
weights and an acceptance redesign into one experiment.

The next discriminating evidence remains the first accepted rotation-growing
iteration at W297–300: factor rotational RHS/step contributions, predicted versus
actual reduction on frozen associations, and separate support gained/lost after
rematching. Comparing only the current matched-point NDT cost is insufficient:
a lower sum may simply mean difficult points disappeared. A defensible acceptance
change needs a complete objective and explicit treatment of lost support, plus
consistent commit/rollback of poses and all carried state/covariance.

This audit does not authorize the previously proposed diagnostic replay or an
algorithm change. The research agent independently checked the gyro equations,
confirmed known-versus-new findings, and cautioned against treating other methods
as guarantees. No new arithmetic cause of W298 is claimed. Charlie remains 2/19.

Evidence: external `livox4-ahrs-gain-prefix/optimizer-audit/optimizer-equation-checks.json`,
SHA256 `14136859a216d5fae8ac565f8755b068535314d8f5af5cadb9c38d10085bd8a8`.
It records source/input hashes, quadrature definitions/results and the scope of
the derivative check. Existing tests were inspected, not executed; their mostly
room/corridor and motion examples do not constitute full joint-gradient coverage.
