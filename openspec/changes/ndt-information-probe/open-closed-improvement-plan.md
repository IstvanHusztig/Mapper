# Proposed plan: reliable motion across confined and open geometry

2026-11-13. User explicitly enabled High and requested Terra research plus
house-plan alignment investigation, with a plan before algorithm edits/new runs.
This document proposes future work; it does not authorize implementation or runs.

## Established evidence, not to rediscover
- Livox3 indoor Huber3 is the frozen preservation reference: crisp per user;
5.56cm observed pre-loop endpoint separation,11.01cm final; zero bends>3deg,
maxloggedvelocity1.303m/s, but conditional chi2 triggers and sharpness tails remain.
- Livox2 Huber3 full-route failure remains~62m, Cauchy~114m. The user identified
the218–238s slice as outside/trees and already distorted. This localizes a scene,
not the first numerical error. New recorder/confidence fixes confound route tests.
- W415 exact-control ablation already establishes causal influence of three thin
coarse cells. Repeating that discovery is not the next task. Its small covariance
widths and large normal residuals support a local confidence-model intervention;
they do not explain all later drift. Cauchy's local gain did not generalize.
- House-plan alignment is a separate registration/measurement question. A plan
fit cannot repair odometry and must not be used to warp a candidate into shape.

## Source audit and research implications
Scratch scan_to_map.cpp754–801 uses sum(n*nT), relative eigenvalue ratios and
M=diag(P,I). It reduces translation directions only; it does not test full coupled
yaw/translation observability. Normal-information accumulation near line603 is
unweighted by robust weights/cell uncertainty and can count returns in both grids.
Thus many unreliable normals may appear diverse, while relative ratios alone
lack absolute/effective support. This is an audit limitation, not a measured new
cause of the flyaway. Prior Livox1 analysis found projection inactive in its
focused failure windows, so do not generalize that projection caused every run.

Legacy ndt_map.cpp67ff fixes mean at the first return, normal at three points and
updates covariance conditionally on viewpoint. ndt_map.h inverse usability is an
elementwise inverse-matrix threshold, not a physical covariance floor. Robust Huber
weights multiply both Hessian/RHS once; no sign inconsistency established. Rotation
and translation units must be normalized before interpreting a full6DoF spectrum.

scan_to_map.cpp1260ff computes carried-state covariance from the undamped system
then accepts successful solves subject to the pose-vs-guess translation guard.
There is no complete-objective decrease/overlap acceptance gate. Existing matched-
only reassociated NDT cost can decrease by losing support; the trace explicitly
marks complete_objective=0. A naive line search on that cost would be unsafe.

Primary sources and detailed assumptions: open-closed-terra-research.md.
X-ICP motivates direction-specific localizability tests, FAST-LIO2 separates local
support/plane quality from pose solving, and PCL regularizes covariance eigenvalues.
None supplies calibrated dMapper thresholds or proves the current cause.

## Recommended sequence and decision gates
1. Finish the plan-registration audit with one fixed rigid transform, scale and
independent structural correspondences. Preserve starting-room ambiguity and
check held-out structural surfaces/visits. Separate scan alignment error from
local map deformation and plan line/scale uncertainty. Do not adjust the map.

2. Bound the remaining mechanism analysis using existing data. Reuse W415 exact
control as the known local test, plus existing W754 failure evidence. Inspect the
later sustained Livox2 escalation around W904–916 from existing logs/snapshots,
including timing of support/geometry change, gyro/bias/velocity carry and map
rebuilds. Report what is actually recorded. Detailed trace ends before that late
interval; do not claim missing factor contributions were measured. If that gap
blocks the decision, specify one targeted checkpoint/trace capture for user
approval rather than launch another full run or recreate approximate states and
label them exact. End this phase with one selected mechanism branch or a precise
missing-evidence statement. No broad parameter search or repeated W415 discovery.

3. Preferred first numerical candidate, subject to approval: an opt-in physically
justified residual-covariance floor, preserving Huber and all other terms/map
statistics initially. Derive a defensible variance range from existing stationary
Livox3 observations and repeatability, keeping plane fit error, range/incidence,
pose/map uncertainty and noise dependence explicit. Do not set the floor by which
value makes Livox2 return. Existing wall MAD alone is not a calibrated point-noise
sigma. Compare existing exact-statistics option as an alternative later, not a
bundled simultaneous change. First check finite/SPD behavior and matching cost/
Hessian/RHS consistency, and verify the disabled path is unchanged.

If thin-cell confidence does not explain the later failure, switch branch before
implementation: support-aware correspondence quality/state-carry protection if
matches are inconsistent, or a unit-normalized full-pose/IMU-relative weak-direction
policy if reliable matches establish rotational degeneracy. Stronger projection,
an environment classifier, hard angular clamps and disabling bias are not default
choices. A trustworthy update-acceptance rule first requires a complete objective
and explicit support handling; that is a larger separate proposal.

4. Proposed future validation, each run separately approved: exact W415 baseline
then one candidate checkpoint; a control-reproducing longer failure segment with
its incoming state/map if available; only a surviving candidate advances to full
Livox2 and the frozen Livox3 indoor preservation check. Stage-specific trajectories,
all predefined matched physical surfaces/coverage, gyro consistency, velocity,
bias/support and revisit geometry must be checked, not just endpoint or one turn.
Use perturbation/noise-floor evidence before ranking numerical improvements; retain
failed members. Old sensor44 remains a regression gate, not Livox calibration.

5. A useful indoor/outdoor fix requires both environment classes and transitions
to behave, plus explicit degraded/unobservable reporting where position cannot be
observed. Large caves may have axial degeneracy or sparse distant support; walls
alone do not guarantee observability. Do not promise IMU-only drift-free recovery.
On acceptance update existing docs HTML and OpenSpec, with limits and evidence.

## Current stopping point
Research and registration analysis only. No algorithm/settings edits or processing
runs. Await user review of this concrete proposal before implementation or runs.
Sol6.1 High is currently user-confirmed and adequate for this planning work; ask
before requesting a more capable model/effort. Terra research used as requested.

Proposed implementation tests (after approval): synthetic room/corridor/open-plane
cases with controlled weak yaw/translation, deliberately overconfident wrong cells,
and separate contaminated-prior inputs. Require correct finite/SPD residual
information, consistent cost/Hessian/RHS, unchanged disabled path, and honest
behavior where motion is unobservable. These discriminate confidence and geometry
rather than classifying environments by a label. Table-rest returns can constrain
an empirical noise/fit range but cannot alone calibrate all ranges/incidences or
separate sensor, deskew, plane and map-state uncertainty.

## Registration outcome and review
Registration audit now provides a broader fixed-scale candidate at19.32690deg
versus prior22.25409deg (about2.93deg difference), with one transform for all visits.
Before/after image: BASE/livox3-indoor/huber3-baseline/registration-audit/
registration-before-after.png. This improves some long-wall directions but does
not reconcile all surfaces; held-out residuals are mixed, including0.1–0.4m
mismatches. Broad correspondence gates can include fixtures, sloped roofs and
windows rather than the intended wall face. It is a preliminary registration
candidate, not a corrected/cloud-warped map or a surveyed accuracy confirmation.
Use the structural residual table and identification of actual wall faces before
calling remaining disagreement odometry deformation. No scale/warp was fitted.
Independent Sol6.1 review found no blocking issue in this proposal's bounded
sequence, trace-gap handling, conditional covariance-floor design or validation
gates. Terra note corrected to preserve already-established W415 causal evidence.
