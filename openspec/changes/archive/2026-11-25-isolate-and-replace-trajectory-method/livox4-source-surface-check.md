# Original W298 source-surface mechanism check

2026-11-15. User authorizes continued fixes and tests without replay approval
stops. This bounded test uses existing data; Charlie remains3/19.

## Evidence and question

The original0.1/0.1 Huber3 trace identifies NDT as the initiating tilt factor.
Its W298 update includes strong translation-to-rotation coupling; ranking only
angular RHS or RHS work does not predict the effect of removing observations.
The continuous native100-call case matches22,500 trace records exactly.

Existing-data PCA on20/40 nearest deskewed source returns finds62 fine and144
coarse W298 pairs with reliable local surfaces disagreeing by more than60deg
(191 distinct source indices). W299 has213/314. Reliability requires both
neighborhood radii below0.15m, a planar eigenvalue ratio below0.1, non-line-like
support, and normal agreement within10deg between neighborhood sizes. These
are diagnostic selection criteria, not an adopted correspondence rule.

Independent reviewer checked source IDs/target keys against both native CSVs.
Only14 of242 high-to-floor coarse W298 matches meet this reliable-mismatch
classification. A negative test cannot rule out the ambiguous majority.

## Native diagnostic

Remove the191 identified source returns and reassemble the complete native
Hessian/RHS, projection and priors, with normal rematching through the local
solve. First require unchanged control to reproduce every original trace record.
Do not change processing settings, production assembly or target map.

Whole-source removal also removes unflagged matches of those sources in the
other grid and later valid associations. Enumerate those extra removals. The
reviewer's preferred pair-specific removal would need a new assembly/cost hook;
the simpler whole-source test can establish sensitivity but cannot isolate only
the initially flagged pairs. Record production convergence separately from any
forced100-step diagnostic continuation. No recording run or candidate adoption
follows automatically.

Artifacts are under external
`livox4-original-tilt-trace-125s/solver-replay-W298/source-surface-ablation/`.

Completed: unchanged control matches22,500 trace records. Both native paths
apply100/100 updates; neither reaches configured delta threshold1e-12 (minimum
about3.06e-7). Removing191 sources also removes33 unflagged matches in each
grid at iteration0. Node164 initial roll/pitch/yaw update changes from
(-0.2573,+0.2369,-0.7703)deg to(-0.2069,+0.2448,-0.3497)deg.
Its final heading change relative to the common seed changes-2.475->-1.107deg.
Maximum final tilt from the seed changes0.62785->0.57474deg; final between-run
position difference is at most5.830mm. Thus the selected sources materially
influence yaw but do not explain most of this window's tilt. This is not a
map fix or a rejection of the broader correspondence hypothesis.

## Possible soft alternative, not selected implementation

[Generalized-ICP](https://www.roboticsproceedings.org/rss05/p21.pdf) models both
source and target local surface covariance through
`Ceff=Ctarget+R*Csource*R^T`. Its different registration formulation is not a
tested dMapper fix. The present fixed-pair probe adds empirical20-neighbor
source covariance to native target covariance; it changes no trajectories.

With Huber3 recomputed, median coarse-grid precision along the target normal
falls to0.231 of baseline for the reliably incompatible W298 pairs, versus0.807
for source-reliable pairs within15deg. W299 ratios are0.129/0.646. This checks
the expected selectivity of a soft covariance model, not the resulting solver
motion. Neighborhood scatter is not calibrated measurement noise, and reducing
information can also allow drift. No manually chosen covariance floor is used.

Probe and tensors: `cell-inspection/source-covariance-probe.json`,
`W298-source-covariance-probe.npz`, `W299-source-covariance-probe.npz`;
archived Python script and hash accompany them. Any native test must preserve
correct body/world covariance transformations, apply the same information to
assembly and cost, reproduce the disabled control and inspect fullH updates.
The gain-fixed run has a different map/state; this original case is mechanism
evidence, not its exact W313 reproduction.

## Authorized bounded covariance test

Proceed with the existing W298 case only, using a nullable diagnostic covariance
input, no settings/default or recording change. Preserve original target-cell
eligibility and all weights. Recompute combined covariance and Huber weights
at each native call in both grids and diagnostic cost; hold that metric fixed
inside its linearization. This omits derivatives of covariance with respect to
rotation and is an iteratively reweighted probe, not the full Generalized-ICP
objective gradient. Explicitly inspect support and wholeH motion, not only
the changed residual value.

Body covariance export validates source coordinates/indices/time against native
cache, converts by inverse initial affine linear transform, verifies world
roundtrip within1e-12, and hashes inputs. W298 has14,367 positive-definite
tensors (minimum eigenvalue6.16465e-7m2). Native loading must recheck identity,
finite/symmetric/PSD data and preserve exact disabled control before the probe.
Tests cover zero source covariance, frame transformation and invalid values.
Keep this diagnostic extension removable and separately committed.

Prepared input: `cell-inspection/W298-source-covariance-case-v2.json`, SHA256
`30e95bb97767dfc1fbc12df42578687f37473b295550e0f4fad52aa6871f67b0`.
V2 preserves V1 and adds seed linear blocks and world covariance for native
identity/roundtrip checks. Transport uses the same raw affine linear blocks as
point placement; maximum initial deviation from a proper polar rotation is
8.57e-8 per matrix entry. Independent reviewer approved this consistent
transport and bounded test, with no full-run/adoption inference.
Results will use `solver-replay-W298/source-covariance-probe/`. No effect on the
actual map is claimed before a later matched prefix and full-route check.

## W298 covariance result

Release focused6 cases/2,180 assertions passed. Independent source review clear;
disabled control22,500 records exact. Covariance i0 matches and projection exact
to control, with original10,824 fine/7,952 coarse matches. Both paths applied100
steps; neither met the1e-12 delta threshold.

An independent SciPy solve reproduced every native i0 update entry within
1.4e-12. Equal aggregate projected XYZ NDT Hessian trace gives a uniform
attenuation control of0.5729401103; all other terms and native pose-only damping
are retained.

| i0 measure | Original | Source covariance | Matched uniform attenuation |
|---|---:|---:|---:|
| Maximum gravity-axis tilt from common seed |0.3590deg|0.2555deg|0.2933deg|
| Node164 Euler yaw update |-0.7703deg|-0.1212deg|-0.7122deg|
| Largest translation increment |10.246mm|11.261mm|8.672mm|

After100 calls, maximum seed-relative tilt changes0.62785->0.39590deg and
node164 Euler yaw change-2.47490->-0.34612deg. Maximum translation from seed
changes16.042->15.114mm. These are solver responses, not independently measured
pose errors. The selective yaw effect exceeds this one uniform attenuation
control; it does not rule out every spatial/directional reweighting alternative.

Final refreshed counts are10,763/7,932 control versus10,778/7,913 covariance.
Independent review found association identities diverge afteri0; effects include
normal rematching feedback. Close counts do not establish identical membership.

Evidence: own `comparison.json`, `independent-i0-comparison.json`, original
and candidate native traces, binary i0 matrix captures and their hashes. Source
and numerical evidence independently reviewed by seed_damping_crosscheck.

Next gate: repeat identical20NN model, i0 equal-trace uniform attenuation and
final native tilt/yaw/translation/support checks on captured W299, requiring
all100 original calls exact first. Use original W299 incoming map/state; do
not call it a sequential candidate continuation fromW298. No recording run yet.

## W299 result and transfer decision

Helper-only extension cfba888 follows covariance implementation f2b697f.
All22,500 original records and initial match/projection identity passed. Both
paths applied100 updates with no early convergence. Independent i0 solves
match native within7.4e-12; equal-trace uniform attenuation is0.4559708342.

| i0 measure | Original | Source covariance | Matched uniform attenuation |
|---|---:|---:|---:|
| Maximum gravity-axis tilt from seed |0.6780deg|0.5706deg|0.6352deg|
| Node164 Euler yaw update |+0.2424deg|+0.3325deg|+0.2269deg|
| Largest translation increment |20.241mm|18.692mm|17.077mm|

After100 calls, maximum seed-relative tilt1.23077->0.81797deg; largest
translation27.034->21.595mm. Node164 yaw+0.59723->+0.61035deg, and final-node
yaw+0.292->+0.637deg: yaw benefit is not universal. Initial matches9,597/7,497;
final counts9,708/7,474 versus9,685/7,455. This is an independent original-state
case, not candidate feedback fromW298.

Independent evidence review supports one default-off gain-fixed prefix as a
transfer test. It does not establish a map fix or general improvement. Source
covariance reduces tilt more than the one matched uniform attenuation in both
cases, but remaining tilt and worsened W299 yaw remain explicit risks.

Runtime preparation now implements the conditional plan below. Persist
`[lidar_odometry_control] use_ndt_source_covariance=false` by default and bind
immutable per-window tensors through optimization and diagnostics. Planned
prefix folder: external `livox4-source-covariance-125s`, compared directly with
`livox4-matched-gain-fix-125s`, changing only this flag. All346 cached windows
have at least13,479 source points, above the20-neighbor requirement. Builds,
review and captured tensor reproduction must pass before launch.

## Conditional recording candidate

Only if W299 supports the same mechanism, prepare one default-off experimental
recording path on the correctly matched gain-fixed baseline. No guard/acceptance
stack, weight retune, target-statistics change or further model is bundled.

Compute the same20 nearest source returns in the initial predicted world frame
once per window, including the query return. Use population covariance (divide
by20), then inverse initial affine transport into each source's body frame.
Keep these tensors fixed through the window; transport them by each current
pose for the existing tested information helper. A deterministic exact3D
nearest-neighbor implementation must match brute-force distances, handle ties
by source index, and reproduce captured-case tensors before recording use.
Reject nonfinite input, invalid pose indices or insufficient support explicitly;
do not silently substitute a new model.

The covariance vector must remain alive for optimization and window diagnostics.
Keep run-level options unchanged and bind the vector only in a per-window option
copy. Selection must be persisted in TOML, default off, with shared GUI/CLI
behavior and result provenance. User-facing replay file remains
`settings/livox-vertical.toml` paired with the candidate executable.

Require disabled behavior identity, numerical/frame/neighbor tests, settings
roundtrip tests and independent review. Then one125s matched Livox4 prefix,
same fixed plan alignment and marked returns, early indoor preservation and
odometry/final checks. A local successful solve alone cannot trigger a full
run. Only a prefix with credible geometry proceeds to full Livox4, indoor
Livox3 and remaining project robustness/regression gates.

## Runtime implementation and prefix launch

Implementation is frozen at92fe2a8aaab9b86e015f22984d19882468e45c17, including
builder61acceb, runtime7418ffa and separate regenerated golden fixtures92fe2a8.
Focused Release and Debug each passed17 cases/3,529 assertions. Full Release
has141 passing cases, the unchanged historical corridor failure17.9039mm versus
7.5mm, and3 skipped cases. Final disabled native W298 matches22,500 records.
Independent review found and resolved GUI preset stickiness: omitting the new
flag now resets it false; malformed input retains the prior configuration/path.

Native20NN reproduces every W299 tensor and all but one W298 tensor within
1e-12m2. At W298 source12278, world-coordinate roundoff reverses two neighbors
whose squared distances differ by approximately3e-17m2. Native selection
matches native brute force; no tie-specific rule was changed. A fresh native
W298 check using all native-builder tensors retains the result: final tilt
differs by0.000026deg, node164 yaw by0.000018deg, and maximum translation by
0.211micrometers from the Python-tensor probe. Original control remains exact.
Artifacts: `solver-replay-W298/source-covariance-native-builder-probe/`.

Prefix launched2026-11-15 04:42:33 local, PID14772, sequence33456, at external
`livox4-source-covariance-125s/`. Frozen executable SHA256:
`c3dffd0287f58642815073534940eb54be21b41865f4e5e2030a6300bb196ea3`.
Both compiled scales are0.1. Actual startup confirms Huber3 and source covariance
exactly once, no gravity/admission activation. The frozen TOML differs from the
matched gain control only by `use_ndt_source_covariance=true`. Project
`settings/livox-vertical.toml` now matches this candidate exactly and requires
its paired frozen executable; this is an experimental replay, not adoption.

Root sequence runs automatic fixed-plan comparison against the matched gain
control after completion, including same marked returns, offset, line direction
and early60s position disagreement. Build/test/provenance evidence is in the
run's `validation/`. No result or full-run acceptance yet.

## Completed prefix: failed geometry gate

All346 windows completed, native exit0; input/build/configuration hashes and
actual source-covariance activation verified. Wrapper time1437.12s versus
582.01s gain control; odometry1129.36s versus347.01s. Neighbor construction
accounts for54.27s, so it does not explain most of the additional runtime.
No further performance investigation is justified for this failed candidate.

Same27,299 marked sources, including23,442 late returns, fixed preliminary
alignment and approximate expected horizontal wall:

| Stage | Absolute offset p50/p90 | Selected line angle |
|---|---|---|
| Gain control odometry |3.543/4.207m|+12.295deg|
| Covariance odometry |1.050/5.691m|-56.942deg|
| Gain control final |3.062/3.673m|+10.941deg|
| Covariance final |1.161/5.804m|-56.829deg|

The lower median is a diagonal crossing the expected line, not alignment.
Both candidate panels visibly deform the marked wall before and after
post-processing. Early60s position disagreement max/p95 is0.258/0.136m;
gross indoor layout remains recognizable but matched indoor revisit preservation
was NOT MEASURED. No robustness or adoption claim.

Saved W301 map/AHRS tilt drops6.729->4.731deg, but W313/314 logged heading
corrections reach11.875/15.005deg. AtW325 the saved relative AHRS heading is
87.828deg versus7.182deg control; W346 starts12.290m apart. This is diagnostic
AHRS disagreement, not independent orientation truth. Complete log coverage:
17 bends above3deg,4 above10deg, maximum speed2.41m/s. Low speed does not cure
the map. Subsequent candidate support/factor attribution is NOT MEASURED.

Root inspected `quality/plan-comparison.png`; independent Codex reviewer
seed_damping_crosscheck verified provenance, same-source geometry and failure.
No full replay of this candidate. Preserve its binary, settings, output and
optional implementation; disable it in the user's replay preset pending a
different justified correction. Evidence remains in its own external run folder.

## Next bounded existing-case check

Terra review eliminated two repeats: exact cell statistics was already tested
and failed, and translation projection was identity throughout originalW298.
It identified the very strong first-node increment regularizer as a separate
structural constraint. Each iteration adds six1e12 diagonal entries with zero
RHS; it is not a fixed-base residual. OriginalW298 i0 moves the first node only
about8nm, while downstream nodes can deform. This does not establish that the
incoming pose is wrong or that removing the anchor fixes the map.

Before another recording, use the existing native W298/W299 matrices to
reproduce the baseline solve and compare a diagnostic free-first-node solve,
removing only those six incremental regularizer entries. Retain all other
priors, damping and NDT terms. Measure base motion, deformation and factor
quadratic responses; no production change or accuracy inference from a lower
fit cost. Preserve outputs in new `origin-anchor-probe/` folders. A common
rigid-space comparison may follow only with explicit frame/nuisance treatment.
