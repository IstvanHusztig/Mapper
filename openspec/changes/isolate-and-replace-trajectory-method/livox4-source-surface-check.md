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
