# W754: exact attribution to coarse-grid cells

Follow-up to full-livox2-range-analysis.md. This investigation reconstructed
the historical map from existing evidence; it did not rerun odometry or alter
settings. Root Python tools extract states and summarize trace statistics;
a standalone C++ helper reuses the exact production map arithmetic.

## Reconstruction gates

Snapshot poses are the post-handoff poses used for map insertion. The helper
loads and sorts the original filtered seed, retains60001 points, applies m_g,
and inserts cached windows1–753 chronologically with each original viewpoint.
There is no map rebuild before754; first retained-run rebuild is882.
Legacy insertion order and per-window call boundaries are preserved.

At iterations0,1,10,20,40,99, both grids match all recorded source-order,
matched-cell and matched-cell-geometry hashes and counts exactly. Per-grid costs
and both range quadratics pass finite normalized1e-9 checks before export.
All-kept projection follows the actual solver's exact-identity path, not the
rounded eigenspace matrix printed in the trace. Inputs are hashed and rechecked.
These gates establish the reconstructed associations, not their physical truth.

Release and Debug standalone builds passed. Initial Debug sandbox compilation
encountered MSVC's PDB-manager error; elevated compilation passed. The helper
does not call the optimizer, processing pipeline or cache writers. A negative
test verified rejection of an existing output directory leaves it unchanged.

## Attribution at iteration20

For the recorded joint step, q=h^T H h−2b^T h:

| Grid / range | q |
|---|---:|
|Indoor0–2m|+501.99|
|Indoor2–5m|+667.43|
|Indoor5–15m|+157.60|
|Outdoor0–2m|+155.67|
|Outdoor2–5m|+199.05|
|Outdoor5–15m|−4162.15|

Thus at this iteration, the coarse-grid longer-range matches favor the joint
step while every other group opposes it locally. This is now grid-separated
evidence, unlike the earlier aggregate trace. Near groups also favor some earlier
steps, so this table must not be generalized to all100 iterations.

Three coarse-grid cells account for most of the favorable far-group contribution:

| Cell key | Matched returns at iteration20 | Stored occupancy | q |
|---|---:|---:|---:|
|77310918653|18|184|−3153.87|
|73015885821|3|292|−634.63|
|81605885949|5|240|−414.96|

Other far terms partly oppose these contributions. These are26 coarse-grid
matches, not26 independently measured constraints. This excludes low occupancy
as the explanation for these particular cells; it does not validate their
covariance or independence of accumulated returns.

The strongest cell's stored mean is approximately[7.375,8.890,−0.807]m in the
estimated world frame. Its smallest covariance eigenvalue is1.251668e−5m²
(model standard deviation3.54mm); its normal is nearly vertical. At iteration20
its18 matched returns span8.38–8.60m range, with median vertical residual−0.202m.
At iteration0, its11 matches already have square-root Mahalanobis distances
2.05/26.78/108.52 (minimum/median/maximum), and it is already the dominant far
cell. These are distances under the cell's model, not calibrated probabilities.
No residual rejection or robust kernel prevents their full covariance-weighted
contribution. Angular leverage further amplifies the long-range contribution.

Do not call this a confirmed floor, reflection or erroneous physical surface.
Its height is above the estimated sensor at this time; a nearly vertical normal
alone does not identify the structure. Nor does q isolate tilt torque: the
recorded step contains pose, velocity and other state updates.

## Provenance and review

Artifacts: `full-livox2-trace/reconstruction-w754/validation.json` and12 per-grid
CSV files. Exact states come from `range-analysis/w754-states-v3.json`, bound to
the validated trace. Scratch tool commit9653ae0; history extensiond87b89f.
Frozen history-tool bundle is `full-livox2-trace/reconstruction-tool-v2/`.
All paths are under the approved dLidar experiment directory.

Independent Codex review (not Sonnet) verified the reconstruction implementation,
gates, exported sums and top cells. It corrected any inference from coarse-grid
influence to physical wrongness or isolated tilt causation. Cell insertion
history was subsequently reconstructed as described below.

## Insertion history

The extension replays only the three selected cells into a separate audit map,
preserving insertion order and viewpoint. Their final means, normals, covariances
and occupancies match the validated full map bit for bit. All earlier gates pass
again; all12 per-return CSVs are byte-identical to the first reconstruction.
Artifacts: `full-livox2-trace/cell-history-w754/`.

The cells were populated in windows623–633. Actual minimum/maximum point
timestamps are224.367075–228.132875s (recording clock), roughly43–47s before754.
There were zero viewpoint-rejected updates. Neither sparse occupancy nor
viewpoint-rejection shrinkage explains these particular three cells.

The dominant cell's184 inserted points form a thin horizontal estimated-map
patch: z ranges−0.812122 to−0.800043m. Its stored weakest standard deviation is
3.54mm. Centered population scatter of those same points has minimum eigenvalue
4.853132e−6m²; this is not the alternative exact-statistics implementation, which
also has a prior and different normal/update rules. It does not establish that
switching statistics would fix the problem.

Registration accepts much larger residuals within the0.4m bucket and applies the
full inverse covariance. Together with long angular leverage and the prediction-
relative translation guard, this explains how a few coarse-grid associations can
outweigh other terms during the recorded update. It does not prove physical
surface identity or that deleting them is a general correction. The normalized
distances use map scatter, not calibrated measurement-plus-pose uncertainty.

Independent review confirmed the history and corrected timestamp reporting:
cache insertion order is not chronological within a window; first/last rows are
not necessarily temporal extrema.

## Proposed next experiment — not executed

Two isolated optimizer checks on reconstructed W754, without a full replay:

1. Control: restore exact initial poses/model poses, IMU/node samples, gravity,
   velocity/bias/offset state and priors from existing trace/snapshot; run the same
   100-iteration loop against the reconstructed map. Require exact reproduction
   of traced states/decisions before interpreting an intervention.
2. Ablation: restart from that identical initial state and omit only the three
   identified coarse-grid cells. Keep indoor map, other cells, alpha/beta, priors
   and iteration schedule unchanged. Compare tilt, endpoint velocity, acceleration,
   accepted updates and residual/support coverage, including deliberately
   unmatched returns.

This tests their causal influence on the window. It is not a shipping filter or
evidence of a corrected full cloud. If control reproduction fails, stop without
interpreting the ablation or automatically broadening runs. New optimizer
execution requires user go-ahead under the stop-before-new-runs instruction.
No optimizer execution was performed during this investigation.
