# General residual-influence diagnostic

2026-11-13. Follows the exact W754 control and three-cell intervention in
`full-livox2-w754-ablation.md`. The user resumed work and then explicitly said
"I give you the go now" for the proposed exact W754 control plus Huber candidate,
after build, synthetic tests and independent review. No full replay is authorized.

## Choice and evidence

Test one general Huber weighting rule, identically on both NDT grids. It does not
identify surfaces by cell key, range or room. Keep alpha0.1/beta0.1, initial state,
priors, map statistics, correspondence lookup and solver/projection rules unchanged.
The change is how much each current residual contributes to the solve.

Source review found no residual robustification in the point terms. Existing
`use_norm` multiplies by range, so it is not a suitable distance attenuation fix.
The exact intervention establishes influential associations, not a defective
sensor, physical reflections or a particular correct uncertainty floor.

Primary references:

- [Ceres robust-loss definitions](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html#lossfunction)
  describe Huber and Cauchy loss. We use positive IRLS weighting; this is not a
  claim to implement Ceres' full robustified Hessian or its solver.
- [PCL NDT implementation](https://raw.githubusercontent.com/PointCloudLibrary/pcl/master/registration/include/pcl/registration/impl/ndt.hpp)
  uses an exponential Mahalanobis score with decaying large-residual influence
  and a step search. Our current quadratic point terms differ from that model.
- [Generalized-ICP](https://www.robots.ox.ac.uk/~avsegal/resources/papers/Generalized_ICP.pdf)
  uses a surface covariance metric whose practical regularization does not make
  it calibrated measurement uncertainty.
- [Probabilistic VoxelMap](https://arxiv.org/pdf/2109.07082)
  distinguishes plane-parameter uncertainty from point spatial scatter and
  propagates sensor/pose uncertainty. That is a separate, broader model change.
- [Ceres trust-region methods](https://ceres-solver.readthedocs.io/latest/nnls_solving.html#trust-region-methods)
  use actual/predicted objective reduction. Such acceptance is a possible later
  safeguard, but decreasing an incorrect objective can still yield a bad pose.

Local source review preferred Huber as the smallest intervention; primary-source
research suggested the stronger Cauchy tail as another option. Read-only
reweighting of the retained W754 correspondences supports trying Huber first:

| Fixed historical calculation, cutoff3 | Quadratic | Huber | Cauchy |
|---|---:|---:|---:|
| Initial indoor mean weight | 1 | 0.9223 | 0.6188 |
| Initial outdoor mean weight | 1 | 0.9363 | 0.6589 |
| Initial three-cell joint-step quadratic | -2196.20 | -20.66 | +42.22 |
| Iteration20 three-cell joint-step quadratic | -4203.46 | -118.29 | +37.30 |

Artifact: `optimizer-w754-control-ablation/robust-influence.json` with source
hashes and archived Python script. These use the original poses, associations
and proposed joint step; they do not predict a reoptimized trajectory or robust
objective decrease. Equal-count mean weight is not retained information/rank.
The evidence does not establish Huber's superiority over Cauchy. Cutoff3 is an
explicit diagnostic value in the existing cell metric, not a calibrated3-sigma
gate. Robust scale sensitivity and physical uncertainty remain untested.

## Exact intervention

For each matched return, with residual r and original cell information I:

`q = r^T I r`, `s = sqrt(q)`, `c = 3`.

`w = 1` for `s <= c`, otherwise `w = c/s`.

Multiply that point's Hessian and RHS by the same w, once, before external alpha.
The robust point cost is `alpha*q` below the cutoff and
`alpha*(2*c*s-c*c)` above it. Recompute w from current residuals each iteration.
This is an IRLS surrogate; the Hessian omits the second derivative of the loss.

Expose only a scratch diagnostic field in ScanToMapOptions, default0 disabled,
and a standalone checkpoint-driver flag. No shipped TOML, GUI or default change.
Disabled mode must reproduce the exact original arithmetic and trace. Enabled
mode is restricted to the already-used norm=false/planarity=false configuration
to avoid silently changing legacy weighting differences between grids.
Nonfinite/invalid metric values must fail explicitly in this diagnostic path.

Keep raw matched-only cost distinct from robust cost. Log per-grid weight totals,
downweighted counts and weighted information as well as actual factor/range-step
contributions. Correlated map points and long angular Jacobians can still matter
after Huber weighting; do not describe it as a complete outlier solution.

## Gates and authorized execution

1. Unit checks: disabled parity, zero residual, cutoff continuity, robust-cost
   finite-difference gradient, consistent IRLS Hessian/RHS, coordinate rotation,
   both grids and serial/parallel behavior. Appropriate build configurations and
   independent source review must pass first.
2. Freeze the new executable/source/input identity in a fresh external folder.
3. Run the same100-iteration W754 control with diagnostic weight disabled. Require
   exact agreement with every original trace record before any candidate step.
4. Restart identical state/map with Huber3 enabled; no cells removed. Preserve
   all traces and outcomes. Stop at any control mismatch, without automatic retry.
5. Compare tilt, endpoint velocity, rotated IMU context, raw/weighted support,
   projection and information. Independently review the result. A beneficial
   checkpoint is not a corrected full cloud, physical accuracy or robustness pass.

This authorization covers that control and candidate only. Full-route replay,
other cutoffs, different kernels or additional checkpoints remain separate
decisions after these readings. High remains useful through causal interpretation.

## Requested Terra 6.1 challenge review

The user explicitly requested Terra research for stronger fixes. Terra's
independent research supports completing this diagnostic, while warning that
lower tilt alone could reflect lost valid constraints. Weight totals are not
observability; inspect directional information and support alongside motion.

Two broader candidates remain design work, not authorized implementation:

1. Model shared plane uncertainty across returns in a cell, for example block
   covariance `D + A*Sigma_plane*A^T` or explicit shared plane nuisance variables.
   Repeated observations should not multiply information in shared uncertain
   modes as though independent. Dividing every cell by occupancy is not a
   substitute: independent measurements and spatial coverage still add evidence.
2. Derive point-to-plane uncertainty from query noise, plane normal/offset
   estimation, surface discrepancy and map-building pose history. Avoid counting
   the same pose uncertainty both as measurement noise and as an optimized prior.

These are Terra's inferred designs, not claims that cited implementations offer
that exact block model. Relevant primary references are
[point-to-plane stochastic modelling](https://isprs-archives.copernicus.org/articles/XXXIX-B5/181/2012/)
and [the VoxelMap authors' implementation](https://github.com/hku-mars/VoxelMap).
They need defensible noise/correlation assumptions and controlled geometry tests.
Cross-cell construction errors may remain correlated. A true-wall displacement,
weak corridor geometry and held-out alignments would help distinguish preserved
geometry from mere motion suppression before accepting any production fix.
