# Why the minority range group can dominate

Existing-data follow-up to full-livox2-trace-analysis.md. No mapper execution,
settings change or corrective filtering. High remains useful for causal work.

## Measurements at window754 iteration0, before the step

| Grid / range | Matched terms | Euclidean residual RMS, m | Alpha-scaled cost per match | Unscaled cell-information trace per match |
|---|---:|---:|---:|---:|
|Indoor0–2m|10046|0.102|1.0|6845|
|Indoor2–5m|6237|0.104|1.5|6495|
|Indoor5–15m|145|0.104|2.0|7351|
|Outdoor0–2m|8583|0.265|0.6|1701|
|Outdoor2–5m|5405|0.304|2.5|1145|
|Outdoor5–15m|275|0.287|28.7|8820|

The outdoor5–15m group has approximately5.2–7.7times the per-match cell
information of nearer groups, with comparable Euclidean residual magnitude.
Its weighted cost per match is much higher. At iteration20 its per-match
information is11125 and cost40.7, with residual RMS0.282m. This supports a
weighting/residual-direction explanation, not simply more points or larger
Euclidean mismatch. Indoor information contrast is much smaller.

The strongest eigenvector of the summed far-cell information is nearly worldZ.
This is not proof of a floor or an isolated tilt constraint. Information sums
exclude rotation Jacobians; the actual angular leverage also depends on return
coordinates. `range_step` combines both grids, so its signed preference cannot
yet be attributed specifically to the outdoor grid or individual returns.

## What the matcher actually does

Source inspection at scratch5eb882e and frozen settings confirms:

- Uses the cell containing the transformed return, rather than nearest-neighbor
  search. Range, cell existence, inverse usability and viewpoint/normal checks
  gate matches. There is no Mahalanobis-residual rejection or robust residual
  kernel in these point loops.
- Rotation Jacobians contain source-point coordinates: long returns can have
  greater angular leverage. There is no explicit range-noise attenuation here;
  norm/planarity weighting is disabled. Both0.15m/0.4m grids can use the same point.
- `use_exact_ndt_bucket_statistics=false` is explicit in the archived TOML.
  Legacy cell means remain the first point; normals use the first three points;
  later covariance updates depend on viewpoint. No registration minimum-cell-
  population gate is applied. This is a candidate mechanism, not proof that the
  implicated cells are sparse or wrong.
- Covariance is inverted directly; cells are rejected when an inverse entry
  exceeds±100000. That is not a covariance eigenvalue floor or eigenvalue clamp.

Relevant source: `src/domain/scan_to_map.cpp` point loops and `trace_matches`,
`src/domain/ndt_map.cpp:update_rgd_legacy`, `src/domain/ndt_map.h:ndt_inverse_usable`.

## Rematching does not show a simple improvement

For this fixed window/map, summed rematched NDT diagnostic cost rises from
45935.4 before iteration0 to69320.0 after iteration99 (about51% higher), while
matched terms across both grids fall30691→26015. All steps were accepted.
Local frozen-association quadratic improvements therefore do not imply a lower
rematched NDT diagnostic. This is informative but excludes other factors and
unmatched-return penalties; it is not a complete-objective or accuracy verdict.
Changing correspondences makes summed local quadratics different from the
finite difference of this rematched diagnostic.

## Scene context and missing attribution

The retained1/20 raw sensor-frame sample for754 has2770 returns,51 in5–15m.
Those far samples lie mainly in sensor+X/+Y; their5/50/95-percentile elevation
angles are about−3.9/9.9/21.3degrees. This is unregistered, sampled raw context,
not the exact decimated matched set. It does not establish an indoor/outdoor
location, a physical surface, moving object or reflection.

Missing evidence: grid-separated and per-return influence at selected754
iterations, with target cell identity, occupancy/covariance eigenvalues,
residual and rotation Jacobian. Aggregate means can hide a few extreme cells.
Do not choose a range cutoff, robust-kernel threshold or exact-statistics setting
from these aggregates alone.

## Provenance and review

`tools/trace_range_summary.py` streams selected windows into external
`full-livox2-trace/range-analysis/ranges.csv` and a hash manifest. All17600 rows
cover11 windows×100 iterations×4 stages×4 bins. All4400 stage totals conserve
matched counts and alpha-scaled costs against the prior summary. Original trace
SHA256 matches the validated replay; input files remain unchanged.

Independent Codex review (not Sonnet) confirmed the reductions and source
interpretation. Corrections incorporated: distinguish unscaled cell information
from the joint Hessian, combined-grid influence from outdoor attribution, matched
terms from unique returns and rematched diagnostic cost from a complete objective.
