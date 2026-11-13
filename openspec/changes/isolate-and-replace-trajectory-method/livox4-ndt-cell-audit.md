# Why W298's 2–5 m matches pull the rotation

2026-11-13. User requested the next causal investigation after the rotation-factor
trace. Existing data only: no new trajectory replay, optimizer solve, parameter
change or algorithm fix. Current high effort is sufficient.

## Result

The local admission/association weakness is concrete: occupying the same voxel
is sufficient to select its Gaussian, even when the incoming returns disagree
strongly with the historical surface or the cell has only two points. Huber3
downweights these residuals but retains them. Their narrow-axis information and
repeated contributions can still pull the joint solve into the observed bend.

This identifies the problematic constraints and the code path admitting them.
It does not prove whether their initial discrepancy arose from a different
physical surface, scene motion or earlier trajectory error. No specific door,
wall, reflection or universal correction is claimed.

## Exact reconstruction gate

Reused `tools/reconstruct_w754.cpp` in the isolated gain worktree, adding an
inspection-only W298 mode (scratch commit `b892823`). Its historical executable
name remains; `target_window=298` in validation is the actual scope. W298 mode
prohibits optional optimizer arguments. The application and optimizer are unchanged.

Read the actual prefix LAZ for its first 60,001 filtered, timestamp-sorted seed
points, transform with snapshot.m_g, then insert saved post-tail W1–297 cache
points in production order using the existing native map updater. The original
trace reports zero map rebuilds; reconstruction also checks the distance limit.
Use actual candidate alpha1/beta1 and Huber3, retaining the prior provenance
correction: the old control/candidate comparison was not gain-only.

For iterations 0, 1, 2, 10 and 99, both grids match the original trace exactly in
source identity, association identity, target mean/normal/covariance identity and
counts. Raw costs and Huber-weighted range quadratics match; largest normalized
quadratic discrepancy is 1.36e-14. Ten grid identity checks and twenty range checks
passed before per-return CSV export. All inputs are hashed and checked again.

## Two influential coarse-grid cells at iteration 0

Both use the 0.4 m grid. Rankings below are RHS work along the actual recorded
Euler-z update across nodes, not each cell's contribution to node164's angle.

| Measurement | Dense cell `281397668020223` | Sparse cell `281393373052926` |
|---|---:|---:|
| Historical points | 1,892 | 2 |
| First/last insertion windows | W285 / W296 | W297 / W297 |
| Historical times | 102.94967–106.78202 s | 107.20790 / 107.20792 s |
| Current 2–5 m matches | 32 | 213 |
| Model narrow-axis spread | 3.162 mm | no observed plane; seeded covariance |
| Defined normal | yes | no; zero vector |
| Mean current Euclidean residual | 258.1 mm | 231.6 mm |
| Mean retained Huber weight | 0.0498 | 0.3942 |
| Euler-z RHS work | 769.84 | 455.18 |

The two cells contribute 55.15% of **net 2–5 m Euler-z RHS work across both
grids** at the first step. This is not 55% of endpoint rotation. Their respective
shares of positive coarse-grid 2–5 m work are 45.65% and 27.33%; opposing terms
remain visible in `cell-ranking-v2.json`.

### Dense cell: incompatible returns survive robust weighting

The historical points form a narrow surface. Current matches lie 90.9–368.4 mm
away along its narrow covariance axis, median 189.7 mm, versus model spread
3.162 mm. Their median Mahalanobis residual magnitude is about 60, versus Huber
cutoff 3. These are model-relative units, not calibrated sensor sigmas.

Huber applies `weight = 3 / residual_magnitude` above the cutoff. It therefore
retains a substantial aggregate force from coherent incompatible matches. The
current group is not a clean parallel copy of the old plane either: its observed
3-D principal spreads are roughly 26.6/78.1/123.8 mm and its least-spread axis is
42.7 degrees from the old one. This supports geometric incompatibility; the
handheld scan pattern and limited sample count preclude a physical surface label.

The old first-point mean is 157.9 mm from the empirical centroid, mostly along
the surface. Moving it to that centroid shifts it only **0.216 mm along the
narrow axis**. Thus the known first-point-mean defect does not explain the
roughly 190 mm normal discrepancy. No covariance updates for this cell were
rejected by the viewpoint test. Do not revive the rejected exact-statistics or
residual-floor experiments on the assumption they were proved correct here.

### Sparse cell: a two-point seed acts as a repeatedly observed constraint

Its only two historical returns arrived 20 microseconds apart in W297, less
than 0.07 s before W298 begins. They are about 31.97 mm apart. The legacy map
has not formed a normal yet; nevertheless its seeded covariance is invertible.
The matching code has no minimum-support requirement. The zero normal gives a
zero viewpoint dot product, which passes the existing `< 0` rejection check.
Consequently 213 new returns are each pulled toward this inadequately supported
cell. The second-minimum eigenvalue is degenerate, so a plane-normal angle for
this cell is undefined, not 90 degrees.

## Code boundary

Executed source: `C:/Repo/dMapper/out/ahrs-gain-worktree/` at `f400479` for the
trajectory, with inspection-only utility change `b892823` afterward.

- `src/domain/scan_to_map.cpp:516–539` and `670–693`: containing-cell lookup,
  inverse usability and viewpoint check; no minimum occupancy or correspondence
  consistency test.
- `src/domain/scan_to_map.cpp:589–598` and `731–740`: Huber weights the existing
  match; it does not reject it or reconsider the surface assignment.
- `src/domain/ndt_map.h:19–33`: inverse-entry threshold is the numerical admission
  check; it is not a cell-support or surface-compatibility measure.
- `src/domain/ndt_map.cpp:66–145`: legacy cell construction, initial covariance,
  first-point mean, third-point normal and subsequent covariance updates.

This explains why the solver receives the rotational pull. It is not evidence
of a new derivative/sign bug: the optimizer is responding to admitted constraints.
It also does not erase the earlier findings about coupled translation, angular
acceptance and changing associations.

## Evidence, review and next boundary

External folder:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ndt-cell-audit/`

- `cells/validation.json`: native reconstruction gates and input identities.
- `cells/iteration-*-*.csv`: per-return target geometry, residuals, information,
  robust weights and six RHS components.
- `cell-ranking-v2.json`, `history-v2.json`: current conclusions and histories.
- `influential-cells.png`: existing map points versus incoming matches before
  the bad update; coordinates are estimated, not surveyed truth.
- `audit-manifest.json`, `tools/`, `inspection-tool.patch`: preserved identities,
  executable and analysis sources. Initial ranking/history remain preserved;
  v2 corrects undefined-normal angle reporting.

Python histories validate selected-cell occupancy and first-point positions;
their coordinate arithmetic is not a bitwise reimplementation of C++ insertion.
The native hash gates are authoritative for the actual cell statistics. The
highlighted cells were born long after startup, so seed-sort ambiguity does not
affect their historical membership. No map-quality improvement is claimed.

An independent Codex reviewer reproduced the key statistics and confirmed this
bounded interpretation. Its undefined-normal correction is included in v2.

The next design target is **cell eligibility and correspondence consistency**,
including how to retain legitimate weak/open geometry without allowing sparse
or incompatible matches to dominate. A hard residual cutoff or occupancy count
alone is not yet a validated solution. No candidate change or new replay has
been authorized by this evidence report. Charlie remains 3/19; this evidence
informs the still-pending interface/design decision in task1.4.
