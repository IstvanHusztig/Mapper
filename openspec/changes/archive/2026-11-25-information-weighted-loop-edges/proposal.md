# Proposal

## Why

Every loop edge now ties all six degrees of freedom with fixed sigmas (5 cm, 0.3°), or, as a cross-floor point edge (X6), only the horizontal position of the shared region's centre. Neither matches what a registration actually determines.

- **Too much:** a round shaft fixes neither the height nor the turn about its axis. Imposing those swung the gallery (findings 1.3 of `floor-alignment-diagnosis`).
- **Too little:** a point edge carries no tilt and no yaw at all, so what the odometry got wrong there stays.
  - The lowest floor of `44_s2` stays 13–18 cm off at the lift shaft and 24 cm at the railing.
  - Floor −1 stays tilted about 1.0–1.1° about y against floor 0 on all three perturbed inputs.

The registration itself knows which directions its geometry pins down: its NDT normal matrix at the converged pose. This change weights each edge by that information.

## What Changes

- **The registration returns its information.** The cross-floor (and optionally the same-floor) registration returns the NDT normal matrix of its final iteration, without the damping term.
- **That information becomes the edge's information matrix.** It is mapped into the edge's relative-pose residual through the numeric Jacobian of the measurement with respect to the registration's pose. It is then scaled so that its best-determined translation direction gets the edge sigma `loop_closure_edge_sigma_m`; the anisotropy and the rotation-to-translation ratio come from the geometry.
- **Cross-floor edges carry no height.** The world-vertical move of the shared region is removed from the edge's information, by marginalizing it as a free parameter (Schur complement). This replaces `loop_closure_cross_floor_keep_height` for these edges.
- **The pose graph takes full 6×6 information matrices.** It keeps the angle wrap and the two-stage Cauchy kernel, with the kernel on the edge's Mahalanobis residual.
- **The drop rule uses the same residual.** An information-weighted edge is dropped by its Mahalanobis residual, expressed in edge sigmas of `loop_closure_edge_sigma_m`, against `loop_closure_max_residual_m`.
- **Each edge is logged with its sigmas.** Every accepted information-weighted edge logs its principal sigmas (translation in cm, rotation in degrees), so we can see that a shaft leaves the height and the axis turn free.
- **New settings in `[loop_closure]`, off by default:** `loop_closure_cross_floor_information_edges` and `loop_closure_information_edges` (same-floor edges). With both off the result is unchanged, bit for bit.
- **Clean-up after the evaluation, with the user's decision:** if the information edges win, the mechanisms they supersede go.
  - `loop_closure_cross_floor_point_edges` and `loop_closure_cross_floor_keep_height` are removed, with their code, tests and documentation (no dead code).
  - `loop_closure_cross_floor_rotation_sigma_deg` goes too, if nothing uses it any more.

## Capabilities

### New Capabilities

(none)

### Modified Capabilities

- `loop-closure`: loop edges can be weighted by their registration's information, with the height of cross-floor edges left free. The drop rule and the robust kernel for those edges work on their Mahalanobis residual.

## Impact

- **Code:**
  - `src/application/loop_closure.{h,cpp}`: `ndt_refine` returns its normal matrix; `LoopClosureEdge` gets an information matrix; the information transform and marginalization; a full-information relative-pose edge in `optimize_loop_pose_graph`; residuals and the drop rule.
  - `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h`: two settings; the golden TOML files are regenerated in their own commit.
- **Tests:** `tests/loop_closure_test.cpp` gets new cases (full-information edge against the diagonal edge; a free direction stays free; the marginalized height).
- **Evaluation:** reprocessing from the odometry snapshots of `44_s0/s1/s2` and `45f`, with `tools/evaluate_floors.py` and the job-folder checks:
  - the slab tilt;
  - the lift-shaft walls;
  - a 2-D ICP between floors.
- **Docs:**
  - `docs/index.html` and `docs/hu/index.html`: loop-closure parameters;
  - `CLAUDE.md`, if a setting ships;
  - `floor-alignment-diagnosis` findings.
- **No change to the odometry.** Snapshots stay valid: the new settings are in `[loop_closure]`.
