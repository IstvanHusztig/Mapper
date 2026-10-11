# Design

## Context

- **The registration.** `run_loop_closure` registers a segment `j` onto an earlier segment `i` (with its neighbours) in world coordinates.
  - First `coarse_search` runs, then `ndt_refine`: a Gauss-Newton point-to-distribution alignment over the Tait-Bryan parameters of the correction `T`, at 0.5 m and then 0.25 m NDT cells.
  - Its normal matrix `AtPA` (6×6) is built in every iteration and then discarded. Before each solve a damping of `1e-3·trace/6` is added.
- **The pose graph.** `optimize_loop_pose_graph` puts relative-pose edges between segment poses.
  - Each edge's residual δ (6) is `relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1`: translation in the frame of pose `i`, then the three angle differences, wrapped.
  - Edges are weighted only diagonally (`w_t`, `w_r`) through the generated `…_AtPA/AtPB_simplified`.
  - The generator defines `AtPA = Jᵀ P J` with `J = relative_pose_obs_eq_tait_bryan_wc_case1_jacobian` (6×12, available in `3rdparty/observation_equations/.../relative_pose_tait_bryan_wc_jacobian.h`).
  - Point edges (X6) use a numeric Jacobian.
- **The settings.** Fixed sigmas `loop_closure_edge_sigma_m` = 0.05 m and `_deg` = 0.3°. Cross-floor edges get `loop_closure_cross_floor_rotation_sigma_deg` instead.
- **What we measured** (floor-alignment-diagnosis 1.3/1.4 and 2026-10-11):
  - full 6-DoF cross-floor edges at 0.3° swing the gallery;
  - point edges tie the floors horizontally, but leave the lowest floor of `44_s2` 13–18 cm off at the lift shaft and a tilt of about 1° about y between floor −1 and floor 0 (all inputs, shipped and X6 alike);
  - over whole floors, the relative yaw between floors is within 0.7° (2-D ICP).

## Goals / Non-Goals

**Goals:**
- One edge model that ties exactly what the shared geometry determines: horizontal position and tilt through a shaft; the turn too where the region has walls in two directions.
- No hand-made per-case switches: the cylinder case falls out of the information, not from a rule.
- The same strength as today in the best-determined direction, so the balance against the odometry edges stays.

**Non-Goals:**
- No change to the registration's result (`T`), the candidate search, the acceptance tests or the odometry.
- No absolute statistical calibration of the NDT information (see D4).
- The gallery drift during its pass (findings 2.1) is not addressed: that is odometry.

## Decisions

### D1. The registration's information: the undamped NDT normal matrix of the final iteration

`ndt_refine` returns, next to `T`, the `AtPA` of its last iteration at 0.25 m (before the damping), denoted `H` (over the six parameters `ε` of `T`).
- This is the matrix the solver itself trusted. Its null space is what the geometry leaves undetermined: for a vertical cylinder the vertical translation and the yaw about the axis; for a single plane every direction in the plane.
- **Alternatives:**
  - The `information_ratio` normal sum (3×3, translation only): it has no rotation and no coupling, so rejected.
  - A covariance from the residuals (Censi's method): more work, and the same shape up to scale. Not worth it, since the scale is replaced anyway (D4).

### D2. Into the edge's residual: Ω_m = G⁻ᵀ H G⁻¹

The edge's measurement is `m(T) = relative_measurement(P_i, T·P_j)` (the six relative-pose parameters the pose graph compares). Then:
- `G = ∂m/∂ε` at the converged `T`, by central differences (h = 1e-6; one call per parameter and sign, so twelve per edge, cheap).
- The registration's information becomes the measurement's information `Ω_m = G⁻ᵀ H G⁻¹`, since `G` is a local diffeomorphism between two 6-parameter descriptions of the same rigid motion.
- An ill-conditioned `G` (Tait-Bryan near ±90° pitch, impossible for floor segments) → the edge falls back to the fixed sigmas and is logged.
- **Alternative:** analytic adjoints. That needs the Tait-Bryan ↔ rotation-vector Jacobians, which are more code and error-prone; the numeric G is checked by a unit test instead.

### D3. A cross-floor edge carries no height: marginalize the world-vertical shift

`u = G·e_z` is how the measurement changes when `T` shifts the source vertically in world coordinates (`T`'s translation is applied in world frame, so this is the `pz` parameter). The height is treated as a free nuisance parameter and eliminated:

`Ω' = Ω_m − Ω_m u (uᵀ Ω_m u)⁻¹ uᵀ Ω_m` (skipped if `uᵀ Ω_m u` ≈ 0: already free).

- After this, no value of the measurement's height (which, between floors, is the registration snapping to the wrong storey or to the similar stairs) can pull the graph.
- **Alternatives:**
  - Shifting the measurement's height back (keep-height, X5): this still ties the height to the odometry's value, with a fixed weight, and so pulls when the odometry's storey is off.
  - Point edges: they also drop the tilt and the turn.

### D4. Scale: the best-determined translation direction gets `loop_closure_edge_sigma_m`

`Ω = s·Ω'`, with `s = w_loop_t / λ_max(Ω'[0:3, 0:3])`.
- Only the shape of `H` is used. The absolute NDT information is overconfident by orders of magnitude (points within a cell are correlated, and the cell covariances are a few millimetres thick), so its scale would overrule the odometry edges.
- With this scaling the rotation weight follows from the geometry: for a region of radius r the rotation information is about r² times the translation information, so a fully determined turn gets about σ_t / r. For the shaft region (r ≈ 1.5 m) that is 0.05 / 1.5 rad ≈ 1.9°, six times weaker than the fixed 0.3° that swung the gallery (findings 1.3); for a lift-shaft box it is similar.
- Negative eigenvalues from rounding are clipped to 0 (symmetric eigen-decomposition, rebuild).
- **Alternative:** a scale from the point count and the residual variance. Rejected: it would need a correlation length that would have to be tuned.

### D5. The pose-graph edge with a full information matrix

A new `add_information_edge(A, b, poses, i, j, measurement, Ω·f)`:
- δ and the angle wrap as in `add_relative_pose_edge`;
- `J` from `relative_pose_obs_eq_tait_bryan_wc_case1_jacobian`;
- `A += Jᵀ Ω J`, `b −= Jᵀ Ω δ` (the generator's convention);
- cost `δᵀ Ω δ`.

The kernel's `s2` is `δᵀ Ω δ`, already in units of sigma², so the two-stage kernel stays as it is. A unit test pins the convention: with `Ω = diag(w_t I, w_r I)` the solution equals the one with `add_relative_pose_edge`.

### D6. Drop rule and log

- **Drop rule:** an information-weighted edge is dropped by `σ_t·√(δᵀ Ω δ)` (metres of the edge sigma) against `loop_closure_max_residual_m`. Free directions contribute nothing.
- **Log:**
  - the residual list shows this value (`=>` for cross-floor, as now);
  - each accepted information edge logs its principal sigmas in the **shared region's frame**: translation of the region's centre in x, y, z (cm), and rotation about the centre in roll, pitch, yaw (°), from the pseudo-inverse of Ω mapped to that parametrization (numeric Jacobian as in D2), with "free" for a direction below 1e-6 of the largest eigenvalue;
  - a shaft should read like `x 5 y 5 cm, z free, roll 1.2 pitch 1.3 yaw 9° (nearly free)`.
- These sigmas are the first evidence (task 2) before any judgement on the floors.

### D7. Two settings: cross-floor first, same-floor separately

- `loop_closure_cross_floor_information_edges` replaces, for those edges, the fixed sigmas, `loop_closure_cross_floor_rotation_sigma_deg`, keep-height and point edges. When it is on, those settings are ignored for cross-floor edges, and the log says so once.
- `loop_closure_information_edges` does the same for same-floor edges, but without D3.
- They are separate because the same-floor edges work today (floor 0's revisits close to ≤ 0.3°), while a corridor edge that is degenerate along the corridor is a possible but unmeasured cause of gallery error. So the evaluation runs each one separately.

### D8. Evaluation from the snapshots, with criteria fixed in advance

Every run reprocesses from the odometry snapshots (`raw data/44_s*_snap`, `45f_snap`; about 3–4 min each, at most two at once, never two on the same input folder).
- **I1:** X3 cross-floor settings (cross-floor on, 10°, fitness 0.3) with `loop_closure_cross_floor_information_edges`, and keep-height and point edges off.
- **I2:** I1 plus `loop_closure_information_edges`.

**Adoption criteria for I1 against X6** (measured with `tools/evaluate_floors.py`, `slabs.py`, `lift_walls2.py`, `icp_floors_t.py`):
1. Floor −2 vs −1 at the stairwell on `44_s2` ≤ 10 cm (X6 24.3); no stairwell offset on s0/s1 more than 3 cm worse than X6.
2. Lowest floor at the lift shaft on `44_s2` ≤ 5 cm (X6 12–13 cm per wall).
3. Slab tilt of floor −1 against floor 0 at most half of X6's (about 1.0°) on at least two of the three inputs, and worse on none.
4. Gallery façade within 8 cm of shipped (the spread of the check); storeys within 2 cm of shipped.
5. `45f` identical to shipped (I1: it has no cross-floor pair); no `big jump` warnings; largest correction ≤ shipped + 0.2 m.

I2 is adopted only if it also improves the gallery or floor 0, with no criterion worse than I1. Criteria missed are reported as measured, not explained away.

## Risks / Trade-offs

- **[The NDT information sees a yaw in the shaft through the stair steps and landings, and imposes it]** → It is weak by D4 (≈ σ_t / r). The principal sigmas are logged per edge (D6) and checked before the floor evaluation (task 2). If a shaft edge still ties the yaw harder than about 3°, that is a finding: stop and report, don't add a rule.
- **[Tilt from the other floor's stairs: roll and pitch biased by stair geometry that differs between floors]** → The slab-tilt check (criterion 3) measures the result directly; vertical walls dominate the roll and pitch information because their lever is the storey height.
- **[The relative scale between edges distorts the balance (a huge region and a tiny one both get 5 cm in their best direction)]** → Same as today: every edge already gets 5 cm. The registration's quality is gated by fitness, as before. An absolute scale is a possible follow-up, not part of this change.
- **[The geometric rotation weights are weak against the odometry]** (measured in the unit tests) → A shaft region (r 1.5 m, 2.5 m high) gets a tilt sigma of about 3.6°, a 2 × 3 m box a turn sigma of about 2.5°, both against 0.5° per odometry edge. One edge alone corrects a tilt by only a few percent, exactly the weighted-least-squares share. On 44 about 17 cross-floor edges act together over about 10 segments, so the expected share is larger but still partial. If criterion 3 (tilt) is missed for this reason, the follow-up is a smaller `loop_closure_edge_sigma_m` for information edges (the 5 cm is conservative against a registration that is good to about 1 cm), measured and not guessed.
- **[Numerical: Schur complement and clipping make Ω only positive semi-definite]** → The pose graph still has the odometry edges and the fixed first pose, so the normal matrix stays positive definite; this is covered by a unit test with a rank-deficient Ω.
- **[Superseded mechanisms linger]** → The clean-up (task 5) removes point edges, keep-height and the cross-floor rotation sigma if I1 is adopted (user's decision), in its own commit.

## Migration Plan

- Off by default; with both settings off the result is bit-identical (unit test, plus `44_short` and full 44 against the manifests when the code lands).
- Adoption in `settings/lidar-vertical.toml` (cross-floor on, with information edges) is the user's decision after task 4. It needs new 44 baselines in their own commit, since cross-floor isn't shipped there today.
- Rollback: switch the setting off.

## Open Questions

- Whether the 0.25 m information is better than the 0.5 m one for a narrow shaft. It doesn't change the approach: both are logged in task 2 and the sharper one is kept.
