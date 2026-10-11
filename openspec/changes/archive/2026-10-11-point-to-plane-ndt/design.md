# Design

## Context

- **The NDT term today:** `optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`) builds each point term with `point_to_point_source_to_target_tait_bryan_wc_AtPA/AtPB_simplified` and the cell's stored inverse covariance `infm` (`refresh_cell_inverse`, `src/domain/ndt_map.h`), indoor grid 0.15 m and outdoor grid. With the default ("previous") statistics a cell's mean is its first point, the normal is fixed from its first 3 points, and the covariance keeps updating from points that pass the view-point test.
- **The in-plane variance option** (`ndt_plane_in_plane_sigma_m`): planar cells (smallest eigenvalue < 1/3 of the next) get in-plane variances of at least σ² before inversion. With σ = 0.3 m the in-plane weight is about 11 m⁻², against 10⁴–10⁵ m⁻² along the normal. That is nearly point-to-plane already, and it was rejected on synchronized data (4.1).
- **The degeneracy projection** uses the matched normals' information Σ n nᵀ, which doesn't depend on the in-plane part.

## Goals / Non-Goals

**Goals:**
- A clean go/no-go answer to whether a pure normal term, unlike the in-plane floor, keeps 44 stable and sharpens it.
- If it does: lower sensitivity to the first-point mean (the `/fp:fast` build and the 1–2 ms perturbations closer together).

**Non-Goals:**
- Changing the cell statistics (exact, settling): rejected, not reopened here.
- Point-to-line terms for edges and poles.
- Removing the degeneracy projection (re-checked in stage 2 only).

## Decisions

1. **Same code path, rank-1 information.** For a planar cell, the term uses `infm = n nᵀ / σₙ²` in the existing point-to-point Jacobian functions. The residual's in-plane components are then multiplied by zero, which is exactly point-to-plane, and the code stays one path.
   - The stored inverse gets a sibling `plane_information`, computed in `refresh_cell_inverse`, so the lookup cache stays one place.
   - Alternative: a dedicated point-to-plane Jacobian. Slightly faster, but a second code path to keep bit-for-bit thread-deterministic.
2. **Normal from the cell's current covariance** (smallest eigenvector), not the legacy first-3-point normal. It has more points, so it is stabler.
   - **The plane passes through the stored mean** (the first point). Its offset error is the sensor noise along the normal (about 1–2 cm on the L2), not the in-plane spread.
   - Alternative: the plane through the running mean. That brings back the drift-following of exact statistics along the normal, so it isn't chosen.
3. **σₙ² = max(smallest eigenvalue, (0.01 m)²)**, so a cell of 3 nearly collinear-plane points can't get an unbounded weight. The floor is the L2's range noise order and is a constant here, not a setting, unless the measurement needs it.
4. **Planarity test** as in the in-plane option: λ₁ ≥ 3 λ₀, plus λ₀ below the floor's scale × 4, so curved or rough cells stay point-to-distribution. Round railings are curved at 0.15 m scale and stay point-to-distribution.
5. **The go/no-go measurement first, before docs.**
   - Evaluation set: the three perturbed 44 inputs, `45f_p`, and the `/fp:fast` build on 44, each a full odometry run with the shipped settings (cross-floor loops on) plus point-to-plane and the window diagnostics. (The LM step check of synchronized-retune candidate 2 is only on its branch, so it is not a variant any more.)
   - Metrics: loops, largest correction, route length (runaways), `tools/evaluate_floors.py` (stairwell offsets, storeys, gallery façade, railing spot), `tools/floor_shape_check.py`, the builds' distance (`trajectory_seams.py`), and the window diagnostics of `44_s1` at 68–73 s.
   - **Pass, all of:**
     - no runaway and no lost loops on any input (accepted loops within 10 % of the reference);
     - **the `44_s1` stall gone:** at 68.5–72.5 s the s1 odometry moves within 0.3 m of s0's (today 1.69 m, findings 2.2), and s1's gallery façade is within 20 cm of s0's (today −56 against +34 cm);
     - either the builds closer together or revisit offsets better on at least three of four inputs.

     Otherwise **fail**: the code is removed with `ndt_plane_in_plane_sigma_m`.
   - The `ScanToMapCost` of candidate 2 doesn't exist on master; nothing else evaluates the NDT cost, so only the two point lambdas change.

## Risks / Trade-offs

- [The same failure as the in-plane sigma: fewer loops, a runaway] → that's what the go/no-go gate measures. No further work before it.
- [Without in-plane pull, windows slide along walls where the IMU is weak (blind passages)] → the accelerometer term (IMU always on) and the degeneracy projection carry those directions. Stage 2 re-checks the projection thresholds.
- [The first point's normal-direction noise biases the plane] → about 1–2 cm, comparable with today. A cell's later points can't move it, by design (decision 2).

## Migration Plan

Off by default. Stage 2 switches it on in `settings/lidar-vertical.toml` with new baselines in their own commit. On fail, the code and `ndt_plane_in_plane_sigma_m` are removed in one commit, and the change is archived as rejected with its numbers.
