# Design

## Context

- **Inside `optimize_lidar_odometry`** (`src/domain/scan_to_map.cpp`), each Levenberg-Marquardt iteration adds one point-to-distribution term per matched point. The terms enter the matched node's 6×6 block (Tait-Bryan parameters px, py, pz, om, fi, ka) and right-hand side.
- **With `degenerate_projection_ratio_full > 0`** (shipped: 0.1 / 0.2) the point terms are already summed per node into `ndt_blocks` / `ndt_rhs` first, in point order, deterministically.
  - The window's Σ n·nᵀ (indoor grid only) is then decomposed, and each node block is projected with M = diag(P, I) before it enters the banded normal matrix.
  - Rotations are never touched.
- **A window has about 360 nodes** tied together by a tight motion model, with a 1e-4 m relative translation prior. What the LiDAR can or cannot fix is therefore the window's common motion, not single nodes.
- **The cells' normals** are fixed from their first three points by default (`use_exact_ndt_bucket_statistics = false`). Exact statistics drifted more on recording 44 in the previous change.

## Goals / Non-Goals

**Goals:**
- Detect, per window and per direction, whether the NDT information really constrains the window's common motion.
  - Translation and rotation separately.
  - Robust to the noise in the cells: a direction counts as constrained only if its signal clearly exceeds what noise alone would give.
- Attenuate smoothly: no hard switch that rounding can flip.
- Make the result as independent as possible of the build: fast and precise within a few centimetres on the full 44.
- Bit-identical behaviour with the new setting off.

**Non-Goals:**
- IMU velocity and bias states, and loop-closure gating: the next two changes.
- Changing the cell statistics or the NDT term itself (plane regularization was measured as harmful at 0.3 m).
- Per-node degeneracy: the window's rigid motion is the unit.

## Decisions

### 1. The information of the window's rigid motion
A rigid increment (δt, δθ) of the whole window moves node k by δt + δθ × (p_k − c) and rotates it by δθ, with c the window's mean node position. The rigid information is H_w = Σ_k G_kᵀ B_k G_k, where:
- B_k is the node's summed NDT block;
- G_k maps (δt, δθ) to the node's six parameters, including the conversion of a small world-frame rotation into Tait-Bryan increments at the node's attitude.

Using c, not the world origin, keeps the analysis independent of the frame. "Degenerate in Whose Frame?" (arXiv:2608.15532, cited in `research.md`) shows that axis labels and corrections change with the origin.

Translation and rotation are separated by Schur complements:
- H_t = H_tt − H_tr H_rr⁻¹ H_rt is what translation is constrained by when rotation is free;
- H_r likewise for rotation.

Each is eigendecomposed. Alternative rejected: one 6×6 eigen-decomposition, which mixes m and rad and depends on scale (X-ICP's argument against it).

### 2. The noise-aware test (Hatleskog & Alexis 2025, adapted to NDT)
- **The principle:** the estimated information is the true information plus a noise term, E[Ĥ] = H + Σ. For each eigen-direction v with estimated information λ = vᵀĤv, estimate the part noise would produce, λ_noise. Then p = P(λ_true > s·λ_noise), with s = 10 as in the paper.
- **Adapting it:** the paper works with point-to-plane terms and point and normal noise. For NDT point-to-distribution terms the corresponding noise is the point noise and the uncertainty of the cell's mean and covariance (the cell's point count and spread).
- **Task 1 derives λ_noise for our term from the paper,** and decides the constants from the sensor data sheet and the cell statistics.
- **Fallback if that derivation is not sound for NDT:** the X-ICP count test. Count, per direction, the matched points whose cell normal aligns with it within a fixed angle; only planar cells count. Then map the counts to p with a smooth ramp between two counts.

Either way p ∈ [0, 1] is continuous.

### 3. Soft attenuation
- Translation:
  - P_t = Σ p_k v_k v_kᵀ over H_t's eigenvectors;
  - each node block is transformed as today, with P_t in the translation part: M = diag(P_t, ·) applied in the frame of the rigid parameters, mapped back to node parameters through G_k.
- Rotation: the same with P_r.
- This replaces the ratio ramp when the new setting is on. `degenerate_projection_ratio*` stays as a separate, older option, so the two can be compared. Only one of them may be on at a time; the settings loader warns otherwise.

### 4. Diagnostics before behaviour
Task 1 only computes and logs (`log_window_diagnostics`): per window the three translation and three rotation p values, their directions, and today's ratio. Results are unchanged. That data decides s and the constants before anything is switched on.

## Risks / Trade-offs

- **The NDT noise model may be off by a large factor.** The decision is soft, and s is tuned on the logged data. The count-based fallback needs no noise model.
- **Rotation attenuation means the gyro and the motion model alone decide that rotation.** Over a long blind stretch, gyro bias then accumulates. The online bias estimate keeps running, but its observability drops: a known trade-off, and the reason the next change brings in IMU states.
- **Cost:** a 6×6 accumulation per node and two 3×3 eigendecompositions per iteration, negligible next to the point loops. Checked with `log_stage_timing`.
- **One recording to tune on:** the cave test the user plans after the three changes is the real check. Settings must not be fitted closely to 44, so a parameter that only helps 44 is rejected.
