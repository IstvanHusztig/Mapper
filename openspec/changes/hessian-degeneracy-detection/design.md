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

## Derivation (task 1.1)

Source: Hatleskog & Alexis, RA-L 2025, arXiv:2410.10784, Eqs. 7, 8, 13 to 15, 20, 23, 24 and 34. Read from the arXiv HTML on 2026-10-08.

**From NDT to point-to-plane.**
- A matched point p (world frame, after the current pose) and its cell, with mean μ and covariance C = Σ_j λ_j e_j e_jᵀ (λ0 ≤ λ1 ≤ λ2), give the term (p − μ)ᵀ C⁻¹ (p − μ).
- It is a sum of three point-to-plane terms, one per eigenvector e_j, with weight 1/λ_j.
- The term along the normal e0 is the geometric constraint. The two in-plane terms are the false stiffness of a flat cell: the cell's own spread, not a surface.
- The detector therefore uses only the normal term of planar cells: λ1 ≥ 3·λ0, the rule the exact cell statistics use for normals, and at least 3 points. All weights are 1, as in the paper. The test then depends only on the geometry and the noise, not on the cells' arbitrary thickness.

**Rigid motion of the window.** For a rigid increment (δt, δθ) about the window's mean node position c, a point's residual changes by vᵀ(δt, δθ), with

    v = [ n ; (p − c) × n ]        (translation part first, rotation part second; n = e0)

so that H = Σ_i v_i v_iᵀ (6×6).

**Noise** (paper, Eqs. 7 and 8):
- point noise ε ~ N(0, σp² I);
- normal noise η, a small rotation of the normal, n̂ = n + [n]× η, with (Eq. 34) Σ_η = (σp² / N) · ( e1 e1ᵀ / λ2 + e2 e2ᵀ / λ1 ).

Tilting the normal about e1 moves it towards e2, whose spread is λ2; N is the cell's point count. Then

    δv_t = [n]× η
    δv_r = ε × n + (p − c) × δn = −[n]× ε + [p − c]× [n]× η

so B_i = [[0, [n]×], [−[n]×, [p − c]× [n]×]] acting on (ε, η), and Σ_i = B_i diag(σp² I, Σ_η) B_iᵀ (Eq. 14). Summed: Σ = Σ_i Σ_i, and E[Ĥ] = H + Σ (Eq. 15).

**Separate translation and rotation.**
- For a translation direction u (unit, 3D), the full-space direction with the rotation re-optimized is u6 = [u ; −H_rr⁻¹ H_rt u]. Then u6ᵀ Ĥ u6 = uᵀ Ĥ_t u, with Ĥ_t = Ĥ_tt − Ĥ_tr Ĥ_rr⁻¹ Ĥ_rt.
- The directions are the eigenvectors of Ĥ_t.
- Rotation is the same with the roles swapped. Its unit is the radian about axes through c.

**The test** (Eq. 20), per direction:
- μ = u6ᵀ Σ u6;
- σ² = Σ_i [ 2 (u6ᵀ Σ_i u6)² + 4 (u6ᵀ Σ_i u6)(u6ᵀ v_i)² ];
- the signal a = u6ᵀ Ĥ u6 − μ (the paper's H ≈ Ĥ − Σ̂);
- p = P(a ≥ s·ξ), ξ ~ N(μ, σ²), so p = Φ((a / s − μ) / σ).
- s = 10 as in the paper. p is set to exactly 1 when it rounds to 1 (above 1 − 1e-12), so a well-constrained window stays bit-identical.

**Constants.**
- σp = `degenerate_point_sigma_m`, default 0.02 m. This is the range noise of the Livox Mid-360 class, which Mandeye uses. The paper used 1 to 1.5 cm for Velodyne and Ouster. To be confirmed by the user for their sensor.
- s = `degenerate_snr_factor`, default 10.

**When it is computed.**
- Once per window, in the first Levenberg-Marquardt iteration, and held for the rest of the window. LIO-SAM likewise decides degeneracy in its first iteration (`iterCount == 0`).
- This avoids the verdict flipping between iterations, and keeps the cost to one pass over the points per window.
- Without a workspace (the unit tests) it is computed in every call.

**Applying it** (Eqs. 23 and 24 adapted to the normal equations).
- Translation: P_t = Σ p_k u_k u_kᵀ over Ĥ_t's eigenvectors.
- Rotation: P_r likewise, over Ĥ_r's.
- Each node's summed NDT block B and right-hand side g become Mᵀ B M and Mᵀ g, with M = diag(P_t, E⁻¹ P_r E). E maps the node's Tait-Bryan increments to a world rotation vector, taken numerically at the node's attitude, so it doesn't depend on the angle convention.
- With every p = 1 nothing is multiplied.
- The coupling of a rotation about c with node positions is left to the translation part. This approximation is exact for directions through the node and small for a window about 0.4 m long.

**Cell data.** `refresh_cell_inverse` also stores the cell's eigenvalues and eigenvectors, only when the detector or its diagnostics are on, so the points don't decompose a cell each time.

## Risks / Trade-offs

- **The NDT noise model may be off by a large factor.** The decision is soft, and s is tuned on the logged data. The count-based fallback needs no noise model.
- **Rotation attenuation means the gyro and the motion model alone decide that rotation.** Over a long blind stretch, gyro bias then accumulates. The online bias estimate keeps running, but its observability drops: a known trade-off, and the reason the next change brings in IMU states.
- **Cost:** a 6×6 accumulation per node and two 3×3 eigendecompositions per iteration, negligible next to the point loops. Checked with `log_stage_timing`.
- **One recording to tune on:** the cave test the user plans after the three changes is the real check. Settings must not be fitted closely to 44, so a parameter that only helps 44 is rejected.
