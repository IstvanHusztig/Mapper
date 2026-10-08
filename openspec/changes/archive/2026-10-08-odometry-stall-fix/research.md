# Research: degeneracy in LiDAR(-inertial) odometry

A literature survey made for this change on 2026-10-08 by a research agent. It is a summary, not a review. The agent read the X-ICP, ICA, Hatleskog, LP-ICP, Wildcat, LAMP 2.0, DARE-SLAM and SubT-survey texts, and LIO-SAM's source. It could not extract the text of Zhang 2016 and Zlot & Bosse 2014; details from those two are from memory. Points marked *inference* are the agent's own reasoning, not reported results. The citations were not checked one by one.

## Degeneracy-aware optimization

- **Solution remapping**, Zhang, Kaess, Singh, *On Degeneracy of Optimization-based State Estimation Problems*, ICRA 2016.
  - Method:
    - Eigendecompose JᵀJ and find the eigenvalues below a threshold.
    - Set those directions to zero in the update: x = V⁻¹V′·x_LM. They stay at the prediction.
  - LIO-SAM uses it with an absolute eigenvalue threshold of 100, decided in the first iteration only.
  - Pitfalls: the eigenvalue scale depends on the number of points and on units, and the hard switch flips. dMapper's gate shows the same flipping behaviour.
- **X-ICP**, Tuna et al., T-RO 2024, arXiv:2211.16335.
  - Per eigen-direction, with rotation and translation analysed separately, it counts correspondences whose normals align with the direction.
  - Each direction gets a class: full, partial or none.
  - "None" and "partial" become equality constraints in a KKT solve.
  - The paper argues against eigenvalue or condition-number thresholds (scale-dependent, binary).
- **Informed, Constrained, Aligned**, Tuna et al., arXiv:2408.11809.
  - Compares equality and inequality constraints, truncated SVD and Tikhonov regularization.
  - Soft (inequality) constraints did best in complex scenes.
  - Rotation and translation must be treated separately.
  - Thresholds stay heuristic.
- **Probabilistic degeneracy detection**, Hatleskog & Alexis, RA-L 2025, arXiv:2410.10784.
  - Point and normal noise add *spurious information* along degenerate directions: E[Ĥ] = H + Σ.
  - For each direction it computes the probability p that the signal exceeds 10× the noise.
  - It attenuates softly in the pseudo-inverse: λ⁻¹ → p·λ⁻¹.
  - Mine tunnel: APE 0.04 m, against 0.57 m with Zhang's method.
  - 500 m road tunnel: length correct, where Zhang's method undershot by 110 m.
  - **Closest to dMapper's symptom.**
- Others:
  - LP-ICP, arXiv:2501.02580: point-to-line terms, soft or hard constraints.
  - DCReg: Schur complement to decouple rotation and translation.
  - LF-GICP, arXiv:2608.19522: voxel-normal localizability field with a temporal median gate.
  - *Degenerate in Whose Frame?*, arXiv:2608.15532: per-axis labels change with the body-frame origin, so use eigen-directions and frame-invariant measures.

## NDT-specific cause (inference)

- In a planar 0.15 m cell, the point-to-distribution term also penalises in-plane offsets. The in-plane sigma is about cell size / √12, roughly 4 cm. That is false stiffness along a passage.
- Partly filled cells at the map frontier have means shifted backwards, which drags the window back: the symptom seen at windows 303 to 311 of the 44.
- Remedy: Generalized-ICP-style plane regularization (Segal, Haehnel, Thrun, RSS 2009).
  - Replace the cell covariance eigenvalues by (1, 1, ε) or inflate the in-plane variance.
  - The residual then becomes effectively point-to-plane.
  - This removes the false along-axis information instead of fighting it with priors.

## What carries motion along a blind axis

- LiDAR-inertial systems (LIO-SAM, FAST-LIO2, CT-ICP) fuse the IMU all the time, with velocity and bias states and datasheet noise. None of them gates the IMU on and off.
- Super Odometry (Zhao et al., IROS 2021) is IMU-centric.
- **Wildcat**, Ramezani et al., arXiv:2205.12595 (CSIRO; same family as Zebedee, which Zlot & Bosse used for handheld cave mapping, J. Cave Karst Stud. 76(3), 2014):
  - Sliding-window continuous-time odometry with IMU residuals always active.
  - Planar surfels only, with a Cauchy kernel.
  - Results: 0.34 to 0.42 m in tunnels, against 0.92 to 1.69 m for LIO-SAM; mean point error 3 cm at the SubT Final.
- SubT teams (Ebadi et al., arXiv:2208.01787):
  - CERBERUS skips LiDAR on low Hessian eigenvalues.
  - CTU reached under 1 m over 600 m in Bull Rock Cave.
  - The survey warns of heavy per-dataset tuning.
- *Inference:*
  - A consumer accelerometer bias of 0.02 m/s² gives 1 m of error after 10 s, so the IMU alone bridges only seconds.
  - The bias along a blind axis is unobservable while it is blind, so its update should be frozen there.
  - The metres of drift with the accelerometer term on in 28 % of windows more likely come from a poorly weighted acceleration-offset estimate than from the term itself.

## Detecting degeneracy robustly

- Use per-direction measures, rotation and translation separately.
- Use a noise-aware signal-to-noise test (Hatleskog) or alignment counts (X-ICP) instead of raw eigenvalues or one min/max ratio.
- DARE-SLAM (Ebadi et al., arXiv:2102.05117):
  - Degenerate when log κ ≥ 2 (ROC AUC 0.887 on 254 labelled scans).
  - Mine drift went from 6 % to under 1 % per 300 m.
- Temporal smoothing: median gate over a few windows. Hysteresis is standard engineering, not a published constant.
- Noisy normals create spurious information. Estimate normals from enough points (running cell statistics), and down-weight or ignore low-planarity cells.

## Loop closure robustness

- Check closures before optimizing:
  - χ² / Mahalanobis gating against the accumulated odometry covariance;
  - pairwise consistency: PCM (Mangelson et al., ICRA 2018) or ICM (LAMP 2.0, arXiv:2205.13135).
- Robust back-end options:
  - GNC (Yang et al., RA-L 2020, arXiv:1909.08605): over 80 % outlier closures rejected in LAMP 2.0;
  - switchable constraints (Sünderhauf & Protzel 2012);
  - dynamic covariance scaling (Agarwal et al. 2013).
- Degeneracy-aware closures:
  - closure factors only in well-constrained directions (Hinduja, Ho, Kaess, IROS 2019);
  - no closure candidates in degenerate places.

## Recommendations for dMapper (agent's ranking)

1. **Fix the NDT terms in planar cells:** in-plane variance inflated, or point-to-plane. High benefit, medium risk: numbers change everywhere, and rooms must keep their accuracy.
2. **Replace the hard accelerometer gate with a continuous weight.**
   - Example: w = clamp((0.35 − r) / (0.35 − 0.15), 0, 1), smoothed by a median over 5 windows.
   - Or keep the term always on with datasheet noise and scale only its weight.
   - Freeze the acceleration offset and bias update along degenerate directions.
   - High benefit, low to medium risk.
3. **Measure degeneracy on the actual cost Hessian** reduced to the window's rigid 6 DoF, rotation and translation separately, with signal-to-noise or count-based thresholds. Diagnostic first.
4. **Soft attenuation along degenerate directions:** Tikhonov toward the prediction, or Hatleskog weighting. Medium risk: it interacts with the 1e-4 m motion prior.
5. **Robust loop closure:**
   - χ² gate against odometry drift (σ ≈ 1 to 2 % of distance);
   - Cauchy, DCS or GNC kernel;
   - partial closures;
   - no candidates from degenerate windows.

## Measuring drift without ground truth

- **Loop misclosure before correction, as a percentage of distance:** run with loop closure off and measure the gap. Reference points:
  - MARBLE: 0.31 m over 2.2 km;
  - DARE: about 1 % per 300 m.
- **Revisit consistency** (ghost walls): cloud-to-cloud or M3C2 distances between passes.
- **No-reference map metrics:**
  - mean map entropy (Razlaw et al., ECMR 2015);
  - mean plane variance (Kornilova & Ferrer, arXiv:2106.11351).
- **Sensitivity:** perturbed starts, voxel sizes, thresholds and build flags. A large spread marks a weak direction (the fast and precise builds are one such test).
- **One passage measured by tape or total station** as a cheap reference.
- **A synthetic corridor unit test** where "no stall along the axis" is a `should_fail` case that must flip.
