# Degeneracy detection and handling in LiDAR / LiDAR-inertial odometry (tunnels, caves, mines, shafts)

Research limits: about 15 tool calls. Several PDFs (Zhang 2016, LOCUS 2.0, X-ICP arXiv PDF) came back as binary and could not be read; X-ICP details come from the ar5iv HTML, Zhang 2016 details come only from secondary statements in other papers. Statements marked "Inference" are my own reasoning, not from a source.

## Detection: how is a direction declared degenerate, and with what thresholds?

### Takeaway
Almost every method eigendecomposes the Gauss-Newton Hessian (J^T J of the point-to-plane problem), usually with rotation and translation blocks separately, and then differs in the test: a fixed minimum-eigenvalue threshold (Zhang 2016), a per-direction count of correspondences that actually constrain it (X-ICP, fixed kappa values across sensors), or a noise-aware probability (Hatleskog/Alexis, no tuned threshold, parameters from the datasheet). Fixed eigenvalue thresholds are reported to need re-tuning per sensor/platform/FOV (24 to 150 across datasets); the correspondence-count and probabilistic tests are the ones reported to transfer without tuning.

### Cited Findings
- Zhang, Kaess, Singh, "On Degeneracy of Optimization-based State Estimation Problems", ICRA 2016: defines a closed-form degeneracy factor for general optimization problems, separates degenerate directions from well-conditioned ones, and solves only in the well-conditioned subspace (solution remapping). Demonstrated with camera and lidar 6-DOF ego-motion. — [CMU page](https://frc.ri.cmu.edu/~kaess/pub/Zhang16icra.html) (abstract level only; PDF unreadable here)
- Zhang's method is binary and uses a single minimum-eigenvalue threshold that must be fine-tuned; X-ICP's authors used a threshold of 120 in their experiments. In simulation it was "over-optimistic" in some cases and flagged the wrong directions in others. — [X-ICP, Tuna et al., T-RO 2024 (arXiv 2211.16335, ar5iv)](https://ar5iv.labs.arxiv.org/html/2211.16335)
- Tuned Zhang lambda_min values reported by Hatleskog and Alexis: 24 (Rümlang construction site, 180 deg FOV VLP-16), 48 (Seemühle mine, 360 deg), 123 (Fyllingsdalen), 150 (Seemühle, best); on a RelyOn dataset a grid search found lambda_min = 0 best, i.e. degeneracy handling was counter-productive. — [Hatleskog & Alexis, "Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization", RA-L 2024/25 (arXiv 2410.10784)](https://arxiv.org/html/2410.10784v2)
- Hinduja et al. (2019) set the eigenvalue threshold automatically from a relative condition number; X-ICP's authors found it over-pessimistic (relied almost entirely on the prior) in a combined corridor-to-open-space test. — [X-ICP](https://ar5iv.labs.arxiv.org/html/2211.16335)
- X-ICP detection (Tuna, Nubert, Nava, Khattak, Hutter; T-RO vol. 40, pp. 452-471, 2024): eigendecompose the translational and rotational Hessian blocks separately (cross terms dropped, so the result is independent of how the environment is oriented in the map frame). Each correspondence contributes a normal (translation) and a moment-normalised torque p x n (rotation), projected onto each eigenvector. Weak contributions are dropped by a per-sensor alignment cutoff (cos 80 deg = 0.174 for VLP-16, cos 60 deg = 0.5 for Ouster OS0-128), a "strong" subset needs alignment above cos 45 deg = 0.707. Summed contributions L_c (combined) and L_s (strong) classify each of the six directions: fully localizable if above kappa1 = 250, partially localizable between kappa3 = 35 and kappa2 = 180 (with L_s as alternative test), non-localizable otherwise. kappa1-3 are stated as fixed across environments and sensors; only the alignment cutoff changes with sensor. — [X-ICP ar5iv](https://ar5iv.labs.arxiv.org/html/2211.16335); [arXiv abstract](https://arxiv.org/abs/2211.16335)
- X-ICP limitations: still has user thresholds (kappa1-3, cutoff), depends on a robust prior (leg odometry degrades on rough terrain), a bad initial guess breaks both correspondence search and the unconstrained directions, partial-localizability re-sampling adds cost, point-to-plane only. — [X-ICP ar5iv](https://ar5iv.labs.arxiv.org/html/2211.16335)
- Hatleskog & Alexis (NTNU ARL): model point noise (Gaussian, covariance per point) and normal noise (small rotation) and show the noisy Hessian is biased, E[H_hat] = H + Sigma (noise inflates eigenvalues, so a clean eigenvalue test over-reports information). For a unit direction u, signal a_u = u^T H u, noise xi_u with mean u^T Sigma u and a variance sum of per-point terms. Degeneracy probability p_u = P(a_u >= s * xi_u) with s = 10 (about 10 % relative error). No hard threshold: eigenvalues are soft-inverted, lambda_k^+ = p_k / lambda_k, update = U P Lambda^-1 U^T J^T b, equal to a Tikhonov zero-prior on weak directions. Datasheet noise sigma_p = 1 cm (VLP-16); normal outliers rejected above sigma_n = 0.10 m. Runtime 17 ms median on a laptop (54 ms on a constrained board), detection about 5.4 % of runtime. — [arXiv 2410.10784](https://arxiv.org/html/2410.10784v2)
- Hatleskog & Alexis real-world result, Seemühle underground mine, 360 deg FOV, full-trajectory APE translation (m)/rotation (deg): proposed 0.04/0.48; Zhang (lambda_min = 48) 0.57/0.60; Hinduja 0.18/1.28; Lee et al. 0.78/0.79. Rümlang (FOV cut to 180 deg) is qualitative only (no ground truth); only one dataset is quantitative. Authors note Gaussian-noise assumption and dependence on datasheet noise values. — [arXiv 2410.10784](https://arxiv.org/html/2410.10784v2)
- Nubert, Walther, Khattak, Hutter, "Learning-based Localizability Estimation for Robust LiDAR Localization", IROS 2022: sparse 3D CNN (ResUNet) on the raw downsampled scan predicts localizability before registration, trained only on simulated data, tested in field experiments with two sensors; reported detection performance is on par with state-of-the-art after environment-specific threshold tuning of the classical methods (i.e. not better, but no tuning). — [arXiv 2203.05698](https://arxiv.org/abs/2203.05698v2)
- D2-LIO (2025) detects by eigendecomposing A^T A (rotation and translation separately) with no cutoff; the regularisation weight scales continuously with 1/lambda_i (see Handling). — [D2-LIO, arXiv 2508.14355](https://arxiv.org/html/2508.14355v1)
- RELEAD (2024): X-ICP-style none/partial/full labelling on the translation and rotation Hessian blocks, applied inside an ESIKF; no numeric threshold given in the text. — [RELEAD, arXiv 2402.18934](https://arxiv.org/html/2402.18934v1)
- GenZ-ICP (Lee, Lim, Han, RA-L 2025, vol. 10 pp. 152-159) avoids explicit detection: correspondences are classified planar by local PCA surface variation, planar pairs use point-to-plane, others point-to-point, mixed with a weight alpha = ratio of planar correspondences to all; claimed resilient in corridors, built on KISS-ICP. Secondary summaries only, I did not read the paper's results. — [arXiv 2411.06766](https://arxiv.org/html/2411.06766v1)
- LOCUS 2.0 (Reinke et al., RA-L 2022, CoSTAR SubT): normals-based GICP plus adaptive voxel grid; the only one of LOCUS 2.0, FAST-LIO and LINS that does not fail in the lidar-slip tunnel dataset (F); released dataset 11 h, 16 km incl. degenerate areas. Its degeneracy-section details (thresholds) could not be read. — [arXiv 2205.11784](https://arxiv.org/pdf/2205.11784); [abstract page](https://arxiv.org/abs/2205.11784v1)

### Inferences
- For an NDT scan-to-map that already builds per-window normal equations, the cheapest detection with the best transfer record is a per-direction information test on the translation block and rotation block separately, with the noise bias (E[H] = H + Sigma) taken into account. NDT cells give a covariance per cell, so a Hatleskog-style noise term is available in principle; none of the papers reviewed apply this to NDT (not verified, I found no such paper).
- Fixed eigenvalue thresholds scale with point count and scan density, so any dMapper threshold would have to be normalised (per-point or per-information-unit) or it will behave differently for L2 and Mid-360S.

### Gaps
- Zhang 2016's own equations and the exact degeneracy-factor formula, and X-ICP numeric tables, could not be read (binary PDF or truncated HTML).
- No source found that evaluates degeneracy detection specifically for NDT (D2 distance) registration.
- DARE and other 2023-2026 named variants asked for in the brief were not found in my searches (only GenZ-ICP, D2-LIO, RELEAD, ALIVE-LIO, LF-GICP surfaced; ALIVE-LIO and LF-GICP were seen only as titles/abstract snippets).

## Handling: what is done once a direction is degenerate, and what works in tunnels and caves?

### Takeaway
Options in increasing softness: drop the update along the direction (Zhang remapping, TSVD, X-ICP "none"), hard equality constraint to the prior, inequality constraint (band), Tikhonov / prior regularisation weighted by 1/lambda or by detection probability, and external-sensor aiding. In the one head-to-head field study (Tuna et al., ICA), all active methods beat plain point-to-plane in a bicycle tunnel by a similar margin, hard constraints applied before or during optimisation were the most consistent without tuning, and soft constraints could be best but only with dataset-tuned parameters. When a reliable prior exists the best results come from using it; when none does, active mitigation is needed.

### Cited Findings
- Tuna, Nubert, Pfreundschuh, Cadena, Khattak, Hutter, "Informed, Constrained, Aligned: A Field Analysis on Degeneracy-aware Point Cloud Registration in the Wild" (arXiv Aug 2024, journal version 2025): compares equality constraints, inequality constraints, TSVD, linear and non-linear Tikhonov (L-Reg, NL-Reg), Zhang remapping, robust kernels, sampling methods and prior-only, all inside ICP. Hard constraints added before or during optimisation worked better in the field than constraints applied after; soft constraints with heuristic parameters could match or beat them in complex ill-conditioned scenes; active mitigation is advantageous when no reliable external estimate exists. Equality constraints set the update along degenerate eigenvectors to zero (targets t0 = r0 = 0) via Lagrange multipliers; inequality constraints bound |L x| <= epsilon (epsilon = 0.0014 translation, epsilon/2 rotation, tuned on the Ulmberg tunnel) as a quadratic program. — [arXiv 2408.11809](https://arxiv.org/html/2408.11809v2)
- Ulmberg bicycle tunnel (one-directional degeneracy), mean relative translation error RTE / ATE in m: Ineq. Con. 0.033/1.21; NL-Reg 0.035/1.097; L-Reg 0.0358/1.099; Eq. Con. 0.039/1.10; Zhang 0.0383/1.125; TSVD 0.0388/1.11; robust-M-estimator-type "RMS" 0.09/1.62; plain point-to-plane 0.073/2.90; prior only 0.056/1.54. Plain point-to-plane and prior-only produce map duplication along the tunnel. Inequality and Tikhonov methods were tuned on this dataset (favours them). — [arXiv 2408.11809 (table 4)](https://arxiv.org/html/2408.11809v2)
- ANYmal forest (open-field degeneracy), RTE/ATE (m): NL-Reg 0.039/0.364; Ineq 0.045/0.405; Eq 0.049/0.490; Zhang 0.083/0.372; TSVD 0.068/0.542; point-to-plane 0.212/2.62. Without tuning, Eq. Con. and TSVD were best; Zhang's metrics look good but its map is rotationally misaligned (translation metrics hide it) and drifts when the payload blocks the view. NL-Reg and NL-Solver cost about 5x more per ICP (about 45-80 ms vs 9-19 ms). (The paper's text calls Zhang best in ATE but its table shows NL-Solver lowest; internal inconsistency.) — [arXiv 2408.11809 (table 5)](https://arxiv.org/html/2408.11809v2)
- Large-residual caveat: the effective condition number grows with the residual (scaled by cond(A)^2), so ICP can fail even when A is well-conditioned if the residual is large and noisy. — [arXiv 2408.11809](https://arxiv.org/html/2408.11809v2)
- X-ICP handling: "full" directions unconstrained; "none" directions forced to zero update (initial guess kept); "partial" directions get a small least-squares problem on re-sampled high-contribution correspondences whose solution becomes an equality constraint; solved with Lagrange multipliers and SVD, with LU/RIF preconditioning. Real tests: Seemühle underground mine, Rümlang construction site, Opfikon park (Leica RTC360 ground truth), ANYmal-C + VLP-16 at 5 Hz, leg odometry as prior; reported qualitatively with error maps (numeric tables not read). — [X-ICP ar5iv](https://ar5iv.labs.arxiv.org/html/2211.16335)
- Hatleskog & Alexis: smooth attenuation (soft-inverse with p_u) instead of binary drop; mine APE 0.04 m vs 0.57 m for Zhang tuned (numbers above). — [arXiv 2410.10784](https://arxiv.org/html/2410.10784v2)
- D2-LIO: weak directions regularised towards the IMU-propagated initial guess with weight 1/lambda_i, summed to a norm weighted by (A^T A)^-1; final weight W = (A^T A P_hat)^-1 with P_hat from IMU preintegration, separate rotation and translation weights. GEODE APE (m) on Tunnel-2: D2-LIO 0.10 vs FAST-LIO2 0.14, LIO-SAM 0.13, PV-LIO 0.16, VoxelMap 0.13; Tunnel-3: 0.14 vs 0.17, 0.17, 0.22, 0.19. Authors say gains are marginal in mild tunnels. Failures: Offroad-3 0.75 (FAST-LIO2 0.34, VoxelMap 0.22), Water-short 0.31 (PV-LIO 0.12), Long_Corridor_1 1.65 (FAST-LIO2 1.08); GEODE Flat Ground: every method fails; authors state LiDAR-IMU fusion alone is insufficient in severe degeneracy. — [arXiv 2508.14355](https://arxiv.org/html/2508.14355v1)
- RELEAD: in the ESIKF, adds equality constraints v_j^T(dt - t0) = 0 (translation) and v_j^T(dr - r0) = 0 (rotation) per degenerate direction and projects the EKF update onto the constraint plane (Lagrange form: dx_bar = dx_hat - G (C dx_hat - d), G = Sigma C^T (C Sigma C^T)^-1). Evaluated on S3E (ATE RMSE LIO-only 0.12/0.36/0.37 m vs FAST-LIO 1.92/0.94/0.73) and a 2942 m urban road tunnel (about 0.5 m end error with GNSS added) and a metro tunnel (qualitative). Without the graduated-non-convexity (GNC) outlier handling in the fusion back-end it was far worse (ATE 15 m vs 0.31 m). Fixed covariances do not adapt, and it is slightly less accurate than its LIO-only variant in some scenes. — [arXiv 2402.18934](https://arxiv.org/html/2402.18934v1)
- Real coal-mine tunnels: in CM-LIUW-Odometry (LiDAR-inertial-UWB-wheel), LiDAR-inertial baselines degrade at an outer/inner tunnel junction, one variant drifting 25 m along X after leaving the area, and a wheel-odometry variant accumulating up to 9.6 m error in Z. — [CM-LIUW-Odometry, arXiv 2511.01379](https://arxiv.org/pdf/2511.01379) (snippet-level reading only)
- Other aiding found in snippets only: DAMS-LIO uses a second odometry as measurement only when degeneration is detected [arXiv 2302.01703](https://arxiv.org/pdf/2302.01703v2.pdf); COIN-LIO adds LiDAR intensity images for tunnels/flat fields [arXiv 2310.01235](https://arxiv.org/html/2310.01235v3); ALIVE-LIO fuses a learned inertial velocity into the ESKF only under detected degeneracy [arXiv 2604.02706](https://arxiv.org/pdf/2604.02706); FIRE-LIVWO adds mmWave radar and wheels for long tunnels [arXiv 2609.05325](https://arxiv.org/pdf/2609.05325). I did not read these beyond abstracts.

### Inferences
- A soft prior (Hatleskog/D2-LIO style: zero or IMU-propagated prior with weight scaled by detection probability or 1/lambda) fits dMapper's existing NDT-normal-equation structure and the user's "precision before robustness-to-failure" stance better than binary dropping, because it degrades continuously and keeps partial information. The ICA field study found hard equality constraints most consistent without tuning, so the choice is between consistency (hard) and possible accuracy gain (soft, needs tuning), and soft with a datasheet-derived probability (Hatleskog) is the only variant reported as untuned and accurate (one mine dataset).
- Zhang-style remapping and TSVD discard rotation-translation cross terms and ignore residual size; the ICA evidence that Zhang can hide a rotation misalignment is a warning for a rotation-weak shaft.

### Gaps
- No peer-reviewed head-to-head in a real cave (as opposed to tunnel/mine) was found. Seemühle mine (Hatleskog, X-ICP), Ulmberg tunnel (ICA), road/metro tunnels (RELEAD), coal mine (CM-LIUW) are the real underground cases found.
- ICA Section 6 and conclusion numbers (beyond the end of the readable HTML) were not read.
- CERBERUS, Explorer, Wildcat, MARBLE and CTU-CRAS degeneracy handling were not reviewed (tool-call budget); only LOCUS 2.0 abstract-level claims.

## How long can a consumer MEMS IMU carry the degenerate direction before 10 cm drift, and how to weight it?

### Takeaway
No paper found reports a measured accelerometer-only bridging time in meters for a tunnel passage. The only quantitative statements are physics-based (Inference below). Papers that carry the degenerate direction by the IMU do it as a covariance-weighted prior (preintegration covariance) rather than with a trusted-time budget.

### Cited Findings
- D2-LIO weights the degenerate-direction regulariser by the IMU preintegration covariance (W = (A^T A P_hat)^-1, separate rotation/translation weights), so the IMU is trusted exactly as much as its propagated covariance says. — [arXiv 2508.14355](https://arxiv.org/html/2508.14355v1)
- RELEAD states IMU keeps the state constrained in degenerate scenes but used fixed covariances and noted that this does not adapt. — [arXiv 2402.18934](https://arxiv.org/html/2402.18934v1)
- Seen in snippets: ALIVE-LIO adds a learned inertial velocity into the ESKF under degeneracy because pure LIO suffers drift and oscillation in degeneracy (suggests IMU alone is not enough); no meter figure found. — [arXiv 2604.02706](https://arxiv.org/abs/2604.02706)
- D2-LIO authors: severe degeneracy (GEODE Flat Ground) breaks every LiDAR-IMU method; state observability is set by the environment. — [arXiv 2508.14355](https://arxiv.org/html/2508.14355v1)

### Inferences
(Own calculation, not from a source; assumes perfect gravity removal and perfect attitude, which are optimistic.)
- Velocity random walk only: position sigma = N * sqrt(t^3 / 3). For N = 0.001 m/s^2/sqrt(Hz): 0.018 m at 10 s, 0.095 m at 30 s (about 31 s to reach 10 cm). For N = 0.01: 0.18 m at 10 s, 0.065 m at 5 s, reaches 10 cm at about 6.7 s.
- A constant uncompensated accelerometer bias b gives 0.5 b t^2: bias 1 mg (0.0098 m/s^2) reaches 10 cm at 4.5 s; 0.1 mg at 14 s. A gravity-misalignment error of 0.1 deg is a 0.0017 m/s^2 bias equivalent, 10 cm at about 11 s. So in practice bias and attitude error, not noise density, limit bridging to roughly 5-15 s for consumer MEMS unless the bias is estimated beforehand in non-degenerate stretches, which dMapper does (acceleration offset random walk). Plan for a bridging budget of the order of seconds; for passages longer than that the position along the axis is unobservable from the sensors in use and only closes via loop closure.
- Weighting rule consistent with the literature: give the IMU prior on the degenerate direction a sigma that grows as N*sqrt(t^3/3) plus the bias term from the estimator's covariance (what D2-LIO does via preintegration covariance); keep the LiDAR weight on the well-constrained subspace unchanged.

### Gaps
- No source gives real tunnel-passage lengths bridged by MEMS IMU with resulting drift in meters; this would need a targeted search (e.g. FAST-LIO / LIO-SAM tunnel failure analyses, GEODE benchmark reports).

## Wrong local minima and multi-modal cost (repeating steps, railings, self-similar tunnels)

### Takeaway
I found little peer-reviewed work that specifically addresses multi-modal scan-to-map cost in LiDAR odometry. Related evidence: the ICA study showed that plain point-to-plane and robust kernels (Cauchy) drift or duplicate the map in degenerate tunnels; the large-residual caveat means a wrong minimum can have a well-conditioned Hessian; RELEAD found GNC essential for outlier rejection in its fusion back-end; globally optimal GNC/TEASER registration targets high outlier ratios, not LM step acceptance.

### Cited Findings
- ICA: robust kernels (Cauchy) drifted the most in the Ulmberg tunnel; Cauchy and plain point-to-plane produced duplicated maps; sampling methods (geometrically stable sampling, redundancy-minimising) beat baseline but broke down in severe degeneracy (t about 120-150 s) and missed a bench in vegetation due to normal estimation problems. — [arXiv 2408.11809](https://arxiv.org/html/2408.11809v2)
- ICA: effective conditioning degrades with residual size, so a well-conditioned Hessian does not guarantee a correct minimum. — [arXiv 2408.11809](https://arxiv.org/html/2408.11809v2)
- Hatleskog & Alexis reject points with unreliable normals (sigma_n above 0.10 m) before building the Hessian, a correspondence-level filter. — [arXiv 2410.10784](https://arxiv.org/html/2410.10784v2)
- RELEAD uses GNC in its factor-graph back-end; without it the fused result had ATE 15.16 m vs 0.31 m on S3E (applied across odometry sources, not in scan matching). — [arXiv 2402.18934](https://arxiv.org/html/2402.18934v1)
- D2-LIO's intro: ICP-style methods stay prone to local minima; global methods such as TEASER (GNC-based) handle high outlier ratios. D2-LIO itself uses a per-point outlier threshold (fixed-threshold ablation baseline 2.0 m) combining a translation term and a range-scaled rotation term. — [arXiv 2508.14355](https://arxiv.org/html/2508.14355v1)
- GenZ-ICP's mixing of point-to-point and point-to-plane is aimed at corridors; no repeating-structure evaluation found. — [arXiv 2411.06766](https://arxiv.org/html/2411.06766v1)

### Inferences
- The candidate remedies listed in the brief (cost-based LM step acceptance, viewpoint/normal rejection, unmatched-point penalties, multi-hypothesis) are standard but I found no paper that evaluates them for stairs, railings or self-similar tunnels in LiDAR odometry. Evidence that exists favours: reject unreliable normals, keep a prior-anchored regulariser, and check the final cost/ residual against the prior-only cost (ICA's residual caveat), but this is a design inference, not a reported result.

### Gaps
- No source on multi-hypothesis scan matching, penalising unmatched points, or LM step acceptance for LiDAR odometry; no source on stair or railing sliding specifically. Needs targeted searches (e.g. "stairs LiDAR odometry degeneracy legged", "symmetric environment registration ambiguity").

## Rotational degeneracy in round shafts and yaw weakness in fast turns

### Takeaway
Rotation is treated in the same framework as translation (separate rotational eigenspace, X-ICP cylindrical-room simulation, D2-LIO separate rotation weights from IMU preintegration covariance). I found no paper with real-shaft numbers or a quantified study of gyro-versus-scan weighting in fast turns.

### Cited Findings
- X-ICP evaluates rotational degeneracy in simulation in a cylindrical room and combined degeneracy in a 246 m corridor-to-open route; its rotational analysis uses torque contributions p x n, filtered by alignment cutoffs, in its own eigenspace. — [X-ICP ar5iv](https://ar5iv.labs.arxiv.org/html/2211.16335)
- Hatleskog & Alexis' Rümlang experiment with a 180 deg FOV induces rotational and translational degeneracy; qualitative only, no ground truth. — [arXiv 2410.10784](https://arxiv.org/html/2410.10784v2)
- ICA: Zhang's method produced a misaligned map (rotation error that translation metrics missed) in the forest; rotation-weighted metrics are needed to see this. — [arXiv 2408.11809](https://arxiv.org/html/2408.11809v2)
- D2-LIO uses separate rotation and translation weights W_r, W_t derived from the IMU preintegration covariance; the text's translation term appears to use W_r (probable typo, per my reading). — [arXiv 2508.14355](https://arxiv.org/html/2508.14355v1)
- RELEAD constrains rotation directions with the same projection as translation. — [arXiv 2402.18934](https://arxiv.org/html/2402.18934v1)

### Inferences
- Because the rotation block is analysed separately and torques are normalised by lever arm, a round shaft shows up as one near-zero rotational eigenvalue (about the shaft axis) and a gyro prior with its bias estimate is the natural carrier; no paper found quantifies gyro-vs-scan weighting in fast turns.

### Gaps
- Nothing found on yaw weakness in fast turns beyond the generic framework; no real round-shaft evaluation; stairwell-specific degeneracy not found.

## Real-world (cave, mine, tunnel) evaluations: which numbers exist?

### Takeaway
Real underground evidence is thin and mostly qualitative. The strongest quantitative real-world number is Hatleskog & Alexis in the Seemühle mine (0.04 m APE vs 0.57 m tuned Zhang); ICA gives RTE/ATE tables in a real bicycle tunnel (ATE about 1.1 m for the best methods vs 2.9 m plain point-to-plane); RELEAD reports about 0.5 m end error over a 2.9 km road tunnel but with GNSS aiding.

### Cited Findings
- Seemühle mine APE table — [arXiv 2410.10784](https://arxiv.org/html/2410.10784v2)
- Ulmberg tunnel and ANYmal forest tables — [arXiv 2408.11809](https://arxiv.org/html/2408.11809v2)
- X-ICP real tests in Seemühle mine, Rümlang, Opfikon — [X-ICP](https://ar5iv.labs.arxiv.org/html/2211.16335)
- LOCUS 2.0 SubT, tunnel dataset where FAST-LIO and LINS failed — [arXiv 2205.11784](https://arxiv.org/pdf/2205.11784)
- GEODE tunnels (D2-LIO) and coal mine (CM-LIUW) — [arXiv 2508.14355](https://arxiv.org/html/2508.14355v1), [arXiv 2511.01379](https://arxiv.org/pdf/2511.01379)

### Inferences
- Reported absolute accuracies in tunnels are 0.04-1.2 m with platforms moving at walking speed and ground truth of varying quality; they are not comparable to dMapper's centimetre-level targets, and none were evaluated on a handheld/hand-carried L2-type recording in a cave.

### Gaps
- No peer-reviewed real-cave quantitative evaluation of degeneracy mitigation found.
- Learned detectors (Nubert 2022) have no numbers read; ALIVE-LIO, LF-GICP (2026) only snippet-level.
