# Noise models and weighting of scan matching against IMU terms in LiDAR-inertial odometry

Method note: values below marked "Cited" were read from the cited page or file in this session. Config files were read through a summarising fetch tool, so numbers are as reported by that tool (spot-check before quoting in a publication). Items from my own background knowledge that I could not verify here are placed under Inferences or Gaps, never under Cited Findings.

## 1. Per-point noise models (FAST-LIO/FAST-LIO2, Point-LIO, VoxelMap, GICP/NDT, others; Mid-360 values)

### Takeaway
Filter-based systems of the HKU MaRS family treat each point's residual as a scalar with a fixed variance: in FAST-LIO2 it is one constant (LASER_POINT_COV = 0.001) for every point, so the update information grows linearly with the point count. VoxelMap is the principled exception: it propagates range (0.04 m) and bearing (0.1 deg) noise to the point and to the plane, giving a per-point, range-dependent residual variance.

### Cited Findings
- FAST-LIO2 models each point as contaminated by ranging and beam-direction noise n_j, and the residual noise as v_j ~ N(0, R_j) with R = diag(R_1..R_m) over all m points; no numeric range/bearing variances are given in the paper text. Kalman gain K = (H^T R^-1 H + P^-1)^-1 H^T R^-1 (information form, so information from m points adds linearly). Default uses 1 of every 4 points, spatial downsample 0.5 m, 5 nearest map neighbours for plane fit, ikd-Tree alpha_bal 0.6, alpha_del 0.5, N_max 1500 — [FAST-LIO2 (Xu, Cai, He, Zhou, Zhang 2022, arXiv 2107.06829)](https://ar5iv.labs.arxiv.org/html/2107.06829)
- FAST_LIO code: `#define LASER_POINT_COV (0.001)`, passed as the measurement noise to `kf.update_iterated_dyn_share_modified(LASER_POINT_COV, ...)`; it is a single constant for all points. Plane fit threshold `esti_plane(..., 0.1f)`; neighbour squared-distance gate 5; point validity `s = 1 - 0.9*|pd2|/sqrt(|p_body|) > 0.9` (a range-dependent residual gate, not a weight); max_iteration default 4; convergence epsilon 0.001 on 23 state entries; filter_size_surf and filter_size_map defaults 0.5 — [FAST_LIO src/laserMapping.cpp](https://raw.githubusercontent.com/hku-mars/FAST_LIO/master/src/laserMapping.cpp). A secondary source also lists LASER_POINT_COV 0.001 — [51cto blog](https://blog.51cto.com/u_13157605/6037957)
- FAST-LIVO README describes laser_point_cov as the point-to-plane residual covariance applied per point — [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO)
- FAST_LIO config for Mid-360 (mid360.yaml): lidar_type 1, scan_line 4, blind 0.5, acc_cov 0.1, gyr_cov 0.1, b_acc_cov 0.0001, b_gyr_cov 0.0001, det_range 100, extrinsic_T [-0.011, -0.02329, 0.04412], extrinsic_R identity, extrinsic_est_en false — [FAST_LIO config/mid360.yaml](https://raw.githubusercontent.com/hku-mars/FAST_LIO/master/config/mid360.yaml). Avia config: acc_cov 0.1, gyr_cov 0.1, b_acc_cov/b_gyr_cov 0.0001, blind 4, det_range 450 — [FAST_LIO config/avia.yaml](https://raw.githubusercontent.com/hku-mars/FAST_LIO/master/config/avia.yaml)
- Point-LIO avia.yaml: lidar_meas_cov 0.001, imu_meas_acc_cov 0.1, imu_meas_omg_cov 0.1, acc_cov_output 500, gyr_cov_output 1000, acc_cov_input 0.1, gyr_cov_input 0.01, b_acc_cov 0.0001, b_gyr_cov 0.0001, plane_thr 0.1, match_s 81, satu_acc 3.0, satu_gyro 35, blind 1.0, imu_time_inte 0.005 — [Point-LIO config/avia.yaml](https://raw.githubusercontent.com/hku-mars/Point-LIO/master/config/avia.yaml). Point-LIO "processes at the sampling time of each LiDAR point" (point-by-point update) — [Point-LIO repo](https://github.com/hku-mars/Point-LIO); paper: Advanced Intelligent Systems 2023, DOI 10.1002/aisy.202200459
- VoxelMap (Yuan, Xu, Liu, Hong, Zhang, "Efficient and Probabilistic Adaptive Voxel Mapping for Accurate Online LiDAR Odometry", RA-L 2022): measured point noise = bearing-direction noise (S2 tangent-plane Gaussian) plus range noise d_i ~ N(0, sigma_d); at short range the covariance is dominated by ranging noise, at long range by bearing noise. Noise is propagated to the world-frame point (including pose uncertainty), then to the plane (n, q) covariance, then to the scalar residual d_i = n^T(p_i - q) with variance w_i = J Sigma J^T (Eq. 10), used as R_i in the iterated EKF (Eq. 12). A match is accepted if the distance falls within 3 sigma of the plane; the plane with highest probability wins; otherwise the point is dropped. Plane normal covariance converges at about 50 points (points with 0.1 m^2 variance in their experiment); after that history points are discarded and only the latest 10 are kept to test for change. Experiments: max voxel size 3 m, max octree layer 3 (min voxel 0.375 m) for KITTI-style data; 0.5 m max voxel, 2 layers for the park/Avia handheld case — [arXiv 2109.07082](https://arxiv.org/abs/2109.07082) (full text via PDF)
- VoxelMap velodyne.yaml: ranging_cov 0.04, angle_cov 0.1 (degrees), max_layer 4, layer_point_size [5,5,5,5,5], voxel_size 3.0, acc_cov_scale 1.0, gyr_cov_scale 0.5 — [VoxelMap config/velodyne.yaml](https://github.com/hku-mars/VoxelMap/blob/master/config/velodyne.yaml)
- Livox MID-360: range precision (1 sigma) <= 2 cm at 10 m, <= 3 cm at 0.2 m; angular precision (1 sigma) < 0.15 deg; IMU ICM40609 — [Livox Mid-360 specs](https://www.livoxtech.com/mid-360/specs). Mid-360S third-party listings give <= 4 cm at 0.2 m — [Livox Mid-360S specs](https://www.livoxtech.com/mid-360s/specs). Unitree L2: accuracy <= 2 cm, distance resolution 4.5 mm, 360x96 deg FOV, 0.05-30 m, IMU 500 Hz reporting (RM variant listing) — [Unitree L2](https://unitree.com/L2/), [L2 manual](https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf)
- GICP: original covariance regularisation replaces eigenvalues by (1,1,eps), turning GICP into plane-to-plane; Open3D's GICP default eps = 1e-3; small_gicp example uses num_neighbors = 10 for normal/covariance estimation — [Generalized LOAM](https://arxiv.org/pdf/2210.16510), [small_gicp](https://github.com/koide3/small_gicp), [Open3D GICP header](https://www.open3d.org/docs/release/cpp_api/_generalized_i_c_p_8h_source.html)
- AKF-LIO replaces FAST-LIO2's fixed constraint covariance by a per-Gaussian-map-element covariance: R_hat = exp(-b * R_hist) * s, with s = n^T Sigma n (plane thickness), updated by a use-count-weighted running average of the post-update squared residual plus posterior state covariance projected through H — [AKF-LIO (arXiv 2503.06891, 2025)](https://ar5iv.labs.arxiv.org/html/2503.06891)

### Inferences
- FAST-LIO2 with R = 0.001 on a residual in metres is a 3 cm 1-sigma per-point residual; with a 1:4 decimated, 0.5 m voxel-filtered scan of a few thousand points the total information is about thousands times 1/0.001, which dwarfs the IMU prior. That is exactly the mechanism making a no-scale NDT weight overconfident; the systems stay usable because the weight is a global trust knob (R) tuned empirically, and the downsampling to about one point per 0.5 m voxel (cap on count) acts as the implicit correlation correction.
- VoxelMap's per-point variance (0.04 m range, 0.1 deg bearing: at 10 m the bearing term is about 1.7 cm, at 30 m about 5 cm) is the physically grounded version; its plane-uncertainty term shrinks with point count, so the NDT-cell analogue is: residual variance = sigma_point^2 + n^T Sigma_cell n, where the second term includes cell mean uncertainty (cell covariance / N), not just the cell's inverse covariance.
- For NDT the cell inverse covariance already encodes rock roughness (cell spread). Using it as the weight with no extra scale counts the same spread as noise per point; a principled scaling is a per-scan information cap (see Question 2).

### Gaps
- Mid-360 specific tuning values in FAST-LIO2 for the range/bearing noise: none exist (a single R). No Mid-360 VoxelMap config was found.
- DLIO, iG-LIO, LIO-SAM/LOAM feature residual weights, KISS-ICP, PV-LIO per-point numbers were not retrieved in this session. From background knowledge (unverified here): DLIO and KISS-ICP use no explicit per-point covariance (DLIO uses GICP with 10 neighbour covariances; KISS-ICP uses a Geman-McClure kernel with adaptive sigma threshold); LIO-SAM/LOAM use unit-weight feature residuals with a distance-based robust scale. Verify before citing.
- Exact Magnusson NDT score function parameters (outlier ratio, d1, d2 from the 2009 thesis) were not retrieved; a search only confirmed d1, d2 are regularisation factors in the score function ([Zaganidis et al.](https://discovery.ucl.ac.uk/id/eprint/10217483/1/Revisiting%20Distribution%20Based%20Registration%20Methods.pdf) and Magnusson's thesis, core.ac.uk/download/55951.pdf not read). Background knowledge: the 3D-NDT score is -d1*exp(-d2/2 * (x-mu)^T C^-1 (x-mu)) with d1,d2 set from an outlier ratio (commonly 0.55); unverified.

## 2. Information scaling and statistical consistency

### Takeaway
No surveyed system normalises by point count; they cap information by downsampling (one point per voxel), a constant R used as a trust knob, gating/robust residuals, and in recent work adaptive covariance. Closed-form ICP covariances ignore correlated errors and wrong correspondences and are orders of magnitude optimistic; the one validated fix (Brossard) adds a shared-bias term and an initialisation-propagation term.

### Cited Findings
- Censi's closed-form ICP covariance is sigma^2 * A^-1 with A = sum_k B_k^T B_k, K matched pairs; A grows with K so A^-1 shrinks. It assumes white, independent per-point noise, ignores wrong convergence and omits the correlated calibration bias and cross-covariance with the initial estimate. In their Fig. 1 example tr(A^-1) = 0.2 cm versus a bias term of trace 2.6 cm. Proposed: Q_icp = (I - J) Q_ini (I - J)^T + sigma^2 A^-1 + A^-1 B cov(b) B^T A^-1, J from an unscented transform (12 sigma points = 12 ICP runs); experiments used 5 cm bias and 5 cm white-noise sigma, 1020 pairs x 1000 initialisations. Consistency measured as normalised norm error (target 1): proposed 0.8 (translation) and 3.8 (rotation); trajectory Mahalanobis 1.0-2.3 translation, 1.2-9.8 rotation ("slightly optimistic") — [Brossard, Bonnabel, Barrau, "A New Approach to 3D ICP Covariance Estimation", RA-L 2020, arXiv 1909.05722](https://ar5iv.labs.arxiv.org/html/1909.05722)
- CELLO-3D: closed-form estimates are "orders of magnitude smaller than the sampled covariances"; average KL divergence from sampled covariance: Censi 9.19e7 (apartment), 2.06e8 (Hauptgebaeude), 7.65e7 (stairs), 2.55e6 (gazebo summer), 4.94e6 (wood autumn) against 34, 34, 34, 21, 13 for a baseline and 27, 27, 27, 20, 11.5 for CELLO-3D. Cause: point reassociation at small scales (plateaus/local minima), and sensor noise under-modeled (Censi grows slowly with noise whereas the sampled covariance increases dramatically). Learned model: kernel-weighted average of training covariances over a 4x4x4 grid descriptor (25 m x 25 m x 10 m), 5.1 million registrations; in self-similar outdoor scenes the weights converge to near uniform and it returns roughly the mean covariance — [Landry, Pomerleau, Giguere, "CELLO-3D", ICRA 2019, arXiv 1810.01470](https://ar5iv.labs.arxiv.org/html/1810.01470). Brossard's table reports CELLO-3D as pessimistic (Mahalanobis 0.1-0.3, i.e. 3-10x too large in sigma).
- AKF-LIO: with FAST-LIO2's fixed Q and R scaled 100x, 1x, 0.01x, FAST-LIO2's ATE RMSE ranged from about 0.78 m to 45.80 m on a dynamic field sequence, while the adaptive filter stayed about 0.18-0.20 m. Fixed covariance is shown to make results very sensitive; one fixed variant of the authors' own method reached 89.46 m in one setting. Adaptive update uses innovation/residual statistics with forgetting factor a (value not given) — [AKF-LIO](https://ar5iv.labs.arxiv.org/html/2503.06891)
- VoxelMap applies a 3-sigma gate on the residual variance (outlier gating/robust rejection) rather than a kernel — [arXiv 2109.07082](https://arxiv.org/abs/2109.07082)
- Equivariant-filter LIO paper states EKF-based LIO can be inconsistent (computed covariance deviates from true covariance) — [arXiv 2409.06948](https://arxiv.org/html/2409.06948v1)

### Inferences
- A solver covariance 15-30x too optimistic in sigma (225-900x in variance) is the same order as Brossard's Censi finding; the cure there is a shared-bias term, not a scale on A^-1. Practical, cheap equivalents:
  1. Effective-sample correction: scale the information by N_eff/N, where N_eff is the number of independent constraints (about one per voxel-sized patch, i.e. scan surface area divided by voxel area); 15-30x in sigma corresponds to N/N_eff of about 225-900.
  2. Normalise the residual variance with an a-posteriori variance factor: sigma0^2 = v^T W v / (n - u) (geodesy; background knowledge), then use sigma0^2 * (A^-1) as covariance. This captures only random residual excess, not correlation-driven optimism, so treat it as a floor.
  3. Window-level NIS/NEES test: chi-square of the scan-matching innovation against the IMU/motion-model prior over windows; its mean should be near the state dimension (6 or 15). Tune one global trust scale until the mean NIS is about 1 per degree of freedom. I found no LIO paper that does this explicitly (see Gaps).
- Fixed R as a trust knob is validated in practice only because every tuned dataset keeps point count roughly constant through voxel downsampling; changing sensor or point density invalidates it, which is why a count-aware scale is preferable.

### Gaps
- No paper found that calibrates a LiDAR odometry term with NIS/NEES chi-square or geodesy variance-factor / variance-component estimation; the web search returned AKF-LIO and equivariant filters only. Bar-Shalom consistency tests and variance component estimation (Helmert, Koch) are standard theory but not retrieved here.
- Robust kernels (Huber, Cauchy, Geman-McClure) in KISS-ICP, LIO-SAM, DLIO: not retrieved; KISS-ICP adaptive threshold paper is arXiv 2209.15397 (not read).
- Numeric forgetting factor and scaling b in AKF-LIO not stated.

## 3. IMU noise values relative to data sheet / Allan variance

### Takeaway
FAST-LIO2 style configs use fixed, very large "covariance" numbers (acc_cov = gyr_cov = 0.1; bias 1e-4) that are not density units and are far above data-sheet values, i.e. inflated heavily to balance against the LiDAR term. A principled alternative derives them from Allan variance with a modest conservative inflation.

### Cited Findings
- FAST_LIO and Point-LIO IMU values above (acc_cov 0.1, gyr_cov 0.1, b_acc 1e-4, b_gyr 1e-4; Point-LIO acc_cov_output 500, gyr_cov_output 1000 for its output-model) — files cited in Question 1.
- VoxelMap velodyne.yaml scales: acc_cov_scale 1.0, gyr_cov_scale 0.5 — [VoxelMap config](https://github.com/hku-mars/VoxelMap/blob/master/config/velodyne.yaml)
- ICM40609 is the IMU in Livox Mid-360 — [Livox Mid-360 specs](https://www.livoxtech.com/mid-360/specs)
- Project Aria derives IMU noise from a 24 h static Allan variance plot, starting bias random walk from the bias instability (floor of the curve) and inflating by a tuning factor — [Project Aria IMU noise model](https://facebookresearch.github.io/projectaria_tools/docs/tech_insights/imu_noise_model). Lethander and Taylor (NAVIGATION 2023) estimate noise parameters from Allan variance so that they overbound truth because filters need conservative values (reported via search summary only).
- ORB-SLAM3 docs: take noise parameters from the datasheet or Allan analysis; example AccWalk 3.0e-3 m/s^3/sqrt(Hz) — [ORB-SLAM3 doc](https://www.mintlify.com/UZ-SLAM Lab/ORB_SLAM3/concepts/imu-integration)

### Inferences
- The 10x-or-more inflation is common practice: the IMU model ignores vibration, scale error, misalignment and time offset, and the LiDAR/vision term is itself over-weighted, so tuners raise IMU noise to avoid the IMU term being overruled in turns. If the LiDAR term is corrected to realistic information, the IMU inflation should come down toward 2-3x data-sheet/Allan values; the two knobs must be set together.
- The user's earlier measured L2 values (0.0013 m/s^2/sqrt(Hz) accel density at rest, 0.1 m/s^2 vibration floor) are consistent with this practice (floor dominates ~80x).

### Gaps
- ICM-40609-D datasheet numbers (gyro density, accel density) not retrieved; no LIO-SAM imuAccNoise/imuGyrNoise values or VINS config values retrieved (the search failed to surface them). Background knowledge only: LIO-SAM defaults imuAccNoise 3.98e-3, imuGyrNoise 1.56e-3, imuAccBiasN 6.4e-5, imuGyrBiasN 3.5e-5 (unverified).

## 4. Map representation for rough rock

### Takeaway
All strong baselines use local plane or Gaussian cells; adaptive-resolution voxels (VoxelMap) beat fixed cells on mixed scales; underground work mostly uses adaptive voxel downsampling plus GICP-style local covariances. I found little published evidence directly comparing representations on rough natural rock.

### Cited Findings
- VoxelMap: voxel fitted as one plane if the minimum eigenvalue of the point covariance is below a threshold, else subdivided up to max layer (3 m max voxel, 3 layers, 0.375 m min; 0.5 m max with 2 layers for handheld Avia); beats FAST-LIO2, Faster-LIO and others on KITTI and handheld Avia sequences, and the w/o adaptive and w/o probabilistic ablations degrade — [arXiv 2109.07082](https://arxiv.org/abs/2109.07082)
- LOCUS 2.0 (JPL, underground): adaptive voxel-grid filter vs fixed leaf 0.25 m, tested on cave datasets, constant compute for 3000 points — [LOCUS 2.0, arXiv 2205.11784](https://arxiv.org/pdf/2205.11784)
- Chen et al., "Tightly Coupled LiDAR-Inertial Odometry and Mapping for Underground Environments", Sensors 2023: NanoGICP with IMU preintegration, adaptive voxel filter, targets large caves to small tunnels — [PMC10422614](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC10422614/)
- FAST-LIO2 in a low-light mine: scans decimated by 2, voxel-filtered at 0.5 m; NDT map matching with 0.1 m voxels against a prior map — [arXiv 2603.04470](https://arxiv.org/pdf/2603.04470)
- Handheld SLAM comparison in a partially collapsed tunnel (ISPRS 2023) — [ISPRS archive](https://isprs-archives.copernicus.org/articles/XLVIII-1-W1-2023/517/2023/isprs-archives-XLVIII-1-W1-2023-517-2023.pdf)
- GICP distribution-to-distribution with plane regularisation eps 1e-3, k = 10 neighbours — sources in Question 1.

### Inferences
- On rough rock the plane assumption holds only below the roughness scale. A cell covariance with eigenvalue regularisation (floor of the smallest eigenvalue at about the sensor noise variance, e.g. (0.02-0.04 m)^2, instead of 1e-3 relative) and a minimum of 5-10 points per cell (VoxelMap uses 5 per layer; GICP k = 10) is the supported practice. Cell sizes 0.5-1 m for caves follow FAST-LIO2's 0.5 m and VoxelMap's 0.375-3 m range; a multi-resolution fallback (0.5 m then 1-2 m) addresses sparse cells.
- Because rock cells are rough, the cell covariance should be added to, not replace, the sensor-noise covariance (residual variance = n^T (Sigma_cell + Sigma_pt) n), else smooth walls and rough rock get radically different weights.

### Gaps
- Surfels (SuMa, Wildcat), NDT-OM, implicit maps, KISS-ICP point-map evidence in caves: not retrieved. No numeric cave-specific voxel size recommendation from a primary source beyond the above.

## 5. Motion distortion (deskewing)

### Takeaway
Not researched in depth in this session; only background statements, no cited sources were retrieved.

### Cited Findings
- Point-LIO processes at each point's sampling time ("point-by-point update"), avoiding scan-level deskew and giving robustness to fast motion — [Point-LIO repo](https://github.com/hku-mars/Point-LIO)
- FAST-LIO2 accumulates scans over 10-100 ms and uses IMU back-propagation for within-scan motion (paper reference, via ar5iv above).

### Inferences
- At walking speed (about 1.5 m/s) and 10 Hz scans the in-scan displacement is 15 cm, which exceeds the point noise (2-4 cm) by an order of magnitude, so per-point pose interpolation is required for a corrected weight to be meaningful; an undeskewed scan adds a coloured residual that no scalar weight corrects.

### Gaps
- CT-ICP, spline-based continuous-time, Wildcat papers and quantitative benefits not retrieved.

## 6. IMU-LiDAR calibration needed for tight coupling

### Takeaway
LI-Init (Zhu, Ren, Zhang) estimates time offset, extrinsic rotation and translation, gravity and IMU bias online, without requiring rest or special motion, and is integrated into FAST-LIO2; accuracy thresholds before a tight gyro helps were not found.

### Cited Findings
- LI-Init calibrates temporal offset, extrinsic, gravity vector and IMU bias, detecting excitation automatically, open source and integrated into FAST-LIO2; earlier methods assumed receive time equals sample time and required stillness for gyro bias — [Zhu, Ren, Zhang, "Robust Real-time LiDAR-inertial Initialization", IROS 2022, arXiv 2202.11006](https://arxiv.org/abs/2202.11006); code [LiDAR_IMU_Init](https://gitee.com/T_O_P/LiDAR_IMU_Init)
- FAST_LIO configs expose time_offset_lidar_to_imu (0.0) and extrinsic_est_en (false), extrinsic_T 4-5 cm lever arm for Livox sensors — config files cited in Question 1.

### Inferences
- With a gyro term treated as high-information, a timing offset delta_t multiplies angular rate (e.g. 80 ms at 0.5 rad/s turn is 2.3 deg), so errors larger than the gyro's own integrated noise bias the estimate. A tight gyro weight is justified only once offset errors are a few ms and the rotation extrinsic is well below 1 deg; this matches the user's own finding of an 80 ms L2 lag. These thresholds are my reasoning, not from a source.

### Gaps
- LI-Init accuracy numbers (time offset, extrinsic error) and gyro scale/misalignment treatment not retrieved. No source gave required accuracy in degrees / mm / ms.

## Search tooling notes
- Tool-call budget was exceeded modestly; PDFs of VoxelMap and Brossard extracted through pdftotext (VoxelMap formulas are lost to font encoding, so Eq. 10 contents are described, not reproduced).
- A mistaken fetch (arXiv 2005.07..., 2007.07627, 2209.13719) returned unrelated papers; none were used.
