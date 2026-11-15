# Subterranean LiDAR SLAM: practical lessons (DARPA SubT, datasets, Mid-360-class sensors, offline refinement, loop closure)

Scope note: research was limited to about 20 tool calls. Several primary PDFs (CERBERUS final paper, Wildcat, LOCUS 2.0, SubT-MRS, Trybala 2023) were too large or binary to read in full, so only abstracts/search snippets are used for them; those gaps are listed explicitly. Most DARPA facts below come from one survey of six SubT teams (Present and Future of SLAM in Extreme Underground Environments), which I read via arXiv HTML up to ~100k of 137k characters.

## 1. DARPA SubT: LiDAR odometry failures, sensors, IMU, map accuracy

### Takeaway
Winning teams all treated degenerate tunnels, narrow squeezes and dust as normal operating conditions and handled them with fallbacks (IMU/kinematic priors, thermal or intensity aids, zero-velocity factors) rather than a single clever LiDAR odometry. Map accuracy at DARPA level was roughly 6-11 % deviation for the best teams; per-robot trajectory error of tens of cm over 0.5-1 km. Tilted or wide-vertical-FOV LiDAR mounting and a good IMU were explicit lessons.

### Cited Findings
- CERBERUS (Final winner): degenerate geometry detected by thresholding eigenvalues of the approximate Hessian; ill-conditioned modules are skipped and the previous estimator's pose propagated. Zero-velocity factors added when more than one modality reports no motion for 0.5 s. Thermal fusion keeps CompSLAM low-drift in fog/smoke. Static LiDAR covariances used because dynamic balancing proved hard. Onboard mean absolute translation error 0.23-1.29 m, reduced to 0.20-0.36 m by the server-side multi-robot graph (M3RM); one robot's rotation error got worse (0.89 deg onboard vs 2.30 deg server). Four ANYmals covered 1.75 km in the Finals. No robot did onboard loop closure; a bad robot map can break the global map and an operator sometimes had to remove it. Environment: DARPA SubT Final (Louisville mine/ tunnel/ cave-like course), legged robots, Robosense BPearl dome LiDARs (90 deg FOV) plus VLP-16 and Alphasense visual-inertial unit. — [Present and Future of SLAM in Extreme Underground Environments (survey of six SubT teams)](https://arxiv.org/html/2208.01787); [CERBERUS technical overview, Tranzatto et al. 2022](https://arxiv.org/html/2207.04914)
- CERBERUS paper itself: narrow passages and smooth corridors made LiDAR odometry hard; degenerate geometry briefly leaves the pose optimization ill-conditioned so the system falls back on other estimates; smoke can corrupt LiDAR returns; keeping only the strongest return per beam is a mitigation (secondhand search summary, not verified in full text). It used manufacturer LiDAR/IMU intrinsics out of the box. — [arXiv 2207.04914](https://arxiv.org/html/2207.04914)
- CSIRO Data61 (tied top score, 2nd on tiebreak): Wildcat with a VLP-16 tilted 45 deg and spinning, effective 120 deg vertical FOV; tilt improved visibility of floors and roofs in narrow tunnels. Surfels created every 0.5 s. Loop closures from Mahalanobis-distance candidate search rather than complex place recognition. Divergences rare, mostly from sensor dropouts (impacts, hardware faults). Top Finals map had under 11 % deviation from DARPA's map; most accurate artifact reports 22 cm (Urban) and 4.8 cm (Finals). One tuned parameter set works broadly, but ground robots and drones use different sets. — [SLAM survey](https://arxiv.org/html/2208.01787); CSIRO's own claim of "0 % deviation" in DARPA's comparison is from [CSIRO news](https://www.csiro.au/en/news/all/articles/2021/november/how-team-csiros-data61-won-silver-at-the-robot-olympics) (self-reported; conflicts with the "under 11 %" in the survey, so treat as unverified).
- Team Explorer (Tunnel winner): Super Odometry uses IMU-centric sub-factor-graphs with point-to-point/line/plane LiDAR factors; keyframes every 0.2 m; Finals map deviation 6 % (second only to CSIRO per the survey); UGV runs of 445 m, 500 m, 597 m; adaptive voxel size in three modes by point spacing (small voxels for narrow corridors, larger for big caves); for UAVs points within 3 m ignored when enough features beyond 3 m (propeller dust); UAVs used for vertical shafts. — [SLAM survey](https://arxiv.org/html/2208.01787)
- MARBLE: visual-inertial odometry unstable under changing light, switched to LIO-SAM (LiDAR 20 Hz, IMU 500 Hz). LORD 3DM-GX5-15 IMU gave 0.4 deg pitch/roll accuracy so the system could run on IMU alone for extended periods; they swapped IMUs after noticing inconsistencies; USB timing produced negative timestamps. In narrow passages many LiDAR points fall below minimum range, degrading rotation accuracy; leaning on the IMU helped but did not fully fix it. Loop-closure search radius cut to 2 m to avoid false matches between tunnel levels. A 2.2 km parking-garage run with staircases and multiple levels returned within 0.31 m of the start (about 0.014 % after loop closure). — [SLAM survey](https://arxiv.org/html/2208.01787)
- CTU-CRAS-Norlab: UGV ICP optimizes only 4 DoF, roll/pitch from a Madgwick IMU filter; ablation over ~150 m showed de-skewing and roll/pitch constraints reduce error. UAV A-LOAM with a local intensity-threshold filter "highly robust" against dust; no de-skewing (compute) hurts accuracy. In Bull Rock Cave, UAV localization error stayed under 1 m over 600 m flights at up to 2 m/s, without loop closure. — [SLAM survey](https://arxiv.org/html/2208.01787)
- Cross-team open problems: localization fails after falls/collisions; systems are rigid and depend on manual parameter tuning; solutions work only under qualified conditions (environment, scale, sensors, tuning, compute). — [SLAM survey](https://arxiv.org/html/2208.01787)
- CoSTAR: LOCUS 2.0 = normals-based GICP, adaptive voxel filter (constant point count), sliding-window map; companion dataset 11 h, 16 km, legged and wheeled platforms in fog, dust, darkness and degenerate geometry. — [LOCUS 2.0, Reinke et al., RA-L 2022](https://arxiv.org/pdf/2205.11784.pdf) (abstract/snippets only; ATE tables not read).
- Vision-aided LIO thesis (CMU, H. Zhang): visual-inertial aid helps in elevator shafts and long corridors. — [CMU RI](https://www.ri.cmu.edu/publications/on-visual-aided-lidar-inertial-odometry-system-in-challenging-subterranean-environments/)

### Inferences
- Percent-of-points-within-1 m was not found in any source; DARPA's metric appears in the sources only as "deviation" percent (6-11 % best teams).
- For a handheld Mid-360S, the transferable lessons are: tilt/orientation to see floor and roof in narrow passages, IMU-led rotation (roll/pitch from gravity) when LiDAR is poor, zero-velocity factors, adaptive voxel size, ignoring near-range points when they are noise.

### Gaps
- Full CERBERUS Final results/lessons section, Wildcat T-RO accuracy numbers and LOCUS 2.0 ATE tables not read (PDFs too large/binary).
- No source found giving "percent of points within 1 m" or percent-of-distance drift for DARPA teams.

## 2. Datasets and benchmarks with underground data

### Takeaway
Smooth-walled "shield" tunnels are the hardest case and defeat nearly all published LIO systems; mine-style rough tunnels are mostly solvable with ~0.1-0.2 m ATE on 64-line sensors, but small-FOV Livox Avia gives wildly variable results and fails on stairs. Hilti scores accuracy in cm and top LiDAR teams reach ~1-3 cm RMSE on construction sites.

### Cited Findings
- GEODE (Chen et al., 2024): 64 trajectories, >64 km, seven scenarios including metro tunnels and stairs; sensors VLP-16 (30 deg vertical FOV), Ouster OS1-64 (45 deg), Livox Avia (non-repetitive, 10 Hz, 200 Hz BMI088 IMU); Mid-360 not used. Ground truth: Leica MS60 total station (tunnels), PALoc against Leica RTC360 map (stairs). — [GEODE paper](https://arxiv.org/html/2409.04961v2)
- GEODE metro-tunnel (mine-method "Tunneling") ATE in m, OS1-64: FAST-LIO2 0.12-0.17, DLIO 0.11-0.15, LIO-SAM 0.13-0.18; one COIN-LIO failure at 3.84 m (Tunneling5, OS1-64). VLP-16: 0.13-0.24 for most, but Tunneling2 failed for every method. Livox Avia: FAST-LIO2 0.15-1.88 (Tunneling1-5: 1.16, 1.88, 0.19, 0.15, 0.27), DLIO 2.48-15.43, LIO-SAM 0.28-14.77, R3LIVE up to 86 m. Sequence lengths are not given in the extract. — [GEODE paper](https://arxiv.org/html/2409.04961v2)
- GEODE smooth-wall "Shield" tunnels: nearly all methods fail on every sequence; only RELEAD succeeded on a few (ATE 1.43-79.6 m); five sequences failed for all methods. Paper conclusion: RELEAD "far from being robustly usable". — [GEODE paper](https://arxiv.org/html/2409.04961v2)
- GEODE stairs (one sequence per sensor): OS1-64 FAST-LIO2 0.38 m, DLIO 0.41, COIN-LIO 0.45, RELEAD 0.21; VLP-16 4.5-6.3 m for most (RELEAD 0.57); Livox Avia: all methods fail, attributed to limited FOV. — [GEODE paper](https://arxiv.org/html/2409.04961v2)
- Third-party GEODE numbers (D2-LIO paper): Tunnel-2 APE FAST-LIO2 0.14 m, LIO-SAM 0.13, VoxelMap 0.13; Tunnel-3 0.17/0.17/0.19. — [D2-LIO](https://arxiv.org/pdf/2508.14355) (not independently verified)
- Hilti SLAM Challenge 2023 (Nair et al.): scoring per ground control point: <0.5 cm 20 pts, 0.5-1 cm 10, 1-3 cm 6, 3-6 cm 5, 6-10 cm 3, 10-40 cm 1, >=40 cm 0. Single-session LiDAR winner KAIST URL RMSE ATE 3.3 cm (AdaLIO odometry + Quatro loop closure + factor graph, plus GTSAM OrientedPlane3Factor on one sequence); multi-session LiDAR winner ETH ASL 29.4 cm (Maplab + FAST-LIO2 + SuperPoint/SuperGlue + global BA); two top submissions at 1.2/1.6 cm were incomplete. Sites: multi-storey construction with staircases, underground parking spiralling three storeys, tunnel and warehouse with repetitive small rooms and IMU frame drops. Robot (Bpearl, ~3x noisier LiDAR) trajectories had a few cm higher error than handheld. LiDAR-based GCP detection median 3-DoF error 2.3 mm. 69 teams in 2023. — [Hilti 2023](https://arxiv.org/html/2404.09765v2). Hilti 2022: top three reached 2 cm or better on some sequences but degraded on hard ones. — [search summary](https://arxiv.org/abs/2404.09765v1)
- SubT-MRS (Zhao et al., CVPR 2024): subterranean multi-robot dataset (aerial, legged, wheeled; RGB, thermal, LiDAR, IMU) with smoke, dust, structureless corridors, varying lighting; benchmark through an ICCV 2023 SLAM challenge; per-method numbers not retrieved. — [SubT-MRS](https://arxiv.org/abs/2307.07607v2)
- Handheld in underground tunnels: Trybala, Kasza, Wajs, Remondino 2023 compared three low-cost handheld LiDAR SLAM systems against survey-grade reference in a partially collapsed tunnel; systems showed "unique and diverse strengths and shortcomings"; no numbers retrieved. — [ISPRS 2023](https://isprs-archives.copernicus.org/articles/XLVIII-1-W1-2023/517/2023/)

### Inferences
- A Mid-360-class sensor (59 deg vertical FOV, non-repetitive) sits between Avia (fails on stairs and shield tunnels) and 64-line spinners in FOV terms; GEODE evidence suggests FOV, not just algorithm, decides stairs success.
- Rough rock cave walls resemble GEODE "Tunneling" more than smooth "Shield"; expected FAST-LIO2-class error there is ~0.1-0.3 m on tens-to-hundreds of metres, which is far above survey-grade, supporting offline refinement.

### Gaps
- Not covered: Newer College, HeLiPR, ENWIDE, M2DGR, Chilean mine datasets, cave-specific LiDAR SLAM datasets (searches not run for lack of budget). Nebula/LOCUS dataset numbers not retrieved. GEODE sequence lengths not extracted.

## 3. Livox Mid-360/Mid-360S, Avia in underground settings

### Takeaway
I found no published head-to-head evaluation of FAST-LIO2/Point-LIO/iG-LIO/DLIO on Mid-360 in tunnels, mines or caves. Evidence is indirect: Avia results (GEODE) and one FAST-LIVO2 mine-tunnel test.

### Cited Findings
- Livox Avia in GEODE: fails on stairs for all methods (attributed to limited FOV) and varies from 0.15 m to tens of metres in rough tunnels. — [GEODE](https://arxiv.org/html/2409.04961v2)
- FAST-LIVO2 evaluated a "Mining Tunnel" sequence with severe LiDAR and camera degeneration, using a Livox Avia (not Mid-360). — [FAST-LIVO2 listing](https://backiee.wasmer.app/http_github_com/oxtonggue233/FAST-LIVO2) (aggregator; check the primary paper)
- FAST-LIO2 on M3DGR with Mid-360: APE 0.161 (listing; unverified details). — [paperswith code listing for FAST-LIO2](https://cs.paperswithcode.com/paper/fast-lio2-fast-direct-lidar-inertial-odometry)
- Handheld Mid-360S guidance: it is a module, not a handheld scanner; usable FOV is the spec minus areas blocked by enclosure, handle, operator; flexible mast ruins mapping quality. Mid-360 spec 360 x 59 deg. — [openelab handheld guide](https://openelab.io/pl/blogs/learn/livox-mid360s-handheld-3d-scanner-slam-mapping-guide) (vendor retail blog, low authority)
- CSIRO's 45 deg tilt of a spinning LiDAR to see floor and roof in narrow tunnels (see section 1) is the closest published evidence for a tilted/vertical mounting recommendation.

### Gaps
- No caver/surveyor forum or GitHub project evidence on handheld Mid-360 cave results found; no known-weakness list (near-range blind zone, vibration) from a primary source; no vertical-mount recommendation found.

## 4. Offline accuracy: batch refinement

### Takeaway
Hierarchical BA gives a globally consistent map in ~12 % of sequence time, but its reported accuracy gains are modest (a few percent ATE) when starting from a good LIO result.

### Cited Findings
- HBA (Liu, Liu, Kong, Zhang, RA-L 2023): bottom-up hierarchical BA plus top-down pose graph. Newer College long_experiment ATE (no loop closure) 0.26 m vs FAST-LIO2 0.35, LIO-SAM 0.53, CT-ICP 0.58, GICP matching factor 0.28. MulRan DCC01-03 ATE 5.67->5.20, 3.48->3.22, 2.84->2.54 m; time ~206-329 s vs original BA 2850-7220 s. KITTI avg (with loop-closure init) 0.7 deg / 1.4 m. Map mean entropy on solid-state indoor factory -2.99 -> -3.06, outdoor park -2.60 -> -2.69 vs FAST-LIO2. Settings: window 10, stride 5, voxel 1 m indoor / 2 m outdoor for solid-state, layers 3-4, LM via GTSAM. Note: no direct HBA vs BALM or vs pose graph comparison in the paper; unit labels inconsistent. — [HBA paper](https://arxiv.org/html/2209.11939)
- CERBERUS server-side graph reduced mean translation error from 0.23-1.29 m onboard to 0.20-0.36 m. — [SLAM survey](https://arxiv.org/html/2208.01787)
- Wildcat combines continuous-time odometry with pose-graph optimisation; outperforms two state-of-the-art methods in tunnels and corridors (abstract-level claim). — [Wildcat, arXiv 2205.12595](https://arxiv.org/pdf/2205.12595)
- Hilti 2023 winners used factor-graph/pose-graph with loop closure; multi-session winner used global bundle adjustment. — [Hilti 2023](https://arxiv.org/html/2404.09765v2)

### Gaps
- No Mid-360 or cave-specific numbers for BALM2/HBA gains; CT-ICP batch gains underground not found.

## 5. Loop closure in self-similar passages, across levels, open traverses

### Takeaway
Underground loop closure is dominated by perceptual aliasing; successful teams used geometric verification, distance/covariance-gated candidates, outlier-robust back-ends (GNC) and short search radii, and several top systems ran with no loop closure at all.

### Cited Findings
- LAMP: long featureless corridors drift; repetitive structures cause aliasing and spurious loop closures; back-end rejects outliers automatically. — [LAMP, arXiv 2003.01744](https://arxiv.org/pdf/2003.01744v1)
- LAMP 2.0 (Chang et al., RA-L 2022): loop-closure prioritization for rapidly growing candidates, GNC outlier-robust back-end; tested on mines, power plants, caves in the US. A CoSTAR-related abstract reports prioritization gave 51 % mean reduction in median error vs odometry and 75 % vs no prioritization, and better error in 1 h than processing all candidates in 4.5 h (attribution to a specific paper unconfirmed). — [LAMP 2.0](https://arxiv.org/abs/2205.13135); search summary
- CoSTAR Finals: GNC rejected up to 90 % of outlier loop closures; candidates from pose-graph distance, geometry via TEASER++ or SAC-IA then GICP refinement; tuned on 12 curated datasets. — [SLAM survey](https://arxiv.org/html/2208.01787)
- MARBLE: loop-closure radius cut to 2 m to avoid false matches between tunnel levels (relevant to shafts and multi-floor). 2.2 km garage with stairs returned within 0.31 m (0.014 %). — [SLAM survey](https://arxiv.org/html/2208.01787)
- CSIRO: loop closures via Mahalanobis-distance candidate search (covariance gating), not place recognition. — [SLAM survey](https://arxiv.org/html/2208.01787)
- CTU-CRAS-Norlab: no loop closure; cave flights of 600 m stayed under 1 m error. — [SLAM survey](https://arxiv.org/html/2208.01787)
- Hilti 2023 winner used Quatro (global registration) for loop closure and a plane factor to exploit Manhattan/Atlanta structure. — [Hilti 2023](https://arxiv.org/html/2404.09765v2)

### Gaps
- No direct Scan Context tunnel failure statistics; no data on loops needed per drift rate; no evidence on out-and-back traverse practice or on vertical-shaft loop closures found.

## 6. Extra constraints underground

### Takeaway
Gravity-based roll/pitch, zero-velocity factors and intensity/thermal aids are the documented extras; I found nothing sourced on barometers, magnetometers or surveyed control points as priors.

### Cited Findings
- Roll/pitch from IMU with ICP optimizing 4 DoF improved UGV error over ~150 m. — [SLAM survey](https://arxiv.org/html/2208.01787)
- Zero-velocity factors in CompSLAM when more than one modality reports no motion for 0.5 s. — [SLAM survey](https://arxiv.org/html/2208.01787)
- Intensity: COIN-LIO fuses intensity with geometry for tunnels (secondhand), but in GEODE tunnels COIN-LIO had one 3.84 m failure on Tunneling5 and no gain elsewhere. — [CatalyzeX listing](https://www.catalyzex.com/author/Patrick%20Pfreundschuh); [GEODE](https://arxiv.org/html/2409.04961v2)
- Local intensity-threshold filter robust against dust (CTU-CRAS UAV). — [SLAM survey](https://arxiv.org/html/2208.01787)
- Thermal fusion for fog/smoke (CERBERUS). — [SLAM survey](https://arxiv.org/html/2208.01787)
- Plane (OrientedPlane3) factors helped one Hilti sequence. — [Hilti 2023](https://arxiv.org/html/2404.09765v2)

### Gaps
- Barometer/known water level, magnetometer, surveyed-station priors: no sources found.

## Source reliability notes
- Strongest: arXiv HTML of GEODE, Hilti 2023, HBA, and the SubT survey (all read directly, but via a summarizing fetch model, so verify table values before quoting).
- Weak: search-engine snippets for CERBERUS, LOCUS 2.0, SubT-MRS, Wildcat; vendor blog for Mid-360S; aggregator pages.
- Conflict: CSIRO "0 % deviation" (news article) vs "under 11 %" (survey).
