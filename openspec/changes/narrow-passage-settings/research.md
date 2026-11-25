# Research: a narrow passage the scans do not see along

A literature and practice survey made for this change on 2026-11-25 by a research agent, with web search and fetched
sources. It builds on `openspec/changes/archive/2026-10-08-odometry-stall-fix/research.md` and
`docs/loop-closure-research.html` and does not repeat them (Zhang 2016, X-ICP, Hatleskog & Alexis, Wildcat, LAMP, PCM,
GNC, the edge-weighting practice). It is a summary, not a review.

**How to read the marks.** Every statement carries one of:

- **READ**: the source's full text was fetched and read (HTML through the fetch tool, or the PDF converted to text).
  Tables were extracted by a tool; a digit can be wrong, so check the paper before a number is reused.
- **ABSTRACT**: only the abstract, a repository page or a search summary was read.
- **INFERRED**: this agent's reasoning. Not a result of any source, and not a measurement on dMapper.

A paper's number is that paper's claim on its data. None of them is a fact about dMapper or about `raw data/livox/5`.

## Summary

1. **No published LiDAR-inertial method holds the travel in a blind passage from geometry and a consumer IMU alone.**
   Those that work add a second source along it (intensity images, camera, wheels, UWB, learned velocity, wall relief).
   With a Mid-360 in a self-similar tunnel FAST-LIO2 and DLIO covered 10 % and 3 % of 40 m (DGV-DLIVO 2026, READ).
2. **Intensity is not the route here.** The methods need image-like scans from 128-line LiDARs (READ); the Mid-360
   manual warns of a large reflectivity error under 2 m (READ), where 99 % of this recording's returns lie.
3. **A speed from the steps is the cheapest aid for a walker**: hand-held step-frequency models report 2.5 to 6 % of
   distance when calibrated, plain Weinberg about 20 % (READ). Applying a velocity only along the blind direction is
   published (ALIVE-LIO, READ). Learned inertial odometry needs data from the same device; its tunnel test is mixed.
4. **Published systems take no scan-match loop from degenerate scans** (DARE-SLAM, LAMP 2.0) or keep only the observed
   directions of the factor (Hinduja 2019); 6-DoF tunnel loops were worse than partial ones (DeCOD 2026). All READ.
5. **The mount is the cause.** All tunnel results read but one come from a LiDAR that sees along the passage. An
   out-and-back loop cannot repair the far end; control at both ends does (ISPRS 2026, READ).

---

## 1. Along-track motion when the LiDAR geometry is degenerate

What the earlier survey said stays true: detection (Zhang, X-ICP, Hatleskog) only decides where the LiDAR is not used;
the blind direction is then left to the prediction. Newer work changes the picture in one way: it stops asking how to
detect better and adds information along the blind direction.

### 1.1 Intensity and photometric terms

| Method | What it does | Result reported, on what data | Code | Mapping to dMapper |
|---|---|---|---|---|
| **COIN-LIO**, Pfreundschuh et al., ICRA 2024, <https://arxiv.org/abs/2310.01235> (READ) | Projects intensity into an image, filters it for brightness consistency, tracks 5×5 patches as a photometric residual in the iterated EKF. Patches are chosen to add information along the directions X-ICP calls uninformative. | ENWIDE, hand-held Ouster OS0-128 with its own IMU (100 Hz): TunnelS 252 m, ATE 0.74 m, relative error 1.6 %; TunnelD 180 m, 0.49 m, 1.6 %. FAST-LIO2, LIO-SAM, KISS-ICP failed on both. Table extracted from a garbled page; check before reuse. | <https://github.com/ethz-asl/coin-lio>; Ouster only | New term; not portable as it is. Authors: it "requires high-resolution LiDARs for dense intensity images". |
| **PG-LIO**, NTNU, 2025, <https://arxiv.org/abs/2506.18583> (READ) | Factor graph over a sliding window. Photometric factor with normalised cross-correlation of patches; new features started along the degenerate eigenvectors of the translation block. | ENWIDE: lower ATE than COIN-LIO on most sequences. Bicycle tunnel flown by a drone, 1047 m at 7.5 m/s, Ouster OS0-128 and VectorNav VN100: about 1 m of drift on return. In ENWIDE TunnelS the estimator did not reach the right gravity and accelerometer bias, which cost accuracy at the turn. | Announced (`ntnu-arl/mimosa`); not checked | New term; needs an organised scan from a spinning LiDAR. The TunnelS remark supports looking at gravity and accelerometer offset first (stage 1, item 3). |
| **Reflectance-aided SLAM**, Mainardi, Marmaglio, Perfetti, Vassena, ISPRS Archives 2026, <https://doi.org/10.5194/isprs-archives-XLIX-B1-2026-25-2026> (READ) | SIFT keypoints on the reflectance image of a 32-line LiDAR give sparse point-to-point terms next to the point-to-plane terms. | Gexcel Heron (two 32-line LiDARs, one horizontal, one inclined 45°), motorway tunnel 200 m long and 15 m wide, out and back, TLS reference. Odometry only, one way: APE RMSE 0.75 m geometric, 0.51 m with reflectance. With control scans at both ends: 0.078 m and 0.035 m. The tunnel had site equipment as features. | No | Shows the size of the gain (about a third) even with a sensor that sees along the tunnel. |
| **Degeneracy-aware intensity-assisted LIO**, Ding, Liu, Zheng, He, Electronics 15(17) 3970, 2026, <https://doi.org/10.3390/electronics15173970> (ABSTRACT; the page refused the fetch) | Iterated EKF with point-to-plane, photometric and inertial residuals; the photometric weight follows the number of weak translation directions. | Abstract: mean ATE RMSE 85.3 % lower than a fixed-weight intensity baseline; a search summary says it was validated on a Mid-360 with motion capture and on GEODE tunnels. Not confirmed by reading. | Not found | The only claim found of reflectivity use on a Mid-360. Read the paper before any work is planned on it. |
| **IGE-LIO**, Chen et al., IEEE TIM 2024, <https://doi.org/10.1109/TIM.2024.3427795> (ABSTRACT) | Intensity-gradient edge points as extra features, for solid-state LiDARs (Livox Avia in the search summary). | No numbers found. | Not found | Feature-based, not for an NDT map. |

**Does the Mid-360 give usable reflectivity?**

- The manual: reflectivity is 0 to 255; 0 to 150 is 0 to 100 % of a diffuse (Lambertian) target, 151 to 255 is a
  retro-reflective target. "When the target is less than 2 m from the Mid-360, it may result in a large reflectivity
  error. The data should only be used to distinguish whether the target is total reflective or diffuse reflective."
  Targets at 0.1 to 1 m with low reflectivity may not be detected. Livox Mid-360 User Manual v1.2,
  <https://www.sachtleben-technology.com/assets/downloads/livox-mid-360-user-manual.pdf> (READ).
- The ALIVE-LIO authors left COIN-LIO out of their comparison as "unsuitable for narrow-FOV LiDARs such as the Livox
  Avia" (READ, <https://arxiv.org/abs/2604.02706>).
- In this recording half the returns are within 1.15 m and 99 % within 2.74 m, so nearly all reflectivity values fall in
  the range the manual warns about (INFERRED from the manual and the proposal's numbers).
- What the manual does allow: telling a retro-reflective target from a wall, also at close range. Retro-reflective
  marks on the wall are therefore detectable by this sensor (INFERRED; see section 7, recording).

### 1.2 Geometry only: detail inside the planes

| Method | What it does | Result reported | Code | Mapping |
|---|---|---|---|---|
| **BIEVR-LIO**, ETH, RSS 2026, <https://arxiv.org/abs/2604.14421> (READ) | Each 0.5 m voxel stores its plane and a "bump image": the height of the points above the plane at 5 cm pixels. The residual is the point's height minus the image value; its gradient adds the two in-plane directions that point-to-plane cannot see. Points are sampled from the voxels with the most relief. | GEODE shield tunnels with a **Livox Avia**: ATE 0.26, 0.28, 0.15 m (Shield1, 4, 5), where most baselines diverge. ENWIDE: better than COIN-LIO on all but one sequence, without intensity. Ablation on Shield1: 0.26 m with the gradient term, 0.40 m without. | <https://github.com/ethz-asl/bievr-lio> | A new map and term (large work). Authors' limits: needs dense points (pixels stay empty with sparse scans), fails where walls are truly smooth (ENWIDE Tunnel), needs a good first pose. |
| **GenZ-ICP**, RA-L 2024, <https://arxiv.org/abs/2411.06766> (READ) | Weights point-to-plane against point-to-point by the share of planar correspondences. LiDAR only. | SubT-MRS Long_Corridor APE RMSE 1.99 m; KISS-ICP 8.72, point-to-plane 33.16, Zhang 20.05, X-ICP diverged. Tuned per dataset. | pip package `genz-icp` | The point-to-point part is the false along-wall stiffness the earlier survey warned about, used on purpose. As a module on FAST-LIO2 it diverged on the same sequence in LODESTAR's ablation (READ). Not recommended. |

BIEVR-LIO is the one geometric method that reports success on a Livox sensor in tunnels. It uses relief of centimetres
on the walls. In this recording such relief (door frames, niche edges) is in view only while the sensor is beside it,
so the term would sharpen the use of what is seen, and add nothing where the wall is flat (INFERRED).

### 1.3 Degeneracy-aware fusion that keeps the prediction along the blind axis

| Method | What it does along the blind direction | Result reported | Code | Mapping |
|---|---|---|---|---|
| **LODESTAR**, KAIST, RA-L 2025, <https://arxiv.org/abs/2511.09142> (READ) | Sliding window on FAST-LIO2. Past states that were well conditioned are frozen and act as anchors through their covariance (Schmidt-Kalman); their measurements are reused along the degenerate directions. | SubT-MRS Long Corridor APE 1.39 m (FAST-LIO2 3.11), Cavern hand-held 0.67 m (3.52). Sensors include a Livox Mid-70. Authors: it "fails when LiDAR returns are absent"; the threshold 1.5 is "neither universal nor optimal". | In the supplement | dMapper's window with its carried priors is the same idea. No new information along the passage. |
| **D²-LIO**, 2025, <https://arxiv.org/abs/2508.14355> (READ) | A penalty that holds the solution at the IMU prediction with strength 1/eigenvalue per direction, weighted with the preintegration covariance. | GEODE Tunnel-2 APE 0.10 m (FAST-LIO2 0.14); worse than FAST-LIO2 on SubT Long_Corridor_1 (1.65 against 1.08). Authors: gain is marginal in mild cases and LiDAR-IMU alone is not enough in severe ones. | No | A soft form of the degeneracy projection (the setting `degenerate_projection_ratio`). The paper's own tables say the gain is small. |
| **RELEAD**, Chen et al., ICRA 2024, <https://arxiv.org/abs/2402.18934> (ABSTRACT) | X-ICP detection, constrained ESIKF update, GNC back-end for other sensors. | In GEODE's benchmark it is the only method that returns a result on the hardest shield and urban tunnels, with errors above 10 m (READ, GEODE paper). | Not checked | Confirms: constraint handling keeps the filter alive, not accurate. |
| **DAMS-LIO**, Han et al., 2023, <https://arxiv.org/abs/2302.01703> (ABSTRACT) | Uses another odometry's relative pose as a measurement only when the LiDAR is degenerate. | SubT data, against LIO-SAM; no numbers read. | Not found | The pattern "add a second source only along the blind direction", which sections 2 and 7 use. |
| **Super Odometry 2.0**, Zhao et al., Science Robotics 2025, <https://arxiv.org/abs/2608.25427> (READ) | Per-axis confidence from the share of correspondences that constrain each axis; the other sensor is weighted up on the weak axis; a learned inertial model is the fall-back. | SubT-MRS average ATE 0.27 m over eight sequences (Corridor01 0.82 m, Cave03 0.20 m). | Data on Zenodo; code not linked | The learned inertial part is in section 2. |
| **Mining tunnels with kinematic regularisation**, Research Square 2026, <https://www.researchsquare.com/article/rs-11764029/v1> (ABSTRACT, search summary only) | A regularised Schur complement weights weak directions continuously. | Lowest ATE on five of six shield-tunnel sequences (preprint, not reviewed). | Unknown | Not read; listed for completeness. |

### 1.4 Tunnel-specific work with a Mid-360 or another Livox

| Source | What it shows |
|---|---|
| **DGV-DLIVO**, 2026, <https://arxiv.org/abs/2611.22385>, code <https://github.com/0RayZhang0/DGV-DLIVO> (READ) | **Mid-360 with its own IMU** plus a camera in self-similar spillway tunnels. Reference: wall marks every 2 m with the passing times noted by hand. On a 40 m cart run: FAST-LIO2 reached 10 % of the travel (end error 36 m), DLIO 3 % (39 m), FAST-LIVO2 0.1 % with "persistent oscillation", R3LIVE 58 %, the proposed camera-aided method ended at 43.5 m (end error 3.5 m). On a hand-held run of about 6 m: R3LIVE 2.3 m. The camera residual is applied only along the degenerate direction, and the LiDAR is suppressed there. |
| **ALIVE-LIO**, POSTECH, 2026, <https://arxiv.org/abs/2604.02706> (READ) | GEODE shield tunnels, hand-held Livox Avia: FAST-LIO2 ATE 37 to 101 m, with a learned velocity 4 to 23 m. See section 2. |
| **LIO-Livox** README, <https://github.com/Livox-SDK/LIO-Livox> (READ) | "Pass through a 4 km tunnel" at about 80 km/h on a car with a Livox Horizon. No error number. A car at speed, not comparable. |
| **GEODE benchmark**, <https://arxiv.org/abs/2409.04961> (READ) | "Failures are common in environments with identical geometry"; most methods fail on Shield7 to 10. |

**What accuracy over what distance with a consumer IMU.** The best published cases with a MEMS IMU and a LiDAR that
sees along the tunnel are about 0.1 to 0.3 % of distance with intensity (COIN-LIO 0.5 to 0.7 m over 180 to 250 m; PG-LIO
1 m over 1 km with a VN100) and 0.25 to 0.4 % without it in a tunnel with features (ISPRS 2026, 0.5 to 0.75 m over
200 m). Without a second source in a self-similar tunnel the same class of system loses most of the travel (DGV-DLIVO;
GEODE). No source gives a figure for IMU-only bridging over tens of seconds with a consumer IMU; the earlier survey's
estimate (0.02 m/s² is 1 m after 10 s) stands as INFERRED.

---

## 2. Speed or displacement of a walking person from a hand-held IMU

### 2.1 Step models (pedestrian dead reckoning)

| Source | Model | Accuracy reported, conditions |
|---|---|---|
| **Renaudin, Susi, Lachapelle**, Sensors 2012, <https://pmc.ncbi.nlm.nih.gov/articles/PMC3444061/> (READ) | Step length s = h·(a·f + b) + c, with user height h and step frequency f. f from the strongest peak of a short-time Fourier transform of the hand's accelerometer (a peak under 1.4 Hz is the stride, so it is doubled). 100 Hz. | **Hand-held**, texting and swinging, 10 test subjects, curved route of about 300 m: distance error 5.7 % on average (3.2 to 9 %) with one universal set of parameters; 4.2 % (2.5 to 5 %) after per-user calibration on two straight walks. The worst case was a subject much taller than the training group. Flat ground only. |
| **Ho et al.**, Sensors 2016, <https://pmc.ncbi.nlm.nih.gov/articles/PMC5038701/> (READ) | The Weinberg form (a gain times the fourth root of the step's vertical acceleration range, as the 2024 overview gives it), with the gain made a function of the step's mean speed. | Phone held in front of the chest, 8 subjects, 10 to 40 m, three paces: plain Weinberg 17 to 28 % (about 20 %), adaptive 4.2 to 4.9 %. Assumes the device keeps its orientation. |
| **Pedestrian inertial navigation, an overview**, 2024, <https://arxiv.org/abs/2407.21676> (READ) | Constant, Weinberg, and a frequency-and-variance model. | One 25.4 m walk in texting mode: under 2 % for the two calibrated models. A single average gain gives about 10 % error. Gains must be calibrated per user, carry mode and sensor place, which the overview calls a burden. |
| **RoNIN** paper's PDR baseline, <https://arxiv.org/abs/1905.12853> (READ) | Step count times a fixed 0.67 m along the device heading. | Works on easy walks (ATE 2.1 to 3.3 m on OXIOD), fails on natural phone handling (ATE 28 to 30 m on the RoNIN data). |

What these do and do not need (READ unless marked): 100 Hz is enough; the device must be carried in a steady way; they
give a **speed along the walk, not its direction**, and nothing when the walker stands (dMapper's rest detection covers
that case, INFERRED). Zero-velocity updates do not apply to a hand-held or upper-body IMU: there is no instant of zero
velocity in the gait (search summary of the P-LINS paper, ABSTRACT).

No study was found that gives the error of a hand-held, cadence-only speed for one user at a steady pace. The figures
above mix several users and paces, so one calibrated user at one pace should sit at the low end (INFERRED).

### 2.2 Learned inertial odometry

| Method | Needs | Accuracy reported | Generalisation |
|---|---|---|---|
| **RoNIN**, 2019, <https://arxiv.org/abs/1905.12853> (READ) | 42.7 h, 100 subjects, three phones, 200 Hz, any carry. ResNet/LSTM/TCN regress horizontal velocity. | ResNet on its own data: ATE 3.54 m and 2.67 m per minute for subjects seen in training; 5.14 m and 4.37 m per minute for unseen subjects. | On another device and data set (OXIOD): 2.40 m seen, 6.71 m unseen. Error grows by half to nearly three times off the training set. |
| **TLIO**, 2020, <https://arxiv.org/abs/2007.01867> (READ) | 60 h, 5 people, one headset IMU (Bosch BMI055), 200 Hz input. The network gives a 3D displacement over 1 s with a covariance; an EKF fuses it with the IMU states and estimates the biases. | 3.12 % of distance on its own data, as re-measured by a later paper (HeadIO, <https://arxiv.org/abs/2611.14895>, READ). | Authors: limited by the training data; unusual motions (side-stepping, rolling on a chair) fail. Head-mounted, not hand-held. |
| **AirIO**, CMU, 2025, <https://arxiv.org/abs/2501.15659> (ABSTRACT) | Body-frame IMU input with attitude; velocity output; EKF with preintegration. For drones. | 66.7 % better than global-frame input on drone data. | Not pedestrian. Its network is the one ALIVE-LIO reuses. |
| **Tartan IMU / Super Odometry 2.0**, CMU, CVPR 2025 and Science Robotics 2025, <https://arxiv.org/abs/2608.25427>, weights <https://huggingface.co/Tartan-IMU/TartanIMU> (READ for the second) | Over 100 h from eight platforms including hand-held, resampled to 200 Hz. ResNet-1D and LSTM; body-frame velocity with covariance; a head per platform type; LoRA adaptation. | Hand-held ATE 4.32, relative error 1.95 (units not stated in the text). About 36 % better ATE than specialised models. | Authors: it "struggles with unseen domains"; adapting to a new vehicle took 60 s of data with a reference. Used as the fall-back when LiDAR and camera fail. |
| **EqNIO**, ICLR 2025, <https://arxiv.org/abs/2408.06321>; **HeadIO**, 2026 (ABSTRACT; READ) | Frame-equivariant inputs; Mamba backbone for head-worn devices. | 1.8 to 5.8 % of distance on head-worn data sets. | HeadIO states that hand-held carry (non-rigid, varied) is a different case from a rigid head mount. Transformer models (CTIN, iMoT) were only seen cited, not read. |

### 2.3 Papers that fuse such a speed into LiDAR or visual odometry

| Source | How it is fused | Result |
|---|---|---|
| **ALIVE-LIO**, POSTECH, 2026, <https://arxiv.org/abs/2604.02706>, code <https://github.com/cocel-postech/alive-lio> (READ) | A network (AirIO's, bidirectional GRU, 5 s of 200 Hz IMU) predicts body-frame velocity with a sigma. X-ICP-style detection per iteration. **The velocity residual is projected onto the degenerate eigenvectors and applied only there, only when degeneracy is detected.** Labels for training come from PV-LIO in well-conditioned stretches. One model for all Livox data; a pre-trained `model_handheld` (200 Hz, Livox built-in IMU) is in the repository, and the training data include a hand-held Mid-360 recording. Velocity noise scale 0.01. | Network velocity error 0.10 to 0.64 m/s, mean 0.22 m/s, on unseen sequences. GEODE shield tunnels (hand-held Avia): ATE 4.2 to 23 m against 37 to 101 m for FAST-LIO2. **Own hand-held tunnel, Avia carried sideways facing the wall, out and back: end-to-end error 12.65 m against 0.72 m for the camera-aided FAST-LIVO2 in the lit run; 30 m against 205 m in the dark run.** Authors' limits: frequent rotation degrades the prediction; bias feedback can drift under long degeneracy; wrong detection leaves the baseline unchanged. GPL-2.0. |
| **P-LINS**, Liao et al., IEEE Sensors Journal, <https://doi.org/10.1109/JSEN.2024.3523131>, code <https://github.com/piapi/P-LINS> (ABSTRACT) | IMU-centric LiDAR-inertial filter for a pedestrian, with constraints on the velocity "based on pedestrian movement physiology" and a degeneracy factor that lowers the LiDAR's weight. | Abstract: on par with other systems in structured scenes, better where the LiDAR is degenerate. The constraint itself and the numbers could not be read (paywall; the repository page is empty). **The closest published match to the proposal's "speed from the walker's steps".** |
| **Body-mounted LiDAR SLAM with a PDR model**, Sadruddin, Carleton thesis, <https://carleton.scholaris.ca/items/07e45474-2e22-4012-bbc4-714544977911> (ABSTRACT) | PDR fed to Cartographer in the place of wheel odometry; 2D LiDAR. | Better accuracy and coverage in featureless areas, no numbers read. |
| **RNIN-VIO**, ISMAR 2021, <https://github.com/zju3dv/rnin-vio> (ABSTRACT) | A neural inertial network as a third measurement in a visual-inertial EKF, for hand-held AR. | "Significantly better" than plain VIO in hard scenes, close to it in normal ones. |
| **TLIO** (above, READ) | Displacement with covariance as an EKF update next to the IMU states. | The design template for a displacement factor. |

No paper was found that fuses a classic step-frequency speed into a 3D LiDAR-inertial odometry as a factor for hand-held
use and reports numbers. P-LINS may be that paper; it was not readable.

### 2.4 The simplest robust version for one user at a steady pace (INFERRED, from 2.1 to 2.3)

- **Step frequency** from the accelerometer over a few seconds (Renaudin's spectral peak with the 1.4 Hz rule). The
  Mid-360S IMU at 500 Hz is more than the 100 Hz these models use.
- **Speed = step frequency × step length**, the step length linear in the frequency, with the two or three parameters
  fitted **for this user from stretches where the scans are well conditioned** (the rooms of the earlier Livox
  recordings, or the ends of this one). This is ALIVE-LIO's labelling idea with three parameters in the place of a
  network, so it needs minutes of data, not hours, and cannot fail outside a training set in an unseen way.
- **One residual per window on the speed along the travel**, applied along the weak axis only, with a sigma of 5 to
  10 % of the speed (the range of 2.1), and absent when the window is at rest or no steady gait is found.
- What it gives against the present state: a 5 % speed error is 5 % of the distance, and does not grow with the square
  of the blind time. An out-and-back at one pace cancels a scale error at the return, so the turning point's place
  carries the error, not the closure.
- What it cannot give: the direction (that stays with the attitude and the across-passage LiDAR terms), slopes and
  steps (the models are for flat ground), crawling or squeezing in a cave (no gait). The last one matters for the
  target use and should be said before the term is built.

---

## 3. Loop closure and place recognition in self-similar passages

### 3.1 Rules of the form "no scan-match loop where the registration saw nothing along the passage"

| Source | The rule | Evidence |
|---|---|---|
| **DARE-SLAM**, Ebadi et al., 2021, <https://arxiv.org/abs/2102.05117> (READ) | A scan with log(condition number) above a threshold is degenerate; degenerate poses and key-scans are "removed from loop closure consideration", query and candidate both. The remaining scans must also be salient in an occupancy image. Then ICP and PCM. | Mines of 500 to 1400 m; a 10 m radius search without the rule distorted an 11 km coal-mine map. The example of a place that does close a loop is a T-junction. |
| **LAMP 2.0**, 2022, <https://arxiv.org/abs/2205.13135> (READ) | "Observability prioritization": candidates are ranked by the eigenvalues of the point-to-plane information matrix, so that loops are not closed "in ambiguous areas with high degree of geometric degeneracy" (wording from the SubT survey). Search radius 0.2 m per node of path between the two. Then ICM and GNC. | Tunnel data set: 9755 candidates, 5656 verified, 1645 inliers (about 29 %). GNC is said to cope with more than 80 % outliers. No covariance for loop factors is given. |
| **AEROS** front-end, Ramezani, Mattamala, Fallon, 2022, <https://doi.org/10.3389/frobt.2022.789444> (READ) | After ICP the matched point set must hold "a diversity of normals, i.e. there is no degeneracy", or the candidate is dropped. | No count of rejections given. |
| **Degeneracy-aware factors**, Hinduja, Ho, Kaess, IROS 2019, <https://www.cs.cmu.edu/~kaess/pub/Hinduja19iros.pdf> (READ; the earlier survey had it from a citation) | The ICP Hessian's eigenvectors are split by a threshold on the eigenvalue ratio. The loop factor's error and covariance are projected onto the well-conditioned eigenvectors (Vf): the factor says nothing along the others. Candidates that fail the threshold are rejected. | Sonar on ship hulls and a tank; lower RMSE than full factors. The threshold is hand-set; its automatic choice is named as future work. |
| **DeCOD**, Kim et al., 2026, <https://arxiv.org/abs/2609.36753> (READ) | The opposite half: loops are taken only from cross-sectional boundaries (pipe joints, tunnel rings), matched in both headings and verified by an inlier count; the factor is 5-DoF (place of the cross-section and axis alignment), with the rotation about the axis left free. | GEODE shield tunnels: trajectory RMSE 61.6 m → 0.77 m (S8), 132.8 → 4.2 m (S10), 94.9 → 24.8 m (S9, few loops). **With full 6-DoF factors: 6.4 m and 9.7 m.** Precision 95.6 % at recall 31.3 % with at least 15 inliers; without verification 11.7 %. Descriptor baselines (STD, BTC) near 0 %. Code not stated. |
| **QCSA**, 2025, <https://arxiv.org/abs/2512.09447> (ABSTRACT) | Admit a loop only when a whole short segment of keyframes agrees, with a threshold calibrated on the strongest false segment. | 53.8 % fewer registration attempts and lower error than the best single match, in a library with repeating aisles. |
| **Loop closing with point cloud density maps**, 2025, <https://arxiv.org/abs/2501.07399> (ABSTRACT) | Self-similar descriptors are pruned before they enter the database. | Fewer false matches on a bridge. |

So the answer to "is there a published rule" is yes, in three forms: do not use degenerate scans for loops at all
(DARE-SLAM, LAMP 2.0, AEROS), give the loop factor no information along the unobserved direction (Hinduja), or accept
a loop only from a structure across the passage and constrain only what it fixes (DeCOD). The proposal's follow-up
"loop edges that carry no information along the passage from scans that see nothing across it" is the second form. In
a recording where **every** segment is degenerate along the passage, the first form means no loops at all (INFERRED).

### 3.2 What the underground mapping teams did (SubT survey, Ebadi et al., <https://arxiv.org/abs/2208.01787>, READ)

- **CoSTAR**: the LAMP 2.0 chain above; GNC rejected up to 90 % of the loop candidates at the Finals.
- **CERBERUS**: no loop closure on the robots. On the base station, an ICP result that differs much from the odometry
  prior is rejected; LiDAR factors have fixed, hand-tuned covariances; a human watches the map and can remove a robot.
  A search summary of later work on the SubT systems (CompSLAM, <https://arxiv.org/abs/2505.06483>, and a 2025 analysis
  whose source was not pinned down) reports that loop detection gave too many false positives and that a wrong loop
  hypothesis "would completely break the mapping system" (ABSTRACT).
- **CSIRO (Wildcat)**: place recognition was "rarely found to be necessary at the scale of SubT"; candidates are earlier
  poses within a Mahalanobis distance; Cauchy kernel.
- **CTU-CRAS-Norlab**: no loop closure; revisits are handled by matching against the kept map.
- **MARBLE**: search radius cut to 2 m, to avoid false loops between levels.
- **Explorer**: radius search; loop closure needed above 1 km.

The common thread: candidates only where the odometry says the two poses are close, a geometry gate before the
back-end, and a robust back-end last. None relies on the back-end alone (INFERRED from the list).

### 3.3 Where newer work changes the earlier picture

- The earlier pages set "information-weighted loop edges" against "fixed sigmas". DeCOD's ablation adds a measured
  case: in tunnels, what a factor constrains matters more than how it is weighted (6-DoF against 5-DoF, READ).
- The earlier pages named PCM and GNC as the outlier defence. In a self-similar passage false matches agree with each
  other and with a loosened odometry, so consistency checks pass them. This is what the proposal reports (34 loops, none
  dropped) and what LAMP 2.0's authors avoid by ranking before verification (READ for LAMP; the reading is INFERRED).
- An out-and-back loop cannot repair the far end: "the loop closure (CS3) is not able to compensate itself the drift at
  the outermost section of the tunnel"; control scans at both ends took APE RMSE from 0.75 m to 0.08 m, with the rest
  in the middle (ISPRS 2026, READ).
- "Information-aware odometry and retroactive loop closure", 2026, <https://arxiv.org/abs/2607.13516> (ABSTRACT):
  odometry edges weighted by a geometry-dependent ICP information, and a protocol for map consistency at revisits. It
  supports the readings rule's direction; not read in full.

---

## 4. How hand-held scanners and Mid-360 users deal with long passages

### 4.1 Documented

| Source | What it states |
|---|---|
| Livox Mid-360 manual (READ, link in 1.1) | "There is no orientation requirement when mounting the Mid-360"; upside down needs 0.5 m to the ground. Field of view 360° by −7° to +52°. |
| Livox showcase of a commercial hand-held scanner, <https://www.livoxtech.com/showcase/18> (READ) | The Mid-360 is mounted "at a 20° tilt to the ground" so that one sensor sees the ground, the area ahead and the ceiling. No tunnel is mentioned. |
| **Mandeye in caves**, Redovniković, Jakopec, Będkowski, Jagetić, Int. J. Speleology 53(3), 2024, <https://doi.org/10.5038/1827-806X.53.3.2535> (READ) | Mid-360, hand-held in confined passages and on a helmet in larger ones. Against a static scan: most differences within 15 cm and the vast majority under 25 cm in the tourist section; in the narrow part most under 50 cm; "the achieved deviations (25–50 cm)". The narrowest, low part of about 3 m gave larger deviations. The device was set down at survey stations of a polygonal traverse as control; "relying solely on georeferencing at the entrance reduces accuracy". Scans of 78 m and 267 m of trajectory. The sensor's orientation is not stated. |
| **Hand-held systems in a collapsed tunnel**, Trybała, Kasza, Wajs, Remondino, ISPRS Archives 2023, <https://doi.org/10.5194/isprs-archives-XLVIII-1-W1-2023-517-2023> (READ) | GeoSLAM ZEB Horizon, an actuated Velodyne VLP-16, and a hand-carried Livox Horizon with FAST-LIO and Scan Context loops. The Livox had the best local accuracy but gaps from its narrow view, and "double-wall and double-floor" errors in one section. All three were above 200 mm standard deviation to the reference (232 to 364 mm; the table's layout was unclear in the extraction), which the authors assign to drift. For one system the largest errors were in the area farthest from the starting point, which was also where the loop closed. |
| Emesent, "Understanding Hovermap", <https://knowledge3.emesent.com/understanding-hovermap> (READ) | The LiDAR has a 30° view; the rotating head turns it into a sphere, densest ahead. Hold it "so it can see both where it has been and where it is going", above all through narrow passages and doorways. Smooth tunnels are a known slip risk: avoid them or add fixed geometric features; "markings and colors don't count". Minimum range 0.5 m. |
| Emesent, "SLAM drift", <https://knowledge.emesent.com/docs/slam-drift> (READ) | Long scans in one direction (tunnels, roads) collect more drift; close the loop; keep scans short; drift cannot be fully removed after capture. |
| CSIRO Zebedee, Bosse et al., 2012, <https://research.csiro.au/robotics/zebedee-design-of-a-spring-mounted-3-d-range-sensor-with-application-to-mobile-mapping/> (ABSTRACT) | The spring makes a 2D scanner rock, "extending the device's field of view outside of its standard scanning plane". This is the stated reason for the moving head in that family (GeoSLAM ZEB). |
| ALIVE-LIO's tunnel data (READ, link in 2.3) | A Livox Avia "carried sideways while mostly facing the tunnel wall", out and back: made on purpose as a degenerate case. The same situation as this recording. |
| GrandTour, <https://arxiv.org/abs/2602.18164> (ABSTRACT, through a citing paper) | A Mid-360 mounted upside down on a legged robot's head, with a limited view as a result. |

### 4.2 Reported informally (search summaries; not read at the source)

- FARO/GeoSLAM guidance for the ZEB Horizon: close the loop; open-loop scanning is not recommended; scans of at most
  15 to 20 minutes (ABSTRACT).
- FAST-LIO issue 284, <https://github.com/hku-mars/FAST_LIO/issues/284>: a Mid-360 mounted upside down drifted; the
  likely cause in a related issue was the extrinsic rotation, not the orientation itself (ABSTRACT).
- A vendor guide for the Mid-360S: there is no single right tilt; more floor in view costs wall and ceiling (ABSTRACT).

### 4.3 Not found

- No paper, issue or vendor note that compares Mid-360 orientations (upright, tilted, axis along the travel) in a
  tunnel or cave.
- No statement by GeoSLAM itself on why the head rotates; the Zebedee abstract and Emesent's page are the sources used.
- The text of Zlot & Bosse 2014 (caves with Zebedee) could again not be fetched (the server refused).

### 4.4 What follows for the mount (INFERRED, from the manual's field of view)

- With the axis along the travel, the band −7° to +52° off the plane across the axis leaves the cone round the axis
  unseen: the passage ahead and behind. This is the proposal's finding.
- With the axis upright, the same band looks at the horizon all round: the passage is seen ahead and behind to the
  range limit, the walls beside, the ceiling up to 52°, the floor only from some metres away (−7°).
- With the commercial 20° tilt, the floor ahead and the ceiling behind come into view, and the passage is still seen
  both ways.
- The tunnel results in section 1 were recorded with a sensor that sees along the tunnel, and those systems still lose
  the travel where the tunnel is smooth. The one exception found is ALIVE-LIO's own tunnel data (an Avia held facing
  the wall), where the LiDAR-inertial methods diverged or ended 12 to 30 m off. A mount that hides the passage removes
  the door frames ahead as well.

---

## 5. Judging a map of an out-and-back passage without ground truth

| Measure | Source | What it shows | What it cannot show |
|---|---|---|---|
| Start-to-end error of a walk that returns to its start, loop closure off | Used as the only measure in ALIVE-LIO's tunnel data and in DegLoc, <https://github.com/ygd551/DegLoc-Dataset> (READ) | Net drift over the whole walk. | A scale error that is the same both ways cancels (the design's remark on M1; no source states it, INFERRED). After loop closure it is a reading of the loop, not of the odometry ("Look Ma, No Ground Truth", <https://arxiv.org/abs/2412.01116>, ABSTRACT). |
| Estimated length against the known length; end-to-start offset | M2TD's description of its baselines, <https://github.com/longlongzxl/M2TD> (READ) | Under-travel, the failure seen here. | Needs the length. |
| Sparse marks along the passage with the passing times | DGV-DLIVO: wall marks every 2 m, times noted by hand (READ) | Travel against time along the weak axis only. | Hand timing; no cross-track or attitude check. |
| Fiducials at the ends of an out-and-back corridor | "Challenges of Indoor SLAM", <https://arxiv.org/abs/2306.08522> (ABSTRACT) | Drift between fixed marks without a full reference trajectory. | Only at the marks. |
| Control scans or surveyed stations at both ends | ISPRS 2026; the Mandeye cave paper (READ) | Fixes the ends; the residual sits in the middle. | The middle. |
| Mean map entropy; mean plane variance; mutually orthogonal metric | Razlaw et al. 2015 (in the earlier survey); Kornilova & Ferrer, <https://arxiv.org/abs/2106.11351> (ABSTRACT); MapEval, <https://arxiv.org/abs/2411.17928> (READ) | Local sharpness; MOM correlates with relative pose error on CARLA and KITTI. | MapEval: the scope is "restricted to local shape analysis without considering any global geometry"; unstable under noise and outliers; unitless. A corridor folded onto itself with thin walls scores well (INFERRED). |
| Agreement of two passes at revisits | A protocol in <https://arxiv.org/abs/2607.13516> (ABSTRACT); the 2023 tunnel paper's double walls (READ) | Whether the way back lies on the way out. | In a self-similar passage the walls agree at any shift along it; only the structures across the passage tell (the design's M3; INFERRED). |
| Spread under small perturbations of the input | "Look Ma, No Ground Truth" (ABSTRACT) | Fragility of the estimate. | Not accuracy. This is dMapper's perturbation sweep. |

No published measure made for out-and-back passages was found. The design's M1 to M3 are not contradicted by anything
read. The literature's own practice, when the passage is self-similar, is a length or marks: the tunnel papers above
that report a number have a reference along the tunnel (total station, TLS, wall marks, UWB), but for ALIVE-LIO's and
DegLoc's own recordings, which fall back on the start-to-end error. The design's open question (pace or measure the
corridor) is what the sources do (INFERRED from the list).

---

## 6. Open data sets with degenerate passages and a Livox-type sensor

| Data set | Sensor and carry | Passages | Reference | Notes |
|---|---|---|---|---|
| **M2TD**, <https://github.com/longlongzxl/M2TD> (READ) | **Livox Mid-360** and Avia with built-in IMUs (200 Hz), camera, UWB tag; hand-held, vehicle, drone. Mount orientation not stated. | Unfinished subsea tunnel and traffic tunnels: hand-held 780 m and 318 m; 11 sequences, 6.5 km. | LiDAR + IMU + one UWB anchor filter; its accuracy is not given (UWB ranging 0.1 m). | ROS2 bags on Baidu Pan (0.9 to 15 GB each). Paper under submission. Licence text contradicts itself. The closest match in sensor; the sensor very likely sees along the tunnel. |
| **GEODE**, 2024 (a journal version is listed under DOI 10.1177/02783649251344967), <https://arxiv.org/abs/2409.04961>, <https://github.com/PengYu-Team/GEODE_dataset> (READ) | **Livox Avia** (BMI088, 200 Hz), Ouster OS1-64, Velodyne VLP-16, Xsens MTi-30; hand-held and others. | Metro tunnels: 23 sequences, 7.5 km (13 tunnelling, 10 shield); also stairs, flat ground. | Leica MS60 total station on a prism, position only. | ROS1 bags for the Avia. The benchmark every 2025/26 paper above uses. The Avia has a 70° forward cone, a different blind pattern. |
| **ALIVE-LIO tunnel data**, <https://github.com/cocel-postech/alive-lio> (READ) | Livox Avia, hand-held, **carried sideways facing the wall**. | One road tunnel, out and back, 357 s and 380 s. | None; start equals end. | The only public data found with the same kind of blind mount. |
| **DegLoc**, <https://github.com/ygd551/DegLoc-Dataset> (READ) | Livox Avia with IMU (200 Hz); carry not stated. | Four corridor bags among 24 or 25; lengths not given. | None; closed loops except two corridors. | ROS1 bags, about 13 GB, Hugging Face. No paper, no licence. |
| **ENWIDE**, <https://projects.asl.ethz.ch/datasets/enwide> (READ) | Ouster OS0-128 with its IMU (100 Hz), on a hand-held stick. | Tunnel, intersection, runway, field, lake shore; a smooth and a dynamic run each; 180 to 390 m. | Leica MS60 total station. | Not a Livox; a spinning LiDAR that sees along the tunnel. Useful only if the reader is extended. |
| **SubT-MRS**, CVPR 2024, <https://arxiv.org/abs/2307.07607> (ABSTRACT) | Velodyne VLP-16; legged, wheeled, aerial, some hand-held (a cave run). | Long corridor, caves, mines. | Scans matched to a FARO map. | Not a Livox. |
| **NTNU degeneracy data**, <https://github.com/ntnu-arl/lidar_degeneracy_datasets> (READ) | Ouster OS0-128, VN100, radar; drone. | 500 m straight bicycle tunnel, 8 m wide; a fog corridor. | None stated. | Not a Livox, not walked. |
| **CAVERS**, 2026, <https://arxiv.org/abs/2604.15052> (ABSTRACT) | VLP-16, RGB-D, thermal; hand-held and rover. | Two rooms of a karst cave. | Motion capture in the cave. | A real cave, but rooms, not passages. |

For dMapper a data set needs a reader for its bags (dMapper reads LAZ and CSV) and the lever arm of its sensor. The
Mid-360 and Avia are not supported sensors today except the Mid-360S (INFERRED from the project's notes). The cheapest
second test is therefore a second recording of the user's own, made with the other mount (section 7).

---

## 7. What this suggests for dMapper

Ranked inside each group. "Evidence" names what was read; the judgement of size and fit is INFERRED throughout.

### 7.1 What can be tried as a setting now

1. **Loop closure off, or only where the scans are not degenerate.** Evidence: DARE-SLAM, LAMP 2.0, AEROS, CTU's choice
   of no loop closure, DeCOD's 6-DoF ablation. In this recording that is `use_loop_closure = false` (already in D3).
   Expect the odometry's errors to stay, not to be repaired.
2. **A short search radius and a small largest correction** (D3's 3 m and 2 m, 2 m and 1 m). Evidence: MARBLE's 2 m;
   LAMP's radius of 0.2 m per node of path; CERBERUS rejecting matches far from the odometry prior. It limits the size of
   a false fold; it does not make a match true.
3. **The acceleration offset slower, or frozen, in blind stretches** (D3's first row). Evidence: ALIVE-LIO's limit
   "bias feedback can drift under prolonged degeneracy"; PG-LIO's TunnelS failure from a wrong gravity and accelerometer
   bias; the earlier survey's inference.
4. **A stronger hold at the prediction where the scans are blind** (D3's projection ratios). Evidence: D²-LIO, whose own
   tables show a small gain. Low expectation.

**Not supported as a setting:** a stricter `loop_closure_min_fitness`. In a self-similar passage a false match overlaps
as well as a true one; no source uses overlap alone as the gate, all add a degeneracy or saliency test (INFERRED from
3.1).

### 7.2 A new term or rule

| Rank | What | Size of the work | Evidence |
|---|---|---|---|
| 1 | **Loop edges that say nothing along the unobserved direction**: project the edge's information off the weak axis of its registration (Hinduja's Vf), and refuse an edge when both segments are blind along the passage and no surface across it was matched. | Small to medium: the edges already carry the registration's information and the time-support reading exists. | Hinduja 2019; DeCOD's 5-DoF against 6-DoF numbers; DARE-SLAM; LAMP 2.0. The strongest and most direct evidence in this survey. |
| 2 | **A speed from the steps along the weak axis** (section 2.4): step frequency, a step-length line calibrated for the user from well-conditioned stretches, one residual per moving window, off at rest and without a steady gait. | Medium: a detector, one term with its weight in the noise model, a calibration tool, tests on a synthetic gait. | Renaudin 2012 and Ho 2016 for the accuracy (2.5 to 6 % calibrated); ALIVE-LIO for applying a velocity only along the degenerate direction and for labels from well-conditioned stretches; P-LINS exists but was not read. No paper shows this exact term with numbers. Limits: flat walking only. |
| 3 | **Loops from structures across the passage only** (door frames, niche edges), matched along the axis: the idea of DeCOD and of the existing fine-time step. | Medium; much of it exists in `loop_fine_time`. | DeCOD: 95.6 % precision at 31 % recall on tunnel rings, with the caveat that few loops leave large errors (S9). The design's M3 warning about evenly spaced door frames applies. |
| 4 | **Relief inside the planes** (BIEVR-LIO's bump image per cell). | Large: a new map content and term; rooms must keep their accuracy. | BIEVR-LIO on a Livox Avia in shield tunnels. Fits a cave wall better than a painted corridor. Its authors name sparse scans and smooth walls as limits. |
| 5 | **A learned velocity** (ALIVE-LIO's network, AirIO training). | Large, and it needs hours of data from this device and carry; GPL code; a GPU runtime in the published form. | Mixed: better than FAST-LIO2 on GEODE shield tunnels, far worse than a camera-aided method on its own hand-held tunnel. RoNIN and Super Odometry 2.0 both report a clear loss on unseen users or devices. Belongs to the planned learned-tuner work, not to this change. |
| 6 | **A photometric term on reflectivity.** | Large. | Against it: the manual's 2 m warning, the need for image-like scans. For it: one unread 2026 paper. A cheap check first: whether the recorded reflectivity of door frames repeats between the way out and the way back. |

### 7.3 A change in how recordings are made

1. **Hold the sensor so that its band of view lies along the passage** (upright, or the commercial 20° tilt). Evidence:
   the manual's field of view; Emesent's "see both where it has been and where it is going"; the tunnel results read
   were made that way, but for one made facing the wall on purpose. Not tested on a Mid-360 in any source: the design's open question (a second recording with the
   other mount) would be the first such comparison, and it is the cheapest second test (section 6).
2. **A reference along the passage.** Paced or taped length; or marks at known spacing. Retro-reflective marks are the
   one thing the Mid-360's reflectivity is documented to tell apart at close range (manual), so they could later serve
   as landmarks as well as a measure. Evidence for the practice: DGV-DLIVO's 2 m marks, the cave paper's survey
   stations, ISPRS 2026's control scans (0.75 m to 0.08 m).
3. **Short scans, and a stop at each end.** Evidence: Emesent and FARO guidance; ISPRS 2026 (the far end is where an
   out-and-back cannot be repaired). The rests at both ends of this recording already do the second.

### 7.4 What the literature says will not work

- **Better detection or a different weight alone.** Detection-only methods hold the blind direction at the prediction;
  the prediction is the IMU. With a Mid-360 in a self-similar tunnel two such systems covered 10 % and 3 % of the travel
  (DGV-DLIVO). D²-LIO and LODESTAR state the limit themselves. A settings file can spread the error, as D6 says.
- **Point-to-point terms to "create" stiffness along the wall** (GenZ-ICP style): it diverged as a module in a
  LiDAR-inertial filter (LODESTAR's ablation), and it is the false stiffness the earlier survey found in the NDT cells.
- **A robust back-end as the only defence in a self-similar passage.** PCM, ICM and the match check test agreement;
  false matches of a repeating corridor agree with each other and with a loosened odometry. The systems that work gate
  before the back-end (3.1, 3.2).
- **Full 6-DoF loop factors from tunnel registrations** (DeCOD's ablation).
- **An out-and-back loop as the repair of the far end** (ISPRS 2026).
- **Reflectivity images from this sensor at this range** (manual; COIN-LIO's stated need).
- **A learned inertial model taken from another device or person** (RoNIN's unseen-subject and cross-data-set numbers;
  Super Odometry 2.0's and TLIO's stated limits).
- **Entropy or plane-variance scores as the judge of such a map** (MapEval): they are local, and a folded corridor is
  locally sharp.

## What could not be found or read

- Full text: the Electronics 2026 intensity paper (Mid-360), P-LINS (the pedestrian constraint), the Research Square
  mining-tunnel preprint, Tunnel-LIVO, Zlot & Bosse 2014, the Carleton thesis. All are marked ABSTRACT.
- A comparison of Mid-360 mounting orientations in tunnels or caves; a forum thread on it.
- A paper that fuses a step-frequency speed into a 3D LiDAR-inertial odometry for hand-held use, with numbers.
- The error of a hand-held, cadence-only speed for one calibrated user at a steady pace.
- A published consistency measure made for out-and-back passages.
- IMU-only bridging accuracy over tens of seconds for a consumer IMU, as a measured figure.
