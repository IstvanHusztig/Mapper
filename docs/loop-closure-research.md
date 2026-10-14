# Loop closure: diagnosis and practice in LiDAR SLAM (research notes, 2026-10-14)

Input for change G (loop closure) of the code-review fix plan (`docs/code-review-tracker.md`, findings C22–C25). Sections 1–2 are measurements on recording 44; section 3 is a literature and open-source survey (a research agent, checked against the sources listed at the end); section 4 is what follows for dMapper. Statements marked *(inference)* are reasoning, not taken from a source.

## 1. dMapper's loop closure today (`src/application/loop_closure.cpp`)

| Part | Today | Finding |
|---|---|---|
| Segments | 12 windows (≈ 4.8 s); anchor = the segment's **first** node pose; correction interpolated in time between anchors | C25 |
| Candidates (same floor) | earlier segments whose end lies ≥ `loop_closure_min_time_gap_s` = **30 s** before this segment's begin, within 8 m horizontally, nearest first; up to 3 tried | C22 |
| Candidates (cross-floor) | gap ≥ 5 s, cropped to the shared footprint (the shaft) | – |
| Registration | coarse yaw/translation search on voxel overlap, then NDT refinement (`ndt_refine`) | – |
| Acceptance | overlap ≥ 0.5 and not worse than before; centre shift ≤ 7 m; angle ≤ 15° (cross-floor: tighter) | C9 (fixed) |
| Edges per segment | **one** per pass (the first accepted candidate) | C24 |
| Loop edge weights | same floor: **fixed** 5 cm / 0.3°; cross-floor: the registration's information (height freed) | C23 |
| Odometry edge weights | **fixed** 0.1 m / 0.5° per segment, whatever was walked or how degenerate the segment was | C23 |
| Outliers | worst edge dropped while its residual exceeds 0.15 m / 2°; Cauchy kernel available but **off** (`loop_closure_kernel_scale = 0`) | C24 |
| Solver | Levenberg–Marquardt with a cost check over Tait–Bryan segment poses (sound) | – |

## 2. Diagnosis on recording 44 (C10 state, s0–s5)

- 31–39 loops accepted per run (≈ 20 same floor, ≈ 15 cross-floor); 1–5 dropped. The counts don't differ between passing and failing runs.
- **No loop touches the gallery.** The gallery is a dead end walked out (≈ 45–58 s) and back (≈ 59–70 s): its out/back segments are only 10–25 s apart, under the 30 s gap, so they are never compared. Loops jump from the corridor before it (segment 7, 34 s) to the corridor after it (segment 15, 73 s). Consequences:
  - its doubled walls (out vs back) are left to the odometry;
  - its heading is set by how the pose graph spreads the cycle error over segments 8–14 (evenly, because the odometry edges are all equal), not by where the drift happened.
- The loops at the end of the recording (start room ↔ return, 218–280 s) correct 2.9 m in s4 but 0.7 m in s1: the odometry drift differs, and the even spread puts the correction in different places, which fits the reshuffled façade pass/fail across s0–s5.
- Experiment (settings only, from the C10 snapshots): `loop_closure_min_time_gap_s = 8` on s0–s5 — result: *pending, to be filled in*.

## 3. Practice in established systems

### 3.1 Candidate gating
- **Time gap:** LIO-SAM (15 m radius, `historyKeyframeSearchTimeDiff` = 30 s). The odd one out: it rejects short out-and-back trips and admits standing still.
- **Path or index gap:**
  - SC-PGO (FAST_LIO_SLAM): excludes recent keyframes by index; its header comment notes that a distance-based exclusion works as well.
  - hdl_graph_slam: travelled-path gap `accum_distance_thresh` (8 m in the code default, 25 m in the launch file), XY radius 15 m, 15 m of path between accepted loops.
  - LAMP 2.0 (mines and caves): search radius scales with the path, d_max = 0.2 m × node-index gap with nodes every 2 m (≈ 10 % of the path travelled).
- **No gap, overlap only:** GLIM (a factor for every submap pair within 100 m with overlap ≥ 0.2, neighbours included); KISS-SLAM (all earlier local maps, split by distance, 40 % overlap).

### 3.2 Loop edge weighting (practice disagrees)
- **Fixed or scalar:** Cartographer (fixed weights, translation 1.1e4, rotation 1e5); SC-PGO (fixed variance 0.5 with a Cauchy kernel); LIO-SAM (isotropic variance = ICP fitness score); hdl_graph_slam (fitness mapped to an isotropic sigma, clamped to 0.1–5 m and 0.05–0.2 rad).
- **From the registration:** MULLS (I = AᵀPA / σ̂², the Hessian over the posterior residual variance); GLIM (GICP Hessian for between factors, the full VGICP cost as its loop factor); HBA (bundle-adjustment Hessian).
- **Pitfalls of Hessian weights:** closed-form ICP covariances are over-optimistic (Bonnabel 2016); their error is dominated by initialization and convergence, not sensor noise (Brossard 2020); points are not independent, so the raw Hessian gives millimetre sigmas; degenerate directions need care (Hinduja 2019 constrains only well-conditioned directions; DARE-SLAM keeps degenerate scans out of the loop search; LAMP 2.0 avoids closing loops in degenerate areas).

### 3.3 Odometry edge covariances
- LIO-SAM and SC-PGO: fixed per keyframe (1 cm, 0.057°; variances 1e-4 m², 1e-6 rad²), but keyframes come every 1–2 m or 0.2 rad, so the covariance effectively grows with distance, not time.
- Cartographer: fixed weights per node, nodes thinned by a motion filter. GLIM: IMU preintegration plus registration Hessians. HBA: BA Hessians.
- Least squares spreads a loop's residual along the cycle in proportion to each edge's covariance (standard property). With fixed sigmas per 4.8 s, a stationary segment absorbs as much correction as a walking one, and a degenerate segment no more than a well-constrained one *(inference)*.

### 3.4 Outliers (20–40 loops)
- No kernel: LIO-SAM (relies on the gates). Huber: hdl_graph_slam, Cartographer (scale 10); convex, so one bad loop still pulls. Cauchy: SC-PGO. DCS and switchable constraints: older robust back-ends.
- **PCM** (pairwise consistency maximization, Mangelson 2018; Kimera-RPGO): the maximum clique of pairwise-consistent loops. LAMP adds an odometry-cycle check (reject when the mean error per edge exceeds 0.1 m or 0.05 rad).
- **GNC-TLS** (Yang 2020): needs no initial guess, tolerates 70–80 % outliers. Kimera2 found it better than PCM on most sequences, worse on some. LAMP 2.0: incremental PCM-style prefilter, then GNC.
- PCM's χ² test needs honest covariances; LAMP fell back to fixed thresholds without them.

### 3.5 Anchors and dense correction
- GLIM: submap origin at the centre frame. LIO-SAM / SC-PGO: target = the candidate keyframe ±25 keyframes, centred on it.
- Cartographer, LIO-SAM: every node or keyframe is in the graph (no interpolation). KISS-SLAM: a second PGO over the dense local trajectories with the keyposes fixed. HBA: top-down PGO over all frames. MULLS: each submap's reference frame fixed, frames inside optimised.
- A start anchor couples a rotation error into translation over the lever arm to the points; a diagonal fixed covariance ignores that coupling *(inference)*.

## 4. What follows for change G (proposal, to be designed and measured)

In order; each step measured from the snapshots on s0–s5 against the C10 noise floor before the next:

1. **G1 — path gap instead of time gap (C22):** a candidate needs ≥ ≈ 10 m of path between the two segments (stationary segments add nothing), radius ≤ min(8 m, 0.1–0.2 × path) in the spirit of LAMP. Lets dead ends walked out and back through. Expected: the gallery's doubled walls; its heading still comes from the junction *(inference)*.
2. **G2 — candidate choice and redundancy (C24):** the best of the passing candidates (by fitness), not the first nearest; 2–3 loop edges per segment. Expected: fewer discrete flips between near-identical inputs *(inference)*.
3. **G3 — honest weights (C23):** odometry edges with translation variance σ0² + k_d·distance and rotation variance σr0² + k_t·Δt + k_ψ·|Δψ|, inflated along the window's weak directions (the degeneracy information exists); same-floor loop edges with the registration information H / σ̂², inflated by a factor calibrated from repeated registrations from perturbed starts, floored (≈ 2 cm, 0.1°), near zero along weak eigen-directions. Calibrate the k values from accepted loops' residuals against path length on 44.
4. **G4 — outliers (C24):** odometry-cycle and pairwise consistency checks (PCM; an exact maximum clique is cheap for ≈ 40 loops), then GNC-TLS with the odometry edges as known inliers, replacing "plain least squares, then kernel, then drop the worst edge". Needs G3 first.
5. **G5 — anchors and dense correction (C25):** anchor at the segment's middle node (or point centroid); correction onto the nodes by path length, or a second small pose graph with the anchors fixed.
6. Later, if doubled walls remain after the pose graph: plane bundle adjustment (BALM2 / HBA) as a final refinement (precision first, costs time).

Open: the gallery's absolute heading (façade angle) needs a constraint from the start, not only consistency; whether one exists in the data (e.g. the façade seen through the start room's windows, 10 m away, outside today's 8 m radius) is to be checked.

## Sources

- LIO-SAM: https://github.com/TixiaoShan/LIO-SAM (`config/params.yaml`, `src/mapOptmization.cpp`)
- SC-A-LOAM / SC-PGO: https://github.com/gisbi-kim/SC-A-LOAM (`laserPosegraphOptimization.cpp`, `Scancontext.h`); FAST_LIO_SLAM: https://github.com/gisbi-kim/FAST_LIO_SLAM
- hdl_graph_slam: https://github.com/koide3/hdl_graph_slam (`loop_detector.hpp`, `information_matrix_calculator.cpp`, `hdl_graph_slam_400.launch`)
- Cartographer: https://github.com/cartographer-project/cartographer/blob/master/configuration_files/pose_graph.lua
- GLIM: https://github.com/koide3/glim (`config_global_mapping_gpu.json`, `sub_mapping.cpp`, `global_mapping.cpp`)
- KISS-SLAM: https://arxiv.org/abs/2503.12660
- MULLS: https://arxiv.org/abs/2102.03771
- HBA: https://arxiv.org/abs/2209.11939; BALM2: https://arxiv.org/abs/2209.08854
- LAMP: https://arxiv.org/abs/2003.01744; LAMP 2.0: https://arxiv.org/abs/2205.13135
- Kimera-RPGO: https://github.com/MIT-SPARK/Kimera-RPGO; Kimera2: https://arxiv.org/abs/2401.06323
- GNC: https://arxiv.org/abs/1909.08605; PCM: https://robots.engin.umich.edu/publications/jmangelson-2018a.pdf
- ICP covariance: https://arxiv.org/abs/1410.7632 (Bonnabel 2016), https://arxiv.org/abs/1909.05722 (Brossard 2020)
- Degeneracy: https://www.cs.cmu.edu/~kaess/pub/Hinduja19iros.html; DARE-SLAM: https://arxiv.org/abs/2102.05117
- DCS: https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/agarwal13icra.pdf
- Not checked: FAST-LIO-LC and maplab specifics. DLO and DLIO have no loop closure.
