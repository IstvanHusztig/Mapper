# Tasks

**Runs:** full corrected 44, precise Release build, from copies of the binaries. Up to three runs in parallel on copies of the input folder (no window may come near `real_time_threshold_seconds`; checked per run). The copies are deleted at the end. Each run is reported with loops and corrections, route, the difference from `44_baseline`, `eval_odometry.py` and the time.

## 1. Rotation and accelerometer (batch 1)

- [x] 1.1 `rotation_sigma_inflation_k` 2 and 1 (shipped 4), and the accelerometer term off (`use_imu_acceleration_term = false`). Record here.

  **Result (2026-10-09).** On the corrected 44, a single run can't pick settings: the noise of one run is larger than every effect.
  - **Noise test:** each candidate re-run with the IMU shifted by +1 and +2 ms, a physically meaningless change:

    | Setting | Loops | Largest correction | 3 cm cells | Window yaw diff |
    |---|---|---|---|---|
    | shipped (k = 4) | 11 / 21 / 10 | 5.56 / 0.79 / 6.29 m | 24,485 / 24,239 / 24,409 | 0.87 / 0.84 / 0.78° |
    | k = 1 | 20 / 15 / 11 | 0.69 / 3.89 / 5.07 m | 24,464 / 26,684 / 23,957 | 0.78 / 0.75 / 0.75° |
    | accelerometer off | 18 / 11 / 13 | 4.10 / 6.49 / 5.05 m | 21,658 / 24,441 / 24,226 | 0.85 / 0.81 / 0.83° |

  - **Single runs, no shift:** k = 2: 13 loops, 4.89 m; k = 0.5: 14 loops, 8.75 m; k = 0.25: 18 loops, 2.01 m, 27,166 cells; k = 1 with the accelerometer off: 16 loops, 2.98 m, 21,459 cells.
  - **New synchronized recording 45f** (dLidar fix, residual offset x −1.4 / z +1.9 ms): shipped, k = 1, accelerometer off and both are equal within the noise (17–18 loops, 0.59–0.75 m, cells within ±1 %).
  - **Conclusion:** no candidate is better than the shipped settings beyond the noise. k = 1 only agrees slightly better with the gyro (0.75–0.78° against 0.78–0.87°). The two apparent wins, k = 1 on drift and the accelerometer off on sharpness, were single lucky runs.
  - **The real finding:** on the corrected 44, loop closure is chaotic. The largest correction ranges from 0.8 to 6.3 m for 1–2 ms changes, and most runs leave the baseline at the same place, t ≈ 74–78 s (the two-solution place noted in `hessian-degeneracy-detection`). From now on, settings are judged by at least three perturbed runs (median) and on 45f. That place in 44 is the precision problem to look at.

## 2. Follow-up (batch 2)

- [ ] 2.1 From batch 1's best: the accelerometer gate's sigma (0.2 m/s²) or the degenerate translation sigma, and the gyro-bias random walk (0.002 °/s/√s), as batch 1 suggests. Record here.

## 3. Ship

- [ ] 3.1 The chosen settings once with the `/fp:fast` build (the difference between the builds at most a few decimetres, as today).
- [ ] 3.2 New `settings/l2-vertical.toml`, new baselines (the old folders renamed with a date suffix, manifests rewritten), the settings tables in `docs/index.html` and `docs/hu/index.html`. Archive, push, memory notes.

## 4. Re-check what was decided on the lagging data (after the retune, each its own change or task)

Everything below was measured or rejected on recordings whose IMU lagged the LiDAR by 50 to 80 ms, with settings tuned around that lag. A "worse" there may be "better" now, and a "better" may have been a compensation. Each item is re-run on the retuned settings and the corrected 44 (and on the user's new synchronized scans, as soon as there are some), with the same measures as in task 1.

- [x] 4.1 **NDT cell statistics.** Exact cell statistics (`use_exact_ndt_bucket_statistics`) against the default "previous" statistics. Measured on lagging data in `odometry-stall-fix`: 2.8 and 8 m off. Also the in-plane regularization of planar cells (`ndt_plane_in_plane_sigma_m`, 8.8 m off) and the archived `ndt-bucket-statistics` change (rejected 2026-10-03). The question is whether the cell statistics decide ghost walls and drift, or whether the lag did. The NDT map is built from poses that the lagging gyro and the lag-compensating rotation weights shaped.
  **Done 2026-10-09, both rejected again, now on synchronized data** (shipped settings with `imu_always_on` and loop-closure gating; three 44 runs with the IMU shifted 0/1/2 ms plus 45f; reference `44_lc_s0..s2`, `45f_lc`):

  | variant | 44 loops | 44 largest correction | 44 route | 45f loops / largest correction | first-chunk cells 3 cm, 44 / 45f |
  |---|---|---|---|---|---|
  | shipped | 23 / 21 / 21 | 1.72 / 1.86 / 1.92 m | 404 / 402 / 419 m | 21 / 0.20 m | 26,373–26,910 / 12,345 |
  | exact statistics | 0 / 0 / 0 | – | 617 / 673 / 674 m (runaway, 6,000–15,500 jump warnings) | 21 / 0.24 m | – / 12,224 |
  | in-plane sigma 0.3 | 10 / 11 / 6 | 1.98 / 0.12 / 2.91 m | 408 / 410 / 567 m (runaway in s2) | 10 / 1.99 m | 24,777–24,829 / 12,283 |

  - Exact statistics drift from the start in all three runs (heading +3° by 15 s, 1 m off by 45 s, runaway at the stairwell). Likely cause: cells that keep averaging in new points follow the odometry's drift, while the default cells keep their first view and anchor the map. On 45f neutral (1 % sharper, the same 23 cm largest revisit offset).
  - The in-plane sigma makes the first chunk 4–8 % sharper but loses half the loops and runs away once.
  - So the lag did not decide these; the cell statistics stay as shipped.
- **The spot in 44 (found 2026-10-09):** the bad runs are stalls in blind passages. Run A (shipped, no shift) and run B (+1 ms) were compared odometry-only with `log_window_diagnostics`:
  - they separate gradually (1 cm at window 52, 5 cm at 120), then sharply in the passage at windows 171–186 (t ≈ 69–76 s, weak direction ≈ (0.96, 0.26, 0));
  - **in A, window 172 reverses the motion** (+0.07 m against a guess of −0.12). Its information ratio was 0.237, just above the projection's 0.2, so it wasn't projected. The carried speed then stays near zero for 15 windows, about 6 m of walking lost, and the solver doesn't converge (step 1e-5 to 1e-4 after 100 iterations);
  - **B** had a ratio of 0.128 in that window (projected), walked on at 0.4 m per window and converged.

  A stall detector (trajectory speed < 0.25 m/s over 1 s for ≥ 1.2 s while the IMU's step rhythm shows walking) finds:
  - a stall at 70–73 s in most runs with bad loop closure, none in any run with good loop closure, and one stall at 125 s (the original ghost-wall passage);
  - in 45f, the same short stalls in every variant, probably real turns on the spot.

  Widening the projection (0.15/0.3, 0.2/0.4) didn't stabilise 44 (8–14 loops). Cause: in a blind passage the speed is unobservable (the LiDAR is blind along the axis, the accelerometer sees only changes), so one wrong window kills the carried speed. The principled fix is 4.3: velocity as a carried state that only the accelerometer may change. Judge it by stalls and loops over three perturbed runs and on 45f.
- [ ] 4.2 **Degeneracy projection** (`degenerate_projection_ratio` 0.1 / `_full` 0.2): the thresholds were chosen for the ghost wall of 44 on lagging data. Check the thresholds and the information-ratio gate (0.3) again.
- [x] 4.3 **Accelerometer on its own:** the gate, its sigma and the continuous weight variant (measured harmful, 4.4 m off), then IMU always on (branch `imu-always-on`; on corrected data v3 was 1.0 m off against 22 m).
  **Done 2026-10-09:** IMU always on shipped (archived `imu-always-on`): no stalls in three perturbed 44 runs, 45f largest loop correction 0.68 -> 0.22 m. The gate stays only for `imu_always_on = false`. Open: a `/fp:fast` build loses a window at the top of the 44 stairwell (t ≈ 86 s, 6 m apart at the end); see that change's task 3.1.
- [ ] 4.4 **Hessian signal-to-noise degeneracy test and its rotation-only variant** (rejected 2026-10-09, in git history at the implementation commit named in the archived change): the test handed rotations to the gyro, which was 80 ms late.
- [ ] 4.5 **The three switched-off options of `odometry-stall-fix`** (`ndt_plane_in_plane_sigma_m`, `imu_degenerate_weight_ratio_full` with its smoothing, `imu_acceleration_offset_freeze_blind`): decide after 4.1 and 4.3 whether to remove them.
  - **2026-10-11, `ndt_plane_in_plane_sigma_m` removed** with the rejected change `point-to-plane-ndt`: a pure point-to-plane term in planar cells (n nᵀ/σₙ², σₙ ≥ 1 cm) ran away on `44_s0` and `44_s1` at the stairwell (window 216, ~87 s, speed up to 6.3 m/s; route 743 m against 455 m, 2–3 loops against 39). The near-point-to-plane in-plane sigma it would have superseded goes with it.
- [x] 4.6 **Loop-closure gating** (chi-square, GNC, PCM): last, because the corrections it judges depend on all of the above.
  **Done 2026-10-09 (a first, simple gating):** archived `loop-closure-gating`: retry the next partner after a rejection, drop the worst edge while its residual exceeds 0.15 m / 2° (iterative, like a one-at-a-time PCM). Ghost walls 51–62 -> 19–25 cm horizontally on 44. Chi-square/GNC can follow if a recording shows a false loop that this misses.
- [ ] 4.7 **Gyro and ARW constants:** the gyro noise values came from a static recording and don't depend on timing, but `rotation_sigma_floor_deg`, `ahrs_gain` and the turn weighting were tuned on lagging data too.
- [ ] 4.8 **The stairwell window of 44 (diagnosed 2026-10-10, not fixed).** A `/fp:fast` build of the shipped code runs away at t ≈ 86 s; the precise build doesn't. Measured with temporary per-iteration logging of the NDT cost (sum of w · rᵀ Σ⁻¹ r over the matched points) and the matched count:
  - the builds part at **window 212**, from nearly identical poses, velocities and bias: final cost per matched point 12.8 (fast) against 5.3 (precise). In window 213 the fast build starts at 3.3× the cost (57.6 against 17.9 per point);
  - **cause 1, the map amplifies rounding:** with the default statistics a cell's mean is the first point that fell into it and never moves (`ndt_map.cpp`). A pose a fraction of a millimetre different moves points across cell borders, so new cells (the stairwell is new ground in windows 208–212) get other means, anywhere within the 0.15 m cell, and the next window matches against a different map. This is also a likely source of the run-to-run chaos of 44 (1–2 ms IMU shifts change loops and corrections by metres);
  - **cause 2, points may escape:** a point without a cell costs nothing. In the fast build's window 213 the cost falls (1.46 → 1.16 million) only because the matched points fall from 25,300 to 19,800 while the window slides 0.3 m sideways and 15 cm up the stairs, about 1 cm per iteration until the damping schedule freezes it (the optimizer checks no cost and raises the damping ×10 every 10 iterations);
  - tried earlier and removed: not handing on the window's velocity, re-solving it with the gyro or the accelerometer trusted more. Exact statistics (running mean) remove cause 1 but let the cells follow the drift (4.1).
  - **Fix candidates** (each its own change, judged on the three perturbed 44 runs, 45f and a `/fp:fast` build): a cell mean that settles after its first N points (stable anchor, not first-point-sensitive); a cost for points that leave the map (or the matched share as an acceptance check), with a real Levenberg–Marquardt step check on that cost.
  - **Candidate 1 tried 2026-10-10, rejected and removed:** a cell's mean and normal from its first N points, then fixed (`ndt_cell_settle_points`). N = 20 got the fast build through window 213 (accelerometer misfit 0.11–0.26 m/s²), but a full fast run still ran away later (27,271 jump warnings, 6 loops, 73 m off the precise run). Precise runs, largest revisit offset / loop correction: N = 20: 10 / 44 / 45 cm, 3.5 / 5.4 / 4.0 m, 45f 31 cm; N = 8: 26 / 37 / 42 cm, 3.3 / 3.2 / 3.0 m, 45f 24 cm; shipped: 35 / 25 / 19 cm, 1.7 / 1.9 / 1.9 m, 45f 23 cm. More drift, no gain. Next: candidate 2.
  - **Candidate 2 tried 2026-10-10, promising (uncommitted, both settings off by default):** `lm_step_check` (each call returns the window's full cost, `ScanToMapCost`: NDT r^T Sigma^-1 r per matched point plus every motion and IMU term; a step that raises it is taken back, damping x10, the last step checked too) and `ndt_unmatched_point_cost` (a cost per point in range that no cell matches). On `44_short` every window rejects about 8 steps, mostly near convergence; 130 s against 120 s. The `/fp:fast` build no longer runs away: full 44 with unmatched cost 0 / 16 / 50: 0 jump warnings, 22 / 21 / 22 loops, largest correction 1.63 / 2.14 / 1.60 m, 0.37 / 0.66 / 0.67 m from the precise baseline (before: runaway at window 213; candidate 1: 73 m off). Precise, cost 16, `44_s0`: 21 loops, 1.78 m, largest revisit offset 16.7 cm (shipped on the same input: 23 loops, 1.72 m, 34.6 cm). Still to run: `44_s1`, `44_s2`, `45f_p` with candidate 2 alone, then pick the unmatched cost (0 would mean the step check alone).
  - **Candidate 1 on top of candidate 2 (N = 20, cost 16), rejected 2026-10-10:** `44_s0` 5 loops and 50 m off, `44_s1` 18 loops with a 5.56 m correction, fast 44 15 loops / 3.09 m / 5.4 m off; only 45f was fine (18 loops, 0.19 m). The drift of settled cell means doesn't come from points escaping; settling stays rejected (patch removed again).
  - **Wave 3 (2026-10-10), candidate 2 (cost 16) against shipped, largest revisit offset:** `44_s0` 16.7 / 34.6 cm, `44_s1` 30.5 / 24.7, `44_s2` 33.0 / 18.7, `45f_p` 36.5 / 22.8; loops 21–22, largest correction 1.78–2.05 m (shipped 1.72–1.92). Cost 0 ran away in the precise build on `44_s0` (801 m, no loops; unmatched points 25 % → 50 % at window 150). So: robustness for the fast build, no precision gain; undecided.
  - **Doubled railings (user, 2026-10-10):** the stairwell railings are drawn twice on each floor in both `44_lm16_s0` and `44_lc_s0`, invisible to `ghost.py` (planar voxels only, revisits ≥ 40 s). `spot.py` at (−3.01, −5.42, −0.12): floor-0 passes agree within 2–5 cm; seen from floor −1 (139–149 s) 24 cm (shipped) / 9 cm (candidate 2) off, from floor −2 (176–181 s) 27 cm. Cause: loop closure never links floors (vertical limit 1.5 m).
  - **Cross-floor loop prototype** (second candidate pass 1.5–6 m, ≤ 1 m / 3°, full 6-DoF edge; on `44_s0`): the railing spot is fixed (floor −1: 2.0–2.6 cm; floor −2: 3–5.5 cm with candidate 2, 21.8 cm on the shipped odometry). With candidate 2 it bends floor −2 (revisit offsets 43.7 cm at fitness 0.3, 116.9 cm at 0.2); on the shipped odometry max 17.7 cm (shipped 34.6). The accepted rotations sit at 1–2.8° (round shaft). Continued as change `robust-loop-closure` (kernel + translation-only cross-floor edges); point-to-plane NDT proposed as `point-to-plane-ndt`.
  - **2026-10-11:** point-to-plane NDT (change `point-to-plane-ndt`, rejected) loses this window too, in the precise build: without the in-plane pull the windows on the stairs slide (window 216 onward). The in-plane pull is what holds the stairwell window; point-to-plane is not the fix for 4.8.
