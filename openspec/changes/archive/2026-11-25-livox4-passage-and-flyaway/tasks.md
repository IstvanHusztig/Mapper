# Tasks

> **Closed 2026-11-25 (housekeeping; the user: "all as suggested"). Its results ship (measured gyro noise, the 3.12 candidate set d6e8860, gyro scale, honest-inertial-carry layer 1); its decisions became their own changes. "Open, for a later look" below stays as the record.**

## 1. Captures

- [x] 1.1 Full Livox 4 run with the shipped settings, every per-window log on, capture 95–125 s
  (`experiments/stage8/run_passage`). Verify: the capture exists; the run's verdict summary equals the stage 4 run's.
  Result: `experiments/stage8/run_passage` at 8a14329: exit 0, wall 2206 s, peak 5.8 GB, capture 513 MB (windows
  265-348). Its `window verdict summary` lines are identical to the stage 4 run's.
- [x] 1.2 Full Livox 4 run, capture 195–285 s (`experiments/stage8/run_flyaway`; loop closure and consistency off).
  Verify: the capture exists.
  Result: `experiments/stage8/run_flyaway`: exit 0, wall 1095 s, peak 5.7 GB, capture 1257 MB (windows 543-792).
- [x] 1.3 Both captures replay identically with their own settings (`tools/run_replay.py`).
  Result: passage 84 of 84 windows identical (202 s, peak 1.5 GB); flyaway 250 of 250 identical (397 s, peak 3.5 GB);
  no window at the time limit.

## 2. Reading

- [x] 2.1 The passage (100–110 s): which term moves the windows, hold and pull per term, which term gave way.
  Result: path 100-110 s 6.47 m (true about 10 m). Windows 288-296 (103.3-106.2 s): speed 1.1 -> 0.15 m/s, each
  result 4-5 cm shorter than its guess. Window 289 at the result, stretch along travel (hold / pull):
  `ndt_unprojected` 1.6 mm / -37 mm, `ndt_indoor` 2.8 / -42, `ndt_outdoor` 5.7 / -121, `velocity_prior` 2.2 / +34
  (cost per residual 78.9), `edge_position` 4.2 / +41; sideways the NDT holds 0.26-0.31 mm with pulls below 1 mm.
  Over windows 286-297 the speed along travel falls by 0.85 m/s at the window boundaries (next start velocity
  against this end velocity) and by 0.66 m/s inside the windows. At 107.3-108.0 s two windows are turned 6.2 and
  7.5 deg against the gyro (matched 17,000, unmatched 19,000); the gyro bias z estimate jumps from about 0 to
  0.23 deg/s at 107.6 s.
- [x] 2.2 The flyaway (onset 205–210 s, then 210–285 s): the same.
  Result: onset 205.9-207.0 s (windows 573-576): speed 0.2 m/s, 17,000-20,000 matched points; roll and pitch bends
  0.9-2.5 deg per window (window 575 at the guess: `ndt_unprojected` pulls roll -1.47, pitch +1.68, yaw -1.92 deg
  with holds 0.037-0.050 deg; `edge_rotation` holds 0.12-0.22 deg). From window 576 to 586 the end velocity along
  +y grows 0.47 -> 5.32 m/s (about 0.5 m/s per window, uneven), vertical 0 -> -1.5 m/s; the acceleration offset
  stays below 0.005 m/s2; the gyro bias estimate goes 0.6 -> 5.1 deg/s by 250 s. 207-285 s: -72.9 m, largest
  speed 8.5 m/s, 87 windows bent over 3 deg. The route there goes down stairs (the user: two flights with a 90
  deg turn, about 3 m per floor, not measured). Livox 3 goes down stairs too (108-161 s, -5.6 m) with bends up
  to 0.83 deg and a bias of 0.05-0.11 deg/s; Livox 4 reaches its stairs with a bias estimate of 0.3 (133-138 s),
  0.85 (first stairs, one window bent 13.9 deg) and about 0.7 deg/s (before 198 s). Different routes: not a
  controlled comparison. A revisit of the area mapped before the passage is not what starts it (nearest earlier
  position 7 m away, another level).
- [x] 2.3 The velocity chi² question carried from stage 4: what the carried velocity's hold is made of.
  Result: the velocity prior holds a window's length to 1.6-2.6 mm (about 5-7 mm/s) in the passage windows: the
  previous window's posterior, in which the NDT is the largest part, grown only by the accelerometer noise over
  the gap. In windows 288-292 it is overruled at chi2 100-370. With the measured gyro noise the chi2 median of
  110-124 s falls from 1339 to 40. What its honest size is, is open (below).

## 3. Experiments

- [x] 3.1 Region job lidar-matching: which part of the NDT term pulls the passage windows back (normal / in-plane,
  ahead / behind, per grid). Ownership check, verification, conclusion review.
  Result: branch `region/lidar-matching/2026-11-18-passage-pull`, commit 0981283 (rows `ndt_normal`, `ndt_in_plane` of
  the per-term table; no result changes), merged 7a9d031. Windows 288-292 at the result: the in-plane part has
  67-84 % of the NDT's stretch-along weight and pulls -33 to -43 mm, the normal part -12 to +3.5 mm; calm windows
  270-280: normal 82 %; windows 305-309: normal -49 mm (47 %), in-plane -26 mm. 94 % of the weight in 288-294 is
  from points ahead of their node (the hypothesis 'cells behind the sensor' is contradicted). Ownership check: 2
  own, 1 shared test file, 2 outside (`window_term_table.{h,cpp}`, granted). Step checks on the commit and on
  main after the merge: everything identical, `window_terms.csv` differs by the 84 added rows only (checked: the
  file without them equals the old reference); references recorded again from 7a9d031. The conclusion reviewer
  recomputed the numbers: they hold; its limits are in 4.1. No separate verification agent: the region agent's
  step checks ran on the commit (log in its folder) and were repeated on main.
- [x] 3.2 Replays of both captures with the gyro's measured noise (ARW 0.0094 / 0.0224 / 0.0144 °/√s, k 1, floor
  0.0002°) and with k 1 alone: distances, bends, speeds, the per-term table where a window still goes wrong.
  Result: passage capture, path 100-110 s / 110-124 s / largest bend: shipped 6.47 / 12.31 m / 7.5 deg; k 1: 6.02 /
  12.62 / 1.2; measured gyro: 6.04 / 12.41 / 0.2. Flyaway capture, largest speed / windows bent over 3 deg /
  height 207-285 s: shipped 8.5 m/s / 87 / -72.9 m; k 1: 5.5 / 0 / -6.1; measured gyro: 1.4 / 0 / -4.5 (208-215
  s -1.7 m, level, 228-238 s -3.0 m, level; the user: roughly the route). Livox 3 capture 100-130 s: largest
  difference to the shipped result 0.032 m / 0.76 deg (measured gyro), 0.012 m / 0.41 deg (k 1). One replay
  each, from the shipped run's map and state. Files: `experiments/stage8/{passage,flyaway,livox3}`, CSVs next to
  the captures; reading scripts in `experiments/stage8/tools`.
- [ ] 3.3 Full runs with the gyro's measured noise: Livox 4 (with a capture of 95-125 s), then Livox 3
  (`experiments/stage8/full_gyro_livox{4,3}`). Run check on both logs; the clouds for the user's look.
  So far (2026-11-18): Livox 4 `full_gyro_livox4`: 0 windows bent (largest 0.48 deg), 0 fast (largest 1.62 m/s),
  largest bias estimate 0.58 deg/s, end height -0.7 m (shipped -78 m), 66 loops with corrections up to 4.36 m,
  passage 5.94 m; run check: bend and speed CLEAR, bias and chi2 TRIGGER. The user's look: "a step in the right
  direction, far from perfect but overall a huge win"; what is still wrong: "generally rotation drifts".
  Livox 3 `full_gyro_livox3`: largest bend 0.30 deg, largest speed 1.24 m/s, bias 0.36 deg/s, start-to-end 0.07 m,
  81 loops (largest 0.22 m); its final trajectory differs from the shipped one by up to 0.53 m (196.5 s). The
  user's look at the Livox 3 cloud is open.
- [x] 3.4 Region job lidar-matching, experiment branch `region/lidar-matching/2026-11-18-in-plane-ablation` (not for
  merge): the in-plane part of the cell information scaled by 0.3 / 0.1 / 0 under two rules (A: everything but
  the normal direction; B: only directions in which the cell's points fill the voxel), replays of the passage,
  then the flyaway, Livox 3 and 44_short captures (the user, 2026-11-18: start it).
  Result: branch commit 0feae07 (keys `ndt_in_plane_rule`, `ndt_in_plane_gain`). Passage capture with the measured
  gyro, path 100-110 s: 6.04 m -> rule A 7.85 / 8.90 / 9.55 m at gain 0.3 / 0.1 / 0; rule B 6.72 / 7.26 / 8.00 m;
  rule A gain 0 with the shipped gyro weight 7.09 m (bends up to 6.3 deg). 110-124 s: 12.4 -> 13.1 m. At gain 0:
  flyaway capture heights within 0.1 m of the gyro-only replay (largest speed 1.65 / 1.52 against 1.38 m/s),
  Livox 3 capture moves 0.11 m / 1.1 deg, 44_short 0.02 m / 1.4-1.7 deg; no window ran away in a replay. With
  rule A gain 0 the windows 288-303 keep 0.9-1.15 m/s with guess and result within millimetres; the standstill
  at 109.8-110.5 s remains with all terms in agreement (chi2 5-41), and the user says slowing down there is
  very possible: taken as real. Full runs with it (`full_gyroA0_livox{4,3}`, experiment build): Livox 4 passage
  9.38 m, but from 329 s the speed runs to 2.4-4.2 m/s until the rest at 354 s (about 50 m of false path; the
  normal-only NDT pulls back 10-80 mm with a hold of 2.2-3.8 mm and loses to the carried velocity; a steady false
  acceleration of about 0.4 m/s2). So gain 0 is not a fix on its own.
- [x] 3.5 Full Livox 4 runs with the yaw gyro rate scaled by 1.004 and 1.008 (copies `raw data/livox/4_gz0040`,
  `4_gz0080`; measured gyro noise): does the bias estimate stop following the turn rate? Background: in both
  measured-gyro runs the yaw bias estimate follows the yaw rate (slope about -0.003, R2 0.20-0.25 on Livox 4,
  0.46-0.58 on Livox 3) and not the yaw acceleration (R2 0-0.02), so a time offset is not what it follows; the
  loop corrections against the net turn were reviewed and do NOT support a scale error (one loop repeated
  about 11 times dominates the fit).
  Result: `full_gyro_4_gz0040` (yaw rate x 1.004): the slope of the yaw bias estimate against the yaw rate goes from
  -0.0022 / -0.0028 / -0.0032 (smoothing 1 / 5 / 15 windows, R2 0.20-0.25) to -0.0006 / 0.0000 / +0.0022 (R2
  0.00-0.10): a scale mismatch of about 0.4 % (0.24-0.53 % by the smoothing), the gyro reading too little. Not
  fitted to it, the run also closes: start-to-end before loop closure 3.73 -> 0.77 m (height 0.70 -> 0.05 m),
  loop corrections' rotation median 2.1 -> 1.2 deg (last minute 3.8-4.0 -> 1.2-2.2 deg), 66 loops / 4.36 m ->
  84 loops / 2.76 m, velocity chi2 73 / 413 -> 58 / 245; passage unchanged (5.79 m). One run; whether it is the
  gyro's sensitivity or the IMU clock's rate cannot be told from these data. The 0.8 % run was cancelled (the
  machine was needed for the replays). The user's look at `full_gyro_4_gz0040/result` is open.
- [x] 3.6 A capture of the stretch that runs away without the in-plane part (315-362 s, experiment build, measured
  gyro, rule A gain 0): `experiments/stage8/runaway_capture`.
  Result: `experiments/stage8/runaway_capture/result/window_capture_315.0-362.0s.bin` (719 MB), settings
  `runaway_capture.toml`, experiment build 0feae07.
- [x] 3.8 Region job, experiment branch `region/lidar-matching/2026-11-18-rotation-prior-experiment` (not for merge):
  a prior with a sigma on node 0's rotation in place of the hard fix (`origin_rotation_prior_sigma_deg` 0.005 /
  0.02 / 0.08) and a gyro scale correction (`gyro_scale_correction_z` 0.004), alone and together, on the passage
  and Livox 3 captures, then on the stairs; then with the in-plane rule A at gain 0 / 0.03 / 0.1 on the runaway,
  passage and Livox 3 captures. Readings: the bias estimate's swing, the tilt the accelerometer sees, how far
  node 0 is turned, the speeds of the runaway stretch.
  Result: commit 5869043 (24 replays, none at the time limit) and four more of mine (`experiments/stage8/extra_rotation`).
  One replay per variant, measured gyro noise everywhere. Passage 100-110 s / stairs largest speed / end stretch
  largest speed walking, standing: gyro only 6.04 m / 1.38 m/s / (full run fine); in-plane removed 9.55 / 1.65 / 3.31,
  4.16; in-plane removed + scale 9.46 / 1.76 / 3.29, 4.16; in-plane removed + scale + prior 0.02 deg 9.05 / 2.58 /
  2.41, 0.67 (mean 0.12 m/s while standing); scale + prior (in-plane as shipped) 6.09 / 4.60 / -; prior alone 6.01 /
  4.96 / -. On the stairs the prior lets node 0's pitch be turned 0.5-0.85 deg per window (windows 689-691). The
  scale correction brings the yaw bias slope to -0.0004 (passage) and -0.0008 (Livox 3). Reviewed: the numbers hold;
  no replay has the prior alone on the end stretch or another prior size on the stairs, and the first staircase's
  path grows 36 % without the in-plane part (6.03 -> 8.19 m), which of the two is right is not known. No variant
  is clean on all three stretches.
- [x] 3.7 Research (two Sonnet passes; `experiments/stage8/research_rotation_and_inplane.md`, the second pass read
  the sources): VoxelMap, FAST-LIO2 and GICP match points to planes only (GICP: a point constrains only along
  its normal); FAST-LIO2 and VINS-Mono correct the attitude as a state, the window's oldest pose carries a prior
  with a covariance, not a hard fix; X-ICP and Zhang's remapping hand weak directions to the prediction, which
  nothing checks (the runaway is that design's known failure). Nothing found on a systematic pull of voxel
  matching along surfaces. Not read: Point-LIO, LIO-SAM, the gyro's datasheet (secondary: sensitivity +-0.5 %).

- [x] 3.9 Code review of the path behind the passage stall (master-reviewer, reading only), and one replay of its top
  finding.
  Result: no sign, frame or Jacobian defect. Three new findings in the default cell statistics: (1) a cell's mean is
  its first point for good (`ndt_map.cpp::new_legacy_cell`); with the sensor-frame key order of `decimate` (C30) and
  this mount's X axis pointing forward, that point is the cell's rearmost one along the walk, so later points lie ahead
  of the stored mean and pull the window back; (2) cells with 1 or 2 points pull with an isotropic 3 cm information,
  pass the view-point gate and are not counted by the degeneracy projection (`ndt_term.cpp:151`, `:233`); (3) the
  covariance about the fixed mean is weighted (n−1)/n² instead of 1/n. One replay of the passage capture with
  `use_exact_ndt_bucket_statistics = true` (measured gyro noise, `experiments/stage8/passage/p_gyro_exact`): path
  100–110 s 6.04 → 9.63 m (true about 10 m), velocity chi² median 123 → 16; 110–124 s 12.41 → 10.90 m with a largest
  speed of 2.07 m/s. The capture's map was built with the default statistics, so a full run has to confirm it.
- [x] 3.10 Region job, experiment branch (not for merge): (a) a reading per window of how far the stored mean of the
  matched cells lies behind the window's own points in them, along travel, by the cell's point count (1, 2, 3–9, 10 and
  more) and per grid; (b) a switch that keeps cells with fewer than 3 points out of the matching. Replays with the
  measured gyro noise on the four captures (passage, stairs, Livox 3, end stretch): default statistics, exact
  statistics, each with and without (b).
  Result: branch `region/lidar-matching/2026-11-18-cell-statistics-experiment` (2cf91a6; step checks hold but for the
  parameters file's new key), 16 replays, one per variant, none at the time limit, every capture map built with the
  default statistics. Default / exact / default + 3-point rule / exact + 3-point rule: passage 100–110 s 6.04 / 9.63 /
  6.20 / 9.63 m; stairs 207–215 s 6.03 / 21.21 / 6.00 / 20.95 m with largest speeds 1.38 / 5.51 / 2.86 / 5.33 m/s; end
  stretch 28.9 / 53.4 / 30.1 / 55.4 m; Livox 3 largest difference to default – / 1.14 / 0.002 / 1.07 m. Two controls
  with the degeneracy projection off (mine, 8 replays): default 5.75 m, exact 6.22 m in the passage; exact then does
  not run away (stairs 5.96 m, 1.43 m/s; end 29.85 m, 1.90 m/s). With the exact statistics the information ratio is
  below 0.1 (the projection's lower threshold) in 43 of 84 passage, 225 of 250 stairs, 32 of 84 Livox 3 and 60 of 131
  end windows, with the default statistics in none (minimum 0.117–0.170): there the projection removes the NDT's
  translation part along the weak direction and the inertial terms decide (result = guess within 0.7 mm in passage
  windows 289–298). So the 9.63 m is not a LiDAR result, and the exact statistics alone do not lengthen the passage
  beyond what one replay resolves. The reading (a), point-weighted, mm along travel, cells with 10 and more points,
  indoor / outdoor grid: default statistics passage (100–110 s) +9.7 / +25.4, stairs +5.3 / +17.3, end +12.4 / +40.2;
  exact (projection off) +2.4 / +9.9, +0.4 / +3.4, +2.3 / +8.2. Young cells in the stall windows 285–300 lie +19 to
  +34 mm ahead with both statistics (the reviewer's recount). The in-plane pull in windows 288–292 falls from −38 …
  −43 to −17 … −26 mm, the outdoor grid's from −77 … −154 to −59 … −75 mm. Reviewed (conclusion reviewer): numbers hold;
  the pulls are single linear steps, not a causal split; the 3-point rule is dropped.
  `experiments/regions/lidar-matching/2026-11-18-cell-statistics/`.
- [x] 3.12 The acceleration offset's random walk (`imu_acceleration_offset_rw_mps2_sqrt_s`, 0.0001 in the Livox file,
  built-in 0.05; the offset is folded into the gravity vector after each window, so it is the state that can absorb a
  tilt of the attitude) and the projection's thresholds, replays on the same four captures (28, one per variant, none
  at the time limit; same folder, `run_offset_rw.py`, `run_low_thresholds.py`, tables `experiments/stage8/tools/
  s8_cs_table.py`). The accelerometer's steady misfit against the solved path as a tilt (10 s block means), default
  settings: passage 0.03–0.15°, Livox 3 0.07–0.10°, stairs 1.01° falling to 0.06°, end 0.64–0.72°; the offset's prior
  sigma is 0.002 m/s². Random walk 0.01 with the default statistics: stairs tilt 1.01 → 0.33°, velocity chi² median
  267 → 37; end 0.72 → 0.22°, 219 → 28; Livox 3 within 0.02 m; but the passage runs to 11.20 m and 20.91 m (true about
  10 and 12) at up to 3.66 m/s: the offset takes up the NDT's braking (cumulative offset +0.3, +0.3 m/s² in windows
  287–297). Exact statistics + random walk 0.01 + projection at 0.1 / 0.2: passage 9.42 / 12.79 m, stairs largest
  speed 2.22 m/s, end 107 m at up to 4.86 m/s. Exact + 0.01 + projection at 0.01 / 0.02 and at 0.02 / 0.04: passage
  8.84 / 9.37 m and 13.07 / 13.05 m, largest speed 1.27 / 1.28 m/s; stairs 207–215 s 5.89 m, 1.42 m/s, chi² 48 / 47,
  tilt 0.35°; Livox 3 within 0.06 m of the default; end 25.12 / 25.32 m, 1.31 m/s; no bent or fast flags (bias-high 4
  against 3 on the stairs); projection off: passage 7.91 m. Against the default the stairs replay ends 7.7 m away
  (vertical move −5.0 against −4.0 m) and the end stretch 4.5 m: which is right is not known from replays.
  Reviewed (conclusion reviewer): the numbers hold. The tilt reading and the chi² are computed with the gravity vector
  the offset is folded into, so their fall is partly by construction and no proof of a better attitude; the
  independent evidence is the passage's length and speeds. The thresholds were chosen on these same four captures, one
  replay each, and the margin is thin (0.1 / 0.2 with the same settings runs away). Not run: the default statistics
  with the low thresholds and the offset at 0.01. How the looser offset drives the speed up in the passage is not
  isolated (0.03 behaves differently from 0.01).
- [ ] 3.11 Full Livox 4 and Livox 3 runs with the candidate of 3.12 (exact statistics, offset random walk 0.01,
  projection 0.02 / 0.04), maps built with the exact statistics from the start; the run check; the user's look at
  the maps.
  Runs done (commit 54dff85, `experiments/stage8/candidate_livox{4,3}`, settings `candidate_livox.toml`; Livox 4 wall
  2016 s, Livox 3 1342 s), against `full_gyro_livox{4,3}`. Livox 4: passage 100–110 s 5.94 → 9.53 m, 110–124 s 11.98 →
  13.07 m; stairs 195–285 s 87.85 → 88.00 m, vertical −5.26 → −5.15 m; end stretch 315–362 s 24.85 → 24.48 m; no bent,
  fast or bias flags (largest speed 1.62 → 1.42 m/s, largest bias 0.58 → 0.49 °/s); velocity chi² median 73 → 38;
  start-to-end before loop closure 3.73 → 5.03 m, in height 0.70 → 0.04 m; loops 66 → 83, largest correction 4.36 →
  5.01 m; the final trajectories differ by up to 5.08 m (168.7 s). Livox 3: start-to-end 0.06 m both, largest
  correction 0.22 → 0.17 m, chi² 39 → 27, final trajectories within 0.33 m. The user's look at Livox 4
  (`alt_full_sheet.png`): fewer double walls than `full_gyro_livox4`, double walls remain around the start rooms.
  Three more Livox 4 runs (commit 54dff85; summary `experiments/stage8/candidate_runs_summary.md`, no verdict yet):
  the candidate on the yaw-scaled copy `4_gz0040` (`candidate_livox4_gz0040`), and the candidate with `ahrs_gain = 0`
  on Livox 4 and on `4_gz0040` (`candidate_gain0_livox4`, `candidate_gain0_livox4_gz0040`). Candidate / gz / gain 0 /
  gain 0 + gz: passage 100–110 s 9.53 / 9.51 / 9.30 / 9.45 m, 110–124 s 13.07 / 13.08 / 13.06 / 13.06 m; stairs
  88.00 / 87.95 / 87.82 / 88.01 m; end stretch 24.48 / 24.82 / 25.47 / 24.87 m; no bent or fast flags, largest bias
  0.49 / 0.45 / 0.49 / 0.49 °/s; chi² median 38 / 38 / 37 / 36; start-to-end 5.03 / 4.79 / 3.82 / 4.24 m (height
  0.04 / 0.13 / 0.69 / 0.04 m); loops 83 / 86 / 85 / 88, largest correction 5.01 / 4.77 / 3.81 / 4.22 m; whole-floor
  plan fit mean |d| 0.087 / 0.084 / 0.087 / 0.073 m, within 10 cm 73 / 72 / 73 / 79 %. Final trajectories against
  the candidate: up to 1.08 / 1.40 / 1.61 m. The gain-0 Livox 4 run check is INCOMPLETE (window 414's uncertainty
  eligibility unresolved). The user's look at the three full sheets is open.

## 4. Report and decision

- [ ] 4.1 The numbers for the user, without a verdict; the conclusion reviewed (Sonnet) before it is stated.
- [ ] 4.2 The user's decision: which fix is proposed as its own change, or what is looked at next.
- [x] 4.3 Knowledge entries of the regions that took part; tracker rows for defects found.

## 5. Corridor slow stretches (found by the repeats of `gyro-scale-correction`, 2026-11-19)

- [x] 5.1 Where the scatter of the six Livox 4 repeats (1.2–3.1 m, limit 0.85 m) is made: the logs and the raw IMU of
  the seven runs. Reviewed (conclusion reviewer).
  Result (`experiments/stage8/scale/analysis/README.md`, section Reading): the odometry of the seven runs agrees within
  10 cm until 121 s and keeps its offsets after 196 s (changes up to 0.7 m); it splits in corridor passes: 126–136 s
  (speeds 1.16–1.52 m/s), 137–141 s (three runs slow to 0.1–0.6 m/s for 3 s) and 177–181 s on the lower storey (the
  base, span_plus and all five earlier runs slow to 0.2–0.45 m/s for 4–5 s), 2–3 m each, while the IMU's step shocks
  stay at walking level. The two runs slow in neither place end 1.20 and 1.37 m from their start before loop closure,
  the others 3.8–5.7 m.
- [x] 5.2 Why: the per-window tables the full runs wrote (`window_features.csv`, `window_terms.csv`), all twelve Livox 4
  runs and Livox 3. Reviewed before it is stated.
  Result (same README, section Mechanism, with the reviewer's corrections): in windows with the information ratio
  below 0.02 the result equals the guess along travel within a few mm (p95 3, largest 5): dead reckoning from the
  carried velocity and the accelerometer, by construction. Livox 4 has 90–110 of 1016 windows below 0.04 in 6–9
  stretches of 3–6 s, Livox 3 24 of 702 in two stretches of 2–3 s. Lower corridor (windows 485–497, 4.65 s): all 11
  runs with the projection lose 0.25–0.78 m/s; the loss orders with the carried gravity vector's offset along travel
  at entry (r = −0.91 over 10 distinct runs, slope half the physical one); the run without the projection ends the
  stretch at 0.02 m/s. Runs leaving it at 0.36 m/s or less stay slow for 3 s: the LiDAR cost has a lower minimum
  15–70 cm ahead in every such window, the window moves 5–47 mm towards it. Carried velocity sigma 4–11 mm/s (mainly
  the motion-edge position terms, sigma 1e-4 m at weight 0.1, pinning the window to a guess built from the same
  accelerometer), against corrections of 0.25–0.7 m/s at velocity chi² of hundreds to 57,000; nearly every
  dead-reckoned stretch ends with a window moved 5–27 cm from its guess. Upper passage exit: edge_plus window 385 is
  moved −158 mm (velocity 0.58 → 0.29 m/s, offset +0.24 m/s²), the base +148 mm. Tested and not supported: a
  sensor-fixed accelerometer bias (3–9 % of the gravity state's variance).
- [x] 5.3 Captures of both stretches, each run stopped once its capture is written (`experiments/stage8/tools/
  s8_stall_captures.py`, `s8_stall_extra.py`, results `experiments/stage8/stalls/`): the lower corridor 160–186 s with
  the base settings, the upper passage 122–143 s with the edge_plus member's settings; replays with the own settings
  (identical) and with settings changed. Verify: own-settings replays identical, no window at the time limit.
  Result (`experiments/stage8/stalls/replay_tables.txt`; 24 replays, none at the time limit, own settings identical
  72 of 72 and 59 of 59 windows; commit 9ddf0dc). Path walked in the slow stretch, upper 134.0–141.6 s / lower
  173.5–182.6 s; the full runs that do not slow down have 8.0–8.24 m / 9.09–9.23 m: own 5.77 / 6.05 m; projection
  thresholds 0.01, 0.02: 7.55 / 9.02 (but 1.2 m short in the rest of the lower range); 0.04, 0.08: 9.02 / 6.63; off:
  8.12 / 6.07 (2.4 m short in the rest); offset random walk 0.003: 8.12 / 7.17; 0.03: 8.10 / 5.92; accelerometer
  floor 0.3: 8.12 / 8.13; `imu_position_information_scale` 0.01: 7.57 / 8.80; 1.0: 7.63 / 7.01; floor 0.3 with scale
  0.01: 8.13 / 10.14 (one window at 2.55 m/s); the same with thresholds 0.01, 0.02: 8.12 / 7.18; with the projection
  off: 8.14 / 5.23. Velocity chi² median own 51 / 38, floor 0.3: 14 / 18.
  Null control (12 replays, changes of 0.1–1 %, the family's span × 1.05, the other member's edge sigma): lower
  5.92–6.36 m, ends within 0.5 m of the own replay (the stall stays in all six); upper 5.77–5.79 m and within 0.21 m
  for the four 0.1–1 % changes, 5.97 m with the base's edge sigma on the edge_plus state (the stall stays, although
  the base's own full run walks 8.16 m there), 7.45 m with span × 1.05. Reviewed: the variants cannot be ranked
  finely from one replay each; a changed setting also changes the state the stretch is entered with (thresholds
  0.01, 0.02 lose 0.33 m/s in the dead-reckoned windows against 0.66), so entry and recovery effects are not
  separated; with the floor at 0.3 the loss is 0.54 m/s and the windows after it move 24–129 mm per window towards
  the LiDAR's minimum (own: 5–47 mm).
- [x] 5.5 `imu_accel_floor_sigma_mps2` by replay (the user's option 1; `experiments/stage8/tools/s8_floor_calib.py`,
  90 replays, time limit 0, every own replay identical): three more captures with the base settings (passage 95–125 s,
  stairs 200–245 s, end stretch 315–362 s), a family of seven per value (0.2, 0.3, 0.5) on both corridor captures, one
  replay per value on the three new ones. Result (reviewed; `experiments/stage8/stalls/floor_calib_summary.md`): path in
  the slow stretch, lower / upper, 0.1: 5.92–6.36 / 5.77–7.45 m; 0.2: 6.67–8.33 / 8.12; 0.3: 6.70–8.62 / 8.12;
  0.5: 8.44–8.86 / 8.12 (reference runs 9.09–9.23 / 8.0–8.24). At 0.2 and 0.3 tiny changes decide whether the lower
  corridor recovers (one of seven and three of seven do); at 0.5 all seven do, 0.23–0.79 m short. The speed drop inside
  the blind windows stays at every value. End stretch: 0.5 ends 1.88 m behind the base replay (one replay, no truth;
  its family is run by `s8_floor_end_family.py`). Passage and stairs: 0.05–0.16 m. No value is proposed for shipping.
- [ ] 5.4 The numbers for the user without a verdict; the user's decision on what is proposed as its own change.

## Open, for a later look at higher effort

(Questions this change does not settle, with what was tried.)

- Why the in-plane residuals of points ahead of their node are biased (they pull the window back by 4-6 mm even in
  calm windows, 33-43 mm in the passage): how a scan samples a cell, how the cell means were built, or a range
  effect at grazing incidence. Not measured; the region agent's split only locates it.
- (Closed 2026-11-18: the standstill of windows 305-309 is taken as real, see 3.4.)
- The honest size of the carried velocity's covariance (it inherits the NDT's hold) and whether process noise
  between windows is missing (carried from stage 4).
- **The rotation model** (the hard part): with the measured gyro noise each window's rotation follows the gyro and
  its first pose is hard-fixed (`fix_origin` adds diagonal weight only, no anchor), so the only channel for a
  correction of the attitude is the gyro bias estimate; it swings by +-0.4 deg/s while walking although it reads
  -0.03 deg/s at rest at both ends of the run. A bias wrong by 0.3 deg/s tilts the attitude 3 deg in 10 s: the
  candidate cause of both the rotation drift the user sees and the false acceleration of the runaway. To
  design: the gyro's error model (noise, bias, scale error) and a prior with a covariance on the window's first
  pose in place of the hard fix; then the in-plane rule (a small floor instead of zero) on top of it.
- Why the gyro bias estimate jumps in one window at 107.6 s, and whether it still does with the measured gyro
  noise in a full run.
