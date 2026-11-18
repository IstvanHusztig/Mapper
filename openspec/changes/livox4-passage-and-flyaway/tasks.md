# Tasks

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
- [ ] 3.5 Full Livox 4 runs with the yaw gyro rate scaled by 1.004 and 1.008 (copies `raw data/livox/4_gz0040`,
  `4_gz0080`; measured gyro noise): does the bias estimate stop following the turn rate? Background: in both
  measured-gyro runs the yaw bias estimate follows the yaw rate (slope about -0.003, R2 0.20-0.25 on Livox 4,
  0.46-0.58 on Livox 3) and not the yaw acceleration (R2 0-0.02), so a time offset is not what it follows; the
  loop corrections against the net turn were reviewed and do NOT support a scale error (one loop repeated
  about 11 times dominates the fit).
- [ ] 3.6 A capture of the stretch that runs away without the in-plane part (315-362 s, experiment build, measured
  gyro, rule A gain 0): `experiments/stage8/runaway_capture`.
- [x] 3.7 Research (two Sonnet passes; `experiments/stage8/research_rotation_and_inplane.md`, the second pass read
  the sources): VoxelMap, FAST-LIO2 and GICP match points to planes only (GICP: a point constrains only along
  its normal); FAST-LIO2 and VINS-Mono correct the attitude as a state, the window's oldest pose carries a prior
  with a covariance, not a hard fix; X-ICP and Zhang's remapping hand weak directions to the prediction, which
  nothing checks (the runaway is that design's known failure). Nothing found on a systematic pull of voxel
  matching along surfaces. Not read: Point-LIO, LIO-SAM, the gyro's datasheet (secondary: sensitivity +-0.5 %).

## 4. Report and decision

- [ ] 4.1 The numbers for the user, without a verdict; the conclusion reviewed (Sonnet) before it is stated.
- [ ] 4.2 The user's decision: which fix is proposed as its own change, or what is looked at next.
- [ ] 4.3 Knowledge entries of the regions that took part; tracker rows for defects found.

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
