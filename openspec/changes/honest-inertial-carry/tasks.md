Rules for every task: Release only (Debug where a task says so); every replay through `tools/run_replay.py`; a
comparison on a capture is a family of seven (the value and six changes of 0.1–5 %) against the family of the same code
without the layer; numbers to the user without a verdict; nothing into `settings/` and no new baseline before the
user's decision. Captures: `C:/Users/User/Downloads/dLidar/experiments/stage8/stalls/{lower,upper,passage,stairs,end}`
and the two guards.

## 1. Groundwork (orchestrator, branch `honest-inertial-carry`)

- [x] 1.1 Measure today's carry on existing full runs and replays (`experiments/stage8/tools/s8_carry_honesty.py`,
  `s8_prior_cap.py`; results `scale/analysis/carry_honesty.txt`, `stalls/prior_cap.txt`): design table E1–E9, reviewed.
- [x] 1.2 Families of seven with today's code on the passage, stairs and end captures (the spread R-harm is read
  against; end: `s8_floor_end_family.py`). Done when the three family tables are in `experiments/stage8/stalls/`.
  Done (`end_family.txt`, `passage_stairs_family.txt`): end within 0.02 m of own, passage path 22.50-22.52 m and end
  within 0.02 m, stairs 40.08-40.15 m and end within 0.08 m.
- [x] 1.3 The three settings (`imu_edges_follow_state`, `imu_carry_joint_covariance`, `imu_accel_window_sigma_mps2`;
  `[imu_translation]`) in `odometry_config.h`, `toml_io.h` (`POINTERS`, `CATEGORIES`), the settings hash's
  `added_later` list (`odometry_snapshot_file.cpp`), `MotionModelTranslationImuOptions` and the `noise budget` log
  line; golden TOML files regenerated in their own commit. Verify: `ctest` with a test that the hash of the default
  configuration is unchanged; 44_short bit-identical; the five stage 8 captures and both guards replay as "own
  settings", bit for bit; an existing odometry snapshot is accepted. Done (e782942, a1d9ba8; manifests' params hash
  updated): step checks identical except the params TOML's three new keys; the five captures' own replays 72/59/84/125/131
  identical. The snapshot check rests on the hash test.
- [x] 1.4 `WindowLoopCarry` gets the velocity–offset cross block; capture format version + 1, the reader accepts the
  previous version with a zero block (design D7). Verify: `tests/window_capture_test.cpp` (a file of the previous
  version reads), `tools/step_checks.py`, the five captures' own replays identical.
- [ ] 1.5 Full run of recording 44 (s0) with `log_window_terms` and `log_window_noise` for the reading (results
  unchanged by the flags: compare with the manifest).
- [ ] 1.6 What makes the correction where the LiDAR sees (design E2): replays of the stairs and upper captures with
  `handoff_tail_nodes = 0`, and with `ndt_information_scale` 0.05 and 0.2 (`s8_tail_off.py`); velocity chi² and the
  correction by direction against the base replay. Report which candidate moves the reading and by how much.
  First result (`experiments/stage8/stalls/tail_off_honesty.txt`, one replay each): stairs (125 windows, none blind)
  correction p50 / chi² median own 28.2 / 39, tail off 28.9 / 37, NDT ×0.5 29.3 / 35, NDT ×2 37.2 / 76: the tail is
  not the cause there, the LiDAR's weight is part of it. Upper capture: tail off 64 → 30 mm/s in its 10 seen windows,
  confounded by the stall. Still open: the accelerometer candidate (task 5.1).

## 2. The honesty reading (verification, J1, worktree region-b)

- [ ] 2.1 `tools/carry_honesty.py` (pure Python) from the prototype: bands with velocity chi², blind stretches with
  the sigma at entry and exit, the first correction after each (size, direction, ratio there) and the number of
  windows until the speed settles, NOT MEASURED for missing columns; `tools/README.md` entry. Verify: its test on a
  small hand-made CSV with known answers.
- [ ] 2.2 Today's tables for 44, Livox 3 and Livox 4 (base and the six repeats) written to
  `experiments/regions/verification/<date>-carry-honesty/`.

## 3. Layer 1: the edges follow the state (inertial, J2, worktree region-a)

- [x] 3.1 `add_motion_edge_terms`: the position measurement follows the start velocity and the acceleration offset in
  windows whose guess was integrated (design D1), behind `imu_edges_follow_state`. Verify: the two invariance tests of
  the spec, the gradient test of the term, a window at rest unchanged, the per-term table still builds each term
  alone; off: 44_short and guards bit-identical (`step_checks.py`).
- [x] 3.2 Replays with layer 1 alone on the five captures, families on lower, upper and end; one full Livox 3 run.
  Report the readings of design D4 (the prior sigma through each blind stretch must pass today's ceiling), largest
  speed, flags. Hand-back with the tables; no verdict.
- [x] 3.3 Orchestrator: ownership check, step checks in the worktree, conclusion reviewer, merge with the switch off;
  the numbers to the user.

## 4. Layer 2: the joint covariance (solver J3 in region-b, then noise-budget and inertial J4 in region-a)

- [x] 4.1 Solver: `take_covariance` hands out the 6×6 block of end velocity and offset, kept in the window problem's
  saved state. Verify: unit test against a dense inverse; a rejected step restores the block; results unchanged while
  nobody reads the cross block.
- [x] 4.2 Noise-budget: the joint prior information on start velocity and offset in `WindowNoiseModel`, with the
  transition over the time between windows; inertial: the prior term on both unknowns, the carry of the cross block
  (`prepare_acc_term`, `prepare_velocity_state`, `update_acc_term_after_window`), a zero cross block in the fallbacks,
  behind `imu_carry_joint_covariance`. Verify: the N-blind-windows test and the fifteen-windows scenario of the spec;
  off: bit-identical.
- [ ] 4.3 Replays with layers 1 + 2 and one full Livox 3 run as 3.2. Hand-back; orchestrator's checks and merge as
  3.3; the numbers to the user. (2026-11-20: replays done, merged 164e9c8; `experiments/regions/inertial/
  2026-11-19-joint-carry/summary.md`. Full Livox 3 run `experiments/stage8/carry/livox3_efs_jc/` (164e9c8): max 0.161 m
  from today at 216.7 s (layer 1: 0.130), first > 5 cm at 165.9 s, never > 20 cm; no flags; chi2 median / p95 28 / 141
  (layer 1 26 / 159); 84 loops, largest correction 0.16 m; run_health INCOMPLETE: 68 windows' uncertainty eligibility
  unresolved (not seen with layer 1, not yet looked at). Paused 2026-11-20 for `consistency-harness`.)

## 5. Layer 3: measured first (noise-budget and inertial, J6)

- [ ] 5.1 Is there an acceleration error common to a window, and how large? With layers 1 and 2 on and the offset's
  prior loose (so the window's estimate is not limited by it), replay the guards and the stairs capture and run
  Livox 3 once: the offset each seen window estimates, its scatter from window to window, its slow part (10 s mean),
  against what the solve says the LiDAR alone resolves. Together with 1.6. Report per sensor: a number or "not
  shown". Conclusion reviewer. **The user's decision: build layer 3?**
- [ ] 5.2 If yes: `imu_accel_window_sigma_mps2` in the offset prior; the split at the carry (gain K, the three
  formulas of design D3), value 0 on today's code path. Verify: the formulas against a numeric conditioning of a
  6-dimensional Gaussian; the two scenarios of the spec; value 0: bit-identical.
- [ ] 5.3 If yes: replays with layers 1 + 2 + 3 at the measured value, half and double, as 3.2, with the gravity
  vector's path and the speed through the blind windows of the lower corridor; one full Livox 3 run. Hand-back;
  orchestrator's checks and merge. If no: the setting of 1.3 and the spec's requirement removed.

## 6. Full runs (orchestrator, J7)

- [ ] 6.1 **The user's decision on the replay numbers: run the full set?** Then Livox 4 base and the six repeats of
  `gyro-scale-correction` 2.3, Livox 3, 44 on s0, s1, s2, with the accepted layers. Sequential (the time limit makes
  parallel runs timing-dependent). `verify-run` on each.
- [ ] 6.2 `verify-map`: wall precision, seams against today's results, `map_quality.py` on 44 with the s0 / s1 / s2
  spread, plan fit and sections; the honesty reading on every run; the 44 stairwell window 213 and the Livox 4 stairs
  named in the table.
- [ ] 6.3 Summary without a verdict, the clouds' full paths; conclusion reviewer. **The user's look and decision per
  layer and sensor.**

- 2026-11-24/25, outside this plan (ghost wall diagnosis of change `loop-closure-weak-runs-and-match-check`): layer 1
  alone on the saved Livox 4 lower-corridor stall with master's settings (`livox4_lever/corridor_capture_lower`): travel
  placed in 173.5-182.5 s 9.19-9.21 m in a family of five (accelerometer span +-5 %, edge sigma +-10 %), master's settings
  3.44-3.86 m, about 9.6 m by the scene and the mouth registrations; layers 1 + 2 9.14 m but -0.61 m in the 3 s after
  (backwards). Full runs with layer 1 in a copy of the shipped Livox file (`livox4_lever/layer1/`): Livox 4 largest loop
  correction 1.08 m (5.83 m before), ghost wall gone, the frozen checks equal or better; Livox 3 within 0.40 m of its
  baseline (214-218 s). The user's look on both: "it's just wow"; layer 1 is ON in `settings/livox-vertical.toml` since
  07c0ae4 (2026-11-25; the user: no L2 test needed now). Layer 2 off; L2 not tested; 6.1-6.3 and 7.1 stay open for the L2
  and for the decision on layer 2.

## 7. After the decision

- [ ] 7.1 Accepted: values in `settings/livox-vertical.toml` and `settings/l2-vertical.toml`, golden TOML, new 44 and
  44_short baselines and guard references in their own commits, the five captures re-made with the new carry; the
  switches of layers accepted for both sensors removed. Rejected: the layer's code removed.
- [ ] 7.2 What is left (design D6 and Open Questions): R-exit on the full runs of 6.1. Proposal of the next change
  (position at re-observation, or the LiDAR's pull in weak windows) if the speed still takes more than two windows to
  settle, a member over-runs, or the first correction against travel is not repaired; else recorded as not needed.
- [ ] 7.3 Docs: the guide's noise-model part and `docs/window-terms.html` (what the edges' rows mean now), the
  knowledge files of inertial, noise-budget, solver and verification (experiments, known flaws, open questions),
  `docs/code-review-tracker.md` (roadmap P2, D1), the main spec synced. Hungarian pages at the end of the body of work.
- [ ] 7.4 Archive the change.
