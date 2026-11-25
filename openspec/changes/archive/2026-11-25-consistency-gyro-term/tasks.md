# Tasks

> **Closed 2026-11-25 (housekeeping; the user: "all as suggested"). Rejected: the gyro term was removed from the code on the user's decision (merge 213d18f); the spec delta keeps only the hold-out mode, which stays.**

> **2026-11-25: rejected and removed from the code on the user's decision** ("remove it completely and if needed
> implement from scratch"): the gyro rotation term (`consistency_gyro_rotation_term`, `consistency_gyro_sigma_scale`,
> `consistency_gyro_gate_deg`, now retired settings) and its test are gone. The hold-out mode (group 2, `holdout_block_s`)
> and `tools/holdout_sharpness.py` stay. The results below and in `results.md` stay as the record.

## 1. Measures that measure what they claim

- [x] 1.1 `tools/wall_precision.py`, findings of the post-push review: per visit pair an expected noise √(MAD₁²/n₁ + MAD₂²/n₂)·1.25 and the pair's difference against it (`pair_excess_m`), each visit's median position along the piece and the |Δalong| of a pair; 3D incidence; `plan_dist_m` summarised by `sides`; length from the line's extent, not its cell count; no KeyError without `gyro_bias_per_window` (windows then empty), the newest results JSON, clear messages on no pieces / no pairs; `--cell` and `--sigma`; legend bins match `searchsorted`; README. Verify: on `44_baseline` the tool runs, `pair_excess_m` is printed, and a run on a result made without gyro-bias estimation (44_short with `use_gyro_bias_estimation = false`) doesn't crash.
- [x] 1.2 `findings.md` of map-precision-leaks: the stop-rule wording (F4) and the noise-floor caveat for F1/F6 with the 1.1 numbers; tracker C35 gets the trailing-chunk line (`consistency.cpp:194`: a chunk of ≤ 10,000 points gets no NDT term). Verify: the diff reads correctly against the code lines it cites.
- [x] 1.3 `tools/holdout_sharpness.py` (design 8) with README section. Verify: on the research results `research2/hold_none`, `hold_base`, `hold_gyro_s3` it reproduces +1.85 / −0.87 mm (paired against hold_none) within 0.05 mm.

## 2. Hold-out evaluation mode

- [x] 2.1 `holdout_block_s` in `OdometryConfig` and a new `[evaluation]` TOML section (`TomlIO::POINTERS`, `CATEGORIES`), default 0; snapshot settings check covers it. Verify: unit test that a snapshot with another `holdout_block_s` is refused.
- [x] 2.2 Domain predicate `is_held_out(t, block_s)` and its use where points enter a map or a point term (design 6: odometry initial map, window map update and rebuild, `optimize_lidar_odometry` point loop, loop closure segment registration, consistency chunk map and point loop); saving untouched. Verify: unit test on synthetic points (odd blocks excluded, even kept, 0 = all kept); a 44_short run with 0.01 writes as many points as one without.
- [x] 2.3 Golden TOML files regenerated (`LO_UPDATE_GOLDEN=1`) in their own commit. Verify: `ctest` passes in Release.
- [x] 2.4 Bit-identity with the mode off. Verify: `compare_baseline.py` on 44_short IDENTICAL against the manifest (parameters file aside).

## 3. Gyro rotation term in consistency

- [x] 3.1 Settings `consistency_gyro_rotation_term` (false), `consistency_gyro_sigma_scale` (3.0), `consistency_gyro_gate_deg` (3.0) in `[consistency]`; `settings/default_settings.toml` and `l2-vertical.toml` get them off; golden TOML files again in their own commit. Verify: `ctest` passes.
- [x] 3.2 The term in `Consistency` (design 1–5): edges inside each window, sigma from the odometry's rule × scale, per-window gate with a log line counting gated windows, assembled in node order into the band. Verify: unit test on the synthetic loop recording (`tests/synthetic_recording`): with the term the node-to-node rotation error against the truth is smaller than without; repeatable bit for bit; if the `should_fail` case "Consistency leaves a trajectory that is already right" starts to pass, its marker goes.
- [x] 3.3 Bit-identity with the term off. Verify: 44_short IDENTICAL against the manifest. (The Debug build + `ctest` moved to 5.1: user, 2026-10-18, to save tokens; low risk, the band drops out-of-band writes.)
- [x] 3.4 Reproduce the experiment with the real implementation: 44_short odometry with `holdout_block_s = 0.01` and a snapshot, consistency from it with the term off / s = 2 / 3 / 5. Verify: `holdout_sharpness.py` gives the same sign and size as the experiment (−0.7 to −0.9 mm at s = 2–5 against odometry only) within 0.2 mm. (Done 2026-10-18, real code, held-out all-time thickness against consistency off: consistency without the term +1.71 mm, term s = 2 / 3 / 5 −1.04 / −1.12 / −1.13 mm, 0 of 70 windows gated, consistency 8.2 → 8.4 s. Same sign, 0.15–0.26 mm better than the experiment (s3/s5 just outside the 0.2 mm band); the reference now also has the hold-out in loop closure.)

## 4. Evaluation on recording 44

- [x] 4.1 Gate check: the term on 44_baseline's snapshot (`raw data/44_snap_full/odometry_snapshot.bin`) with the gate at 3°: count of gated windows, whether windows 629–639 (the gyro fault, C38) are gated, and the rotation over 255.8–259.9 s against the odometry result (must stay within 1°). Verify: numbers written into the change's `results.md`.
- [x] 4.2 Hold-out runs: s0/s1/s2 odometry with `holdout_block_s = 0.01` and a snapshot each (3 × ~25 min), then from each snapshot loop closure + consistency with the term off and on (s = 2, 3, 5). Verify: `holdout_sharpness.py` table (paired against the term off, interval including the s0/s1/s2 spread) in `results.md`.
- [x] 4.3 Full-density runs: s0/s1/s2 from their existing snapshots (or full runs) with the term off and at the chosen s; `map_quality.py` (geometry, revisits, jitter, `--floors`), `wall_precision.py` (`pair_diff_m`, `pair_excess_m`, thickness), jitter per IMU sample, consistency time. Verify: table in `results.md`; differences judged only beyond the s0/s1/s2 spread.
- [ ] 4.4 Have the conclusions reviewed (Sonnet subagent), then show the numbers and before/after images (plan_section overlay, wall_precision images) to the user and ask whether to ship. Verify: the user has answered.

## 5. Ship (only if the user accepts in 4.4)

- [ ] 5.1 Debug build + `ctest` (moved from 3.3); `settings/l2-vertical.toml`: the term on with the chosen scale and gate; tracker C1 → fixed with the evidence; CLAUDE.md and `docs/PERFORMANCE.md` (consistency time) updated. Verify: the diff.
- [ ] 5.2 New 44 and 44_short baselines in their own commit (folders renamed with a date suffix, manifests rewritten). Verify: `compare_baseline.py` IDENTICAL on both new manifests; master-reviewer after the push.
