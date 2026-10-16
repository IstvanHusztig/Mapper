## Context

- Test bed: `raw data/44c95b_s0..s5` (44_sX cut to −10 … 95 s with `cut_recording`), odometry only (`use_loop_closure`, `use_consistency` false, window diagnostics on). The cut reproduces the full run's trajectory exactly (0.000 m, checked on s0 with and without a code change), so an effect measured on it is the full run's effect up to 95 s. About 7.5 min per input instead of 20.
- Reference: master (33d7abd build) on the same cuts, `44c95bo_master_sX`.
- Metric: the façade line fitted per time window (`map_quality_lib` façade fit, job tool `facade_time.py`): start (0.9–5.9 s), gallery way out (48.9–59.9 s), way back (59.9–70.9 s); the angle gallery − start is the global-geometry measure. Also the start's split (0.9–5.9 vs 5.9–10.9 s), the onset in 40–49 s, and the runs' sanity (largest window time, big-jump rejections, route).

## Phase 1: screening variants (settings only)

| variant | setting | tests |
|---|---|---|
| V1 coarse grid off | `min_distance_lidar_outdoor_grid` = 1000 (no point reaches the 0.4 m grid) | C18 |
| V2 projection off | `degenerate_projection_ratio` = `degenerate_projection_ratio_full` = 0 | C16 |
| V3 exact statistics | `use_exact_ndt_bucket_statistics` = true | C20 on top of C10 |
| V4 short initial map | `threshold_initial_points` = 10 000 (≈ 0.16 s instead of ≈ 0.94 s) | C21 |
| V5 finer cells (positive control) | indoor / outdoor resolution 0.10 / 0.27 m | known ≈ 25 % less drift; validates the test bed |

Plus **V4b** `threshold_initial_points` = 120 000 (dose-response for V4) and **V6**, V1 + V2 + V3 + V4 combined (do small effects add up?). Run order: master s0–s5 (reference; s0 checked against the known odometry values), V5 (stop if it fails), then V1, V2, V3, V4, V4b, V6, each on s0–s2, one at a time, with one pinned build.

### Screening rule (written 2026-10-16, before the runs; revised after the Sonnet review the same day)

- **Readings per run** (façade line, `map_quality_lib`, room frame, absolute): start A (0.9–5.9 s), start B (5.9–10.9 s), gallery (mean of way out 48.9–59.9 s and way back 59.9–70.9 s, one sample: the same wall in one map), the angle gallery − start A, and a heading profile (40–49 s, 71–85 s) for the onset. Sanity: largest window time, big-jump rejections, `trajectory_seams.py` against master, jitter.
- **Noise floor:** master on all six cuts s0–s5: mean M and range [min, max] of each reading.
- **Moves:** a variant's mean over s0–s2 lies outside master's s0–s5 range **and** at least 0.5° from M, with all three inputs on the same side of M. **Inconclusive:** 0.25–0.5° from M or inside the range. **Gain** counts only if the gallery reading moves towards the start's (the angle towards 0) without start A and start B moving apart by more than master's range, or if start A − start B moves towards 0 (the truth is one straight wall).
- **Worse:** a variant that moves the angle away from 0 marks a protective feature, not a drift source.
- **Positive control:** V5 must pass the same rule (gain). If it does not, the test bed or the metric is in doubt and the screening stops.
- **V1:** dropping the coarse grid also halves the NDT information against the IMU and motion priors, so a V1 effect reads "C18 or the NDT / prior balance" until phase 2 re-runs it with the priors rescaled. **V4:** a shorter initial map also moves the window grid (windows start where the initial map ends) and leaves a sparser first map; V4 and V4b must move in opposite directions (monotone) to count, and their start A / start B are read first.
- **V6:** if the combination doesn't move the angle, the map-model thesis is closed **for 44's angle** (not for caves); if it moves while the single ones don't, they add up.
- **Unusable as is:** a window over 10 s, more than 100 big-jump rejections, or a position difference to master of more than 1 m before 40 s.
- Screening decides only what phase 2 works on; nothing is shipped from phase 1. The machine is kept free of other heavy jobs (the 10 s window limit equals `real_time_threshold_seconds`, so load can change results).

## Phase 2 (designed after the screening)

For each mechanism that moves the angle: a fix in the code where the setting was only a probe (for example a down-weighted or hierarchical coarse grid instead of none, a consistent degenerate projection, motion-compensated initial points), its own acceptance rule written before its runs, full runs on s0–s5 with loop closure and consistency, `map_quality.py --floors` against master's results (global geometry, consistency, sharpness, equal weight), 45f, the user's visual check, baselines, push with the user's go.
