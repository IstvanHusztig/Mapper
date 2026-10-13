# Tasks

## 1. Diagnostics (bit-identical)

- [x] 1.1 Heading diagnostic: with `log_window_diagnostics`, log per window the yaw correction against the guess (first and last node) and the total yaw stiffness. Verify: Release build; `44_short` bit-identical.
- [x] 1.2 Silent events, each with a log warning and a per-run counter: map rebuild, window dropped (≤ 1000 points), solver step skipped, big-jump partial update. A summary line at the end of the odometry. Verify: Release `ctest`; on 44 the rebuild line appears at window 586; `44_short` bit-identical.
  Done 2026-10-13 (c9efa06): 44_short IDENTICAL (11 files), Release ctest passes; the counters read 0 on 44_short. The rebuild on 44 at window 586 is checked with the noise-floor runs (3.1). Found on the way: a missing `return` was not flagged by the compiler (tracker C27).

## 2. Harness

- [x] 2.1 Move the helpers into `tools/` (point cache, façade lines, revisit registration, plan anchoring, jitter) and write `tools/map_quality.py`. Verify: it reproduces the known numbers (spec scenario).
- [x] 2.2 README entries; CLAUDE.md's testing section names the harness. Verify: the pages describe it.
  Done 2026-10-13 (cbce08e): reproduces 44_wd_s0 −0.40 / −0.73° and 44_odo_h0_s0 −3.87 / −4.19°, jitter p99 4.49 / 1.29°. New: the final map's revisits differ by 1.33° (median) in heading at 0.11 m offset; the odometry's by 0.18° at 1.00 m. Only 4 of 11 pairs pass the quality threshold on the final map (jitter blurs the slices).

## 3. Noise floor and config-only experiments

- [x] 3.1 This change's master on s0/s1/s2: full and odometry-only runs, snapshots saved; harness table. Verify: the table is in this task.
  Done 2026-10-13 (code c9efa06, `raw data/44_nf_s{0,1,2}`, settings lidar-vertical + diagnostics + snapshot, harness with `--floors`):

  | | façade out / back [°] | revisit yaw med / worst [°], offset [m] | jitter p50 / p99 [°] | railing [cm] | stair max [cm] |
  |---|---|---|---|---|---|
  | s0 | −0.40 / −0.73 | 1.33 / 2.29, 0.11 | 0.94 / 4.49 | 4.9 | 8.5 |
  | s1 | −3.71 / +0.16 | 0.51 / 2.33, 0.03 | 0.94 / 4.38 | 3.4 | 11.9 |
  | s2 | −0.58 / −0.78 | 1.00 / 3.53, 0.09 | 0.94 / 4.44 | 5.9 | 8.5 |
  | spread | 3.31 / 0.94 | 0.82 / 1.24, 0.08 | 0.00 / 0.11 | 2.5 | 3.4 |

  Identical to the shipped results of 2026-10-11/12 (s0 −0.40 / −0.73°, s1 −3.71 / +0.16°): the diagnostics change nothing.
- [ ] 3.2 `/fp:fast` build on s0 (fragility probe). Verify: `trajectory_seams.py` against the `/fp:precise` run, recorded in this task.
- [ ] 3.3 Consistency off, and 2–3 passes, from the snapshots, on s0/s1/s2. Verify: harness table.
- [ ] 3.4 Map rebuild off (threshold very large), on s0/s1/s2. Verify: harness table.
- [ ] 3.5 Event counters on 44 and 45f. 44 done: map rebuilds 1 (window 585 s0), solver steps skipped 0, big-jump node updates 0, windows dropped 0 on s0, s1 and s2; 45f still to run. Verify: the counts are in this task; they decide which change-B fixes can stay bit-identical.

## 4. Wrap-up

- [ ] 4.1 Tracker rows updated (C1, C4, C7, C8 evidence); `master-reviewer` on the commits; memory. Verify: the entries exist.
