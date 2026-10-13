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
- [x] 3.3 Consistency off, and 2–3 passes, from the snapshots, on s0/s1/s2. Verify: harness table.
  Done 2026-10-13 (s0 / s1 / s2): consistency off: façade out −0.37 / −4.18 / −0.49°, railing 5.8 / 4.4 / 8.0 cm, stair 7.2 / 11.1 / 9.6 cm, jitter p99 1.3°; 2 passes: railing 5.2 / 3.9 / 7.6, jitter p99 2.1°; 10 passes (floor): railing 4.9 / 3.4 / 5.9, jitter p99 4.4°. Railing improves monotonically with the passes on all three inputs (dose-response): consistency stays on (10 passes). Whether its trajectory jitter harms the cloud is not shown; change H keeps it as an improvement, lower priority. Odometry only: façade −3.87 / −4.32 / −4.23° (spread 0.45°), revisit yaw median 0.18 / 0.50 / 0.41° (final map 1.33 / 0.51 / 1.00°: loop closure and/or consistency add heading error at revisits), offset 1.00 / 1.99 / 1.67 m.
- [x] 3.4 Map rebuild off (threshold very large), on s0/s1/s2. Verify: harness table.
  Done 2026-10-13: façade out −5.27 / −2.84 / +1.14°, back −5.55 / +1.12 / +1.00°, railing 17.1 / 3.9 / 25.7 cm, stair 7.9 / 9.8 / 68.7 cm. Not uniformly worse (s1 equal or better), s2 lands in a different outcome; the mechanism is untested. The rebuild stays (status quo); odometry-only from these snapshots and the six-input floor (s0–s5) follow.
  **Decisions (user, 2026-10-13, after a Sonnet review):** the final-map façade spread is a loop-closure pass/fail outcome (s1 not corrected), not variation: odometry fixes are judged on odometry-only results, the final-map verdict on s0–s5 counted as pass/fail; loop-closure fixes (G) run in parallel, pulled forward, on snapshots.
- [x] 3.5 Event counters on 44 and 45f. 45f: all 0. 44 done: map rebuilds 1 (window 585 s0), solver steps skipped 0, big-jump node updates 0, windows dropped 0 on s0, s1 and s2; 45f still to run. Verify: the counts are in this task; they decide which change-B fixes can stay bit-identical.

## 4. Wrap-up

- [ ] 4.1 Tracker rows updated (C1, C4, C7, C8 evidence); `master-reviewer` on the commits; memory. Verify: the entries exist.
