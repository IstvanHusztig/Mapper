# Tasks

## 1. Diagnostics (bit-identical)

- [ ] 1.1 Heading diagnostic: with `log_window_diagnostics`, log per window the yaw correction against the guess (first and last node) and the total yaw stiffness. Verify: Release build; `44_short` bit-identical.
- [ ] 1.2 Silent events, each with a log warning and a per-run counter: map rebuild, window dropped (≤ 1000 points), solver step skipped, big-jump partial update. A summary line at the end of the odometry. Verify: Release `ctest`; on 44 the rebuild line appears at window 586; `44_short` bit-identical.

## 2. Harness

- [ ] 2.1 Move the helpers into `tools/` (point cache, façade lines, revisit registration, plan anchoring, jitter) and write `tools/map_quality.py`. Verify: it reproduces the known numbers (spec scenario).
- [ ] 2.2 README entries; CLAUDE.md's testing section names the harness. Verify: the pages describe it.

## 3. Noise floor and config-only experiments

- [ ] 3.1 This change's master on s0/s1/s2: full and odometry-only runs, snapshots saved; harness table. Verify: the table is in this task.
- [ ] 3.2 `/fp:fast` build on s0 (fragility probe). Verify: `trajectory_seams.py` against the `/fp:precise` run, recorded in this task.
- [ ] 3.3 Consistency off, and 2–3 passes, from the snapshots, on s0/s1/s2. Verify: harness table.
- [ ] 3.4 Map rebuild off (threshold very large), on s0/s1/s2. Verify: harness table.
- [ ] 3.5 Event counters on 44 and 45f. Verify: the counts are in this task; they decide which change-B fixes can stay bit-identical.

## 4. Wrap-up

- [ ] 4.1 Tracker rows updated (C1, C4, C7, C8 evidence); `master-reviewer` on the commits; memory. Verify: the entries exist.
