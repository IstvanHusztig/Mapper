## Context

`load_data` sorts each LAZ file's points by timestamp with `std::sort` (`lidar_odometry.cpp`); `decimate` (`ndt_map.cpp`) sorts point/cell pairs by cell index with `std::sort` and keeps the first point of each cell; the consistency pass sorts (cell, point) pairs by cell only (`consistency.cpp`). None is stable. Points of the L2 share timestamps, so equal keys are common, and introsort orders them depending on the whole array.

Found 2026-10-15 while building a 95 s test bed for change F/E: master's odometry on `44c95_s0` (44_s0 cut to 0–95 s) differs from the same build on the full `44_s0` from window 1 on (window 1's final delta 7.4e-7 vs 8.6e-7; > 5 cm from 1.7 s, 0.11 m over 18–49 s, 0.52 m at 88 s). The full run itself reproduces the 10-14 reference exactly, so the code changes since then are not the cause.

## Goals / Non-Goals

- Goal: the odometry of a window depends only on the data up to it; a cut recording reproduces the full run.
- Goal: no loss of map quality; the s0–s5 scatter not wider.
- Non-goal: choosing a better representative point per voxel (nearest to the centre, mean): a different numeric choice, possibly later in change F.

## Decisions

1. `std::stable_sort` for the points by timestamp (file order kept for equal timestamps), for `decimate` by cell index (the earliest point in that order represents the cell), and for the consistency pass's (cell, point) pairs. The IMU sort stays (timestamps are strictly increasing by contract).
2. Unit test: `decimate` of a point set and of the same set with extra points appended elsewhere keep the same representatives for the shared cells, and the representative is the first of the cell's points in input order.

## Acceptance rule (written 2026-10-15, before the runs)

1. **Exact cut:** the change's build on `44c95_s0` and on the full `44_s0`, odometry only (`use_loop_closure`, `use_consistency` false): largest position difference ≤ 1 mm up to 90 s (`trajectory_seams.py`).
2. **Map quality:** the change's build with `settings/lidar-vertical.toml` on full s0–s5, against master's results from the C10 snapshots (`44_g3b2bs6_s*`, whose odometry equals today's master on s0): a run counts as better or worse only beyond the effect sizes façade |out| 0.3°, gallery out − back 0.1°, revisit worst 0.3°, railing 0.5 cm, stair max 1 cm; no measure worse in 5 or more of 6 inputs; the façade spread not wider by more than 0.5°; hard criteria: start room 0–9 s within ±0.3° of the odometry, gallery out − back ≤ 0.3° in every input. The spreads of all measures are reported (expected: not wider).
3. **45f:** no loss of sharpness (`eval_odometry.py` jitter within 5 %), a top view against master's.
4. Runtime reported.
5. Then new 44 and 44_short baselines (own commit), the user's visual check, push with the user's go.
