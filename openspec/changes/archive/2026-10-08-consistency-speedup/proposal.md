# Proposal

## Why

After the odometry fixes of 2026-10-05 to 08 (`docs/PERFORMANCE.md`, fixes 7 to 10) the full 44 takes about 907 s, and the **consistency pass is the largest piece left that nobody has touched: about 320 s, 35 % of the run** (the odometry is 510 s). It is serial. The shipped settings run `Consistency` (`use_mutliple_gaussian = false`) ten times (`num_constistency_iter = 10`), about 32 s each.

Reading `Consistency` (`src/application/consistency.cpp`) shows where the time most likely goes. Each pass:

1. loads every window's points from the point store again and appends them to one vector (16.5 million points, about 900 MB);
2. in chunks of 1 million points: finds each point's node with `lower_bound`, copies the chunk twice (local and global), and builds an NDT map with `update_rgd`;
3. runs one serial loop over the points behind a mutex (`multithread` is hard-coded `false`) that, per point, pushes about 30 `Eigen::Triplet`s (18 for `A`, 9 for `P`, 3 for `B`), so about 30 million per chunk;
4. builds three huge sparse matrices from them (`A`: 3 million rows, `P`: 3 million × 3 million, `B`) and forms the normal equations with two sparse products, `Aᵗ·P` and `(Aᵗ·P)·A`;
5. solves once with `SimplicialCholesky`.

The structure is the one the odometry already exploits: **every point's term touches only its own node's 6×6 block**, and the smoothness terms couple three neighbouring nodes. The normal matrix is therefore block-tridiagonal, and the triplets and the sparse products can be replaced by direct per-node accumulation into a `BandedNormalMatrix`, which is what `optimize_lidar_odometry` does since fix 8. With the nodes processed in parallel in point order (fix 7) the accumulation is also deterministic.

This is a guess from reading the code, so the change **measures first**, like `stage-timing` did.

## What Changes

- **Measure** (task 1): per-phase wall-clock timers in `Consistency`, summed over the passes and logged at the end of `run_consistency` when `log_stage_timing` is on: loading the points, preparing the chunks (node lookup, copies), `update_rgd`, the point loop, building the sparse matrices, the sparse products, the solve, and applying the update. Results bit-identical.
- **Direct accumulation** (task 2): replace the triplet lists and sparse products of the NDT term, the smoothness term and the fix-origin term by a `BandedNormalMatrix` (smoothness couples nodes up to two apart, so a band of 17 as in the odometry) and a dense right-hand-side vector, filled per node in parallel (the points grouped by node in their original order, as in fix 7). The solve stays `SimplicialCholesky` on the sparse copy. This changes the order of the additions, so the result differs from today's in the last bits; it is a numerical change of rounding size, not a refactor.
- **Keep the points between passes and overlap the chunks** (task 3, only if the measurement shows loading or `update_rgd` to matter): load each window's points once per run instead of once per pass, and build the chunk maps and their partial sums in parallel, merged in chunk order, so the result does not depend on the thread timing.
- **Baselines** (task 4): a full 44 and new baselines for `44` and `44_short` from the final code, as in `fixed-pattern-precise-fp`; the docs get a fix entry in `docs/PERFORMANCE.md` and in the performance page (`docs/performance.html` and its Hungarian copy).

**Not in this change:** `Consistency2` (`use_mutliple_gaussian = true`, off in the shipped settings; it has the same structure and gets the same treatment in a later change if it is wanted), the loop closure (54 s), the save (19 s) and the odometry.

## Capabilities

### Modified Capabilities

None. A performance change with a numerically negligible effect to be confirmed by measurement (`skip_specs: true`).

## Impact

- **Code:** `src/application/consistency.cpp` (and possibly a small shared helper next to `BandedNormalMatrix` in `src/domain/normal_equations.{h,cpp}`); `src/application/lidar_odometry.cpp` (`run_consistency`, for the timing summary); tests in `tests/consistency_test.cpp`.
- **Results:** within rounding of today's (expected about 1e-9; to be measured against the current baselines before they are replaced); `eval_odometry.py` metrics unchanged to the printed precision is the bar. New baselines and manifests for both datasets.
- **Docs:** `docs/PERFORMANCE.md`, `docs/performance.html`, `docs/hu/performance.html`.
- **Debug build:** no new asserts expected; Release only unless a task adds configuration-dependent code (`CLAUDE.md` rule).
