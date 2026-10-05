# Design

## Context

`optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`) runs once per LM iteration, 100 times per window. Its two NDT passes (`hessian_fun_indoor`, then `hessian_fun_outdoor` with `use_hierarchical_rgd`) run one lambda per point. For a point of node `k`, the lambda:
1. transforms the point and looks up its bucket, read-only;
2. computes a 6×6 `AtPA` and a 6×1 `AtPB`;
3. under `mutexes[k]`, adds them into `AtPAndt.block<6,6>(6k,6k)` and `AtPBndt.block<6,1>(6k,0)`, and in the indoor pass `n nᵀ` into `normal_information[k]`.

Today:
- `multithread == false` runs `std::for_each` over the points in their order.
- `multithread == true` runs `std::for_each(par_unseq, …)`, so the order of additions into a block is the order in which the threads take the lock.

Facts the design relies on:
- A point writes only its own node's diagonal block and `normal_information` entry. Nothing else is shared or written.
- The serial result of block `k` is therefore the left-to-right sum, in point order, of the terms of the points with `index_pose == k`, starting from zero. The indoor pass finishes before the outdoor pass starts, so for each block all indoor terms come before all outdoor terms.
- The points are not grouped by node: `decimate` orders them by cell key (`ndt_map.cpp`). Every `index_pose` is in `[0, poses.size())`, because `poses[index_pose]` is already read unchecked.
- Profile (`stage-timing`, 44): indoor 6.05 ms + outdoor 4.63 ms per iteration, 47 % of the odometry. The machine has 16 logical cores.
- The domain may use the standard library (`<execution>`) but not TBB or the `ParallelExecutor` port.

## Goals / Non-Goals

**Goals:**
- Results bit-identical between multithreaded and single-threaded runs, and with today's serial path. On the precise build this is exact; on the `/fp:fast` build it is exact or within 1e-6, see decision 5.
- A large share of the NDT-loop time parallelized, with the gain measured.
- `useMultithread = true` by default once that holds.

**Non-Goals:**
- Parallelizing anything else in the iteration (setup, motion terms, solves); those are later changes (`banded-normal-matrix`).
- `Consistency` / `Consistency2`, which keep their local flags and stay serial.
- `robust_lidar_odometry`'s loops, which are already deterministic.
- Routing the domain's parallelism through the `ParallelExecutor` port.

## Decisions

1. **Group the points by node once per call, keeping their order.**
   - `std::vector<std::vector<int>> by_node(poses.size())`, filled by one pass over `intermediate_points` that pushes each index to its node's list. A counting sort into one flat array with offsets is the same thing with fewer allocations; use it if the timing shows the grouping costs more than about 1 % of an iteration.
   - The lists are built inside `optimize_lidar_odometry`, because the points don't change during a window but the function has no per-window state. Cost: one pass over the window's points (no arithmetic), against the 10.7 ms of the two NDT passes. It is measured with `log_stage_timing` and counted in the `ndt_indoor` phase.
   - Alternative: compute per-point `Blocks` in parallel, then sum them in point order, as `optimize_sf2` does. Rejected: about 340 bytes per point per iteration, plus a serial O(points) sum that would limit the speed-up.

2. **One code path, two execution policies.**
   ```cpp
   const auto run_nodes = [&](const auto &per_point) {
       const auto per_node = [&](int k) { for (int idx : by_node[k]) per_point(intermediate_points[idx]); };
       if (scan_to_map.multithread)
           std::for_each(std::execution::par, nodes.begin(), nodes.end(), per_node);
       else
           std::for_each(nodes.begin(), nodes.end(), per_node);
   };
   run_nodes(hessian_fun_indoor);
   if (use_hierarchical_rgd) run_nodes(hessian_fun_outdoor);
   ```
   - `nodes` is `0 … n-1`, built with `std::iota`.
   - The serial path visits node 0's points, then node 1's, and so on. Per block, that is the same sequence of additions as today's point-order loop, because blocks don't interact. So the single-threaded result doesn't change either.
   - `std::execution::par`, not `par_unseq`: the per-node body is a sequential loop, and nothing gains from unsequenced execution.
   - On MSVC, `par` runs on the Windows thread pool. 360 nodes per window give enough tasks for 16 threads.

3. **The mutexes go.** The point lambdas lose the lock and add directly; the per-node partition gives each block exactly one writer. `normal_information[k]` follows the same rule. Its sum into `lidar_translation_information` after the loops stays serial, in node order.

4. **The default becomes `true`, only after the checks of decision 5 pass.**
   - It changes in `OdometryConfig` and in both settings files, and the golden TOML files are regenerated in their own commit.
   - The setting itself stays; `false` remains the way to run on one core.
   - The params TOML of every run changes (`useMultithread = true`), so its hash is updated in the 44 and 44_short manifests and the precise reference, as in `stage-timing`.

5. **Proof of bit-identity, in this order:**
   1. **Unit tests** (`tests/scan_to_map_test.cpp`), on the synthetic room and corridor, with points in scene order and in shuffled order:
      - one iteration and a full 30-iteration optimization with `multithread = true` and `false` give exactly equal poses (`==` on every matrix element), gyro bias and translation information;
      - two multithreaded runs are equal.
   2. **Precise build, `44_short`:** with `useMultithread = false` it is IDENTICAL with the precise reference (the serial path didn't change); with `true` it is IDENTICAL too.
   3. **Fast build, `44_short`:** IDENTICAL with the manifest for both values. If the rewritten loop changes `/fp:fast` code generation, the CLAUDE.md rule applies instead: within `--tolerance 1e-6` of `raw data/44_short_baseline`, and the setting-on and setting-off fast runs must still be IDENTICAL with each other.
   4. **Full 44 with the new default:** IDENTICAL with the manifest apart from the params TOML, or within 1e-6.

   A run where a window hits `real_time_threshold_seconds` can't be identical by construction, in either mode. The multithreaded run is faster, which makes that rarer, not more common.

6. **Speed measurement** with `log_stage_timing` (a scratchpad copy of `lidar-vertical.toml` with the setting on):
   - `44_short` with the setting off and on, and the full 44 with it on;
   - compared with the `stage-timing` profile: the NDT phases, the odometry time, and the whole run;
   - the results go into a new `docs/PERFORMANCE.md` fix entry.

## Risks / Trade-offs

- **The `/fp:fast` build may change last bits** because the loop's shape changes. That is handled by decision 5.3: precise identical, fast within 1e-6. If so, the 44 and 44_short baselines are refreshed from the new output, as `chain-next-window-only` did.
- **Load imbalance:** nodes at the window edges or in sparse scenes have few points. 360 tasks for 16 threads balance well enough; if the measured speed-up of the NDT phases stays below about 4×, chunk several nodes per task, or group the tasks by point count.
- **Contention with the GUI:** during a GUI run, the processing now uses all cores. The redraw is capped at about 10 fps (`gui-frame-rate-cap`) and runs on the main thread. The user checks that the GUI stays responsive and that its `44_short` result is identical.
- **`real_time_threshold_seconds`:** shorter windows hit the time limit less often, so results become less load-dependent, not more. The finding in `PERFORMANCE.md` stays open, since a slow machine can still hit the limit.
- **Memory:** the per-node index lists take about 4 bytes per point per call, which is negligible.
