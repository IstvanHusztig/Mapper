# Proposal

## Why

`useMultithread = true` makes each window about 40 % faster, but results then differ from run to run, so it has always been off. The measured profile (`stage-timing`, `docs/PERFORMANCE.md`) shows that the NDT point loops it parallelizes are **47 % of the odometry and 37 % of a whole 44 run** (737 s of 1990 s) on one of the machine's 16 cores. It is the largest single speed-up left, and the first change of the performance plan agreed on 2026-10-05.

The cause is narrow. In `optimize_lidar_odometry`, each point adds its terms into its own trajectory node's block under a per-node lock. With parallel execution, the points of one node arrive in thread order instead of point order, and floating-point sums depend on order. The serial path adds each node's points in point order. A parallel version that keeps that order per node does exactly the same additions, so it gives the same bits as the serial run.

## What Changes

- **Deterministic parallel accumulation in `optimize_lidar_odometry`.**
  - The points are grouped by trajectory node, keeping their original order within each node.
  - The nodes are processed in parallel. Each node's points are added into that node's block in their original order, with no lock.
  - The indoor grid pass runs before the outdoor one, as now.
  - Serial and parallel runs use the same code with a different execution policy. Their results are therefore **bit-identical** with each other and with today's serial path.
- **The mutexes in `scan_to_map` go,** and with them the `par_unseq` loops that took them, which were undefined behaviour (a blocking call inside `par_unseq`). The new node loop uses `std::execution::par`.
- **`robust_lidar_odometry`:** its two parallel loops already compute per point without a lock and sum in order, so they are deterministic and stay as they are. Only the stale `// ToDo fix for this case` comment and the unused `<mutex>` include go.
- **New default: `useMultithread = true`** in `OdometryConfig`, `settings/default_settings.toml` and `settings/lidar-vertical.toml`. This happens only after the checks show results identical with the setting off. The setting stays, so a run can still be made single-threaded.
- **Docs:**
  - `docs/PERFORMANCE.md`: a new fix entry with the measured speed-up and the updated settings table.
  - `docs/index.html`: the setting's row and the "Results differ between runs" troubleshooting row.
  - `docs/ARCHITECTURE.md`: the parallelism decision and the bit-identical refactor rule, which today rely on the setting being off.

**Not in this change:**
- `Consistency` / `Consistency2` have their own hard-coded multithread flags, which are not `useMultithread`. They stay serial; the consistency analysis is a later step of the plan.
- Moving the domain loops behind the `ParallelExecutor` port: the domain keeps using `<execution>` from the standard library.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `lidar-odometry-motion-model`: the requirement "Reproducible results" is replaced by "Reproducible results with or without multithreading". Runs SHALL be byte-identical with multithreading on or off, and the default becomes multithreaded.

## Impact

- **Code:**
  - `src/domain/scan_to_map.cpp`: the accumulation;
  - `src/domain/robust_lidar_odometry.cpp`: the stale comment and the unused include;
  - `src/domain/odometry_config.h`: default and comment.
- **Settings:** `settings/default_settings.toml` and `settings/lidar-vertical.toml`, plus the golden TOML test files, regenerated in their own commit.
- **Tests:** `tests/scan_to_map_test.cpp`. On the synthetic room and corridor, a multithreaded window optimization gives exactly the same poses, gyro bias and translation information as the single-threaded one, also when the points are not grouped by node.
- **Results:** bit-identical. With `useMultithread = false`, the precise `44_short` must equal the precise reference, and with `true` too. The `/fp:fast` build must stay IDENTICAL with the manifest, or within 1e-6. Changing the default changes the params TOML of every run, so its hash is updated in the manifests, as `stage-timing` did.
- **Speed:** measured with `log_stage_timing` on `44_short` and the full 44, against the `stage-timing` profile (44 odometry 1578.5 s).
- **GUI:** the processing runs on more cores during a GUI run. The user checks that the GUI stays responsive and that its result is identical.
