# Design

## Context

Layout of `app/lidar_odometry_utils_optimizers.cpp` (2,922 lines):

| Lines | Content | Used by |
|---|---|---|
| 8–9 | `const double DEG_TO_RAD`, `RAD_TO_DEG` (double; `core/include/utils.hpp` has float versions of the same names) | 30 uses: scan-to-map, the window loop, consistency, robust mode |
| 11–57 | anonymous namespace: `Blocks`, `SumBlocks`, `pose_band_to_sparse` | `optimize_sf2`, `optimize_rigid_sf`, `optimize_lidar_odometry` |
| 59–334 | `optimize_sf2` (declared in `lidar_odometry_utils.h`) | `compute_step_2` |
| 335–606 | `optimize_rigid_sf` (no declaration) | `compute_step_2` |
| 607–1306 | `optimize_lidar_odometry` | `compute_step_2` |
| 1307–1318 | `get_motion_model_rotation_options` | `compute_step_2` |
| 1319–1459 | anonymous namespace: the IMU kinematics helpers | `compute_step_2` |
| 1460–2102 | `compute_step_2` | `OdometryPipeline::run_odometry` |
| 2103–2922 | `Consistency`, `Consistency2` | `run_consistency` (`app/lidar_odometry.cpp`) |

**Build settings that matter:**
- The Release build uses `/fp:fast` and whole-program optimization.
- A function that moves to another translation unit, or loses internal linkage, can in principle be inlined differently. Under `/fp:fast` that can change the last bits of the results.
- `44_short` must stay bit-identical anyway (ARCHITECTURE.md, Decisions).

**Baselines.**
- `max-prior-velocity` (archived 2026-10-03) set `max_prior_velocity_mps = 5.0` in `settings/lidar-vertical.toml`. It was checked only on `44_short`, where the limit never fired.
- Only the parameters entry of `tools/baseline/44.manifest.json` was updated.
- The baseline folders still hold the old parameters TOML.

## Goals / Non-Goals

**Goals:**
- One file per service, so the files of the old optimizer file each hold one responsibility. Each new file should be 900 lines or fewer.
- The IMU kinematics and the normal-equation helpers become reachable from tests and get their first synthetic tests.
- Bit-identical results.

**Non-Goals:**
- Changing any function body, name or signature. Exceptions: `optimize_rigid_sf` gets a declaration, and the helpers lose `static`/anonymous-namespace linkage.
- Breaking up `compute_step_2` (6b).
- A synthetic test of `optimize_lidar_odometry`, which would need the tests to link `core` (6c).
- Renaming `compute_step_2`, `Consistency`, `Consistency2`, or the `optimize_sf*` functions (6b at the earliest).
- Moving `run_consistency` out of `app/lidar_odometry.cpp`.
- Folders and CMake layers (step 7).

## Decisions

1. **File names follow the service, not the old step numbers.**
   - `scan_to_map` and `consistency` are what the code does.
   - `robust_lidar_odometry` is named after the setting `use_robust_and_accurate_lidar_odometry`, which selects these optimizers.
   - `run_odometry` is the use-case name from ARCHITECTURE.md. The function inside keeps `compute_step_2` until 6b breaks it up, so 6a is a move and nothing else.
   - *Alternative:* a single `optimizers/` folder. Rejected for now: folders are step 7.
2. **The old file becomes `app/run_odometry.cpp` by `git mv`.** The other parts move out first, one commit each, and the rename is last. `git log --follow` then keeps `compute_step_2`'s history, which is the most edited part.
3. **Helpers get external linkage in their own header.**
   - `normal_equations.h` declares `Blocks`, `SumBlocks` and `pose_band_to_sparse`. The default argument `max_offset = 17` goes into the declaration.
   - `imu_kinematics.h` declares the seven IMU functions.
   - Both headers include only Eigen and `<vector>`. `imu_kinematics.cpp` also includes `<transformations.h>` for `TaitBryanPose`. The tests can then compile them without `lidar_odometry_utils.h`.
   - *Alternative:* `inline` in headers. Rejected: it changes where the compiler sees the bodies, which is a larger change for the bit-identity check than a plain move to a `.cpp`.
4. **`app/angle_units.h`** holds `const double DEG_TO_RAD = M_PI / 180.0f;` and `RAD_TO_DEG`, written exactly as today.
   - It is included only by the files that used the old constants.
   - A namespace-scope `const` has internal linkage, so the header form is the same as today's definition in each translation unit.
   - It must never be included together with `core/include/utils.hpp`, which defines float versions of the same names. The header says so in a comment.
5. **The rotation and translation IMU option structs move to `optimizer_options.h`.**
   - The structs, and `get_motion_model_rotation_options` from the optimizer file, move to `optimizer_options.{h,cpp}`.
   - `optimize_lidar_odometry`'s declaration then sits in `scan_to_map.h`, which includes `optimizer_options.h`. The forward declarations added in 5b go away.
   - The new test case checks the degree-to-radian conversion and every mapped field. Like the 5b test, it gives every field a distinct value.
6. **Each move is its own commit, with the step checks.** If a move breaks `44_short` bit-identity, it is not accepted: stop and report which move did it and the size of the difference. Possible fixes, such as keeping the helper in the same translation unit, are discussed with the user then.
7. **Full 44 reference refresh first, run with a copy of master's Release `bin` folder.**
   - The copy goes into the scratchpad, so later builds can't overwrite the exe during the 30-minute run.
   - No builds or `44_short` runs happen during that run. CPU load changes results through `real_time_threshold_seconds`. Editing (the moves) can go on meanwhile.
   - **IDENTICAL with the manifest:** the limit didn't fire on 44. `raw data/44_baseline` is replaced by the run output, and the old folder is kept as `raw data/44_baseline_2026-10-03`.
   - **DIFFERENT:** the limit fired. The manifest is rewritten with `--write-manifest` in its own commit, the baseline folder is replaced the same way, and the number of "not used as prior" messages is reported.
   - `raw data/44_short_baseline` is replaced by the next `44_short` run on master, so its parameters TOML has the new key too.

## Risks / Trade-offs

- **[Different inlining after a move breaks bit-identity]** → One move per commit, each checked on `44_short`. Decision 6 says what happens if one fails.
- **[A file still includes `lidar_odometry_utils.h` for everything]** → Accepted in 6a. Narrowing the includes of `lidar_odometry_utils.h` is step 7 work. The tested helper headers are already narrow.
- **[`DEG_TO_RAD` name clash with `core/include/utils.hpp`]** → `angle_units.h` is included only where the old file's constants were used. If both headers reach one translation unit, the build fails loudly rather than silently changing precision.
- **[The 44 refresh may show that 5 m/s changed the 44 result]** → That is a finding for the user, not a failure of this change. It is reported with the drop count before the refactor is judged.

## Migration Plan

Nothing to migrate. To roll back, revert the commits. Output and settings formats are unchanged.
