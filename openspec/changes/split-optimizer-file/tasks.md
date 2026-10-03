# Tasks

**Step checks, on every code commit:**
- both presets build `dMapper` and `lidar_odometry_tests`;
- `ctest` passes;
- a Release CLI run on `44_short` from the repo root compares IDENTICAL with `tools/baseline/44_short.manifest.json`.

The GUI must be closed while building. Every move goes through `git diff`, which must show only moved lines plus includes, declarations and linkage (`namespace {` / `}` removed, nothing else).

## 1. Reference first

- [x] 1.1 Copy `out/build/x64-Release/bin` of master (dc71dcb) to the scratchpad. Run it on the full 44 in the background with `settings/lidar-vertical.toml` (design decision 7), then compare with `tools/baseline/44.manifest.json`. Count the "not used as prior" lines in the log. No builds or `44_short` runs until it finishes.
  - **IDENTICAL:** keep the old folder as `raw data/44_baseline_2026-10-03` and make the run output the new `raw data/44_baseline`.
  - **DIFFERENT:** do the same, also rewrite `44.manifest.json` with `--write-manifest`, and commit the manifest on its own.

  Report which case applied and the drop count. Result 2026-10-03: IDENTICAL, 35 files, 0 drops (2018 s); raw data/44_baseline replaced, old kept as 44_baseline_2026-10-03.
- [x] 1.2 Replace `raw data/44_short_baseline` with a `44_short` run of master. Verify that the run is IDENTICAL with `44_short.manifest.json` before replacing the folder. Result 2026-10-03: IDENTICAL, 11 files; old folder kept as 44_short_baseline_2026-10-03.

## 2. Moves

- [x] 2.1 Add `app/angle_units.h` (design decision 4). Make the optimizer file include it instead of defining the constants. Run the step checks. Commit.
- [x] 2.2 Move `Blocks`, `SumBlocks` and `pose_band_to_sparse` to `app/normal_equations.{h,cpp}` (design decision 3).
  - Add `tests/normal_equations_test.cpp`:
    - `pose_band_to_sparse` of a matrix whose pose part has entries only within `max_offset` of the diagonal, plus full extra rows/columns, equals `sparseView()` of it;
    - entries outside the band are dropped;
    - `SumBlocks` adds each block at its offset and subtracts the right-hand side.
  - Compile `app/normal_equations.cpp` into the tests. Run the step checks. Commit.
- [ ] 2.3 Move the IMU kinematics helpers to `app/imu_kinematics.{h,cpp}`. Add `tests/imu_kinematics_test.cpp`, which builds synthetic trajectories in code:
  - `exp_so3`: the zero vector gives the identity, and (0, 0, π/2) gives a 90° turn about z;
  - `remove_gyro_bias_from_rotations`:
    - a trajectory turning at a constant rate b about z, with bias b, becomes non-rotating;
    - node 0 and all translations stay unchanged;
    - a zero bias leaves the trajectory unchanged;
  - `world_acceleration`, with identity orientations and level AHRS angles:
    - an accelerometer reading of `gravity_norm · z` gives zero;
    - adding a known acceleration gives that acceleration in m/s²;
    - with `half_width_s > 0`, the result is the average over the window;
  - `observed_acceleration_offset`, on a constant-velocity trajectory with the accelerometer at gravity + e: the returned offset ≈ e. It returns false for a window shorter than 2 × 20 + 1 nodes;
  - `is_window_stationary`:
    - at rest, it returns true;
    - one acceleration above the threshold gives false;
    - a rotation rate above the gyro threshold over ≥ 20 ms gives false.

  Compile `app/imu_kinematics.cpp` into the tests. Run the step checks. Commit.
- [ ] 2.4 Move `MotionModelRotationOptions`, `MotionModelTranslationImuOptions` and `get_motion_model_rotation_options` to `app/optimizer_options.{h,cpp}` (design decision 5). Add a test case to `tests/optimizer_options_test.cpp`: every mapped field gets a distinct value, and the degree-to-radian conversion is checked. Run the step checks. Commit.
- [ ] 2.5 Move `optimize_lidar_odometry` to `app/scan_to_map.{h,cpp}`. Its declaration leaves `lidar_odometry_utils.h`, and the 5b forward declarations go away. Run the step checks. Commit.
- [ ] 2.6 Move `optimize_sf2` and `optimize_rigid_sf` to `app/robust_lidar_odometry.{h,cpp}`. Both are declared there. Run the step checks. Commit.
- [ ] 2.7 Move `Consistency` and `Consistency2` to `app/consistency.{h,cpp}`. `app/lidar_odometry.cpp` includes the new header. Run the step checks. Commit.
- [ ] 2.8 `git mv app/lidar_odometry_utils_optimizers.cpp app/run_odometry.cpp` and add `app/run_odometry.h` with `compute_step_2`'s declaration. Callers include it. Update `CMakeLists.txt`.
  - Verify that `git log --follow app/run_odometry.cpp` shows the old history.
  - Verify that `wc -l` of every new or remaining file is ≤ 900.
  - Verify that `lidar_odometry_utils.h` no longer declares any of the moved functions.

  Run the step checks. Commit.

## 3. Integration

- [ ] 3.1 Full 44 in the background. Verify IDENTICAL with `tools/baseline/44.manifest.json`, or else that `compare_baseline.py --tolerance 1e-6` against the refreshed `raw data/44_baseline` passes. Report which.
- [ ] 3.2 Docs. Commit.
  - `CLAUDE.md`: pipeline step 4 and the Testing list name the new files; drop `optimize_icp`.
  - `docs/ARCHITECTURE.md`:
    - roadmap step 6 is split into 6a ✅, 6b (window loop into named steps) and 6c (synthetic scan-to-map test);
    - the §4 item about the 3,000-line file is marked solved.
  - `docs/PIPELINE.md` and the five `where` lines of `docs/index.html`: the new file names.
- [ ] 3.3 Run `openspec validate split-optimizer-file --strict` and confirm it passes.
