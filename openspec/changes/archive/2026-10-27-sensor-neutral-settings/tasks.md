# Tasks

## 1. Rename the L2 settings (no numbers change)

- [x] 1.1 `git mv settings/lidar-vertical.toml settings/l2-vertical.toml`, `git mv tests/data/lidar_vertical.golden.toml tests/data/l2_vertical.golden.toml`, `git mv settings/lidar-hardware-specs.md settings/l2-hardware-specs.md`; verify: `git status` shows three renames
- [x] 1.2 Update the references: `tests/toml_golden_test.cpp` (case name, path, golden name), `tests/gui_view_model_test.cpp` (3 paths), and the comments in `tests/scan_to_map_test.cpp`, `consistency_test.cpp`, `loop_closure_test.cpp`, `window_cut_test.cpp`; verify: `grep -rn "lidar-vertical\|lidar_vertical" tests src` is empty
- [x] 1.3 Update `tools/compare_baseline.py` usage, and the `"params"` field in `tools/baseline/44.manifest.json` and `44_short.manifest.json`. In `CLAUDE.md`, `tools/README.md`, `docs/*.md` and `openspec/specs/*/spec.md`, rename only the `lidar-vertical` path to `l2-vertical` and change nothing else; verify: `grep -rn "lidar-vertical\|lidar-hardware-specs" --exclude-dir=3rdparty --exclude-dir=out --exclude-dir=changes .` is empty
- [x] 1.4 Build Release `dMapper` + `lidar_odometry_tests`, run ctest; verify: all pass. Commit "Rename lidar-vertical.toml to l2-vertical.toml"

## 2. Start-up gyro bias window and gate as settings

- [x] 2.1 Add `startup_gyro_bias_nr_samples = 1000` (int) and `startup_gyro_bias_max_variance_deg2_s2 = 5.0` (double) to `OdometryConfig` (`src/domain/odometry_config.h`), with neutral comments. Register both in `TomlIO::POINTERS` and the `madgwick_filter` CATEGORIES entry (`src/infrastructure/toml_io.h`); verify: builds
- [x] 2.2 Pass both into `calculate_trajectory` (`src/application/lidar_odometry.{h,cpp}`) after `use_remove_imu_bias_from_first_stationary_scan`. Replace `1000` with n, `500` with n / 2, and `5.0` with the gate, and guard with `imu_data.size() > n`. Update the callers `odometry_pipeline.cpp:62` and `tests/orientation_test.cpp:66`; verify: builds, `orientation_test` passes
- [x] 2.3 Add a test in `tests/orientation_test.cpp`: with n = 200 the bias is the median of the first 200 samples, and samples after 200 don't change it; with a gate below the samples' variance the bias is 0; verify: test passes
- [x] 2.4 Add both keys with the old values to `settings/l2-vertical.toml` and `settings/default_settings.toml`; verify: ctest shows only the two golden TOML tests failing (the expected diff: the two new keys)
- [x] 2.5 Commit the code change. Regenerate the golden TOMLs (`LO_UPDATE_GOLDEN=1`, ctest) in their own commit; verify: the golden diff is exactly the two keys, and ctest passes
- [x] 2.6 Run `44_short` with `settings/l2-vertical.toml` and compare with `tools/compare_baseline.py` against `tools/baseline/44_short.manifest.json`; verify: IDENTICAL

## 3. Sensor-neutral code comments and the L2 provenance

- [x] 3.1 In `src/domain/odometry_config.h` (lever arm, `gyro_arw_*`, `imu_accel_noise_density_mps2_sqrt_hz`, `imu_accel_floor_sigma_mps2`) and `src/infrastructure/recording_folder_source.cpp:192-194`, rewrite the comments to describe the quantity ("set per sensor in its TOML") and keep the values; verify: `grep -n -i "unitree\|\bL2\b\|43/static" src` is empty
- [x] 3.2 Move the provenance into `settings/l2-vertical.toml` comments (gyro ARW and accel noise measured at rest; accel floor from the data sheet `acc_cov`; lever arm with its native-frame values), and make `tests/scan_to_map_test.cpp:709` neutral ("values as in l2-vertical.toml"); verify: `l2_vertical` golden test still passes (comments aren't saved)

## 4. Provisional Livox Mid-360S settings

- [x] 4.1 Create `settings/livox-vertical.toml`, a copy of `l2-vertical.toml` with these changes:
  - a header "Livox Mid-360S, vertical handheld mount, PROVISIONAL until measured on the first recordings";
  - `filter_threshold_xy_outer = 40.0`;
  - `threshold_nr_poses = 72`, `handoff_tail_nodes = 6`, `startup_gyro_bias_nr_samples = 200` (IMU at 200 Hz, user manual §3.5);
  - the lever arm from the manual's IMU position (11.0, 23.29, −44.12) mm, in the point frame and with IMU axes = point axes. It is mapped to the output axes with the vertical mount's axis mapping that the user gives, using the same convention as `l2-vertical.toml`. A comment carries the native values and "sign to be checked on the first recording";
  - TODO comments on `gyro_arw_*`, the accel noise density and floor, `decimation`, and `startup_gyro_bias_max_variance_deg2_s2`.

  Verify: dMapper GUI/CLI loads it without error.
- [x] 4.2 Create `settings/livox-mid360s-hardware-specs.md`. Verify: file exists.
  - **From the spec sheet:** 200,000 pts/s, 40 m at 10 % (100 m limit), 0.1 m minimum range, FoV 360° × −7…52°, range precision ≤ 2 cm at 10 m, angular precision < 0.15°, 10 Hz frame rate, ICM40609 IMU, PTPv2/GPS sync, 265 g, 9–27 V (12 V recommended), 6.5 W.
  - **From the user manual:**
    - IMU at 200 Hz, with axes = point axes, at (11.0, 23.29, −44.12) mm;
    - no-return points output as (0, 0, 0);
    - reflectivity 0–150 diffuse and 151–255 retroreflective, unreliable closer than 2 m;
    - point tags (confidence for rain, fog and dust, and dragging noise);
    - ns timestamps.
  - **Open:** the noise values.
- [x] 4.2a Record that (0, 0, 0) points are dropped: a doctest case that `load_point_cloud`'s xy filter rejects a point at the origin with `filter_threshold_xy_inner > 0` (or a comment at the filter if the loader can't be tested without a file). Verify: test passes
- [x] 4.3 Add a golden test case for `settings/livox-vertical.toml` in `tests/toml_golden_test.cpp` (`livox_vertical.golden.toml`, generated with `LO_UPDATE_GOLDEN=1`); verify: ctest passes. Commit

## 5. Sensor-neutral user documentation

- [x] 5.1 `README.md`: replace L2 with general terms, remove recording-44 and L2-defect content, rename the settings file references, and add a "Supported sensors" section: any LiDAR+IMU writing `lidar*.laz` + `imu*.csv` in this format, with tuned settings `l2-vertical.toml` (Unitree L2) and `livox-vertical.toml` (Livox Mid-360S, provisional). Also say that `lidar-vertical.toml` was renamed. Verify: grep for "Unitree|L2|recording 44|44" hits only the settings and hardware-sheet names
- [x] 5.2 `docs/index.html`, `docs/performance.html`, `docs/degeneracy-information.html` and their `docs/hu/` counterparts: the same rewrite, with English and Hungarian in step. Verify: the same grep over `docs/*.html docs/hu/*.html` hits only settings or hardware-sheet names, and the pages open in a browser with their links working
- [x] 5.3 Code and test comments outside 3.1/3.2 that name the L2 or recording 44 as a sensor fact: make them neutral. Verify: `grep -rn -i "unitree\|\bL2\b" src tests` is empty. Commit "Sensor-neutral user documentation"

## 6. Final verification

- [x] 6.1 Release build, then `ctest --test-dir out/build/x64-Release`; verify: all pass (Release only: settings, a rename and two scalar settings, nothing that depends on the configuration)
- [x] 6.2 Run the full `44` with `settings/l2-vertical.toml` in the background, then `tools/compare_baseline.py` against `tools/baseline/44.manifest.json`; verify: IDENTICAL
- [x] 6.3 Update the memories that name `lidar-vertical.toml`, and add a feedback memory: Claude's scope is dMapper only, dLidar is not its concern; verify: MEMORY.md index updated
