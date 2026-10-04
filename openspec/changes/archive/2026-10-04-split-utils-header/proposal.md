# Proposal

## Why

Roadmap step 7 moves the code into layer folders with CMake libraries whose links enforce the dependency rule. That only works if a domain file doesn't pull infrastructure in through its includes, and today almost every file does:
- `app/lidar_odometry_utils.h` is a catch-all header. It includes LASzip, Fusion, nlohmann json, `core`'s NDT and transformation headers, four observation-equation jacobians and the Cauchy kernel, and 10 app files (8 of them headers, which pass it on) and a test include it. Each file then compiles against all of that, whether it uses it or not.
- Through it, every user of `NDT::Bucket` or `NDT::GridParameters` also includes `core/include/ndt.h`, which includes `point_cloud.h`, which includes `GL/freeglut.h`, and `session.h`, which includes the GUI's manual loop closure. So the scan-to-map optimizer, which should be pure math, sees OpenGL.

This change cleans up the includes first, so that step 7b can move files and set up the libraries without rewriting code at the same time.

## What Changes

- **`lidar_odometry_utils.h` is replaced by three focused headers** in `app/`:
  - `ndt_map.h` / `ndt_map.cpp` (renamed from `lidar_odometry_utils.cpp`): the NDT map types (`NDTBucketMapType`, `NDTBucketMapType2`), `get_index`, `get_rgd_index`, `decimate`, `update_rgd`, `update_rgd_spherical_coordinates`;
  - `odometry_config.h`: `OdometryConfig`, `RunPaths`, `GuiPreferences`, `get_lidar_imu_lever_arm`, `get_software_version`;
  - `odometry_state.h`: `OdometryState`.
- **Every app file and test includes what it uses.** The third-party headers move into the files that need them, for example LASzip into the LAZ readers and writers, json into the result sink, and the jacobians into the optimizers. The `namespace fs` alias goes into the files that use it.
- **The NDT value types leave `ndt.h`.** A new light header, `core/include/ndt_types.h` (Eigen and the standard library only), defines `NdtGridParameters`, `NdtBucket`, `NdtBucketCoef` and `NdtBucket2` with the same fields. Class `NDT` keeps its names as aliases (`using Bucket = NdtBucket;` …), so `core` compiles unchanged. The app and the tests use the new names, and `decimate` gets its own index-pair struct instead of `PointCloud::PointBucketIndexPair`.
- `lidar_odometry_utils.h` is deleted; nothing includes it any more.
- **No behaviour change.** Settings, TOML files, output files and results stay the same. The precise build must stay bit-identical. The normal `/fp:fast` build must be IDENTICAL or within 1e-6.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None. This is a pure refactor (`skip_specs: true`).

## Impact

- **Code:** about 45 files in `app/`, `tests/` and `CMakeLists.txt` / `tests/CMakeLists.txt` (source list). In `core/`, only the new `ndt_types.h` and the aliases in `ndt.h`.
- **Not touched:** `input_ports.h` still includes `Fusion.h`, because the IMU port type uses Fusion (that is roadmap step 9). `lidar_odometry.h` (`save_result`) still includes `session.h` and `export_laz.h`; it belongs to infrastructure and is placed in 7b.
- **Docs:** `CLAUDE.md` (the file names in the architecture section and the tests' source list) and the step-7 entry in `docs/ARCHITECTURE.md`, which is split into 7a and 7b.
- **Follow-up:** step 7b, the `src/` layer folders and the `lo_domain` / `lo_application` / `lo_infrastructure` libraries, is proposed separately.
