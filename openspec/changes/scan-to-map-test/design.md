# Design

## Context

`optimize_lidar_odometry` (`app/scan_to_map.cpp`) runs one Levenberg–Marquardt step for one window.

**Inputs:**
- the window's points, each in the frame of its node (`Point3Di::point`, `index_pose`);
- the `Window` with its `poses` (the current estimate), `motion_model_poses` (the guess the motion-model priors refer to), `timestamps` and `gyro_bias`;
- the indoor and outdoor NDT maps (`NDTBucketMapType`), built with `update_rgd` (`app/lidar_odometry_utils.cpp`);
- the option structs from `app/optimizer_options.h`.

**Cost terms:**
- point-to-NDT terms for each point that falls into a map bucket. The fine indoor grid is used beyond `min_distance`; the outdoor grid is used beyond `min_distance_outdoor_grid` when `use_hierarchical_rgd` is on. With `use_view_point_and_normal_vectors`, a bucket is used only if its normal faces the node;
- relative-pose priors between consecutive nodes, taken from `motion_model_poses`. With `time_scaled_sigma` the rotation sigmas are time-scaled, and with `estimate_bias` a gyro-bias term enters them;
- a fix-origin prior that holds node 0 where it is (sigma 1e-6);
- optionally the bias prior, the velocity prior and the accelerometer term.

**Outputs:**
- `poses` updated; the update is rejected for a node that would move more than 1 m from its `motion_model_poses`;
- `gyro_bias.mean` updated;
- `delta`, the sum of the update's absolute values;
- optionally `lidar_translation_information`, the sum of `n·nᵀ` over the matched surface normals.

The loop around it is `run_lm_iterations` (`app/run_odometry.cpp`, file-local). The damping starts at `lm_factor_initial = 5` and is multiplied by 10 every 10 iterations, for up to `nr_iter = 100` iterations or until `delta` falls below `convergence_delta_threshold`.

**Symbols (checked with `dumpbin /symbols` on the Debug objects of `scan_to_map.cpp` and `lidar_odometry_utils.cpp`):**
- the only project symbols they need are `get_rgd_index` and `pose_band_to_sparse`; everything else comes from the C++ runtime;
- `get_rgd_index` is defined in `lidar_odometry_utils.cpp` (and again in `core`'s `hash_utils.cpp`, which the test won't link);
- the headers they include (`ndt.h`, `structures.h`, observation-equation headers, `laszip_api.h`, `Fusion.h`) contribute only declarations, inline functions or types;
- the test target already has all those include folders.

## Goals / Non-Goals

**Goals:**
- Test the optimizer against a known truth: a perfect guess, rotation drift, gyro-bias recovery, and the degeneracy information.
- Keep the tests fast (seconds in Release), deterministic (`multithread = false`), and independent of the shipped settings files.
- Don't change app code. The test target only gains sources.

**Non-Goals:**
- Robust mode (`optimize_sf2`, `optimize_rigid_sf`), the accelerometer term and the velocity prior. They can get tests later; this change covers the default path of `settings/lidar-vertical.toml`.
- Linking `core`, or any change to `CORE_SOURCE_FILES`.
- Testing the whole window loop (`optimize_windows`). It needs the `PointStore` and the map state, and belongs to a later step.

## Decisions

1. **Scene: a closed box room, 8 × 5 × 3 m.**
   - The six faces are sampled on a 5 cm grid, with a deterministic ±5 mm jitter (a fixed-seed `std::mt19937`), so the NDT buckets are flat but not degenerate.
   - Both maps are built with `update_rgd`, with the viewport at the centre of the room:
     - indoor at 0.15 m resolution;
     - outdoor at 0.4 m resolution.
   - The **corridor** scene is the same room without its two end walls, and 30 m long. That makes it open along x.
   - Alternative: random planes or boxes. Rejected, because a box room gives a known, easy-to-read answer for the degeneracy test.

2. **Window: 50 nodes, 10 ms apart (0.5 s).**
   - The window moves along x at 1 m/s, starting at (−1, 0, 1.5) m, while yawing at 20°/s, so the rotation is observable.
   - Each node sees about 60 scene points, a deterministic subset chosen differently for each node: every k-th point, offset by the node index, within 0.15–25 m. They are given in the node's frame: `true_pose⁻¹ · p`.
   - `timestamps` are set, so the time-scaled sigmas and the bias term are active.
   - About 3,000 points and 300 unknowns keep one iteration in milliseconds.

3. **Options from a default `OdometryConfig`, with the `lidar-vertical.toml` values set in the test.** These include:
   - the grids;
   - `min_distance_lidar = 0.15`, `max_distance_lidar = 25`, `min_distance_lidar_outdoor_grid = 1.0`;
   - the motion-model sigmas (1e-4 m, 0.1/0.1/0.15°), and the fix origin at 1e-6;
   - the ARW values with k = 4 and the 0.002° floor;
   - the bias prior sigma of 10°/s;
   - hierarchical RGD and the view-point check on, as in the defaults.

   The options are built with the app's `get_scan_to_map_options`, `get_motion_model_options` and `get_motion_model_rotation_options` (`optimizer_options.cpp` is already in the test target), so the test exercises the same mapping from settings to options as a run.

4. **An iteration helper in the test copies `run_lm_iterations`:** `lm_factor` starts at 5, ×10 every 10 iterations, 30 iterations, `multithread = false`. It returns the last `delta`. Copying the ten-line loop is preferred over exposing `run_lm_iterations`, which needs `OdometryState`, `AccTerm` and a logger. A `NullLogger` like the one in `recording_files_test.cpp` takes the "big jump" messages.

5. **The drift is built as a gyro bias makes it.**
   - The guess starts at the true node 0 (the fix origin keeps node 0 there) and has the true translations. Its yaw grows by `b · t`, with `b = 0.5 °/s` about the vertical axis, and `motion_model_poses` is set to that guess.
   - **Without bias estimation:** the time-scaled rotation sigmas allow the LiDAR terms to rotate the nodes back. The test checks the heading error at the last node against the guess's (decision 6).
   - **With bias estimation:** the bias is a window unknown. The term models `measured = true + b · dt` (`scan_to_map.cpp`, the rotation rows of the relative-pose prior), so the estimate should approach +0.5 °/s about the vertical axis. The prior is zero mean with 10 °/s sigma, which hardly pulls it back.
   - The sign convention is checked when this is implemented. If the test shows the opposite sign, the convention is documented in the test, not "fixed" in the optimizer.

6. **The thresholds are targets, not tuned until they pass:**
   - perfect guess: within 1 mm / 0.01°;
   - drift: the end heading error at most 10 % of the guess's;
   - bias: within 20 % of the injected value;
   - degeneracy: an eigenvalue ratio of at least 100.

   If a target is missed, implementation pauses and the numbers are reported: it may be a real property of the optimizer, for example the 1e-4 m translation priors. The user decides whether to accept a different target or to look into the optimizer.

7. **No app change, so no full 44.** `dMapper` is rebuilt from unchanged sources, so a `44_short` IDENTICAL check is enough to show it. This is the usual rule's exception for test-only changes, stated here so it isn't silently skipped.

## Risks / Trade-offs

- **[A target fails for a real reason]** → decision 6: pause and report. The test is meant to find this.
- **[The bucket mean is the bucket's first point]** `update_rgd` never updates `mean` after the first point. The point-to-bucket residual therefore has a bias of up to half a bucket inside the plane, but none across it, because the covariance flattens across the plane. → The thresholds are about the poses, and the in-plane bias is mostly absorbed by the flat covariance. If it limits the 1 mm target, that is reported as a finding (decision 6), not hidden.
- **[Debug run time]** Unoptimized Eigen is maybe 20× slower. → The scene is kept small (decision 2). If Debug `ctest` takes over 60 s, the point count is reduced.
- **[The 1 m jump guard]** Nodes are rejected if the update moves them more than 1 m from `motion_model_poses`. The drift is at most 0.25° here, so this never triggers.
