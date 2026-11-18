# Region: front-end

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

The front-end turns a recording folder into the windows the odometry works on: it reads the LiDAR and IMU chunks, runs
the AHRS over every IMU sample, cuts the IMU track into windows of a fixed number of nodes, and attaches every point to
a node. It also owns the IMU kinematics helpers that the window loop and consistency call.

## Contract

**Takes**
- The recording through the `RecordingSource` port (`input_ports.h`), built by `make_folder_inputs`
  (`folder_inputs.cpp`) on the pairs of `pair_recording_files` (`recording_files.cpp`): files sorted, `.laz`/`.las` and
  `.csv` paired by the last four characters of the stem; a missing partner in the middle fails, a missing last partner
  is ignored with a message.
- `OrientationEstimator` and `ParallelExecutor` from `RunServices` (`service_ports.h`).
- `load_imu` reads the legacy CSV format only when there is no `timestamp` column, and keeps a legacy line only if
  its `imuId` is the one in use. `calculate_trajectory` gets `imu_data` already merged and sorted by `load_data`; it
  does not check that the timestamps strictly increase, it only guards `dt <= 0`.
- Settings from `OdometryConfig`: `filter_threshold_xy_inner/outer`, `lidar_imu_lever_arm_*`, `fusionConvention*`,
  `ahrs_gain`, `use_remove_imu_bias_from_first_stationary_scan`, `startup_gyro_bias_nr_samples`,
  `startup_gyro_bias_max_variance_deg2_s2`, `threshold_initial_points` (60 000 in both sensor settings; built-in
  default 10 000), `threshold_nr_poses` (360 in `l2-vertical.toml`, 180 in `livox-vertical.toml`, 300 in
  `default_settings.toml`), `decimation` (0.03 m in both sensor settings, 0.01 built in).

**Gives** (stages 1–3 of `OdometryPipeline::load`, in this order)
1. `load_data` (`lidar_odometry.cpp`):
   - `Imu` (`imu_sample.h`): one `ImuSample` per CSV row; `timestamps.first` = IMU time [s] (CSV ns / 1e9),
     `.second` = unix time [s]; `gyro` [rad/s] and `acc` [g], float, body frame (NWU output axes of the recorder).
     The chunks are merged and sorted by time.
   - `points_per_file`: `Point3Di` per chunk, sorted by `timestamp` (`std::sort`, not stable), in the sensor frame
     shifted by the lever arm (translation only, `load_point_cloud`). The xy radius filter acts on the raw point,
     before the shift; points with timestamp 0 are dropped. Point time (`gps_time`) and IMU time share one clock (the
     recorder's contract, memory `dlidar-recording-format.md`). The points arrive factory-calibrated: never re-apply the
     L2 calibration.
2. `calculate_trajectory`: `ImuTrajectory` (`window.h`, a `std::map` from IMU time to (4×4 pose, unix time)), one
   orientation per IMU sample, translation zero. The gyro goes to the filter in deg/s minus the start-up bias: the
   per-axis median of the first `startup_gyro_bias_nr_samples` samples if their summed variance is at most
   `startup_gyro_bias_max_variance_deg2_s2`, else 0 (a warning). The gain is `ahrs_gain` throughout, after Fusion's own
   3 s ramp from 10 (`FusionAhrs.c:22–27, 94–120`). The adapter (`FusionOrientationEstimator::update`) calls
   `FusionAhrsUpdate` with a zero magnetometer, so no heading reset during the ramp (D1). First step dt = 1/200 s,
   a dt ≤ 0 becomes 0.002 s; a repeated timestamp overwrites the map entry.
3. `compute_step_1`:
   - `state.initial_points`: the first `threshold_initial_points` + 1 points over the chunks (raw, sensor frame);
     the time of the last one starts the windows.
   - `Window`s of exactly `threshold_nr_poses` consecutive nodes (one per IMU sample); a remainder shorter than that is
     not used. Per node: `poses` = `motion_model_poses` = the AHRS pose, `timestamps`, `imu_angles` (Tait-Bryan
     om/fi/ka [rad]), `imu_acc` (raw [g], the sample at the node). `imu_motion_from_previous` = the IMU motion from the
     last kept window's last node to this window's first node (D5).
   - A window's points: from its first node's time up to the next window's first node's time (the last window: up to its
     last node). `index_pose` = the node just before the point (`lower_bound` − 1); a point more than 0.01 s after it
     is dropped silently, and so is a point exactly at the window's first node time (its index comes out as −1).
   - A window with ≤ 1000 points is dropped with a warning and counted (`silent events: windows dropped for too few
     points`).
   - The `PointStore` gets `WindowOriginal` (all kept points) and `WindowIntermediate` (`decimate` at `decimation`, in
     the sensor frame; all points if decimation is off), numbered by kept window.
   - `state.m_g = windows[0].poses[0]`.

**What the others rely on**
- The window loop (orchestrator, inertial): one node per IMU sample, node times strictly increasing, `imu_acc` in g.
  `prepare_run` takes the gravity norm as the mean |acc| of the first window: the recording starts at rest
  (`run_odometry.cpp:42–49`).
- lidar-matching and map: `index_pose` indexes the window's nodes; `WindowIntermediate` points are in the sensor frame.
- map: `seed_map` places `initial_points` with `m_g` (C21).
- inertial and back-end: `imu_kinematics` (`exp_so3`, `rotation_of`, `relative_rotation`,
  `remove_gyro_bias_from_rotations`, `world_acceleration`, `is_window_stationary`) is called from `window_steps.cpp`,
  `run_odometry.cpp` and `consistency.cpp`.

## Owns

- `src/application/lidar_odometry.{h,cpp}`: `load_data`, `calculate_trajectory`, `compute_step_1` (and the helper
  `roll_pitch_yaw_deg`). Three functions of this file are not this region's, by function entries in `owners.toml`:
  `run_consistency` (back-end), `save_result` and `save_trajectory_to_ascii` (orchestrator).
- `src/domain/imu_kinematics.{h,cpp}`: rotation chains, gyro-bias removal, world-frame acceleration, rest test.
- `src/domain/imu_sample.h`: `ImuSample`, `Imu`.
- `src/infrastructure/recording_files.{h,cpp}`: `pair_recording_files`.
- `src/infrastructure/recording_folder_source.{h,cpp}`: `RecordingFolderSource`, `load_imu` (CSV with header, or the
  legacy 7-column format), `load_point_cloud` (LASzip, lever arm, xy filter).
- `src/infrastructure/fusion_orientation_estimator.{h,cpp}`: the Fusion AHRS adapter.
- `src/infrastructure/folder_inputs.{h,cpp}`: `make_folder_inputs` (cache and preview folders next to the recording).
- Tests: `recording_files_test.cpp`, `orientation_test.cpp`, `fusion_orientation_test.cpp`, `imu_kinematics_test.cpp`,
  `window_cut_test.cpp`.

## How to check your work

- **Unit tests** (Release): `out/build/x64-Release/bin/lidar_odometry_tests.exe
  -sf="*recording_files_test.cpp,*orientation_test.cpp,*fusion_orientation_test.cpp,*imu_kinematics_test.cpp,*window_cut_test.cpp"`
  - `recording_files_test`: pairing in sorted order, missing middle / last partner, too few files; the xy filter drops
    (0, 0, 0) returns.
  - `orientation_test` (`-tc="orientation*"`): first convention set, the gain, the start-up bias in deg/s, time steps
    and one node per timestamp, and a turn during the filter's start-up integrated, not zeroed (D1).
  - `fusion_orientation_test`: the adapter equals Fusion with the public settings through the start-up.
  - `imu_kinematics_test`: `exp_so3`, bias removal, `world_acceleration`, `is_window_stationary`.
  - `window_cut_test` (`-tc="window hand-off*"`): chaining keeps the rotation between windows, the predicted guess keeps
    it, the points between windows belong to the earlier window's last node.
- **Replays do not exercise the front-end.** A capture holds the windows and points that `compute_step_1` made, so
  `python tools/run_replay.py` sees only a change to `imu_kinematics` (called inside the loop). A change to loading, the
  AHRS or the window cut shows only in the 44_short run of `python tools/step_checks.py` (and the orchestrator's full
  runs).
- **Log lines that are the region's:** per chunk `total - filter = kept` and `(points with 0 timestamp: n)`; `GYRO
  BIAS` and `start-up gyro bias: movement ...`; `number of poses`; `window slice n dropped`; `silent events: windows
  dropped for too few points`; with `log_stage_timing` the times of `load_data`, `calculate_trajectory`,
  `compute_step_1`.
- **No row of its own** in the per-term table, the `window noise` line or the window verdict. The verdict's bend
  readings compare the result with the guess, which starts from the AHRS orientation: a front-end fault shows there as
  bends.
- **Recording check:** skill `.claude/skills/verify-recording` with `python tools/imu_health.py <folder>` (rate, gaps,
  non-increasing timestamps, rest at start, gravity, bias, false gyro offsets, noise). Timing tools:
  `tools/imu_time_offset.py` (IMU lag against the LiDAR), `tools/restamp_imu.py` (corrects it), `tools/fix_imu_timestamps.py`
  (rebuilds a steady clock; does not align). `cut_recording` (CMake target) drops LiDAR points before the first IMU
  sample: cut from −10 s for a window grid identical to the full run (`tools/README.md`).

## Known flaws

- **C2** (verified in the tracker; partly fixed in the code). The heading reset during Fusion's start-up is gone
  (D1, 11bdd55, `fusion_orientation_estimator.cpp::update`), measured on the cuts s0–s5 without a verdict
  (`openspec/changes/imu-front-end/tasks.md`, task 2.3 open). Still true: the windows (and `m_g`, so the seed map) start
  inside Fusion's 3 s ramp with gain 10 → `ahrs_gain` and acceleration rejection off while initialising
  (`FusionAhrs.c:116–146`). The tracker row is not updated.
- **C19** (reported). `lidar_odometry.cpp::compute_step_1` (point loop after the window's nodes): a point takes the node
  before it, no interpolation; a point more than 10 ms after its node is dropped, with no counter. Recording 44 has 58
  IMU gaps of 3–12 ms and one of 0.40 s at 2.6 s. Also roadmap finding T2 (`docs/agents/roadmap.md`).
- **C21** (verified; crosses to map). `compute_step_1` collects the first 60 001 points (`threshold_initial_points`
  60 000 in the sensor settings, about 0.94 s on the L2) and `run_odometry.cpp::seed_map` places them all with one pose, `m_g` (the
  orientation at their end): no motion compensation.
- **C38** (reported; a fault of recording 44's data, the recording check's too). The gyro at 253–259.7 s disagrees with
  gravity and the LiDAR (raw gyro 89.6° over 255.86–259.93 s, gravity predicted 29.6° off; raw-scan ICP 0.3–2.9° per
  window against the gyro's 8–11°). Cause unknown; the gyro-bias carry (inertial) swings −0.86 → +0.48 °/s.
  `imu_health.py`'s false-offset check exists for this kind of fault (the user's L2 unit held false rates of 11–32 °/s).
  Evidence: `openspec/changes/map-precision-leaks/findings.md` F3.
- **Crossing findings owned elsewhere:** C14 and C30 (map: `decimate`, which `compute_step_1` calls in the sensor
  frame); C15 (inertial): its front-end parts are done in the code (the hand-off by D5, a203f1c/76e2922; dropped
  windows now warned and counted), but the tracker row still says "reported".
- **Not in the tracker** (code, 2026-11-18):
  - T1 (roadmap): no setting for an IMU–LiDAR time offset; 44 was restamped outside dMapper.
  - T3 (roadmap): first AHRS step 1/200 s and dt ≤ 0 → 0.002 s are fixed constants (`calculate_trajectory`).
  - `load_imu` reports missing CSV columns only when all three groups are missing (`recording_folder_source.cpp:62`);
    a partial header throws inside the row loop, which is caught and logged at info level, and the samples read so
    far are returned.
  - `compute_step_1` reads `windows[0]` with no check that a window was kept.

## To split in the first job

From `python tools/function_sizes.py src --min 80` (2026-11-18):

| Lines | Function |
|---|---|
| 180 | `compute_step_1` (`lidar_odometry.cpp:244`) |
| 143 | `calculate_trajectory` (`lidar_odometry.cpp:100`) |
| 110 | `load_imu` (`recording_folder_source.cpp:34`) |
| 100 | `pair_recording_files` (`recording_files.cpp:9`) |
| 89 | `load_point_cloud` (`recording_folder_source.cpp:145`) |

`save_result` (146) lies in this region's file but is the orchestrator's, as is `save_trajectory_to_ascii`;
`run_consistency` there is the back-end's.

## Experiments

- **2026-11-16** Dynamic AHRS gain (0 while |a| outside 0.9–1.1 g), made effective by e41bd08, on Livox 3 with Huber 3 +
  information scales 0.1: same odometry, loop-closure corrections up to 1.48 m (0.33 m without), route 355 vs 313 m.
  The switch and `set_gain` were removed. Tracker C3; memory `livox-state-2026-11-16.md`.
- **2026-11-10** The user's L2 gyro fault confirmed from the raw IMU alone (take2 148–150.5 s: gyro [32.1 −3.7 4.0] °/s
  with gyro and accelerometer noise at the rest floor, then snapping to ~0); the Livox Mid-360S showed no held offsets
  (0 of 12 and 0 of 71 quiet blocks). Memory `fast-turn-fragility-state.md`.
- **2026-10-18** Start-up gyro bias from a hand-held stand-still, online estimate started from it (branch
  `startup-gyro-bias`, 8190736): 44 s0/s1/s2 façade out −3.63/−2.81/−4.49 → −2.28/−3.45/−2.68°, stair max
  9.7/12.3/16.4 → 7.6/5.4/6.8 cm, railing NN 3.5/5.2/6.5 → 5.1/10.2/9.1 cm, revisit pairs 8/10/11 → 7/6/6 of 11.
  Mixed; parked until a new recording (`openspec/changes/startup-gyro-bias/results.md` on that branch).
- **2026-10-18** C38 analysis: gyro against gravity and raw-scan ICP at 253–259.7 s on 44
  (`openspec/changes/map-precision-leaks/findings.md` F3).
- **2026-10-17** D1 on the cuts `44c95b_s0..s5`: gallery − start A mean −4.30° (master −4.31°); no verdict written.
  D5 test: six chained windows at 1 rad/s lose 0.005 rad before, 1e-16 after; 500 points between windows kept.
  D2 pre-measurement: the ±0.1 g gate would zero the gain for 44.5 % of 44's samples (11.7 % on 45f).
  `openspec/changes/imu-front-end/tasks.md`.
- **2026-10-17** IMU timestamps shifted ±10 ms on the cuts s0–s2: gallery unchanged (timing ruled out); gyro z ×1.0067:
  +0.21°. Memory `fix-plan-state.md` (change corridor-heading, deleted).
- **2026-10-16** Hand-eye fit on 44: gyro bias +0.61/+0.46/−0.82 °/s, yaw scale −0.58 ± 0.25 %; the 44 start is
  hand-held (gyro sd ≈ 2 °/s in the first 3 s). Memory `fix-plan-state.md`.
- **2026-10-16** The start turns ≈ 0.95° between 0.9–5.9 s and 5.9–10.9 s on 44: not the 0.40 s IMU gap, not the start-up
  heading reset alone. `openspec/changes/odometry-map-model/design.md` ("After phase 2", check d).
- **2026-10-16** Stable sorts of the points by time (C14, change `deterministic-point-order`): no effect on run-to-run or
  cut-vs-full results; the cut difference came from `cut_recording` dropping points before the first IMU sample.
  `openspec/changes/archive/2026-10-16-deterministic-point-order/`.
- **2026-10-16** Initial-map size (C21): 10 000 / 120 000 points instead of 60 000, gallery − start A +0.48 / +0.26°
  against master, not monotone: does not count. `openspec/changes/odometry-map-model/design.md`.
- **2026-10-09** Recording 44's IMU lagged the LiDAR by 79 ms at the start and 52 ms at the end (`imu_time_offset.py`);
  restamped with `restamp_imu.py` (79.4 ms − 0.0983 ms/s · t). `tools/README.md`; memory `imu-lidar-time-offset.md`.

## Ideas rejected

- **Dynamic AHRS gain switch** (gain 0 outside 0.9–1.1 g): removed 2026-11-16 once it took effect (tracker C3).
- **Stable sorts for a deterministic point order** (C14): rejected 2026-10-16, code reverted (cc6481f).
- **Re-applying the L2's calibration in dMapper:** not needed, the recorder already applies the 8 factory parameters
  (memory `dlidar-recording-format.md`).

## Open questions

- D1's own verdict (imu-front-end task 2.3); the change imu-front-end is parked since 2026-10-17.
- The hand-held start-up bias (branch `startup-gyro-bias`): re-evaluate on a new recording.
- T1 time-offset setting, T2 point interpolation and a dropped-point counter, T3 real first dt (roadmap 2026-11-15).
- Settings counted in samples (`threshold_nr_poses` 360 on the L2, 180 on the Livox; `startup_gyro_bias_nr_samples`
  1000 / 500) instead of durations (roadmap phase 2).
- Recording format 2 (LAZ extra bytes): not yet checked to load unchanged on a real recording.
- Whether 44, with its data fault (C38), should stay a regression baseline only (verify-recording says so).

## See also

- [Regions](README.md)
- [Pipeline](../PIPELINE.md) (how the windows are cut)
- [Code-review tracker](../code-review-tracker.md)
- [Window chaining](../window-chaining.html)
- [tools/README.md](../../tools/README.md) (imu_time_offset, restamp_imu, cut_recording)
- [All documents](../README.md)
