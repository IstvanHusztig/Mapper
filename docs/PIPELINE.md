# How a run works: windows

A run is cut into short consecutive pieces of the recording, called **windows**. This page explains what a window holds and how each processing stage uses it. The code is in `src/` (one folder per layer, see `docs/ARCHITECTURE.md`). For the parameters, see `docs/index.html`; for the target architecture, see `docs/ARCHITECTURE.md`.

## What a `Window` is

A `Window` (`src/domain/window.h`) holds `threshold_nr_poses` consecutive **trajectory nodes**, one per IMU sample:
- That is 360 nodes with `settings/l2-vertical.toml`, about 0.4 s of recording.
- `44_short` has 70 windows, and the full 44 recording has 690.

| Field | What it holds |
|---|---|
| `poses` | **The result:** the pose (position + rotation, world frame) of every node. The odometry optimizes these. |
| `motion_model_poses` | **The prior:** where the IMU says each node should be. The optimizer is pulled towards it with the motion-model sigmas (`[motion_model_uncertainty]`). |
| `timestamps` | Per node: (IMU timestamp, unix timestamp), in seconds. |
| `imu_angles` | The AHRS orientation (om, fi, ka) per node, in radians; written to `trajectory_lio_*.csv`. |
| `imu_acc` | The raw accelerometer per node, in g (body frame), for the optional acceleration term and the zero-velocity check. |
| `gyro_bias` | A `GyroBiasState`: this window's estimated gyro bias (`mean`, rad/s) and its uncertainty (`covariance`). |

**The points are not in the window.** They live in the `PointStore` (`src/application/input_ports.h`), under the window's index:
- the original points;
- a decimated "intermediate" set, which the optimizer uses.

On disk that is `<recording>/cache/original_points_<i>.cache` and `intermediate_points_<i>.cache`. So a window is small (poses and IMU data), while its heavy point data stays on disk.

The AHRS orientation track the windows start from is an `ImuTrajectory` (also in `src/domain/window.h`): IMU timestamp → (pose, unix timestamp).

## How a run uses the windows

```
recording ──► ImuTrajectory (AHRS orientation per IMU sample)
                │
  compute_step_1│  cut into windows of threshold_nr_poses nodes:
                │   poses = motion_model_poses = IMU-based start guess
                │   + timestamps, imu_angles, imu_acc; points -> PointStore
                ▼
  optimize_windows for each window i, in order:
                │   - start from the end of window i-1 (velocity, gyro bias as prior)
                │   - load its intermediate points, optimize the poses against the
                │     NDT map + motion model + gyro bias  -> poses, gyro_bias
                │   - re-propagate the last nodes (hand-off), add its points to the map
                ▼
  loop closure     groups windows into segments, finds revisits, corrects poses
  consistency      smooths the poses across windows (optional)
                ▼
  save_result      joins 51 windows into one output chunk:
                     points placed with the poses -> scan_lio_N.laz
                     poses relative to the chunk  -> trajectory_lio_N.csv
                     gyro_bias per window         -> results JSON (gyro_bias_per_window)
```

In more detail:
- **`compute_step_1`** (`src/application/lidar_odometry.cpp`):
  - It builds the windows from the `ImuTrajectory`.
  - Every point is attached to the node just before its timestamp (at most 10 ms), so each point gets its own pose (motion compensation). Windows with too few points are dropped.
  - The original and decimated points go to the `PointStore`.
- **`optimize_windows`** (`src/application/run_odometry.cpp`, steps in `src/domain/window_steps.cpp`) runs the windows in order:
  - The previous window's end gives the start guess and the velocity.
  - Its `gyro_bias` becomes the next window's prior (the prior's uncertainty grows with time).
  - `optimize_lidar_odometry` adjusts the window's `poses` against the NDT map of the points so far, the motion-model prior and the gyro bias.
  - Afterwards the window's last nodes are re-propagated from the IMU (`handoff_tail_nodes`), and its points are added to the map. The map is a sliding local map, emptied as the trajectory advances.
- **Loop closure** (`src/application/loop_closure.cpp`, with `use_loop_closure`):
  - It groups `loop_closure_segment_windows` windows into segments and looks for places visited twice.
  - Accepted loops correct the `poses` of all windows through a small pose graph.
- **Consistency** (`run_consistency`, with `use_consistency`) adjusts the `poses` of all windows together for point-cloud consistency and trajectory smoothness.
- **`save_result`**:
  - It concatenates the windows into output chunks of 51 windows each, so the output has far fewer files than windows.
  - It places each chunk's original points with the final poses, and hands everything to the `ResultSink`.
- **The GUI** draws the `poses` of every window as the trajectory. The live point-cloud preview places each finished window's stored points with its `poses`.

## Why windows at all

Optimizing all nodes of a run at once (248,000 on the full 44 recording) would be far too big a problem. Window by window, each optimization is small (360 nodes). The map built from the windows so far keeps them aligned, and loop closure removes the drift that remains.
