## Why

The odometry turns the map by about −4° between 30 s and the gallery of recording 44 (façade gallery − start: −3.9 … −4.7° over s0–s5, odometry alone), and loop closure cannot place that error (the gallery is walked out and back through the same places). Global geometry is one of the three equal map-quality measures. The earlier investigation (branch `odometry-map-horizon-wip`, tasks 2.5) put the cause on the registration / map side: with a 100× weaker IMU yaw prior the LiDAR side alone drifts −7° per 50 s; finer cells remove about 25 %, ignoring grazing points about 20 %; ruled out were grid orientation, insertion order, range, sparse cells, de-skew timing and the gyro-bias double subtraction. Untested are the map-model findings of the code review: the coarse 0.4 m grid at full weight (C18), the fractional degenerate projection that acts in corridors (C16), the legacy cell statistics on top of the fixed cell index (C20 after C10), and the initial map placed with one pose (C21; the start's façade line already turns by about 1° within the first 10 s).

## What Changes

- **Phase 1, screening (settings only, no code):** five odometry-only variants on the exact 95 s cuts of 44 (`44c95b_s0..s2`, cut from −10 s, which reproduce the full run's windows exactly), against master on the same cuts, with a rule written before the runs.
- **Phase 2:** a proper fix for each mechanism that moves the angle (a code change where the setting is only a probe), measured on full runs by the map-quality rule (all three measures), with the user's visual check before a push.
- Mechanisms that don't move it are recorded and closed in the tracker.

## Capabilities

### New Capabilities

### Modified Capabilities
- Decided in phase 2 (likely `ndt-reference-map` and `lidar-odometry-motion-model`).

## Impact

- Phase 1: job-tmp settings files and runs only.
- Phase 2: `src/domain/scan_to_map.cpp`, `src/domain/ndt_map.cpp`, `src/application/run_odometry.cpp` / `lidar_odometry.cpp` depending on the outcome; baselines.
