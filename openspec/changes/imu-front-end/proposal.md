## Why

The code review (2026-10-13, `docs/code-review-tracker.md`) found five defects in how the IMU enters the odometry (plan change D):
- C6: the carried gyro bias is subtracted twice. The estimate is 30–50 % low on every axis: on 44 it reads z ≈ −0.45 °/s, while the gyro-against-trajectory fit gives −0.81 °/s.
- C5: the rotation prior is multiplied by a Cauchy density, `cauchy(x, 1) = 1 / (π (1 + x²))`. It is π× weaker than configured, and not robust in any useful way.
- C2: the odometry starts inside Fusion's 3 s initialisation, which zeroes the heading on every sample. The IMU prior of the first windows therefore says "no yaw".
- C3: `set_gain` writes a value Fusion no longer reads after its initialisation, so the dynamic AHRS gain has no effect.
- C15: windows don't share their boundary IMU sample, and each window's first node is pinned (σ = 1e-6) to the previous window's last pose. Every hand-off loses one IMU step of motion (1/360 of all rotation and translation).

What the F/E change measured (`openspec/changes/odometry-map-model/design.md`, "After phase 2"):
- the start's façade turns ≈ 0.95° between 0.9–5.9 s and 5.9–10.9 s;
- the bias estimate swings early (+1.03 °/s at 2 s, settled by ≈ 10 s; 44 starts hand-held, so the stationary-start bias is rejected and the estimate starts at 0 ± 10 °/s);
- the gyro-against-LiDAR fit suggests a yaw scale of −0.6 ± 0.25 % (unstable between halves).

These are correctness fixes. They are not expected to remove the ≈ 3° gallery turn of recording 44: an earlier investigation ruled out the gyro bias and the IMU prior as its cause, and the hand-off loss predicts ±0.3° with the wrong sign there.

## What Changes

- **D3 (C6):** the motion-model measurement subtracts only the bias update (estimate − carried), since the carried bias is already removed from the IMU increments.
- **D4 (C5):** the Cauchy factor goes. The rotation sigmas in the shipped settings are scaled by √π, so the effective prior stays as it was; a stiffer prior is a separate, measured probe.
- **D1 (C2):** the AHRS update no longer zeroes the heading during Fusion's initialisation. The gain ramp for gravity alignment stays.
- **D2 (C3):** the dynamic gain takes effect after the initialisation (through Fusion's settings API).
- **D5 (C15):** consecutive windows share their boundary IMU sample, so the pinned first node gets its own time's pose and no points are lost between windows.
- **Probes and diagnostics (no shipping by themselves):** a prior-strength dose on the test bed, and the gyro scale / misalignment fit on every recording (`hand_eye_gyro.py`).

## Capabilities

### New Capabilities

### Modified Capabilities
- `lidar-odometry-motion-model` (rotation prior weighting, gyro bias measurement) and the orientation estimation / window hand-off requirements, decided per fix in the design.

## Impact

- `src/domain/scan_to_map.cpp` (bias measurement, prior weight), `src/domain/window_steps.cpp` / `src/application/lidar_odometry.cpp` / `src/application/run_odometry.cpp` (hand-off, window cutting), `src/infrastructure/fusion_orientation_estimator.cpp` (heading reset, gain).
- `settings/lidar-vertical.toml`, `settings/default_settings.toml`, `src/domain/odometry_config.h` defaults (√π rescale), golden TOML files, unit tests.
- New baselines after each accepted numeric fix.
