# Unitree L2: SLAM Tuning Reference

Reference data for tuning the SLAM algorithm's `settings/l2-vertical.toml` to the Unitree L2 as recorded by dLidar. It collects the sensor specs, the operating environment, the parameters we want to tune, and how dLidar's output matches up with them.

## Hardware Specifications (Unitree L2)

| Property | Value |
|---|---|
| Sensor type | 4D ToF solid-state, non-repetitive hemispherical scan |
| Effective point rate | 64,000 pts/s (sampling frequency 128 kHz) |
| Horizontal rotation | 5.55 Hz (~180 ms per full rotation) |
| Vertical mirror frequency | 216 Hz |
| Horizontal FOV | 360° |
| Vertical FOV | 90° (standard), up to 96° (NEGA mode) |
| Accuracy | ≤ 2 cm |
| Distance resolution | 4.5 mm |
| Minimum range / blind zone | 0.05 m |
| Maximum range | 30 m @ 90% reflectivity, 15 m @ 10% reflectivity |
| IMU | 6-axis (3-axis accel + 3-axis gyro) |
| IMU sampling / output rate | 1000 Hz sampling, 500 Hz output |
| Point density | Non-uniform, dense near the optical axis (non-repetitive pattern) |

## Environment & Platform

- **Environment:** cave and outdoor
- **Platform:** handheld
- **Mounting:** vertical (see [dLidar Output Conventions](#dlidar-output-conventions))

## Goals

- Minimize drift.
- Correctly compensate motion distortion (deskewing).
- Produce a clean map.

## Parameters to Tune in `l2-vertical.toml`

1. **VoxelGrid / downsampling**
   - Leaf sizes for feature extraction and feature matching, chosen from the 64k pts/s density and the 0.05 m close-range capability.
2. **Distance culling (blind zone & max range)**
   - Min cutoff around 0.1–0.2 m, to reject self-reflections off the housing.
   - Max cutoff set to the realistic range (30 m at best, 15 m on dark surfaces).
3. **IMU & point cloud deskewing**
   - IMU noise parameters: accelerometer noise, gyroscope noise, and bias random walk for the 500 Hz output.
   - Timestamp sync: time offset and extrinsic calibration between the LiDAR and IMU frames.
4. **Point-LIO / Fast-LIO specifics**
   - Handling of non-repetitive scanning (N-scans vs. continuous-time processing).
   - Registration iteration count and covariance thresholds, consistent with ≤ 2 cm measurement noise.
