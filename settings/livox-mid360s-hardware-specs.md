# Livox Mid-360S: SLAM Tuning Reference

Reference data for `settings/livox-vertical.toml` (Livox Mid-360S, vertical handheld mount). The sources are the spec sheet (https://www.livoxtech.com/mid-360s/specs) and the user manual (Livox Mid-360S User Manual, section 3.5 "Output Data"). The TOML is **provisional**: the values marked TODO in it are measured on the first Mid-360S recordings.

## Hardware Specifications (Livox Mid-360S)

| Property | Value |
|---|---|
| Point rate | 200,000 pts/s (first return) |
| Frame rate | 10 Hz (typical) |
| Horizontal FOV | 360° |
| Vertical FOV | −7° … 52° (typical) |
| Range | 40 m @ 10 % reflectivity (typical); 100 m limit |
| Minimum range / blind zone | 0.1 m |
| Range precision (1σ) | ≤ 2 cm @ 10 m; ≤ 4 cm @ 0.2 m |
| Angular precision (1σ) | < 0.15° |
| IMU | ICM40609 (3-axis accelerometer + 3-axis gyroscope) |
| IMU output rate | 200 Hz (default after power-on) |
| IMU axes | the same as the point cloud axes |
| IMU position in the point frame | x = 11.0 mm, y = 23.29 mm, z = −44.12 mm |
| Timestamps | 64-bit, ns; synchronised over IEEE 1588-2008 (PTPv2) or GPS (PPS + GPRMC) |
| Interface | 100BASE-TX Ethernet |
| Power | 9–27 V DC (12 V recommended), 6.5 W typical, 14 W peak when self-heating at −20…0 °C |
| Weight / size | 265 g, 65 × 65 × 60 mm |

## Point data (user manual 3.5)

- **No return:** a point without a return, or beyond the detection range (e.g. > 100 m), is output as (0, 0, 0). dMapper's inner xy filter (`filter_threshold_xy_inner`) drops it.
- **Reflectivity:** 0–255. Values 0–150 are 0–100 % reflectivity (Lambertian); 151–255 are retroreflective targets. Below 2 m the reflectivity error can be large.
- **Tags:** 8 bits giving the confidence that a point is affected by atmospheric particles (rain, fog, dust), by dragging noise between objects, or by other effects. 0 means a normal point.
- **Effective range across the vertical FOV:** shorter near the upper edge and longer near the lower edge. The 40 m @ 10 % holds near the lower edge.

## What the settings take from this

| Setting | Value | From |
|---|---|---|
| `threshold_nr_poses` | 72 | 0.36 s windows at the 200 Hz IMU |
| `handoff_tail_nodes` | 6 | 30 ms at 200 Hz |
| `startup_gyro_bias_nr_samples` | 200 | 1 s at 200 Hz |
| `filter_threshold_xy_outer` | 40 m | range at 10 % reflectivity |
| `filter_threshold_xy_inner` | 0.35 m | the operator's body; also drops (0, 0, 0) points |
| `lidar_imu_lever_arm_*` | TODO | the IMU position above, in the vertical mount's output axes |

## Open (measured on the first recordings)

- **The lever arm in the output axes:** needs the vertical mount's axis mapping. Check the sign on a turn on the spot.
- **Gyro angle random walk per axis, and accelerometer noise density:** from a static recording on a table.
- **The start-up bias variance gate:** from the gyro noise at rest.
- **`decimation`:** for the higher point rate.
