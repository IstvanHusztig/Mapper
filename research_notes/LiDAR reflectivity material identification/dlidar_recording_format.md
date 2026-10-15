# What dLidar records from the Unitree L2

Answers from the Claude Code session working on the dLidar repository (2026-10-15), from the dLidar code (file:line refers to that repo) and from recording 0001 (lidar0001.laz, 11.29 M points, 190 s, recorded 2026-10-09). GUESS marks inferences of that session.

## Reading the L2
- Unitree unilidar SDK 2.0.4, a prebuilt static lib with headers only; repo/commit not recorded. UDP.
- dLidar runs its own modified copy of `parseFromPacketToPointCloud` (include/unitree_lidar_utilities.h:222-327) on raw packets. Vendor math unchanged: `range_float = range_scale*(ranges[j]+range_bias)`; alpha = `angle_min+alpha_angle_bias+j*angle_increment`, theta = `com_horizontal_angle_start+theta_angle_bias+j*com_horizontal_angle_step`; `a_axis_dist`/`b_axis_dist` offsets applied; `ring = 1` hard-coded.
- `setLidarWorkMode(0)` (GUESS: standard FOV, 3D, IMU on). No speed, power, gain or reflectivity commands. `range_scale`/`range_bias` are the per-packet values the sensor sends.
- Filters, in order: raw range 0; outside the packet's range_min/max; range < 0.1 m; range > 100 m; **intensity < 5 dropped** (kMinPointIntensity). No de-duplication.
- Firmware version not logged (`getVersionOfLidarFirmware` exists, never called).

## LAZ contents
- LAS 1.2, point format 1, scale 0.0001, offset 0, no VLRs, no extra bytes.
- `intensity` = the raw 0–255 byte, unscaled (exact integers 5..255).
- `user_data` = ring = constant 1. `classification`, `point_source_ID`, `return_number`, scan angle, flags: all 0.
- `gps_time` = seconds since the L2 stamp of the first point packet (not GPS time, although global_encoding says so). The IMU CSV `timestamp` is ns on the same clock and zero.
- Frame: the L2's calibrated sensor frame, remapped for vertical mounting (x = orig_z, y = orig_y, z = −orig_x; +X front, +Y left, +Z up); the IMU gets the same remap. **No mounting rotation or lever arm applied.** Point origin is the L2 model origin after the a/b axis offsets, so |xyz| is not exactly the measured range.
- One lidarNNNN.laz + imuNNNN.csv per recording.

## Available at parse time but discarded
- raw range (mm) and corrected range (m); scan angles alpha/theta; sample index j (0..299); packet seq (wraps at 1024; only in the diagnostics log packetsNNNN.bin, not per point).
- Per-packet `LidarInsideState`: **apd_temperature, apd_voltage, laser_voltage**, imu_temperature, **dirty_index**, packet_lost_up/down, rotation periods. All discarded.
- Storing them: LAZ extra bytes (LASzip `laszip_add_attribute`), `point_source_ID` (u16) could hold seq; `user_data` (u8) can't hold j. The temp file layout `points_tempNNNN.bin` would change (crash recovery needs a version check). Estimate about a day.

## Intensity info in SDK/repo
- Only `uint8_t intensities[300]; // Point Reflect [0-255]`. No Unitree PDFs, no ROS driver. Emitter count not in the code.
- Measured: 300 points per packet, time_increment ≈ 7.73 µs (≈129 kHz), one packet spans 2.32 ms but packets arrive every 4.64 ms (≈64.6k pts/s). GUESS: two interleaved sample streams, or every other sample dropped.

## IMU CSV
- Space-separated: gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix. Gyro rad/s, acc in g, same axis remap as points.
- Not resampled; measured ≈1000 Hz in rec 0001 (not the 500 Hz of the spec).
- The 50–80 ms lag in older recordings (host arrival time on the IMU) was fixed in dLidar commit d68191b (2026-10-10). In rec 0001 the IMU-minus-point minimum delivery latency is 1.6 ms; not yet verified against motion.

## Measured on recording 0001
- intensity: min 5, max 255, 251 distinct values; **37.8 % exactly 255**; p1 53, p5 109, p25 193, median 238, p75+ 255. 16-wide bins (%): 0-15 0.02, 16-31 0.18, 32-47 0.56, 48-63 0.70, 64-79 0.77, 80-95 1.31, 96-111 1.75, 112-127 2.20, 128-143 2.69, 144-159 3.42, 160-175 4.31, 176-191 6.23, 192-207 7.93, 208-223 8.92, 224-239 10.05, 240-255 48.97.
- range |xyz| 0.13–40 m: <0.3 m 0.09 %, 0.3-0.5 2.1 %, 0.5-1 17.4 %, 1-2 35.9 %, 2-3 21.2 %, 3-5 17.7 %, 5-10 4.0 %, 10-15 0.8 %, 15-20 0.7 %, 20-30 0.17 %, 30-40 0.04 %.

| range (m) | points | mean intensity | % at 255 |
|---|---|---|---|
| 0.10–0.30 | 9 849 | 144.5 | 0.0 |
| 0.30–0.50 | 233 364 | 158.8 | 0.0 |
| 0.50–0.75 | 975 707 | 172.7 | 0.1 |
| 0.75–1.00 | 990 822 | 190.1 | 0.3 |
| 1.00–1.50 | 2 236 358 | 221.0 | 23.1 |
| 1.50–2.00 | 1 813 773 | 229.5 | 60.9 |
| 2.00–3.00 | 2 392 285 | 231.5 | 62.1 |
| 3.00–5.00 | 1 995 470 | 227.7 | 50.7 |
| 5.00–7.50 | 394 180 | 208.0 | 23.5 |
| 7.50–10.0 | 51 799 | 197.9 | 21.6 |
| 10.0–15.0 | 91 488 | 215.0 | 25.3 |
| 15.0–20.0 | 80 260 | 199.0 | 14.3 |
| 20.0–30.0 | 19 048 | 200.9 | 18.9 |
| 30.0–45.0 | 4 743 | 192.4 | 7.7 |

One scene, target mix not controlled. 19 lost point packets out of 41 004.
