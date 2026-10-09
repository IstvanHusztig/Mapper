# Task: fix the LiDAR–IMU time alignment in dLidar (Unitree L2 recordings)

You maintain **dLidar**, the app that records a Unitree L2 (LiDAR with a built-in 6-axis IMU) into recording folders. A separate program, **dMapper** (LiDAR-inertial odometry, another repository; don't change it), processes those folders. dMapper has measured that **the IMU timestamps in the recordings lag the LiDAR point timestamps by tens of milliseconds**. Find the cause in dLidar, fix it, and show that it's fixed.

**The scanner is currently broken and waiting for repair.** You can't make new recordings now. Work from the code, the SDK documentation, any raw packet captures or logs you have, and unit tests. Plan the field check for when the scanner is back.

## What dMapper reads (keep this format)

- `lidar000N.laz`: one record per point. `gps_time` is read as seconds, on the same zero as the IMU `timestamp`; `intensity` and `user_data` (the LiDAR id) are read too.
- `imu000N.csv`: space-separated, header `gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix`.
  - Gyro in rad/s, accelerometer in g.
  - `timestamp` and `timestampUnix` in ns, strictly increasing.
- Files are paired by sorted order and by the last four characters of the name (`imu0001.csv` ↔ `lidar0001.laz`).
- **dMapper assumes that for one physical instant, IMU `timestamp` / 1e9 equals the points' `gps_time`.** It doesn't correct any offset between them.

## Evidence

1. **Recording 44** (276 s walk, recorded 2026-10-01): the IMU sample that shows a motion carries a timestamp **79 ms later at the start and 52 ms later at the end** than the LiDAR points that show the same motion. Four independent measurements agree:
   - phase of the cross-spectrum between the accelerometer and the acceleration of the LiDAR trajectory (0.5–4 Hz): 66–74 ms;
   - time-domain cross-correlation: 53–84 ms;
   - the shift that lets the accelerometer best predict the LiDAR trajectory: +70 to +105 ms (the opposite shift makes it worse);
   - gyro: the LiDAR trajectory's per-window yaw agrees best with the gyro shifted by −80 to −100 ms (mean difference 0.86° instead of 1.38°).
2. **The lag shrinks steadily over the recording,** by 0.098 ms per second (±0.019), from 79 to 52 ms. That is a clock rate error of about 1e-4.
3. **Recording 43** (short shake test, 2026-09-30): 17–39 ms, same direction. The lag differs between sessions or app versions.
4. **What the stamps look like:**
   - Recording 44's IMU stamps are an almost perfect grid: 99.9 % of spacings 1.12 ms, p1–p99 1.110–1.131 ms, 889.7 Hz.
   - Recording 43's are bursty (p1–p99 0.42–1.49 ms).
   - `timestampUnix − timestamp` is exactly constant in both.
   - The L2 data sheet gives 1000 Hz sampling and 500 Hz output; ~890 Hz matches neither.
5. **History:** around 2026-10-01 the IMU stamps were known to be packet arrival times (bursts, then gaps). A repair (dMapper's `tools/fix_imu_timestamps.py`, and apparently a dLidar change around the same date) rebuilds a steady clock from the lower edge of the arrival times, with a period fitted over the file. **That removes the bursts but doesn't align the IMU with the LiDAR's clock.** The result keeps:
   - the IMU's minimum delivery latency relative to the LiDAR stamps: the constant part, 50–80 ms;
   - an error in the fitted period: the drift.

   This is the most likely explanation of points 1–4.
6. **Proof that it matters:** dMapper re-stamped recording 44's IMU by the measured lag.
   - The residual drops to 0–9 ms.
   - dMapper's accelerometer variant went from 22 m off with 10 loop closures to 1.0 m off with 18.
   - A 1 ms re-stamp, as a control, changed nothing.
   - Every IMU-based improvement from the literature had failed on the unsynchronized data, because at walking frequency (1.2–2 Hz) 70 ms is 30–50° of phase.

## 1. Verify: document how each timestamp is made

With file and line references:
1. **Point `gps_time`:**
   - which SDK field (packet or frame stamp, per-point relative time);
   - which clock (sensor or host);
   - which convention (start or end of frame or packet; the sign and base of per-point offsets);
   - every conversion until the LAZ is written.
2. **IMU `timestamp` and `timestampUnix`:**
   - SDK stamp or host arrival time;
   - the steady-clock reconstruction (algorithm, period estimate, anchor);
   - any buffering, queue, thread hand-off, resampling, interpolation, filtering or duplication;
   - where the ~890 Hz comes from.
3. **The common zero:** how the two streams' zeros are aligned when a recording starts.
4. **The packet headers:** whether the L2/SDK provides a sensor timestamp for IMU packets and for point packets on **one** clock. If it does, why isn't it used?

## 2. Fix

- Write the IMU `timestamp` and the points' `gps_time` from **one clock**, preferably the sensor's hardware stamps.
  - Each value is the measurement time of that sample or point, not the arrival or send time of its packet.
  - Keep `timestampUnix` = `timestamp` + one constant per session.
- If the sensor has no common clock:
  - align the IMU to the LiDAR explicitly;
  - estimate the IMU's period against the LiDAR's clock, not the host's, so there's no drift;
  - estimate the latency difference between the two streams;
  - record the method and the applied correction in the recording (for example a small metadata file).
- Don't resample, interpolate or filter IMU samples. Irregular spacing is fine if it is real.
- Keep the timestamps strictly increasing, and keep units, columns, axis conventions and file naming.
- **Tests:**
  - unit tests of the timestamp conversion with synthetic SDK packets: known sensor times, delayed and bursty arrival;
  - the written IMU and point times must agree to within 1 ms for the same instant, with no drift over 10 minutes of simulated data.
- If you have raw packet captures or logs from earlier sessions, replay them and report the latencies and the IMU–LiDAR offset before and after the fix.

## 3. Report back

- The root cause, with file and line references, and the change.
- Test results, and replay results if you had captures.
- Any change to the files (ideally none), so dMapper can follow.
- **A field check for when the scanner is repaired:**
  - a walking recording of at least 60 s, starting with 3–5 s at rest (as the data sheet asks), with turns;
  - a rotation test: turn the scanner back and forth about its vertical axis a few times in a room.

  The user will check them with dMapper's `tools/imu_time_offset.py`. The target is a residual offset below 5 ms on every axis and in every 60 s segment.
