# Task for the dLidar agent: verify and fix the LiDAR–IMU time alignment of Unitree L2 recordings

You maintain dLidar, the app that records a Unitree L2 (LiDAR with a built-in 6-axis IMU) into recording folders. A separate program, dMapper (LiDAR-inertial odometry, another repository; don't change it), processes those folders. dMapper's analysis shows that **the IMU timestamps in the recordings are tens of milliseconds behind the LiDAR point timestamps**. Find out why, fix it in dLidar, and prove the fix.

## What dMapper reads (keep this format)

- `lidar000N.laz`: one record per point. `gps_time` is read as seconds, on the same zero as the IMU `timestamp`. `intensity` and `user_data` (the LiDAR id) are also read.
- `imu000N.csv`: space-separated, header `gyroX gyroY gyroZ accX accY accZ imuId timestamp timestampUnix`.
  - Gyro in rad/s, accelerometer in g.
  - `timestamp` in ns; `timestampUnix` in ns. The timestamps must be strictly increasing.
- Files are paired by sorted order and by the last four characters of the name (`imu0001.csv` ↔ `lidar0001.laz`).
- dMapper assumes that for one physical instant, IMU `timestamp` / 1e9 equals the points' `gps_time`. Nothing in dMapper corrects an offset between them.

## Evidence (measured on the user's recordings)

- **Recording 44** (276 s walk, recorded 2026-10-01): the IMU sample that shows a motion carries a timestamp **60–80 ms later** than the LiDAR points that show the same motion. Three independent estimates agree:
  - phase of the cross-spectrum between the accelerometer and the acceleration of the LiDAR trajectory (0.5–4 Hz): 66 / 74 / 70 ms (x / y / z);
  - time-domain cross-correlation (0.25–2 Hz): 53 / 80 / 84 ms;
  - the time shift that lets the accelerometer best predict the LiDAR trajectory 0.4 s ahead: +70 to +105 ms. Shifting the other way makes the prediction worse.
- **Over the recording** the lag stays at 60–80 ms (60 s segments), and may shrink to about 50 ms over the last minute.
- **Recording 43** (short shake test, recorded 2026-09-30): 17–39 ms, same direction. The lag differs between sessions (or app versions), so it is a latency in the recording chain, not a fixed property of the sensor.
- **The IMU timestamps themselves are very regular:**
  - 99.9 % of the spacings are 1.12 ms: 889.7 Hz in 44 (spacing p1–p99 1.110–1.131 ms), 892.7 Hz in 43 (0.42–1.49 ms);
  - 32 gaps above 5 ms and one above 20 ms in 44;
  - `timestampUnix − timestamp` is exactly constant.
- **The rate matches neither data-sheet value:** the L2 data sheet says 1000 Hz internal sampling and 500 Hz output, the recordings have about 890 Hz.
- **Consequence:** at walking frequency (1.2–2 Hz), 70 ms is 30–50° of phase. The accelerometer then contradicts the LiDAR by about two thirds of the walking acceleration, and the gyro's attitude belongs to 70 ms earlier than the points it is used to place. Every attempt in dMapper to use the accelerometer, or to trust the gyro more, made the maps worse by metres.
- **Re-stamping test in dMapper (recording 44, IMU stamps minus 80 ms, everything else unchanged):**
  - the residual offset drops to 0–9 ms;
  - the gyro's per-window yaw now agrees with the LiDAR's: 0.86° mean difference instead of 1.38°, no window above 3° instead of 5. Scanning the shift gives a clean minimum at 80–100 ms;
  - dMapper's accelerometer variant went from 22 m off with a 641 m route and 10 loop closures to 1.0 m off with a 554 m route and 18 loop closures;
  - a 1 ms re-stamp, as a control, changed nothing (21 loop closures, as before).

  So the timing is the cause of the accelerometer's failure. dMapper's current settings were tuned on lagging data and will be retuned after your fix.

## 1. Verify: how is each timestamp made?

Document precisely, with file and line references:
1. **Point `gps_time`:**
   - which SDK field it comes from (packet or frame stamp, per-point relative time);
   - which clock (sensor or host);
   - which convention (start or end of the frame or packet, the sign and base of per-point offsets);
   - every conversion until the LAZ is written.
2. **IMU `timestamp` and `timestampUnix`:**
   - SDK stamp or host time;
   - any re-gridding to a fixed period, interpolation, resampling, filtering, duplication, buffering, queue or thread hand-off;
   - where the ~890 Hz rate comes from.
3. **The common zero:** how the two streams' zeros are aligned when a recording starts.
4. **Direct measurement:** if possible, log for one minute the raw SDK stamps of IMU packets and point packets together with the host receive time of each. Report the latencies of both streams and the offset between their clocks.

## 2. Likely causes to check

- One stream is stamped with host receive time, or with an anchor taken from host time at the first packet, and the other with sensor time.
- Point times are based on the frame or packet *end*, or per-point relative times are applied with the wrong sign or base. The L2's rotation period is about 180 ms, so frame-convention errors have the observed size.
- IMU samples are buffered and stamped when released, or re-gridded to a fixed 1.12 ms period from a start anchor. Re-gridding would also explain the slow drift if the true period differs slightly.
- A low-pass or smoothing filter on the IMU adds group delay.

## 3. Fix

- Write the IMU `timestamp` and the points' `gps_time` from **one clock**, preferably the sensor's hardware stamps.
  - Each value is the measurement time of that sample or point, not the arrival or send time of its packet.
  - Keep `timestampUnix` = `timestamp` + one constant per session.
- Don't resample, interpolate or filter IMU samples. Keep their real sample times; irregular spacing is fine.
- Keep the timestamps strictly increasing.
- Keep units, columns, axis conventions and file naming as they are.
- If the sensor can't provide one clock for both streams: measure the offset per session and correct it before writing. Record the method and the applied offset in the recording (for example a small metadata file), and say so in your report.
- Add a test of the timestamp conversion (unit test with synthetic or recorded SDK packets).

## 4. Acceptance

- **Report:** the root cause with file and line references, and the change.
- **Recordings with the fixed app:**
  - one walking recording of at least 60 s, starting with 3–5 s at rest (as the L2 data sheet asks), with turns and normal walking;
  - one rotation test: turn the scanner back and forth about its vertical axis a few times in a room.

  The user will check them with dMapper's analysis. The target is a residual offset below 5 ms on every axis, the same over the whole recording.
- **Your own check, if you can:** from the rotation test, estimate the rotation rate the LiDAR sees, for example by aligning 20 ms slices of the points projected onto the horizontal plane against each other. Cross-correlate it with the gyro's z rate and report the lag.
- **Format:** no changes beyond the above. If anything about the files must change, list it so dMapper can follow.
