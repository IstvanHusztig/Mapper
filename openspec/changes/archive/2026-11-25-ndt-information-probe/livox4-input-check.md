# Livox4 quick input check, 2026-11-13

User supplied C:/Users/User/Downloads/dLidar/raw data/livox/4 after recording a
larger indoor/outdoor route, starting and finishing on the same table in Wohnen,
with a slightly quicker walking pace. Asked to check for jumps before processing.
The intervening stop was honored; this check resumed only after the new path
was supplied. No dMapper replay was launched.

tools/imu_health.py: 183,137 IMU samples over 366.3 s, median rate 502 Hz,
zero non-increasing timestamps, zero gaps above 2.5 times median step, longest
step 3.0 ms. Quiet startup estimated 23 s; gravity 1.0005 g. Startup gyro bias
[0.146,-0.184,-0.084] deg/s. No detected false-offset stretches. Maximum gyro
rate 160 deg/s, maximum acceleration 3.19 g. These peaks alone do not establish
a sensor fault. Tool reports zero failures/warnings; no end-rest bias-drift
measurement was reported and none is inferred.

All 60,037,017 LAZ points decoded; zero nonfinite coordinate/time values and zero
decreasing point timestamps. Point span 0–366.272415 s. Packet diagnostics:
762,965 rows; zero backward/duplicate packet stamps, zero nonzero reserved-header
rows. Confidence policy low; 217,226 reported confidence rejections.

Recorder timing selects sensor timestamps: zero lost samples, stamp jumps,
nudges or duplicate/non-increasing drops. Reported minimum IMU-minus-LiDAR
transport latency -0.136910 ms at start, -0.141025 ms at end; this is not a
measurement of internal sensor synchronization. Layout includes additional
LAZ dimensions; no application change or recorder-code audit was performed.

Conclusion: quick raw timing/finite-value checks are clear. They cannot establish
absence of an estimated trajectory flyaway; no trajectory exists from this
recording yet. The rejected residual floor must not be restored for this scan.

Evidence: out/livox4-imu-health.log, out/livox4-recording-check.json (including
input SHA256 identities), out/livox4-recording-check.log and adapted existing
out/check_livox4_recording.py. Original recording files were read only.
