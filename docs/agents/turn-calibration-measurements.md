# Turn-recording measurements, 2026-11-12

Collection only: no new sensor calibration values or default settings accepted.
Input: `C:/Users/User/Downloads/dLidar/raw data/livox/turn`, 71.8 seconds,
hand-turned left and right with rest at both endpoints (setup recovered from
Claude's final session). Static-table analysis already exists separately.

## Recording check

imu_health.py: PASS, zero failures/warnings. 35882 samples, median rate 499 Hz,
no non-increasing timestamps, gaps or false-offset stretches. Start rest 14.5 s;
gravity 0.9993 g. Raw gyro bias first rest [0.762, -0.743, -0.063] deg/s, last
rest [0.793, -0.743, -0.060], reported change 0.03 deg/s.

Recorder metadata reports zero loss/jumps and approximately -0.12 ms minimum
IMU-to-point latency. This supports timestamp transport health, not a calibrated
internal sensor time offset. Its method description still mentions L2 although
sensor_model identifies Mid-360S.

## Baseline run

Used Release executable SHA256
`033EA75ACD39C14752E734E0AC473252034F97B2E2BA2A57A018C8CDA09CBCB0`,
the build validated for removal commit e196692. Shipped livox-vertical settings,
with only log_window_diagnostics enabled in an output-local TOML. Lever arm stays
zero. No source/default-settings edits. Processing exit 0, 170.24 seconds.

Artifacts: `out/turn-calibration-2026-11-12/` contains baseline.toml, baseline.log,
baseline/ result files, run-health.txt, raw-rest-registration.txt and
trajectory-summary.json. Input cache/preview were written by the normal pipeline.

| Health reading | Value | Current draft limit |
|---|---|---|
| Windows | 197 | — |
| Bends >3 deg | 1 (0.5%) | <=1% |
| Bends >10 deg | 0 | 0 |
| Largest bend | -7.2 deg at 27.7 s, window 78 | — |
| Speed >2.5 m/s | 0; max 0.42 m/s | 0 |
| Largest estimated gyro-bias components | 0.12 / 0.14 / 0.38 deg/s | <=0.5 |
| Warning lines / dropped slices / iteration time caps | 0 / 0 / 0 | — |

run_health.py reports PASS under its draft thresholds. Bias estimates here are
optimizer residual-bias states, not the raw stationary startup bias above. The
tool did not report velocity-jump chi-squared statistics for this run; absence
is not a passing uncertainty calibration measurement. No walking-loop closure
criterion was applied to this turn-on-the-spot recording.

Loop closure accepted no loops; consistency refinement ran. Final world poses
were reconstructed from chunk-relative trajectories using session_poses.mrp.
Comparing mean stationary intervals 5-10 s and 67-70 s gives displacement
0.01637 m, relative attitude angle 3.393 deg and world-yaw difference -3.273 deg.
These are behavior of a gyro/accelerometer-constrained, refined trajectory, not
independent calibration truth or measured placement error.

## Independent coarse LiDAR check

Existing chain_yaw2.py registered raw, accelerometer-levelled snapshots centered
at 5 and 68 s, 0.5 s wide. It selected yaw without gyro guidance, then compared
gyro integration: LiDAR -4 deg, gyro -2.8 deg, difference -1.2 deg.
Confidence 1.21 is below its 1.25 threshold: **ambiguous, not an accepted match**.
The 1-degree yaw grid and coarse raster cannot support precise gyro scale or
millisecond timing calibration. In-motion snapshots also need attention to
distortion and acceleration-dependent leveling.

turn_check.py separately gives gyro net 3D rotation 2.67 deg, integrated rotation
about gravity -3.22 deg, and gravity agreement 0.75 deg. These are different
rotation summaries; they should not be interchanged or treated as a scale fit.

## Unresolved calibration

- Scale: left/right cancellation cannot identify it from net angle alone;
  optimized dMapper poses are not an independent gyro reference.
- Alignment: stationary gravity constrains tilt checks, not a full 3D extrinsic
  rotation from mostly yaw motion.
- Lever arm: metadata candidate [-0.04412, 0.011, 0.02329] m has unresolved sign
  semantics versus the loader's point-plus-configured-offset operation. Handheld
  translation confounds lever-arm fitting. Do not copy the candidate into defaults.
- Timing: timestamp-health metadata and optimizer-coupled trajectory comparisons
  are insufficient for an independently calibrated time offset.

A Codex research agent reviewed local tools, frame semantics and observability
limits (not Claude Sonnet). Review these measurements with the user before a
calibration verdict or additional calibration method. The next roadmap work
remains completed-batch readings and phase 0a; phase 1 is still partial.
