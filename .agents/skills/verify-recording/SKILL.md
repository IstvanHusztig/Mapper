---
name: verify-recording
description: Check whether a LiDAR + IMU recording folder is usable before dMapper processes it or any conclusion is drawn from it - sample rate and gaps, false gyro offsets, gyro bias at rest, noise, rest at the start, recorder timing. Use this whenever a new recording arrives, whenever a run on a recording looks broken and the sensor could be the cause, and before a recording is used for tuning or as a baseline, even if the user only says "new scan is in folder X" or "is the IMU ok?".
---

# Verify a recording

## Agreed evaluation rules (2026-11-12)

Read `openspec/changes/sensor-neutral-yardstick/criteria-proposal.md` before any
verdict. The user agreed this contract; it supersedes legacy tool defaults.
Report missing evidence as NOT MEASURED. Diagnostic triggers are not map-accuracy
verdicts. Overlapping empirical envelopes are inconclusive, not proof of equality
or non-regression. Preserve every perturbation member and check odometry and
final results separately. The user's review of readings and clouds remains required.


A bad recording wastes every run made on it, and a sensor fault looks exactly like an odometry bug in the map (the
Unitree L2's gyro held false rates of 11-32 deg/s; the maps "broke" the same way a mis-weighted odometry breaks them).
So the recording is checked from its raw files, without dMapper, before anything else.

## Steps

1. Run the IMU check on the recording folder (the one with `imu*.csv` and `lidar*.laz`):

   ```
   python tools/imu_health.py "<recording folder>"
   ```

   It prints one line per check with `ok`, `WARN` or `FAIL`, and exits 1 on a failure.

2. Read the recorder's timing file, if the folder has one, for the IMU-to-point latency and the time source:

   ```
   grep -h -o '"decision": "[^"]*"' "<recording folder>"/timing*.json
   ```

3. Ask the user how the recording was made, if they have not said: how the sensor was held or mounted, whether it
   stood still at the start and for how long, and whether the walk returns to its start. The files cannot tell a
   sensor resting on a table from one held still in a hand, and the answer decides which later checks apply
   (closure needs a walk that returns).

## What must hold

| Check | Needed | Why |
|---|---|---|
| Non-increasing timestamps | 0 | dMapper requires strictly increasing IMU timestamps |
| False gyro offsets | none | the gyro reports a turn that does not happen; no setting repairs that |
| Lost samples (timing file) | 0 | gaps drop accelerometer terms and points |
| IMU-to-point transport latency | record value and clock decision | not a calibration of internal synchronization |
| Rest at the start | at least 2 s | the start-up gyro bias and the first map are taken there |
| Gravity at rest | 0.98-1.02 g | otherwise the accelerometer's unit or scale is off |
| Gyro bias, and its change from first to last rest | below about 1 deg/s, change below 0.5 | larger values mean warm-up or a fault |

A quiet-block warning alone does not establish missing startup rest or a sensor fault.
Check acquisition notes and actual startup statistics/gates. Historical44 inputs
retain their regression role with documented defects, not a calibration role.

A `WARN` on the noise lines usually means the rest was handheld: the noise values are then upper bounds, not
measurements. Noise for the settings file comes only from a recording made on a table for that purpose.

## Report

Show the tool's table as it is, then one line each for the latency and for what the user said about the recording.
End with one of: usable, usable with the caveats listed, not usable (and which check says so). Do not call a recording
usable before step 3 is answered. Recordings under `unsynced_archive` are never processed.
