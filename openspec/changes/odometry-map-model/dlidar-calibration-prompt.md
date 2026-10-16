# Task for the dLidar agent: how are the L2's point coordinates computed, and is the unit's calibration applied?

You maintain dLidar, the app that records a Unitree L2 (LiDAR with a built-in IMU) into recording folders (`lidar000N.laz` + `imu000N.csv`). A separate program, dMapper (LiDAR-inertial odometry and mapping, another repository; don't change it), builds 3D maps from those folders. **Report first; don't change the recording format or the conversion until the user decides.**

## Why we ask

dMapper's maps of the user's recording 44 have a heading error that no odometry setting explains. On a straight building façade:
- the map turns by about 3–4° over a 3 m wide gallery walked out and back along the same wall, and turns back when the path returns to known ground;
- inside single scans, the far parts of the straight façade (6–15 m ahead or behind the sensor, grazing incidence) bend about 0.5–0.7° away from the sensor, in both walking directions.

One hypothesis is an error in how each point's direction is computed: a beam-angle calibration error bends straight walls, more at long range, like lens distortion in a camera. It is not proven. The data show only the bend, and dMapper's other checks are inconclusive. Before the user records a dedicated calibration scan, we need to know what dLidar does.

As far as we know, Unitree's `unilidar_sdk2` computes each point from the measured range and the mirror and motor angles with per-unit calibration parameters (in `unitree_lidar_utilities.h`: `a_axis_dist`, `b_axis_dist`, `theta_angle_bias`, `alpha_angle_bias`, `beta_angle`, `xi_angle`, `range_bias`, `range_scale`). Check this against the SDK version you use.

## 1. Verify and document (with file and line references)

1. **The conversion path:** how a packet from the L2 becomes the `x y z` written to the LAZ.
   - Does dLidar call the SDK's own parsing or point-cloud function, or does it convert raw packets itself? Which SDK version or commit?
   - Is any further transform applied: axis flips, a rotation, the IMU-to-LiDAR transform, a lever arm, a mounting correction?
2. **The calibration parameters:**
   - Does the unit send them (in the packet header or a separate message)?
   - Does dLidar read them from the device, or use defaults or constants?
   - Are they applied to every point? If they are cached, when?
3. **The values of this unit:** print the calibration parameters the user's L2 reports, next to the SDK's defaults, if any. Also say where in a recording they could be found later (they probably aren't stored now).
4. **Precision of the written points:**
   - Is there float32 rounding before the LAZ is written?
   - What are the LAZ scale and offset (e.g. 0.001 m)?
   - Is there any filtering: range or intensity cuts, dropped returns, a minimum distance, downsampling?
5. **Per-point direction against time:** is each point's direction computed from its own angle sample, or from a packet-level angle plus an index? This is especially relevant for the non-repetitive scan pattern.

## 2. Prove it with a small test

Take a few raw packets, from a saved dump or live:
- compute the points once with the SDK's reference conversion and the unit's calibration parameters;
- compute them once through dLidar's actual path;
- compare `x y z` point by point (max and RMS difference, and the angle difference in degrees as a function of the beam direction).

They should agree to well below 1 mm. If they don't, show where the difference comes from.

## 3. Answer these questions for the user

- Are the unit's factory calibration parameters applied to every recorded point: yes, no, or partly?
- If not, what would change if they were applied? Estimate the size of the angle error as a function of beam direction, if the defaults differ from the unit's values.
- Can dLidar store the unit's calibration parameters in each recording folder (e.g. a small `lidar_calibration.json`), so that later processing can check or correct them? Don't implement it yet; say how.
- Can dLidar record a static calibration scan as it is? Planned setup:
  - sensor at rest, 1.5–3 m from a long flat wall and pointing along it (10–25 m);
  - about 30 s per orientation, several turns about the sensor's own axis.

  Should anything be set differently for it, for example a raw-packet dump alongside the LAZ?

## Deliverable

A short report with the answers above, the file and line references, the test's numbers, and the unit's calibration values. Make no format changes unless the user asks.
