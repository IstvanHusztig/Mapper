# Research: code errors or limits of the method? (2026-11-20)

The evidence behind this change. Four read-only agents: a literature search and three derivation audits, one per data
path (accelerometer and hand-off, gyro and rotation, LiDAR and solver), on branch `honest-inertial-carry` at ba0ec25.
Their full reports and scripts are outside the repository, in
`C:/Users/User/Downloads/dLidar/experiments/consistency/` (`literature.md`, `audit_accelerometer_handoff.md`,
`audit_gyro_rotation.md`, `audit_lidar_solver.md`, `scripts/`). This file is the merged summary after the conclusion
reviewer's corrections. Sources in brackets: [acc], [gyro], [lidar], [lit]. "Checked" = the orchestrator read the code
lines named.

## Limits of the evidence

- All run numbers are from Livox 3 full runs; [gyro] also used one older Livox 4 run. No L2 run was analysed.
- The audited settings are the stage 8 candidate set, not the shipped Livox file: exact cell statistics, projection
  0.02 / 0.04, offset random walk 0.01, `ahrs_gain = 0`. Thresholds and counts do not transfer to the shipped file.
- The blind-exit evidence is one stretch (Livox 3, windows 595-604), seen in three runs.
- Nothing was built or run. Every "expected today" of a proposed test is a prediction.
- Not re-derived: the generated relative-pose Jacobians and edge blocks (covered by the gradient unit tests only),
  loop closure and consistency, the Livox IMU's timestamp semantics.
- The shares below are regression R^2 values of one run, not causal parts; the regressors overlap.

## The three symptoms

| # | Symptom | Explained by | How far |
|---|---|---|---|
| 1 | In windows the LiDAR sees, the start velocity moves from its prior by 23-24 mm/s (3-axis norm; about 10 mm/s median and 15 mm/s rms per axis) where the prior's sigma is 3.8-4.7 mm/s per axis: a factor 3.5-5 in sigma | F2 (hand-off) 10 % [acc] / 16 % [lidar]; lever arm 10-12 %; LiDAR-IMU time offset about 7 % | 17-21 % [acc], 25 % [lidar]; **75-80 % unexplained**; it is not known whether the LiDAR's velocity or the carried one is in error |
| 2 | After a stretch without LiDAR information along travel, the first correction of the velocity is 0.6-1.2 m/s against a believed 10-90 mm/s | F1: a position correction of 0.10-0.24 m (-98 / -179 / -238 mm in the three runs) taken as velocity in one window | the one traced event; F5 and F7 reach the same size and are untested |
| 3 | One seen window later the velocity sigma is back at about 5 mm/s | F1 (the covariance handed on is conditional on an exact first pose) and F4 | same event |

What the runs say about the unexplained part of symptom 1 [acc] [lidar]:
- |jump| median is 23.3-24.1 mm/s in four runs. Three of them differ only in the two layer switches of
  `honest-inertial-carry`; the fourth also has AHRS gain 0.05, offset random walk 0.0001 and legacy cell statistics,
  all at once. So the switches do not set it, and the other three settings do not set it jointly.
- It grows with rotation rate (16.8 -> 28.6 mm/s by quarter) and with the acceleration at the window boundary (19.8 ->
  29.0), not with speed or the information ratio. It is alike along travel, sideways and vertical. At rest: 3.6 mm/s.
- Lag-1 autocorrelation -0.12 to -0.35. An error of each window's own displacement gives -0.5; an acceleration error
  common to a window gives +0.5. As a displacement it is about 5 mm per window.
- The LiDAR term claims 0.26 mm for a window's end-to-end displacement; the solved windows end 4.2-5.3 mm per axis
  from where the carried velocity put them.
- Untested candidates, all scaling with rotation rate: the first node's fixed attitude, the LiDAR's own displacement
  error per window (points placed with the node before them, cell effects), a time offset, a rotation between the
  IMU's and the LiDAR's axes.

## Findings

### Read and found correct
In the terms that were read: the accelerometer terms (residuals, Jacobians, weights; each sample interval used once);
layer 1 (`imu_edges_follow_state`) and layer 2 (`imu_carry_joint_covariance`) as designed, each growth applied once;
the rotation path's units, bias sign and single removal; no Tait-Bryan singularity (fi within +-16 deg on Livox 3,
-26..14 on Livox 4); the NDT Jacobian, Huber and scale; the projection for s = 0 and s = 1; `take_covariance` (the
undamped matrix of the linearization that made the step, true marginals, restored on a rejected step); the band of
the normal matrix against every writer; no window matched against its own points; gravity constant 9.80665.

### Code contradicts its own stated model

| # | What | Where | Effect | Status |
|---|---|---|---|---|
| F2a | Over the gap between windows the velocity prior's mean is not moved by the measured acceleration, and the last node's accelerometer sample is used by no term | `run_odometry.cpp:148`, `accelerometer_term.cpp:88`; stated model `window_noise_model.h:149-151` | a x 2 ms: 1.1-1.5 mm/s rms per axis | checked in code |
| F3 | With `ahrs_gain = 0` the rest test removes gravity with an open-loop filter tilt (1.6-1.8 deg off at the end of Livox 3). A setting the test was not written for | `imu_kinematics.cpp:59-65`, `window_steps.cpp:57-64` | 21 rest windows at 245-252 s of Livox 3 missed (51 found, 72 with gain 0.05); Livox 4 finds 98 of 100. Not symptom 1 or 2 | checked in code; count reproduced by [gyro] |
| F6 | The velocity chi2 divides the posterior-minus-prior move by P_prior; under a correct linear-Gaussian model the move's covariance is P_prior - P_post, so the expected value is 3 - tr(P_prior^-1 P_post), between 0 and 3 (rest windows log 1.0) | `run_odometry.cpp:349-352`, `:1112-1114` | diagnostic only; the excess is 3.5-5.5x in sigma. The verdict's limits (5 / 20) rest on this statistic. P_post is conditional on the exact first node | checked in code and by derivation |
| C16 | Fractional projection: curvature s^2, gradient s (known) | `ndt_term.cpp:310-333` | active in the blind-exit window 602 (s = 0.8) | known, present |
| - | The +-1e5 test on a cell's inverse is not rotation-invariant (inherited) | `ndt_map.h:20-24` | drops cells thinner than 3.2 mm axis-aligned, 2.2 mm at 45 deg; effect not measured | new |
| C37 | Degree sigma on radian rows of the origin fix (HDMapping's "Fix deg to rad") | `window_noise_model.cpp:228-230` | harmless here (1e12 against curvatures 30-300x smaller) | known, present |

### The scheme drops or double counts something (design)

| # | What | Evidence | Status |
|---|---|---|---|
| F1 | **Every window's first pose is held exact (1 um, 1 urad) and nothing of its uncertainty is carried.** Published methods carry the boundary pose jointly with velocity and biases [lit]; fixing it is an approximation that holds while the pose error is small. A window cannot shift, so a position disagreement c with the map becomes a velocity change ([acc]: 1.5 c / T with the offset held to 4 c / T with it free; [lidar]: c / T to 1.5 c / T) and, with layer 2, an offset change folded into gravity | Window 602 of Livox 3: d T - e T^2 / 2 = -0.2380 m against the logged -0.2377 m; the window moves 7 mm where its guess moves 245 mm; velocity 0.76 -> -0.01 m/s handed on with sigma 4-6 mm/s; speed after it 0.8-0.9 m/s; the next window takes +500 mm/s back | checked in code (`window_solver.cpp:7-12`, `window_problem.cpp:77-88`); one event; whether the correction itself was right is unknown. It is D6 of `honest-inertial-carry`, now with the accounting |
| F2b | The last 15 nodes of a window are replaced by a constant-velocity line from a 20 ms finite difference; the next window's first position comes from that line, the carried velocity from the solve (C32 covered the rotation half) | first node off by 8.3e-4 s^2 x acceleration (0.4-0.7 mm); with F2a the jump follows 8.5-10.4 ms x boundary acceleration; the tail's points (8 %) enter the map at the extrapolated poses | checked in code (`window_steps.cpp:201-225`, `run_odometry.cpp:1202-1208`, `:1232`); a design approximation (`handoff_tail_nodes`) |
| F4 | **The LiDAR term claims more than it knows:** each point an independent measurement against an exact map, in two grids | 99.8 % of the points used in both grids: +57 to +80 % information (C18); map as exact: at least x1.4; claimed hold of a rigid shift 0.15 mm; the in-plane part sees 0.41 of an offset and pulls moving windows backward 0.16-0.35 mm per window; fine and coarse grid disagree by 0.9 mm when moving. Cost per residual is 1.1 before the 0.1 scale: the per-point weights are right, the independence is not. The literature reports the same for ICP by orders of magnitude; practical systems hand-tune a fixed weight [lit] | measured on one run; how much to reduce is not derived |
| F5 | **The gyro-bias state is the attitude's only integrator and is far too confident** (believed 0.01 deg/s, moving +-0.3-0.5 deg/s); no attitude covariance is carried; gravity evidence goes to the offset, never to the attitude | a rotation statistic from the existing tables: median 44 (Livox 3) and 39 (Livox 4) against 2.4; it is built like F6's statistic, so its true expected value is lower and the excess larger. Livox 4's lower floor is tilted 1.6-2.1 deg world-fixed for 110 s. A bias error of 0.3 deg/s gives 0.41 / 0.92 m/s after 4 / 6 s if the LiDAR does not hold the tilt | over-confidence on two runs; the link to symptom 2 is untested |
| F7 | World-frame offset against a body-fixed misfit (C17) | [gyro]: 0.05 m/s^2 body-fixed on both Livox recordings, 0.07-0.10 m/s^2 wrong after a 90-180 deg turn. [acc]: 2 % of it in the innovations. The two sit uneasily together; neither shows that the innovation should carry it | known flaw, size disputed |
| F8 | Accelerometer noise white only; the offset (random walk 0.01) is a fast state | the handed-on 4.5 mm/s is set by the 0.1 floor; with layer 2, 78 % of the offset estimate's variance mirrors the velocity move and jitters gravity by 0.024 m/s^2 per window | model limit |
| C34 | Edge position rows restate the guess (known; removed for velocity and offset by layer 1) | not what sets symptom 1 on the Livox | known |

### Calibration (plausible, not confirmed)
- **Lever arm** (0, 0, 0) in the Livox settings; the accelerometer term has none of its own. Two regressions on the
  same run give (-30, 0, -1) mm [acc] and (-24, 1, 9) mm [lidar]; the fit is attenuated, and a lever arm is not the
  only thing that scales with rotation rate. Data sheet: (0.011, 0.023, -0.044) m in native axes; the mount's axis
  map is not set.
- **LiDAR-IMU time offset** 5-7 ms by regression (1 ms of it the zero-order hold of the points) [lidar].
- **Rotation between the IMU's and the LiDAR's axes** 0.1-0.25 deg: the x / y bias estimates follow the yaw rate
  (-0.18 %, +0.11 %, the same on both recordings) [gyro].
- Start-up gyro bias: a median of samples quantised at 0.061 deg/s, 0.054 deg/s off on Livox 3 y [gyro].

### The four upstream (HDMapping) fixes
deg -> rad: present as C37, harmless. Cell struct initialisation: no uninitialised read today, the struct still lacks
initialisers (`ndt_types.h:31-36`). Gravity constant: fine. Indoor / outdoor table bug: that mode does not exist here.

## Lineage: what reaches the solve more than once

| Source | Reaches | Twice? |
|---|---|---|
| Accelerometer sample of a window | one accelerometer term; the guess -> the edges' position rows | yes: C34 by default; with layer 1 only the shape (+9 %) |
| Accelerometer sample of the last node | nothing | dropped (F2a) |
| Accelerometer -> AHRS orientation | guess rotations -> rotation edges -> attitudes in the accelerometer terms | yes at gain > 0 (shipped L2 file); at gain 0 only in windows 1-8 |
| Gyro increments of the last 15 edges | weighted edges of window k, then the exact first pose of window k+1 | exact the second time; the LiDAR's evidence on the last 30 ms is dropped |
| Carried end velocity | start-velocity prior; the guess -> edges | yes by default (C34); removed by layer 1 |
| Finite-difference tail velocity | the next window's fixed first position | a second, 40 ms older velocity next to the prior (F2b) |
| Solved last pose (position, attitude) | the next window's first node, fixed | conditioned on, not marginalised: covariance and correlations dropped (F1) |
| LiDAR point | fine-grid term; coarse-grid term; the projection's gate | yes: C18 |
| LiDAR points of window k | the solve of k -> priors of k+1; the map -> the NDT term of k+1 (87-94 % of its points fall in cells k filled) | yes: map and priors share errors |
| Carried gyro bias, offset, velocity | their priors | correlations with the pose and with each other (except velocity-offset with layer 2) dropped |

## Not to conclude
- That the term maths is free of bugs (see "not re-derived").
- That fixing F1 fixes the Livox 4 corridors: one event on Livox 3; F5 and F7 reach the same size.
- That the 0.24 m correction at the blind exit was wrong: it may have been right.
- That F2, F3 or F6 matter for the map: none was tested by a run.
- Anything about the L2 or the shipped thresholds.
- The orchestrator's inference, not in the sources: that a joint carry of the whole boundary state (pose, velocity,
  offset, bias) would take in layer 2 and D6, and that F1 and F4 belong together (a fixed-sigma rotation prior on node
  0 ran away on 2026-11-18, because an honest prior loses against the LiDAR's claimed stiffness).
