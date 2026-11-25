# Design

## Context

See proposal.md (Why). `Consistency` (`src/application/consistency.cpp`) solves one 6-DoF Tait-Bryan pose per trajectory
node (IMU rate, ~1.1 ms) over the whole run, 10 passes: NDT point terms per chunk of 1e6 points (each chunk with its own
map, C35), a second-difference smoothness over three nodes (weights 1e6/1e8/1e8 for translation, (0.1°)⁻² for
rotation), node 0 held. The normal matrix is banded (`BandedNormalMatrix`, bandwidth 17). The windows it gets carry
`imu_angles` (IMU orientation per node, rad), `gyro_bias.mean` (rad/s) and `timestamps`; the odometry snapshot stores
all of them, so the pass can be rerun from a snapshot in about a minute.

The odometry's rotation prior (`scan_to_map.cpp:514-540`) weights each consecutive-node edge with
`1/σ²`, `σ = max(k·ARW·√dt, floor)` (`MotionModelRotationOptions`, `get_motion_model_rotation_options`), and the
relative-pose observation equations `relative_pose_obs_eq_tait_bryan_wc_case1_*` with zero translation weights.

The scratch experiment (`openspec/changes/map-precision-leaks/research_experiments.diff`) added exactly this term to
`Consistency`: measurement = relative rotation of `imu_angles[k] → imu_angles[k+1]` times `exp(−bias·dt)`, sigma
`s·max(k·ARW·√dt, floor)`, rotation weights only; on 44_short (half density, hold-out) s = 2–5 gave −0.73 to −0.87 mm
on never-fitted points against odometry only, s = 1 0.00, s = 0.1 much worse.

## Goals / Non-Goals

**Goals:** consistency follows the gyro between nodes and stops overfitting single nodes; an honest, ground-truth-free
judge (hold-out) usable on caves too; the evaluation tools measure what they claim.

**Non-Goals:** cross-chunk registration in consistency (C35), the
velocity anchor (C34), the gyro fault itself (C38) beyond gating it, loop-closure changes.

## Decisions

1. **Measurement from `imu_angles` minus the window's bias**, as in the scratch experiment, not from
   `motion_model_poses`. It is the tested form, it doesn't depend on what loop closure did to the guesses, and the
   snapshot holds it. The bias is the window's final `gyro_bias.mean` (the odometry's best estimate for that window).
2. **Only edges inside a window.** Nodes of different windows are joined by the hand-off, not by a measured IMU
   increment; the boundary keeps the smoothness term only. (Alternative: bridge with the IMU between windows — needs the
   raw samples, which the pass doesn't have.)
3. **Sigma = s × the odometry's rule, s = 3 by default** (flat optimum 2–5 in the experiment; 3 keeps the odometry the
   stronger gyro user, the pass only corrects). k, ARW and floor are the existing `[motion_model_uncertainty]` settings,
   so the two stay consistent when they are retuned. Rotation weights only; the translation keeps the smoothness term.
4. **Per-window gate.** The IMU rotation over a whole window (first → last node, bias removed) is compared with the
   odometry result over the same window; above `consistency_gyro_gate_deg` (3°) the window gets no gyro edges and is
   counted in the log. Reason: recording 44's gyro is wrong at 253–259.7 s by 6–10° per window (C38) while the
   odometry followed the LiDAR; without a gate the pass would drag that stretch towards the gyro, its nodes being
   nearly free. The threshold is checked on 44 (task 3): the fault windows must be gated, normal turns not.
5. **The term goes into the existing band** (it couples k and k+1 only, inside bandwidth 17) and is assembled in node
   order like the smoothness terms, so results stay deterministic and independent of thread timing.
6. **Hold-out by time blocks**, `holdout_block_s`, odd blocks left out: blocks of whole nodes, so a held-out point's
   node is fitted by its neighbours' points (the experiment's `DM_ODO_HOLDOUT_BLOCK_S` / `DM_CONS_HOLDOUT=2`). One
   predicate `is_held_out(t, block_s)` in the domain, applied where points enter a map or a point term: the odometry's
   initial map, its window map updates and rebuild, `optimize_lidar_odometry`'s point loop, loop closure's segment
   registration, and the consistency chunks (map and point loop). Saving is untouched, so all points are written.
7. **The hold-out setting lives in a new `[evaluation]` section**, not `[lidar_odometry_debug_info]`: the snapshot
   check ignores the debug section, but the hold-out changes the odometry, so a snapshot made with another hold-out
   must be refused.
8. **`tools/holdout_sharpness.py`** (from `research2/slice_planarity.py`): planar spots (PCA of neighbours within
   0.12 m) of the held-out points only, the spot set fixed on the first result, thickness paired between results;
   the interval combines the spot bootstrap with the s0/s1/s2 spread instead of the bootstrap alone (review of F2).

## Risks / Trade-offs

- [The gyro term trades map fit for smoothness and could worsen global geometry or revisits] → judged with
  `map_quality.py` on s0/s1/s2 (geometry, consistency, sharpness of equal weight, beyond the noise spread), plus the
  hold-out and `wall_precision.py`; it ships only if nothing gets worse beyond the spread.
- [Half the points in hold-out mode make the odometry weaker and may exaggerate overfitting] → hold-out results are
  compared only with hold-out results; the shipped decision also uses the full-density `map_quality.py` runs.
- [The gate threshold is recording-specific] → it is a setting; the log count shows how many windows it removes; for
  caves, the same logic (gyro vs LiDAR per window) applies without building constants.
- [About +10 % consistency time] → accepted (precision before speed); reported.
- [Hold-out code touches the odometry's hot point loop] → one branch on a setting that is 0 by default; bit-identity
  with the setting off is checked on 44_short and 44.
