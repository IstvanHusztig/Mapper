# Sensor-neutral retune: findings, direction and plan (draft for discussion)

<!-- docs-nav -->
> [dMapper docs](README.md) › Plans and research · [All documents](README.md) · [User guide](index.html)

Current evaluation rules: the user agreed on 2026-11-12 to
[the OpenSpec evaluation contract](../openspec/changes/sensor-neutral-yardstick/criteria-proposal.md).
It supersedes the draft numerical limits and verdict rules below. Preserve the
older readings as historical evidence, not current acceptance decisions. Current
phase0a inputs are Livox/2 0–60 s, whole Livox/1 and 44_short; the mid-walk cut
below is superseded. Direction A and obsolete-mode removal were already agreed.

Status: **draft, 2026-11-10. Nothing here is decided.** It collects what was found on the first two Livox Mid-360S
recordings, lists what in dMapper was tuned or calculated for the Unitree L2, and proposes a process and an order of
work. Goal: a clean, precise map from any supported sensor, with no setting that only works because another one is wrong.

## 1. What we know

| # | Finding | Evidence |
|---|---|---|
| F1 | The L2 unit's gyro is faulty: it holds a false rate of 11–32 °/s for 2.5–4 s while gyro and accelerometer noise are at rest level. | `imu_health` check on 46, before, before/take2; also 44 at 252–261 s (22–31 °/s). |
| F2 | The Livox gyro shows no such fault. Bias at rest 0.1–0.35 °/s, scale within 1 % on a 223° turn, peak rate 116 °/s. | livox/1 (0 of 12 quiet stretches), livox/2 (0 of 71). A fault that appears only in motion is not excluded. |
| F3 | With the shipped weights the scan matching turns the heading against a correct gyro. livox/2 breaks at 307 s. | livox/2 base: 168 windows bent > 3°, 46 > 10°, max 43.6°; z falls to −84.8 m. |
| F4 | Trusting the gyro 7× more (k = 1) removes the break but not the bad map. | livox/2 k = 1: 24 windows > 3°, max 17.1°; the path still ends 14 m from its start after 310 m (4.5 %); height wanders ±1.4 m on one floor; loop closure corrects up to 11.17 m. |
| F5 | What is left at k = 1 are position slides: 3–4.5 m/s for about 2 s while the accelerometer shows < 0.5 m/s². | livox/2 k = 1 at 20–22 s (stairs) and 320–323 s; NDT cost has deeper minima 1.2–2 m along the weak axis. |
| F6 | The gyro bias estimate is corrupted at those moments: 0.08 → −1.6 °/s with a claimed sigma of 0.015 °/s, decaying over about 25 s. | livox/2 k = 1, windows 57–130. True bias ≤ 0.35 °/s. |
| F7 | **The solve is overconfident on both sensors.** The next window breaks the velocity carried from the previous one in 94–99 % of windows. | Median chi² of the start-velocity jump: 857 (livox/2 k = 1), 1375 (livox/2 base), 183 (livox/1), 217 (L2 44), 116 (L2 46). About 3 would be consistent. Carried sigma ≈ 3.7 mm/s; real jumps ≈ 0.1 m/s. |
| F8 | Gyro weighted by a rough estimate of its own noise (k = 1, ARW 0.024) gives no bends on livox/1 but thicker walls. | livox/1: wall thickness p50 38 mm (k = 1 with L2 ARW: 19 mm; shipped: 23 mm), loop-closure correction 1.32 m. One run. |
| F9 | On livox/2 the same weighting passes the bend check and fails everything else, worse than k = 1: the error moves from the heading into position slides and the bias. | livox/2 own noise: 6 windows bent > 3° (0.4 %), none > 10°; 22 windows above 2.5 m/s (max 5.7); bias up to 3.9 °/s; chi² median 2386; the path ends 75.7 m from its start (22 m per 100 m), 20.9 m too low. One run. |

F7 is the central one. The fact is that the solve claims a velocity good to a few mm/s and is then wrong by about
0.1 m/s, 2 ms later. The **likely cause, not yet proven**: the scan-matching term adds every point with its cell's
inverse covariance and nothing else (`scan_to_map.cpp:208-295`): no measurement-noise model, no normalisation, no
robust kernel. Thousands of points that share the same map errors count as independent, so the information grows with
the point count. The Livox delivers about 3× the L2's points, which would make it worse. Every IMU-side weight was
then tuned by hand against that term (translation edges of 0.1 mm per sample, gyro sigma inflated 7×, a bias allowed
to move 20 °/s), partly while the L2's IMU lagged by 50–80 ms and its gyro was failing.

Other explanations the reviewer raised, to be separated by the first experiment (phase 0b): the velocity at a window's
end is its least observed state; the carried covariance grows by almost nothing between adjacent windows, so it has no
process noise; the tight translation edges, not the scan matching, may be what makes the covariance small.

What the existing logs already say about them (2026-11-10, livox/2 at k = 1 and L2 44):
- **Real motion is not the cause.** The two velocities are 2 ms apart. The walker's acceleration (median 0.6 m/s²)
  explains about 1 mm/s over that gap; the median jump is 102 mm/s (60 mm/s on 44).
- **The carried sigma is nearly constant:** 3.4–4.0 mm/s over all windows, 4.2 mm/s on 44. A covariance set by the scan
  matching would follow the geometry and the point count, which vary several-fold. So the sigma is more likely set by
  terms with fixed weights: the accelerometer term (0.1 m/s² over 10 ms claims a position second difference of 10 µm)
  and the translation edges.
- **The chi² follows the jump (correlation 0.99), not the sigma,** and the jump correlates only weakly with the scan
  matching's stiffness, its cost, the speed or the acceleration (0.1–0.3).

So the overconfidence is real (jump 14–28× the sigma), but it is open which term is wrong: scan matching that lets
millimetre jitter of the node positions through because it outweighs the smoothing terms, or IMU-side sigmas that claim
more than a handheld sensor delivers. Phase 0b reads the jump and the sigma separately for that reason.

F8 says the other half: a gyro trusted at its noise level needs its bias, scale and alignment to be right, which
nobody has measured.

## 2. Inventory: what was made for the L2

`settings/livox-vertical.toml` is `l2-vertical.toml` except for the range (25 → 40 m), five sample counts and the lever
arm (zero, TODO).

### 2.1 Settings

| Class | Settings | Action |
|---|---|---|
| Measured on the L2 | `gyro_arw_x/y/z` (0.028 / 0.042 / 0.057), `imu_accel_noise_density` (0.0013), `imu_accel_floor_sigma` (0.1), `imu_acceleration_offset_rw` (1e-4, data sheet, unit unclear), `startup_gyro_bias_max_variance` (5.0) | Measure on a static Livox recording (phase 1). Never tune. |
| Tuned on L2 data with a lagging IMU | `rotation_sigma_inflation_k` (7.09), `rotation_sigma_floor_deg`, `ahrs_gain`, `degenerate_projection_ratio` 0.1 / `_full` 0.2, `gyro_bias_rw` (0.005) | Re-derive in phase 3; they compensated for the lag and for F7. |
| Tuned on L2 data | `lidar_odometry_motion_model_x/y/z_1_sigma_m` (1e-4), `use_exact_ndt_bucket_statistics` (off), `imu_degenerate_*`, loop-closure sigmas, gates and fitness limits, consistency on | Phase 3 (odometry), phase 5 (loop closure, consistency). |
| No recorded reason | `gyro_bias_initial_sigma` (10), `gyro_bias_max` (20 °/s), `imu_velocity_initial_sigma` (1.0), `imu_zero_velocity_sigma` (0.01), `lidar_odometry_motion_model_acc_1_sigma` (1.0), `loop_closure_odometry_sigma_*` | Phase 3: set from the sensor or from consistency statistics. |
| Depends on point rate, range, field of view | `decimation` (0.03), cell sizes 0.15 / 0.4 m, `threshold_initial_points` (60 000 = 1 s on the L2, 0.3 s on the Livox), `max_distance_lidar`, `threshold_nr_poses` (window length), `imu_degenerate_information_ratio` | Phase 2 (time-based where it is a time), phase 4 (tuned last). |
| Counted in IMU samples | `threshold_nr_poses`, `handoff_tail_nodes`, the three `*_span_nr_samples`, `startup_gyro_bias_nr_samples` | Phase 2: express as times, so a firmware rate change needs no retune. |
| Dead with `imu_always_on` | `imu_acceleration_only_when_degenerate`, `imu_acceleration_sigma_mps2`, `imu_degenerate_translation_sigma_m`, `imu_degenerate_max_duration_s` | Decide with the user whether the gated mode stays (no dead code rule). |

### 2.2 Code

| Where | What it assumes | Phase |
|---|---|---|
| `scan_to_map.cpp:208-295`, cost `:104-115` | Each point is an independent measurement at the cell's covariance; information grows with the point count | 3 |
| `scan_to_map.cpp:160-162` | Translation motion-model sigma is a constant per IMU sample, so its stiffness changes with the IMU rate | 2 |
| `lidar_odometry.cpp:383-391` | A point takes the pose of the IMU sample before it, with no interpolation: 2 ms at 500 Hz, 0.2° of smear at 100 °/s | 2 |
| `lidar_odometry.cpp:390` | Points more than 10 ms from an IMU sample are dropped | 2 |
| `lidar_odometry.cpp:214`, `:224` | First IMU step 1/200 s; fallback step 2 ms | 2 |
| `lidar_odometry.cpp:203` | Accelerometer correction off outside 0.9–1.1 g (open as D2 in `imu-front-end`) | 2 |
| `lidar_odometry.cpp:400` | A window slice with ≤ 1000 points is dropped (a point-density assumption) | 2 |
| `ndt_map.cpp:126`, `ndt_map.h:30` | A new cell starts at 3 cm sigma; a cell whose inverse has an entry above 1e5 is skipped | 3 |
| `ndt_map.cpp:30-55`, `:60-134` | Decimation keeps the first point per voxel; a cell's mean is its first point (default statistics) | 3 |
| `run_odometry.cpp:245`, `window_steps.cpp:104`, `imu_kinematics.cpp:126` | Rest detection: 25 ms smoothing, 20 ms span, 0.3 m/s² from the L2's noise | 2 |
| `scan_to_map.cpp:1024`, `window_steps.cpp:72` | A step or shift of 1 m is a "big jump" | 3 |
| `scan_to_map.cpp:987-988` | The carried velocity covariance is the solve's marginal, so it inherits F7 | 3 |
| `run_odometry.cpp:492` | An iteration loop cut by wall-clock time (`real_time_threshold_seconds`): results can depend on machine load | 0 (set high for experiments) |
| Optimizer loop | No cost check on a step; a point that leaves the map costs nothing (diagnosed 2026-10-10 on 44's stairwell; branch `candidate-2-lm-step-check`) | 3 |

## 3. Directions

| | Direction | For | Against |
|---|---|---|---|
| A | **Physically grounded weights.** Every IMU sigma comes from measured sensor noise. The scan-matching term gets one calibration (an information scale or a normalisation), chosen so that the consistency statistics come out right. Terms without a physical meaning are re-derived or removed. | Removes the cause (F7) instead of compensating; few free parameters; the same recipe for the next sensor and for caves. | Needs calibration quality the code never needed (bias, scale, alignment: F8). The whole weighting moves at once, so L2 results change too. |
| B | Retune the existing knobs for the Livox. | Fastest first result. | Repeats what happened with the L2: knobs compensate for F7, single runs are within run noise, a new firmware or sensor starts over. F4 and F9 show the gyro knob alone moves the error instead of removing it. |
| C | Replace the front end by a standard tightly coupled LiDAR-inertial formulation (IMU preintegration, per-point noise model). | The well-trodden path. | Largest effort; throws away what works; A moves the same way in steps. |

Recommendation: **A**, built in small steps that each stand on their own, with B's knobs touched only at the end
(phase 4) and C kept as the fallback if phase 3 fails its gate. Two limits, from the review (Sonnet, 2026-11-10):
- The recommendation rests on the reading of F7. Phase 0b tests it in about an hour before anything is built on it.
- A scale on the scan-matching information is necessary at most, not sufficient. It cannot repair a cost with several
  minima (F5), map errors that all points share, or cell means taken from the first point. Those are phase 3d.

## 4. Process

1. **Measured values are not tuned.** Noise, bias, lever arm, time offset and rates come from a recording made for the purpose.
2. **Structure before tuning.** A defect with an obvious correct form is fixed first; nothing is tuned around it.
3. **Consistency statistics find and calibrate; maps and held-out data decide.** One short run gives hundreds of windows. For each term: does the result break it more often than its sigma allows (chi² of the velocity jump, heading bend against the gyro sigma, bias steps against the random walk, accelerometer misfit)? These need no map and do not depend on the building. But they can be gamed: a looser sigma lowers a chi² while the map gets worse. So the median and the 95th percentile are both read, and no setting is accepted on them alone: it must also hold on the map measures and on data the solve did not use (`holdout_block_s`, `tools/holdout_sharpness.py`).
4. **Short pieces for iteration, full recordings at gates.** Pieces of livox/2 cut with `cut_recording` (0–60 s holds the first slide and a bend; 295–335 s holds the old break) run in about 5 minutes instead of 47.
5. **A noise floor for every verdict.** A candidate is run with 3–6 neutral perturbations; an effect counts only beyond that spread, and the spread itself must not grow.
6. **One family at a time, each in its own change,** with the verify skills run before and after.
7. **The L2 stays a guard.** A refactor keeps the 44 baselines bit-identical. A numeric change is judged on 44 s0–s2 with `map_quality.py` and must not make it worse beyond its spread.
8. **The user's look decides a ship.** Numbers pick candidates; before/after clouds go to the user before any push.

## 5. Phases

| Phase | Work | Needs | Cost | Gate |
|---|---|---|---|---|
| 0a. Yardstick | Verify skills (section 6); short pieces of livox/2; noise floor of the shipped and the k = 1 settings; pass criteria agreed | — | 1 session, about 1 h of runs | The checks reproduce F3–F7 on the existing runs |
| 0b. Decisive experiment | A scratch build with one multiplier on the scan-matching information (1, 0.3, 0.1, 0.03, 0.01), on the short pieces and on `44_short`. Read the chi² median and 95th percentile, bends, slides, wall thickness; also whether the chi² follows the point count, the window length or the gap. | 0a | About 1 h of runs, no shipped code | F7's cause is confirmed or replaced; direction A is kept or dropped |
| 1. Measure the sensor | Lever arm and its sign first (it acts in every turn; the existing recordings may be enough once the mount's axes are known). Static recording → gyro ARW and bias stability, accelerometer density and floor, start-up variance gate. Gyro against LiDAR on clean stretches → scale, alignment of IMU and point axes, time offset. | Static recording ≥ 20 min on a table; 1 min turning on the spot | 1 session | Every "measured on the L2" row of 2.1 has a Livox value with its uncertainty |
| 2. Sensor-neutral code | Times instead of sample counts; translation sigma scaled by the time step; points interpolated between IMU samples; the constants of 2.2 marked "2"; the open `imu-front-end` defects D1, D2, D5 | — | 2–3 sessions | New synthetic tests: the same scene at 1000 / 500 / 200 Hz and at 64k / 200k points gives the same trajectory within tolerance. 44 stays bit-identical where the L2's numbers do not change. |
| 3. Weighting | a) the scan-matching information: scale or normalisation calibrated by the consistency statistics; b) gyro sigma = noise + bias + scale/alignment terms, bias model for a healthy gyro; c) carried velocity covariance; d) step acceptance and unmatched-point cost (branch `candidate-2-lm-step-check`), cell statistics re-tested | Phases 1 and 2 | 3–5 sessions, design at xhigh | Chi² medians 1–5 on both Livox recordings and on 44; criteria of the table below on the short pieces, then on the full recordings |
| 4. Geometry knobs | Cell sizes, `decimation`, window length, range, initial map; one at a time. Two additions (user, 2026-11-10): **stricter point rejection**, since the Livox gives about 3× the L2's points (drop uncertain points: grazing incidence, mixed edge returns, low reflectivity, near range, isolated points), and **reflectivity normalisation** of the points' intensity (by range and incidence angle, so a wall reads the same from near and far) | Phase 3 | 1–2 sessions | Better than the phase 3 result beyond the noise floor, or left alone |
| 5. Loop closure and consistency | Their sigmas and gates were set for the old odometry; re-check with the new one | Phase 4 | 1–2 sessions | Loop closure's largest correction < 1 m; consistency does not thicken walls |
| 6. Ship | New `livox-vertical.toml`, Livox baselines next to the 44 ones, docs, the user's visual check | Phase 5 | 1 session | The user's go |

Draft pass criteria (to be agreed in phase 0; today's values in brackets):

| Check | Pass | livox/1 shipped | livox/2 shipped | livox/2 k = 1 |
|---|---|---|---|---|
| Windows bent > 3° against the gyro | 0 on livox/1; ≤ 1 % on livox/2 | 16 of 260 | 168 of 1517 | 24 |
| Windows faster than 2.5 m/s | 0 | — | 111 | 10 |
| Path closure before loop closure, when the walk returns to its start | < 0.5 m per 100 m | — | broken | 4.5 m per 100 m |
| Height error on return to a known level | < 0.2 m per 100 m | — | broken | not measured yet |
| Loop closure's largest correction | < 1 m | 0.01 m | 6.89 m | 11.17 m |
| Bias estimate against the bias at rest | within 0.5 °/s | — | 6.3 °/s | 1.9 °/s |
| Chi² of the velocity jump, median / 95th percentile | 1–5 / < 20 (diagnostic, never the only reason) | 183 / — | 1375 / — | 857 / — |
| Wall thickness p50 / p90 | ≤ 20 / ≤ 60 mm | 23 / 73 mm | 23 / 99 mm | 22 / 118 mm |
| Neutral perturbations | every perturbed run passes the rows above, and they differ by < 0.5 m per 100 m | up to 18 m on 30 m | — | — |

Still missing for caves, to be defined with the user: drift through a long passage with little structure, and a
check against hand-measured distances (the planned map quality report).

## 6. Verify skills

Each skill runs fixed tools with fixed thresholds and prints a pass / warn / fail table, so every change is judged the
same way.

| Skill | Question | Tools |
|---|---|---|
| `verify-recording` | Is this recording usable? Rate, gaps, false gyro offsets, bias at start and end, noise, rest at the start | `tools/imu_health.py` (new), the recording's `timing*.json` |
| `verify-run` | Did the odometry behave? Bends, slides, bias, consistency statistics, closure, height, loop closure | `tools/run_health.py` (new), run with `log_window_diagnostics = true` |
| `verify-map` | Is the map better than the reference? Wall thickness, distance between two results, the three map-quality measures on 44, plan overlay for the user's look | `wall_precision.py`, `trajectory_seams.py`, `map_quality.py`, `plan_fit.py` + `plan_section.py` |
| `verify-robustness` | Does a small neutral change move the result? The noise floor | `tools/perturbation_sweep.py` (new), run in the background |

## 7. Open changes this touches

- `synchronized-retune` (L2, 4 of 12 tasks): its open items (degeneracy thresholds, sigma floor, `ahrs_gain`, the stairwell window) move into phases 3 and 4. Close as superseded, or keep for the L2: the user's decision.
- `imu-front-end` (5 of 13): D1, D2, D5 go to phase 2; its probe P1 (prior strength) is phase 3; P2 (gyro scale fit) is phase 1.
- `settings-audit` (not started): its sensor-dependent constants are phase 2; the rest stays a separate refactor.
- `map-quality-harness`: phase 0 extends its yardstick from 44 to the Livox recordings.
- `consistency-gyro-term`, `robust-loop-closure`, `floor-alignment-diagnosis`, `odometry-map-model`, `map-precision-leaks`: they study symptoms downstream of the odometry on 44; parked until phase 5.

## 8. From the cave research (2026-11-10)

Report: `reports/Cave LiDAR mapping best practice.md`, notes in `research_notes/`. Its numeric criteria are the
researchers' inferences from the sources, not published standards, and it found no real-cave evaluation of a
Mid-360-class sensor. Proposed changes to this plan, **not yet agreed**:

| Topic | What the research says | Proposal |
|---|---|---|
| Closure before loop closure | Cave grades expect a loop error below 1–2 %; independent tunnel tests of commercial scanners show 0.2–0.36 m spread on a 120 m loop | Two levels per 100 m: minimum 1 m, target 0.3 m (the draft's 0.5 m lies between) |
| Closure after loop closure | DistoX-class surveys reach about 0.1 % | Minimum 0.3 m, target 0.1 m per 100 m |
| A test without ground truth | Double walls far from the loop closure are the documented failure of commercial scanners | Standard cave test: walk the passage out and back; compare the two passes after a rigid alignment only (95th percentile ≤ 0.3 m, target 0.1 m) |
| Hand references | No published protocol; errors accumulate along the passage axis | Laser-disto distances between fixed marks (scale within 1 %, target 0.3 %); a water level or plumb line for height. Belongs to the planned map quality report. |
| Sharpness on rock | Sensor noise 2–4 cm; rock is rough, so wall thickness limits for buildings do not transfer | A separate limit for rock (5–8 cm), and "no growth between first and last visit" |
| Blind stretches | A consumer IMU carries the position for seconds, not tens of seconds (researchers' arithmetic: 4.5–31 s to 10 cm) | The result flags stretches the scan did not constrain for longer than that; they close only by loops, marks or a second pass |
| Degenerate directions | Best untuned result in a real mine: detection thresholds derived from the sensor's noise, a soft prior from the IMU on the weak directions only | Phase 3: replace the tuned information-ratio thresholds this way |
| Loop closure | Underground it fails by matching the wrong, similar-looking place; outlier rejection was essential (15 m without, 0.31 m with) | Phase 5: keep the gates, add outlier rejection of that kind |
| Calibration | Time offset within a few ms and axis alignment well below 1° before a gyro is trusted | Phase 1, as planned |
| Refinement after the odometry | Batch adjustment gains are modest (about 25 % on one benchmark) | After phase 5, not before |

One caution: the report's first-ranked design choice (cap the scan matching's information) was written on the reading
of F7 that section 1 has since put in doubt. Phase 0b still decides it.

## 9. Questions for the user

1. Direction A, B or C?
2. May L2 results on 44 change when a compensation is removed, as long as `map_quality.py` does not get worse?
3. Does the gated accelerometer mode (off under `imu_always_on`) stay, or go?
4. Static recording and the turn on the spot: when?
5. Are the draft pass criteria the right ones for "clean and precise", and what is missing for caves?

## See also

- [Roadmap (agent notes): current status](agents/roadmap.md)
- [User guide: tuning cheat sheet](index.html#tuning)
- [All documents](README.md)
