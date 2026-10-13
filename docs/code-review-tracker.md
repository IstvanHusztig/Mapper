# Code-review tracker (review of 2026-10-13)

One row per finding of the code review of 2026-10-13 (four review agents; main claims checked in the code). The full write-up with code quotes is on branch `odometry-map-horizon-wip`, `openspec/changes/odometry-map-horizon/code-review-2026-10-13.md`; the plan that orders the work is in the change `map-quality-harness` (change A) and the changes B–I that follow it.

Map quality is judged on three equal measures: **global geometry** (angles and distances true; façade angle start room vs gallery on 44), **consistency** (no doubled walls) and **sharpness** (railing nearest-neighbour check), on s0/s1/s2 against the noise floor (change A).

Status flow: `reported → verified → fixed → measured → accepted | rejected | correct-but-unmasks-compensation (→ retune)`.
- **verified**: a red unit test or a code proof (noted in the row); nothing is fixed while only *reported*.
- **correct-but-unmasks-compensation**: right in the code but measures worse because settings were tuned around the bug; the settings are retuned and it is measured again, not rejected.

**Retuning the settings** (user, 2026-10-13): settings tuned around a bug are retuned in two stages.
1. **Within the change of the fix**, where the compensating setting is obvious. Examples: `rotation_sigma_inflation_k` and `gyro_arw_*` after C5/C6; the consistency weights and passes after C1.
2. **Once all fixes are in (change I)**, a systematic retune of the important settings, measured with the harness on s0/s1/s2 and also on 45f and, if possible, another recording, so the settings are not fitted to recording 44 alone.

The settings audit (`openspec/changes/settings-audit`) follows after that.

**Separate task: a cave recording for tuning** (user, 2026-10-13). A cave scan will be made for the final retune; it has to be organised as a task of its own (site, route with deliberate revisits and a closed loop, stationary start of 30–50 s for the gyro bias, ideally a few reference measurements such as tape distances between marked points). Stage 2 of the retune waits for it.

| ID | Finding | Where | Change | Status | Evidence / notes |
|---|---|---|---|---|---|
| C1 | Consistency adds high-frequency pose jitter: one 6-DOF unknown per IMU node, smoothness weights constant (not scaled by dt), no prior to the input poses, only node 0 held, 10 passes | `consistency.cpp` | H (A.5a first) | verified | measured: rotation over 10 nodes p50/p99 0.40/1.29° (odometry) → 0.94/4.49° (final); route at node rate 218 → 455 m |
| C2 | AHRS start-up: windows start inside Fusion's initialisation (heading reset to 0 every sample) while the gyro-bias estimator starts | `fusion_orientation_estimator.cpp`, `FusionAhrs.c:279`, `lidar_odometry.cpp` | D1 | verified | code; early z-bias estimates swing −1.2…−0.2 °/s (log) |
| C3 | `set_gain` writes `settings.gain` only; Fusion integrates with `rampedGain`, latched at the end of initialisation: the dynamic gain has no effect | `fusion_orientation_estimator.cpp` | D2 | verified | code |
| C4 | Sliding-map rebuild at 200 m: map cleared and rebuilt from the newer half of the points (re-decimated, viewport = current position) | `run_odometry.cpp` `add_window_to_map` | F1 (A.5b first) | verified | fires at window 586 (≈ 237 s) on 44; coincides with a −100° yaw correction and the bias estimate collapsing at 240–260 s; counter: 1 rebuild on each of s0/s1/s2 |
| C5 | IMU rotation prior × `cauchy(measurement)` = 1/π: prior π× weaker than configured, not robust | `scan_to_map.cpp:586–588` | D4 | verified | code |
| C6 | Carried gyro bias subtracted twice (guess + residual); estimate 30–50 % low | `window_steps.cpp` `apply_gyro_bias_prior`, `scan_to_map.cpp:507–512` | D3 | verified | code + true bias from the trajectory |
| C7 | Solver step skipped silently when an entry of the sparse solution is exactly zero | `scan_to_map.cpp`, `consistency.cpp:320` | B | verified | code; hit by the rotation-only experiment; 0 times on 44 s0/s1/s2 (counter, 2026-10-13) |
| C8 | "big jump": a node > 1 m from its guess is not updated while the rest of the step is | `scan_to_map.cpp:1111` | B | reported | 0 times on 44 s0/s1/s2 (counter) |
| C9 | Loop closure `shift` and plausibility measured as translation of a world-frame transform (rotation about the origin counts; cross-floor limit 1.0 m hard-coded) | `loop_closure.cpp:771`, 636–639, 778 | B | verified | code |
| C10 | `ndt_cell_index` truncates towards zero: cells at index 0 double width | `ndt_map.cpp` | B | verified | code; grid rotation showed no effect on drift |
| C11 | `.mrp` / `.mri` written with 6 significant digits | `folder_result_sink.cpp:109` | B | reported | ≈ 5 mm beyond 1 km |
| C12 | `point_sizes_per_chunk.json` holds sizes per window; CSV `om/fi/ka` are raw AHRS angles, not the optimized pose | `lidar_odometry.cpp:482`, 579 | B | reported | |
| C13 | LM never checks the cost; damping ×10 every 10 iterations; always 100 iterations; depends on wall-clock time (`real_time_threshold_seconds`) | `scan_to_map.cpp`, `run_odometry.cpp` | C1 | reported | "every window runs all 100 iterations" matches the logs |
| C14 | `decimate` keeps the first point after an unstable `std::sort` | `ndt_map.cpp:42` | C2 | reported | |
| C15 | Hand-off off-by-one (node 0 of the next window at t_n gets the pose of t_{n−1}; points in [t_{n−1}, t_n) dropped); windows with ≤ 1000 points dropped silently | `run_odometry.cpp`, `window_steps.cpp`, `lidar_odometry.cpp:353, 385` | D5 | reported | |
| C16 | Fractional degenerate projection: curvature scaled s², gradient s | `scan_to_map.cpp:464–485` | E1 | reported | |
| C17 | Acceleration offset modelled in the world frame; accelerometer bias is body-frame | `scan_to_map.cpp:805`, 900–904 | E2 | reported | possible "tilt ≈ 1°" |
| C18 | Coarse 0.4 m grid at full weight on top of the fine grid | `scan_to_map.cpp:349–444` | E3 | reported | finer cells reduced drift ≈ 25 % |
| C19 | Points placed with the node before them, no interpolation; IMU gaps (58 of 3–12 ms, one 0.40 s at 2.6 s on 44) | `lidar_odometry.cpp:368–376`, `point_placement.h` | E4 | reported | |
| C20 | Legacy cell statistics: mean = first point, covariance about it | `ndt_map.cpp` `update_rgd_legacy` | F2 | verified | code; insertion order alone does not change the drift |
| C21 | Initial map: 60 000 raw points placed with one pose (≈ 1 s, no motion compensation) | `lidar_odometry.cpp` `compute_step_1`, `run_odometry.cpp` `seed_map` | F3 | verified | code |
| C22 | Loop closure: 30 s minimum gap leaves the turnaround of an out-and-back gallery without loop edges | `loop_closure.cpp:728–730` | G1 | verified | setting |
| C23 | Same-floor loop edges with fixed weights (5 cm / 0.3°); odometry edges equally stiff | `loop_closure.cpp:472–475` | G2 | verified | code |
| C24 | No robust kernel (`loop_closure_kernel_scale` 0); one edge per segment; coarse-search ties broken in loop order | `loop_closure.cpp` | G3 | verified (kernel) / reported | |
| C25 | Loop edge pinned at the segment's first node, correction interpolated between first nodes | `loop_closure.cpp:659`, 935–937 | G4 | reported | |
| C26 | Gyro bias absorbs a constant LiDAR yaw drift (observability) | optimizer / bias carry | feature change | reported | expected by theory |
| C27 | The build does not warn about a non-void function without `return` (MSVC C4715 not active): found when a missing `return` in a new diagnostic made a check fire every window (2026-10-13, fixed before commit) | `CMakeLists.txt` warning flags | B | verified | 44_short run with the bug: rebuild warning in all 70 windows, summary counter 0 |
| — | Tait-Bryan gimbal lock with the vertical mount | — | — | rejected | pitch never above 23° on 44 (measured) |
