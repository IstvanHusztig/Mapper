# Design

## Context

- **Point terms:** each NDT point term in `optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`, the indoor and outdoor lambdas) uses the cell's stored inverse covariance (`ndt_cell_inverse`, `src/domain/ndt_map.h`) in `point_to_point_source_to_target_tait_bryan_wc_AtPA/AtPB_simplified`.
- **A planar cell's inverse:** its in-plane stiffness is about 1/(cell size²/12), roughly 500 m⁻² at 0.15 m, against 10⁴ m⁻² or more along the normal. That in-plane part is the frontier drag.
- **The window's weakness:** after each window, `update_acc_term_after_window` (`src/application/run_odometry.cpp`) computes the LiDAR translation information's eigen-decomposition, ratio and weakest direction (`carry.weak_direction`), but only when the accelerometer gate is on (`setup.acc_gated`). The shipped settings have `imu_always_on`; whether `acc_gated` is then set needs checking (task 1.1).
- **The degeneracy projection** (shipped 0.1 / 0.2) removes all NDT translation information along weak directions below ratio 0.2, inside the window from its own matched normals.
- **Measured (rejected `point-to-plane-ndt`, findings 2.2):**
  - the stall windows 169–178 have ratio 0.21–0.40 and weakest direction ±(0.95, 0.3, 0.1), stable from window to window;
  - the stairs (210–217) have 0.25–0.53, weak direction about (±0.5, ∓0.8, 0) along the run;
  - pure point-to-plane jumped one step at window 212.

## Goals / Non-Goals

**Goals:**
- Remove the drag along a passage without opening the stairs' step alias.
- A clean go/no-go before any tuning.

**Non-Goals:**
- Changing the projection (`degenerate_projection_*`) or the accelerometer term.
- Point-to-plane in general; the cell statistics.
- The loop closure (a segment-weighted pose graph was tried and rejected, findings 2.2).

## Decisions

1. **Release only the in-plane part of the weak direction, weighted by how much of it lies in the plane.**
   - For a planar cell with normal n and the window's weak direction w (unit):
     - u = w − (w·n) n, the in-plane part, with |u|² ∈ [0, 1];
     - û = u / |u|;
     - the strength is s_cell = s · |u|².
   - The released information is M' = Q M Q with Q = I − (1 − √(1 − s_cell)) û ûᵀ. Its stiffness along û is scaled by (1 − s_cell); it is unchanged along n and along the in-plane direction across û (Q leaves both unchanged).
   - Corridor walls and floor (n ⟂ w, |u| ≈ 1) lose their along-axis drag. Risers on stairs (n ≈ run ≈ w, |u| small) keep their vertical in-plane anchors, which break the step alias. Treads lose their along-run pull; the risers' normals and the IMU carry that.
   - **Alternatives:**
     - Pure point-to-plane in weak windows: the ratio doesn't separate the stall from the stairs (0.21–0.37 against 0.25–0.36 on s1), so it would break the stairs.
     - Releasing all of the in-plane stiffness only in cells whose normal is ⟂ w: a hard switch at a normal angle; |u|² gives the same effect smoothly.
2. **The weak direction and the ratio come from the previous window**, as the accelerometer gate already does, so the window's terms are fixed before its iterations.
   - The strength is s = clamp((r_off − ratio) / (r_off − r_full), 0, 1), with r_off = `ndt_in_plane_release_ratio` and r_full = `ndt_in_plane_release_ratio_full`.
   - The release doesn't change between iterations, so the solve's convergence is unaffected.
   - **Alternative:** the window's own normal information from its previous iteration. It is more current, but the terms then change between iterations and the LM step test gets harder. Not chosen; a follow-up if the previous window lags at passage entries.
3. **Planarity** as in the rejected point-to-plane change: λ₁ ≥ 3 λ₀ and λ₀ < (4 cm)². The test runs on the cell's covariance, and is stored in the cell next to its inverse (normal plus a planar flag), only with the release on. The per-point work is a 3×3 product, with no eigen-decomposition.
4. **Settings:** `ndt_in_plane_release_ratio` (0 = off) and `ndt_in_plane_release_ratio_full` in `[lidar_odometry_control]`. The measurement starts with 0.4 / 0.25: full release at the stall windows' 0.21–0.30, partial up to 0.40, none in ordinary rooms (median ratio 0.48).
5. **Go/no-go first.**
   - Runs: full odometry with the shipped settings plus the release and the window diagnostics, on `44_s0`, `44_s1`, `44_s2` and `45f_p`, and the `/fp:fast` build on `44_s0`. Two at a time; the memory reaper stopped a batch once, so each run is its own background job.
   - **Pass, all of:**
     1. no runaway on any input: route within 2 % of shipped, no over-speed windows, no `big jump` warnings beyond shipped's;
     2. accepted loops within 10 % of shipped on every input;
     3. the stairs hold: windows 205–225 within 5 cm per window of shipped on s0 and s2;
     4. the `44_s1` stall: the s1 − s0 odometry difference along x at 72.5 s ≤ 0.3 m (today 1.65 m);
     5. `44_s1`'s gallery façade within 20 cm of `44_s0`'s (today −56 against +34 cm);
     6. no criterion of `tools/evaluate_floors.py` worse than shipped by more than its spread on s0 and s2;
     7. `45f_p` within 10 cm of shipped (`trajectory_seams.py`).
   - **On fail:** the code is removed, the numbers recorded, and the change archived as rejected.

## Risks / Trade-offs

- [The release lets treads slide along the run, and the risers' normals plus the IMU don't hold the step] → criterion 3 measures exactly that window range.
- [The previous window's weak direction is stale at a passage entry or exit] → there it is mostly not weak (ratio above r_off), so the release is off. The diagnostics show any lag.
- [Interaction with the projection: below 0.2 both act] → there the projection already removes the direction completely, so the release changes little. That is accepted.
- [A run killed by memory pressure] → one background job per run, and logs checked before judging.

## Migration Plan

- Off by default, bit-identical.
- On pass: `settings/lidar-vertical.toml` gets the two ratios, new 44 and 44_short baselines in their own commit, and docs.
- On fail: removal in one commit.
