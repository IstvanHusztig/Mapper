# Region: map

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

The map region owns the NDT reference maps the odometry registers against: how placed points become cell statistics
(mean, covariance, normal, stored inverse), and what enters the maps and when: the seed from the initial points, the
update after each window, and the sliding rebuild. It owns no term: how a point is scored against a cell is
lidar-matching's.

## Contract

**Takes**
- Placed points (`Point3Di`, world frame) and a viewport (a position the normals are turned towards), from four callers
  of `update_rgd`:
  - `run_odometry.cpp::seed_map`: `state.initial_points` (front-end, raw sensor frame) placed with `state.m_g`; viewport
    `m_g.translation()`.
  - `run_odometry.cpp::add_window_to_map`: the window's `WindowIntermediate` points (decimated in the sensor frame by
    the front-end) placed with the window's final poses; viewport `window.poses[0].translation()`.
  - `consistency.cpp` (back-end, per chunk, its own cells) and `loop_closure.cpp` (back-end, target points).
- Held-out points (`holdout_block_s`, `holdout.h`) are removed before both odometry inserts.
- Settings: `in_out_params_indoor` / `in_out_params_outdoor` (`NdtGridParameters`, only `resolution_*` is read here;
  0.15 / 0.4 m in `l2-vertical.toml` and `livox-vertical.toml`, 0.1 / 0.3 m in `default_settings.toml`),
  `use_exact_ndt_bucket_statistics` (false in all three), `decimation` (0.03 m in `l2-vertical.toml` and `livox-vertical.toml`; 0.01 in
  `default_settings.toml` and built in),
  `sliding_window_trajectory_length_threshold` (200 m shipped; the built-in default in `odometry_config.h` is 5 m).

**Gives**
- `OdometryState::buckets_indoor` and `buckets_outdoor` (`NDTBucketMapType`, an `unordered_map<uint64_t, NdtBucket>`,
  `ndt_map.h`). Both grids are always filled: `ablation_study_use_hierarchical_rgd` is `true` and not a TOML key.
- Per cell (`NdtBucket`, `ndt_types.h`): `mean`, `cov`, `normal_vector` (zero = no normal), `number_of_points`, and the
  stored `cov_inverse` with `inverse_usable` / `inverse_computed`, refreshed by `refresh_cell_inverse` for every cell an
  `update_rgd` call added or changed, at its end. Readers go through `ndt_cell_inverse`: a cell with an inverse entry
  beyond ±100 000 is skipped.
- `decimate(points, x, y, z)`: one point per cell of the given size, the first after an unstable `std::sort` by cell
  key; the output is in key order. Called by `compute_step_1` (front-end, sensor frame) and by the rebuild (world frame).

**Invariants others rely on**
- Cell key: `ndt_cell_index` = `floor(p / resolution)` per axis, cast to `int16_t` and packed into 48 bits
  (`ndt_cell_key`), so the index wraps beyond ±32 768 cells per axis (±4.9 km at 0.15 m). The same key is used by
  `tools/ndt_cell_history.py`.
- **Default statistics** (`update_rgd_legacy`, `ndt_map.cpp:152`, with its steps `new_legacy_cell`,
  `add_point_to_legacy_cell`, `add_to_legacy_cov`, `legacy_cell_normal`; `refresh_cell_inverses` is shared with the exact
  path): a new cell's mean is its first point and never moves;
  its covariance starts at 0.03² I; the 2nd and 3rd points update the covariance about that mean; at the 3rd the normal
  is fixed (eigenvectors col 1 × col 2, turned towards that call's viewport); later points update the covariance only if
  `normal · (viewport − mean) ≥ 0`, but `number_of_points` counts every point (C33). The result depends on insertion
  order.
- **Exact statistics** (`use_exact_ndt_bucket_statistics = true`): Welford mean and covariance with the 0.03² I prior
  worth one point; a point on the far side of an earlier normal is skipped; the normal is recomputed after the call for
  the changed cells with ≥ 3 points, none if λ1 < 3 λ0 (`ambiguous_normal_ratio`). Order-independent (unit test).
- **When** (`process_windows` → `finish_window`, orchestrator's skeleton): the maps are constant during a window's
  iterations. After a window: gyro-bias and accelerometer carry, the tail re-propagation (`repropagate_window_tail`),
  the distance check, the next window's placement, then `add_window_to_map`. So the points enter with the re-propagated
  tail poses (C32, inertial's).
- **Sliding rebuild** (`add_window_to_map`, `run_odometry.cpp:718–746`): when `carry.acc_distance` (the sum over windows
  of the first-to-last-node distance) exceeds the threshold, both grids are cleared (`state.reference_points` is never
  filled in a run, only copied by a capture, so the clear always happens), the newer half by count of
  `carry.points_global` (every window point since the last reset, world frame, including the current window) is kept,
  decimated in the world frame (only if `decimation` > 0; the decimation comes after the choice of the newer half, so
  the kept points are not those of the window loop's own decimated cloud) and inserted with the current window's
  first node as viewport. The initial points are not
  in `points_global`, so the first rebuild drops them. A warning line and the `map rebuilds` counter report it.
- **Held-out points** (`holdout_block_s`) are removed by the callers before the map sees them: in `seed_map`
  (`run_odometry.cpp:658`) and in `load_and_predict` (`:947`), not inside `add_window_to_map`; the rebuild re-inserts
  from `carry.points_global`, which is already filtered. `update_rgd` takes `points_global` by non-const reference
  (`ndt_map.h:51`) and may change it.

## Owns

- `src/domain/ndt_map.{h,cpp}`: `ndt_cell_key`, `ndt_cell_index`, `decimate`, `update_rgd` (with `update_rgd_legacy`),
  `refresh_cell_inverse`, `ndt_cell_inverse`, `ndt_inverse_usable`.
- `src/domain/ndt_types.h`: `NdtGridParameters`, `NdtBucket`.
- `src/application/run_odometry.cpp::seed_map` and `::add_window_to_map` (the rest of the file is the orchestrator's;
  `carry.points_global`, `acc_distance` and `map_rebuilds` live in `WindowLoopCarry`, `window_loop.h`, the
  orchestrator's).
- Test: `tests/ndt_map_test.cpp`.

## How to check your work

- **Unit tests** (Release): `out/build/x64-Release/bin/lidar_odometry_tests.exe -sf="*ndt_map_test.cpp"`. Eight cases:
  equal cell width through 0; exact statistics on a plane (centroid, normal), independent of insertion order, no normal
  below 3 points or for points on one line, the far side of a thin wall kept out; the default statistics (first point as
  mean, normal from the first 3); the stored inverse equal to `cov.inverse()` after one and two calls.
  `tests/scan_to_map_test.cpp` builds its room and corridor maps with `update_rgd` and the exact statistics.
  Nothing tests `decimate`, `seed_map`, `add_window_to_map` or the rebuild directly.
- **Replays** (`python tools/run_replay.py <capture> <settings>`): the captures (44_short 20–40 s, Livox 3 100–130 s)
  hold the maps from before their range, so `add_window_to_map` and `update_rgd` run in them; `seed_map` does not. The
  rebuild fires on the full 44 once, at window 586 (≈ 237 s, tracker C4); neither capture nor 44_short reaches 200 m.
  A replay with a lowered threshold (a settings copy) makes it fire inside a range; with other settings the replay
  reports how far the windows move.
- `python tools/step_checks.py`: whether a change keeps every bit (the stage 7 try-out, splitting
  `update_rgd_legacy`, is checked this way).
- **Readings that are the region's:** the warning `window n: reference map rebuilt from the newer half of its points`;
  `silent events: map rebuilds n` at the end of the odometry; with `log_stage_timing` the window loop's `add_to_map`
  phase. The map has no row of its own in the per-term table, the `window noise` line or the window verdict: its cells
  act through the NDT rows (lidar-matching). `log_window_diagnostics`' yaw stiffness and NDT cost profile read the maps
  (noise-budget's lines).
- **Tools:** `tools/map_quality.py`, `tools/plan_fit.py` / `plan_section.py` through the skill `verify-map` (any map
  change); `verify-robustness` perturbs `decimation` by ±1 mm (`tools/perturbation_sweep.py`). `tools/ndt_cell_history.py`
  and `tools/analyse_ndt_cells.py` recover cell insertions and per-cell evidence from the ndt-information-probe's
  exported traces, snapshots and caches; they don't work on a plain run's output.

## Known flaws

- **F9** (verified, `consistency-harness` 2026-11-20). `ndt_map.h::ndt_inverse_usable` skips a cell when an ENTRY of its
  inverse covariance exceeds 1e5, which is not rotation-invariant: a plane thinner than 3.162 mm along its normal is
  dropped when axis-aligned, 2.238 mm when turned 45 deg. Pinned by `tests/unit_cases_ndt_test.cpp` "unit case K9 marker"
  (`doctest::should_fail`). The synthetic loops use 1 cm point jitter for this reason. Planned as step 7 of the
  harness's order (`openspec/changes/archive/2026-11-21-consistency-harness/tasks.md` 8.2). Details: `lidar-matching.md`, `verification.md`.
- **C4** (verified). `run_odometry.cpp::add_window_to_map`: the rebuild above. On 44 it fires at window 586 (≈ 237 s),
  together with a −100° yaw correction and the gyro-bias estimate collapsing at 240–260 s; one rebuild on each of
  s0/s1/s2. Planned as F1.
- **C14** (reported; crosses to front-end). `ndt_map.cpp::decimate` keeps the first point of each cell after an unstable
  `std::sort` (`ndt_map.cpp:44`). Stable sorts were tried and changed nothing (2026-10-16).
- **C20** (verified). `ndt_map.cpp::update_rgd_legacy`: the mean is the first point, the covariance is about it, the
  normal fixed from the first 3 points. The exact statistics exist behind `use_exact_ndt_bucket_statistics`, off in all
  shipped settings (see Experiments). Next step recorded in the tracker (F2): exact statistics on top of C10 under the
  shipped thresholds, mean and covariance tested separately.
- **C30** (reported; crosses to front-end). `compute_step_1` decimates in the sensor frame (`lidar_odometry.cpp:404`),
  `decimate` returns the points in sensor-frame key order (`ndt_map.cpp:44–52`), and a new cell's mean is its first point
  (`ndt_map.cpp::new_legacy_cell`): each new cell's mean is its smallest-key point, a constant body-fixed pull. Test asked: reversed /
  shuffled insertion order, yaw probe.
- **C33** (reported; neutral for yaw). `update_rgd_legacy` increments `number_of_points` (`ndt_map.cpp:117`,
  `add_point_to_legacy_cell`) before the viewpoint gate skips the covariance update (`:141`), so later updates are weighted by a count that includes skipped
  points.
- **C21** (verified; crosses to front-end). `seed_map` places the ≈ 0.94 s of initial points with the one pose `m_g`.
- **C32** (inertial's, crosses to map): the window's points enter the map with the tail re-propagated from the gyro
  (`finish_window`, `run_odometry.cpp:1179` before `:1206`).
- **Not in the tracker:** M6 (`docs/agents/roadmap.md`, 2026-11-15): every window enters the map unchecked; one wrong
  window corrupts the map and the next windows confirm it.
- Accepted as they are: C10 (`ndt_cell_index` floors instead of truncating towards zero; the fix accepted by the user,
  c22d79a, 2026-10-14).

- **C40** (verified): `state.reference_points` is never filled in a run, so the "clear only if empty" test of
  `add_window_to_map` is always true and a rebuild always clears the maps (see the rebuild above).

## To split in the first job

None (2026-11-18): `update_rgd_legacy` was split in the stage 7 try-out (now 21 lines). The longest function of
`ndt_map.cpp` is `update_rgd` (74 lines, the exact statistics). `seed_map` and `add_window_to_map` are under 80.

## Experiments

- **2026-11-18** Stage 7 try-out (the first region job): `update_rgd_legacy` split into `new_legacy_cell`,
  `add_point_to_legacy_cell`, `add_to_legacy_cov` (the covariance update written out three times before),
  `legacy_cell_normal` and `refresh_cell_inverses` (shared with the exact path), bit-identical: `step_checks.py` ALL
  HOLD on the committed branch (both replays 0.000000 m, the per-term table and 44_short identical), the map's 8 unit
  tests pass, the conclusion reviewer found every expression unchanged. Merged f285eda. Evidence:
  `experiments/regions/map/2026-11-18-split-update-rgd-legacy/`.
- **2026-11-13** Livox 2 window 754 (271.08 s): three high-occupancy outdoor-grid cells carry the far-range terms;
  omitting only those three: tilt 15.19 → 2.33°, end speed 0.89 → 0.36 m/s. Their insertion histories (windows
  623–633): no rejected viewpoint updates, thin geometry. Not a filter to ship (hard-coded cells).
  `openspec/changes/archive/2026-11-25-ndt-information-probe/full-livox2-cell-attribution.md`, `full-livox2-w754-ablation.md`.
- **2026-11-10/12** `decimation` ±1 mm moves Livox 1 by 18.67 m before loop closure (18.46 m final); on the Livox 1
  sweep 0.029 m broke the run at both rotation-sigma scales tried (18.5 m, 24.3 m). `docs/agents/roadmap.md` (phase 0a);
  memory `fast-turn-fragility-state.md`.
- **2026-10-17** Map insertion order reversed (cut s0–s2): gallery unchanged, start +0.4°. Memory `fix-plan-state.md`
  (change corridor-heading, deleted). Whether this is the order C30 asks for is not recorded.
- **2026-10-16** Finer cells 0.10 / 0.27 m (V5), odometry only on the 95 s cuts: gallery − start A −3.16° against
  master's −4.31° (+1.26° paired); dose-response F1 0.12/0.32 +0.32, F3 0.08/0.21 +0.89, F4 indoor only +0.57, F5
  outdoor only +0.74. Full 44 s0–s5 with loop closure and consistency: façade −0.11 / 0.00° (fails the 2a rule), jitter
  p99 −1.03°, largest loop correction 0.76–1.21 m (master 1.06–3.00 m). Odometry-only full maps: façade −3.01 / −3.22°
  against −4.18 / −4.45°. Parked without a verdict. `openspec/changes/archive/2026-11-25-odometry-map-model/design.md`.
- **2026-10-16** Screening on the cuts: exact statistics (V3, C20 on top of C10) ran away on s0 and s2 from ≈ 45 s, no
  move on the gallery; 10 000 / 120 000 initial points (V4, C21) do not count. Same design.md.
- **2026-10-16** Grazing points dropped (α > 70°): no move on 0.10 m cells (+0.08 ± 0.11°), +0.46 ± 0.12° on 0.15 m
  cells: cell size and grazing points act on the same part of the drift. Planar cells are 5.5 mm thick (median), flat in
  incidence and range. Same design.md, 2b (crosses to lidar-matching).
- **2026-10-16** Stable sorts in `decimate` and of the points by time (C14): no effect.
  `openspec/changes/archive/2026-10-16-deterministic-point-order/`.
- **2026-10-14** C10 (floor in `ndt_cell_index`) on 44 s0–s5: gallery coherent in 6/6 runs (3/6 before), revisit worst
  spread 6.3 → 1.1°, offset spread 7.2 → 0.2 m. Tracker C10.
- **2026-10-13** Exact statistics with halved projection thresholds on s0–s2: façade out −3.64/−3.05/−3.64° (0.5–1.3°
  less drift) but revisits broken on s0/s1, 240–2220 big-jump rejections; the control (default statistics, halved
  thresholds) did not change the drift. Tracker C20.
- **2026-10-12/13** Map horizon (`ndt_map_horizon_m`, branch `odometry-map-horizon-wip`, never on master): fixes the
  corridor snap, but loop closure then no longer removes the heading drift. Cause search for the −4.0 ± 0.4° drift: cell
  size 0.10/0.27 m → −3.1°; insertion order, grid orientation, range > 12 m ruled out; ignoring sparse frontier cells
  −5.9°. Memory `odometry-map-horizon.md`.
- **2026-10-11** Point-to-plane NDT (a plane information stored with the cell inverse): 44 s0 route 455 → 743 m, 6669
  big jumps, loops 39 → 2. Removed. `openspec/changes/archive/2026-10-11-point-to-plane-ndt/tasks.md`.
- **2026-10-08** Stored cell inverses (`refresh_cell_inverse`): bit-identical, odometry 517 → 490 s on 44.
  `docs/PERFORMANCE.md` fix 12.
- **2026-10-04** Exact statistics and the ambiguous-normal rule: synthetic corridor information along / across 925 / 155
  → 1.7 / 987, room position error 1.5 → 0.37 mm; on 44 the degeneracy gate then fired far more often. Rejected for
  now, kept behind the switch. `openspec/changes/archive/2026-10-04-ndt-bucket-statistics/`,
  `docs/degeneracy-information.html` section 5.

## Ideas rejected

- **Exact cell statistics as the default:** rejected 2026-10-04; runaways again 2026-10-13 and 2026-10-16 (above). The
  code stays behind `use_exact_ndt_bucket_statistics` with its tests.
- **Exact statistics with the degeneracy threshold lowered to 0.10–0.15:** loop-closure corrections of metres
  (`docs/degeneracy-information.html` 5b).
- **Planar cells regularized to 0.3 m in the plane** (less stable; 5b) and **point-to-plane NDT** with
  `ndt_plane_in_plane_sigma_m` (removed 2026-10-11).
- **Stable sorts in `decimate`** (C14), rejected 2026-10-16.
- **Ignoring sparse frontier cells:** the drift grew (−5.9° against −4.0°; memory `odometry-map-horizon.md`).
- **A flat lookup table for the cells:** measured as not worth it (`docs/PERFORMANCE.md`, planned order item 3).

## Open questions

- No unit test covers the default statistics' view-point gate on the 4th and later points, nor the count C33 that
  the gate leaves behind; the exact statistics' shared tail (`refresh_cell_inverses`) is checked by the unit tests only
  (no replay runs `use_exact_ndt_bucket_statistics = true`).
- The map horizon: reject, or keep for an "old cells constrain rotation only" design (pending since 2026-10-13).
- Finer cells (V5): judge again on top of the IMU front-end fixes, with consistency kept at 0.15 m as a control and a
  map-level sharpness measure (`odometry-map-model/tasks.md` 3.4).
- C20 (F2) and C30's order test; C4's fix (F1): whether a rebuild is needed at all, and if so by distance rather than
  by point count, keeping the initial points.
- A check before a window enters the map (M6).
- An incidence-dependent cell covariance (G2) was never run; left for the cave retune (change I).
- From `livox4-double-walls-cause` (closed 2026-11-25, its fixes shipped elsewhere): whether the odometry's map
  horizon, the consistency pass (off for the Livox) or the gaps between visits still leave double walls on another
  recording; its sections 4-6 and the physical check 3a.2b are the record
  (`openspec/changes/archive/2026-11-25-livox4-double-walls-cause/`).

## See also

- [Regions](README.md)
- [Degeneracy information](../degeneracy-information.html) (section 5: the cell normals)
- [Performance](../PERFORMANCE.md) (fix 12, stored cell inverses)
- [Code-review tracker](../code-review-tracker.md)
- [Pipeline](../PIPELINE.md)
- [All documents](../README.md)
