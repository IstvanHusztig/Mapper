# Region: lidar-matching

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

The LiDAR side of a window's optimization: every point of the window against the NDT cell it falls into, as a 6x6 block
and a right-hand side for its node. It applies the per-point rules the noise model names, collects the window's
translation information from the matched cells' normals, and projects the NDT blocks away from translation directions
that information calls blind.

## Contract

**Takes**
- From the window loop, through `WindowProblem::linearize` (solver region): the window's points (`std::vector<Point3Di>`,
  each with its node in `index_pose`, coordinates in the sensor frame after the lever arm), the window's node poses
  (`Window::poses`, world frame), the two NDT maps (`NDTBucketMapType`, map region) and `ScanToMapOptions`
  (`optimizer_options.h`).
- From the noise-budget region: `NdtNoise` (`window_noise_model.h`, built by `ndt_noise`): planarity, the range weight and
  its cap (`ndt_weight_cap` = 10, indoor grid only), the Huber cutoff (`ndt_huber_cutoff`), the information scale
  (`ndt_information_scale`), the projection ratios (`degenerate_projection_ratio`, `_full`). The formulas of these weights
  are the noise-budget's; this region applies them in this order: cell information, planarity, range weight and cap,
  Huber weight, information scale (`ndt_term.cpp::add_ndt_point_term`, `apply_huber_and_scale`).
- From the map region: each cell's `mean`, `cov`, `normal_vector` and stored inverse (`ndt_cell_inverse`, `ndt_map.h`),
  and the cell index `ndt_cell_index`.

**Gives**
- `sum_ndt_point_terms` adds into the solver's `WindowEquations` (`window_equations.h`). Without the projection the
  point blocks go straight into the matrix; with it (`projection_ratio_full > 0`) they are collected per node in
  `NdtSums` and `add_projected_ndt_blocks` adds them as `M B M`, `M g` with `M = diag(P, I)`.
- `NdtSums`: per-node blocks, the normal information `sum n n^T` (indoor grid only, cells with a normal of squared norm
  > 0.5), and with the cost (`lm_step_check`, the per-term table) the cost `w r^T Sigma^-1 r` (Huber cost when on, times
  the scale) and the matched / unmatched counts.
- `window_translation_information(NdtSums)`: the window's 3x3 translation information (world frame). `WindowProblem`
  writes it at every linearization; the window verdict's information ratio and the weak axis of `log_window_diagnostics`
  read it.
- To the noise-budget's per-term table (`window_term_table.cpp`): `indoor_ndt_grid`, `outdoor_ndt_grid`,
  `sum_ndt_grid_blocks`, `ndt_degeneracy_projection`, `add_ndt_blocks`.

**What the others rely on**
- Unknowns: 6 per node, `[px py pz om fi ka]`, Tait-Bryan in the world frame (`TaitBryanPose`). A point term writes only
  its own node's diagonal 6x6 block; the residual is `R p + t - mean` with the cell's inverse covariance, Jacobians from
  the generated `point_to_point_source_to_target_tait_bryan_wc_*_simplified` (`3rdparty/observation_equations`).
- Bits: the nodes run in parallel (`std::execution::par` when `scan_to_map.multithread`), each node's points in point
  order (`index_points_by_node`), so the sums are the same with or without threads. The indoor grid is summed before the
  outdoor grid. The order of these additions must not change (`tests/scan_to_map_test.cpp` checks threads vs no threads).
- A point takes part only if its range lies in `[min_distance, max_distance]` (indoor `min_distance_lidar`, outdoor
  `min_distance_lidar_outdoor_grid`), its cell exists with a usable inverse, and (view-point test,
  `use_view_point_and_normal_vectors`) the node lies on the normal's side of the cell. Otherwise it is "unmatched" and
  costs nothing.
- With the Huber kernel on, a non-finite distance throws `std::domain_error` after the grid's loop.
- Not TOML settings (fixed in `OdometryConfig`, `ablation_study_*`): planarity off, range weight off, the outdoor grid
  on (`use_hierarchical_rgd`), the view-point test on. So in the shipped settings each point's weight is 1 before Huber
  and scale. Shipped grids: 0.15 m indoor, 0.4 m outdoor (`l2-vertical.toml`, `livox-vertical.toml`); projection 0.1 /
  0.2 in both; Huber 3 and scale 0.1 in `livox-vertical.toml` only.
- `ndt_window_cost` (`scan_to_map.cpp`, the solver's file) repeats the point rules for the diagnostics
  (`log_window_diagnostics`): planarity, the range weight and its cap, the Huber cost and the information scale have a
  second copy there, beside `ndt_term.cpp::ndt_point_weight` and `apply_huber_and_scale`. A change of a rule here must
  be made there too (orchestrator / solver).

## Owns

- `src/domain/ndt_term.h`, `src/domain/ndt_term.cpp`: the point terms of both grids (`sum_ndt_point_terms`,
  `sum_ndt_grid_blocks`, `NdtGrid`), the Huber kernel (`ndt_huber`), the per-point weight, the degeneracy projection
  (`ndt_degeneracy_projection`, `add_ndt_blocks`, `add_projected_ndt_blocks`) and `window_translation_information`.
- Its cases in the shared test files `tests/window_terms_test.cpp` and `tests/scan_to_map_test.cpp` (below).

## How to check your work

- **Unit tests** (`out/build/x64-Release/bin/lidar_odometry_tests.exe`):
  - `tests/scan_to_map_test.cpp`: a perfect guess stays at the truth; where a plane lies in its cell; a yaw drift is
    corrected; translation information in a closed room and a degenerate corridor; degeneracy-aware NDT terms in a
    corridor and a room; threads vs no threads and two threaded runs bit for bit; stored cell inverses vs inverting on
    the spot; the window cost lowest at the truth; the NDT point terms' right-hand side is minus half their cost's
    gradient (with and without Huber and scale); the per-term table's NDT cases (sideways offset and stretch, corridor).
  - `tests/window_terms_test.cpp`: "the degeneracy projection passes strong directions and removes weak ones".
  - Run these (the `scan-to-map*` pattern also runs the solver's and the inertial region's cases of that shared file): `lidar_odometry_tests -tc="scan-to-map*,window terms: the degeneracy*,window terms: the NDT*,window term table: NDT*,window term table: in a corridor*"`.
  - The characterization guard ("scan-to-map characterization", skipped; it covers Huber and scales, the projection
    and the cost) runs inside `python tools/step_checks.py`.
- **Replays:** `python tools/run_replay.py <capture> <settings>`; then `python tools/step_checks.py` (both captures,
  44_short 20-40 s and Livox 3 100-130 s, the per-term table, 44_short) says whether a change changes a number.
- **Readings that are this region's:**
  - per-term table (`log_window_terms = true`, `python tools/window_terms.py <csv> --window N` or a range): the rows
    `ndt_indoor`, `ndt_outdoor` (as solved, after the projection) and `ndt_unprojected` (before it); their hold and pull
    per motion, their share of the weight. `window_features.csv`: information ratio, weak axis, matched points.
  - window verdict: the information ratio (a reading, no flag) and, with `log_window_terms`, the LiDAR's weight share
    along travel (`lidar_share_along`, from the `ndt_*` rows).
  - `window noise` line (`log_window_noise`): `ndt information scale`, `huber cutoff`.
  - `log_window_diagnostics`: `window diagnostics N: ... info ratio, weak` and, below a ratio of 0.45, the weak-axis line
    (NDT stiffness along the weak axis against the velocity prior and motion model, NDT cost profile ±2 m and its minima)
    and the heading line (yaw stiffness of the NDT cost).
- **Tools:** `tools/window_terms.py`, `tools/trajectory_seams.py` (two results);
  verify skills: `verify-run` on every replay log or run, `verify-robustness` before one variant is said to beat another.

## Known flaws

- **C16** (reported): fractional degeneracy projection. For a direction with scale `s` between 0 and 1 the projected
  block carries `s^2` of the curvature but `s` of the gradient, so along that direction the NDT step is `1/s` times the
  unprojected one. Today: `ndt_term.cpp::ndt_degeneracy_projection` (the ramp of `s` between `projection_ratio` and
  `projection_ratio_full`) and `add_ndt_blocks` (`M B M`, `M g`). Active in both shipped settings (0.1 / 0.2).
  Verified 2026-11-20 (red unit test; marker K7): on a hand-made one-node weak-axis block the block's own step is
  g / (s k): 1 g/k at s = 1, 1.33 at 0.75, 2 at 0.5, 4 at 0.25. Pinned by `unit case K7 marker: the fractional
  projection's own step at s = 0.5 is at most g / k` (`tests/unit_cases_ndt_test.cpp`, `doctest::should_fail`; the
  oracle is an argued bound, a projection that removes information must not lengthen the step: the weakest of the
  markers). The audit: active in the Livox 3 blind-exit window 602 (s = 0.8); with the shipped 0.1 / 0.2 thresholds 166
  of 702 windows would lie in the ramp. Order: step 7 of `openspec/changes/consistency-harness/tasks.md` 8.2, after F9,
  each alone.
- **C18** (reported): the coarse 0.4 m grid adds its point terms at full weight on top of the fine grid
  (`ndt_term.cpp::sum_ndt_point_terms`, `outdoor_ndt_grid`; on through `ablation_study_use_hierarchical_rgd`, default
  true, not a TOML key). Crosses to the map (its cells) and to the noise-budget (its weight). Measured 2026-11-20 as
  part of F4 (K8, Experiments): the second grid adds 1.67-1.84× claimed information while the scatter stays.
- **F9** (from the map's side, read here: the code is `ndt_map.h`, map region; verified: red unit test; marker K9): the
  ±1e5 test on a cell's stored inverse (`threshold = 100000.0`) is not rotation-invariant. Measured: the thinnest usable
  normal sigma is 3.162 mm axis-aligned and 2.238 mm turned 45° in XY; a 3 mm plane is unusable axis-aligned (entry
  1.1e5) and usable turned (5.6e4). Pinned by `unit case K9 marker: the cell test gives the same usability for the plane
  axis-aligned and turned 45 degrees` (`tests/unit_cases_ndt_test.cpp`; oracle rotation invariance). Effect on a
  recording not measured. Order: step 7, before C16, each alone.
- **C31** (reported): the projection `M = diag(P, I)` removes the along-axis pull from the translation rows but keeps
  the rotation rows and their right-hand side, so a persistent along-axis pull can still turn the window in degenerate
  stretches (`ndt_term.cpp::ndt_degeneracy_projection`, `add_ndt_blocks`). Master-reviewer 2026-10-17; turning the
  projection off (V2) didn't move the gallery angle on 44.
- From the map's side, read here: the translation information uses each cell's stored normal; with the default
  statistics that normal comes from the cell's first three points (map region, C20 / C30; roadmap P1). A wrong normal
  changes the information ratio, so the projection and the verdict's ratio inherit it.

## To split in the first job

None (`python tools/function_sizes.py src --min 80`, 2026-11-18: no function of `ndt_term.*` is above 80 lines).

## Experiments

- **2026-11-20, consistency harness: the NDT term's claim (F4, K8) and the cell test** (`consistency-harness` 5.4, 5.6,
  5.8; 003103c, 54c04db, 907dc5b; tests `tests/lidar_claim_test.cpp`, `tests/unit_cases_ndt_test.cpp`; log
  `experiments/consistency/fixtures/lidar_claim.log`; `experiments/consistency/fixtures.md` section 4). The term alone
  (no motion model, no projection), information scale 1, 1 cm noise on every scan and map point, R = scatter / claimed
  sigma (1 honest, above 1 claims more than it has), 500 room, 150 corridor, 60 wall trials. Room, exact map: fine grid
  1.28 / 1.14 / 1.03 (x 1.28 [1.21, 1.37] excludes 1, not explained), coarse 1.00-1.03, both grids 1.40-1.54; map from
  one earlier window: fine 1.66-2.85, coarse 1.46-1.52, both 2.00-2.56 (audit's hypothesis 1.4 / 1.8). Duplication: the
  second grid adds 1.67-1.84× claimed information (audit 1.57-1.80) while the scatter stays (both / fine 0.93-1.00).
  Map noise: R(one window) / R(exact) 1.61-2.22 fine, 1.43-1.66 both (hypothesis 1.41). The shipped scale 0.1 divides R
  by 3.162 for a pure NDT solve (room 0.32-0.90); not exact in a real window. Corridor, constrained directions: exact
  map 0.90-1.03 one grid, 1.25-1.34 both; one-window map 1.39-1.96 / 1.86-2.05. Blind directions (corridor along, wall
  in plane): the term claims 0.09 / 0.2 mm, a 5 mm offset is pulled back 0.008-0.04 of it in the first step, 4.0-4.8 mm
  left after 5 steps, not converged: R = 4-12 is not a scatter ratio. In the fixtures' loop chains the projection cut
  the NDT weight along travel to at most 1.2e-4 of the sideways weight (the term practically off there, not shown
  honest); with the shipped cell statistics a plain corridor was 925 along against 155 across (design D8). Fixture
  artefact: a face 2.5 cm from a cell boundary gives the fine grid a 0.16 mm normal bias. K7 / C16 and K9 / F9: Known
  flaws. Not concluded: a universal inflation factor (it depends on the geometry); the noise model has uniform density,
  no ray geometry, no occlusion; nothing on a recording.
- **2026-11-18** Livox 4 passage stall (stage 8), NDT information split by each cell's normal (new per-term rows
  `ndt_normal` / `ndt_in_plane`, merged 7a9d031): in windows 288–292 the in-plane part has 67–84 % of the NDT's
  stretch-along weight and pulls −33 to −43 mm at the result, the normal part −12 to +3.5 mm; calm windows 270–280:
  normal 82 %, in-plane −5.7 mm. 94 % of the weight in 288–294 comes from points ahead of their node.
  `experiments/regions/lidar-matching/2026-11-18-passage-pull/`.
- **2026-11-18** In-plane ablation (experiment branch `…-in-plane-ablation`, 0feae07; rule A: normal + g × in-plane,
  rule B: only directions with σ ≥ 0.2 cell scaled), measured gyro noise. Passage 100–110 s (true about 10 m): 6.04 m
  → A 7.85 / 8.90 / 9.55 m at g 0.3 / 0.1 / 0, B 6.72 / 7.26 / 8.00 m. Full runs with A g 0: Livox 4 passage 9.38 m
  but the speed runs to 2.4–4.2 m/s at 329–354 s (about 50 m of false path); Livox 3 start-to-end 0.07 → 1.65 m. On
  the stairs the first flight's path grows 6.03 → 7.5–8.2 m (207–215 s); which is right is not known.
  `experiments/regions/lidar-matching/2026-11-18-in-plane-ablation/`, `experiments/stage8/`
- **2026-11-17** Per-term table, Livox 3 100-130 s (84 windows, replay): the NDT grids hold a median 90 % of the weight on
  stretch along travel (indoor 56 %, outdoor 33 %); turn about vertical: NDT indoor 56 % / outdoor 44 %, gyro 0.13 %. The
  projection cuts the NDT weight in 12 windows (to 14-15 % in windows 313-314, information ratio 0.12-0.13). The two grids'
  pulls for the travel distance differ by a median 1.6 mm, up to 25 mm (window 312). `docs/window-terms.html`.
- **2026-11-16** The Livox 3 map the user judged good came from a side build with Huber cutoff 3 and NDT information x0.1
  (and position prior x0.1); ported as `ndt_huber_cutoff`, `ndt_information_scale`, shipped in `livox-vertical.toml`.
  Memory `livox-state-2026-11-16.md`.
- **2026-11-13** Livox 2 window 754 (exact isolated replay): leaving out 3 coarse-grid cells changed the window's tilt
  15.19° -> 2.33° and end speed 0.89 -> 0.36 m/s; Huber 3 alone: 15.19° -> 2.15°, 0.37 m/s. Physical accuracy not
  measured. `openspec/changes/ndt-information-probe/full-livox2-w754-ablation.md`, `full-livox2-huber-results.md`.
- **2026-11-12** NDT information multipliers 1 / 0.3 / 0.1 / 0.03 / 0.01 (phase 0b, nine runs): on livox2 0-60 s the
  largest bend 23.7° (1) -> 6.4° (0.1) -> 23.0° (0.01); on Livox 1, 0.1 deformed the result (31 m from the control);
  44_short bend 2.1 -> 1.9°. `openspec/changes/ndt-information-probe/results.md`.
- **2026-11-10** Livox 1: NDT yaw stiffness 7e3-2e5 per deg² against the gyro edges' ~34 per deg² in a window; on livox/2
  along the weak axis NDT 400-2700x stiffer than the velocity prior and motion model. Memory `fast-turn-fragility-state.md`.
- **2026-10-14** C18, synthetic room: a perfect guess with walls on 0.4 m cell boundaries costs 1.52 mm, floor and
  ceiling on 0.15 m boundaries 0.61 mm, none 0.34 mm. Tracker C18.
- **2026-10-13** Heading drift on 44 (gallery vs start façade, odometry about -4.0°): cell sizes 0.10 / 0.27 m -> -3.1°;
  ignoring points more than 70° / 60° from the cell normal -> -3.4° / -3.8°; ignoring sparse frontier cells -> -5.9°.
  Memory `odometry-map-horizon.md`.
- **2026-10-13** Halved projection thresholds (0.05 / 0.1) with the default statistics: façade identical to the noise
  floor. Tracker C20.
- **2026-10-08** Degeneracy-aware NDT terms (projection 0.1 / 0.2) shipped in the L2 settings: the `/fp:fast` and
  `/fp:precise` builds of 44 differ by 0.31 m instead of 1.03 m; ghost wall gone (user's look); 587 s, no cost.
  `openspec/changes/archive/2026-10-08-odometry-stall-fix/`.
- **2026-10-08** Stored cell inverses: NDT loops 1.613 ms (indoor) and 0.810 ms (outdoor) per iteration on 44,
  bit-identical. `openspec/changes/archive/2026-10-08-ndt-lookup-cache/`.
- **2026-10-04** Synthetic 30 m corridor with the default cell statistics: information 925 along / 155 across (the
  opposite of the truth), 60 % of the normals more than 30° off. `docs/degeneracy-information.html` section 5.

## Ideas rejected

- **Point-to-plane NDT** (`ndt_point_to_plane`, 2026-10-11): 44 s0 route 455 -> 743 m, loops 39 -> 2, 6669 big jumps;
  the stairs slide without the in-plane pull. Removed with `ndt_plane_in_plane_sigma_m` (in-plane sigma 0.3 m, rejected
  twice before). `openspec/changes/archive/2026-10-11-point-to-plane-ndt/`.
- **Directional in-plane release** (2026-10-11): swapped the corridor lag between s0 and s1 (s0 -1.42 m, s1 +1.59 m),
  loops 39 -> 32. Removed. `openspec/changes/archive/2026-10-11-directional-in-plane-release/`.
- **Signal-to-noise degeneracy test on the Hessian** (2026-10-09): builds 7.45 m apart (0.31 m shipped), 15-20 % slower;
  the rotation-only variant 8.29 m from the baseline, 11 loops instead of 21. Removed.
  `openspec/changes/archive/2026-10-09-hessian-degeneracy-detection/`.
- **All-point normals for the projection** (`degenerate_projection_all_point_normals`, roadmap P1 fix (a)): on a 125 s
  Livox 4 prefix it ran away (62 m off by 100 s). Never on; removed 2026-11-16 (stage 0). `docs/agents/roadmap.md`.
- **A cost for unmatched points** (`ndt_unmatched_point_cost`): unit test showed 16 too small (500 m off the map 96,000
  against 337,000 for 8 cm off); never on; removed 2026-11-16. The cost still counts unmatched points.
  `docs/agents/roadmap.md` (M2).

## Open questions

- The in-plane part causes the passage stall and cannot be removed yet: without it the inertial carry must hold where
  the normal part says little along travel, and it runs away in one stretch (Experiments 2026-11-18). What biases the
  in-plane residual of points ahead of their node (sparse cells first seen from far are the candidate) is not measured.
- Does `sum n n^T` over the stored normals measure blindness well enough on real data (C20 / P1), and should an honest
  measure be rebuilt on the split code (roadmap step 3)?
- What do C16 and C31 do to a real degenerate stretch? Neither has been measured on its own.
- Why do the indoor and outdoor grids disagree by centimetres in some windows that are not degenerate (Livox 3 windows
  350-351)?
- Window 754 of Livox 2: are the three coarse cells' far returns wrong, or only influential? Not resolved.
- About -3.4° of 44's heading drift from the LiDAR / map side remains unexplained (2026-10-13).
- Why does information scale 0.1 help Livox 2 and Livox 3 but deform Livox 1?

## See also

- [Regions](README.md)
- [Degeneracy and the information matrix](../degeneracy-information.html)
- [The per-term table](../window-terms.html)
- [Code-review tracker](../code-review-tracker.md)
- [Code-split plan](../code-split-and-region-agents-plan.md) (stages 2 and 3)
- [All documents](../README.md)
