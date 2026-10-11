# Tasks

## 1. Information-weighted edges in the pose graph (settings off: nothing changes)

- [x] 1.1 Add the settings `loop_closure_cross_floor_information_edges` and `loop_closure_information_edges` (both false) to `OdometryConfig` with comments, to `TomlIO::POINTERS` and to the `[loop_closure]` category. Regenerate the golden TOML files in their own commit (`LO_UPDATE_GOLDEN=1`). Verify: the TOML round-trip tests pass.
- [x] 1.2 `ndt_refine` also returns the undamped normal matrix of its final 0.25 m iteration (D1). `LoopClosureEdge` gets `bool information` and `Eigen::Matrix<double, 6, 6> omega`. Verify: build; with the settings off the edge list and the log are unchanged.
- [x] 1.3 The information transform as a file-local function, unit-tested through `optimize_loop_pose_graph`:
  - G by central differences and Ω_m = G⁻ᵀ H G⁻¹ (D2);
  - the vertical marginalization for cross-floor edges (D3);
  - the scaling and the clipping (D4);
  - the fixed-sigma fallback for an ill-conditioned G, with a log line.

  Verify (`tests/loop_closure_test.cpp`): for an H built from a synthetic box room, the transformed Ω reproduces, in the edge's residual, the registration's weighted squared error of a small test perturbation within 1e-6 relative.
- [x] 1.4 `add_information_edge` and its cost (D5); the kernel's `s2`, `loop_edge_residuals` and `loop_edge_drop_ratio` for information edges (D6). Verify with new doctest cases:
  - (a) with Ω = diag(w_t I, w_r I) the solution equals the fixed-sigma edge's within 1e-9;
  - (b) a rank-deficient Ω (no height, no yaw) corrects the horizontal offset and the tilt, but leaves the height and the yaw as the odometry has them (spec scenario "A shaft leaves its axis free");
  - (c) an edge off only in a free direction is not dropped, one off by 0.3 m in a tied direction is.
- [x] 1.5 Wire it in `run_loop_closure`. With `loop_closure_cross_floor_information_edges` an accepted cross-floor edge gets its Ω, and keep-height, point edges and the cross-floor rotation sigma are ignored for it (logged once). The same for same-floor edges with `loop_closure_information_edges`, without D3. Each information edge logs its principal sigmas in the shared region's frame (D6). Verify:
  - build Release;
  - run `ctest`, plus the loop-closure cases with `--no-skip`;
  - `44_short` bit-identical against `tools/baseline/44_short.manifest.json` with the settings off.

  Debug is not needed: no threading, lifetime or assert changes.
- [x] 1.6 Docs only once the design proves right (user, 2026-10-11): the two settings and the edge model go into `docs/index.html` and `docs/hu/index.html` with task 5.2 (or with 5.1 if the setting stays off but is kept). Verify: done in 5.1/5.2; this task is closed with them.

## 2. Evidence first: what the information says on 44 (before judging the floors)

- [x] 2.1 Make the floor-shape checks permanent: move the job-folder scripts `slabs.py`, `lift_walls2.py` and `icp_floors_t.py` into `tools/floor_shape_check.py`, with subcommands `slabs`, `lift` and `floors-icp`, the levels, boxes and pass times as arguments, and a `tools/README.md` entry. Verify: on `raw data/44_x6_s2` it reproduces the numbers in the findings (floor −1 slab tilt +1.03°, lift bottom wall −12.9 cm on the lowest floor, floor ICP within 0.7°).
- [x] 2.2 Reprocess `44_s0` with I1 from `raw data/44_s0_snap`, and collect the logged principal sigmas of every cross-floor edge, grouped by shared-region centre (round shaft / straight staircase / lift shaft / other). Verify against the expectation of D4/D6:
  - shaft edges: z free, yaw ≥ 3° or free;
  - roll and pitch ≤ about 1.5°.

  If a shaft edge ties the yaw harder than about 3°, stop and report (Risks). Also compare the 0.5 m and 0.25 m information on two shaft edges (Open Question) and keep the sharper one.

## 3. Evaluation of I1 (cross-floor information edges)

- [x] 3.1 Reprocess `44_s1` and `44_s2` with I1 from their snapshots (at most two runs at once, never two on one input folder). On all three inputs run `tools/evaluate_floors.py` and `tools/floor_shape_check.py` (slabs, lift, floors-icp). Verify: one results table against X6 and shipped, covering every D8 criterion, saved as `eval_i1.txt` in the job folder.
- [x] 3.2 `45f`: run I1 from `raw data/45f_snap` and `tools/trajectory_seams.py raw\ data/45f_lc raw\ data/45f_i1`. Verify: nothing above 5 cm (it has no cross-floor pair, so it should be identical).
- [x] 3.3 Record the result in `floor-alignment-diagnosis/findings.md` as a new section: the table, which criteria were met and which missed (as measured), and the per-edge sigma summary from 2.2. Verify: the section exists, with the D8 criteria listed one by one.

## 4. Evaluation of I2 (same-floor edges too)

- [x] 4.1 Reprocess the three 44 inputs and `45f` with I2, with the same checks as 3.1–3.2, plus `trajectory_seams.py` of `45f_i2` against shipped. Verify: results table `eval_i2.txt`; the I2 rule of D8 applied and the outcome written into the findings.
- [x] 4.2 Report I1/I2 against the criteria to the user and ask which to adopt (none, I1, or I2) and whether to switch it on in `settings/lidar-vertical.toml`. Verify: the user's decision is recorded in the findings.

## 5. Adoption and clean-up (only after the user's decision in 4.2)

- [x] 5.1 If I1 or I2 is adopted, remove the mechanisms the information edges supersede, with their code, settings, tests, docs and golden-file entries:
  - `loop_closure_cross_floor_point_edges` (and `LoopClosureEdge::point`, `point_i/j`, `add_point_edge`, `point_edge_residual`);
  - `loop_closure_cross_floor_keep_height`;
  - `loop_closure_cross_floor_rotation_sigma_deg`, if nothing uses it any more.

  Make the cross-floor information edge the only cross-floor edge model. If neither variant is adopted, remove this change's code instead. Verify:
  - a grep for the removed names finds nothing in `src/`, `tests/`, `docs/` and `settings/`;
  - the build and `ctest` pass;
  - the golden TOML files are regenerated in their own commit.
- [x] 5.2 If it is switched on in `settings/lidar-vertical.toml`: run full 44 (about 20 min) and `44_short`; set new baselines in their own commit (manifests, `raw data/44_baseline` and `44_short_baseline`, old ones renamed with a date suffix); update `CLAUDE.md` and the docs (stairwell limitation, the cross-floor parameter description). Verify: `tools/compare_baseline.py` is bit-identical against the new manifests.
- [x] 5.3 Update `robust-loop-closure`'s spec delta and design to the edge model that shipped (point edges and keep-height out, if removed). Verify: `openspec validate robust-loop-closure` and `openspec validate information-weighted-loop-edges` pass.
