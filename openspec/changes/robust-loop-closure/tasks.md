# Tasks

## 1. Start from a clean tree

- [x] 1.1 Separate the working tree: candidate 2 (LM step check, unmatched-point cost) committed on branch `candidate-2-lm-step-check` (1c64cd6), the cross-floor prototype on master. Verify: `44_short` with the new settings off bit-identical with `tools/baseline/44_short.manifest.json` apart from the parameters TOML.
- [x] 1.2 `tools/spot_check.py`: per-pass offsets around one point, documented in `tools/README.md`. Verify: on `raw data/44_lc_s0` at (−3.0121, −5.4233, −0.1179), r 0.8, the 24.4 cm pass at 139–149 s.
- [x] 1.3 `tools/stairwell_check.py`: floors from the trajectory, the railing circle per floor band and pass, floor-to-floor offsets (stack and seen from below); documented in `tools/README.md`. Verify: on `44_lc_s0` with centre (−4.7, −4.9), floor −1 30 cm off floor 0, floor −2 31 cm off floor −1, fits with 1–3 cm residuals.

## 2. Robust kernel and the solve

- [x] 2.1 `loop_closure_kernel_scale`: Cauchy weights and cost on the loop edges in `optimize_loop_pose_graph`, two stages (design decision 6); the drop runs after the solve. Verify: scale 0 gives `44_short` bit-identical.
- [x] 2.2 The angle wrap in `add_relative_pose_edge` (design decision 7). Verify: the full 44 with the kernel off bit-identical with `tools/baseline/44.manifest.json` apart from the parameters TOML (23 loops, 1.72 m).
- [x] 2.3 Pose-graph unit tests in `tests/loop_closure_test.cpp` on two laps: a 15 cm wrong edge moves the solution clearly less with the kernel (0.5σ: 6.4 → 3.2 cm); consistent edges within 0.5 mm; a false edge still stands out. Verify: Release `ctest` passes.
- [ ] 2.4 The kernel on real data: X3 settings plus `loop_closure_kernel_scale` 0.5 and 1.0 on `44_s0/s1/s2`. Verify: a results table here. If the kernel improves the floor offsets or the largest correction without losing loops on any input, it is switched on in `settings/lidar-vertical.toml`; otherwise the setting and its code are removed (no unused code), the pose-graph tests trimmed, and the golden TOML files regenerated.

## 3. Cross-floor edges

- [x] 3.1 The second candidate pass (design decision 5), the acceptance test and the log lines with the summary count. Verify: the 44 log lists the cross-floor tries and accepted edges.
- [x] 3.2 The drop judges cross-floor edges by translation (`loop_edge_drop_ratio`, design decision 2). Verify: the unit test with a 3° turned cross-floor edge — not dropped for its rotation, dropped as a same-floor edge.
- [x] 3.3 The cross-floor time gap and the shared-footprint registration with its search range (design decisions 3, 4). Verify: X3 on `44_s0/s1/s2` (below).
- [ ] 3.4 Full 6-DoF edges (design decision 1): remove `loop_closure_cross_floor_rotation_sigma_deg` (the edge uses the loop-edge sigmas, identical to X3's 0.3°), set the defaults to the measured values (`_max_angle_deg` 10, `_min_fitness` 0.3, `_min_time_gap_s` 5, `_search_m` 1), update the unit test of 3.2 to a 6-DoF edge, regenerate the golden TOML files. Verify: Release `ctest` passes; X3's settings without the removed key give the same `44_s0` result as X3.
- [ ] 3.5 Unit test of the shared-footprint crop (exposed like `loop_edge_drop_ratio`): two clouds overlapping in one corner keep only that corner's points. Verify: Release `ctest` passes.

## 4. Measure

- [x] 4.1 Evaluation on the perturbed inputs (`stairwell_check`; floor −1 vs 0 / floor −2 vs −1, cm):

  | variant | `44_s0` | `44_s1` | `44_s2` |
  |---|---|---|---|
  | shipped | 29.6 / 31.0 | 46.7 / 37.1 | 18.2 / 69.1 |
  | V3′: 6-DoF, ≤ 3°, fitness 0.3 | 4.4 / 23.5 | 9.0 / 28.7 | 8.4 / 69.3 |
  | translation-only, ≤ 1.5° | 12.1 / 28.5 | 46.7 / 37.1 (no edge) | 11.2 / 70.0 |
  | X1: 6-DoF, ≤ 3°, + time gap + crop, fitness 0.5 | 33.4 / 2.8 | 47.8 / 6.0 | 14.8 / 69.3 |
  | X2: as X1, fitness 0.3 | 5.8 / 2.7 | 11.7 / 3.9 | 7.4 / 67.8 |
  | **X3: as X2, ≤ 10°** | **5.4 / 1.2** | **11.4 / 1.4** | **13.7 / 7.2** |

  X3: 37–42 loops (shipped 21–23), largest correction 1.88 / 1.96 / 2.39 m (shipped 1.72 / 1.86 / 1.92), 0–2 edges dropped, no jump warnings, floor 0 start against end 2.0–7.5 cm, worst pass at the railing spot 4.5–7.5 cm (shipped 14–33 cm). Visual check by the user on `44_x3_s0`: far better, not perfect.
- [ ] 4.2 Harm checks with X3: the `/fp:fast` build against the precise build on the unperturbed 44 (`trajectory_seams.py`, `stairwell_check.py`); `45f_p` against `45f_lc` (loops, largest correction, `trajectory_seams.py`); where the 2.39 m largest correction on `44_s2` comes from. Verify: results here; no runaway in the fast build, 45f no worse than shipped.
  - **Run 2026-10-10:**
    - **45f:** a single floor, so no cross-floor candidate. The result equals shipped (21 loops, 0.20 m, the trajectory identical).
    - **Unperturbed 44:** X3 equals `44_x3_s0` (5.4 / 1.2 cm, 42 loops, 1.88 m).
    - **`/fp:fast` build:** its odometry diverges by metres from about 35 s (0.6 m at 35 s, 4.5 m at 100 s, up to 20 m). That is the known fragility of the fast build (synchronized-retune 4.8; the shipped fast build had 11 loops), not caused by X3. With X3 it closes 26 loops. The builds are not within decimetres, but X3 doesn't change that.
    - **The 2.39 m on `44_s2`:** the same-floor loops are identical to shipped. The solved graph moves floor 0's far stretch (40–90 s, y ≈ +15) 0.4–0.9 m further in +x than shipped, and floors −1 and −2 0.3–0.8 m. Which is right there is open: no reference covers that stretch.
  - **Still open:** a check of the far stretch (a revisit, or the building plan).
  - **Possible problem:** `trajectory_seams.py` seems to compare the trajectories without the chunk poses (0.49 m where the world comparison shows 0.94 m). To verify before using it for loop-closure variants.

## 5. Ship

- [ ] 5.1 `settings/lidar-vertical.toml` with cross-floor edges on (6 m) and the kernel decision of 2.4; docs (EN/HU) on loop closure: cross-floor edges, the time cost, the remaining heading drift on stairs. Verify: the docs mention every new setting.
- [ ] 5.2 New 44 and 44_short baselines in their own commit (old `raw data/44_baseline`, `44_short_baseline` renamed with a date suffix, manifests rewritten). Verify: `compare_baseline.py` IDENTICAL against the new manifests.
- [ ] 5.3 Release `ctest` passes. Debug build + `ctest` only if the change touches threading or memory (it doesn't). Verify: test output.
