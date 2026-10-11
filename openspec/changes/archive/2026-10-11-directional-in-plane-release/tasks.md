# Tasks

## 1. Implementation (off by default)

- [x] 1.1 The settings `ndt_in_plane_release_ratio` and `ndt_in_plane_release_ratio_full` (`OdometryConfig`, `TomlIO::POINTERS` + `[lidar_odometry_control]`, golden TOML files regenerated in their own commit). The window loop computes the previous window's translation-information ratio and weakest direction whenever the release is on (not only with the accelerometer gate), and hands them to the window's `ScanToMapOptions` copy (direction w, strength s, design decision 2). Verify: off → `44_short` bit-identical with `tools/baseline/44_short.manifest.json` apart from the parameters TOML.
- [x] 1.2 The cell's planarity and normal stored next to its inverse in `refresh_cell_inverse` (decision 3, only with the release on), and the released information M' = Q M Q in both NDT point lambdas (decision 1). Verify: Release build; with the setting off the lambdas take the unchanged path (bit-identical, 1.1's check).
- [x] 1.3 Unit tests in `tests/scan_to_map_test.cpp`, matching the spec scenarios:
  - corridor means moved along the axis: unchanged with full release along the axis, changed without;
  - means moved across (vertically): changed as without the release;
  - room yaw drift as accurate as without;
  - multithreaded = single-threaded and stored = on-the-spot, bit for bit, with the release on.

  Verify: Release `ctest` passes.

## 2. Go/no-go (design decision 5)

- [x] 2.1 Full runs with the shipped settings + `ndt_in_plane_release_ratio = 0.4`, `ndt_in_plane_release_ratio_full = 0.25`, `log_window_diagnostics = true`: `44_s0`, `44_s1`, `44_s2`, `45f_p` (two at a time, one background job per run), and a `/fp:fast` build of the same code on `44_s0`. References: `44_diag_s0n`, `44_diag_s1n` (shipped with diagnostics), `44_i1_s2` (shipped, from the snapshot), `45f_lc`, and the fast build with the shipped settings. Verify: all runs finished (log ends with `exit 0`), route, loops, warnings collected.
- [x] 2.2 The criteria table: the seven criteria of decision 5 one by one, with the measured values and the verdict. Verify: the table is in this task, and the verdict is pass or fail.
- [x] 2.3 **On fail:** remove the settings and the code (one commit, golden files regenerated), record the numbers in floor-alignment-diagnosis findings 2.2, archive as rejected; group 3 is dropped. Verify: `44_short` bit-identical, no trace in `src/`, `tests/`, `docs/`, `settings/`.


  **Result (2026-10-11): FAIL.** `44_s0` and `44_s1` full odometry, shipped settings plus release 0.4 / 0.25 and diagnostics. `44_s2`, `45f_p` and the fast build were not run once criteria 2 and 4 had failed.

  | criterion | `44_s0` | `44_s1` | verdict |
  |---|---|---|---|
  | 1. no runaway | 0 jumps, 0 over-speed, route 453.2 m (shipped 455.2) | 0 / 0, 457.4 m | pass |
  | 2. loops within 10 % | **32** (shipped 39) | 36 (36) | **fail** |
  | 3. stairs, windows 205–225 within 5 cm per window | max 4.5 cm | – | pass |
  | 4. corridor, s1 − s0 along x at 72.5 s ≤ 0.3 m | **−1.36 m** (shipped +1.65) | | **fail** |
  | 5. s1 gallery façade within 20 cm of s0 | s0 not measurable | −16.6 cm (shipped −56.0) | fail / unclear |
  | 6. floors | stairwell floor −1/0 13.9 cm (6.6) | 7.6 (11.9) | mixed |

  - **The release doesn't remove the corridor ambiguity, it swaps it:** s0 now lags by 1.42 m against its shipped run, and s1 is 1.59 m ahead of its shipped run.
  - **Cause found with the weak-axis diagnostics** (kept, behind `log_window_diagnostics`):
    - the corridor on the way out (68–73 s) is the one walked on the way in (40–48 s);
    - with `sliding_window_trajectory_length_threshold = 200 m` the odometry registers against both copies;
    - in s0, window 176's NDT cost is 47 % lower 0.6 m along the axis, the correction loop closure later applies.
    - Confirmed with map horizons of 20 and 50 m: the corridor jump disappears (s1 − s0 changes by ≤ 0.2 m). But the periodic map halving breaks the odometry elsewhere: 14–27 loops, 16–31 m error at 133 s on floor −1. So it isn't a fix as a setting.
  - The code was removed (2.3). `44_short` is bit-identical apart from the parameters TOML. The numbers are in floor-alignment-diagnosis findings 2.2.

## 3. Ship (only after a pass) — dropped: go/no-go failed

- [ ] 3.1 `settings/lidar-vertical.toml` with the two ratios. Docs: EN/HU parameter tables, `docs/degeneracy-information.html` (the in-plane pull and its release), `docs/performance.html` (time cost). Verify: the pages describe the settings.
- [ ] 3.2 New 44 and 44_short baselines in their own commit (old folders renamed with a date suffix, manifests rewritten). Verify: `compare_baseline.py` IDENTICAL against the new manifests.
- [ ] 3.3 Release `ctest`, plus a Debug build and `ctest` (the `NdtBucket` struct grows). Verify: both pass.
- [ ] 3.4 Findings 2.2 updated with the shipped result, and memory updated. Verify: the gallery façade of `44_s1` in the findings table.
