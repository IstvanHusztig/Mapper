# Tasks

## 1. Stage 1: prototype and go/no-go

- [x] 1.1 `ndt_point_to_plane` (`OdometryConfig`, `ScanToMapOptions`, `TomlIO::POINTERS` + `[lidar_odometry_control]`). `plane_information` next to the stored inverse in `refresh_cell_inverse` (design decisions 1–4), computed only with the setting on. Used in both NDT point lambdas of `optimize_lidar_odometry` when on (consistency keeps point-to-distribution). Verify: off → `44_short` bit-identical with `tools/baseline/44_short.manifest.json` apart from the parameters TOML.
- [x] 1.2 Unit test in `tests/scan_to_map_test.cpp`: a planar cell's mean moved along its plane doesn't change the result with the setting on, and the room keeps its accuracy; `tests/scan_to_map_test.cpp`'s parallel-vs-sequential case with the setting on. Verify: Release `ctest` passes.
- [x] 1.3 Go/no-go runs (design decision 5), full odometry with the shipped settings (`settings/lidar-vertical.toml`, cross-floor loops on) plus `ndt_point_to_plane`, `log_window_diagnostics` on: `44_s0/s1/s2`, `45f_p`, and the `/fp:fast` build on 44. Verify: results table in this task against the shipped references (`44_diag_s0n`, `44_diag_s1n`, `44_i1_s2`, `45f_lc`; the fast build against its own shipped-settings run), with the pass/fail verdict. Updated 2026-10-11: the LM step check (candidate 2) is not on master, so it is no longer a variant here; the s1 gallery stall (floor-alignment-diagnosis findings 2.2) is added as a criterion.
- [x] 1.4 **On fail:** remove the setting and its code together with `ndt_plane_in_plane_sigma_m` (one commit, golden TOML files regenerated). Record the numbers in synchronized-retune 4.1 / 4.5 / 4.8, and archive this change as rejected; tasks 2–3 are then dropped. Verify: `44_short` bit-identical, the docs no longer mention the in-plane sigma.


  **Result (2026-10-11): FAIL.** Full odometry, shipped settings plus `ndt_point_to_plane` and the window diagnostics.

  | | `44_s0` shipped → point-to-plane | `44_s1` shipped → point-to-plane |
  |---|---|---|
  | route | 455.2 m → **742.8 m** | → **728.8 m** |
  | loops accepted | 39 → **2** | 36 → **3** |
  | cross-floor registrations accepted | 19 → 0 of 3 | → 0 of 3 |
  | `big jump` warnings | 0 → **6669** | 0 → **1224** |
  | over-speed windows (> 5 m/s) | 0 → 102 | 0 → 98 |
  | largest difference to shipped | **242 m** (first > 5 cm at 10.9 s, > 50 cm at 33.8 s) | |

  - **Where it fails:** the stairwell descent, window 216 (t ≈ 87 s), with a speed of up to 6.3 m/s; the same window where the `/fp:fast` build runs away (synchronized-retune 4.8). Without the in-plane pull the windows on the stairs slide.
  - **What it did right before that:** the `44_s1` gallery stall at 68.5–72.5 s fell from 1.69 m to 0.48 m against s0 (still over the 0.3 m criterion).
  - **Unit tests:** in the synthetic room and corridor point-to-plane was as accurate as point-to-distribution, and the result didn't move when planar cell means moved 2 cm along their planes (10⁻¹³ mm).
  - **Not run:** `44_s2` and `45f_p` (stopped once s0/s1 had failed; s2 already reported 5.06 m/s at window 310) and the `/fp:fast` build.

  The code was removed, together with `ndt_plane_in_plane_sigma_m` (1.4). `44_short` is bit-identical apart from the parameters TOML. The result is recorded in synchronized-retune 4.5 / 4.8 and in floor-alignment-diagnosis 2.2.

## 2. Stage 2 (only after a pass) — dropped: stage 1 failed

- [ ] 2.1 Re-check the degeneracy projection thresholds (0.1 / 0.2 against 0.05 / 0.1 and off) with point-to-plane on. Verify: results table; the chosen thresholds keep the original ghost-wall passage (125 s) free of a stall.
- [ ] 2.2 Remove `ndt_plane_in_plane_sigma_m` (superseded). Verify: `44_short` unchanged by the removal with point-to-plane settings fixed.

## 3. Ship (only after a pass) — dropped: stage 1 failed

- [ ] 3.1 `settings/lidar-vertical.toml` with `ndt_point_to_plane = true` and the chosen projection; docs (EN/HU) on the NDT term, `docs/degeneracy-information.html` where it describes the in-plane pull; `docs/PERFORMANCE.md` with the time. Verify: docs mention the setting and the removed option.
- [ ] 3.2 New 44 and 44_short baselines in their own commit (old folders renamed with a date suffix, manifests rewritten). Verify: `compare_baseline.py` IDENTICAL against the new manifests.
- [ ] 3.3 Release `ctest`. Debug build + `ctest` only if the change touches threading or memory layout (the stored cell information does: the `NdtBucket` struct grows). Verify: both pass.
