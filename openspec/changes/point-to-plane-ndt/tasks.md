# Tasks

## 1. Stage 1: prototype and go/no-go

- [ ] 1.1 `ndt_point_to_plane` (`OdometryConfig`, `ScanToMapOptions`, `TomlIO::POINTERS` + `[lidar_odometry_control]`). `plane_information` next to the stored inverse in `refresh_cell_inverse` (design decisions 1–4). Used in both NDT point lambdas and in `ScanToMapCost` when on. Verify: off → `44_short` bit-identical with `tools/baseline/44_short.manifest.json` apart from the parameters TOML.
- [ ] 1.2 Unit test in `tests/scan_to_map_test.cpp`: a planar cell's mean moved along its plane doesn't change the result with the setting on, and the room keeps its accuracy; `tests/scan_to_map_test.cpp`'s parallel-vs-sequential case with the setting on. Verify: Release `ctest` passes.
- [ ] 1.3 Go/no-go runs (design decision 5): `44_s0/s1/s2`, `45f_p`, the `/fp:fast` build on 44, each with the shipped odometry and with the LM step check (cost 16). Verify: results table in this task against `44_lc_s*` / `45f_lc` and `44_lm16_s*` / `45f_lm16`, with the pass/fail verdict.
- [ ] 1.4 **On fail:** remove the setting and its code together with `ndt_plane_in_plane_sigma_m` (one commit, golden TOML files regenerated). Record the numbers in synchronized-retune 4.1 / 4.5 / 4.8, and archive this change as rejected; tasks 2–3 are then dropped. Verify: `44_short` bit-identical, the docs no longer mention the in-plane sigma.

## 2. Stage 2 (only after a pass)

- [ ] 2.1 Re-check the degeneracy projection thresholds (0.1 / 0.2 against 0.05 / 0.1 and off) with point-to-plane on. Verify: results table; the chosen thresholds keep the original ghost-wall passage (125 s) free of a stall.
- [ ] 2.2 Remove `ndt_plane_in_plane_sigma_m` (superseded). Verify: `44_short` unchanged by the removal with point-to-plane settings fixed.

## 3. Ship (only after a pass)

- [ ] 3.1 `settings/lidar-vertical.toml` with `ndt_point_to_plane = true` and the chosen projection; docs (EN/HU) on the NDT term, `docs/degeneracy-information.html` where it describes the in-plane pull; `docs/PERFORMANCE.md` with the time. Verify: docs mention the setting and the removed option.
- [ ] 3.2 New 44 and 44_short baselines in their own commit (old folders renamed with a date suffix, manifests rewritten). Verify: `compare_baseline.py` IDENTICAL against the new manifests.
- [ ] 3.3 Release `ctest`. Debug build + `ctest` only if the change touches threading or memory layout (the stored cell information does: the `NdtBucket` struct grows). Verify: both pass.
