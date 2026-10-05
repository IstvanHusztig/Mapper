# Tasks

**Checks** (design decisions 6 and 7): both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes, on every commit. Until the baselines are replaced (task 3.1), `44_short` is compared with the old manifests and must be IDENTICAL in every file except the two version-stamped ones (normal and precise build).

Close the GUI before building.

## 1. One version

- [x] 1.1 `src/domain/dmapper_version.h` (decision 1); `DMAPPER_VERSION_*` = 0.1.0 in `CMakeLists.txt` in place of `HDMAPPING_VERSION_*`; `shared/` and `include_directories(shared/include)` removed; `get_software_version()` uses the new header; `folder_result_sink`, `toml_io` and the GUI as in decisions 2 and 3 (the unused `Version.hpp` includes in `lidar_odometry_gui.cpp` and `viewer.cpp` go); `settings/*.toml` `[version_info]` say 0.1.0 (decision 4); `tests/toml_config_repository_test.cpp` expects the build's version. Verify: both presets build; `ctest` passes except the golden TOML tests (they change in 1.2); `grep -rn "HDMAPPING_VERSION\|Version.hpp\|HDMapping_" src CMakeLists.txt` finds nothing; the GUI title is "dMapper v 0.1" (`MainWindowTitle`). Commit.
- [x] 1.2 Regenerate the golden TOML files (`LO_UPDATE_GOLDEN=1`, decision 4) and check their diff is only the version lines. Verify: `ctest` passes in both presets. Commit.

## 2. Tools

- [x] 2.1 `compare_baseline.py` and `eval_odometry.py` match the `dMapper_*` names (decision 5); `tools/README.md` follows. Verify: run `44_short` with the normal and the precise build; against the current manifests every file except the two version-stamped ones is IDENTICAL; the two files, diffed against `raw data/44_short_baseline`'s with versions and date-times masked, show no other difference (record the diff). Commit.

  - **Recorded (2026-10-05):** the new build's `44_short` (fast 204 s, precise 205 s) differs from the old manifests in the two renamed files and `session.mjs`, which also carries the version (`"lidar_odometry_version": "v0.1.0"`). Mapped back to HDMapping 0.95.0 (file names plus exactly five lines: the TOML's two header comments and `software_version`, the JSON's `software_version`, `session.mjs`'s `lidar_odometry_version`), both folders are IDENTICAL with the old manifests (11 files each) under the old `compare_baseline.py`. (A direct diff against `raw data/44_short_baseline`'s params TOML also shows `use_exact_ndt_bucket_statistics` / `log_iteration_convergence`: that folder predates those settings, the manifest doesn't.)

## 3. New baselines

- [ ] 3.1 The new baselines (decision 6): the full 44 run (background or subagent), first checked like 2.1 (every data file IDENTICAL with `tools/baseline/44.manifest.json`, the two text files differing only in version and date-time); then new `44.manifest.json` and `44_short.manifest.json`, a new precise manifest from this build, and new `raw data/44_baseline` / `44_short_baseline` folders (old ones renamed `*_2026-10-05`). Verify: `compare_baseline.py` reports IDENTICAL for the new runs against the new manifests. Commit (manifests only).

## 4. Docs

- [ ] 4.1 `CLAUDE.md` (Build: the version in `CMakeLists.txt` and `dmapper_version.h`; Testing and elsewhere: the `dMapper_*` file names) and every other `HDMapping_params`/`HDMapping_results`/`Version.hpp` mention in `docs/` and `tools/README.md`. Verify: `grep -rn "HDMapping_params\|HDMapping_results\|Version.hpp\|HDMAPPING_VERSION" CLAUDE.md docs tools src` lists only history; `openspec validate dmapper-version --strict` passes. Commit.
