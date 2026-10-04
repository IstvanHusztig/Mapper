# Tasks

**Checks on every code commit, with the switch off** (design decision 5):
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- the precise build's `44_short` is IDENTICAL with a precise reference made from master before the first code commit;
- the fast build's `44_short` is IDENTICAL with `tools/baseline/44_short.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_short_baseline` (report the largest difference).

Close the GUI before building.

## 1. Switch and legacy path

- [x] 1.1 Make the precise `44_short` reference from master (`precise_check.sh --reference`). Add `bool use_exact_ndt_bucket_statistics = false;` to `OdometryConfig`, `TomlIO::POINTERS` and the `lidar_odometry_control` category. Set `false` in `app/default_settings.toml` and `settings/lidar-vertical.toml`. Regenerate the golden TOML files in their own commit, and update the manifests' params hash, as `max-prior-velocity` did. Verify: the step checks. Commit. Result: the precise reference (11 files) was made from master `e33a264`. Both presets build and `ctest` passes. Precise and fast `44_short` differed only in the params file; its hash `d61da98f` → `74020d53` is updated in the 44, 44_short and precise manifests, and both runs are then IDENTICAL (11 files). The baseline folders still hold the old params file, so a `--tolerance` comparison against them flags that one file.
- [x] 1.2 Move today's body of `update_rgd` unchanged into `update_rgd_legacy`. `update_rgd` gets a `bool exact` parameter and calls the legacy function when it is `false`. Pass `params.use_exact_ndt_bucket_statistics` from every caller: `seed_map`, `add_window_to_map`, `run_loop_closure` and the consistency pass. Verify: the step checks. Commit. Result: `update_rgd` takes `bool exact` (no default; the viewport default is gone too, every caller passed one). Loop closure passes it into `ndt_refine`. Both presets build, `ctest` passes, and precise and fast `44_short` are IDENTICAL (11 files).

## 2. Exact statistics

- [ ] 2.1 Implement the exact path (decisions 1–3): the running mean and covariance with the one-point prior, the view-point gate against the current normal, and the normals of touched cells recomputed at the end of the call. Add `tests/ndt_map_test.cpp` (decision 6):
  - planes in line order and in random order: the mean within 1 mm of the centroid, the normal within 5°, and both orders agree within 1e-9;
  - the legacy path's quirks are pinned;
  - a thin wall seen from two viewports in two calls.

  Add the file to `tests/CMakeLists.txt`. Verify: the step checks with the switch off (the default), and `ctest` passes. Commit.
- [ ] 2.2 Switch `tests/scan_to_map_test.cpp` to exact statistics. Remove `should_fail` from the corridor test, and tighten the room limits to the measured values with margin. Report the old and new numbers:
  - perfect-guess error;
  - drift;
  - bias;
  - iterations;
  - corridor information.

  If the corridor test still fails, pause and report. Verify: `ctest` in both presets. Commit.

## 3. Evaluation (the user decides)

- [ ] 3.1 Run `44_short` and `44` with the switch on, in the background, from a scratchpad copy of `lidar-vertical.toml`. The switch-off result is the current baseline. Collect, for off against on (decision 7):
  - `eval_odometry.py` metrics, after the CloudCompare export;
  - degenerate-window and accelerometer-term counts, and the weakest directions;
  - loop-closure edges;
  - odometry time.

  Write them as a table in this task, and **pause for the user's decision**.
- [ ] 3.2 If accepted:
  - switch the default to `true` in `OdometryConfig`, `app/default_settings.toml` and `settings/lidar-vertical.toml`, and regenerate the golden TOML files;
  - in a commit of their own: new baselines for `44_short` and `44`, with the old folders kept as `*_2026-10-04`, and the refreshed precise reference;
  - note the refresh in `tools/README.md`.

  Verify: both manifests check IDENTICAL against their new folders. Commit. If rejected, record the decision here and leave the default `false`.

## 4. Docs

- [ ] 4.1 Update the docs:
  - `docs/degeneracy-information.html`: section 5 says fixed (or not), with the new corridor numbers and the 44 degeneracy counts;
  - `docs/index.html`: the Known-limitations entry, and the new setting's row in `[lidar_odometry_control]`;
  - `docs/PERFORMANCE.md`: the run time;
  - `CLAUDE.md`: `update_rgd`, the switch and the new test file.

  Verify: `openspec validate ndt-bucket-statistics --strict` passes. Commit.
