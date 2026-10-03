# Tasks

**Step checks on every code commit** (design decision 4):
- both presets build `dMapper` and `lidar_odometry_tests`, and `ctest` passes;
- the normal Release build runs `44_short` within `--tolerance 1e-6` of `raw data/44_short_baseline`;
- the precise build (`HD_CPU_OPTIMIZATION=GENERIC`, in the scratchpad) runs `44_short` within `--tolerance 1e-6` of `raw data/44_short_precise_baseline`;
- report the largest difference for both.

Close the GUI before building.

## 1. Helper and tests

- [x] 1.1 Add `place_windows_after(std::vector<Window> &windows, size_t first)` to `app/window_steps.{h,cpp}`. It places `windows[first..]` each after the end of its predecessor, and does nothing when `first == 0` or `first >= windows.size()` (decision 3). Add tests to `tests/window_steps_test.cpp`:
  - after the call, each window from `first` on starts at its predecessor's last pose;
  - each window keeps its relative poses within 1e-12;
  - windows before `first` don't change;
  - one call gives the same poses, within 1e-9, as the old loop run once per earlier window (the repeated placement);
  - `first = 0` and `first = size` leave the windows as they were.

  Verify: both presets build and `ctest` passes. Commit.

## 2. Place only the next window

- [ ] 2.1 In `optimize_windows` (`app/run_odometry.cpp`), replace the loop over all later windows with `place_window_after(windows[i + 1], <end of window i>)` when `i + 1 < windows.size()` (decision 1). Update the comment above it to say why only the next window is placed. Verify: run the step checks and report both largest differences. Commit.
- [ ] 2.2 On the failure path, call `place_windows_after(windows, i + 1)` before `return false`, after window `i` has its old poses back (decision 2). `add_window_distance` restores the poses, and the caller calls the helper when it returns `false`. Verify: run the step checks; the results don't change from 2.1, because the path doesn't run on `44_short`. Commit.

## 3. Full run, timing, baseline

- [ ] 3.1 Run the full 44 in the background with the normal build. Verify it is within `--tolerance 1e-6` of `raw data/44_baseline`, and report the largest difference per file type. If it goes over 1e-6, pause: compare with `tools/eval_odometry.py` and ask before going on.
- [ ] 3.2 Measure the run time. Compare the odometry's `elapsed time` line between master (`master_bin`) and this change, on `44_short` and from the full 44 log, and report the numbers as they are (decision 6).
- [ ] 3.3 New baseline in a commit of its own (decision 5):
  - rename the old `raw data/44_short_baseline` and `raw data/44_baseline` to `*_2026-10-03b`;
  - make the 44_short and 44 outputs of this change the new baseline folders;
  - rewrite `tools/baseline/44_short.manifest.json` and `tools/baseline/44.manifest.json` with `--write-manifest` (`--params settings/lidar-vertical.toml`);
  - refresh the precise `44_short` reference in the scratchpad.

  Verify: `compare_baseline.py` of each new folder against its new manifest reports IDENTICAL. Commit the manifests.

## 4. Docs

- [ ] 4.1 Update the docs:
  - `docs/ARCHITECTURE.md`: replace the re-chaining note under 6b with "fixed in `chain-next-window-only`";
  - `tools/README.md`: the baseline refresh, with the date, the reason and the old folder names;
  - `docs/PERFORMANCE.md`: add this fix with its measured time;
  - `CLAUDE.md`: only if something it says is no longer true.

  Verify: `grep` finds no "re-chains all later windows" left in `docs/` or `CLAUDE.md`. Commit.
