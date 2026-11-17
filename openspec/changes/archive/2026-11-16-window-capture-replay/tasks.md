# Tasks

**Step checks, on every code commit:**
- the Release build of `dMapper` and `lidar_odometry_tests`;
- `ctest` passes;
- a Release CLI run on `44_short` matches `tools/baseline/44_short.manifest.json`; only the params TOML may differ, by
  the new keys.

## 1. The loop runs any range

- [x] 1.1 Move `OdometryRunSetup`, `OdometryTiming`, `WindowLoopCarry` and `AccTerm` to `src/application/window_loop.h`
  (pure move).
- [x] 1.2 Split `optimize_windows` into `start_window_loop`, `process_windows(first, last)` and `finish_run`, with the
  hooks for forced iterations and outcomes. `prepare_run` and `start_carry` read only the first window. Run the step
  checks. Result: 46c56b2; 44_short identical.

## 2. Capture and replay

- [x] 2.1 Add `capture_windows_from_s` / `capture_windows_to_s` and the port `WindowCaptureSink` with `capture_range`.
  The loop runs in three parts around the range and keeps the outcomes.
- [x] 2.2 Add `src/infrastructure/binary_io.h` (the snapshot's writer and reader) and `window_capture_file.{h,cpp}`,
  with every cell field, the carry and the windows including `imu_motion_from_previous`.
- [x] 2.3 Add `replay_window_capture` and `dMapper --replay <capture> <params.toml>`. Move `MemoryPointStore` to
  `src/application`.
- [x] 2.4 Add `tests/window_capture_test.cpp`:
  - the capture leaves the run unchanged;
  - the file gives the capture back;
  - a replay with the same settings repeats the run bit for bit;
  - a replay with other settings runs and reports the difference;
  - the range is the windows that start within it.
- [x] 2.5 Capture 44_short 20–40 s and replay it. Result: 6a75686; the run matches the manifest except the params TOML
  and the capture file; the replay gives 21 of 21 windows identical in 35 s.
- [x] 2.6 Regenerate the golden TOMLs. Result: 41fb121.
- [x] 2.7 Document the capture and replay in `CLAUDE.md` and the plan's progress. Result: a16edde.

## 3. End of stage

- [x] 3.1 Full 44 against its manifest. Result (41fb121): identical except the params TOML.
- [x] 3.2 Livox 3 with a capture of 100–130 s, against its manifest, then replayed. Result: identical except the
  params TOML and the capture file (587 MB); the replay gives windows 279–362, 84 of 84 identical, in 93 s.
- [x] 3.3 Debug build and `ctest`. Result: pass.
- [x] 3.4 A plain Livox 3 run; rewrite the three manifests for the params TOML. Result: 133f63a.
- [x] 3.5 Document the capture and replay in the user guide. Result: 249acac.
