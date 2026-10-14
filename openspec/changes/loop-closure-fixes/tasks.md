# Tasks

Rules: `docs/code-review-tracker.md` (status per finding); acceptance rule in `design.md`, written before the runs; one CPU job at a time; snapshot runs from `raw data/44_c10_sX/odometry_snapshot.bin`; Release `ctest` always.

## 1. G1 — revisits by path (C22)

- [ ] 1.1 Settings `loop_closure_min_path_m` (8.0) and `loop_closure_radius_per_path` (0.5) in `OdometryConfig`, `TomlIO::POINTERS` and the `[loop_closure]` category, `settings/lidar-vertical.toml`, `settings/default_settings.toml`; `loop_closure_min_time_gap_s` removed everywhere (code, table, settings files, `docs/index.html`). Golden TOML files regenerated in their own commit. Verify: `ctest`.
- [ ] 1.2 `run_loop_closure`: each segment records the path walked up to its first and last window (summed over the windows' first nodes); a same-floor pair qualifies when path(i end → j begin) ≥ `loop_closure_min_path_m` and the horizontal centre distance ≤ min(`loop_closure_search_radius_m`, `loop_closure_radius_per_path` × path). The candidate rule as a pure function with a unit test: out-and-back accepted; standing still, the next-but-one segment, and a place seen from 7 m after 9 m of path rejected. `tests/loop_closure_test.cpp` fixture: `loop_closure_min_time_gap_s` replaced. Verify: `ctest` (and the skipped loop-closure cases run once with `--no-skip`).
- [ ] 1.3 Measure: snapshot runs s0–s5, `map_quality.py --floors`, gallery grid image, loop counts and dropped edges, loop-closure runtime. Verdict by the acceptance rule (high effort, Sonnet review); user decides.
- [ ] 1.4 If accepted: new `44_short` and full 44 baselines in their own commit; `docs/loop-closure-research.html` and the tracker (C22) updated.

## 2. G2–G5

- [ ] 2.1 Design G2 (best candidate, edges per segment) from G1's result; then G3 (weights), G4 (outliers), G5 (anchors), each with its own acceptance rule before its runs.
