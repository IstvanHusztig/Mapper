# Tasks

## 1. Loop closure capture and replay (back-end region, its branch and worktree)

- [ ] 1.1 Split `run_loop_closure` at the registrations: a first part that makes the try records, a second part that consumes them (choice of tries and edges, weighting, pose graph, pruning, correction of the nodes); no result changes; unit tests and 44_short bit-identical
- [ ] 1.2 Capture port and file (`src/application`, `src/infrastructure`): the windows, the segments and one record per try; `capture_loop_closure` in `[lidar_odometry_debug_info]`, outside the settings hash, off by default; golden TOML files regenerated in their own commit
- [ ] 1.3 `dMapper --replay-loop-closure <capture> <params.toml> <output_folder>`: the second part on the records; trajectory files, loop log, a JSON of the edges used and pruned; bit for bit with the capture's own settings, exit non-zero otherwise; missing tries named; settings that change segments or registrations refused by name
- [ ] 1.4 Option to record all qualifying tries up to a cap, for replays with stricter rules
- [ ] 1.5 Tests on the synthetic loop recording (capture, replay with own settings, a stricter acceptance, a missing try, a refused setting); the cases that take long as `doctest::skip` like the other loop closure cases
- [ ] 1.6 `tools/run_replay.py` gates loop closure replays as well; `tools/step_checks.py` unchanged in time

## 2. Wall pairs from poses (tools, pure Python)

- [ ] 2.1 `tools/wall_pairs.py freeze`: the set from a result and its wall piece table (returns with time, node, position relative to the node; piece, visit, side); its origin and counts stored
- [ ] 2.2 `tools/wall_pairs.py measure`: trajectories from a result folder, an odometry snapshot (`tools/snapshot_trajectory.py`) and a replay folder; the gap table per trajectory and the pairs crossing the threshold between two; refusal of other node times
- [ ] 2.3 Tests in `tests/test_verification_tools.py` (rigid move, shifted visit, refusal); the tool's own check against a cloud: the frozen result reproduces its cloud
- [ ] 2.4 Cross-check on the Livox 4 reference run against the numbers of `experiments/consistency/doubles/codex/analysis-codex.md` (708 pairs, odometry / loop closure / final); differences explained or reported

## 3. Short Livox dataset

- [ ] 3.1 Cut the first part of Livox 4 (`cut_recording`); a run with the current settings; check that it holds accepted loop closures and that its wall pairs behave like the full run's first part; report its run time
- [ ] 3.2 After the user's decision on the Livox baselines: the manifest under `tools/baseline/`, the run in `tools/end_runs.py`, `tools/README.md`

## 4. Timing and the time limit

- [ ] 4.1 One log line per stage with its duration and one with the number of windows stopped at the time limit, in every run; no result changes
- [ ] 4.2 `tools/run_health.py` reports both (zero: independent of load; no count: NOT MEASURED); tests
- [ ] 4.3 Measure on the short dataset: alone, and two copies side by side on two input folders; bit-for-bit comparison and the counts; show the user and ask about the rule for parallel runs

## 5. Close

- [ ] 5.1 Ownership check, `tools/step_checks.py` on the branch and on the main tree after each merge; full 44 and 44_short bit-identical once at the end (`tools/end_runs.py`); say which configurations were run (Release only)
- [ ] 5.2 Capture the existing odometries of `livox4-double-walls-cause` (one reprocessing each, one after another) and freeze its wall set
- [ ] 5.3 Docs: `docs/regions/back-end.md`, `docs/regions/verification.md`, the guide's instruments chapter, `CLAUDE.md` (Run, Testing), `docs/PERFORMANCE.md`; `python tools/build_docs_index.py`
- [ ] 5.4 Conclusion reviewer on the measurements; master reviewer after the push; archive
