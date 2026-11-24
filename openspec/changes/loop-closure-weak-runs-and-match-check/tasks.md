# Tasks

## 1. Blind-stretch rule in dMapper (back-end agent, Opus)

- [x] 1.1 Readings on the window (`Window::readings`, set in `report_window`), snapshot and capture format 2 (format 1 still loads), `tools/loop_graph.py read_capture`; verified by the snapshot tests and `tests/test_loop_graph.py` (branch `region/back-end/2026-11-24-odometry-step-covariance`, faabae5)
- [x] 1.2 The rule as a pure function (`loop_odometry_step_information`) used by solve, cost and pruning; doctest cases (multi-step run, single-step run, bridged window, run at the last segment); cross-check with the prototype's inputs: 25 runs, stall T 8.95 s, S1 2.00, S2 4.63, sigma 5.04 m, shares 0.16/0.24/0.24/0.24/0.12
- [x] 1.3 Settings `loop_closure_odometry_from_readings` (false) and the three constants; golden TOML files in their own commit (0248599)
- [x] 1.4 `tools/step_checks.py` on the branch: build, 245/245 tests, both guards, both replays, term table identical; 44_short differs only in the parameters TOML (four new lines)
- [ ] 1.5 Review of the branch (ownership diff, conclusion reviewer, master-reviewer on the diff). master-reviewer 2026-11-24 on 5e5d148..647629d: nothing changes with the switches off, nothing blocks the merge; to fix first: a quantile outside (0,1) silently disables the check, and the check with the robust kernel is not the held-out innovation (both: refuse); recorded as limits of the frozen rules: a window without any LiDAR information is loosened along one arbitrary axis only (also windows weak in two axes), each test solve starts from the odometry poses, the dof threshold mixes translation and rotation units. The agent's third job (the two refusals) is running; `src/domain/chi_square.h` needs its entry in `owners.toml`
- [ ] 1.6 User's decision: new 44_short and 44 manifests for the parameters TOML (the four, later six, new setting lines; no other file changes)
- [x] 1.7 Replay exits 1 when the readings are missing (`loop_closure_replay_cli.cpp`); `tools/snapshot_trajectory.py` accepts format 2 (c5420a1; `tests/test_verification_tools.py` 40 tests pass)

## 2. Confirmation of the blind-stretch rule on Livox 4

- [x] 2.1 One full Livox 4 run from the branch build with `capture_loop_closure` and `save_odometry_snapshot` (switch off): `experiments/consistency/livox4_lever/readings_run/run`. 0 windows at the time limit; against the 2026-11-23 run the largest position difference is 0.000 m (`tools/trajectory_seams.py`), the same 13 edges pruned, 79 kept: the run already verified, reproduced
- [x] 2.2 Replay of the new capture with the switch on (`readings_run/replay_rule_on`): 25 runs, the stall at sigma 5.04 m with shares 0.16/0.24/0.24/0.24/0.12, the same 10 edges pruned in the same order and with the same weighted residuals as the Python mirror, 82 kept. The same graph solved in Python from the capture's own readings: largest node position difference 0.0018 mm (`scripts/rule_cpp_against_python.py`)

## 3. Match check: prototype (Fable/high) and C++ (back-end agent, Opus)

- [x] 3.1 Design look: test values of all edges on the rule graph and on uniform steps, three family definitions (`scripts/match_check_look.py`, `match_check/look.csv`)
- [x] 3.2 Procedures compared on the kept sets (`scripts/match_check_sets.py`, `sets.log`); dMapper's pruning mirrored (gives the shipped 79 on uniform steps)
- [x] 3.3 Rule frozen before any map check: `match_check/RULE.md`
- [x] 3.4 The frozen procedure's trajectory, checks beside Step A, one map on the drawing (`scripts/match_check_runs.py`, `match_check/R_mc`): 10 pruned, the check drops 24->63 (T 36.0), 81 kept; 54 cm double +0.015, facade -0.07 deg, Wohnen +0.308, reference 5/0, far faces 18/0, gallery +0.635, corridor gain +5.72 m; conclusion reviewer's corrections taken (the 79 are the uniform-step kept set; "the check rejects on uniform steps edges the rule graph accepts"; all double columns shown). Numbers and overlay given to the user 2026-11-24; the user's look is open
- [x] 3.5 C++: the match check in `close_loops` after the pruning (D3, D4), settings `loop_closure_match_check`, `_quantile`; doctest cases (influential false edge dropped; edges across a blind stretch kept; an edge no loop reaches kept; refusal without the readings rule); log lines; step checks identical with the switch off (c5420a1, 647629d: 251/251 tests, guards and replays identical, 44_short differs only in the parameters TOML's six new lines)
- [x] 3.6 Replay of the new Livox 4 capture with both switches on: the same 10 pruned edges, 24->63 dropped at T 35.99 (limit 22.46, 2 held out, 2.08 m), 81 kept, largest T left 13.71 / 7.77 / 5.88 / 5.23; trajectory against the prototype's R_mc within 0.000001 m; loop closure step 1.12 s without the check, 6.71 s with it
- [x] 3.7 dMapper's own map: `--from-snapshot` run with both switches on (`readings_run/both_on_from_snapshot`, settings `readings_run/settings_both_on.toml`): the same log lines as the replay; `all_checks.py` and `facade_wohnen.py` equal the prototype's R_mc in every column; `overlay.png` and `readings_run/before_after_dmapper.png` for the user's look. User, 2026-11-24, on both pictures: "They look good"

## 4. dMapper reproduces the map

- [x] 4.1 Merge after review (f009881, a1dd169); `tools/step_checks.py` on the main tree: build ok, 253/253 tests, characterization, loop guard, both replays and the term table identical; 44_short differs only in the parameters TOML (the six new lines; the user OK'd new manifests 2026-11-24)
- [ ] 4.2 Full Livox 4 run (or `--from-snapshot`) with both switches on in a settings copy; verify-run; before/after maps and the overlay with the user's fit beside the Step A overlay, for the user's look. `--from-snapshot` done (task 3.7, the user: "They look good"); the plain full run from the raw recording with the merged main tree's dMapper and `readings_run/livox4_both_fixes.toml` (`readings_run/full_both_fixes`): 0 windows at the time limit, the same loop closure lines, largest position difference to the `--from-snapshot` result 0.000 m. Open: verify-run on it (its log has no window diagnostics; the odometry is the verified run's)
- [ ] 4.3 User's decision 2026-11-24: "They should go into the settings file": both switches on in `settings/livox-vertical.toml` (edited, not committed). Open before it is committed and pushed: the tests that load the file, the golden TOML, Livox 3 and Livox 4 with the shipped file before/after for the user's look, new Livox 3 reference. Note: the shipped file's odometry settings differ from the run the judged map came from (`loop_capture/settings.toml`: degenerate projection ratios 0.02/0.04, acceleration offset random walk 0.01, exact NDT statistics, consistency off), so the shipped file with both rules is another map; whether those settings ship is the user's separate decision. User, 2026-11-24: "Yes put experiment settings too": the shipped file now equals `readings_run/livox4_both_fixes.toml` except explicit defaults (also `ahrs_gain` 0.0 instead of 0.05 and `gyro_scale_z` 1.004, which I had not listed to the user; reported to them). Recording 44 (map-quality gate s0-s2) before the rules go on for the L2
- [ ] 4.4 The new recording: verify-recording, full run with capture, both rules unchanged, Codex's hybrid form as a second arm in Python; numbers and maps to the user

## 5. Close

- [ ] 5.1 Docs: the six settings and the two rules in the guide and the back-end region's knowledge file (the agent's hand-back entry), `CLAUDE.md`, `tools/README.md`; `python tools/build_docs_index.py`
- [ ] 5.2 Code-review tracker; master-reviewer after the push
- [ ] 5.3 Archive; `livox4-double-walls-cause` tasks 3a.2-3a.5 pointed here
