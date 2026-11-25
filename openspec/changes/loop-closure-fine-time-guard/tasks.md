# Tasks

Who and on what: group 1 the orchestrator (the rule and the reading of results on Fable / high, the tool's code on Opus /
medium); groups 2 and 3 the back-end region agent in a standing worktree (Opus / medium), the orchestrator's files
(`OdometryConfig`, `TomlIO`, settings, the guide) by the orchestrator; group 4 the orchestrator (runs on Opus / medium, in
the background, one at a time; the reading on Fable / high). Release builds only. Every number goes to the user before a
verdict.

## 1. Prototype in Python on the stall capture (no dMapper run, no C++)

- [x] 1.1 Check the inputs: the format 2 capture `livox4_lever/readings_run/run/loop_closure_capture.bin`, the trajectory
      `readings_run/full_both_fixes`, the recording's `cache/` (window count and point times fit the capture's windows) and
      the free disk space. Done when a short script rebuilds one try's source and target points from the cache and gets the
      capture's stored overlap before and after for that try (as Codex's `reviewer-temporal.py` did).
- [x] 1.2 Freeze the rule in `experiments/consistency/livox4_lever/fine_time/RULE.md` before anything is scored: design
      D2-D7 with every constant, the classes and the limit of D6, what "reproduces Codex's result" means (D11) and the
      reading list (D12.1). Done when the file exists, the conclusion reviewer has read it against the design, and no result
      file in the folder is older than it.
- [x] 1.3 Write `tools/loop_fine_time.py` (D2-D7 with D3's point sets; outputs: a trajectory folder, the per-window table,
      the edge table) and `tests/test_loop_fine_time.py` (no data: the profile against brute force on a small grid, the tie
      rule, the pinned ends, plane normals on a synthetic wall, the time-support residual and the classes on constructed
      numbers). Add both to `tools/README.md` and to `docs/regions/owners.toml`. Done when the tests pass with
      `python -B -m unittest` and `python tools/region_owners.py` holds.
- [x] 1.4 Run it on the stall capture. Deliver `fine_time/results.md`: each supported window's offset beside Codex's
      (`registration.csv`), the edge table (first-pose residual, time-support residual before and after, informed share,
      class) for 71 -> 81, 75 -> 82, 71 / 74 -> 84, 71 / 74 -> 85 and 85 -> 92, the cloud (`tools/cloud_from_trajectory.py`), the
      ghost line per height band, the 54 cm double, facade, Wohnen, reference contacts, far faces; as readings, not for
      tuning: the profile with the coupling at a third and at three times its value and with the support limit at 32
      patches, and the number of unsupported windows that moved with their largest offset. Done when the table states for
      each line of D11 whether the prototype reproduces Codex's result, and whether the two mouth edges agree under D6 (if
      they do not, that is the first line of the report in 1.5).
- [x] 1.5 Report to the user: the numbers of 1.4, the before / after images of the stair, what differs from Codex and from
      my estimate in the design, reviewed by the conclusion reviewer. Stop here; groups 2-4 start on the user's go.
      Done 2026-11-25: `fine_time/results.md` (sections 6 and 7 hold the reviewer's additions and the pictures). One
      criterion of the frozen rule is missed (window 481: -1.0 m against Codex's -0.5 m), traced to the denser source
      points; the two mouth edges agree at 0.13 m against limits of 0.21 / 0.22 m. The stop: while group 1 ran the user
      wrote "go until u finish apply too, you have my ok to take your sugested option if questions arise"; groups 2-4
      go ahead on that, with the rule unchanged. The conclusion reviewer advised asking again; noted, the user's
      instruction stands. Not covered by it and left for the user: `settings/` (4.5, 4.6), any push, any Debug build.

## 2. C++: registrations keep the time of their information (no result changes)

- [x] 2.1 `segment_points` keeps each point's window and node; `ndt_refine` also sums the translation block of the normal
      matrix per source window into the try (`LoopClosureTry`). Done when a doctest shows the per-window blocks sum to the
      translation block of `information` and `-tc="loop closure*"` passes with every registration unchanged.
- [x] 2.2 Loop closure capture format 3 with the per-window information; formats 1 and 2 still load. `tools/loop_graph.py`
      `read_capture` reads format 3. Done when the round-trip doctest, a doctest loading a format 2 file and
      `tests/test_loop_graph.py` pass.
- [x] 2.3 The time-support residual as a function of its own (design D6) with a doctest on a constructed edge (information
      late in the segment: first-pose residual 1 m, time-support residual near 0), and in the replay's JSON for every kept
      edge with a source window in a weak run. Done when the doctest passes and a replay of a format 3 capture lists it.
- [x] 2.4 `python tools/step_checks.py` in the worktree: all hold (the capture's new content changes no result). Hand-back
      with the entry for `docs/regions/back-end.md`; the orchestrator checks ownership
      (`tools/region_owners.py --diff`), merges, and runs the step checks on the main tree.
      Done 2026-11-25: branch `region/back-end/2026-11-25-edge-time-support` (6dc3d9c; 8 registrations bit-identical with
      the base, blocks sum to the try's block within 2.7e-15; step checks all hold; six files outside the region, all
      granted), merged as 2d9bdf4, step checks on the main tree all hold. On the real stall case (`fine_time/stall_fmt3`,
      a `--from-snapshot` run of the merged build): the replay of the format 3 capture gives 182,880 of 182,880 nodes
      bit for bit; the blocks of the 81 kept edges sum to their try's block within 1.6e-14; dMapper's time-support
      residuals equal the prototype's with the capture's blocks within 1.5e-9 m and the prototype's own rebuilt blocks
      within 0.007 m (`fine_time/stall_fmt3_time_support.log`). The agent advises a Debug test run of the loop closure
      cases (new index and label vectors); not started, the user's to decide.

## 3. C++: the fine-time step

- [x] 3.1 Settings `loop_closure_fine_time_placement` and `loop_closure_fine_time_min_sigma_m` in `OdometryConfig`,
      `TomlIO::POINTERS` and `CATEGORIES` (`loop_closure`, outside the odometry settings hash), the golden TOML files
      regenerated in their own commit, the refusal without the readings rule. Done when the TOML tests pass, a snapshot
      written before the settings still loads, and a doctest shows the refusal's outcome.
- [x] 3.2 The pure parts, each with doctests: exact nearest neighbours on a grid index (against brute force), patches with
      normals, the score curve of a window, the profile by dynamic programming (against brute force; ties; pinned ends),
      the edge classes and the apply rule of D7. Constants in one struct with the values of `RULE.md`. Done when
      `-tc="loop fine time*"` passes.
- [x] 3.3 The scores port, the run's adapter on the points, the step in `close_loops`, the report and the log lines of
      D8; capture format 4 with the curves and their graph result (format 3 is group 2's per-window information); the replay's adapter, the refusal for missing or stale
      curves, the replay's JSON and `loop_fine_time.csv`. Done when, on the synthetic loop recording, a replay of a capture
      made with the guard on gives every node bit for bit, a replay with a changed graph or another set of eligible runs
      and the guard on exits 1 naming the cause, and with the guard off every existing loop closure test is unchanged.
- [x] 3.4 Truth cases on a synthetic scene: an along-axis error growing inside the blind stretch is recovered to one grid
      step where surfaces across the passage are seen; a corridor with nothing across it comes out `not timed` and
      unchanged, bit for bit; door frames every 2 m with a search range beyond one spacing move no window by more than one
      grid step; an edge with late information agrees, an edge contradicted by informative scans stops the run, a later
      visit registered onto a moved part stops the run. Done when they pass with `--no-skip`.
      Changed 2026-11-25: the scene is the prototype's corridor (walked out and back, 100 windows), not the synthetic
      route R2. The agent found that an R2 case needs R2's windows with hand-set readings and truth poses plus an
      injected error; the corridor has the same parts (a blind stretch, surfaces across it, another visit 20 s later)
      and lets the C++ be checked against the Python window by window. All six cases pass (`-tc="loop fine time*"`, 22
      of 22); the offsets of all 27 windows and the held-out means equal the Python bit for bit. An R2 case is listed
      as a follow-up in `docs/regions/back-end.md`.
- [x] 3.5 Equality with the prototype: the orchestrator runs `--from-snapshot` of `readings_run/run/odometry_snapshot.bin`
      with that run's settings plus the guard and `capture_loop_closure`. Done when the supported windows and every
      window's offset equal `tools/loop_fine_time.py`'s on that capture (a difference at a limit is named and explained),
      and the replay of the new capture is bit-identical.
      Done 2026-11-25 on the final build 72b75c6 (`fine_time/stall_guard2`, `results_dmapper.md` section 1): the same
      support and offset in all 37 windows, 22 edge rows with equal classes, 182,880 nodes within 1.0e-9 m, the replay
      bit for bit. The same on today's Livox 4 (28 windows, 23 edge rows). No difference at a limit appeared.
- [x] 3.6 Cost and checks: the loop closure stage's time with and without the guard on that run, written into the
      hand-back; `python tools/step_checks.py` in the worktree all hold. The agent states in its hand-back whether a Debug
      test run is advisable (new index and container code); the orchestrator asks the user before any Debug build.
      Done 2026-11-25: the step logs its own time: 8.9 s on the stall case and 7.2 s on today's Livox 4 (one eligible
      run each) in a loop closure stage of about 600 s. Step checks: everything identical except 44_short's parameters
      TOML (the two new keys; manifests: task 4.4). A Debug test run is advised by the agent for the new index and
      label vectors; **not started, the user's decision**.
- [x] 3.7 Merge: ownership check, the conclusion reviewer on the hand-back, merge (no number changes with the switch off),
      `tools/step_checks.py` on the main tree, master-reviewer on the push.
      Done 2026-11-25, locally, nothing pushed: 19e1c45 (the step) and 72b75c6 (review fixes). The master reviewer read
      both merges before any push: no result change with the switch off, no blocking defect, six minor findings, all
      fixed in 72b75c6. The push itself, and the master reviewer on it, wait for the user.
- [x] 3.8 Docs outside the guide: `docs/regions/back-end.md` (the hand-back's entry, the known limits of the design),
      `CLAUDE.md` (Run: the fine-time step and the capture format), `tools/README.md`. Done when the three files describe
      the step as built. (The guide is task 5.1.)

## 4. Judged on the recordings

- [x] 4.1 The stall case from dMapper (the result of 3.5): the list of design D12.1 beside `readings_run/full_both_fixes`,
      the edge table, verify-map, the before / after images of the stair on every storey. Done when
      `fine_time/results_dmapper.md` holds the table and the image paths.
- [ ] 4.2 Today's Livox 4: one full run with `settings/livox-vertical.toml`, `save_odometry_snapshot` and
      `capture_loop_closure` (guard off; about 35 min, background), verify-run; then `--from-snapshot` with the guard on.
      Done when the same list beside the guard-off result is in the results file, with the eligible runs and what the
      guard did with each.
- [ ] 4.3 Livox 3: the same two runs with the shipped file. Done when the eligible runs, the largest position difference
      to its baseline (`tools/trajectory_seams.py`) and the before / after images are in the results file.
- [ ] 4.4 Switch off: `tools/end_runs.py` (full 44, Livox 3, 44_short against their manifests; about 50 min, background).
      Done when all three are IDENTICAL.
- [ ] 4.5 The conclusion reviewer on the results; numbers and maps to the user. Stop for the user's decision: on in
      `settings/livox-vertical.toml`, off, or wait for the new recording.
- [ ] 4.6 Only on the user's decision to ship: the setting in `settings/livox-vertical.toml`, new manifests, in their own
      commit, after the user's look at the before / after maps. Done when `tools/end_runs.py` is IDENTICAL against the new
      manifests.

## 5. Close

- [ ] 5.1 The guide, by the `sonetd` agent (the user, 2026-11-25: after the apply, document this with sonetd in the
      dMapper guide). Once groups 1-4 are done, start `sonetd` in the background with: this change's `design.md` and
      `specs/`, the hand-backs, the results files, and the guide's rules (`CLAUDE.md`, Documentation: English only, terms
      defined once in Fundamentals, no names of this workstation's recordings in the guide's text). It writes the step,
      the two settings, the edge classes and the `not timed` report into `docs/index.html` (loop closure and settings; a
      new term in Fundamentals if one is needed) and runs `python tools/build_docs_index.py`. Done when
      `python tools/build_docs_index.py --check` holds and the orchestrator has read the agent's text against the code.
- [ ] 5.2 `docs/code-review-tracker.md` (C25: what this step covers and what it leaves); the open follow-ups written
      into `docs/regions/back-end.md` (edges read at their time support inside the graph, two runs with different axes, the
      target side's time support). Done when both files name them.
- [ ] 5.3 The new recording (task 4.4 of `loop-closure-weak-runs-and-match-check`): the guard runs unchanged beside the two
      rules; its table goes to the user with that task's numbers. Done when that task's report includes it.
- [ ] 5.4 Archive this change; task 4.3a of `loop-closure-weak-runs-and-match-check` points here.
