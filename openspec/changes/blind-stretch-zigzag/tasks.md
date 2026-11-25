# Tasks

Experiment folder: `C:\Users\User\Downloads\dLidar\experiments\consistency\livox4_lever\zigzag\` (rule, scripts,
results). No dMapper run in group 1. Numbers before verdicts; the user judges the maps.

## 1. Freeze the rule, prototype it, read it (design D3-D6)

- [x] 1.1 `zigzag/RULE.md`: the changes to `fine_time/RULE.md` (design D3), the criteria and readings (D5), what was
      known when it was frozen (D4). Done when the conclusion reviewer has read it against the design and the code of
      `tools/loop_fine_time.py` it changes, its findings are worked in, and the file is dated before any run of the new
      form.
      (2026-11-25: frozen. The conclusion reviewer read it against the design, the trace and the tool: the core holds; its
      corrections are in (see design D4). No run of the new form before this.)
- [x] 1.2 `tools/loop_fine_time.py`: `--straight-gaps` (the knots, the nodes' move, the effective offsets, section 8's
      two changed conditions), `--fit-total` (reading F2), the fall in the summary and the correction, effective offset
      and total in `windows.csv`. `tests/test_loop_fine_time.py`: the corridor cases of D5 (a reversal with an even true
      error: every gap node within 0.1 m of the truth; the error only in the second half: the line's error equals the
      analytic value; a right trajectory stays; equal jambs unchanged; the fall on made-up corrections, 1.8 m and 0).
      Done when the tests pass, the existing 18 pass unchanged (the check register: additions only), and without the
      new options the tool's output on `stall_fmt3` equals `rule_fmt3` byte for byte.
      (2026-11-25, merged: 29 tests in tests/test_loop_fine_time.py, 11 new; without the options the output on the three
      captures equals the earlier results byte for byte; the register +11 cases, additions only, recorded.)
- [x] 1.3 The stall capture: the new form and the six criteria of D5; scripts in `zigzag/scripts/` (the gap-returns
      share; the trace table per window: correction, effective offset, total, for the loop closure, today's rule and the
      new form). Done when `zigzag/results.md` has the table of criteria 1 to 6 with pass or miss for each, the trace, the
      edge table, and the cloud of the new form with the before / after picture of the stretch.
      (2026-11-25: `zigzag/results.md` sections 1-4. C1-C5 pass (C1 by construction, C2/C3/C5 cannot tell the forms apart),
      C4 passes (71 -> 81 unconfirmed at -3.547 m; 75 -> 82 -0.267 m), **C6 misses**: 32.43 % against 21.53 %, allowed +1.0.)
- [x] 1.4 The readings F2 and R1 to R4 on the stall capture (design D5). Done when `results.md` has each with its
      numbers and one sentence on what it does and does not show.
      (2026-11-25: section 5. F2 moves three supported windows by 0.1 m; R1: without edge 71 -> 81's along-axis information
      the fall is 0.040 m and nodes outside the support move at most 0.078 m; R2: a pin-to-pin line misses the knots of
      496-506 by up to 1.41 m; R3: 0.467 m at window 482; R4: 4 of 25 weak runs have a fall, 1.837 m and three below 0.03 m.)
- [x] 1.5 Today's Livox 4 capture and the Livox 3 capture: the same table (design D5, last part). Done when `results.md`
      has both, and the two requirements (supported windows unchanged; no agreeing edge contradicted) are checked. If the
      Livox 3 run's status changes: stop, numbers to the user.
      (2026-11-25: sections 6-7. Both requirements hold on both captures. Livox 3 unchanged (not applied, edge 91 => 100).
      Today's Livox 4: applied in both forms, fall 0.694 -> 0.000 m, largest node difference 0.941 m, map checks equal.)
- [x] 1.6 The conclusion reviewer on `results.md`; the numbers, the pictures and the clouds' paths to the user. **Stop
      for the user's decision** on the port.
      (2026-11-25: the conclusion reviewer read the results and the readings made after the miss (section 10); its
      corrections are in 10.4. Reported to the user with the miss and its limits. Waiting for the user's decision.)

## 2. In dMapper, off by default (design D7; only after the user's go)

- [ ] 2.1 The back-end region's agent in its worktree: the setting `loop_closure_fine_time_straight_gaps` (refused
      without the fine-time placement), the knots and the nodes' move, the held-out at the effective offsets, the falls
      in the run's report, the log and the replay's JSON, the new columns of `loop_fine_time.csv`; the corridor cases in
      `tests/loop_fine_time_test.cpp` with the prototype's numbers; golden TOML files in their own commit. Done when the
      step checks hold on the branch (identical everywhere with the setting off; the register: additions only).
- [ ] 2.2 dMapper against the prototype on the three captures (`--from-snapshot` with the setting on; the replay of each
      capture with and without it): the same effective offset in every window, node positions within 1e-9 m, the replays
      bit for bit. Done when the comparison table is in `results.md`.
- [ ] 2.3 Docs with the port: `CLAUDE.md` (the fine-time bullet), `docs/regions/back-end.md`, `tools/README.md`, the
      guide's loop closure step and settings row, `checks/instruments.toml` (the `loop_fine_time` section names the new
      planted-fault cases); the register recorded. Done when `python tools/build_docs_index.py --check` holds and
      `python checks/check_register.py` exits 0.
- [ ] 2.4 Merge, the step checks on the main tree, the master reviewer on the push.

## 3. The corridor recording (when it exists)

- [ ] 3.1 The recording check (the verify-recording skill) and how it was held and walked, asked of the user, before it
      is called usable.
- [ ] 3.2 A full run with the shipped Livox settings, with a snapshot and a loop closure capture; the run check; the
      fall of every weak run. Done when the table of weak runs (time, sigma, eligible or not, what the fine-time step did,
      the fall) is in `results.md`.
- [ ] 3.3 From the snapshot: the fine-time step without and with straight gaps; the edge tables; the position
      differences; pictures of every treated stretch; the clouds. Done when the numbers and the clouds' paths are with
      the user.
- [ ] 3.4 The readings R1 and R2 on this recording, if it has an eligible run or a run that is not timed.

## 4. Close

- [ ] 4.1 The user's decision: on in `settings/livox-vertical.toml` (then the old interpolation is removed with the
      setting, new manifests in their own commit), off, or the graph-level repair as its own change.
- [ ] 4.2 The end runs; the master reviewer; archive.
