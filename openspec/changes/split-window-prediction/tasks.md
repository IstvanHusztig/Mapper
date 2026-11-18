# Tasks

**Step checks, on every commit:**
- the Release build;
- the unit tests;
- the characterization guard and the loop guard, byte-identical with the references recorded in task 2.1;
- the replays of the 44_short capture (20–40 s, 21 windows) and the Livox 3 capture (100–130 s, 84 windows), every
  window identical;
- 44_short against its manifest;
- the per-term table of the 44_short replay against the reference recorded in task 2.1.

## 1. Design

- [x] 1.1 Write the design into the plan and have it reviewed against the code. Result: plan subsection "Stage 6 design",
  2026-11-18 (6672aac); the review's findings worked in.
- [x] 1.2 The user's decisions on the five points. Verify: the plan's heading reads "Decisions (user, <date>)". Result:
  2026-11-18, all five as recommended (83fc1c4).

## 2. Guards

- [x] 2.1 Record the characterization reference, the loop guard's reference and the per-term table of the 44_short replay
  from the code at the change's start, and the fit diagnostic's lines (`accelerometer fit`) of the 44_short replay with
  `log_window_diagnostics`. Verify: the guards equal stage 5's references (the code is unchanged since). Result: recorded
  twice at b4a91ea, identical with stage 5's; 21 `accelerometer fit` lines.

## 3. The split (pure refactors)

- [x] 3.1 `window_rests` (`window_steps`) with the named half width; `predict_window_poses`, `window_at_rest` and
  `log_accelerometer_fit` call it, their size checks and the threshold conversion unchanged. Test: a still and a walking
  window. Verify: step checks; the `accelerometer fit` lines identical with task 2.1's. Result: step checks and the
  per-term table identical; the 21 fit lines identical; the test passes.
- [x] 3.2 `predict_window_poses` in four steps: `mean_shift_of`, the chaining through `place_window_after` (with the
  empty-window guard), the rest test, `guess_window_translations`. Tests of each step (mean shift and its drop, chaining
  bit-identical with `place_window_after`, the four translation branches). Verify: step checks; the existing
  `predict_window_poses` tests pass unchanged; `tools/function_lengths.py` (task 4.1) or a manual count shows every
  function of `window_steps.cpp` under 80 lines.
  Result: `mean_shift_of`, `place_window_after` (empty-window guard), `window_rests`, `guess_window_translations`;
  `predict_window_poses` 30 lines; 3 new test cases, the existing ones unchanged; step checks and the per-term table
  identical; no function of `window_steps.cpp` above 80 lines.

## 4. Tool and docs

- [ ] 4.1 `tools/function_lengths.py <limit> <files or folders>`; `tools/README.md`. Verify: on `src/` with limit 80 it
  lists the plan's table, without `predict_window_poses`.
- [ ] 4.2 Docs: the plan's table and progress, `CLAUDE.md` where it lists the window steps (`window_rests`). Verify: the
  text matches the code.

## 5. End of stage

- [ ] 5.1 Full 44 and Livox 3 against their manifests with the shipped settings (sequential, in the background). Verify:
  IDENTICAL (no setting changes, the manifests stay).
