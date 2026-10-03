# Design

## Context

`run_lm_iterations` (`app/run_odometry.cpp`, file-local) runs up to `nr_iter` (100) calls of `optimize_lidar_odometry` for one window. It stops early only when:
- `delta` falls below `convergence_delta_threshold` (1e-12 by default, never reached);
- or the window has taken longer than `real_time_threshold_seconds` (10 s).

It returns a `WindowOptimization` (`last_iteration`, `delta`), which `log_window_result` prints as `finished at iteration N/100 …`.

The synthetic measurement in `tests/scan_to_map_test.cpp` ("iterations needed") defines the two numbers this change reports, and it is the template for them:
- **N_final:** the first iteration after which the poses stay within 0.1 mm and 0.001° of the result after all iterations;
- **N_fast:** the first iteration with `delta < 1e-2`.

## Goals / Non-Goals

**Goals:**
- Measure N_final and N_fast per window on real data, with the error at N_fast, and report their distribution for a whole run.
- Leave the results bit-identical, whether the setting is on or off.

**Non-Goals:**
- Changing `nr_iter`, `convergence_delta_threshold` or the LM damping. That is a later change, judged with `eval_odometry.py` on 44.
- A new stopping rule, such as stopping when the poses settle. This change only measures whether one would pay off.

## Decisions

1. **A debug setting, off by default:** `log_iteration_convergence` in `[lidar_odometry_debug_info]`, next to `save_calibration_validation`. It is added to `TomlIO::POINTERS` and `CATEGORIES`, and the golden TOML files are regenerated in their own commit (the usual rule in `CLAUDE.md`). Alternative: always log. Rejected, because it would add one line per window to every run.

2. **The observation lives in `run_lm_iterations`.** With the setting on, it keeps `std::vector<std::vector<Eigen::Affine3d>>` of the poses after each iteration and a `std::vector<double>` of the deltas, and computes N_final, N_fast and the error at N_fast after the loop. The results go into a new `IterationConvergence` struct inside `WindowOptimization`, which `optimize_windows` passes to a new `log_iteration_convergence` step and adds to `WindowLoopCarry` for the summary.
   - The pose error is the largest per-node position difference and the largest rotation angle, the same as `max_error` in the test.
   - **Rule for stops:** if the loop stopped early because of the time limit or the threshold, N_final is measured against the last iteration that ran, and the line says `stopped at N`.

3. **A pure helper, `convergence_of(poses_after, deltas)`, in `app/window_steps.{h,cpp}`,** computes N_final, N_fast and the error from the recorded data. It is tested in `tests/window_steps_test.cpp` on made-up sequences:
   - poses that settle at a known iteration;
   - deltas that cross 1e-2 at a known iteration;
   - a run cut short.

   The test file then has the same definitions as the 6c measurement, and they can't drift apart.

4. **Summary at the end** (`finish_run`), only with the setting on:
   ```
   iteration convergence over 690 windows: settled after median 9, p90 23, max 100; delta < 1e-2 after median 6, p90 15; error at that point median 0.02 mm, p90 0.4 mm, max 3.1 mm
   ```
   Percentiles use nearest-rank.

5. **`tools/convergence_summary.py`** (pure Python 3) parses the `convergence window …` lines from a saved console log. It prints:
   - a histogram of N_final in steps of 5;
   - the same for N_fast;
   - the ten windows with the largest error at N_fast, with their window numbers, so they can be looked at.

   The console has to be redirected to a file for this (`dMapper … > run.log`), which is how the step checks already run.

6. **Bit-identity is checked in both modes.** The observation only reads `window.poses` after each call. `optimize_lidar_odometry` (`scan_to_map.cpp`) is untouched, so the arithmetic that produces the results doesn't change.
   - `44_short` runs with the setting off and on, and both must be IDENTICAL with the manifest.
   - If the `/fp:fast` build changes last bits anyway, because `run_lm_iterations` is reshaped, the precise-build rule of `window-loop-steps` applies: the precise build must stay bit-identical, and the fast build must be within 1e-6.

7. **The measurement:** one full 44 run with the setting on, from a copy of `settings/lidar-vertical.toml` in the scratchpad with only this key changed. The output must be IDENTICAL with the 44 manifest (decision 6). Then:
   - `tools/convergence_summary.py` gives the distribution;
   - `docs/PERFORMANCE.md` gets it next to the synthetic numbers, with what it suggests: a smaller `nr_iter`, a convergence threshold, or neither.

## Risks / Trade-offs

- **[The time limit]** Copying the poses adds a little time per iteration. A window close to the 10 s `real_time_threshold_seconds` could stop one iteration earlier and change its result. → The copy is about 46 KB per iteration, a few microseconds against ~24 ms per iteration on 44. The 44 run with the setting on must still be IDENTICAL (decision 7). If it isn't, implementation pauses.
- **[Memory]** 100 copies of 360 poses is about 4.6 MB per window, freed after the window. → Negligible.
- **[Real windows don't converge in the sense measured]** Many windows may never settle to within 0.1 mm, because of the LM damping and the motion-model terms. → Then N_final shows 100 for them, which is itself the answer: these windows still move at iteration 100, and fewer iterations would cost accuracy.
