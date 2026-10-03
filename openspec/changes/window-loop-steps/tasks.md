# Tasks

**Step checks, on every code commit** (changed on 2026-10-03, design decision 7):
- both presets build `dMapper` and `lidar_odometry_tests`;
- `ctest` passes;
- the **precise build** (Release without `/fp:fast`, `HD_CPU_OPTIMIZATION=GENERIC`, built in the scratchpad) runs `44_short` **bit-identical** with the precise reference that master made;
- the normal Release build runs `44_short` IDENTICAL with `tools/baseline/44_short.manifest.json`, or within `--tolerance 1e-6` of `raw data/44_short_baseline`. Report the maximum difference.

The GUI must be closed while building. Code is moved, not rewritten: `git diff --color-moved` must show each step's body as moved lines.

## 1. Setup and carried state

- [x] 1.1 Add `OdometryRunSetup` and `prepare_run` in `app/run_odometry.cpp` (design decision 3), and `WindowLoopCarry` (decision 4). Replace the setup block (lines 41–90) and the carried locals with them. Run the step checks. Commit. Result: precise IDENTICAL (11 files); fast within tolerance, at most 1e-11 (the first attempt under the old bit-identical rule failed here, which led to decision 7).

## 2. Pure steps (`app/window_steps.{h,cpp}`, tested)

- [x] 2.1 Add `app/window_steps.{h,cpp}` with `place_window_after` (decision 5). Use it for window 0 and for re-chaining the later windows. Add `tests/window_steps_test.cpp`:
  - the chained window starts at the given pose;
  - it keeps every relative pose.

  Compile `app/window_steps.cpp` into the tests. Run the step checks. Commit.
- [x] 2.2 Add `estimate_previous_velocity`, with the span as a parameter, called with 20 (decision 2). The loop logs the drop. Tests:
  - constant velocity gives that velocity and the age;
  - a velocity above the limit is dropped, and `speed` is reported;
  - a previous window with fewer than 2 nodes, or with mismatched timestamps, gives no prior.

  Run the step checks. Commit.
- [x] 2.3 Add `predict_window_poses`: chaining from the previous end, IMU integration, ZUPT, constant velocity and mean shift, with the same branch order. It returns `stationary`. Tests:
  - constant velocity: the guess moves at `prev_velocity`;
  - IMU integration with a known acceleration gives `p = v·t + a·t²/2`;
  - a window at rest gives no motion and `stationary`;
  - the mean-shift fallback.

  Run the step checks. Commit.
- [x] 2.4 Add `apply_gyro_bias_prior` and `is_turn_window`. Tests:
  - the prior covariance grows by rw² · duration;
  - the bias is removed from the guess;
  - a window turning faster than the threshold is a turn, a slower one is not.

  Run the step checks. Commit.
- [x] 2.5 Add `repropagate_window_tail`. Tests:
  - with `tail_nodes = 0`, nothing changes;
  - the tail nodes continue at the velocity of the last kept 20 nodes, with the IMU relative rotations;
  - nodes up to the last kept one are unchanged.

  Run the step checks. Commit.

## 3. Steps in `run_odometry.cpp`

- [x] 3.1 Extract the accelerometer-term gating and options (lines 259–300) and their update after the window (lines 454–488) as two functions. Run the step checks. Commit.
- [x] 3.2 Extract robust mode (lines 305–388) as one function. Run the step checks; this is build-only coverage. Commit.
- [x] 3.3 Extract the LM iteration loop with its window log (lines 390–439), and the gyro-bias update with its log (lines 441–452). Run the step checks. Commit.
- [ ] 3.4 Extract the NaN check (lines 534–549), the map update (lines 566–622), and the end-of-run summary (lines 625–647). Verify:
  - the loop body is a sequence of step calls;
  - `wc -l app/run_odometry.cpp` is about 400 or less, with the loop function about 200.

  Run the step checks. Commit.
- [ ] 3.5 Rename `compute_step_2` to `optimize_windows` with `tools/rename_identifiers.py` (dry run first) in `app/`. Fix the comment mentions by hand: `app/odometry_pipeline.h` and `app/optimizer_options.h`. Run the step checks. Commit.

## 4. Integration

- [ ] 4.1 Full 44 in the background. Verify IDENTICAL with `tools/baseline/44.manifest.json`, or else that `compare_baseline.py --tolerance 1e-6` against `raw data/44_baseline` passes. Report which.
- [ ] 4.2 Docs. Commit.
  - `CLAUDE.md`: `optimize_windows` and its steps (`app/window_steps.cpp`), plus the Testing list;
  - `docs/PIPELINE.md` and `docs/index.html`: the function name;
  - `docs/ARCHITECTURE.md`: 6b ✅, and a note on the quadratic re-chaining. A new decision: refactors of numeric code are checked bit for bit with a precise build, while the `/fp:fast` build only has to be within 1e-6;
  - `CLAUDE.md` (Testing) and `tools/README.md`: how to make the precise build and its reference.
- [ ] 4.3 Run `openspec validate window-loop-steps --strict` and confirm it passes.
