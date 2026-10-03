# Design

## Context

In `optimize_windows` (`app/run_odometry.cpp`), the step after the NaN check places every later window after its predecessor's end:

```cpp
// every later window continues from the new end of this one
for (int j = i + 1; j < windows.size(); j++)
    place_window_after(windows[j], windows[j - 1].poses[windows[j - 1].poses.size() - 1]);
```

`place_window_after` (`app/window_steps.cpp`) rebuilds the window from its own relative poses (`tmp[k - 1].inverse() * tmp[k]`), starting at the given pose.

Before window `i` is optimized, these read its poses:

| Reader | What it uses |
|---|---|
| `estimate_previous_velocity` | only the timestamps of window `i` |
| `predict_window_poses` | re-chains window `i` from the end of window `i - 1`, using only its relative poses; runs when `use_motion_from_previous_step` is on and (`i > 1`, or the velocity prior is on and `i > 0`) |
| `apply_gyro_bias_prior`, `is_turn_window`, `prepare_acc_term` | the poses as they are at that point |

So the absolute placement of window `i` matters only when `predict_window_poses` does not run, and then it comes from the placement made after window `i - 1`. No step reads window `j > i + 1` before its turn.

`finish_run` and `post_process` run after the loop, when every window has been processed. The GUI preview reads only the windows reported done.

The one exception is the failure path. When `add_window_distance` finds a NaN at window `i`:
- window `i` gets back its poses from before the optimization;
- the function returns `false` before the placement step;
- `OdometryPipeline::run_all` still runs `post_process`, and the CLI and GUI save the result.

Today, windows `i + 1 …` at that point are still chained after the placements of iteration `i - 1`, so they form one continuous chain.

## Goals / Non-Goals

**Goals:**
- One placement per window in a normal run, instead of `O(n²)`.
- Results that differ only in rounding: within `--tolerance 1e-6` of the current baselines, on both builds.
- A failed run still saves a continuous trajectory.

**Non-Goals:**
- No change to the initial guess (`predict_window_poses`), the optimizer, or the velocity span. The longer velocity span is a later, separate change.
- No other speed work. This change is about the quadratic loop; the 30-minute run time is spent in the scan-to-map optimization.

## Decisions

1. **Place only window `i + 1` after window `i`.** This is the only placement that any later step reads (see Context). Alternative: drop the placement altogether and rely on `predict_window_poses`. Rejected: when the guess is off (`use_motion_from_previous_step = false`, or window 1 without the velocity prior), the window would start at its step-1 IMU-only placement, which is a real behaviour change, not rounding.

2. **On the failure path, place all remaining windows once with `place_windows_after(windows, i + 1)`.** It is called just before `return false`, after window `i` has its old poses back. The remaining windows then chain from the restored window `i`.
   - Today they chain from window `i`'s placement before its initial guess. When `predict_window_poses` changed window `i`'s translation (IMU or constant-velocity prediction), that start differs slightly. The new start, the restored window's actual end, is the more consistent one.
   - This path never runs on 44. It is covered by a unit test of the helper, not by a run.

   Alternative: keep the full loop on the failure path only, which is the same thing done inline. Rejected, because a named helper can be tested.

3. **`place_windows_after(std::vector<Window>&, size_t first)` is a new pure helper in `app/window_steps.{h,cpp}`** that loops over `place_window_after`. When `first == 0` or `first >= windows.size()`, it does nothing. The normal path calls `place_window_after` directly for the one window.

4. **Validation.** Both builds' last bits change, so the exact checks from step 6b don't apply. Every code commit is checked like this:
   - **fast build:** `44_short` within `--tolerance 1e-6` of `raw data/44_short_baseline`;
   - **precise build:** `44_short` within `--tolerance 1e-6` of `raw data/44_short_precise_baseline`;
   - report the largest difference.

   Then the full 44 runs once, with the fast build, within `--tolerance 1e-6` of `raw data/44_baseline`.

   If a check goes over 1e-6, pause and compare the trajectories and map sharpness with `tools/eval_odometry.py` before deciding anything.

5. **New baseline in a commit of its own**, once the checks pass:
   - the old folders are renamed `44_short_baseline_2026-10-03b` and `44_baseline_2026-10-03b`;
   - the new outputs become `raw data/44_short_baseline` and `raw data/44_baseline`;
   - both manifests are rewritten with `--write-manifest` and record the commit;
   - the precise reference in the scratchpad is refreshed too;
   - `tools/README.md` notes the refresh and why.

6. **Run time is measured, not assumed.** Both runs log the odometry's `elapsed time` (from `finish_run`). Master and this change are compared on 44_short and on 44, and the result is reported as it is, even if it is only seconds.

## Risks / Trade-offs

- **[Rounding grows over the run]** Small differences can grow through 690 windows of nonlinear optimization. Step 6b moved 44 by 1e-9 at most, so 1e-6 is expected to hold. If it doesn't → decision 4: metrics before a decision, no silent baseline refresh.
- **[Failure path changed without a run]** No dataset triggers the NaN path. → It is covered by the helper's unit test, and the code path is three lines.
- **[Gain is small]** About 238,000 placements of a few hundred nodes each is likely seconds out of 30 minutes. → The real value is a cleaner loop and less meaningless rounding. The proposal says this plainly, and task 3.2 measures it.
