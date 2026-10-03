# Proposal

## Why

After every window, `optimize_windows` (`app/run_odometry.cpp`) places all later windows again, each after the new end of the one before it:

```cpp
for (int j = i + 1; j < windows.size(); j++)
    place_window_after(windows[j], windows[j - 1].poses[windows[j - 1].poses.size() - 1]);
```

The cost of this grows with the square of the number of windows. On `raw data/44` (690 windows) that is about 238,000 window placements, where 690 would be enough.

The extra placements are not needed. The windows after `i + 1` are placed again anyway before their turn comes, so only window `i + 1` uses the result. And each placement rebuilds a window's relative poses as `tmp[k - 1].inverse() * tmp[k]`, so window `j` has its relative poses rebuilt `j` times. That adds up rounding error that has no meaning.

Step 6b (`window-loop-steps`) found this, and the user asked for it as its own change. The fix changes the last bits of the results, so it can't be part of a pure refactor.

## What Changes

- **Only the next window is placed** after the end of window `i`: `place_window_after(windows[i + 1], <end of window i>)` when `i + 1 < windows.size()`. A window is still placed after the optimized end of its predecessor before it is processed. That matters whenever its initial guess doesn't do the placing:
  - when `use_motion_from_previous_step = false`;
  - for window 1 when the velocity prior is off.
- **The failure path keeps a continuous result.** When the NaN check stops the run at window `i` (`return false`), the run is still post-processed and saved. Today all windows after `i` lie in one chain from the last placement, so the saved trajectory has no jump. On the failure path, the code now places all remaining windows in order, starting with window `i + 1` after the restored window `i`. That is one placement per remaining window, done once.
- **New pure helper `place_windows_after(windows, first)`** in `app/window_steps.{h,cpp}`. It places `windows[first..]` each after the end of its predecessor and is used on the failure path. Tests in `tests/window_steps_test.cpp` check:
  - each window starts at its predecessor's end;
  - the relative poses are kept;
  - a single call matches the old repeated placement within 1e-9.
- **New baselines.** The results change only in the last bits.
  - Both the fast and the precise build are checked against the current references `raw data/44_short_baseline` and `raw data/44_baseline` with `--tolerance 1e-6`.
  - Once they pass, `44_short` and `44` get new manifests and baseline folders, in a commit of their own. The old folders are kept with a date suffix.
- **Docs.** The re-chaining note leaves `docs/ARCHITECTURE.md`. `tools/README.md` gets the baseline refresh. `CLAUDE.md` is updated if it needs to be.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

None. The odometry results stay the same up to rounding: within 1e-6, checked against the baseline. The failure path still saves a continuous trajectory. So no requirement changes (`skip_specs: true`).

## Impact

- **Code:**
  - `app/run_odometry.cpp`: one loop becomes one call, plus the failure path;
  - `app/window_steps.{h,cpp}`: the new helper;
  - `tests/window_steps_test.cpp`.
- **Results:** last-bit differences in trajectories, LAZ, poses and the results JSON. Both `44_short` and `44` must stay within `--tolerance 1e-6` of the current baselines, after which the new baselines are written: `tools/baseline/44_short.manifest.json`, `tools/baseline/44.manifest.json`, `raw data/44_short_baseline` and `raw data/44_baseline`.
- **Run time:** slightly shorter. The removed placements are cheap compared with the scan-to-map optimization, so expect seconds on the 30-minute 44 run, not minutes. The odometry time is measured and reported.
- **GUI preview:** reads only windows that are already done, so it doesn't depend on the later windows' placement. With this change the worker also stops writing to windows that aren't done yet.
