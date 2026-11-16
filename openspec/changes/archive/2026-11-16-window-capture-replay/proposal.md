# Proposal

## Why

Each look at a bad stretch of a recording costs a full run: about 6 minutes on Livox 4, and 25 on the full 44. The two
Livox 4 faults (the passage stall at 100–125 s, the flyaway at 200–280 s) need many such looks. Stage 1 of
`docs/code-split-and-region-agents-plan.md`: save the window loop's state before a stretch, and run only that stretch
again, bit-identical to the run.

## What Changes

- **The window loop runs any range from a given state** (pure refactor):
  - `WindowLoopCarry`, `OdometryRunSetup`, `OdometryTiming` and `AccTerm` move to `src/application/window_loop.h`;
  - `optimize_windows` becomes `start_window_loop` + `process_windows(first, last)` + `finish_run`;
  - `prepare_run` and `start_carry` read only the first window;
  - `process_windows` has two hooks that a normal run leaves unset: forced iteration counts, and the per-window
    outcomes.
- **Capture:** with `capture_windows_from_s` / `capture_windows_to_s` (`[lidar_odometry_debug_info]`, off by default),
  the CLI writes `window_capture_<from>-<to>s.bin` to the output folder. It holds:
  - the loop's state before the range: the maps with every cell field, the run state, `WindowLoopCarry`;
  - the run's first window, the two windows before the range, the range and the window after it;
  - the range's points;
  - each window's last iteration and final poses.

  The loop runs the windows in three parts around the range, in the same order.
- **Replay:** `dMapper --replay <capture> <params.toml>`.
  - With the capture's own odometry settings, the iteration counts are forced (no time limit), and every window must
    come out bit for bit; exit 1 if not.
  - With other settings it reports how far the windows move.
- **Code:**
  - `src/application/window_capture.h` (port, `capture_range`), `window_replay.{h,cpp}`;
  - `memory_point_store.h` (moved from `tests/`);
  - `src/infrastructure/window_capture_file.{h,cpp}`;
  - `binary_io.h`, the snapshot's writer and reader, now shared.
- **Docs:** `CLAUDE.md`, the user guide.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `processing-pipeline`: window capture and replay.
- `unit-tests`: the capture and the replay are tested on the synthetic loop recording.

## Impact

- **Results:** unchanged; the params TOML has the two new keys.
- **File size:** 73 MB for 21 windows of 44_short, 587 MB for 84 windows of Livox 3.
- **Speed of a replay:** 44_short windows 50–70 in 35 s; Livox 3 windows 279–362 in 93 s (the full run takes 23 min).
- **Limit:** a replay with changed settings starts from the run's own map. It screens an idea on a stretch; a full run
  confirms it.
