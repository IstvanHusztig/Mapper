# Design

## Context

Stage 1 of `docs/code-split-and-region-agents-plan.md`. Before the change, a Sonnet reviewer checked which state the window
loop reads.

**What a capture needs besides the maps and `WindowLoopCarry`:**
- `windows[i - 1]` and `windows[i - 2]` (velocity estimate, pose prediction);
- the first window (`prepare_run`: `have_acc`, `gravity_norm`);
- every `NdtBucket` field, including the stored inverses;
- `state.reference_points`.

**What the reviewer found unproblematic:**
- No code on the odometry path iterates over an NDT hash map; there are only lookups by key, and `update_rgd` sorts its
  index lists. So a map loaded in another order gives the same results.
- `ScanToMapWorkspace` (the Cholesky analyses) affects speed only.
- The time limit is the loop's only wall-clock dependence.

## Goals / Non-Goals

**Goals:**
- Rerun a stretch in seconds to minutes, bit-identical with the run.
- Screen a setting on that stretch.

**Non-Goals:**
- Replay from a GUI run.
- A replay that writes a result folder.
- Any change of numbers.

## Decisions

1. **The loop runs in parts, not with a mode switch.** `process_windows(first, last)` is the one loop body; the full run,
   the capture and the replay call it.
2. **The capture is taken where the loop is, not reconstructed.** It is written after the range has run, with the
   outcomes, so the replay can compare.
3. **Forced iterations, not a time limit.** The last iteration index replaces the time check in a replay with the same
   settings. A break on delta or on time ends at the same index, so forcing the index reproduces both.
4. **Same settings means the same odometry settings hash** (`odometry_settings_hash`, as for snapshots). The capture
   settings themselves are in `[lidar_odometry_debug_info]`, outside the hash.
5. **Windows between the first window and the range are empty in a replay.** The loop never reads them; the vector
   ends at the window after the range.
6. **A port in the application layer, the file in infrastructure.** `WindowCaptureSink` keeps the loop free of I/O. The
   binary writer and reader of the snapshot file are shared through `binary_io.h`.

## Risks / Trade-offs

- **Capture size.** The maps and the map points dominate: hundreds of MB for long stretches. The settings choose the
  range.
- **A changed-settings replay is not a run.** It starts from the map the original settings built, which can differ from
  the map the new settings would have built by then.
