# Proposal

## Why

The user wants the GUI to show dMapper's own version in its title bar: "dMapper v 0.1". Today the `gui-workflow` spec requires the title to be exactly "dMapper", without a version number.

## What Changes

- The GUI window title becomes exactly "dMapper v 0.1" (`winTitle` in `lidar_odometry_gui.cpp`).
- The `gui-workflow` requirement "Application name and window title" allows this one version suffix: the title is the name, then " v ", then dMapper's version. It still never shows a folder name, and the executable stays `dMapper`.

Not changed: the HDMapping version (`HDMAPPING_VERSION_*`, 0.95.0) that the result files carry in their names (`HDMapping_params_0.95.0_…`, `HDMapping_results_0.95.0_…`). It identifies the processing heritage and stays as it is.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `gui-workflow`: "Application name and window title" — the title is "dMapper v 0.1" instead of exactly "dMapper".

## Impact

- **Code:** one string in `src/presentation/gui/lidar_odometry_gui.cpp`.
- **Docs:** `CLAUDE.md` (the GUI line that names the title).
- **Results:** unchanged; the title isn't read by any processing code, so no `44_short` / 44 runs are needed.
