# Proposal

## Why

The CLI and the GUI drive the processing stages separately, and they have drifted apart:
- **CLI:** step 2, then loop closure, then consistency (by the TOML flags).
- **Simple GUI:** step 2 only.
- **Advanced GUI:** step 2, then an optional consistency button.

So the GUI never runs loop closure, and on raw data 44 its result is up to 0.42 m worse than the
CLI's, which the user confirmed is the better one. The stages are also wired through file-scope
globals in `app/lidar_odometry_gui.cpp`. And both front ends first load `default_settings.toml`
from the *current directory*, so a CLI result can depend on where it is started. This is step 2
of the `docs/ARCHITECTURE.md` roadmap: one pipeline, two front ends.

## What Changes

- **New `OdometryPipeline`** (`app/odometry_pipeline.{h,cpp}`) owns one run's data: the point
  chunks, IMU, AHRS trajectory, windows (`worker_data`) and `OdometryState`. Its stages:
  - `load(files)`: `load_data` → `calculate_trajectory` → `compute_step_1`;
  - `run_odometry(progress, pause)`: `compute_step_2`;
  - `post_process()`: loop closure if `use_loop_closure`, then consistency if `use_consistency`
    and `num_constistency_iter > 0`, exactly the CLI's current order and conditions;
  - `save(out, gui_prefs, elapsed)`: `save_result`.

  It reads the config `const` and keeps no globals.
- **CLI** (3-argument mode) runs `load` → `run_odometry` → `post_process` → `save` on its own
  pipeline object. The output is unchanged (bit-identical to the baselines).
- **GUI:**
  - Both flows run the same stages. The simple flow ("Open"/`openData`) runs load, odometry,
    post-processing and save. In the advanced flow, "Compute all" runs odometry plus
    post-processing.
  - **BREAKING (GUI only):** the GUI now runs loop closure and consistency when the settings
    enable them, so its output changes and becomes identical to the CLI's for the same input and
    settings.
  - The separate "Point cloud consistency and trajectory smoothness" button is **removed**.
  - The globals `worker_data`, `trajectory`, `imu_data`, `pointsPerFile` and `state` become one
    `OdometryPipeline` owned by the GUI. Only UI state (view flags, gizmo, progress, window) stays
    file-scope, because GLUT callbacks need it.
- **Settings sources:**
  - **CLI:** built-in defaults plus the given TOML only. It no longer reads `default_settings.toml`
    from the current directory, and it exits non-zero if the given TOML can't be parsed (today it
    silently runs with defaults).
  - **GUI:** at start-up it loads `default_settings.toml` from the **exe's folder** (where the
    build copies it), not from the current directory.
- Docs: `CLAUDE.md` (Run section and pipeline description) and `docs/ARCHITECTURE.md` (step 2
  done).

## Capabilities

### New Capabilities
- `processing-pipeline`: the sequence of processing stages shared by CLI and GUI, where settings
  come from, and the guarantee that both front ends produce identical output for the same input
  and settings.

### Modified Capabilities
- None. `regression-baseline` is unchanged: the CLI output must stay bit-identical to its
  manifests.

## Impact

- New: `app/odometry_pipeline.h`, `app/odometry_pipeline.cpp` (added to `SOURCES` in
  `CMakeLists.txt`; not part of `core`, so `CORE_SOURCE_FILES` is untouched).
- Changed: `app/lidar_odometry_gui.cpp` (main, CLI branch, `step1`/`step2`/`openData`/
  `save_results`, about 120 references to the former globals in display and editing code), docs.
- No change to `core/`, the optimizers, the TOML format or the output files.
- **Verification:**
  - **CLI:** `44_short` per commit and full 44 at the end, both bit-identical.
  - **CLI from another directory:** a run of `44_short` with the working directory set to the
    exe's folder must also be identical, which proves the result no longer depends on the current
    directory.
  - **GUI (done by the user):** the simple flow on `44_short` with `settings/lidar-vertical.toml`
    loaded must be bit-identical to `44_short_baseline` on **all 11 files**, not just the step-2
    files as today.
