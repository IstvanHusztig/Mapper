# Proposal

## Why

`odometry-stall-fix` (archived 2026-10-08) removed the ghost wall of recording 44. It did so with degeneracy-aware NDT terms: along a direction the geometry doesn't constrain, the NDT terms carry no translation information, so the motion model places the window there. The fast and precise builds of the same code still differ by up to **0.31 m** on the full 44. The target was a few centimetres. dMapper is meant for surveying caves hundreds of metres long, where blind passages are common, so the result must not hinge on rounding.

The weak point is how blindness is detected today:
- **A proxy:** Σ n·nᵀ over the normals of the cells the points matched, not the information the optimization actually has.
- **Noisy normals:** with the default cell statistics a normal is fixed from the cell's first three points. Hatleskog & Alexis (RA-L 2025, arXiv:2410.10784) show that such noise adds *spurious* information along the blind axis, the very direction the test must see.
- **Translation only:** rotation is never examined. A round shaft or tunnel can hide rotation about its axis, and a rotation error in a passage becomes a position error further on.
- **One fixed ratio ramp (0.1 to 0.2),** chosen on one recording, with no notion of how many points support a direction.

The literature survey of the previous change (`openspec/changes/archive/2026-10-08-odometry-stall-fix/research.md`) points to a better detector:
- the optimization's own information matrix;
- rotation and translation examined separately (X-ICP, Tuna et al., T-RO 2024; "Informed, Constrained, Aligned", Tuna et al., arXiv:2408.11809);
- a noise-aware signal-to-noise test per direction, with soft attenuation by the probability that the direction is really constrained (Hatleskog & Alexis 2025).

The figures below are what the papers report, not our measurements. Hatleskog & Alexis report 0.04 m error in a mine tunnel against 0.57 m with the classic method (Zhang et al. 2016, the basis of the current projection). On a 500 m road tunnel their method got the length right where the classic method came up 110 m short.

## What Changes

- **Measure first (no change to results):** per window, the information the NDT terms give to the window's common (rigid) motion. Translation and rotation are examined separately, and each eigen-direction's signal is compared with an estimate of the information that noise alone would produce.
  - Logged behind the existing `log_window_diagnostics`.
  - Compared with today's Σ n·nᵀ ratio on the full 44: the blind passage at t = 123–126 s (windows 303–311), the place at t ≈ 74 s with two solution families, and the first room.
- **Detector and soft attenuation:** each direction's NDT information is scaled by the probability p that its signal exceeds s times the noise (s = 10 in the paper). Translation and rotation are both covered, and p replaces today's fixed ratio ramp.
  - Settings: on/off and s, plus the noise model's constants, which are derived in the design task.
  - The current projection stays available (`degenerate_projection_ratio*`) and is the fallback.
- **Judged on the full 44, both builds** (`/fp:fast` and `/fp:precise`), by:
  - the difference between the builds (target a few centimetres, at least clearly below 0.31 m);
  - no ghost wall (the user's visual check);
  - `eval_odometry.py` metrics and loop-closure corrections not worse;
  - time cost below 5 %.
- New baselines if the shipped settings change, and docs.

**Not in this change** (planned afterwards, in this order):
- IMU always on, with velocity and bias as states (Wildcat, LIO-SAM, FAST-LIO2);
- chi-square gating and a robust solver for loop closures (GNC, PCM, LAMP 2.0).

After all three, the user will compare dMapper's map with a real cave map.

## Capabilities

### Modified Capabilities

- `lidar-odometry-motion-model`: the requirement "Degeneracy-aware NDT terms" changes from a geometric ratio on translation to a signal-to-noise test on the NDT information, covering rotation and translation.

## Impact

- **Code:**
  - `src/domain/scan_to_map.cpp`: the per-node NDT blocks exist already when the projection is on; the window information, the test and the attenuation go here.
  - Possibly a small helper in `src/domain/normal_equations.{h,cpp}` or a new `src/domain/degeneracy.{h,cpp}`, testable on its own.
  - Settings in `odometry_config.h`, `toml_io.h` and the shipped TOMLs, with the golden files regenerated in their own commit.
  - `run_odometry.cpp` for the diagnostics.
- **Tests:**
  - synthetic room: nothing attenuated;
  - synthetic corridor: translation along the axis attenuated, across it not;
  - a synthetic tunnel with a round cross-section: rotation about the axis attenuated;
  - noise added to the normals must not create a "constrained" verdict along the blind axis;
  - multithreaded equals single-threaded.
- **Results:** change on purpose when the new detector is switched on (new baselines). With it off, bit-identical.
- **Debug build:** Release only, unless a task adds configuration-dependent code (`CLAUDE.md` rule).
