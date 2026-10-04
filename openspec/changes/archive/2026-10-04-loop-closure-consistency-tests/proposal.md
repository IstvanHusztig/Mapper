# Proposal

## Why

Roadmap step 8. The goals in `docs/ARCHITECTURE.md` ask that every algorithm can run in a unit test on synthetic data. The motion model, scan-to-map, gyro bias and the NDT map now have such tests. Loop closure and the consistency pass don't, and both run on every CLI and GUI run with `settings/lidar-vertical.toml`.
- **Loop closure** (`run_loop_closure`, `src/application/loop_closure.cpp`) builds segments, finds revisits, registers them (coarse search plus NDT refinement), solves a pose graph and spreads the correction over the nodes. Today it is checked only through the full 44 run, which takes 30 minutes and only shows its end result.
- **The consistency pass** (`run_consistency` with `Consistency` / `Consistency2`, `src/application/consistency.cpp`) re-optimizes the whole trajectory against an NDT map of all its points. It has the same gap.

The 6c scan-to-map test found a real defect (the NDT cell normals) within its first run. These two algorithms have more code and fewer checks, and the next roadmap steps (ports for Fusion and TBB, splitting `core`) will touch them. They need a safety net that runs in seconds.

## What Changes

- **A shared synthetic recording for the tests:**
  - a closed scene with enough structure to register in every direction;
  - a true trajectory that drives two laps of a loop through it;
  - windows with nodes, timestamps and their points as the scanner sees them (in the node frame, with noise), kept in an in-memory `PointStore`.

  The tests set every option themselves, as 6c does, so tuning the shipped settings files doesn't change them.
- **Loop-closure tests:**
  - **drift on the second lap:** a revisit is accepted, and the drift at the end is reduced against the truth;
  - **a trajectory that is already right:** it stays within a few millimetres;
  - **no revisit:** the call returns 0 and leaves the trajectory untouched.
- **Consistency tests**, for both `Consistency` and `Consistency2`:
  - **windows with small rigid offsets** (window seams, as odometry leaves them): the pass moves the poses closer to the truth and makes the map sharper (fewer occupied 3 cm cells);
  - **a trajectory that is already right:** it stays within a few millimetres.
- **Measured, not assumed.** Each limit is set after a first measurement, with margin, and the measured numbers are written next to it, as in 6c. If an algorithm misses an expectation, the case is marked as an expected failure that names the cause and reported, the way the corridor case was. The code under test is not changed in this change.
- **The test target copies its runtime DLLs**, as `dMapper` does. `run_consistency` lives in a file that also uses TBB, so the test executable needs the TBB DLL next to it.

## Capabilities

### New Capabilities

None.

### Modified Capabilities

- `unit-tests`: two new requirements, "Loop closure tested on synthetic data" and "Consistency pass tested on synthetic data".

## Impact

- **Code:** new files in `tests/` (the synthetic recording, the in-memory point store, `loop_closure_test.cpp`, `consistency_test.cpp`) and `tests/CMakeLists.txt`. No change to `src/` or `core/`, so the results and baselines stay as they are.
- **Docs:** the Testing section of `CLAUDE.md`, and step 8 in `docs/ARCHITECTURE.md`. Any finding is described where the algorithm is documented (`docs/index.html`).
- **Run time:** the new tests should take seconds in Release. The Debug time is measured and reported.
