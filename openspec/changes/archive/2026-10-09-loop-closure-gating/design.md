# Design

## Context

`run_loop_closure` (`src/application/loop_closure.cpp`) groups the windows into segments of 12 windows (about 5 s), picks for each segment one earlier partner (the nearest centre within 8 m, at least 30 s earlier), registers it (coarse voxel search, then NDT), and accepts it by overlap, shift and angle. A pose graph over the segment poses (odometry edges 0.1 m / 0.5°, loop edges 0.05 m / 0.3°) is solved once, and the correction is interpolated onto the nodes.

Two failures on the corrected 44 with `imu_always_on` (see the proposal) leave ghost walls of 50–60 cm: a missed loop (only one partner tried) and a false loop accepted (no check after the graph).

## Goals / Non-Goals

**Goals:** no false or missed loop decides a ghost wall in the three perturbed 44 runs and 45f; behaviour unchanged where every first registration is accepted and every edge agrees.

**Non-Goals:** a new registration method, robust kernels, switchable constraints, or loop edges between more than one pair per segment. The edge sigmas stay.

## Decisions

### 1. Try the next partner only after a rejection
The nearest centre is a cheap proxy for "same view" and usually right. A short last segment, or one that sees a room from its doorway, can have its nearest centre in a segment that saw a different part. Trying the next nearest after a rejection fixes that at the cost of extra registrations only where the first failed. Up to 3, because the target already includes the partner's two neighbours.

### 2. Drop the worst edge while it disagrees, then solve again
After the solve, each loop edge's residual is (initial_i⁻¹ · correction · initial_j)⁻¹ · (optimized_i⁻¹ · optimized_j): how far the solution is from what the registration measured. A false edge spreads its error over the good edges too, so a fixed threshold on every edge at once would also drop good ones; the unit test (three loops, one 1 m off) gives 0.16 m for the false edge and 0.08 m for the good ones. The false edge always has the largest residual, though, and after it is dropped the good ones fall to about 1 cm. So only the worst edge is dropped per round, and the graph is solved again. The graph has about 58 poses; a solve takes milliseconds.

Thresholds 0.15 m and 2°: three loop-edge sigmas (0.05 m), and well below the 0.5–1 m errors of the false matches seen. The residuals of the kept edges are logged, so the limit can be checked on new recordings.

### 3. Alternatives not taken
- **Robust kernel (Cauchy/Huber) on the loop edges:** needs a scale as well and keeps a false edge's pull, only weaker; dropping makes the log say exactly which loop was refused.
- **Comparing neighbouring loop corrections before the graph:** catches the 49-against-6 case, but not a false edge without neighbours, and duplicates what the graph residual tells.
- **Looser loop-edge sigma:** lets every good loop correct less.

## Risks / Trade-offs

- **A true loop that disagrees with many false ones would be dropped.** Unlikely, since false matches are rare and scattered; the log shows it.
- **Dropping changes which loops are used, so the run-to-run chaos of 44 changes too.** It is judged by three perturbed runs plus 45f, not by one run.
- **Runtime:** a rejected registration now costs up to three registrations. Loop closure took about 54 s on 44.
