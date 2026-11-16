# Design

## Context

The full design, with the function as it stood after stage 0 (line ranges, what each part reads), is the section "Stage
2" of `docs/code-split-and-region-agents-plan.md` (f326a1e). A Sonnet reviewer checked it against the code. This file
keeps the decisions.

## Goals / Non-Goals

**Goals:**
- One function per term, each with a signature that shows what it reads.
- Every output bit-identical.
- A test per term.

**Non-Goals:**
- No change of numbers.
- No per-term log yet (stage 3).
- No solver unit or verdict yet (stage 4).
- No weights in one place yet (stage 5).

## Decisions

1. **The terms are moved, never reordered.** Every entry of the matrix and the right-hand side must receive the same
   additions in the same order. The parts are strictly sequential, except the gyro-bias coupling, which sits inside the
   edge loop and stays there.
2. **Rotation and position edges stay one function.** One generated call builds each edge's 12×12 block from all six
   weights, so two functions would round differently.
3. **One running motion cost.** It is added to by the edges, the bias prior, the velocity continuity and each
   accelerometer part, in that order. Per-term sums added afterwards would round differently, and with `lm_step_check`
   that can flip a step decision. The edges add their cost only when it is wanted (`with_cost`), as before.
4. **Narrow signatures, no "everything" struct.** A term takes the window, the layout, its own options, the equations,
   and the acceleration offset where it reads it.
5. **The solve no longer reads NDT internals** (since stage 0). The NDT part hands on only `NdtSums`: the node blocks for
   the projection, the normal information, and the cost counts.
6. **One NDT point term with a per-grid policy:** cells, cell size, nearest range, and what only the indoor grid does
   (range weight, cap at 10, normal information). For the outdoor grid the weight is planarity × 1.0, which is
   planarity exactly. The merge was the last commit, kept because the guard stayed identical.
7. **A characterization guard for the branches no run reaches.** The replays and baselines exercise only the shipped
   settings. The guard covers:
   - the cost, planarity and range weights;
   - no projection, no outdoor grid or view-point test;
   - constant rotation sigmas, no bias estimation, no timestamps;
   - the translation information, Huber and the scales, the motion-model correction;
   - the velocity continuity, and the accelerometer at rest and with IMU gaps;
   - the bias clamp, a big jump, the cost-only call.

   The file is not committed: the bits may differ between compilers and configurations.
8. **The test per term is the same check for every term with a cost:** the right-hand side is minus half the numeric
   gradient of the cost; for linear terms the matrix diagonal is half the second derivative. Special cases:
   - the rotation part of the accelerometer Jacobian is first order;
   - the weak end tie treats the start velocity as fixed;
   - the projection and the origin fix have their own checks;
   - the NDT check uses a step too small to move a point to another cell.

## Risks / Trade-offs

- **A refactor that changes a bit only in an untested branch** would pass the replays and the baselines. The guard closes
  this gap for every branch of the function.
- **The guard is bound to one compiler and configuration.** It is recorded and compared on the same Release build. It
  is a development tool, not a regression test of values.
