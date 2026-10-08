# Design

## Context

`optimize_lidar_odometry` (`src/domain/scan_to_map.cpp`) builds the normal equations of one LM iteration in a dense `Eigen::MatrixXd AtPAndt(n_params, n_params)` (plus a dense vector `AtPBndt`). The unknowns are `[pose_0 … pose_n-1 | gyro bias (3) | acceleration offset (3)]`: 6 per node, plus up to 6 extra. For a 360-node window `n_params` is 2166, so the matrix is 37 MB.

Who writes it (all with `+=` on single elements or 6×6 / 3×3 blocks):
- the NDT point loops: the diagonal 6×6 block `(6k, 6k)` of the point's node;
- the motion-model edges between consecutive nodes: the four 6×6 blocks of nodes `k-1`, `k`, so up to 11 columns from the diagonal;
- the gyro-bias coupling: elements between a node's rows and the bias columns, both triangles;
- the velocity prior, a few elements between nodes at most 2 apart;
- the accelerometer term: elements between nodes of one term (`A`, `B`, `C`, anchors about `imu_acceleration_span_s` apart) and the offset columns, both triangles, and the offset prior.

Who reads it: only `pose_band_to_sparse(AtPAndt, n_pose_params, pose_band_offset)`, which copies the nonzero elements with `|row − col| ≤ pose_band_offset` in the pose block, and all nonzero elements of the extra rows and columns, into a sparse matrix. `pose_band_offset` is 17, raised by the accelerometer term to `(last node − first node) × 6 + 5` of its widest term. Anything outside that band is never read.

The fix-origin constraint (6 diagonal elements, weight `1/σ²`) goes a different way: through `matA`, `matP`, `matB` and two sparse products into `AtPA`, which then gets `+= pose_band_to_sparse(...)`.

Measured on the full 44 after `deterministic-multithread` (`docs/PERFORMANCE.md`): setup 4.09 ms, rest-of-iteration 1.71 ms, assembly 1.30 ms per iteration; the covariance solve (1.78 ms) and the damped solve (2.06 ms) are 25 % together.

With the default settings the accelerometer term is on but gated to degenerate windows, and `imu_acceleration_span_s = 0.01`, so its anchors are about 9 nodes apart. A term then spans about 18 nodes, a band of about 113 columns; windows without the term have 17.

## Goals / Non-Goals

**Goals:**
- Remove the dense matrix without changing a single bit of the result.
- Cut the three phases above to a small fraction.

**Non-Goals:**
- Changing the maths, the LM damping, or the solver.
- A banded Cholesky or another solver. The solves use `SimplicialCholesky` on a sparse matrix as today.
- Making the accelerometer-term band narrower.

## Decisions

1. **`BandedNormalMatrix`, one type for the whole matrix.** In `normal_equations.{h,cpp}`:
   - constructed with `n_pose_params`, `n_extra` and the half-width `band` of the pose part;
   - pose–pose elements `(r, c)` with `|r − c| ≤ band` in a row-major array of `n_pose_params × (2·band + 1)` doubles;
   - extra columns of pose rows `(r, n_pose + j)` in an `n_pose_params × n_extra` array, the transposed part `(n_pose + j, c)` in an `n_extra × n_pose_params` array (the code writes both triangles separately, so both are stored, not mirrored), and the extra–extra block as a small dense `n_extra × n_extra`;
   - `add(r, c, v)` for single elements, `add_block6(c, M)` for the NDT point loops' diagonal block (always in band), and a read accessor for tests;
   - the pose–pose write outside the band is dropped, as today (it was never read), and `assert`s in Debug builds, because it would mean a wrong band.
   - Storage is zero-initialized by the constructor, so the first `+=` sees the same `0.0` as the dense matrix.

2. **The same additions in the same order.** The type has no summation of its own: each element gets exactly the sequence of `+=` the dense element got, because the call sites are unchanged apart from the type. With the parallel NDT loops, each node's diagonal block still has one writer (`deterministic-multithread`). The result per element is therefore the same bit pattern.

3. **The band is known before the loops run.** It depends only on `window.timestamps` and `translation_imu_options` (anchors, from `span_s`), not on poses. The anchor list and the band are computed once, at the top, by a small function used for both the allocation and the accelerometer term (which today computes the same list further down). The term's `max(...)` update becomes an `assert` that its span fits the band.

4. **Sparse conversion equal to today's.** `BandedNormalMatrix::to_sparse()` emits the same triplets as `pose_band_to_sparse` (nonzero elements only, same ranges) and builds the matrix the same way, so the sparsity pattern, and with it the ordering of the Cholesky factorization, are unchanged. This is what keeps the solve bit-identical.

5. **The fix-origin term is added to the band.** `AtPA` today holds `1/σ²` on 6 diagonal elements from the sparse product (`1 · p · 1`, exact) and then gets `+= band matrix`, one IEEE addition of two values; addition is commutative, so adding `p` to the stored element after all other terms gives the same value. The three small sparse products and their triplet lists go. `AtPB` is `AtPBndt.sparseView()` as before (the fix-origin right-hand side is zero).

6. **`pose_band_to_sparse` becomes a test helper.** It is moved to `tests/normal_equations_test.cpp` as the reference the new type is compared with; production code no longer uses it.

7. **Cholesky symbolic reuse is decided by a measurement.** `SimplicialCholesky` redoes the ordering (AMD) in each of its two `compute` calls per iteration. Reusing it needs state across iterations, so a `ScanToMapWorkspace` (holding two solvers) would be passed in by `run_lm_iterations`, created once per window. Reuse is safe only if the pattern is the same: the workspace stores the pattern's `outerIndexPtr` and `innerIndexPtr` and compares them on every call, calling `analyzePattern` again when they differ. With an equal pattern `analyzePattern` would produce the same permutation, so the result is bit-identical by construction. Task 3 first measures what the two solves cost after the band store and what share is the symbolic part (a timed `analyzePattern` call in a scratch build); it goes ahead only if the saving is at least about 5 % of the odometry. If not, it is dropped and documented as measured.

8. **Not in scope here but noted:** the update step collects `x` with an `InnerIterator` and requires `h_x.size() == n_params`. If a solution entry were exactly `0.0` the update would be skipped for the whole iteration. It has not happened in any run and this change leaves it as is.

## Risks / Trade-offs

- **A call site writes outside the band.** It would be dropped silently, exactly as today (`pose_band_to_sparse` never read it). The Debug assert and the Debug `ctest` (which runs the synthetic scan-to-map tests) are the check; this is why the Debug build runs for this change.
- **`/fp:fast` code generation** of the changed function can differ, even though the arithmetic is the same. The CLAUDE.md rule applies: precise identical, fast identical or within 1e-6, and new baselines if the fast build moves.
- **Memory for the accelerometer term:** a band of 113 is 2160 × 227 doubles, about 4 MB; the dense matrix was 37 MB. A very large `imu_acceleration_span_s` grows it linearly; the dense case is its limit.
- **Hot-path cost:** the NDT loops add 6×6 blocks to the diagonal through `add_block6`, the same 36 additions to contiguous rows of 35 doubles; no slower than the strided dense block.

## Measurement

With `log_stage_timing` on (scratchpad copy of `lidar-vertical.toml`) on `44_short` and the full 44, against the numbers above: setup, rest-of-iteration, assembly, odometry and whole run.
