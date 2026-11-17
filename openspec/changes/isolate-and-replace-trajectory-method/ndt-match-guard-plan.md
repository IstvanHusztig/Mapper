# Local detour: validated, fixed NDT correspondences

User authorized implementation after the W298 cell audit. This detour preserves
Charlie at 3/19 and does not claim the larger method interface is implemented.
Independent design review: trace_review (Codex, not Sonnet).

Implement in the existing experimental worktree, starting at b892823. Keep the
shipped tree and legacy behavior unchanged. Add a default-off TOML option shared
by CLI and GUI. On each window, after prediction and before LM, admit cells only
with at least 6 points, finite geometry, symmetric positive-definite covariance
and information, usable inverse, and initial model-relative residual magnitude
at most 6. Preserve the existing range/viewpoint filters. These provisional
thresholds are not calibrated sensor sigmas. Volumetric cells and zero normals
are allowed when supported; this is not a wall/planarity requirement.

Freeze admitted source/target identities and target statistics for that window.
Recompute residuals, Jacobians and Huber weights on that fixed set. Excluded
sources cannot enter later; admitted sources cannot disappear at voxel/viewpoint
boundaries. Assembly, costs and diagnostics must use the same set. Recreate it
for every window. Require at least 6 unique source returns across grids, without
double-counting two-grid matches; this is a fail-closed support guard, not proof
of observability. Unsupported windows fail before optimizer/state/map updates,
with no silent inertial fallback or retry loop.

Disabled mode preserves the legacy arithmetic and historical snapshot hash.
Enabled mode includes guard settings in snapshot identity. Reject unsupported
robust preprocessing rather than silently bypassing the guard. No acceptance
rule redesign, velocity freeze or new covariance model is bundled.

Existing W298 data: provisional admission retains 10,751 unique source returns,
including 312 fine and 486 coarse 2–5 m terms. This is coverage evidence only.
Frozen matches reduce capture range when prediction is wrong and can retain
initially plausible wrong matches. Trajectory improvement is NOT MEASURED.

## Implementation checks

- [x] Cell validity, initial compatibility and fixed-set lifecycle implemented.
- [x] Shared settings, snapshot identity, fail-closed orchestration and diagnostics.
- [x] Focused synthetic tests, Release/Debug verification and independent review.
- [x] Candidate executable/settings prepared; exact provenance and commands saved.

Completed preparation: see `ndt-match-guard-implementation.md`. Candidate commits
426b8ed / 1268e5a. No recording replay or adoption is implied by these checks.

No new recording or captured-window optimizer replay is authorized. Prepare the
smallest faithful prefix comparison, preserving actual alpha1/beta1/Huber3 of
the W298 reference (the older alpha0.1 control was not gain-only). Obtain approval
before launch. Map quality, coverage, indoor preservation and perturbation gates
remain required before adoption. No push/default change is authorized.

## Authorized iteration refresh (implementation pending verification)

The user authorized refreshing valid matches between iterations while retaining
cell validity guards. This supersedes whole-window freezing in the experimental
candidate only. Implementation and validation are tracked in
`ndt-iteration-refresh.md`; no recording replay is authorized by this instruction.
