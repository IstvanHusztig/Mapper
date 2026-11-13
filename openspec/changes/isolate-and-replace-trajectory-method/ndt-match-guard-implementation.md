# NDT match guard candidate: implemented and prepared, not replayed

## Scope and identity

User authorized fixing the local cell-validity/correspondence-consistency
weakness. The reviewed design is `ndt-match-guard-plan.md`; evidence is
`livox4-ndt-cell-audit.md`. This is an experimental detour, not adoption or the
larger trajectory-method extraction. Charlie remains 3/19.

Implementation: `C:/Repo/dMapper/out/ahrs-gain-worktree`, branch
`codex/livox4-ahrs-gain-only`, commit **426b8ed**. Serialization fixtures are a
separate explicit commit **1268e5a**. The historical branch name does not imply
that the earlier comparison was gain-only. No push. Shipped root code/settings
remain unchanged.

Candidate bundle:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ndt-match-guard/`

`manifest.json` says `prepared_not_run`, approval false, and hashes 37 artifacts,
including inputs, reference snapshot/log, candidate executable/settings/patch,
runtime DLLs and checks. Executable SHA256:
`43aae373c4f5166eee87776b8c486ac03d688e1d48112e6e4007aafbb24cd114`.

## Implemented behavior

New domain component `ndt_match_guard.{h,cpp}` owns immutable copies of admitted
targets. Admission checks occupancy, finite geometry, symmetric positive-definite
covariance/information, usable and consistent inverse, existing range/viewpoint
rules and initial model-relative Mahalanobis distance. It does not require a
plane or nonzero normal. Source identity and pose count are checked before each
assembly/cost call. Invalid inputs and unrepresentable grid coordinates fail.

`run_lm_iterations` constructs the set afresh after prediction, before any LM
iteration. Assembly, Huber costs, traces and heading/weak-axis diagnostics share
it. Target statistics and source membership stay fixed through the window;
residuals, Jacobians and Huber weights are reevaluated. Crossing a voxel or
viewpoint boundary cannot change membership. Fewer than six unique source
returns across grids throws with window/support context before optimization;
there is no successful inertial-only fallback or repeated false-return retry.
This is a minimal support rule, not an observability test. Predictions already
made in the failed window are not claimed to be rolled back; no optimizer result
or subsequent map/carry update is committed after this admission failure.

Shared CLI/GUI TOML settings, under `[lidar_odometry_control]`:

```toml
ndt_match_guard_enabled = true
ndt_match_guard_min_cell_points = 6
ndt_match_guard_max_mahalanobis = 6.0
```

Missing/false enable selects unchanged legacy behavior. Active malformed
thresholds, norm/planarity weighting and alternate robust preprocessing are
rejected. Disabled guard fields are omitted from snapshot identity to preserve
historical legacy hashes; enabled settings distinguish candidate snapshots.
The three thresholds (including fixed minimum six source returns) are provisional.
Mahalanobis magnitude describes the map model, not calibrated sensor sigma.

## Verification

- Release and Debug new tests: **7 cases / 206 assertions each**, all pass.
  Covers sparse/NaN/Inf/asymmetric/indefinite/stale-inverse cells, volumetric cells,
  threshold boundary, target ownership after map removal, fixed membership across
  voxel/viewpoint changes, stale sources, fresh-window preparation, unique-source
  support, settings/identity and serial/parallel equality. Six-axis finite
  differences agree with the assembled NDT gradient; trace costs/membership
  agree with assembly.
- Eight frozen legacy assembly/cost fixtures, captured before changing the build,
  are **byte-identical** afterwards (both grids, projection, inertial variants).
  These are synthetic equivalence checks, not recording equivalence.
- Full Release CTest passes: **137 cases accepted, 3 skipped**. Of 78,723
  assertions, 3 are the existing expected failures in the two consistency truth
  tests (6.270 mm; 10.718 mm / 0.350 degrees). They remain explicitly marked
  `should_fail`; they are not repaired or counted as new passing assertions.
- Three TOML goldens deliberately add only the disabled guard and its thresholds.
- Independent Codex review (`trace_review`, not Sonnet): no blocking defect.
  Its request for actionable unsupported-window context was incorporated.
- OpenSpec strict validation passes. Debug was included because of target
  ownership/lifetime and parallel reads. An initial new test incorrectly expected
  the existing assembly-only probe to solve; corrected the test to assemble and
  solve separately, then reran successfully. No algorithm workaround was made.

## Next run: prepared, requires approval

One full-context **125-second Livox4 prefix**, estimated **10–20 minutes**. Reuse
`livox4-ahrs-gain-prefix/input` without copying the recording. Compare with the
existing `livox4-ahrs-gain-trace/result`, actual **alpha1 / beta1 / Huber3**. The
candidate TOML differs only by the three guard fields; reference runtime DLLs
are hash-verified and preserved. Sanitize inherited `DMAPPER_*` variables, then
set `DMAPPER_DIAGNOSTIC_NDT_HUBER3=1` and `DMAPPER_TRACE_LIVOX4=1`. The manifest
contains the exact executable/input/settings/output command. No replay launched.

Before launch recheck hashes, unused result/log paths, free disk and cache/process
ownership. Do not use the diagnostic runner's exact-output equality gate for a
numerical candidate. Check support/failure logs, run-health, W297–300 first-step
and cumulative rotation, match identity/turnover, retained coverage and the cloud.
The guard operates from the beginning, so changed W298 inputs are expected;
this is not an isolated captured-state causal replay. Stop on failure, no retry.

Trajectory improvement, map quality, indoor preservation, full-route behavior
and perturbation robustness are **NOT MEASURED**. Frozen matches reduce capture
range when prediction is already wrong and can retain initially plausible wrong
associations. The physical origin of the old cell mismatch remains unresolved.
No general flyaway cure, default change or HTML adoption update is claimed.
Medium effort suffices for routine monitoring; numerical interpretation needs high.
