# W754 control and three-cell ablation — completed checkpoint

2026-11-13. The user authorized these two isolated checks after switching to
High, then requested finishing this work and saving state for later continuation.
Both checks completed. The user subsequently said "Ok, then dont stop go on",
superseding the pause. Continue correction-design work from this checkpoint;
no additional optimizer/full replay has started.

## Result

| Measurement | Exact control | Three outdoor cells omitted |
|---|---:|---:|
| Completed / accepted iterations | 100 / 100 | 100 / 100 |
| Endpoint tilt change from initial pose, relative to incoming gravity | 15.188597 deg | 2.325955 deg |
| Final endpoint velocity norm | 0.889912 m/s | 0.355548 m/s |
| Minimum projection scale across all iterations | 1 | 1 |
| Initial indoor matched terms | 16,428 | 16,428 |
| Initial outdoor matched terms | 14,263 | 14,252 |
| Final indoor matched terms | 13,501 | 16,700 |
| Final outdoor matched terms | 12,514 | 14,709 |

All 15,400 control trace records (154 per iteration) reproduce the retained full
Livox2 trace exactly. This includes state, solver decisions, factors, associations
and diagnostics. The intervention starts from identical state and removes only
outdoor cells `77310918653`, `73015885821` and `81605885949`.

Only eleven outdoor match terms disappear at the identical initial poses. Indoor
initial counts, costs and association/geometry hashes are unchanged. Later
associations differ because the optimized poses differ. Each grid sees the same
23,470 source returns; the two grids can use the same return, so their counts
must not be read as unique physical points.

Same raw IMU samples, expressed through the estimated poses, give sample-mean
`(R*a - incoming_gravity)*to_mps2`:

| State | X | Y | Z (m/s²) |
|---|---:|---:|---:|
| Common initial poses | -0.327713 | 0.170025 | 0.145355 |
| Control final poses | 0.022392 | -1.468805 | 0.004646 |
| Ablation final poses | -0.305020 | 0.495626 | 0.135838 |

The independent left-time-weighted calculation agrees with this contrast:
final Y is -1.458559 versus +0.498304 m/s². These are rotated measurements under
estimated poses, without acceleration-offset subtraction, not the complete
optimizer residuals or independent physical acceleration.

## What this establishes

Omitting these three cells substantially reduces this window's estimated tilt
excursion under otherwise identical frozen inputs. This is a causal intervention
on their collective influence, strengthening the earlier per-cell attribution.
The influence is not explained by a projection-scale change in this window.

It does not establish that the surfaces are physically wrong or reflections,
which individual cell is necessary, or that hardcoded cell removal is a general
correction. There is still a 2.33-degree estimated attitude change, and the map
and incoming priors already contain earlier trajectory history. Physical attitude
error is NOT MEASURED. No downstream window or full cloud was generated, so
elimination of the full-route flyaway, map accuracy and robustness are NOT
MEASURED. The retained final point cloud remains the same known failed result.

Final reassociated matched-only NDT costs total 69,320.02 for control and
29,980.09 for ablation. These exclude other factors and unmatched penalties and
use different support; they are not a common complete objective or accuracy score.
Standard whole-run closure/map checks do not apply to this isolated checkpoint.

## Execution and provenance

Artifact directory:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-trace/optimizer-w754-control-ablation/`

- `run.json`, `run.log`, `bundle.json`: one invocation, exit0, 140.38 seconds
  including reconstruction; all frozen files unchanged.
- `result/validation.json`: exact historical map/source/association gates,
  numerical contribution and cell-history gates, 759 input hashes rechecked
  after analysis, control100 and ablation100.
- `result/control-exact.json`, `result/control-trace.log`,
  `result/three-outdoor-cell-ablation-trace.log`: full evidence.
- `comparison.json`: per-iteration states, decisions, support and local joint-step
  quadratics; produced with `tools/compare_window_traces.py`.
- `acceleration-context.json`: same-sample means and initial support.
- `bin/`: frozen executable, sources, extractor, DLLs and successful build logs.
- `w754-optimizer.json`: exact input state/prior matrices and original records.

Scratch commit `7f99b292a20db63e1b067c1f635d7af9901caff2` on
`codex/phase0b-ndt-information`, checkout `C:/Repo/dMapper/out/phase0b-worktree`.
No core algorithm or shipped-setting change. Release and Debug diagnostic-target
builds passed; production mapper tests were not rerun for this standalone helper.
The Python comparison was checked against retained evidence, reproducing the
previously measured control tilt and speed before use on the intervention.

Release executable SHA256:
`970a93c9ddcbb8aa081ad199ba0b8d2c15a6b4be9aa37b3c302fad3a2ac16af2`.
Driver source SHA256:
`9d51fa0d1d59265d38f6ebba356d61695bf1c1ed40304c9fd5bb47d9c64c43fe`.
Original full trace SHA256:
`37e3b3decf92cf9ed2a14bf2392b0cb5e9ad4edf7121760df9806d2cd2b83e61`.

The driver uses the production optimizer and LM schedule, including convergence
checks. It restores a fresh workspace and auxiliary state per condition. The
offline driver does not use the wall-clock cutoff; both conditions reached the
same100 iterations. Any exact-control mismatch would have blocked the ablation.
Neither original recording/cache nor previous experiment artifacts were changed.

Independent Codex review (not Sonnet) checked the driver/extractor/launcher before
execution, independently rechecked every control record and recomputed final tilt
and speed afterward. Review corrected misleading optional-optimizer metadata,
required post-analysis input checks and kept the conclusion limited to W754.

## Resume here

The current authorized checks are complete. Keep all artifacts and commits.
The user's subsequent instruction resumes substantive work; it does not itself
specify a new experiment matrix. No additional run should start merely because
this report is opened.

Current substantive task: use the now-confirmed cell
influence to choose a general treatment of these associations/information weights
and specify its validation. Do not ship or generalize the three hardcoded cell
IDs as a filter. High remains useful for that correction-design decision; the
finished execution and saved checkpoint require no additional analysis.
Any new optimizer/full-recording run needs the user's next authorization.

The broader roadmap remains phase0b diagnosis, before selecting and validating a
general correction. Full-route, perturbation, map-quality and user visual gates
are still outstanding. No candidate acceptance, push or OpenSpec archive.
