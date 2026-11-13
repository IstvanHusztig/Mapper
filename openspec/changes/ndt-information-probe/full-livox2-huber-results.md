# W754 Huber3 diagnostic — completed

2026-11-13. The user explicitly approved the exact-control plus Huber3 check
after preparation. Both completed; independent Codex source/test and output
reviews passed. The implementation and rationale are in
`full-livox2-robust-design.md`.

## Readings

| Measurement | Exact control | Huber3, all cells retained |
|---|---:|---:|
| Completed / accepted iterations | 100 / 100 | 100 / 100 |
| Estimated endpoint tilt change relative to initial pose/incoming gravity | 15.188597 deg | 2.148048 deg |
| Final endpoint velocity norm | 0.889912 m/s | 0.373934 m/s |
| Minimum projection scale, all iterations | 1 | 1 |
| Initial indoor matches | 16,428 | 16,428 |
| Initial outdoor matches | 14,263 | 14,263 |
| Final indoor matches | 13,501 | 16,666 |
| Final outdoor matches | 12,514 | 14,705 |
| Final rotated-IMU sample-mean Y acceleration | -1.468805 m/s² | +0.443396 m/s² |

All 15,400 control records reproduce the retained original trace exactly.
Initial match records for both grids are identical, including association and
geometry hashes. No cells or returns are explicitly removed; only residual-based
weights change. Later correspondences can differ because the poses change.

The same-sample rotated acceleration is `(R*a-incoming_gravity)*to_mps2`, without
offset subtraction. Full final XYZ means are control
`[0.022392, -1.468805, 0.004646]` and Huber3
`[-0.321903, 0.443396, 0.137268]` m/s². The left-time-weighted Huber values are
`[-0.320901, 0.446099, 0.132458]`. These describe estimated orientation context,
not independently measured physical acceleration or the complete IMU residual.

## Information and support

Initial average scalar weights are0.92235 indoors and0.93630 outdoors; final
values are0.94142 and0.95450. These averages alone conceal a larger reduction in
some information directions. For the identical initial poses, sorted eigenvalues
of summed translational cell information are:

| Grid | Original eigenvalues (millions) | Huber-weighted eigenvalues (millions) |
|---|---|---|
| Indoor | 2.353, 3.707, 4.974 | 1.687, 3.235, 4.700 |
| Outdoor | 0.434, 0.487, 1.400 | 0.253, 0.419, 1.181 |

The smallest eigenvalue falls about28% indoors and42% outdoors. Eigenvectors can
also change; these are sorted spectra, not ratios along a fixed common axis.
They describe a sum of cell-information matrices, not full trajectory
observability, calibrated pose covariance or independently retained geometry.
Both grids can match the same source return.

The candidate final raw reassociated matched-only cost is39,046.74; its robust
cost is21,976.91. The control raw cost is69,320.02. These are different losses
and/or support, omit other factors and unmatched penalties, and cannot be ranked
as a complete common objective or physical accuracy score.

## Interpretation and limits

A general residual-weighting rule, without cell-specific keys or deletion,
substantially reduces W754's estimated attitude excursion. It supports the
residual-influence mechanism identified by the previous three-cell ablation.
It does not prove the estimate is physically correct or that Huber is superior
to another robust loss. Do not rank its2.15-degree change against the ablation's
2.33 degrees as accuracy: there is no ground truth or perturbation envelope.

This is a frozen W754 test: preceding poses, map and priors come from the failed
run. A fresh-start run will change earlier mapping and states too. No later
window, cloud, route closure or robustness result was produced here. Full-cloud
improvement and elimination of the flyaway remain NOT MEASURED. The previously
exported full cloud remains unchanged.

Terra6.1's requested challenge is preserved in the design report. Huber's cutoff
uses map scatter and is not a calibrated sigma gate. Shared plane/cell uncertainty
and correlation, physical point-noise modelling and consistent objective
acceptance remain possible broader work; none is established as necessary by
this single result.

## Validation and provenance

Artifact root:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-trace/optimizer-w754-huber3/`

- `run.json`: exit0,136.93s including reconstruction, frozen files unchanged.
- `result/validation.json`: historical map/count/hash/contribution/history gates,
  all759 input hashes rechecked, control100 exact, Huber100, no ablation.
- `result/control-trace.log`, `result/huber3-trace.log`: original-format traces.
- `comparison.json`: all100 candidate iterations require cutoff3 and16 Huber
  support records each;200 range-quadratic conservation checks per condition,
  400 active raw-cost conservation checks and zero association mismatches.
- `w754-optimizer.json`, `bundle.json`, `bin/`: frozen inputs, sources, binary,
  libraries, build and test logs; archived analysis script/manifest accompany them.

Scratch commit `c4b3b6f` on `codex/phase0b-ndt-information`.
Executable SHA256:
`eb4eedf4a4e9678d0a05fdb8f183a16ff7bfab78b0550414620c7f16cac6f6d7`.
Source `scan_to_map.cpp` SHA256:
`ff78c0790896c601db1ccb33625c96557457d71b5e501e6f100083a91d1ae008`.

Release and Debug focused tests each passed3 cases/469 assertions, including
active range/Hessian/RHS, cost and weighted-information consistency. Full Release
reported127 cases,126 passed,1 unexpected failure,3 skipped;52,086 assertions,
four printed failures. All four printed values exactly match the retained
baseline: three belong to expected-failure coverage; the unexpected corridor
assertion remains0.0179039 <0.0075. This is not a fully passing suite, but there
are no new observed failures. Production default remains disabled; no shipped
settings or GUI controls changed.

Independent Codex review (not Sonnet) rechecked every control record, candidate
Huber coverage, identical initial associations, integrity, tilt, velocity and
same-sample IMU calculation. It emphasized stronger directional information
reduction despite high average weights and the full-route evidence boundary.

## Next decision

Proposed: one fresh full original Livox2 replay with Huber3, alpha0.1/beta0.1 and
other settings unchanged, monitored for onset relocation, later flyaway, coverage
and geometry. This needs a clearly identified full-run opt-in build/configuration
and separate user authorization; no full run is currently launched. The user has
been asked and subsequently authorized proceeding until the goal is achieved or
new information is needed. Full-run preparation is active; launch follows its
build, test, provenance and review gates. Keep High for interpretation/design; routine build
and run monitoring can use Medium. Candidate acceptance/push remain separate.
