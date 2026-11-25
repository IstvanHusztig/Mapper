# Residual-floor candidate rejected and removed

2026-11-13. User decision: "Reject and remove" after reviewing the W415
rotation correction of 9.142869 -> 8.727052 degrees. The small local response
does not justify retaining this feature as a fix or spending a full replay on it.

Scratch commit e79125c reverts 5be333f, 90dad3c and 4358c5f. The resulting entire
tracked source tree is identical to pre-floor commit c52b25f. Removed: floor
math/header, TOML setting/load/save/hash exception, option propagation, optimizer
integration/counters, floor tests/golden additions, and --floor6mm checkpoint
mode. Existing Huber/other pre-floor experiments are untouched; restoring the
tree does not adopt the historical Cauchy experiment or change shipped defaults.

Root tools/run_residual_floor_checkpoint.py and
tools/analyze_residual_floor_checkpoint.py were removed after verifying their
archived experiment copies match. Source history and original experimental
inputs, results, traces, binaries and analysis remain available in
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/open-closed-floor6mm-w415/`.
Retained binaries are evidence, not active recommended builds.

OpenSpec artifacts now identify this candidate as rejected and its numerical
requirements as withdrawn. Historical checkboxes describe what was done, not
current feature availability. Incomplete candidate validation/promotion is
cancelled, not marked passed. No delta specs were synced and the broader
open/closed-space problem is not declared complete. No new run started.

Verification: exact source-tree comparison against c52b25f passed. Release
dMapper, reconstruct_w754 and lidar_odometry_tests rebuilt successfully. Focused
golden TOML, configuration round-trip and snapshot checks: 8 cases / 51 assertions
passed. No floor identifiers remain in scratch src/tests/tools. Build and test
logs: out/floor-removal-build.log and out/floor-removal-tests.log. Debug/full
recording runs were not repeated for this exact source rollback; the earlier
known full-suite corridor failure remains documented, not declared fixed.

The user suggested synthetic trajectory fault injection, then correctly noted
that synthetic scene, contaminated-prior and state-propagation checks had already
been performed. Reuse those results; repeating them is not a new investigative
plan. They do not identify the first bad calculation in the real W904–916
sequence. That missing real incoming-state evidence remains the next unresolved
diagnostic gap. No follow-up capture or recording replay has been authorized.
