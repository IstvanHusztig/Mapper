# Full Livox4 Huber3 replay

2026-11-13. User explicitly authorized full Livox4 processing with Huber3,
trajectory-jump monitoring and notification when the cloud is ready. Routine
execution is suitable at medium effort; request higher effort before deeper
causal analysis if diagnostics require it.

Input: C:/Users/User/Downloads/dLidar/raw data/livox/4 (original files remain in
place). Start/end: same Wohnen table, slightly quicker walking, indoor/outdoor
route. Quick input checks are in livox4-input-check.md; all five source hashes
are pinned in out/livox4-recording-check.json.

Output: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-indoor-outdoor/huber3-baseline/.
Reuses frozen full-livox2-huber3 binary/runtime and identical Livox-vertical
settings used for the crisp Livox3 baseline: alpha/beta 0.1, Huber cutoff 3,
online gyro bias enabled, diagnostics and snapshot enabled. Cauchy, detailed
trace and rejected residual floor are not activated. No algorithm/settings
change or build is part of this replay.

Runner: out/livox4_huber_replay.py, adapted from the completed Livox3 runner.
It verifies raw/bundle/settings hashes before launch, refuses existing output,
records approval/build/configuration identities, and rechecks input/frozen
identities on completion. No automatic native retry. First setup attempt found
a missing parent directory before any mapper process/output was created; mkdir
was corrected, then the single authorized native replay started (PID 32748).

## Completed collection

Native exit 0; collected after 1798.64 s (about 30 minutes). Source/frozen hashes
rechecked; artifact_integrity_complete=true. Cloud:
result/scan_lio_merged.laz, 305,742,899 bytes. Run.json and output hashes retain
the definitive identity. No retry or extra run was performed.

All 1,016 windows completed; 182,880 poses in both stages. Actual first/last
pose separation: odometry 36.5421 m, final 36.7170 m. Vertical displacement:
odometry -2.2930 m, final -1.7239 m. User reports return to the same Wohnen table;
these are estimated endpoint separations, not independently surveyed error
measurements. They indicate an unresolved substantial return mismatch.

| Diagnostic | Reading |
|---|---:|
| Bend/speed/bias rows | 1016/1016 each |
| Bends above 3 deg / 10 deg | 11 / 1 |
| Largest bend | -11.704 deg, W397, 142.56 s |
| Speed crossings above 2.5 m/s | 8 windows |
| Largest speed | 5.08615 m/s, W320, 114.84 s |
| Residual bias maximum absolute XYZ | 0.21 / 0.32 / 0.64 deg/s |
| Selected velocity quadratic median / p95 | 86 / 675 over 520 rows |
| Solver failures / big-jump nodes | 0 / 0 |
| Loop closures / largest correction | 51 / 1.63 m |

run-health exit 2: diagnostics_complete=false because conditional velocity
uncertainty eligibility is unresolved/missing at windows 95 and 736. This is
retained, not silently converted to a pass. Completed native output and its
hash integrity are separate from this diagnostic gap. Accuracy not accepted;
map sharpness/visit consistency/physical-region comparisons NOT MEASURED here.
The full cloud is ready for the user's visual review.

The user requested a focused code analysis while a lower-cost monitor watched
the replay. See livox4-code-audit.md: independent review confirmed prediction
dominates displacement escalation at W301–320, the translation-only acceptance
guard permits the observed angular corrections, and W321's speed drop is an
explicit prior reset. This is a narrowed investigation, not a proven full cause
or an implemented fix. No algorithm/settings change was made.
