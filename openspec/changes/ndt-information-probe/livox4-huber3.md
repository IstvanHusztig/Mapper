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

Status: running. Native logs and live trajectory diagnostics are monitored;
complete run-health and stage endpoint readings will be recorded after saving.
Diagnostic triggers alone are not map accuracy verdicts. Prior Livox3 runtime
was about 23 minutes; this longer scan may take longer. No completion claimed.
