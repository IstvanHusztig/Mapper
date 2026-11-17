---
name: verify-robustness
description: Measure the noise floor of a dMapper settings file on a recording by running it with small, physically meaningless changes and reporting how far the results move. Use this before any setting or numeric code change is called better or worse, when two candidates must be ranked, when a result looks fragile (tiny changes move the map by metres), and at the gate of each phase of a retune, even if the user only asks "is this setting really better?" or "why does the result change between runs?".
---

# Verify robustness

## Agreed evaluation rules (2026-11-12)

Read `openspec/changes/sensor-neutral-yardstick/criteria-proposal.md` before any
verdict. The user agreed this contract; it supersedes legacy tool defaults.
Report missing evidence as NOT MEASURED. Diagnostic triggers are not map-accuracy
verdicts. Overlapping empirical envelopes are inconclusive, not proof of equality
or non-regression. Preserve every perturbation member and check odometry and
final results separately. The user's review of readings and clouds remains required.


One run cannot rank two settings: on these recordings a change with no physical meaning (5 % on an averaging span,
1 mm on a voxel size) has moved results by metres. So every verdict needs the spread such changes cause, and a
setting whose perturbed runs disagree by metres is fragile whatever its best run looks like.

## Procedure and limits

State run cost first, check disk/process/cache ownership, and use verified inputs.
Use `python tools/perturbation_sweep.py <input> <settings> <fresh-output> --profile short`.
Full routes require --profile full --reference-route-m and --reference-id, frozen
before candidates. SHA256 identity covers input/config/binaries/tools/evaluation
and output artifacts; resume refuses missing/changed/failed evidence. All clouds
are preserved. Unknown outputs are never overwritten; use a fresh directory.
The sweep checks snapshots against logged windows and compares odometry/final
stages separately. Exit2 is incomplete/invalid, exit1 diagnostic triggers or
stability violations, exit0 complete diagnostic/stability evidence, not accuracy.
Do not delete existing results without permission. Synthetic seven-member/resume
tests pass; state run cost and preserve existing cache ownership before collection.

Use a fixed family of base plus six perturbations: averaging span +/-5%,
translation-edge sigma +/-10%, decimation +/-1 mm. Check configuration meaning
and timestamps; do not silently substitute a historical family. Capture input,
settings and executable identity and completion for every run; never reuse by
folder name alone. Missing/failed members remain visible and block a full verdict.

For selected short inputs (Livox2 0-60 s, Livox1, 44_short), maximum displacement
from base must be <=0.10 m. For full routes it must be <=0.3 m per 100 m of a
frozen reference route length. Report absolute max, p95 and time above 0.05 m;
check odometry and final stages separately. All valid members must meet targets.
Compare metric envelopes under the agreed contract; overlapping ranges are
inconclusive, and do not establish shipment non-regression. Recording44 retains
its s0/s1/s2 map-quality gate. Tool aggregate PASS is never sufficient.

## Report

The table as written, the spread line, and which runs fail which checks. Then stop and ask the user how to proceed.
