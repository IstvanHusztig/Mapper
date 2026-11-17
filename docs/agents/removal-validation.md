# Accelerometer mode removal: validation record

Checked 2026-11-12 on branch `remove-gated-accelerometer`, with uncommitted
removal code based on HEAD `19c92e9`. This record does not attribute the removal
implementation to the Codex takeover or authorize a commit/push.

## Changes and retained behavior

The user approved removing the gated accelerometer mode and its older fixed-
rotation term. The always-on accelerometer term and no-accelerometer mode remain.
Ten obsolete settings and their registrations, helper code, tests and docs were
removed by Claude. Codex's independent research/review found two introductory
paragraphs still describing the removed mode; these were corrected in both
`docs/index.html` and `docs/hu/index.html`.

The reviewer found no functional defect in the reviewed removal diff. It checked
activation, velocity unknowns/covariance, acceleration-offset handling, retained
gravity update, band width and removal of unused members/callers. This is review
evidence, not exhaustive correctness proof. The reviewer was a Codex agent,
not Claude Sonnet.

## Build and tests

On November 12, initialized MSVC and invoked the existing Release dMapper/test
targets. Ninja reported `no work to do`, confirming no rebuild was necessary for
the current sources. CTest passed 1/1 tests in 13.72 seconds. Log:
`out/removal-current-build-test.log`. Debug was not run: this removal does not
introduce a configuration-dependent code path.

Claude's November 10 logs show the first test run failed on obsolete golden
TOMLs, then the golden files were regenerated and the full test suite passed.
The final doctest log reported 121 cases passed and 3 skipped.

## Recording baselines

Read and hashed the existing November 10 outputs against the original checked-in
manifests, then compared normalized old/new parameter text. Results:

| Dataset | Unchanged output files | Sole difference |
|---|---|---|
| 44_short | 10 of 11 | Parameters TOML removes exactly the ten approved keys |
| Full 44 | 34 of 35 | Parameters TOML removes exactly the same ten keys |

All manifest-tracked cloud, trajectory, pose, result and session files remain
bit-identical under compare_baseline.py's existing normalization rules. No numeric
tolerance was used. The full run's original log reports exit 0 and 1474 seconds.
No recordings were rerun during this validation because the existing outputs and
current build provide the required evidence.

Both manifests were regenerated from those outputs. Only the parameters hash,
commit metadata and manifest creation date changed. During validation the source
was dirty; after user approval the manifest commit metadata was updated to
`e196692`, the committed removal revision matching the validated code.
Both removal output directories now pass their updated manifests (11/11, 35/35).

The external baseline folders' top-level parameter TOMLs were synchronized to the
new serialization. Original TOMLs are preserved under each baseline folder's
`serialization_history/2026-11-12/`. No maps/trajectories were replaced or deleted.
The full baseline folder also contains 14 pre-existing `scan_lio_*.asc` analysis
exports outside the manifest. An unfiltered comparison of that folder reports
those as extra; all 35 manifest-tracked files match. Do not delete the exports
or add them to the application-output manifest merely to silence that message.

Local sources:

- Claude scratch outputs: `C:/Users/User/AppData/Local/Temp/claude/b4c2de060720d60b/cfb570e5-ebb1-4b0b-8768-6620c1e0ef05/scratchpad/44_short_removal/` and `44_removal/`.
- Original task logs: sibling `tasks/bh021o1cb.output` and `tasks/b5v3oy8j1.output`.
- External baselines: `C:/Users/User/Downloads/dLidar/raw data/44_short_baseline/` and `44_baseline/`.

## Next handoff

After user approval, removal code/docs were committed as `e196692`, golden TOMLs
as `e0b29ab`, and baseline manifests as `8d230a1`. Takeover setup, draft retune
plan, reports/research and diagnostic tools remain separate and uncommitted.
Push still needs the user's go. The shipped always-on reference maps did not
change, so this is not evidence for a new map-quality claim.

After that decision: turn-recording calibration, completed-batch readings,
phase 0a inputs starting at rest (Livox/2 0-60 s, whole Livox/1, 44_short), then
phase 0b information-scale experiment. The mid-walk 295-335 s cut in the draft
plan was superseded in Claude's final session. Notify before changing model/effort
for interpretation or weighting design; Sol Medium is adequate for mechanical work.
