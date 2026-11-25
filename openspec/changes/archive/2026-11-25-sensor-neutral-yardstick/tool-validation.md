# Verification-tool implementation and validation

Approved by the user on 2026-11-12. Application algorithms/settings unchanged.

Implemented: complete per-window diagnostic coverage; expected-window totals;
duplicate/orphan/malformed-line rejection; conditional uncertainty coverage;
diagnostic triggers separated from accuracy; exact Decimal trajectory times;
finite data and strict count checks; snapshot v1 world-frame reader; independent
odometry/final measurements; SHA256 input/config/binary/tool/policy/output
identity; refusal of unidentified/changed/failed resume; preserved clouds;
nonzero failures and incomplete-family exits; frozen full-route denominator.

Validation:

- `python -B -m unittest discover -s tests -p test_verification_tools.py -v`:
  10 tests pass, including synthetic seven-member sweep and successful resume
  without relaunch, failed subprocess retention, mutated identity, corrupt
  snapshots, absent diagnostics, missing trailing window coverage, and exact
  timestamp/coordinate cases. Output: out/phase0a-2026-11-12/verification-tests.txt.
- Real historical Livox2 base snapshot: 273060 nodes,1517 windows; all window
  starts match logged world XYZ within0.0004999845m (log rounds to millimetres).
  Final export also has273060 nodes. Evidence:
  out/phase0a-2026-11-12/snapshot-reader-validation.json.
- Independent Codex review (not Sonnet) found missing-window, cancellation,
  empty-accelerometer, frozen-denominator and diagnostic-limit identity defects.
  All corrected; regression tests added/updated. Parent ran final passing suite.
- Both agents' guidance updated and parity checked; OpenSpec strict validation
  and git diff whitespace checks run. No C++ changes, so no mapper rebuild or
  new recorded-data processing was needed for this tool-only change.

Limits: no real seven-member batch has run through the new orchestrator yet.

Follow-up live collection validated the first real baseline and resume. A sparse
shipped TOML omitted save_odometry_snapshot; the evidence-option helper now
inserts absent debug options in their proper table. Added shipped-TOML coverage:
11 targeted tests pass. No shipped config was changed. The current baseline
completed with identified inputs/outputs and exactly reproduces historical final
positions. A preselected perturbation is collecting; no full-family pass claimed.
The old Livox2 log has two windows (422,520) with absent conditional uncertainty
and rounded eligibility at the boundary; conservatively INCOMPLETE rather than
inventing readings. Actual source eligibility cannot be recovered from rounded
logs in all cases. This may require better diagnostic evidence in later work;
it does not establish faulty hardware. Stage checks do not replace map coverage,
held-out data, full44 regression evaluation or the user's visual acceptance.

Task2.5 is satisfied by the approved implemented fixes and failure-path tests;
the earlier restricted-use document is historical. Phase0a remains incomplete
until tasks4.2–4.4 resolve existing evidence, the minimal run list and readiness.
