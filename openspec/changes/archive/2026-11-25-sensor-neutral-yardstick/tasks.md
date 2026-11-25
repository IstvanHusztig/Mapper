# Tasks

Status recorded on 2026-11-12. Checked tasks record work already evidenced before
this change was created; they do not imply unexecuted tests or approved thresholds.

## 1. Prerequisites and provenance

- [x] 1.1 Link removal commits and validation evidence; verify e196692/e0b29ab/8d230a1 and docs/agents/removal-validation.md.
- [x] 1.2 Record turn-run readings and unresolved calibration; verify docs/agents/turn-calibration-measurements.md and referenced artifacts.
- [x] 1.3 Inventory existing reference/sweep families and configuration differences; verify evidence.md paths and research-agent findings.
- [x] 1.4 Record the user's revised-limits-before-comparisons decision; verify evidence.md and leave comparisons blocked below.

## 2. Input and diagnostic readiness

- [x] 2.1 Prepare Livox/2 0-60s with cut_recording; verify successful exit and point-copy verification in out/phase0a-2026-11-12/cut.log, then raw IMU health/rest coverage. Never use the superseded mid-walk cut.
- [x] 2.2 Verify whole Livox/1 and44_short input presence, rates, initial rest and provenance; record commands and outcomes in evidence.md without candidate rankings.
- [x] 2.3 Replay diagnostic tools on known logs and check expected counts/coverage; record missing metrics as NOT MEASURED and distinguish recording checks from algorithm criteria.
- [x] 2.4 Audit generic sweep/seam/run-health limitations; verify the enumerated findings in evidence.md. No fixes claimed.
- [x] 2.5 Resolve tool findings before relying on affected comparisons: agree a scoped fix or documented restriction, verify with meaningful failure/misalignment/provenance examples, and record remaining limitations. Do not silently expand this documentation change into code changes.

## 3. Revised limits: user gate

- [x] 3.1 Define metrics and separate building/cave, pre/post loop closure, local shape, recording validity and diagnostic coverage; verify a concrete criteria document with rationale and research review.
- [x] 3.2 Obtain explicit user agreement to revised limits and applicability; record the dated decision in this change. Until complete, do not compare or rank candidates.

## 4. Comparison readiness after the gate

- [x] 4.1 Reconcile reference settings and provenance for reusable families; verify copied L2 values, rotation-floor changes, removed keys and executable limitations are explicit before comparison.
- [x] 4.2 Apply agreed criteria to valid existing references/perturbation evidence; verify a table includes failures, missing coverage and spread, without treating uncertainty diagnostics as map accuracy.
- [x] 4.3 Identify only the additional runs needed to fill evidence gaps and state their cost; verify user scope/criteria and isolated inputs/output provenance before executing.
- [x] 4.4 Present phase0a evidence and unresolved limits to the user; verify all readiness tasks are complete before proposing phase0b as a separate experiment. Do not declare a better final cloud from preparation alone.

## 5. Approved verification-tool implementation (2026-11-12)

- [x] 5.1 Reject missing/incomplete diagnostic coverage, distinguish triggers from accuracy, and test missing/malformed evidence.
- [x] 5.2 Enforce exact timestamp/count matching, finite data and separate stage selection; test mismatch/truncation and valid coordinate transformations.
- [x] 5.3 Fingerprint inputs/config/executable and outputs for resume, propagate failures, retain clouds, and test resume refusal and failure paths.
- [x] 5.4 Validate snapshot extraction against writer conventions and real evidence; document limitations and synchronize both agents' guidance.
- [x] 5.5 Run targeted tests and independent review; record results and remaining phase0a work.
