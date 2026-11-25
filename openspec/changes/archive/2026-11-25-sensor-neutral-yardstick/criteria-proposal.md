# Agreed evaluation contract

2026-11-12. **Agreed by the user: "yes i agree".** These are project
engineering choices, not survey standards or demonstrated sensor accuracy.
No candidate comparisons were used to select these limits.

## Separate improvement from readiness to ship

A useful experimental improvement need not already meet the final quality goal.
Report two separate answers: whether a candidate improves the reference, and
whether it meets the agreed product targets. A failed reference does not become
a good map merely because the candidate is slightly less bad.

For each lower-is-better metric, measure the same physical regions and stages
across the same perturbation family. Report the base value and min/max envelope.
Call improvement resolved only when max(candidate) < min(reference) for a
predefined error metric. Use the same predefined perturbations and do not drop
failed or missing members. Overlap means inconclusive, not equal. Require no
resolved regression in geometry, revisit consistency or sharpness and no wider
perturbation range. Do not average those three axes into a score. With only
three to seven runs these are empirical envelopes, not confidence intervals.
Show readings first; candidate acceptance remains the user's decision.
No demonstrated regression is not proof of non-regression: overlapping envelopes
cannot alone satisfy the shipment gate. Unresolved regressions require further
evidence or a separately agreed non-inferiority tolerance before shipping.
Absolute targets must hold for every valid perturbation member, not just base.

## Agreed numeric targets and their applicability

| Measure | Proposal | Reason and limitation |
|---|---|---|
| Building planar-wall sharpness | Keep 20 mm p50 / 60 mm p90 as final targets; remove them as a prerequisite to investigate an improving candidate | `wall_precision.py` measures 1.4826 MAD per piece, then quantiles across pieces. It is not literal full wall width. Same surfaces, range, incidence, height and coverage must be retained. |
| Short indoor-run perturbation displacement | At most 0.10 m maximum base-to-perturbation position difference on each of the selected short inputs | Proposed local ghost-wall budget instead of allowing a longer estimated path to excuse larger local errors. Report p95 and duration above 0.05 m too. Not an accuracy claim. |
| Full-route perturbation displacement | At most 0.3 m per 100 m of a frozen reference route length, and no wider metric envelopes | Tighter than old 0.5 m/100 m; freeze the denominator before candidates so drift cannot improve its own score by lengthening the estimated route. Report absolute maximum too. |
| Pre-loop closure on a confirmed return | Investigation ceiling 1.0 m/100 m; desired target 0.3 m/100 m | The ceiling is looser than old 0.5 m/100 m, the product target tighter. Exceeding it flags a failed diagnostic target but does not forbid measuring an improving experiment. Use actual corresponding endpoint poses and physical displacement; returning to a room alone is insufficient. |
| Final local alignment at a confirmed return | 0.10 m maximum endpoint separation only when the sensor revisits the same physical position; otherwise NOT MEASURED | A loop solver can force its own endpoint closed. This cannot prove whole-route accuracy; independent revisit geometry and references still required. |
| Return to a known floor/level | Absolute vertical residual at most 0.10 m, only with independently known sensor-height difference | Replace route-length-scaled floor tolerance; changing sensor height is not drift. Without the physical measurement report the observed difference, not an error. |
| Loop-closure correction size | Report maximum and spatial extent; remove the universal 1 m fail rule | A correction can be large and correct, or small and wrong. Pre-loop drift and post-loop geometry decide acceptability. |
| Natural cave surfaces | No absolute wall-thickness pass limit yet | Rock roughness and geometry require different sampling. Use matched revisit surfaces and independent distances; a building pass does not certify cave performance. Define cave limits with actual cave data and reference uncertainty. |

The short-input rule applies to Livox/2 0–60 s, whole Livox/1 (~95 s), and the
44_short regression comparison. The full-route rule applies to longer phase-gate
recordings; it does not waive the short-input rule. Both are proposed stability
budgets. They are not a guarantee that ten-centimetre errors are acceptable
everywhere. Local shape and revisit checks remain independent requirements.
Apply stability budgets separately to odometry and final trajectories; loop
closure must not conceal unstable odometry. For closure rates also use a frozen
reference route length. Endpoint error is the norm of estimated displacement
minus independently known physical displacement; separation is its special case
when the true displacement is zero. Report reference uncertainty for endpoint
and height measurements; an unresolved threshold boundary cannot be called PASS.

## Diagnostics and recording validity

- Retain 3-degree bend share, 10-degree bend count, 2.5 m/s speed crossings,
  residual bias above 0.5 degree/s, and velocity chi-square median/p95 5/20 as
  investigation triggers, not standalone accuracy verdicts. Report counts,
  times and coverage. Motion/setup and independent evidence determine whether
  a trigger is physical. Raw startup bias is distinct from optimized residual bias.
- Read velocity jump magnitude and carried sigma separately. Conditional
  weak-axis diagnostics are a selected subset; do not compare their chi-square
  against a full-population distribution without validating selection and DOF.
- Timing must be strictly increasing. Gaps, missing points and false-offset
  suspicions require explicit disposition. Recorder transport latency is not a
  measurement of internal LiDAR/IMU time alignment.
- At least two seconds of supported startup stillness is the intended input
  requirement. A data-relative quiet-block heuristic alone neither proves nor
  disproves it. Check acquisition notes, first-sample statistics and the actual
  startup-bias gate. Handheld rest does not calibrate sensor noise.
- Keep 44/44_short as historical regression guards with recorded sensor/timing
  defects; do not use them to infer healthy-Livox noise or calibrate a physical
  uncertainty model. Any numerical change still needs the full 44 s0/s1/s2 gate.

## Measurement validity before any verdict

Require identified input/configuration/build, successful run completion, complete
expected diagnostics, and separately identified odometry/final stages. Missing
required evidence means NOT MEASURED, never PASS. The current run-health tool's
first/last window-start distance is a closure proxy, not the endpoint metric above.

For seams, require equal timestamps and counts before invoking the existing tool;
never accept its truncated, index-matched result with a timestamp warning. Use
the same starting coordinate frame; do not align away an error independently
for each region or perturbation. For different sampling schedules, require a
separately validated timestamp-matching method before comparison.

For maps, freeze physical regions and extraction settings, show coverage and
discarded/missing regions. Automatic wall extraction can omit doubled walls
outside its 0.2 m band; a smaller reported thickness with lost surfaces is not
an improvement. Same-side visit differences and half-split estimates are useful
diagnostics, but half-splits share drift and are not ground truth. Use held-out
points where fitting bias is at issue and preserve the user's visual gate.

Existing generic sweep resume and exit-status behavior are not trustworthy enough
for unattended decisions. No sweep or behavior change is authorized by this
document. Task 2.5 must establish and verify a restricted invocation procedure or
obtain separate scope for fixes before relying on those outputs.

## Review and adoption

Independent Codex review (not Sonnet) completed on November 12. Incorporated:
inconclusive versus established non-regression, exact envelope definition, all
perturbation members, separate processing stages, explicit looser investigation
ceiling, and endpoint-reference uncertainty. No numerical budget is claimed to
be externally validated by that review.

After user agreement, reconcile the draft roadmap, relevant tool defaults and
both agents' verification skills before operational use. In particular, replace
the old statements that overlap means no difference, diagnostic triggers prove
map failure, and every loop correction over 1 m fails. Behavior changes still
require the task 2.5 scope decision. Agreement establishes the evaluation rules;
it does not make existing tool verdicts compliant. Until tool restrictions are
verified, no automated PASS should be used to rank candidates.
