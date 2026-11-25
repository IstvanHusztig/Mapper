# Livox3 quality measurement and overlay

2026-11-13. User reports the new cloud looks crisp and requests existing quality
tools plus a house-plan overlay before agreeing open/closed-space research.
The user hypothesizes the algorithm relies on surrounding walls. This is a
research hypothesis, not a causal verdict: route and acquisition/filtering both
changed between Livox2 and3. Cave suitability is not established by a room scan.

## Existing measurement tools, completed
Used tools/wall_precision.py with identical default extraction settings at world
Z=+0.50,-2.10,-4.70m, each +/-0.05m. These are declared sampling heights spanning
the vertical extent, not surveyed floor elevations or a certified floor ID.
Default0.02m cells, sigma3cells, threshold.03, min_length.5m, piece.5m,
across-band.2m, visit gap2s, min15points/visit. No candidate-specific removal of
mirrors/reflections or high-error pieces. Point loader transforms exported chunks
with their recorded session poses; no additional registration of revisit pieces.

| Section Z | Slice returns used | Pieces / ridge length | Wall spread p50/p90 | Visit difference p50/p90 | Pieces with visits |
|---|---:|---:|---:|---:|---:|
|+0.50m|499113|223 /97m|11/66mm|15/47mm|194|
|-2.10m|231447|122 /52m|8/65mm|12/41mm|118|
|-4.70m|116792|118 /51m|10/45mm|17/45mm|109|

Wall spread is1.4826MAD per extracted piece, then quantiles across pieces;
not literal full wall width. Selected same-side visit differences are diagnostic,
not independent truth. The reference20/60mm p50/p90 sharpness target is exceeded
in the p90 tail of the first two sections; no global PASS. Furniture, nonwall
ridges and known mirror/window returns have not been physically classified.
The0.2m extraction band can miss distant ghosts. Ridge length and selected point
counts are coverage descriptions, not proof all physical walls were measured.
No frozen-same-region candidate ranking or perturbation envelope is available.
Recording44-specific facade/revisit scores do not apply to this different route;
holdout sharpness is not measured (holdout disabled). Pre-loop/final displacement
and diagnostic coverage remain as recorded in livox3-indoor-plan.md.

Evidence: BASE/livox3-indoor/huber3-baseline/result/map_quality_cache,
wall_precision_+0.50/-2.10/-4.70.csv and thickness/visits PNGs. Tool logs archived
under quality-measurements/. The input cloud remains unchanged.

## Overlay
Independent overlay task uses the existing apartment56 reference PDF and wall-mask
fitter, fitting a fresh scale-preserving rigid transformation for this recording.
Do not reuse the Livox2 world transform blindly. One fixed transform must apply
to the top-floor comparison; no per-segment fit to conceal drift. Fit ambiguity,
window/mirror projections and plan limitations must be retained. Overlay is
preliminary visual registration; not independent validation of surveyed accuracy.

## Proposed next phase
After quality/overlay review, a research phase should compare methods for weak
geometric constraints, reliable correspondences in vegetation/open space,
uncertainty/bias/velocity-prior behavior, and transitions between confined and
open environments. Define falsifiable tests and preserve the successful indoor
result before numerical implementation or more runs. Avoid assuming planar walls
are necessary, or that one loss-function setting resolves all environments.
Primary papers/official implementations should inform the design. User prefers
research agents when useful, Terra when appropriate, and notification before
higher effort/model use. Sol6.1 High is a reasonable starting recommendation for
this next research phase, not a guarantee of solving the numerical problem.
Existing HTML docs should document an accepted general fix only after acceptance;
this run and current measurements do not yet establish that condition.

## Overlay completed
Output quality-overlay/livox3_plan_hypotheses_and_return.png shows both leading
rigid registrations and shared-transform beginning/end rest. Fixed nominal plan
scale, no warp; top-floor worldZ.45..55m;498639 selected returns. Fit uses only
first0–15s, apartment56 Wohnen wall-mask. Lowest truncated-cost rotation22.25409deg,
translation[22.15166,13.68324]m; alternate201.88604deg,[20.00067,12.88826]m is only
4.36% higher cost. Both retained; PRELIMINARY_AMBIGUOUS, not unique or surveyed.
Visible wall correspondence is encouraging but is not an accuracy confirmation.
The apartment crop shows379015returns for first hypothesis,119624 outside crop;
whole plan/hypotheses images and JSON preserve coverage. No mirror/window points
manually removed; ghosts are not automatically scored as pose drift. This is the
available apartment56 top-floor plan; lower floors are not independently overlaid
against their own plans. Start0–15s and end246–253.327s use the same registration.
Root visually inspected the overlay and retained both hypotheses.

Model recommendation: Sol6.1 High for the next research/design phase. Official
model docs describe Sol6.1 as suitable for complex coding; High is our task-specific
starting choice, not an OpenAI guarantee or measured capability boundary.
https://developers.openai.com/api/docs/models/gpt-6.1-sol
Ask before escalation if the geometric/estimator design needs more capability.
No new odometry run or numeric change was made for this quality task.

Independent Sol6.1 review checked measurement interpretation and overlay limits:
no all-quality pass, no per-visit registration, ambiguity/reflections retained,
no perturbation or causal claim. Approved bounded conclusions for user review.
