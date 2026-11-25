# Early heading failure after the Sol audit

User resumed investigation on 2026-11-13. No new full-route replay or production
change. Existing Huber3 evidence remains subject to its W798 diagnostic gap.

## Established local failure

Exact original IMU timestamps match all 180 saved nodes in each of W408–416.
Startup-corrected body gyro integration is compared with local relative rotation
using SO(3) distance, not Euler subtraction. This is a short-interval consistency
check, not surveyed absolute attitude. The startup upper medians reproduce the
logged calibration; integration is not a bit-identical Fusion implementation.

| Window | Recording seconds | Gyro/AHRS disagreement | Optimizer/gyro before tail | Saved/gyro after tail |
|---|---|---|---|---|
| 415 | 149.424–149.782 | 0.040° | 9.188° | 8.951° |
| 416 | 149.784–150.142 | 0.084° | 9.743° | 9.632° |

Left/right sample integration differs by 0.046°/0.022°. Max IMU time steps are
2.513/2.851 ms. The roughly 9° departures arise inside optimization, before the
handoff tail; they are not explained by an AHRS-only reference discrepancy.

All 200 optimizer steps in these two windows were accepted. Source
`scan_to_map.cpp:1288` onward tests solve success and a translation-distance
guard, without cumulative angular or objective-decrease acceptance. This enables
the observed bend; it does not prove an objective-decrease test would reject it.

At W415 iteration20, frozen quadratic changes along the actual joint update are:
NDT −51.23, motion +30.86, carried velocity +14.49, acceleration +2.88.
The 2–5 m NDT range contributes −83.98; the under-2 m range opposes with +32.50.
Angular RHS dot-step isolates the same conflict: NDT +16.38 versus motion −13.29.
These are local linearized contributions, not actual nonlinear objective changes.
The 2–5 m range is the strongest supporting range in 59/60 sampled steps1–30.

Bias adapts alongside the bend. Its magnitude over a 0.358 s window cannot
directly account for 9°, but coupled effects are not ruled out. The failed full
bias-off experiment remains evidence against disabling the entire mechanism.

## Code checks and exact correspondence reconstruction

Independent Sol6.1 review found no bias Jacobian sign reversal or repeated
per-iteration bias covariance carry. It found a degree/radian mismatch in the
first-node increment regularizer and omitted bias correction across window gaps.
Neither is established as the principal cause here: first-node motion remains
below0.00011°, and the ordinary inter-window gap is approximately2ms.

The existing native reconstruction tool was extended to select W415 and apply
the native Huber helper before alpha. All six source/association/target-geometry,
count and raw-cost checks and all12 range-quadratic checks reproduce the saved
trace. Input identities are rechecked. No odometry is run in this reconstruction.
The first attempt passed these checks but stopped at a W754-specific historical
cell check; its failed output is preserved. V2 scopes that optional history to
W754 while retaining all matched-cell geometry gates; V2 completed successfully.

At W415 iteration5, three outdoor-grid cells supply171.88 of183.60 net angular
RHS dot-step: 85899345919,85899345918,85899280383. Their minimum covariance
standard deviations are5.25,10.36,7.71mm; mean Euclidean residuals approximately
0.275,0.297,0.304m; mean Huber weights0.157,0.243,0.110. Residual norms are not
normal distances. Both grids may use the same return. This identifies influential
correspondences, not a physical object, sensor defect or permanent range cutoff.

## Bounded causal check

Prepared: exact100-iteration W415 Huber3 control, then an identical restart with
only those three outdoor cells omitted. Gyro/bias/accelerometer machinery and
Huber cutoff3 remain unchanged. No intervention is permitted unless every
control trace record matches. Independent Sol6.1 review checked state reset,
control gating and selected cells. This is a diagnostic ablation, not a proposed
production removal or proof of a full-route correction.

Completed exit0: every emitted record across all100 control iterations matched
the retained trace exactly. The same initial state with just the three cells
omitted reduced endpoint/max orientation change from9.142869° to2.656545°
(70.94% reduction). Both runs accepted100 steps. Maximum node-position change
was0.057334m; endpoint velocity norm increased0.502139→0.600809m/s, so this is
not a claim that every local metric improved. Estimated bias Z changed
0.249715→0.162265°/s. Both runs retained Huber cutoff3 and identical IMU machinery.

This is causal evidence that these coarse-grid correspondences produce much of
the early W415 bend. It does not explain the entire62m route error, identify the
physical reflecting/moving object, or establish a general correction. Residual
bend remains. Permanent cell deletion or a route-wide range cutoff is not a fix.
The next correction should bound inconsistent NDT influence while preserving
gyro/bias estimation, and first reproduce/improve this checkpoint before another
full replay. Huber downweighting alone demonstrably leaves excessive influence
here. Selection of a general kernel/noise model still needs a bounded test.

Tool changes are diagnostic-only: selected-window reconstruction, native Huber
weights, angular contribution export and the exact-control-gated W415 ablation.
No odometry production equations/settings were changed. Failed attempt, successful
v2 reconstruction and checkpoint have separate immutable output directories.

Artifacts: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-huber3/early-heading-audit/`.
Trace SHA256: `33aaa21d15feba3fe9662741fc706d2786ca6b901a57fcaf3d178af917def604`.
Scripts and structured results are archived there; the original failed bias-off
records remain untouched. Release analysis-tool build passed; no production
numerical change or mapping-quality acceptance is claimed.
