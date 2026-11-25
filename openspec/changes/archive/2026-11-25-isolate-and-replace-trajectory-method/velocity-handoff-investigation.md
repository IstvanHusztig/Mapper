# Completed 125-second result investigation

2026-11-14. User reports a major visual improvement, remaining double walls and
a displaced exterior section. Investigated existing result and source d213b75,
with independent read-only review by seed_damping_crosscheck. No new recording
processing or algorithm edits were performed.

## Findings

- Recovery first activates at W301/108s. All 1004 completed diagnostic rows from
  the preceding capture match the candidate prefix exactly. Earlier errors are
  therefore not introduced by this translation-recovery change.
- Recovery is used 127 times in windows 301, 302, 303, 307 and 312.
- Largest logged start-velocity disagreement is W313/112.32s, 0.3494 m/s.
  Its reported prior sigma along the weak direction is 0.0039 m/s and chi-square
  is 7535.2. Earlier examples include W133/47.52s at 0.1955 m/s.
- W312 ends with velocity [-0.61581738,0.55895964,-0.10229657] m/s.
  Its saved geometric tail velocity is [-0.68146461,0.51499083,-0.08129227],
  a difference of 0.08176 m/s. W313 start is
  [-0.84540133,0.29560487,-0.10807654]. Substituting the tail velocity would
  still leave approximately 0.275 m/s disagreement. Such substitution is not
  established as physically correct.

## Concrete code inconsistency

In scratch src/application/run_odometry.cpp, update_acc_term_after_window
(called at line1234) copies optimized velocity.end and its covariance into carry.
repropagate_window_tail runs afterward (line1249). In domain/window_steps.cpp
line198 it replaces the last15 poses using constant geometric velocity estimated
from10 preceding samples and reconstructed IMU rotations. It does not reconcile
the saved velocity or covariance with these replacement poses.

For W312, retained poses end at node164. Nodes165–179 are replaced. The end-IMU
factor in scan_to_map.cpp uses anchors171 and176, so both positions supporting
the optimized end velocity are subsequently replaced. This is an existing
handoff consistency weakness exposed by the current analysis. Its contribution
to the full exterior displacement remains unproven.

The alternative claim that recovery simply leaves nuisance variables permanently
frozen is unsupported. W312 recovery at iteration98 is followed by a joint
accepted iteration99. Its velocity-prior/offset costs change and IMU cost falls
while poses barely move. Detailed W312 end-velocity increments were not captured.

## Post-processing and geometry

Compared all62280 snapshot and final-CSV poses with verified matching timestamps.
Final displacement from odometry reaches2.84118m at122.04265583s. In112–125s its
median is2.73390m. These are stage changes, not measured accuracy errors.
Endpoint displacement over112–125s is9.84m before postprocessing and9.75m after.
The earlier refreshed-match control gives4.36m, but has its own large rotation
error and is not ground truth. The trajectory difference develops in odometry
and is additionally modified by loop closure/consistency.

The annotated screenshot has no world-coordinate registration. Its exact black
rectangle has not been uniquely bound to a point/time subset. Generated world-XY
time views support further localization without assuming screenshot colours
encode time. Full map-quality and perturbation gates remain outstanding.

## Next discriminating check

Use an existing frozen solver case to compare the end-IMU velocity factor directly
before and after the exact tail replacement. Measure overwritten anchors,
velocity disagreement, covariance meaning and next-window seed. This can isolate
the handoff consistency defect without another recording replay. Select a fix
only after this check, preserving the rotation and merit guards. Astra high
remains appropriate for that numerical analysis.

Evidence scripts and views are archived in the result bundle's
velocity-investigation directory. Independent review confirmed the handoff
ordering and overwritten anchors and constrained the causal conclusions above.

The authorized captured-call check is complete in tail-factor-check.md. It
confirms that native tail replacement raises end-factor residuals to about
0.026m/s in all four saved cases while retaining velocity/covariance. Independent
review and native complete-objective gradient checks passed.
