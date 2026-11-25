# Livox2: confirmed return to apartment 56

## New physical context

On November 13 the user confirmed that full Livox2 finished in apartment 56,
the apartment where recording began (Wohnen). This resolves the pending route
question. The exact sensor position, orientation and height at the end are not
known, so do not impose zero endpoint displacement or report 62 m as an exact
surveyed closure error.

The retained Huber3 odometry has endpoint separation 62.147743 m, with relative
XYZ [-57.374455, -23.884900, -0.159468] m. Final separation is 62.338117 m,
XYZ [-53.371723, -31.793888, 5.162238] m. Together with the confirmed apartment
return and the existing scene views, this establishes a gross failure to place
the revisited apartment consistently. Postprocessing has not repaired it.

## Continue using existing evidence

Use the verified odometry snapshot, not overwritten motion-model poses, to
locate accumulated orientation disagreement and translation growth. Compare
relative rotations in SO(3), not differences of Euler components. Keep the
early traced interval and later untraced interval distinct; no new mapper
execution is needed for this analysis.

Terra 6.1 source research confirms that saved imu_angles are the initial Fusion
AHRS orientations (XYZ Euler convention), using startup-bias-corrected gyro and
accelerometer with dynamic gain. They are not pure gyro integration or physical
truth. Saved optimized poses include handoff/tail changes. Posterior gyro bias
is residual to startup bias. Exact gravity/acceleration-offset/scale after W885
are absent from the retained snapshot.

The full prediction sequence matters more than individual TOML flags. Although
use_imu_translation_prediction and use_zero_velocity_detection are false,
imu_always_on enables rest detection. After generic prediction and gyro-bias
correction, run_odometry.cpp:1054 calls integrate_window_translations for enabled
non-rest windows. This overwrites moving-window translations using the selected
velocity, bias-corrected predicted orientations, raw acceleration, and carried
world gravity. Solved acceleration offsets subsequently update that gravity.
Root review corrected two incomplete Terra interpretations of this call chain.
The acceleration/velocity feedback hypothesis remains applicable; it is not
established merely by these source facts. No numerical correction selected.


The original W798 conditional uncertainty gap remains unresolved. Existing
structural identity and return evidence do not turn incomplete diagnostics into
a PASS. No shipped setting, acceptance, push or archive is implied.

## Existing-snapshot result

The reviewed orientation check completed in about seven seconds, reading all
1,517 windows / 273,060 poses. Source is archived under
stage-clouds/analysis_scripts/full_livox2_orientation.py, SHA256
28ee65ad2f377679269ed7412aac85b2c915b8e3143ee36bc16d1ffdc38f4c5d.
Results are in full-livox2-huber3/return-orientation/.

Projected body-X azimuth disagreement from the once-anchored initial AHRS is
-13.4 degrees at 120 s, -41.4 at 150 s, -126.3 at 210 s, -150.6 at 330 s and
-107.7 at the end. This is not native Euler yaw or physical attitude error.
Large within-window relative-rotation discrepancies occur at W416 (149.4 s,
9.63 degrees), W565 (203.04 s, 9.80 degrees), W576 (207.0 s, 9.84 degrees),
and W1068 (384.12 s, 14.39 degrees). Thus orientation disagreement substantially
predates the later translation-speed escalation. Bias changes accompany it;
that association does not establish which component initiates the error.

## One focused correction test

An independent Codex reviewer (not Sonnet) supports a single full-start replay
with use_gyro_bias_estimation=false. Keep the existing frozen Huber3 executable,
alpha0.1/beta0.1, startup bias removal, and every other parsed TOML setting.
Disable diagnostic trace output, retain standard per-window diagnostics and
odometry/final clouds. No source rebuild or further diagnostic framework.

This disables the entire online bias-feedback mechanism: bias unknowns and
couplings/prior, carried-bias pose correction, and bias-update tail correction.
It is not merely freezing a posterior value. Rotation-sigma inflation remains
active, so LiDAR-driven orientation changes can still occur. Compare early
departure, pre-loop apartment return, and actual final geometry. AHRS agreement
alone cannot accept the candidate. A negative result does not rule out weakness
of the rotation prior.

The unchanged run-health tool unconditionally expects bias rows. Preserve its
raw result and explicitly label bias inapplicable for this intervention; retain
all other missing evidence without converting the result into PASS. Trace
motion_cost_supported is false with bias estimation disabled, so no zero trace
cost is interpreted. The new run needs no trace-based scalar-cost comparison.

The previous full replay took about 68 minutes with tracing. This trial may be
faster without tracing; completion, not elapsed time, controls the result. The
standing authorization to continue toward a fixed full cloud covers this test.
No shipping acceptance or push is implied.


## Replay launch
The minimal runner passed its settings/environment checks and independent Codex
review (not Sonnet). Root also confirmed no mapper, postprocessing, stage-export
or reconstruction process was active. Runner SHA256
`d1a1f1a8ea6f0cecc337de354882cd6d6b88467ddd868bf41d60552744ad2297`.
Root session92411 launched its preflight/full-run workflow. New output root:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-huber3-no-online-bias`.
It reuses the original frozen Huber3 binary and records the sole parsed TOML
change. The normal CLI may rewrite derived raw/cache and raw/preview; original
recording files and retained experiment results remain identity-checked.
Completion and comparison remain pending. No automatic retry.


## User stop and independent Sol 6.1 audit (2026-11-13)
User explicitly stopped further investigation/runs/changes and requested Sol6.1
review of decisions and code. No mapper remains active. Sol6.1 read-only audit
is running; no implementation authorized by that audit.
Latest no-online-bias mapper completed exit0, native2395.60s, runner2474.37s;
result is substantially worse (window-start endpoint proxy209.94m,minZ-87.34m,
maxspeed8.60m/s). Native clouds saved. DO NOT RERUN or adopt setting.
Collection then failed serializing NumPy health['speeds']; unsafe overwrite and
same exception-handler save truncated run.json, run-health.json also partial.
run-health.txt complete; W814conditionaluncertainty unresolved, biasrowsdisabled.
Original partial files and frozen runner preserved. No recovery collector has
been executed; any later recovery must be offline into a separate record with
original bytes/hashes preserved. Do not silently repair original evidence.
Current authorized work is independent Sol6.1 audit only. Main cloud goal remains
unfinished. User requests OpenAI investigation; no escalation channel available
or submission claimed. Root will report audit without implementing fixes.
