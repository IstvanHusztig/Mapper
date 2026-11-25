# Livox4 targeted trajectory-code review

2026-11-13. User requested code analysis at the current medium effort while a
lower-cost agent monitors the authorized Huber3 replay. This is read-only
analysis; no algorithm modification, synthetic experiment or extra replay.

Actual replay uses frozen Huber source cbca36d and original Huber3 settings.
Code references below use scratch e79125c (exact pre-floor tree c52b25f);
relevant acceptance/carry behavior was cross-checked against frozen cbca36d.
The rejected floor is not part of this review or run.

## Observations and their calculation path

All 1,016 odometry windows logged 100 iterations: 101,600 iterations total.
The native summary reports zero failed solver steps and zero big-jump nodes.
No window was dropped for too few points. These are execution diagnostics,
not evidence that those updates were accurate.

| Window | Time | Observed state/decision |
|---|---:|---|
| 301 | 108.00 s | Last-node yaw correction -3.480 deg; result-minus-guess translation [0.063,0.050,-0.010] m |
| 313 | 112.32 s | Joint end-speed 2.542 m/s |
| 320 | 114.84 s | Joint end-speed 5.086 m/s; result-minus-guess [0.005,-0.012,-0.027] m |
| 321 | 115.20 s | Pose-derived previous velocity 5.05361 m/s is explicitly discarded by the 5 m/s prior guard; start prior resets to zero with 1 m/s sigma; joint speed drops to about 0.394 m/s |
| 397 | 142.56 s | Yaw correction -11.704 deg; result-minus-guess [-0.056,-0.130,0.017] m |

This narrows two different events: speed escalation and an explicit guard reset
around W301–321, followed by a large heading correction around W395–410.
Do not label them one sudden translation jump or claim the reset recovered the
physical trajectory. Trigger times do not establish the first inaccurate pose.

## Confirmed acceptance limitation

scan_to_map.cpp stepped_poses (233ff) compares candidate translations against
motion_model_poses with a fixed 1 m ceiling. It does not test relative rotation.
optimize_lidar_odometry (1310ff) accepts a successful solve if that translation
guard passes; it then updates poses, bias, velocities and acceleration offset.
There is no complete nonlinear objective-decrease or support-preservation gate.
next_lm_factor (264ff) increases damping after rejection or every tenth iteration;
it is a schedule, not an actual-versus-predicted reduction trust-region test.

Thus the 11.704-degree correction is compatible with zero translation-jump
rejections. Likewise a drifting high-speed prediction can receive a centimetre
correction and pass the 1 m test. The new scan confirms this already audited
weakness in another recording; it does not prove the missing guard initiated
the wrong matches or that adding an angular threshold fixes them.

## Explicit velocity reset and carry distinction

window_steps.cpp estimate_previous_velocity (29ff) derives velocity from the
post-tail pose trajectory and discards it above max_prior_velocity_mps.
run_odometry.cpp prepare_velocity_state (254ff) separately accepts the carried
joint velocity only below the same ceiling. Above it, a discarded pose velocity
leaves the fallback mean zero with initial 1 m/s sigma. integrate_window_translations
then uses this new prior to build the next translation guess.

That branch is visible at W321. It is a discontinuous reset of the prediction,
not a measurement of physical deceleration. The reported speed recovery must
not be treated as autonomous relocalization.

Joint end velocity/covariance are carried by update_acc_term_after_window before
repropagate_window_tail alters the last 15 nodes using a pose-derived velocity.
These estimates can therefore differ. Independent reviewer measurement over
all windows found difference p50 0.0416, p90 0.0727, max 0.1257 m/s. At W320 the
difference is 0.0645 m/s; both estimates exceed 5 m/s (joint 5.08615 versus
post-tail 5.05361). This does not support a large isolated handoff mismatch as
the explanation for that escalation. Do not prioritize it as a proven defect.

## Existing evidence limits

Snapshot v1 contains an array named motion_model_poses, but finish_run overwrites
it with final odometry poses before saving (run_odometry.cpp 933ff; verified
in frozen cbca36d). It therefore cannot restore the original pre-LM guesses.
The reviewer initially suggested using it, then identified and corrected this
provenance issue. Merely finding the array is insufficient evidence of a restart.
The saved joint state and post-tail poses do support the carry comparison above.

The existing matched-only, reassociated NDT cost can decrease by losing support;
it is not a safe complete-objective acceptance score. Previous mixed LM and
unmatched-cost experiments, synthetic tests and failed Cauchy/floor candidates
remain counterevidence against presenting another generic solver adjustment as
a demonstrated solution. No such adjustment is implemented here.

## Prediction versus optimizer correction: new bounded localization

Using logged within-window displacements (world vectors, rounded to 1 mm):

| Windows | Sum of predicted displacement magnitudes | Sum after optimization | Corrections opposing prediction |
|---|---:|---:|---:|
| 301–320 | 14.921 m | 14.227 m | 18/20 |
| 306–320 | 13.718 m | 13.140 m | 15/15 |
| 312–320 | 10.732 m | 10.462 m | 9/9 |

These sums omit between-window gaps and are not an independent travelled path.
The sign is dot(result-minus-guess, guess displacement). The reviewer used a
separate parser and confirmed the values and signs, including using differences
of the rounded displacement fields instead of the explicit correction field.

Most displacement escalation is already present in prediction; the net
positional correction reduces it during this interval. This does not isolate
joint velocity-factor updates or exclude earlier matching errors contaminating
the carried attitude/velocity. Calling scan matching the sole immediate
accelerator during this interval would misdescribe these readings.

Further decomposition uses exact window node duration T from the odometry
snapshot and previous logged joint end velocity v, accepted by the carry path
for W301–320 (all below 5 m/s, logged rest=0):

`guess displacement = v*T + displacement from integrated acceleration`.

For W320, T=0.35794005 s and incoming v=[2.916,3.584,-0.363] m/s. Constant
velocity accounts for [1.04375,1.28286,-0.12993] m of the predicted
[1.091,1.364,-0.151] m. The remainder is [0.04725,0.08114,-0.02107] m.
An equivalent constant acceleration producing that remainder is
[0.738,1.267,-0.329] m/s^2. At W313 this equivalent is [1.389,1.595,0.262].
These are summaries of the predictor's displacement, not measured physical
acceleration or isolated attitude/gravity error. Rounded velocity/displacement
logs limit precision (about 0.5 mm and 0.5 mm/s per component). The acceleration
fit log's world/body means are *residuals*, not the acceleration to substitute
in this equation. No such substitution was made.

Evidence: out/livox4-prediction-vs-matching.json and
out/livox4-prediction-decomposition.json. The relevant call chain is
prepare_velocity_state -> integrate_window_translations -> freeze model before
LM. Integration uses `a=(R*raw_acc-carried_gravity)*to_mps2`.

## Reviewed next focus

Independent code/evidence review confirms the guard/reset findings and narrows
the next investigation to incoming velocity and attitude-to-gravity integration
around W301–320. Separate an already excessive prior from acceleration introduced
by attitude/gravity disagreement. Some of this is now bounded using existing
data above; exact pre-LM orientation/gravity attribution is still unavailable.
Do not repeat synthetic tests already done, increase the rejected floor, or
present an angular cap or naive matched-cost line search as an established fix.

No algorithm change, extra replay or full-map accuracy verdict follows from this
review. The complete Livox4 cloud and final diagnostic coverage remain the
separate deliverable of the authorized replay.
