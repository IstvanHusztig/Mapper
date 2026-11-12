# Full Livox2 flyaway: existing-run diagnosis

User authorized this investigation after switching to High on 2026-11-12 and
reported that the final cloud appears to fly outside the building. Only retained
data were analyzed. No mapper run, build, setting change or application fix was
performed. Root source remains unchanged; candidate source is scratch commit
16807ac, alpha0.1/beta0.1. This is diagnostic evidence, not candidate acceptance.

## Evidence and time convention

Experiment root:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-motion-prior/`.
Inputs: run.log, run.json, result/odometry_snapshot.bin, exported trajectory CSVs
and session_poses.mrp, original raw data/livox/2/imu0002.csv. Prior completion
checks and hashes are in run.json. New evidence is in jump-diagnosis/:
summary.json, all-windows.json, windows.csv, trajectory-and-prediction.png and
manifest.json. The offline extractor is out/full_livox2_jump_diagnosis.py;
its hash and input/output hashes are recorded. All1517 log windows were matched
to snapshot starts within the printed log precision. Each stage has273060poses.

Times below are WINDOW START times in seconds from the first trajectory node,
whose sensor timestamp is 0.38408168s. Endpoint velocity estimates belong near
the end of that window: W759 starts at 272.88s and ends near 273.24s. Raw IMU was
aligned by that same origin, not its own first sample. Window labels are
one-based. Log displacement is last-minus-first position
within a window; result-minus-guess is a difference of those displacement
vectors, not a direct measurement of every node's correction. The displayed
window duration is rounded to0.36s. Cross-stage plots/metrics are evaluated
separately; no approximate timestamp pairing or interpolation is used.

## Where it develops

There is no ground-truth trajectory to identify the very first wrong pose.
The first sustained speed escalation is localized to W754 onward, preceded by
heading and accelerometer-fit disagreement in W747–750. Earlier heading events,
including W414 at148.68s (-11.192deg), show this is not proof that all preceding
poses were correct or that W747 is the unique initiating fault.

| Window / relative time | End velocity norm [m/s] | Predicted / optimized displacement [m] | Evidence |
|---|---:|---:|---|
| W745 /267.84s |0.686|0.246 /0.246|Small0.002m displacement correction, yaw+0.196deg |
| W747 /268.56s |0.721|0.211 /0.246|Accelerometer-fit RMS0.509; multiple lower rematched NDT profile samples |
| W749 /269.28s |0.270|0.206 /0.110|Yaw-5.171deg, accelerometer-fit RMS0.826 |
| W750 /269.64s |0.266|0.125 /0.090|Yaw-7.085deg; geometry ratio0.439 |
| W754 /271.08s |0.890|0.159 /0.231|Start of sustained estimated-speed growth |
| W755 /271.44s |1.535|0.507 /0.386|Prediction already large; yaw-5.930deg |
| W756 /271.80s |1.873|0.700 /0.564|Yaw+8.292deg |
| W759 /272.88s |2.617|0.887 /0.900|First end-speed crossing of2.5m/s diagnostic trigger |
| W771 /277.20s |5.242|1.788 /1.774|Only0.022m displacement correction; end sigma0.00674m/s |
| W772 /277.56s |0.839|0.068 /0.358|Carry-speed fallback; prior sigma1m/s, successive-start speed4.925m/s |
| W870 /312.84s |7.647|See full CSV|Second, larger escalation; end sigma0.00758m/s |

The pre-loop snapshot already contains the long excursion. Its XY bounds are
[-43.611,16.004] by[-68.458,9.477]m and minimumZ=-49.289m. Final bounds are
[-43.616,14.632] by[-70.704,9.018]m, minimumZ=-49.076m. These are estimated
coordinates, not measured building-boundary or physical-height errors. Export
and loop closure therefore did not originate the large excursion, although
postprocessing changes the trajectory and does not repair it.

The pre-loop path's largest adjacent-node step is0.01731m over0.002352s at
313.14465s; peak adjacent-node speed is7.454m/s at313.16836s. Thus the retained
trajectory shows accumulated rapid motion, not a single multi-metre node teleport.
The final stage also has a separate short-timescale artifact: maximum adjacent
step0.03761m at143.06205s and maximum finite-difference speed20.076m/s at
143.26592s (0.03039m/0.001514s). This is a derivative of exported poses, not
physical velocity; it must not be substituted for the7.647m/s solver-state peak.
Its postprocessing origin is a separate unresolved issue, not evidence that the
much later long excursion was introduced at export.

## Supported propagation mechanism

1. The previous optimized endpoint velocity/covariance feed the next window's
   velocity prior. Its IMU-integrated translation becomes the initial prediction
   and motion-model reference. See scratch src/application/run_odometry.cpp
   prepare_velocity_state:215 and update_acc_term_after_window:243, and
   src/domain/window_steps.cpp integrate_window_translations:177.
2. Large predicted displacement is visible before each high-speed peak. W771's
   guess is(-1.514,-0.936,-0.170)m and result(-1.511,-0.913,-0.172)m. The solver
   stays near an already rapidly moving prediction. This establishes a mechanism
   for carrying the excursion forward, not which factor first created it.
3. stepped_poses in src/domain/scan_to_map.cpp:134 checks each proposed node's
   position against its predicted motion-model position, with a1m limit. It is
   not a limit on absolute speed, physical plausibility or accumulated drift.
   A large inherited prediction can therefore pass. The small W771 displacement
   correction is consistent with this; normal logs do not expose every trial node.
4. The separate max_prior_velocity_mps fallback is5m/s in the archived settings.
   It activates after W771 and repeatedly in the second episode. It changes the
   next prior selection/uncertainty; it does not undo already accepted poses or
   reconstruct a correct map. Tail-estimated previous velocity and carried
   endpoint velocity have separate eligibility guards.
5. Map insertion uses optimized poses. Thus erroneous accepted states can also
   influence subsequent matching. Map rebuilds occur after W882 andW1390; their
   timing excludes them as the initiators of the two earlier speed escalations.
   They are sliding-map maintenance, not a recovery/reset guarantee.

At W772, distinguish the preceding optimized endpoint speed (5.242m/s), the
separately rejected tail-velocity estimate (5.17062m/s), and the successive
window-start displacement rate (4.925m/s from the snapshot). Earlier live logs
gave about4.923m/s for the last quantity because start positions were rounded.

The native29big-jump count is rejected NODE updates: W874:15, W959:4, W964:10.
It is not29rejected iterations. There were0solver skips. These guards activate
after the first escalation and do not establish recovery.

## What this evidence cannot decide

- Association/map ambiguity, attitude–acceleration coupling, and the changing
  balance of projection, scan matching and carried priors remain alternatives.
  Rematched NDT profiles have lower sampled costs away from the final solution
  near onset, but their cost excludes other factors and changes associations;
  they do not prove the joint optimizer chose a uniquely wrong minimum.
- A small local covariance (about7mm/s at the peaks) is conditional internal
  confidence. It does not certify the correct association basin or physical
  velocity. The log alone cannot assign the runaway to that covariance.
- The ordinary printed motion-model directional stiffness omits experimental
  beta: when acceleration is active, its XYZ information is multiplied by0.1
  in scan_to_map.cpp:257–264. Do not use the unadjusted printed value as the
  implemented weight. Printed NDT stiffness is rematched finite-difference
  cost curvature, not the optimizer's positive-semidefinite Hessian.
- Final-call geometry ratios imply no projection suppression at W750/W754,
  suppression at W771 (weakest scale approximately0.12) and W864 (about0.43),
  and none atW870. They do not expose earlier iterations, so they neither
  establish nor exclude projection as an initiating cause.
- Snapshot motion_model_poses are overwritten with optimized poses by finish_run
  in run_odometry.cpp:896. They cannot reconstruct the missing incoming guesses
  or per-iteration factors. No detailed iteration trace was enabled for this run.

Raw IMU around268–272s has2000samples, maximum gap2.971ms, rotation-norm
maximum34.77deg/s and acceleration norm0.797–1.192g. Around307–316s it has
4500samples, maximum gap2.970ms, rotation maximum64.74deg/s and acceleration
0.792–1.245g. Full-input checks already found no gaps/false-offset stretches.
This supplies no evidence of a gross sensor dropout at onset; it does not
validate calibration, internal synchronization, extrinsics or the motion model.
The user-identified mirror/window reflections in the short apartment section
are not established as the cause of this later full-route excursion.

## Review and next evidence boundary

Local source research and independent Codex evidence review completed read-only
(the reviewer was not Sonnet). Review found no critical blocker. Its corrections
are incorporated above: times label window starts, and tail-velocity rejection,
optimized endpoint speed and successive-start speed are distinct quantities.
The user requested Terra 6.1 research after a clue emerged. That agent completed
a focused read-only investigation; root verified these primary sources:

- LIO-SAM checks optimized velocity and IMU bias before repropagation and resets
  its preintegration state on gross failure. Its 30 m/s velocity threshold would
  miss this event and is not a proposed dMapper setting. This is evidence for a
  distinct absolute-state diagnostic, not proof that resetting fixes our map.
  [Frozen LIO-SAM source, lines 401–405 and 437–453](https://github.com/TixiaoShan/LIO-SAM/blob/08af3f32f01725372d4269838dc44c19c6d9e76b/src/imuPreintegration.cpp#L401-L453).
- FAST-LIO discusses pose–velocity correlation and propagates the coupled state
  and covariance. This motivates inspecting attitude, rotated acceleration,
  velocity and covariance together. It does not establish a missing-correlation
  defect in dMapper from these logs.
  [FAST-LIO, sections II-B and III-B/C](https://arxiv.org/html/2010.08196).

Orientation–gravity coupling is a testable local hypothesis: a one-degree tilt
misestimate can project about 9.81 sin(1 degree) = 0.17 m/s² of gravity into a
horizontal direction. This is an illustrative calculation, not a measured tilt
error here. Wrong associations and inherited velocity remain alternatives. No
fix is selected from literature alone, and no sensor fault is inferred.

To distinguish the initiating factors, a proposed next step is one diagnostic
replay of this exact candidate/input/settings, with focused trace windows around
W408–416, W742–773 andW854–875. Capture incoming/outgoing carry and gravity,
prediction, per-factor signed RHS/Hessian, association support, projection and
accepted/rejected updates. Verify reproduction against existing pre-loop/final
trajectories and cloud before causal comparison. Existing full processing cost
was about50minutes, and trace adds overhead; a short cut would change map/history
and would not be an equivalent replay. This proposal requires user authorization
before instrumentation/build/processing. No new run has been started.

Current conclusion: propagation through an already rapidly moving prediction is
supported; the unique initiating factor is unresolved. High remains useful for
instrumentation design and causal trace interpretation if that next step is
approved. Routine replay monitoring would not itself require High.
