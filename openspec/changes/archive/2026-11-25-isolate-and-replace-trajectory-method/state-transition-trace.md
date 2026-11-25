# Existing-data transition trace (task 1.2, reviewed)

This table separates calculation stages. It uses the frozen Livox4 Huber3 run,
not the later gyro4x sensitivity run. The numbered window logs round vectors
to 1 mm and velocities to 1 mm/s. The independently checked decompositions are
in `out/livox4-prediction-vs-matching.json` and
`out/livox4-prediction-decomposition.json` (hashes in evidence-inventory.md).

| Boundary | Observed transition | Calculation / retained state | Still unknown |
|---|---|---|---|
| Incoming W301, 108.00 s | Previous joint velocity [-0.234,-0.288,0.142] m/s; predicted move [-0.045,-0.089,0.011] m over 0.358 s. | `prepare_velocity_state` chooses prior; `integrate_window_translations` uses that velocity, transformed raw acceleration and carried gravity. | Whether the prior already departs from physical motion; raw IMU and pre-LM attitude/gravity contribution not fully saved. |
| W301–320, 108–115 s | Predicted within-window lengths total 14.921 m; optimized lengths 14.227 m. Corrections oppose predicted displacement in 18/20 windows. | Prediction is frozen in `motion_model_poses` before LM. NDT/IMU joint solve updates poses, bias, velocity and acceleration offset; outgoing state feeds next window. | Which earlier update first corrupted the carried state; these sums omit between-window gaps and are not a travelled path. |
| Incoming W320, 114.84 s | Previous joint velocity [2.916,3.584,-0.363] m/s contributes [1.044,1.283,-0.130] m of [1.091,1.364,-0.151] m predicted displacement; integrated-acceleration remainder [0.047,0.081,-0.021] m. End joint speed 5.086 m/s. | `integrate_window_translations` applies `a=(R*raw_acc-carried_gravity)*to_mps2`; most of this window's displacement follows incoming velocity. | The remainder is a fitted decomposition, not measured physical acceleration or an isolated tilt error. |
| W321, 115.20 s | Pose-derived prior speed 5.054 m/s and joint end-speed 5.086 m/s exceed the 5 m/s guard. The next velocity prior falls back to zero with initial 1 m/s sigma; optimized end speed about 0.394 m/s. | `estimate_previous_velocity` and `prepare_velocity_state` take separate guarded paths. | The speed drop is a programmed reset, not evidence of physical recovery. |
| W397, 142.56 s | -11.704° last-node yaw correction; translation from guess about 0.142 m. | `stepped_poses` rejects translation >1 m from the frozen guess but has no corresponding angular bound; successful LM solve can commit this correction. | Whether the matching update is physically wrong and whether it began the later drift. The guard weakness is not proof of initiating cause. |

Livox2 is a distinct later-route case: `../ndt-information-probe/early-heading-cause-audit.md`
isolated three influential W415 outdoor NDT cells in one local angular solve;
that ablation does not explain the full roughly 62 m return mismatch. Its later
W904–916 state was not captured completely. Do not collapse Livox2 W415 and
Livox4 W301–320 into a single demonstrated cause.

Snapshot v1 is insufficient for exact pre-LM attribution:
`finish_run` overwrites `motion_model_poses` with final poses before saving;
incoming attitude/gravity and full prior history are not stored directly. The next
investigation must distinguish an excessive prior entering a window from
acceleration added by orientation/gravity integration and from the joint solver's
outgoing velocity.

## Independently checked velocity update path

The distinction between displacement and velocity is essential. In the active
non-rest path, `prepare_velocity_state` initializes BOTH velocity unknowns to
the carried prior. `integrate_window_translations` integrates a local `v` to
place predicted poses, but never writes that integrated velocity into
`window.velocity.end`. The accepted solve in `scan_to_map.cpp` adds
`h_x[ie+r]` to that end velocity. `update_acc_term_after_window` then saves it
as the next window's prior before the tail pose propagation.

Consequently a small pose correction does not mean the optimizer leaves the
carried velocity unchanged. At W320, the prior is [2.916,3.584,-0.363] m/s,
the solved start is [2.943,3.543,-0.428], and the solved end is
[3.198,3.939,-0.355]. The accepted optimizer updates add a net
[0.282,0.355,0.008] m/s to the end velocity. At W301 the corresponding net
increment is [0.490,0.297,-0.274] m/s. These values are rounded log readings.

The non-rest end-velocity residual ties that unknown to the last two anchor
positions and the mean transformed acceleration from their midpoint to the
window end. With its sign convention:

`r_end = (pQ-pP)/(tQ-tP) - v_end + (mean_acc-offset)*(tN-mid)`.

This factor couples velocity to poses, rotations and acceleration offset. The
full joint solve includes LiDAR and priors through those shared variables.
The observed feedback path is therefore:
accepted joint end-velocity update -> carried prior -> next predicted poses ->
next joint optimization. This identifies where velocity growth is committed;
it does not yet establish which residual or upstream attitude error initiated it.

Independent reviewer confirmed this code path and the two numerical increments.
Existing snapshot Euler angles and exact saved bias may permit an approximate
pre-LM orientation reconstruction; evaluate its uncertainty before requesting
a capture. Exact replay still requires validated original state and map.
