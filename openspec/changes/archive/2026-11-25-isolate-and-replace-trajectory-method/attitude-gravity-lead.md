# Existing-data attitude/gravity decomposition

Independent review completed 2026-11-13. Analysis only, no mapping replay.
Reused saved AHRS Euler angles, previous post-tail orientation, exact saved
gyro bias, raw acceleration and timestamps to approximate pre-LM orientation.
Probe: `out/isolation_orientation_probe.py` and `.json`, including input hashes,
snapshot validation and exclusions for stationary/reset windows.

The active frozen cbca36d code uses scale = 9.80665 / norm(carried_gravity).
Thus gravity and scale can be inferred from the logged predicted displacement,
known incoming velocity and reconstructed rotated acceleration. Exactly one
positive scale solution exists for each included window. This inference fits
the prediction, so it is not an independent measurement of gravity.

| Window | Attitude/gravity disagreement contribution XYZ (m/s²) |
|---|---|
| 295 | [0.074, 0.092, -0.001] |
| 301 | [0.744, 0.461, -0.042] |
| 306 | [1.119, 0.643, -0.090] |
| 313 | [1.190, 0.593, -0.097] |
| 320 | [0.929, 1.029, -0.102] |

At W320 total effective predicted acceleration is approximately
[0.738,1.266,-0.331] m/s². Removing the above contribution leaves the
AHRS-gravity-removed proxy [-0.191,0.237,-0.229] m/s². This identifies a
substantial persistent horizontal term around the buildup. AHRS is not ground
truth; no physical acceleration or full causal attribution is established.

First-node axis-angle yaw corrections reconstructed this way agree with the
logged corrections within 0.000186 degrees. This is a useful check, not a bound
on full attitude error. Position-log rounding contributes about 0.0078 m/s² per
axis; other reconstruction uncertainty remains unbounded. The reviewer and
primary agent checked the production scale formula and yaw convention directly.

The joint optimizer creating outgoing velocity is ordinary estimation behavior,
not itself a defect. Freezing or replacing that velocity is not justified by
this finding. The next bounded question is why attitude/gravity disagreement
grows between W295 and W301 and how its contribution enters both prediction
and the joint accelerometer residual. Existing-data inference should be exhausted
before requesting a new capture. No candidate correction or new run is selected.
