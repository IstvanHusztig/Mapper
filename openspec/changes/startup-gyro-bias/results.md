# Results (2026-10-18, recording 44 + s1/s2; parked, not shipped)

The start-up bias is accepted on 44, s1, s2 (median −0.08 / −1.54 / −0.67 °/s; quarter spread ≤ 1.78 °/s, tilt 0.5°,
scatter ≤ 1.91×); the online estimate no longer jumps to −3.3 °/s, but windows 1–2 still overshoot by 0.5–0.8 °/s
(residual −0.8 / −1.1 °/s, settled about −0.3).

| measure (s0 / s1 / s2) | shipped | new |
|---|---|---|
| start room vs whole floor (plan_fit) | +0.93 / +0.26 / +0.56° | +0.42 / +0.69 / +0.36° |
| façade out | −3.63 / −2.81 / −4.49° | −2.28 / −3.45 / −2.68° |
| stair max | 9.7 / 12.3 / 16.4 cm | 7.6 / 5.4 / 6.8 cm |
| railing NN | 3.5 / 5.2 / 6.5 cm | 5.1 / 10.2 / 9.1 cm |
| revisit pairs | 8 / 10 / 11 of 11 | 7 / 6 / 6 of 11 |
| pair_diff | 3.0 / 2.2 / 2.9 cm | 2.8 / 2.8 / 3.6 cm |

Mixed: not shipped. The wall the user marked (start room, lower-left flat) turned out not to be a pose error: its
off-plan line is a stationary object seen at 0–5 s, 5–7.5 s and 276–280 s at the same place (0.8–2.6 m from the
sensor), next to a door swing on the plan (an open door leaf?), and the start room's plan fit includes it.

Parked until the user's new recording (user, 2026-10-18: "I will redo the whole scan in the coming days, let's stop
guessing on this sample"). The stand-still test itself (tasks 1.x) is implemented and tested; re-evaluate (2.1) on the
new recording.
