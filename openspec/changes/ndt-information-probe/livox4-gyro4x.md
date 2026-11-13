# Livox4 stronger gyro comparison

2026-11-13. User explicitly requested another full scan 4 replay trusting the
gyro more overall, alongside the interface proposal. One replay is authorized;
no sweep, algorithm change or default adoption is authorized.

Use the identical frozen cbca36d Huber3 executable and Livox-vertical settings
as livox4-indoor-outdoor/huber3-baseline, with exactly two settings changed:

| Setting | Reference | Candidate |
|---|---:|---:|
| rotation_sigma_inflation_k | 7.0898154 | 3.5449077 |
| rotation_sigma_floor_deg | 0.0035449077 | 0.00177245385 |

For active time-scaled rotation sigma, halving both branches of
max(k * ARW * sqrt(dt), floor) halves sigma on all axes/edges and multiplies
the rotation residual information by four. Independent code review confirmed
this against the frozen source. This is a sensitivity experiment, not a new
sensor-noise calibration. Online gyro bias and its priors remain unchanged;
stronger rotation information can still affect their joint solution. It does
not directly change translation weights or enable consistency gyro factors.

Raw input remains C:/Users/User/Downloads/dLidar/raw data/livox/4.
Output: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-indoor-outdoor/huber3-gyro4x/.
Runner: out/livox4_gyro4x_replay.py, archived in the output bundle. It asserts
the parsed TOML differs only in those two values, checks input/build hashes,
disk/process ownership, refuses output reuse and performs no automatic retry.
Expected runtime about 30 minutes, based on the 1798.64-second reference.

Collect native completion, log coverage, odometry/final endpoints and trajectories
separately. Inspect W301-320 prediction escalation and W395-410 angular changes;
do not confuse a speed reset with recovery. Preserve missing diagnostics.
Provide the new cloud for visual review. A single comparison cannot establish
robustness or adoption; perturbation runs need separate authorization. Physical
region quality not yet measured must remain explicit.

## Completed comparison and visual review

The one authorized native replay finished (exit 0) in 1995.14 s. All 1016
windows completed; frozen input/build/settings hashes and artifacts verified.
The output cloud is `result/scan_lio_merged.laz` (305,791,452 bytes). Diagnostic
coverage is complete, but run-health exits 1 for investigation triggers; neither
its exit nor the user's look is an independently surveyed accuracy measure.

| Reading | Huber3 reference | Gyro4x |
|---|---:|---:|
| Odometry start/end separation | 36.542 m | 44.791 m |
| Final start/end separation | 36.717 m | 44.394 m |
| Bends >3 degrees | 11 | 14 |
| Speed windows >2.5 m/s | 8 | 27 |

The user reports that the gyro-weighted cloud is also a mess. Same-table return
is known from acquisition notes, so the endpoint separation remains a serious
failure signal, though it is not a whole-map accuracy metric. Odometry curves
diverge by at most 37.000 m and final curves by 35.418 m; timestamps/counts
match (182880 poses each). First >0.5 m separation is 108.66 s odometry and
108.84 s final. A fourfold stronger relative-rotation constraint did not solve
this recording and worsened these specific triggers and return separations.
Do not adopt this weighting as a default or infer gyro quality is the root cause.
The isolation-first plan remains the next algorithm step. No further replay is
authorized by this comparison.
