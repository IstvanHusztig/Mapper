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

Status: prepared; runtime state is recorded in the external run.json.
