# Bounded NDT influence correction

2026-11-13: user requested fixing the diagnosed early bend, then asked whether
the supplied LAZ/CSV could be faulty. Work continues in the isolated checkout.

## Input check

All89,002,885 LAZ points decode, matching the header. No nonfinite coordinates or
timestamps. In148–151s,364,513 retained returns have no decreasing timestamps or
nonpositive ranges. Stored range agrees with XYZ norm within0.00000191m; integer
raw range agrees within0.000500m, consistent with millimetre quantization. Packet
spacing remains0.480ms; retained returns have a longest10.170ms gap. The recorder
reports no lost IMU samples or stamp jumps. Existing full IMU health reports no
non-increasing stamps, gaps or false-offset stretches, and0.20deg/s first/last
rest bias change. Noise estimates remain handheld upper bounds.

The packet table's low_confidence field sums358 in this interval; its precise
semantics are not established here, so this is not labelled358 bad packets.
These checks exclude obvious file-format/nonfinite/timing failures in the
tested data. They do not certify calibration, internal synchronization, or the
physical validity of reflected/dynamic returns. The recorded acquisition was
vertical mounting, initial rest and return inside apartment56; exact endpoint
coincidence is unknown. No evidence here establishes a defective sensor.

## Correction under test

For squared model-relative residual q=rᵀI r, use Cauchy cost
rho(q)=c² log(1+q/c²), with IRLS weight c²/(c²+q), c=3.
Weight each NDT Hessian and RHS once in both grids, preserving all cells and
the IMU, bias and motion machinery. No scene-specific deletion or range cutoff.

Why this addresses the finding: for a fixed nonzero residual along an increasingly
precise but inconsistent plane, Huber's pull still grows as1/sigma. Cauchy's pull
tends to c²/residual instead. The existing Huber and disabled paths remain exact.
Costs and support diagnostics identify the selected loss explicitly; both losses
cannot be enabled together. The experimental opt-in is
DMAPPER_DIAGNOSTIC_NDT_CAUCHY3=1, with the Huber flag unset. No shipped defaults change.

This bounds a demonstrated failure mechanism; it does not repair map statistics
or association identity. Cauchy can also weaken useful geometry and has a
nonconvex residual cost. Cutoff3 is a controlled first test, not calibrated sigma
or a universal accepted setting. Sol6.1 independently reviewed this design.

## Verification contract

Tests cover bounded inconsistent-plane influence, finite/invalid inputs, loss
gradient and coordinate invariance, both-grid H/RHS scaling and PSD, matching
cost diagnostics, serial/parallel determinism, strict opt-in and mutual exclusion.
Build/test Release; no Debug-specific/lifetime changes are introduced.

Require exact reproduction of all100 retained W415 Huber3 control iterations
before one all-cell Cauchy checkpoint from identical state. Report raw-gyro
agreement, displacement, velocity, bias and support; do not compare unlike loss
costs as if they were one objective. Full-route geometry and perturbation gates
remain NOT MEASURED for this correction. No new full-route replay yet.

## Paused for sensor-agent audit (user request, 2026-11-13)
User requested a pause to answer questions from the sensor-reading/code-audit agent.
Cauchy implementation is UNCOMMITTED in scratch. Release build completed;
7 diagnostic cases/817 assertions passed. Full suite:130/131 passed, same prior
17.9039mm corridor failure as archived Huber baseline. One premature CTest launch
was blocked by the linker; retained separately, not counted as a test result.
The Cauchy checkpoint was stopped at user request during raw seed loading;
process terminated, exit-1, no Cauchy optimizer result or acceptance. Preserve
BASE/full-livox2-huber3/early-heading-audit/cauchy-w415 and its log; no retry into
that directory. No full replay started. Await sensor agent questions, no further
implementation/runs until user resumes. Prior W415 cell-ablation evidence remains
valid; Cauchy efficacy is NOT MEASURED. Inputs unchanged.

## Resumed by user (2026-11-13)
User requested continuation. Frozen Cauchy binaries verified against their saved
SHA256 manifest; current source diff matches the frozen source patch. No other
mapper/reconstruction active. Resumed the exact-control-gated W415 check in fresh
cauchy-w415-resumed, preserving the cancelled directory. No full-route replay.
Sensor-agent questions file is absent; no external sensor audit result assumed.

## Completed saved-window correction check

Fresh `early-heading-audit/cauchy-w415-resumed` completed exit0. Every emitted
record of the100-iteration Huber3 control reproduced exactly before the Cauchy
trial. Both runs used identical initial poses and match records. All map cells
were retained; no IMU/bias settings or other model terms changed.

| Measure, W415 | Huber3 control | Cauchy3 |
|---|---:|---:|
| Endpoint orientation away from initial prediction | 9.142869° | 1.590224° |
| Local relative rotation disagreement with raw gyro | 9.188469° | 1.639269° |
| Estimated endpoint velocity norm | 0.502139m/s | 0.644160m/s |
| Final indoor matched observations | 8891 | 8377 |
| Final outdoor matched observations | 7085 | 6498 |

Maximum node-position difference between outputs is0.070786m. Velocity and
position changes are descriptive, not known improvements. Matched observations
across the two grids can duplicate returns. Reduced correspondence support and
robust weights are not automatically better or worse geometry. Costs of different
losses are not directly comparable. Sol6.1 independently checked the script and
native validation and confirmed the local gyro-consistency improvement.

The general influence correction is implemented and validated on this failure
window. It is not yet a verified correction for the full Livox2 cloud. Full-route
map quality and perturbation robustness remain NOT MEASURED. No production
defaults changed and no full replay has started.

Artifacts: `early-heading-audit/compare_w415_cauchy_resumed.py/.json`, resumed
checkpoint traces/validation, source patch and frozen `bin-cauchy/dMapper.exe`.
Release build and7 diagnostic cases/817 assertions passed. Full suite130/131,
with precisely the existing17.9039mm corridor failure also present in the archived
Huber baseline; not described as a passing full suite. No Debug-only changes.

Next concrete validation: one full original Livox2 run using the frozen Cauchy
executable, unchanged Huber-control TOML (including online bias enabled),
alpha0.1/beta0.1, Cauchy3 enabled, Huber/trace flags cleared, fresh output
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-cauchy3`.
Full-run launch remains separate from this completed bounded check, respecting
the user's instruction to stop before new runs. The old Huber approval already
produced its full run and is not recycled as approval for this new candidate.

## Full Cauchy replay authorized and launched (2026-11-13)

User explicitly requested the full replay. One fresh run is active in
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-cauchy3`.
PID23284; native source c52b25f; frozen candidate copied from the reviewed bundle.
The Huber-control TOML is byte-identical, alpha0.1/beta0.1, online gyro bias enabled;
Cauchy3 enabled and inherited Huber/trace flags removed. Original Livox2 is input.
`preflight.json` pins raw files, executable/DLLs, settings, runner and analysis
helpers. Preserved Huber reference diagnostics include the W798 coverage gap.

Runner `out/full_livox2_cauchy_replay.py` is archived with the run. Its real-health
selftest passed NumPy array serialization, atomic replacement, failed-object/NaN
preservation and environment checks. Sol6.1 independently reviewed the runner and
reference schemas: no launch/collection blocker. No retry is authorized by this
runner. Native completion is saved before expensive collection. Accuracy remains
unaccepted regardless of health/comparison completion.

Next: monitor, collect complete odometry/final outputs and evaluate return error,
excursions and cloud geometry. No additional replay or parameter sweep starts
without another user decision. High effort is unnecessary for routine monitoring.

User proposed a new indoor-only scan down to the first floor. This would provide
an additional observation separating indoor stair/floor transitions from outdoor
matching, rather than reproducing the same route. Suggested unchanged mounting,
marked same start/end in apartment56, and a still pause at both ends. It is not a
prerequisite for the active replay; no new recording has been received.
