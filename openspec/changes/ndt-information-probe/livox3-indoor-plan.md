# Livox3 indoor recording: input checks and proposed replay

2026-11-13. Status: input checks completed; replay plan awaiting user agreement.
No mapper run started on Livox3. Sol6.1 medium is sufficient for these checks and
routine baseline replay; ask before deeper correction work needs High.

## Acquisition and capture changes
User: mounting unchanged; entirely indoors, floors3,2,1, no ground floor.
Start and finish in Wohnen, scanner resting on the same table in the same
direction for a few seconds. Exact repeated position and floor elevations are
not surveyed; do not turn approximate return into a precise closure PASS.

User reports daemon fixes for zero-valued first sensor timestamp; duplicate/late
point packet skipping within50ms, faults on larger backward jumps and sensor-timed
backward IMU jumps>50ms; finalization of valid buffered points/IMU after faults;
rejection of NaN/Inf IMU;2048-packet chunk bound; zero empty-LAZ bounds; confidence
policies legacy/low/strict (now low), expanded confidence/raw-tag/reserved-byte
packet diagnostics and rejection of unsupported headers when filtering enabled.
Layouts, units, axes and normal timestamps reported unchanged; no gyro-bias
correction applied. These are acquisition-agent statements, not a source-code
audit by this agent. Low policy is confirmed in this recording's timing metadata.

## Read-only checks
Original folder: C:/Users/User/Downloads/dLidar/raw data/livox/3.
Evidence: BASE/livox3-indoor/recording-check (BASE is the established phase0b
external experiments directory). Includes hashed input inventory, complete LAZ
stream check, packet summary, script, exact IMU-tool output and timing sidecar.

-253.2s,126611 IMU samples,501Hz median; zero non-increasing timestamps/gaps,
maximum step3ms. IMU health reports zero failures/warnings.
-Initial quiet block19s; gravity0.9999g. First-rest gyro bias[-.121,-.567,.061]
deg/s; last-rest[-.101,-.546,.061] at246s, change.02deg/s. These are raw biases,
not a calibrated sensor-noise model or reason to disable startup correction.
-42153826 LAZ points decoded, matching header; no nonfinite coordinates/times,
no timestamp decreases, stored point-time range0..253.223455s.
-527469 diagnostic packet rows, first relative stamp0ns; zero duplicate/backward
packet stamps, zero nonzero reserved-header rows. Timing reports zero lost samples,
stamp jumps, dropped duplicates/non-increasing/unmapped samples and nudges.
-Confidence policy low; reported rejected-confidence total133050. Categories can
overlap; do not sum category counts into unique rejected points or explain all
missing returns through confidence alone.
-Sensor clock chosen, IMU offset0. Minimum transport latency approximately
-.127ms at start/-.136ms at end; this is not internal synchronization calibration.
-Largest recorded acceleration4.18g and rotation145deg/s are descriptive extremes;
the health tool reports no failure. Do not call a motion extreme a capture bug.

Input is usable for the proposed baseline, with no claim of independently
calibrated timing/noise or exact surveyed endpoint. No algorithm-quality verdict
follows from recording health.

## Proposed next steps, not replay authorization
1. Run one complete original-data indoor baseline using the frozen Huber3 build
already used for full Livox2, alpha0.1/beta0.1, online gyro bias/startup correction
on, tracing off and the byte-identical baseline TOML. Outputs under a fresh
BASE/livox3-indoor/huber3-baseline. Preserve original recording files in place.
Cauchy3 failed full-route validation and is not the proposed baseline.
2. Check coverage and diagnostics, actual pre-loop/final endpoint displacement,
height evolution against the reported floors3/2/1 (visit order unconfirmed), and individual map continuity.
Provide the actual recomputed cloud and a simple trajectory view for user review.
Floor elevations and exact route/durations are not yet known; do not assert a
numerical height/closure acceptance from approximate acquisition notes.
3. Agree the next action after the user sees that cloud. If it fails, localize the
first affected segment from that run before any new numeric correction/replay.
If it succeeds, this establishes a useful indoor result, not proof that outdoors
alone caused Livox2 failure: environment, route and capture filtering all changed.
Perturbation robustness and production acceptance remain separate later gates.

## Prior work state
The earlier original-data Livox2 prefix replay was explicitly stopped by the user;
PID8008 terminated, wrapper exited1, partial outputs preserved. It was not a
completed/accepted run and must not resume automatically. User then requested
waiting for this indoor scan and making a plan together. That scope governs this
check; no old run permission is recycled for Livox3.

Independent Sol6.1 review supports this frozen experimental reference run and
requires separate odometry/final results, endpoint uncertainty, floor coverage
and matched beginning/end views. No route order or floor heights are assumed.
