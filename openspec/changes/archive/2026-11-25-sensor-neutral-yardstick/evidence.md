# Evidence ledger

## Latest decision, November 12

User explicitly accepted criteria-proposal.md with "yes i agree", then requested
continuation after the usage limit reset. The revised-limits gate is satisfied.
Historical statements below that no limits were accepted are superseded. Tool
readiness, provenance and evidence coverage remain prerequisites to verdicts.

Both agents' four verification skills now reference and reflect the agreed
contract; tools/check_agent_setup.py passes with identical copies. The historical
roadmap links to the superseding contract. Existing tool defaults are deliberately
not relabeled as compliant: diagnostic-readiness.md records reproduced false-PASS
and mismatched-trajectory examples, and restricts their current use to checked
raw evidence extraction. Task 2.5 remains open for a complete comparison workflow.

Updated 2026-11-12. This records observations, not candidate rankings.

## Completed prerequisites

Removal code/docs, golden TOMLs and manifests: e196692, e0b29ab, 8d230a1.
Release tests passed; all old manifest-tracked non-parameter files matched for
44_short and full44. See docs/agents/removal-validation.md for exact evidence.
Not pushed.

Turn baseline: exit0, 170.24 s. Draft run-health checks passed but included a
7.2-degree bend at27.7s. Raw rest registration was ambiguous (1.21 confidence,
threshold1.25), so scale, alignment, lever arm and internal timing remain open.
See docs/agents/turn-calibration-measurements.md. No calibration accepted.

## Reusable reference inventory

Root: C:/Users/User/Downloads/dLidar/raw data/livox/.

- runs_500hz/sweep/A_k7p08982_base plus six B_k7p08982_*: shipped family.
- runs_500hz/sweep/A_k1_base plus six B_k1_*: k=1 family (also floor changed).
- runs_500hz/sweep/C_own_base plus six C_own_*: own-noise family.
- runs_2/base.log and livox-500hz-diag.toml; bestk.log/toml; own.log/toml.
- runs_500hz/sweep/results.json and results_own.json retain earlier readings.

Independent research verified all listed result directories have session poses.
Shipped shared configuration keys match today's settings except diagnostics.
k=1 changes rotation floor to0.0005 as well as inflation. Own-noise uses ARW0.024
on all axes and floor0.0002105263158. Historical files retain removed accel keys.
Executable provenance is not established by these directories alone. No family
has been ranked under revised criteria.

## Tool-review findings requiring disposition

Research inspection found: sweep resume lacks input/config/executable identity;
failed runs/checks can exit0; seam comparisons truncate by index despite timestamp
warnings and the sweep scrapes their numeric output; run-health omits absent
metrics from judgment; its iteration-stop count uses observed maximum iterations;
its gyro-significance approximation ignores floor and axis projection. These are
findings to verify/address, not fixes already implemented.

## User decisions and pending gates

Direction A selected. Numerical changes on44 permitted if map quality does not
worsen. Both obsolete accelerometer modes removed with approval. On November12
the user explicitly chose revised limits before comparisons. No new numerical
limits accepted. Phase0a is incomplete and phase0b has not begun.

## Rest-start input preparation

Built existing cut_recording target and cut Livox/2 first60s into
out/phase0a-2026-11-12/livox2_0_60/. Successful exit; cut.log records30000 IMU
lines and10270298 copied points, re-read and verified. Raw imu_health.py reports
no timestamp reversals/gaps, but WARN for0.0s initial rest and unreliable handheld
noise estimates (3warnings). Thus input-readiness task2.1 remains unchecked:
starting at the recording's beginning is not proof it starts sufficiently still.
Resolve rest-detection/setup evidence before relying on it as the experiment
input. No candidate processing or comparisons were started.

### Follow-up: startup warning resolved, with limits

The warning above is retained as the tool's original output, not the final input
disposition. Independent Codex research (not Sonnet) traced it to the first
0–1 s block: gyro-X std 0.513758 degree/s versus cutoff 0.499659, only 2.8%
over. Other channels pass, as do the overlapping blocks from 0.5–1.5 through
5–6 s. The script requires a quiet run beginning at index zero and consequently
prints 0.0 s. Parent independently recomputed the first-block values.

First 500 samples have summed variance 0.915908 deg²/s², below the configured
startup gate 5.0. Startup upper medians are [0.111720, -0.204914, -0.125528]
degree/s; source src/application/lidar_odometry.cpp startup logic and historical
livox/runs_2/base.log agree. Gravity over first 5 s is 1.000197 g. Research
confirmed the cut's first 30000 numeric IMU rows equal the original rows.

Acquisition history is in the original Claude memory
C:/Users/User/.claude/projects/C--Repo-dMapper/memory/livox-mid360s-switch.md:
vertical mount, still first 0–5 s, route through stairs/halls/outside and back
to the room. Exact endpoint coincidence is not established. On November 12 the
user reiterated that the Livox is new and previous Claude testing found it likely
healthy. Nothing here establishes a Livox hardware defect.

Disposition: Livox/2 cut is usable for the planned startup-based diagnostic
experiment with the heuristic warning explained. It is not a stationary noise
calibration dataset. No detector threshold or application setting was changed.
Original timing0002.json records zero lost samples/stamp jumps and transport
latency -0.119616/-0.140409 ms at start/end; link that original metadata to the
cut, which does not carry its own timing JSON. Internal synchronization remains
a separate calibration question.

### Other input checks

Commands: python tools/imu_health.py on original livox/1 and 44_short. Saved full
tables in out/phase0a-2026-11-12/livox1-imu-health.txt and
44-short-imu-health.txt. Both commands exit 0 with WARN, not aggregate PASS.

- Livox/1: 47258 samples/94.5 s, median 1.997 ms (501 Hz), zero reversals/gaps,
  initial quiet span 6.5 s, gravity 1.0006 g, startup-rest mean
  [-0.093,-0.508,-0.021] degree/s, no false-offset stretches. One warning for
  nonwhite/handheld noise. Earlier acquisition notes describe handheld vertical
  mounting and roughly 4.5 s stillness; heuristic duration is not exact ground
  truth. timing0001.json: zero lost/jumped stamps, transport latency
  -0.129990/-0.130972 ms. Usable with handheld-noise caveat; exact return unknown.
- 44_short: source is the synchronized historical 30 s regression cut, containing
  imu0001.csv and lidar0001.laz. 26304 samples, median 1.124 ms (890 Hz), zero
  reversals, 13 gaps above 2.5 times median. Largest interval 402.4137 ms begins
  at relative 2.599502 s; same gap in Unix stamps. Initial quiet span 3.0 s,
  gravity 0.9952 g, first-rest bias [-0.140,-0.764,-1.060] degree/s. No detected
  false-offset stretch in this cut. Three warnings: gaps, bias and noise.
  Documented affine restamp scales intervals by 1.0000983, so it cannot create
  this large gap from regular samples. Missing samples versus clock pause is
  unresolved. Usable as the established regression/stress input with caveats,
  not a healthy-sensor calibration reference. No timing JSON accompanies the cut.

Input presence/health verification does not complete historical executable or
input-hash provenance for candidate reuse (task 4.1). Neither short cut has a
confirmed closed route; do not enable closure verdicts for them.
