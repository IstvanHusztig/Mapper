# Independent Sol 6.1 audit

Requested by the user after stopping work, November 13, 2026. Conducted by a
separate Sol 6.1 agent through read-only inspection. No optimizer, build, fix or
additional experiment was executed. This report records its findings; the full
cloud goal remains unmet.

## 1. Definite collector regression

`tools/run_health.py:102` returns a NumPy array in `speeds`. The new
`out/full_livox2_no_online_bias.py:77` stores that object in the run record, then
its save function at line20 opens the destination and calls plain json.dump.
Serialization fails after partial output. The exception handler at line124
repeats the same unsafe save and truncates run.json too. Its current contents
end at the speeds field. Native cloud outputs remain intact.

The earlier runner reused an array-aware serializer in the archived
`motion-prior-probe/motion_prior_probe.py:45`. Replacing that helper introduced
an avoidable regression. The new selftest at line128 exercised settings and
environment only. A test using existing health output would have caught the
serialization failure before the approximately40-minute experiment.

## 2. The experiment itself failed

The native log completed successfully in2395.60seconds. Its saved health report
shows82 heading-bend triggers,5 above10degrees,164 speed triggers, minimum
height-87.34m and a209.94m window-start endpoint proxy. Earlier Huber3 had22,
1 and12 corresponding triggers and minimum odometry height-7.87m.

Disabling online bias estimation was harmful under these conditions. These are
not exact surveyed closure measurements. Candidate W814 conditional uncertainty
remains unresolved; absent bias rows are expected with the option disabled.
The original partial records must be preserved, not silently repaired.

Evidence is under:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-huber3-no-online-bias/`.
The native completion marker is in run.log:11164; the complete text health
report is run-health.txt. No mapper rerun is needed to establish this outcome.

## 3. Disproportionate investigation of a secondary problem

The exact postprocessing replay was useful: it separated loop closure from
consistency and bounded consistency's position changes to0.331m. The subsequent
native exporter, exact final-control export, three82.7-million-return clouds,
geometry tools and additional factor research were disproportionate to the
unresolved roughly62m odometry placement problem. Valid evidence was produced,
but much of that work had limited likelihood of fixing the dominant failure.
The auditor could not independently quantify all token expenditure or delays.

## 4. Bias removal was a hypothesis test, not an established correction

Large AHRS-relative orientation disagreement and changing bias estimates
justify investigating their interaction; they do not establish that bias
initiated the drift. Fusion AHRS includes accelerometer feedback and is not
independent attitude truth. Calling the experiment a correction test overstated
the evidence. Its negative result rules out this particular removal as a remedy,
not every bias-model or rotation-prior defect.

The auditor checked the suspected residual-family switch in scratch
`src/domain/scan_to_map.cpp:853`. The same Euler relative-pose assembly is used;
bias corrections and couplings are conditional. No evidence was found that the
flag selects a different rotation residual family.

## 5. Incomplete source reasoning

Two research interpretations inferred absent rest handling/acceleration
integration from disabled TOML flags. The complete call chain contradicts
them: always-on mode enables rest detection and later moving-window integration
at scratch `src/application/run_odometry.cpp:1043`. Root corrected these claims
before execution, but the incomplete reasoning still wasted effort and could
have directed the wrong experiment.

This bounded audit found no proven numerical defect explaining the full mapping
failure. Claiming one would repeat the unsupported causal inference.

## Recommendation and stop boundary

Only if the user later authorizes continuation: recover a separate collection
report from intact outputs, preserve damaged originals, and record the failed
bias-off trial. No new run is needed. Renewed mapping work should identify one
specific violated numerical contract near early divergence using existing
evidence before another full-route experiment or tooling branch.

All implementation and experiment work remains stopped. No OpenAI investigation,
support ticket or escalation has been submitted. This local audit does not
establish an OpenAI service/model incident.
