# Existing-data decision — 2026-11-13

## References

All30 checked recording/bundle/binary/config files match their retained hashes.
Implementation checkout: out/phase0b-worktree, branch codex/phase0b-ndt-information,
clean base c52b25ff4b2bae5bdf4c2c63e5c5776c51543784. Frozen Huber reference:
cbca36d2747cfcd1fc3ceac3d906bd221080d8f8. Subsequent commits add diagnostic tools
and opt-in Cauchy; Cauchy remains disabled for this intervention.
The first check accidentally compared raw TOML bytes with the runner's normalized
text identity. Correcting the check to use settings_bytes_sha256 gives all matches;
no reference file changed. Retain the initial failed manifest. Both current frozen
settings files hash to2a69a32187a507f659cccf091d800347dc763d6d685957a441005b1f4ba00a55.
W415/W754 evidence is reused, not rediscovered or rerun.

## Later runaway

Independent read-only review of full-livox2-huber3/run.log:6882 onward:
W904–915 starts325.08–329.04s show endpoint velocity~2.571->5.154m/s.
Pose-versus-prediction corrections remain mostly centimetres (max~0.126mW914),
endpoint yaw corrections remain below3degrees, and residual gyroZ bias stays
~0.307–0.335deg/s. Growth precedes the map rebuild after W914 (line6969).
W914 velocity-prior sigma is already1m/s; lower chi-square is not better accuracy.
W916's5m/s guard rejects previous pose-derived velocity5.13281m/s (line6978),
explaining the subsequent speed reduction as an intervention.

Prediction/state carry is supported during escalation; the initiating cause and
recurrence of W415 thin-cell dominance remain unresolved. Negative reassociated
matched-only finite-difference stiffness does not prove negative assembled curvature.

Snapshotv1 cannot provide an exact later restart: it lacks pre-LM model poses,
per-window gravity/conversion, offset prior covariance/history and complete
incoming sequential carry. Needed: initial poses/model, bias/velocity/offset
priors and initialization, gravity/to_mps2, rest/previous-use decisions, map and
points. Sequential reproduction additionally needs velocity validity/time/cov,
bias/gravity/offset carry, future IMU increments and map-history points/distance
through W914 rebuild. Rounded logs cannot substitute. No scoped capture started.

## Stationary surfaces and selected engineering candidate

tools/stationary_plane_noise.py reads raw Livox3 sensor-frame points from1–15s
of the verified19s quiet startup; user reports table rest in Wohnen. Fit1–7s,
fixed plane/footprint evaluation7–11/11–15s, stride10, seed20261113. No alignment,
deskew or map fitting. Automatic patches are not independently identified walls.

|Patch|Held-out1.4826MAD, two intervals (mm)|Range p10–p90 (m)|Interpretation|
|---|---|---|---|
|0|4.20/4.11|1.83–3.15|Dominant plane|
|1|3.88/3.85|1.03–1.93|Dominant plane|
|2|7.32/7.38|0.54–1.50|Close parallel patch; fixture/mixing possible|
|3|4.75/4.71|3.21–3.89|Dominant plane|
|4|27.44/28.11|1.43–3.44|Broad/mixed; unsuitable noise calibration|

Selected medians remain within~0.5mm of the fitted planes. JSON retains counts,
normals/centers/footprints, incidence, time splits and range. Fixed+/-50mm bands
truncate tails; physical identity and correlation remain uncertain. Near-range
stationary repeatability cannot establish moving/outdoor total residual noise.

Predeclare **sigma=0.006m** for the local W415 gate. Motivation: observed4–5mm
planar spread and a conditional two-return uncertainty heuristic. sqrt(2)*4.1mm
is~5.8mm, but MAD scales do not generally add as Gaussian variances, independence
is unproved, and legacy covariance already includes scatter around its first-point
mean. Thus this is empirical regularization, not a calibrated missing noise term.
Changed Livox3 recorder/filtering further limits transfer to Livox2.

At the frozen W415 state it affects the5.25mm minimum width, not7.71/10.36mm.
Unrobust information in that direction becomes(5.25/6)^2=0.765625 of baseline;
Huber reweighting can increase other directional contributions. A small effect
does not refute all confidence hypotheses; local benefit cannot establish a full
fix. Do not enlarge sigma after observing results without a new rationale/approval.

Independent Sol6.1 evidence review supports this limited exploratory value and
required the MAD/independence/first-point covariance qualifications above. It
confirms the later cause remains unresolved. No alternate algorithm branch chosen.

Evidence: experiments/phase0b-2026-11-12/open-closed-existing-data/ contains checked
reference manifests, stationary-planes.json/log and archived scripts. No replay or
checkpoint optimization was launched; no higher effort required for these checks.
