# Phase0a readiness, 2026-11-12

Ready for a bounded diagnostic experiment, NOT candidate acceptance.
All20 tasks complete; no change archived or pushed. User approved the nine-run
phase0b plan while the final verification run completed.

Current identified Livox1 base and decimation0.029 runs completed successfully
in393.76s and407.69s. Both have261 complete logged windows and46980 exactly
matching stage timestamps. Pre-loop displacement max18.670800m,p9517.319826m;
final max18.459663m,p9517.091058m. Both final trajectories exactly reproduce
their corresponding historical outputs. Independent Codex review recomputed
metrics and input/build/tool/output fingerprints and confirmed the result.

The family remains INCOMPLETE: five members were not collected. The two-run
counterexample establishes spread AT LEAST these values and disproves the0.10m
universal short-input target; it cannot establish a full noise floor or rank an
accepted correction. No new calibration, map-accuracy or shipping claim.
Full matched families, fixed map regions/coverage, held-out confirmation, other
inputs and the user's visual check remain acceptance gates.

Evidence: out/phase0a-2026-11-12/current-results.json and
livox1-current/results.json. Clouds retained in livox1-current/base and dec_minus.
Next: separate OpenSpec change ndt-information-probe, approved9-run scope.
