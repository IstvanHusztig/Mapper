# Phase 0b: nine-run diagnostic results

2026-11-12. All nine approved runs completed, exit0; no global setting accepted.

Source: scratch commit4d1ea34 on base8d230a1. Shipped code/settings unchanged.

## Run readings

All effective configs match their frozen dataset controls. All runs logged100 iterations per window: iteration-cap evidence, not convergence. Raw conditional cohorts differ; these medians are descriptive, not directly matched comparisons.

| Recording | Weight | Bend max deg | Speed max m/s | Jump median mm/s | Sigma median mm/s | Chi2 median |
|---|---:|---:|---:|---:|---:|---:|
| livox2_0_60 | 1 | 23.732 | 2.742 | 41.99 | 3.75 | 181.55 |
| livox2_0_60 | 0.3 | 8.862 | 0.774 | 40.12 | 4.10 | 105.10 |
| livox2_0_60 | 0.1 | 6.385 | 0.691 | 40.62 | 4.40 | 86.30 |
| livox2_0_60 | 0.03 | 11.352 | 5.723 | 50.13 | 4.80 | 75.95 |
| livox2_0_60 | 0.01 | 22.988 | 8.118 | 19.95 | 5.90 | 1.15 |
| livox1 | 1 | 18.027 | 1.151 | 49.06 | 3.80 | 183.25 |
| livox1 | 0.1 | 24.686 | 9.908 | 140.75 | 4.60 | 340.05 |
| 44_short | 1 | 2.104 | 1.281 | 55.59 | 3.90 | 198.20 |
| 44_short | 0.1 | 1.919 | 1.278 | 38.01 | 5.10 | 53.30 |

44_short control has ambiguous conditional eligibility from rounded ratio logging; observed rows retained, not a complete uncertainty-coverage pass. Its402ms input gap restricts it to regression use. All other required run evidence complete. Solver warnings/dropped slices and full bias/loop/coverage fields are preserved in summary.json and run records.

## Matched conditional windows

| Recording / weight | Shared windows | Jump control -> candidate mm/s | Sigma control -> candidate mm/s | Chi2 control -> candidate | Axis angle median deg |
|---|---:|---:|---:|---:|---:|
| livox2_0_60 / 0.3 | 15 | 43.61 -> 32.59 | 3.80 -> 4.00 | 197.20 -> 71.40 | 0.79 |
| livox2_0_60 / 0.1 | 13 | 40.37 -> 35.79 | 3.80 -> 4.30 | 165.90 -> 79.10 | 1.63 |
| livox2_0_60 / 0.03 | 14 | 41.99 -> 39.65 | 3.80 -> 4.70 | 181.55 -> 79.50 | 1.12 |
| livox2_0_60 / 0.01 | 9 | 38.69 -> 16.28 | 3.80 -> 4.90 | 118.30 -> 4.90 | 5.89 |
| livox1 / 0.1 | 25 | 58.39 -> 59.16 | 3.80 -> 4.50 | 281.80 -> 162.80 | 9.26 |
| 44_short / 0.1 | 21 | 55.59 -> 38.01 | 3.90 -> 5.10 | 198.20 -> 53.30 | 0.80 |

The common all-five Livox2 cohort contains only7 windows, including startup1/2. Weak-axis direction can change; matched IDs/times do not guarantee identical projected quantities. Conditional selection is not independent calibration. Full per-window vectors, axes, unmatched IDs and hashes are retained in matched-diagnostics.json.

## Frozen-region cloud readings

Local side-wise XY PCA orthogonal MAD, across frozen control-selected patches. Candidate points remain included even when they leave the extraction band. These are fitted-point diagnostics, not independently surveyed wall thickness or held-out accuracy.

| Recording / weight | Patches | Local MAD p50/p90 mm | Frozen-normal MAD p50/p90 mm | Mean escaped fraction |
|---|---:|---:|---:|---:|
| 44_short_final / alpha1 | 136 | 18.05 / 48.34 | 17.89 / 55.57 | 0.000 |
| 44_short_final / alpha0.1 | 136 | 22.32 / 54.23 | 20.55 / 59.55 | 0.014 |
| livox1_final / alpha1 | 111 | 19.50 / 66.14 | 23.32 / 73.28 | 0.000 |
| livox1_final / alpha0.1 | 111 | 143.94 / 885.07 | 102.81 / 3346.28 | 0.636 |
| livox2_final / alpha1 | 114 | 25.63 / 87.55 | 20.55 / 88.51 | 0.000 |
| livox2_final / alpha0.3 | 114 | 22.15 / 94.99 | 24.15 / 97.67 | 0.253 |
| livox2_final / alpha0.1 | 114 | 22.50 / 90.73 | 26.36 / 99.18 | 0.269 |
| livox2_final / alpha0.03 | 114 | 41.79 / 291.62 | 39.98 / 456.79 | 0.303 |
| livox2_final / alpha0.01 | 114 | 107.39 / 477.60 | 200.38 / 2851.17 | 0.636 |

Coverage is a small selected fraction of the full cloud; reference selection may miss pre-existing ghosts. Local PCA can rotate or become ambiguous on distorted/isotropic patches. Frozen-normal spread mixes rotation and thickness. Read both with geometry, normal-angle/eigenvalue and visit diagnostics retained in measurements.json. No perturbation envelope was collected for these candidates; no resolved ranking or robustness claim.

## Visual evidence and reflection qualification

All five overlays reuse the control-only frozen rigid fit, anchored by the user-confirmed apartment56 Wohnen start. No candidate-specific refitting or scale change. Height slice +0.50 +/-0.05m can omit vertically displaced points; it is not the matched-return measurement. Alignment is preliminary, not surveyed accuracy.

The user reports alpha0.1 has no visible double walls and is a step forward. Blue-box returns above/below the apartment are user-identified window/mirror reflections. Mirror approximately x25.6,y14.9 on the plan near the upper Schlafzimmer entrance. Preserve annotations and separate physical-wall misalignment from reflective returns. No filtering or silent masking was performed; ray-level reflection attribution is NOT MEASURED. Alpha0.3 also received positive feedback with two local defects marked earlier.

## Validation and scope limits

Alpha1 Release full suite passes122 cases (3 skipped); Debug control passes. All four intervention Release frozen-equation and serial/parallel checks pass. Alpha0.1 Debug has one actual corridor accuracy failure:17.9039mm residual versus7.5mm limit; no threshold relaxed. Frozen NDT scaling and unchanged priors verified, including negative controls. Alpha1 regression matches44_short nonparameter artifacts and Livox1 trajectories.

Intentional weight changes are not neutral perturbations: do not compare their displacement to the0.10m perturbation gate. Lower chi2 alone is not accuracy: Livox1 alpha0.1 has severe deformation, approximately31.24m maximum final trajectory difference from control. This is difference, not surveyed error. Alpha0.3 is not cross-checked on Livox1. No inference of sensor damage.

## Delivery and next decision

All clouds, snapshots, logs, binaries, scripts and annotations are retained. Final delivery root after verified relocation: C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/. Historical JSON paths remain original provenance; relocation-manifest.json maps roots without rewriting recorded identities.

The bounded nine-run experiment is complete; no full-route or extra-weight runs are authorized by this report. Candidate acceptance requires robustness and physical-wall evidence beyond the present single runs. Independent review and the next bounded proposal are recorded in evidence.md.
