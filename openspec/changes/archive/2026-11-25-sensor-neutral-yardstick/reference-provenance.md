# Historical family reuse audit

2026-11-12. Independent Codex research review, not Sonnet. Read-only inventory;
no dMapper jobs or candidate rankings. Root below is
C:/Users/User/Downloads/dLidar/raw data/livox/.

| Livox1 family under runs_500hz/sweep | Differences from current shipped shared numeric keys |
|---|---|
| A_k7p08982_base + six B_k7p08982_* | None; diagnostics enabled |
| A_k1_base + six B_k1_* | inflation 7.0898154 to 1 AND rotation floor 0.0035449077 to 0.0005 |
| C_own_base + six C_own_* | inflation 1, all gyro ARWs 0.024, rotation floor 0.0002105263158 |

All use acceleration spans 0.0095/0.0105 s, translation sigmas
0.00009/0.00011 m and decimation 0.029/0.031 m as their six perturbations.
Historical always-on mode is enabled. Ten obsolete parameter keys remain in
their TOMLs; do not infer numerical equivalence from abbreviated family names.
For the three bases, input versus saved TOMLs differ only in preview path.
Shipped Livox retains L2 uncertainty values; range, lever arm and sample counts
are the sensor-specific differences, not independently measured new noise values.

All 21 logs reach Processing time after all five pipeline stages. Historical
results.json/results_own.json record exit0; merged clouds/session poses exist.
All contain 46980 final trajectory rows, identical sorted timestamp-token hash
prefix 4116e5b471f4fe25. Shipped-family exact numeric timestamps range from
374971770 to 94333086290 ns, with zero duplicates. Count/time correspondence is
supported; that alone does not establish build or input provenance.

Missing evidence:

- No executable hash or source commit in the historical scripts/results.
  Version 0.1.0 and build_date 'Mar 28 2026' cannot identify the executable.
  Scripts hardcode today's executable path; paths are not identities.
- Logged input paths/counts but no contemporaneous input hashes.
- No odometry snapshots among these 21; saved trajectories are after loop
  closure and consistency. Final-stage stability cannot certify odometry.
- holdout_block_s=0. Existing wall reports use automatic extraction at height
  0.5 m, without evidence of fixed physical regions and preserved coverage.

runs_2/base, bestk, own have the corresponding configuration differences,
complete logs and odometry snapshots. They are full-recording single runs, not
a family on the new 0–60 s cut. Processing times 2715.93/2958.35/3030.30 s.
Snapshot presence does not establish a validated comparison reader.

## Disposition under the agreed contract

Historical final-stage readings remain useful diagnostic evidence. They cannot
pass current-build acceptance or establish pre-loop perturbation stability.
Task 4.1 reconciliation is complete because the limitations are explicit; it
does not mark missing provenance established or complete task 4.2.

Before a new batch, seek contemporaneous build/run identity records and establish
a validated way to compare odometry and final stages. A fresh identified Livox1
base is the smallest bridge check (historical estimate about six minutes).
Matching it would establish reproduction for that run, not fingerprint all21.
A seven-member Livox1 family costs roughly42 minutes at that historical rate;
measure a fresh base first. Do not rerun the ~45–50 minute full Livox2 singles
merely to compensate for missing short-input or stage evidence.

No new processing batch started. Task 4.3 stays open until the measurement method
and exact minimal run list, output ownership and cost are established.
