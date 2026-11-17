# Historical evidence under agreed criteria

2026-11-12. Independent Codex research performed strict timestamp comparisons
and independently checked each worst displacement with NumPy. Root:
C:/Users/User/Downloads/dLidar/raw data/livox/runs_500hz/sweep/.
Bases A_k7p08982_base, A_k1_base, C_own_base; members B_k7p08982_*, B_k1_*,
C_own_* with suffixes below. These are historical final-stage results, not
current-build candidate acceptance or causal isolation.

| Member suffix | Shipped max / temporal p95 m | k1 max / temporal p95 m | Own-noise max / temporal p95 m |
|---|---|---|---|
| span95 | 0.718 / 0.560 | 1.310 / 1.192 | 2.898 / 2.580 |
| span105 | 17.436 / 17.355 | 0.706 / 0.656 | 1.275 / 1.105 |
| edge09 | 0.827 / 0.381 | 5.828 / 5.212 | 1.277 / 1.104 |
| edge11 | 0.597 / 0.404 | 1.179 / 1.078 | 0.893 / 0.512 |
| dec029 | 18.460 / 17.091 | 24.272 / 22.485 | 1.238 / 1.098 |
| dec031 | 0.592 / 0.474 | 1.345 / 0.963 | 4.404 / 4.216 |

All18 perturbations exceed the agreed0.10m short-input stability target. Every
pair has46980 exactly corresponding timestamps. All21 logs have261/261 heading,
speed and bias readings and expected total261. Conditional uncertainty missing
or rounded-eligibility ambiguous: shipped span95 window49, shipped edge11
window232, k1dec029 window223. Chi-square triggers occur in all21 runs.

Odometry-stage perturbation spread: NOT MEASURED (snapshots absent).
Historical build/input identity: NOT ESTABLISHED. Map fixed-region and held-out
coverage: NOT ESTABLISHED. These gaps cannot be converted into a PASS. The
three families change different combinations of inflation/floor/ARW; no ranking
or accepted correction follows from this table.

## Minimal new collection

First collect current shipped Livox1 base with unchanged numerical settings,
diagnostic logging and snapshot enabled. About6 minutes historically; actual
time to be recorded. Fresh directory out/phase0a-2026-11-12/livox1-current,
single process owns Livox1 input cache and the Release executable. Free disk
before start134.39GB. Executable SHA256:
033EA75ACD39C14752E734E0AC473252034F97B2E2BA2A57A018C8CDA09CBCB0.
The orchestrator records full input/settings/tool/output identity.

Compare this baseline to its historical counterpart only after diagnostics and
exact time correspondence. Decide missing collection from that evidence; do not
rerun all long experiments merely because historical fingerprints are absent.
No baseline result is yet recorded in this document; process started this turn.

Update: current base completed, mapper exit0,393.76s processing. Required
heading/speed/bias coverage261/261 each; conditional chi-square52 windows with
no missing required readings. Final trajectory exactly reproduces all46980
historical shipped-base positions (max and p95 difference0). This establishes
reproduction of this one baseline, not retroactive identity of all21 old runs.
Base readings: bends>3deg16,>10deg3,max18.027deg; peak speed1.1513m/s;
max residual bias[0.385,0.413,1.789]deg/s; chi-square median183.25,p959060.62;
median velocity jump0.0490566m/s, carried sigma0.0038m/s. No accuracy verdict.
Raw comparison in out/phase0a-2026-11-12/current-results.json.

dec_minus is now collecting with verified base reuse and identical input/build.
Measured base duration supports an additional6–7minute estimate. Aggregate
collector status remains INCOMPLETE because the full seven-member family was
not requested; this does not mean the completed base mapper process failed.

## Preselected follow-up and stopping rule

Independent review recommends base plus decimation0.029 (preselected from the
historical shipped worst case) as the minimum current-build counterexample test.
Collect both stages and retain original numerical settings otherwise. Estimate
about6 minutes per run until the fresh base supplies an actual duration. Run
sequentially in the same identified family and preserve both clouds.

If either stage exceeds0.10m, this establishes a lower bound on current
instability and disproves universal compliance; it is not a complete noise-floor
estimate, candidate ranking, or positive robustness verdict. The family stays
INCOMPLETE. That negative evidence can inform a separately scoped0b diagnostic
experiment without repeating7+7 runs merely to rediscover instability. If the
pair remains below0.10m, it proves no stability and further members are needed.
Full matched families remain required before accepting an improvement. This
distinction does not relax the agreed acceptance contract.
