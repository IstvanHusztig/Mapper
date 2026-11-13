# Frozen starting evidence (task 1.1)

2026-11-13. This inventory identifies controls; it does not adopt an experiment
as the shipped algorithm. SHA-256 values refer to files as checked today.

| Control | Identity and role |
|---|---|
| Shipped configuration | `settings/default_settings.toml`, SHA-256 `cd9420288ac0107c43eda9bdeac7f6a1e49f56100d00e5f67ecc6e7654b0594e`. This is the CLI built-in/default and GUI distributed-settings reference, not the Huber experiment. |
| Historical vertical configuration | `settings/l2-vertical.toml`, SHA-256 `bd6976c41de7048f51e7c534e370dbb3c6aa43ed373ee0a3c28d04d062f1b907`; used for 44 regression, not Livox sensor calibration. |
| Golden references | `tools/baseline/44_short.manifest.json` SHA-256 `2bc2d9b92b5f495063c9794c0bd1e265d7c21dece8fbf1752786c281d9ad757d`; full `44.manifest.json` SHA-256 `a86f344fc0943b0c844b1ff439ab2f1e820d44e173dd61f8980bce95731858c5`. |
| Experimental Huber3 | Frozen source `cbca36d2747cfcd1fc3ceac3d906bd221080d8f8` and `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/full-livox2-huber3/bundle.json`, SHA-256 `df8abfa7691b26506056c34f1e2d390d7bddc32e4766dbeb1dee1776016faa`; settings SHA-256 `2a69a32187a507f659ccfc091d800347dc763d6d685957a441005b1f4ba00a55`. Huber3 activates through an environment variable, not the shipped default. |
| Scratch pre-floor | `out/phase0b-worktree` at `e79125c`, tracked tree restored exactly to `c52b25f`; contains inactive experiments and is not a shipped control. See `../open-closed-space-robustness/rejection.md`. |
| Livox4 Huber3 | External `livox4-indoor-outdoor/huber3-baseline/` preflight SHA-256 `657a9c22b2bdd633e9202c01913fae24b06442dc95137ee2913f7c8e1d7bbf6a`, completed run SHA-256 `47a0de19b1da6b729aa3818c39f9844e5669cb31f9296e75492a27659213bb69`; linked `../ndt-information-probe/livox4-huber3.md`. |
| Stronger gyro sensitivity | External `livox4-indoor-outdoor/huber3-gyro4x/` preflight SHA-256 `ccbd7a957817c5a53a532fe829b0feb78ed2dcac3b4595f2632c256f856ccf37`, completed run SHA-256 `8829877d33942bdc745fa9d26623da2850f96f9e8ce80f1f0e798a22537690b9`; user judged the cloud a mess. See `../ndt-information-probe/livox4-gyro4x.md`. |

Existing-data calculations: `out/livox4-prediction-vs-matching.json`
SHA-256 `6d5bcc7d2be4c52a93acd8d5337915ec95c32acb76b695fff187fc5e6a6f161a6`;
`out/livox4-prediction-decomposition.json`
SHA-256 `d1ff918d965ad181e4351cfde687d96b36aa611907ae02c2609023e8545f2f7e91`.
Reviewed interpretation: `../ndt-information-probe/livox4-code-audit.md`.

Prior tests include the synthetic room, corridor, open-plane, wrong-cell,
contaminated-prior and state-carry checks under
`../open-closed-space-robustness/implementation-checks.md` and its task 2.3.
They exercise local math/state propagation, not the real Livox4 initiating cause.
Rejected candidates remain evidence: the 6 mm residual floor was explicitly
removed (`../open-closed-space-robustness/rejection.md`); full-route Cauchy and
bias-off results were worse (`../ndt-information-probe/sol61-decision-code-audit.md`).
No new synthetic or recording run is included in this inventory.
