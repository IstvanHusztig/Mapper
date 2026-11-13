# Charlie — roadmap checkpoint before a focused fix

Checkpoint name: **Charlie**. Saved 2026-11-13 at the user's request. The larger replacement roadmap is
parked for a possible focused-fix detour, not abandoned or complete.

## Progress and resume point

2/19 tasks complete: 1.1 evidence inventory, 1.2 independently reviewed
state-transition trace. Resume at 1.3: exhaust bounded existing-data attribution
of the W295–301 attitude/gravity change, then prepare a minimal capture only if
needed. Task 1.4 requires the reviewed isolation decision and concrete interface
contract before any extraction. No interface or replacement algorithm exists yet.

Remaining stages: isolate -> extract legacy with numerical equivalence and saved
method selection -> research/select one candidate -> implement opt-in -> validate
indoor/outdoor quality and perturbations -> user adoption/removal decision.

## Findings to carry forward

- Livox4's incoming prediction explains most displacement growth at W301–320,
  but the joint optimizer creates outgoing velocity increments subsequently
  carried into prediction. W320 net accepted end-velocity change is approximately
  [0.282,0.355,0.008] m/s. This is a feedback path, not proof of a defective update.
- Approximate reconstruction shows persistent horizontal attitude/gravity
  disagreement of about 1 m/s² during buildup. See attitude-gravity-lead.md for
  equations, scale provenance and uncertainty. AHRS is not ground truth and the
  gravity decomposition fits predicted displacement; initiating cause unresolved.
- W321 is an explicit velocity-prior reset; W397 is a separate large angular
  correction. The translation-only acceptance guard does not catch both forms.
- Snapshot v1 overwrites motion_model_poses; it is not an exact odometry restart.
  Prefer validated bounded checkpoints to repeated full routes, preserving
  incoming state and map. Raw cuts reinitialize the estimator.

## Controls, files and code

Root branch `remove-gated-accelerometer`; last analysis commit `b416fc6`.
Scratch `out/phase0b-worktree` at `e79125c` restores pre-floor tree `c52b25f`.
Frozen Huber3 reference remains `cbca36d`, alpha0.1/beta0.1. No new numerical
code or shipped defaults changed during this roadmap. Preserve unrelated dirty
files; consult evidence-inventory.md for source/configuration identities.

Livox4 gyro4x completed: final endpoint separation 44.394 m versus reference
36.717 m. User judged its cloud a mess; no adoption. Residual floor was explicitly
rejected and removed; failed Cauchy/bias-off evidence also remains relevant.

Generated analysis was copied with SHA-256 verification to:
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/trajectory-method-checkpoint-2026-11-13/`.
Its manifest covers the orientation probe source/result and both displacement
decomposition JSON files. Original complete run bundles remain in place.

## Detour boundaries

No particular quick fix has yet been selected or implemented. Evaluate a focused
candidate separately; do not silently mark larger roadmap tasks complete because
it looks better. New mapping/checkpoint runs still require explicit user approval.
Commits authorized, no push. Keep OpenSpec updated; use existing Python tools.
Latest user-confirmed model/effort: Astra high. Tell before needing a change;
medium is sufficient for routine execution once numerical analysis is settled.

After the detour, record its evidence here and resume the remaining tasks or
explicitly revise the roadmap with the user. Do not restart the prior research
or rerun rejected candidates as new ideas.
