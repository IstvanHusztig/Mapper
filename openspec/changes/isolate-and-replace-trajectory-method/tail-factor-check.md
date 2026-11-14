# Captured-call tail-factor check

User authorized this existing-case check on 2026-11-14. Diagnostic source lives
in scratch tools/replay_solver_case.cpp, optional --check-tail. Recording replays
require separate approval. Source odometry behavior is unchanged.

The diagnostic replays a saved W301 solver call, evaluates its end-velocity IMU
factor, applies production repropagate_window_tail with recording settings
15 tail nodes and 10 velocity samples, and evaluates the factor again.
These are counterfactual handoffs after individual saved calls. The final W312
pre-handoff state was not captured.

Residual is (pQ-pP)/dt + (mean_acceleration-offset)*tau - v_end, with the captured
accelerometer scale, normalized node rotations, left-sample interval overlaps,
anchor selection and gap gates. Weight is
1/((noise_density^2/tau + floor_sigma^2)*tau^2).

For each stage, finite differences of the native full objective with respect to
each end-velocity coordinate must agree with the analytical factor gradient.
The check excludes resting cases, where a second end-velocity factor would be
active, and window1, where the captured bias flag differs from application setup.
It verifies that the native tail preserves carried velocity/covariance exactly.

Independent reviewer seed_damping_crosscheck verified the source formula and
handoff bias prior. Explicit finite-gradient checks were added after review.
Results and source identity will be recorded below after execution.

## Results

All four native checks exited0. Full-objective finite-difference gradient errors
were at most8.5816e-7. Original solver decision, cost and serialized output rows
remain exactly equal to the previously validated four calls.

| W301 iteration | Residual before [m/s] | Residual after [m/s] | Weighted cost before | Weighted cost after |
|---|---:|---:|---:|---:|
| 0 | 0.0000007373 | 0.02657792 | 0.000001499 | 1948.2754 |
| 5 | 0.0000001312 | 0.02597152 | 0.00000004744 | 1860.3864 |
| 10 | 0.00000009515 | 0.02612374 | 0.00000002497 | 1882.2576 |
| 51 | 0.0001563722 | 0.02576938 | 0.06744154 | 1831.5397 |

P173/Q179 are both in the overwritten tail, which starts after node164.
Maximum positional movement is0.5536–0.5759mm. Carried end velocity and its
covariance remain exactly fixed. Small positional changes over the short anchor
interval create the observed velocity-factor disagreement. i0/i5/i10 use the
ordinary solver path, so this effect exists before translation recovery.

Independent reviewer seed_damping_crosscheck verified case/log/executable hashes,
recomputed residuals and weighted costs, and reviewed the finite-gradient checks.
The native full-merit cross-check covers12 components before and12 after across
the four cases. This establishes that tail replacement can invalidate the fitted
end-velocity relationship. It does not quantify its share of the actual W313
jump or the displaced exterior geometry.

## Evidence and next decision

Frozen binary, four logs, stderr and hash manifest are under
`C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-translation-recovery-125s/velocity-investigation/tail-factor/`.

Next correction design should publish a coherent pose tail, terminal velocity
and covariance, preserving existing rotation/objective guards. A plain geometric
velocity substitution drops the inertial terminal correction. Compare retaining
the optimized tail with a consistent inertial handoff before selecting a change.
No algorithm correction, recording replay or claim of map improvement is part
of this check. Medium is sufficient for result bookkeeping; high is appropriate
when designing and evaluating that coupled-state correction.
Diagnostic source committed as9987917 in the experimental worktree. No push.

## Preparation for a retained-tail candidate

The application already supports retaining the optimized tail by setting
`handoff_tail_nodes = 0` in the experimental replay TOML. The window-steps
test checks that zero tail nodes leave the poses unchanged. This gives a
configuration-only comparison using the tested recovery binary and the
completed 125-second replay as control. Disable tail rewriting throughout
that experimental run. A production policy may need a narrower scope.

The comparison changes both the terminal pose and the geometric previous
velocity estimate consumed by the next window. Test these alongside the
carried optimized velocity and covariance. Check window 301 completion,
velocity changes around window 313, and the 31-second region. Compare
pre-loop-closure trajectory and final cloud separately. Retaining the tail
can remove the measured end-factor discontinuity while giving up the
existing IMU rotation reconstruction over those nodes.

Before choosing this as the production policy, review the effect on bias
correction in the tail and on the next window's seed. Keep pose, velocity and
covariance from the same accepted state if retaining the optimized tail.
For an inertial rewrite, require an explicit state and uncertainty model.
The captured-call result alone does not choose between those policies.

This is preparation only. The evaluation uses the existing verified prefix,
the completed control settings with the one edit above, Huber3, and a fresh
result directory. The user must approve any recording replay separately.

High-effort follow-up is complete: see `retained-tail-comparison-plan.md` for
the reviewed bias/velocity/covariance audit and frozen candidate. Bias estimation
remains active with zero tail nodes. Candidate prepared externally, not started;
no production policy adopted. Medium suffices for preparation and monitoring.
