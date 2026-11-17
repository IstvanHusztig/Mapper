# Design

## Context

The full design is the section "Stage 4: solver and window verdict" of `docs/code-split-and-region-agents-plan.md`
(2026-11-17): where the loop is today, the solver's interface, the loop guard, the four steps of the window loop, the
verdict's flags and readings, the tests and the commit order. A Sonnet reviewer checked it against the code at
`b234b6a`; its ten findings are worked in. The verdict follows the agreed evaluation contract
(`openspec/changes/sensor-neutral-yardstick/criteria-proposal.md`). This file keeps the decisions and the risks.

The user took all seven decisions of the plan as recommended (2026-11-17).

## Goals / Non-Goals

**Goals:**
- A solver that can be tested without a map and owns the iteration rules.
- A window loop whose body reads as four steps.
- Every run says how its windows behaved, with the contract's triggers, and records why each window's iterations
  stopped.
- Results bit-identical.

**Non-Goals:**
- No change of when the iterations end, or of the damping schedule.
- No acting on the verdict (a window kept out of the map, or solved again).
- No limits for the new readings; no change to `tools/run_health.py`.
- No work on Livox 4's faults (its run here is a reading).

## Decisions

1. **The solver behind a small interface** (linearize, covariance of the undamped unknowns, apply a step, save and
   restore), with `WindowProblem` as its one implementation. Why: the loop today can't be tested without a map, and its
   rules are spread over two layers. Alternative: move `run_lm_iterations` into the domain as it is. Rejected: it stays
   tied to the window, and the solver region of stage 7 would still own no unit.
2. **`lm_iteration` keeps today's single call, in both modes** (with a step; without one it ends after the covariance).
   `optimize_lidar_odometry` stays as a thin call of it, so the characterization guard of stages 2 to 5 keeps checking
   the same thing.
3. **`lm_solve` is the loop statement for statement.** Kept on purpose: a rejected step skips the convergence test, the
   scheduled factor update and the time check; a forced last iteration switches off only the time check. Each is a
   candidate for a measured change later, not for this refactor.
4. **The state is saved between "covariance" and "apply", and only with the step check.** It equals today's "save
   before, then copy the covariances in" (the step writes no covariance). The Cholesky analyses and the event counters
   are not state: a rejected iteration's events still count.
5. **The clock and the per-iteration record are callbacks**, so the domain reads no clock and a replay simply passes no
   time limit.
6. **A second guard for the loop.** The replays and baselines never run the step check, and the characterization guard
   runs single iterations. The loop guard must show rejected steps and early ends in its recorded file, or its fixture
   is changed until it does.
7. **The verdict is a pure function plus a summary**, in the domain. Per window: readings and flags. Per run: the
   contract's triggers. Why split: only "any fast window" style triggers are per window; "more than 1 % bent" is a
   property of the run.
8. **Agreed limits only.** The three flags use the numbers of the contract and of `run_health.py`. The new readings
   (stop cause, chi² of every window, tilt bend, per-term costs, remaining step) are recorded without a limit; a
   readings table of 44, Livox 3 and Livox 4 goes to the user, who decides. Alternative: propose limits now. Rejected:
   the contract says limits are not chosen by looking at candidates, and there is no distribution to choose from yet.
9. **Missing is `NOT MEASURED`.** A window without the accelerometer term has no speed and no chi², although today's
   log prints zeros for it and `run_health.py` reads them as measured. The contract's rule decides; the cross-check
   lists these windows.
10. **The summary in every run, the lines behind a setting.** The summary costs one small sum per matched point (the
    translation information, collected in every run from now on). Alternative: everything behind the setting.
    Rejected: a run that says nothing about its windows is how bad stretches went unnoticed.
11. **The summary's state is not captured.** It lives in `WindowLoopCarry` but is not written to a capture file, like
    the timing and convergence fields, so the capture format and the existing captures stay valid.

## Risks / Trade-offs

- [The loop's step-check logic changes a bit unnoticed] → the loop guard, recorded before the refactor, with the step
  check on and windows that end early.
- [The time-limit path can't be compared between runs] → it moves as one `if` evaluated at the same point; reviewed by
  reading, and the full 44 and Livox 3 runs at the stage's end would show a window cut differently.
- [Collecting the translation information in every run costs time] → measured on 44_short and reported; it enters no
  matrix, so no result can change.
- [The summary and `run_health.py` disagree] → the cross-check on 44_short, with the differences that are meant
  (printed precision, windows without a reading) listed one by one.
- [The verdict is read as a statement about the map] → the log, the docs and the spec call it an investigation trigger
  and show the coverage; `NOT MEASURED` never counts as clear.
- [Flags flood the log on a bad recording] → the per-window lines are behind `log_window_verdict`; the summary is a
  few lines whatever the run.
