# Regions: who owns what, and how a problem is worked

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [User guide](../index.html)

dMapper's code is divided into eight regions. Each has an owner agent that starts cold and knows its region from one
file in this folder: its contract, its files, its known flaws, what was tried and what was rejected. This page is what
every region agent reads besides its own file: the regions, the rules, the workflow from a question to a merged change.
Stage 7 of the [code-split plan](../code-split-and-region-agents-plan.md) built it.

## The regions

| Region | Its contract, in one sentence | Knowledge file |
|---|---|---|
| front-end | From a recording folder to windows: points with their times, IMU samples, an orientation per sample, windows with their first poses, every point attached to its node. | [front-end.md](front-end.md) |
| map | The NDT map: the cell statistics of placed points, and what enters the map and when (the seed, the update after each window, the sliding rebuild). | [map.md](map.md) |
| lidar-matching | A point against its cell: residual, Jacobian, the per-point rules applied, the degeneracy projection, the window's translation information. | [lidar-matching.md](lidar-matching.md) |
| inertial | A window's guess and its inertial terms: motion edges, velocity continuity, accelerometer terms, the gyro-bias and velocity carried from window to window, the hand-off. | [inertial.md](inertial.md) |
| solver | From the terms' equations to the window's result: the normal equations, the Levenberg-Marquardt iterations and why they stop, the step, the window verdict. | [solver.md](solver.md) |
| back-end | The corrections after the odometry: loop closure and the consistency pass; the odometry snapshot. | [back-end.md](back-end.md) |
| noise-budget | Every weight of a window and its effect: the noise model, the per-term table, the weak-axis and heading diagnostics. It owns the weights' formulas, not the terms. | [noise-budget.md](noise-budget.md) |
| verification | Whether a result is what it claims: the verification kit, the verify skills, the baselines, the analysis tools. It never edits `src/`. | [verification.md](verification.md) |

The **orchestrator** is the main session. It owns what the regions share (the shared types and settings, the window
loop's skeleton, capture and replay, the pipeline and the ports, the presentation layer, CMake, `settings/`, the docs),
talks to the user, starts the agents, and decides what is merged.

## Who owns what

[`owners.toml`](owners.toml) is the list: files per region, single functions in shared files (`file::function`, for
example the inertial region's `prepare_acc_term` in the orchestrator's `run_odometry.cpp`), and three shared test files
whose cases any region may add to for its own code. **Whatever the list does not name is the orchestrator's.**

```
python tools/region_owners.py                                      # is the list consistent?
python tools/region_owners.py --owner src/application/run_odometry.cpp
python tools/region_owners.py --diff <base>..<branch> --region <name>   # what does a branch touch outside its region?
```

Two boundaries worth knowing:

- **Weights and terms.** Every weight of a window is computed in `window_noise_model` (noise-budget). A term's region
  owns its residual and its Jacobian. A change of a weight's formula is the noise-budget's, with the term's region asked
  for its view.
- **Findings that cross.** `decimate` is the map's but `compute_step_1` (front-end) calls it; the hand-off runs through
  the inertial steps and the map update. Such a job names both regions, and the orchestrator edits what lies between.

## Rules for a region agent

1. **Read first:** `CLAUDE.md`, your knowledge file, this page.
2. **Say what you will do before you do it**: the first lines of your report are your plan.
3. **Edit only what you own.** What you need in a shared file or in another region you report; you don't edit it.
4. **Work only in the worktree you were given, on its branch.** Commit your change there. Never touch the main tree,
   never merge, never push.
5. **Work by replay**, started through `python tools/run_replay.py` (never `dMapper --replay` directly). You never start
   a full run, a `--from-snapshot` run or a perturbation sweep, and never change `settings/`: a settings idea is tried
   on a copy in your experiment folder, under a file name of your own. Where a replay does not reach your code (loading,
   the AHRS and the window cut of the front-end; loop closure and consistency of the back-end), ask for the runs you
   need in your hand-back ("runs wanted": which recording, which settings, built from which branch); the orchestrator
   runs them, one at a time, and gives you the result folders in a follow-up task.
6. **Numbers, not verdicts.** No statement about the map, no "better". What a reading means for the map is the user's
   to judge, by eye.
7. **No dead code**: a rejected idea is removed, not switched off. **Precision before speed**: when the two trade, take
   the precise one and report the cost.
8. **Hand back in the fixed form** (below), with your knowledge entry. You can't start other agents; the orchestrator
   calls verification and the reviewer.

## Worktrees and the gate

Every task names the worktree, the commit it was reset to (the base for `region_owners.py --diff <base>..HEAD`) and,
where needed, the capture. The shell does not keep a working directory between commands: give every command the
worktree's absolute path (`git -C <worktree> ...`, `python <worktree>/tools/run_replay.py ...`). The captures of the
step checks are listed in `tools/local_paths.toml`: `<guards>/44short-20-40/window_capture_20.0-40.0s.bin` and
`<guards>/livox3-100-130/window_capture_100.0-130.0s.bin` with their `settings.toml`, `<guards>` being
`C:/Users/User/Downloads/dLidar/experiments/guards`. An agent creates its own experiment folder
(`C:/Users/User/Downloads/dLidar/experiments/regions/<name>/<date>-<question>/`). A quick build without the checks:
`python <worktree>/tools/step_checks.py --build-only`.

- **The step checks count for a commit only when run on it.** Commit first, then run `step_checks.py` (its first line
  prints the commit and whether the tree has uncommitted source changes). A run before the commit is a rehearsal.
- **Long commands**: the shell does not allow waiting with `sleep`. Start `step_checks.py` (about 7 min) in the
  background and wait for its completion notice.
- **The references** the step checks compare with are shared by all worktrees (`<guards>/refs`); the line `references:
  commit ...` names the commit they were recorded from, which is normally older than yours. That is right: they are
  the bits the code must keep. They are recorded again only by the orchestrator, after a change that is meant to move
  them.
- `tools/region_owners.py --owner` and `--diff` take paths relative to the repository (`src/domain/ndt_map.cpp`).
- **A build needs the worktree's executable free.** While a run or replay started from a worktree is going, that
  worktree cannot be built (the linker cannot replace `dMapper.exe`); `step_checks.py` then says so and builds nothing.
  Wait for your replays before you build or run the step checks.

Each editing agent gets a git worktree and a branch of its own (`region/<name>/<date>-<question>`). There are two
standing worktrees, `C:/Repo/dMapper-worktrees/region-a` and `region-b`, reset to the main tree's commit before each job;
each has its own build (`out/build/x64-Release`, the same preset as the main tree). At most two editing agents work at
once. The main tree stays free for full runs and for the orchestrator.

Replays cost memory (about three times the capture file) and most cores, so every replay goes through the gate:
`tools/run_replay.py` waits until memory and a slot are free (two replays at once, none beside a full run), runs the
replay, and prints its cost and how many windows stopped at the time limit. If that count is above 0 the result depended
on the machine's load: run it again. Full runs are the orchestrator's alone, one at a time, from the main tree.

## From a question to a merged change

1. **Question and capture.** The orchestrator states the question and captures the stretch (a full run with the capture
   settings).
2. **Triage.** It reads the stretch's verdict lines, per-term table and noise lines and names the region the rows point
   to, with the rows. If two regions are possible, both are asked for their view, read-only.
3. **Region job.** The region's agent gets the question, the capture, the settings, what is known and what is ruled out.
   It works in its worktree and hands back in the fixed form.
4. **Verification.** The orchestrator checks the branch against the ownership list (`region_owners.py --diff`); the
   verification agent runs `tools/step_checks.py` in the agent's worktree. All identical: the change changes no number.
   Otherwise: the replays before and after, the run check on their logs, and the spread of small perturbations when one
   variant is claimed to beat another.
5. **Review.** The conclusion reviewer checks the finding against its evidence.
6. **Merge or decision.**
   - A branch that changes no number: the orchestrator decides, merges and reports.
   - A branch that changes a number: the user decides on the numbers, shown without a verdict. Reject (it goes to the
     knowledge file's "Ideas rejected"), or full runs with the run and map checks and the before and after clouds for the
     user's look, or accept. An accepted change is an OpenSpec change, baselines in their own commit. Nothing that changes
     the map is pushed before the user has looked at the clouds.
7. **After each merge** the orchestrator runs `step_checks.py` on the main tree: two branches merged one after the other
   were never tested together.

## The hand-back form

1. **The question**, as you understood it.
2. **What you ran**: the replays (capture, settings file), the tests, in order.
3. **The finding**, with its numbers.
4. **The evidence**: the folder (`experiments/regions/<name>/<date>-<question>/`), the files in it.
5. **The proposed change**: the branch and its commits, the files and functions it touches, whose contract it touches.
6. **What you could not measure.**
7. **Your knowledge entry**: the lines for your file's "Experiments", and an open question if one is left.

## The knowledge files

One form for all eight:

1. **Contract**: what the region takes, what it gives, what others rely on.
2. **Owns**: its entries of `owners.toml`, in words.
3. **How to check your work**: its tests, the replays, the readings that are its own (which rows of the per-term table,
   which parts of the noise line and of the verdict), the tools and the verify skill that apply.
4. **Known flaws**: the findings of the [code-review tracker](../code-review-tracker.md) that are not fixed, in the
   region that owns the code today.
5. **To split in the first job**: its functions above 80 lines (`python tools/function_sizes.py`).
6. **Experiments**: a dated log, newest first: the question, what was run, the numbers, the evidence folder.
7. **Ideas rejected**, with the evidence.
8. **Open questions.**

An agent does not edit its knowledge file. It hands in its entry; the orchestrator adds it after the review, and changes
"Known flaws", "Ideas rejected" and the contract only after the user's decision. A file stays under about 300 lines: an
old experiment shrinks to one line and its evidence folder.

## The checks, in one place

| Command | What it says | Time |
|---|---|---|
| `python tools/run_replay.py <capture> <settings>` | one replay through the gate, with its cost | 0.5 to 1.5 min |
| `python tools/step_checks.py` | does this worktree's code give the same bits (build, tests, both guards, both replays, the per-term table, 44_short)? | about 7 min |
| `python tools/end_runs.py [--debug]` | the full 44, Livox 3 and 44_short against their manifests (orchestrator only) | about 50 min |
| `python tools/region_owners.py --diff ... --region ...` | does a branch stay in its region? | seconds |

Details: [tools/README.md](../../tools/README.md).

## See also

- [The code-split and region-agents plan](../code-split-and-region-agents-plan.md) (stage 7: the design and its decisions)
- [Architecture](../ARCHITECTURE.md)
- [Code-review tracker](../code-review-tracker.md)
- [User guide: Internals](../index.html#internals)
- [All documents](../README.md)
