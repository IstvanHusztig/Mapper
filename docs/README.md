# dMapper documentation

Start with the dMapper guide, [index.html](index.html) (open it in a browser). This file is the map of every page in `docs/`, by what it is for; the same map is the guide's section [All documents](index.html#documents). The Hungarian pages are under `hu/`. Also outside `docs/`: [CLAUDE.md](../CLAUDE.md) (how the code is built, tested and laid out), [tools/README.md](../tools/README.md) (the analysis tools) and `openspec/specs/` (the requirements).

## Using dMapper

Start here: what a run does and what every setting means.

- [User guide](index.html): The basic terms ([Fundamentals](index.html#fundamentals): pose, node, window, term, bend …, defined once and linked from every page), the pipeline step by step, every parameter, best practices, the tuning cheat sheet, known limitations.
- [How a run works: windows](PIPELINE.md): How a run cuts the recording into windows and processes one window (short, technical). *From pipeline step 3.*

## How the odometry works

Explanations with measurements: one mechanism per page.

- [Poses, windows and window chaining](window-chaining.html): What a pose is, how a window's trajectory and its points' positions are computed, and the window-chaining fix. *From pipeline steps 3, 5 and 7.*
- [Degeneracy information](degeneracy-information.html): When the LiDAR can't see where the sensor moved, how dMapper measures it, and how to read its log line. *From pipeline step 6.*
- [The per-term table](window-terms.html): What the per-term table is (which term of the optimization holds a window and where it pulls it), how to get it and read it, with an example reading on a real stretch. *From pipeline step 6.*
- [Performance](performance.html): How the full recording got 2.2 times faster: the overview. *From the parameters ([performance]).*
- [Performance fixes](PERFORMANCE.md): Every performance fix with its method and numbers. *From the parameters ([performance]).*

## Code and architecture

Where the code is going, the current plan, and the review findings.

- [Architecture](ARCHITECTURE.md): The target architecture (layers, ports, SOLID) and the migration roadmap (steps 0-11, done). *In the menu under Reference.*
- [Code split and region agents](code-split-and-region-agents-plan.md): The current plan, stages 0-8, with its progress. *In the menu under Reference.*
- [Code-review tracker](code-review-tracker.md): The findings of the code review of 2026-10-13 and their status. *In the menu under Reference.*

## Plans and research

Drafts, research notes and task texts; each says how current it is.

- [Sensor-neutral retune (draft)](sensor-neutral-retune-plan.md): Draft of 2026-11-10: findings and direction for settings grounded in the sensor; its current status is in agents/roadmap.md.
- [Loop closure research](loop-closure-research.html): Loop closure in dMapper: diagnosis, and how other systems do it. *From the parameters ([loop_closure]).*
- [Machine-learning learning guide](ml-research-roadmap.html): Modules M1-M9 to work through before any machine-learning work on dMapper.
- [dLidar time alignment (task text)](dlidar-timestamp-fix-prompt.md): Historical task text for the recorder (dLidar): the LiDAR-IMU time alignment of L2 recordings. The recorder is outside dMapper's scope.

## Region knowledge files

One knowledge file per code region: what an agent that starts cold reads to own its part of the code (stage 7 of the code-split plan). Kept by the orchestrator from the agents' entries; English only.

- [Regions: who owns what](regions/README.md): The eight regions, who owns which file and function (`owners.toml`), the rules for a region agent, worktrees and the replay gate, the protocol from a question to a merged change, the hand-back form.
- [Region: front-end](regions/front-end.md): Loading, the AHRS orientation, cutting windows, attaching points to nodes.
- [Region: map](regions/map.md): The NDT map: cell statistics, the seed, the update after each window, the sliding rebuild.
- [Region: lidar-matching](regions/lidar-matching.md): A point against its cell: residual, Jacobian, the per-point rules, the degeneracy projection.
- [Region: inertial](regions/inertial.md): A window's guess, the motion edges, the accelerometer terms, the states carried from window to window, the hand-off.
- [Region: solver](regions/solver.md): The normal equations, the Levenberg-Marquardt iterations and why they stop, the step, the window verdict.
- [Region: back-end](regions/back-end.md): Loop closure, the consistency pass, the odometry snapshot.
- [Region: noise-budget](regions/noise-budget.md): Every weight of a window (the noise model), the per-term table, the weak-axis and heading diagnostics.
- [Region: verification](regions/verification.md): The verification kit and its gate, the verify skills, the baselines, the analysis tools.

## Agent working notes

Working notes of the Codex agent setup (read through AGENTS.md); kept by the agents, not edited as documentation.

- [Roadmap](agents/roadmap.md): Current status of the retune and Livox work as the agents keep it.
- [Handoff](agents/handoff.md): The latest save point of an agent session.
- [Shared preferences](agents/preferences.md): Working preferences shared by the agents.
- [Environment](agents/environment.md): Local environment and workflow for builds and runs.
- [Removal validation](agents/removal-validation.md): Validation record of the accelerometer-mode removal.
- [Turn calibration measurements](agents/turn-calibration-measurements.md): Measurements of the turn recording, 2026-11-12.

A new page goes into this map and into the guide's [All documents](index.html#documents), and gets the top line and the "See also" block the other pages have.
