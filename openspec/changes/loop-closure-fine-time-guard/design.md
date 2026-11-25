# Design

## Context

See proposal.md for the motivation. What shapes the design:

- **How the loop closure places a blind stretch today** (`close_loops`, `src/application/loop_closure.cpp`). The windows are
  cut into segments of 6 windows (Livox: 2.16 s). A loop edge is the rigid registration of a whole source segment onto an
  earlier one, and the pose graph reads it at the source segment's first pose. With `loop_closure_odometry_from_readings`
  the odometry steps of a weak run are loosened along the run's axis, the run's variance shared among the steps by time.
  After the solve the per-segment correction is interpolated by time between the segments' first poses onto every node.
- **What Codex measured** on the Livox 4 capture from before `imu_edges_follow_state`
  (`experiments/consistency/ghost_wall_review/codex/analysis-codex.md`, `stage_findings.md`; capture
  `livox4_lever/readings_run/run`, map `readings_run/full_both_fixes`; the stall is windows 482-506, 173.5-182.5 s, sigma
  5.04 m):
  - Times: Codex's device seconds are elapsed seconds + 0.354; below "device" or "elapsed" is said where it matters.
  - The scans of device 178.5-182.25 s lie 1.85 to 0.54 m too far along the passage, against three other visits of the same
    surfaces. His matching shifts along the passage only; across it the registrations ask for 2-6 cm, and no shift across
    was fitted.
  - The two mouth edges 71 -> 84 and 74 -> 84 are read at elapsed 181.44 s. Their information along the passage lies on
    average 1.25 and 1.34 s later; the last three of the six source windows carry 64 % and 74 % of it. Fitted alone, the
    six windows ask for about -1.0 (71 -> 84) / -1.1 (74 -> 84), -0.55, -0.15, 0, 0, 0 m beside the rigid answer.
  - The graph meets the two interior edges 71 -> 81 and 75 -> 82 at their first pose within 3 cm, and the nodes later in the
    same source segments lie 0.9-2.2 m from the rigid placement the registration describes. The corrections at the
    segment starts in the stall are +4.43, +2.27, +4.04, +5.80 m: not monotonic, with speed jumps of 2-3 m/s in 40 ms.
  - The frozen prototype (`frozen-prototype.md`, `prototype.py`): ghost line / control strip per height band from
    2,437 / 0, 7,644 / 44, 7,251 / 72, 2,149 / 0 to 0 / 0, 0 / 0, 13 / 10, 15 / 0; held-out scene misfit (RMS over seven
    windows, device 178-182 s) 0.435 -> 0.216 m, part of it because three windows start at the 0.5 m cap; the 54 cm double,
    the facade, Wohnen, the reference contacts and the far faces unchanged. It moves the first pose of segment 84 by 1.0 m,
    so the two mouth edges read there show 0.995 / 1.012 m (before 0.006 / 0.021 m).
  - 13 of the stall's windows (482-491 and 493-495) have no usable surface across the passage (0-2 patches), and two
    more have few (481: 40 training patches, 492: 12). The prototype moves the ten windows 482-491 by 0.5 to 0.1 m all
    the same: they follow window 481 (-0.5 m) along the coupled profile. Alone, the weak windows point anywhere (480:
    +5.8 m, 492: +5.6 m, the well-supported ones -1.7 m); only the coupling holds them.
- **An estimate of mine from Codex's per-window tables** (`reviewer-temporal-windows.csv`, `stage_rigid_source.csv`; rounded
  values at the windows' middles, not a measurement; task 1.4 measures it): read at the times of their own information,
  the two mouth edges are off by about 0.10-0.13 m after the prototype, not 1.0 m (the limit of D6 is about 0.21-0.22 m
  for them: they pass with half the limit to spare, not more). 75 -> 82 is off by about 1.3 m on master and close to 0
  after the prototype (64 % of its information comes from its last window, where its +2.27 m fits the scene). 71 -> 81 is
  off by about 0.9 m on master and about 1.2 m after the prototype: the prototype makes it worse by about 0.3 m, and none
  of its source windows has a surface across the passage.
- **Today's state:** with `imu_edges_follow_state` the Livox 4 run's largest segment correction is 1.08 m and its largest run
  sigma 1.46 m (windows 481-496, the only run above 1 m); the ghost line's excess is about 500 returns
  (`livox4_lever/layer1/`). No loop capture of that run exists.
- **Constraints.** `close_loops` reads no points: the run and the capture replay share it, and a replay must give the run's
  nodes bit for bit. The readings rule and the match check are frozen until the next recording. Every rule must port from
  the prototype to C++ one to one. The evidence is one stall on one recording, and the design was made knowing it.

## Goals / Non-Goals

**Goals:**
- A guard with bounded reach: it moves nodes only inside an eligible run's support and only along the run's axis, both
  ends stay where the pose graph put them, and a run without a single supported window is not moved. Inside a support a
  window without surfaces follows its supported neighbours (a continuous path allows nothing else), so the guard can move
  a blind window wrongly; how many such windows moved and how far is reported (D8).
- A written rule for a disagreement between a loop edge and the scans, decided before the numbers are read.
- The C++ gives the prototype's profile on the same capture; a replay reproduces the step bit for bit.
- A run the scans cannot time is said so, with its uncertainty.

**Non-Goals:**
- Removing the non-monotonic corrections where the scans are silent (they come from where the graph reads the edges; this
  change only measures that, see D6).
- Any change to the pose graph, the pruning, the match check, the readings rule or the odometry.
- Turning the guard on in `settings/`.

## Decisions

**D1. A last step of `close_loops`, one offset along the axis per window.** After the node correction (step 5), for each
eligible run the nodes in its support move by `d(t) * a`: `d` the fitted profile [m], `a` the run's axis turned by the
rotation of the graph's correction at the run's middle (one straight line per run). Orientations are untouched;
`motion_model_poses` follows the poses as today.
- *Rejected: one pose-graph node per window inside a run.* It solves edges and scans together, but it changes what the
  segment index means for the slots, the revisit search and the match check's family rule (all frozen), and a 6-DoF
  registration of a 0.36 s scan in a passage is the case most open to false matches. A 1-D search over a grid with coupled
  neighbours handles several minima; a local refinement does not. Possible later, if the guard proves itself.
- *Rejected here: reading every edge at its time support inside the pose graph.* It changes every graph result, and a
  piecewise-linear correction between 2.16 s anchors cannot follow the knee at 182.5 s inside segment 84. D6's table is the
  evidence for deciding it as a change of its own.

**D2. Eligible runs and their support.** A run of `loop_odometry_step_information` is eligible when its sigma is at least
`loop_closure_fine_time_min_sigma_m` (1.0). Support: the run's windows plus `loop_closure_segment_windows` windows before
and after (the flanks), cut at the recording's ends. When the supports of two eligible runs overlap, both runs are left as
they are and reported (D8); a profile across two axes is not built in this change.

**D3. Reference surfaces from other visits.** The segments' points the loop closure already holds (0.1 m voxels per
segment), each with its window and node, placed with the corrected poses. Used: points of windows at least 12 s before the
support's start or after its end, and of no eligible run's support. Reference patch: a point with a local plane (14
nearest neighbours; the frozen planarity limits) whose normal is within 60 degrees of the axis, i.e. a surface across the
passage.

**D4. Scene score per window** (Codex's frozen form). Source patches: the window's own points from the point store, with
the filters `segment_points` applies (held-out blocks, range, node index), 0.07 m voxels, the same plane and normal rule.
They are split by a spatial hash of their position before any offset (1 m cells, one in five held out), at most 256 of
each kind. For an offset `s` on the grid (0.10 m steps within `+-min(1.5 sigma, 8 m)`): each patch moved by `s * a`, its
distance to the nearest of 4 reference patches within 0.5 m whose normal agrees (|cos| >= 0.8), else 0.5 m; the score
`E_w(s)` is the mean squared distance over the training patches, and the held-out score the same over the held-out ones.
A window with fewer than 8 training patches is unsupported (`E_w = 0`); a held-out score needs 8 held-out patches.
Nearest neighbours are exact (a grid index in C++, checked against brute force), so the prototype and the C++ agree apart
from rounding at a limit. The limit of 8 is thin (Context: windows with 12 and 40 patches point metres away on their
own); it stays as frozen, and task 1.4 reports the profile with the limit at 32 as a reading.

**D5. Profile.** `d` minimises `sum_w E_w(d_w) + 0.10 sum_w (d_{w+1} - d_w)^2` over the grid, with `d = 0` at the
support's first and last moment, solved exactly by dynamic programming (ties: the smaller |d|, then the more negative).
`d` is interpolated linearly between the windows' mid times onto the nodes. The coupling is Codex's frozen constant.
- *Considered: the coupling from the readings rule's own variance* (about 1 m per window in the stall). It is the model's
  own number, but it lets a single window with a false minimum pull the profile; the frozen constant is of the size of the
  scores' contrast and keeps a lone window in line. The profile's change with the coupling at a third and at three times
  its value is reported (task 1.4), as a reading, not for tuning.
- `d` is relative to the pose graph's result, so the graph's placement is the starting point, and a stretch far from any
  supported window keeps it. Unsupported windows next to supported ones move with them (Context: ten windows, up to 0.5 m).
  The zigzag in the blind first half of the stall is therefore neither kept exactly nor repaired; it is out of scope.

**D6. The edge's time support, and the conflict rule.** `ndt_refine` adds each source point's contribution to the normal
matrix; the translation block is also summed per source window (`H_w`, 3 x 3; their sum is the translation block of the
try's `information`). For a run's axis `a`: `s_w = a^T H_w a / sum_w a^T H_w a`. The edge's **time-support residual**
along the axis:

    rho_e = sum_w s_w * a . (x_w - K_i C_e x0_w)

`x0_w`: the odometry's position of window w's middle node; `x_w`: its final position; `C_e`: the registration;
`K_i`: the final correction at the target segment's first pose. The residual the graph uses is the same expression at the
source segment's first node alone. Classes, per edge with a source window in a support:
- a source window is **blind** when it lies in the support and is unsupported (D4); the edge's informed share `q` is the
  sum of `s_w` over its other windows;
- `q < 0.5`: **unconfirmed** - its scans have no surface across the passage where the readings call the passage blind; it
  is reported and stops nothing;
- else **agrees** when `|rho_e|` after the profile is within the pruning's limit read along the axis
  (`|rho_e| / sigma_e,a * loop_closure_edge_sigma_m <= loop_closure_max_residual_m`, 3 sigma with the shipped values) or
  no larger than before the profile; otherwise **contradicted**.
- An edge whose target segment's first pose lies in a support is read by the same formula: its source windows keep their
  weights and do not move, `K_i` takes the profile's offset at that moment, so `rho` after is `rho` before minus that
  offset; it is judged by the same limit (`RULE.md`, section 7). The target side has no per-window information, so a later visit registered onto a moved part
  of the run counts as contradicted and stops the run: the safe side, at the price of a guard that then does nothing. The
  stall capture has no such edge.
- Known bias of `s_w`: `segment_points` keeps the first point in each 0.1 m voxel, so a surface seen by several windows of
  a segment is credited to the earliest. The true time support can lie later than `s_w` says (Codex's table has the same
  bias).
The table (first-pose residual, `rho` before and after, `q`, class) is logged for every edge touching a support, also for
a run the guard leaves alone. The replay's JSON also gives `rho` for every kept edge with a source window in any weak run
(eligible or not), along that run's axis, with the guard on or off.
- *Why scans decide the timing and edges are only compared:* along a run's weak axis a whole-segment registration uses the
  same scans as the scene scores, against one earlier visit instead of all, as one rigid body instead of per window, from
  a local refinement instead of a search. Where the per-window score finds nothing across the passage, the edge's
  along-axis value has nothing to rest on either.
- *Rejected: the edges as terms of the profile's fit.* A term over six windows breaks the dynamic programming, and it would
  let a false interior edge (71 -> 81) pull the profile where no scan can object.

**D7. When a run's profile is applied.** All of: at least one supported window in the run itself; the profile is not zero
everywhere; the held-out score, averaged over the windows that have one (D4), is lower after than before (no such window:
not applied); no edge is contradicted (D6). Otherwise the run stays exactly as the pose graph left it. The guard never
drops or reweights an edge. The held-out patches are other points of the same scans against the same reference: the
check catches a profile that fits its training patches only, not a wrong reference.

**D8. What is reported.** Per eligible run one log line: windows, time, axis, the largest angle between a window's weak
axis and the run's axis, sigma, supported windows of the run's windows, the longest unsupported stretch [s], largest
offset, the number of unsupported windows that moved and their largest offset, held-out before -> after, and `applied` or
`not timed` with the cause (no surface across the passage seen by another visit; held-out not better; edge i -> j contradicted; supports
overlap). `LoopClosureReport` carries the same, the per-window offsets and scores, and D6's table; the replay writes them
to its JSON and a per-window CSV (`loop_fine_time.csv`: window, time, offset, supported, patches, scores before / after),
which is also the learning-data record of this step.

**D9. Port, capture, replay.** `close_loops` asks a new port (next to `LoopClosureRegistrations`) for the score curves
of a support, given the corrected poses. The run's adapter computes them from the points (D3, D4); the replay's adapter
returns the captured curves when the replayed graph gave the capturing run's segment poses bit for bit and the eligible
runs are the same, and nothing otherwise. Capture format 3 adds `H_w` per try and, per eligible run, the training and
held-out curves with their patch counts, the segment poses and the set of eligible runs they belong to. Formats 1 and 2
load; they carry neither.
- With the guard on and the curves missing or belonging to another graph result or another set of eligible runs, the loop
  closure does not run, an error names the cause (the remedy: a `--from-snapshot` run), and a replay exits 1. No silent
  fall-back to the graph's result.
- With the capture's own settings a replay gives every node bit for bit, the guard's move included: that is what the
  replay is for here. Another `min_sigma` changes the eligible runs, and with them the reference of every run (D3), so it
  needs the points. Variants of the rule are screened with `tools/loop_fine_time.py`, which reads them.

**D10. Settings** (`[loop_closure]`, outside the odometry settings hash): `loop_closure_fine_time_placement` (false),
`loop_closure_fine_time_min_sigma_m` (1.0). The other numbers (12 s, 60 degrees, 0.07 / 0.10 m, 256, 8, 0.5 m, 0.8, the
coupling 0.10, 1.5 sigma and 8 m, the informed share 0.5) are named constants in one struct beside `LoopOdometryRule`,
with the values of `RULE.md`. The guard needs `loop_closure_odometry_from_readings`; without it the loop closure is
refused as for the match check (outcome `fine_time_without_readings`).

**D11. Prototype first, in operations that port.** `tools/loop_fine_time.py` (numpy / scipy like `loop_graph.py`; reads the
capture, the recording's `cache/` and a trajectory folder; writes a trajectory folder and the tables) implements D2-D7
with D3's point sets, which differ from Codex's (the merged cloud, every 16th return). Before anything is scored, the rule
is frozen in `experiments/consistency/livox4_lever/fine_time/RULE.md`, with what counts as "reproduces Codex's result" on
the stall capture: every supported window's offset within 0.2 m of his, the four cleared height bands at or below 50
returns on the line, the other checks of his table unchanged. If the rebuilt prototype misses that, the difference is
found and reported before any C++.

**D12. How it is judged** (the list of `ghost_wall/PLAN.md`, guard rail 3, beside the state without the guard):
1. *Stall capture* (`--from-snapshot` of `readings_run/run`, the capture's settings plus the guard): the ghost line per
   height band, the scene table by time, D6's table, the 54 cm double and the other doubles, facade, Wohnen, reference
   contacts, far faces; C++ against the prototype (same supported windows, same offset in every window).
2. *Today's Livox 4* (one full run with the shipped file and a capture, then replay): the same list; expected little to
   move; nothing on the list may get worse.
3. *Livox 3* (shipped file, a capture): eligible runs, largest move against its baseline, the user's look.
4. *Truth cases* on the synthetic route R2 (out and back through a corridor with a blind middle): an along-axis error that
   grows inside the blind stretch is recovered to a grid step where the far room's faces are seen; a corridor with nothing
   across it comes out `not timed` and unchanged; door frames at equal spacing must not move the profile by one spacing.
5. *Switch off:* every step check and recording 44 bit-identical.
6. The conclusion reviewer; the user judges the maps. The new recording (task 4.4 of
   `loop-closure-weak-runs-and-match-check`) is the first independent test, with the rule unchanged.

## Risks / Trade-offs

- [One stall, one recording, a rule designed knowing it, D6's limits chosen knowing my estimate] -> truth cases with other
  geometry, two more captures, off by default, the new recording before any claim beyond this one.
- [The reference shares the map's errors; held-out patches are other points of the same scans against the same reference,
  not independent] -> 12 s separation, eligible supports never a reference, ends pinned; the report says "agreement between
  visits", not accuracy.
- [A window with few patches can point metres away, and windows without any follow their neighbours] -> the coupling and
  the pinned ends; the count and size of such moves in every report; the profile with the support limit at 32 as a reading.
- [A run's axis is the mean of weak axes that may differ by up to 30 degrees; a move along it has a part in a direction
  one window's LiDAR did hold] -> the largest such angle is logged per run; no rule on it until a case shows the need.
- [The two mouth edges of the stall case pass D6 with about half the limit to spare, on an estimate] -> task 1.4 measures
  it before any C++; if they fail, the guard does nothing in the very case it was made for, and that goes to the user.
- [Repeating structure gives several equal offsets] -> the coupling and the pinned ends, the truth case with door frames;
  what remains is a wrong but smooth profile inside one support, visible in D8's CSV.
- [The first-pose residuals of the edges into a moved segment grow (about 1 m in the stall case) and stay in the graph's
  report] -> the log gives both readings side by side; the consistency pass, where it is on, runs afterwards on the moved
  poses (Livox: off).
- [Where no scan speaks, the graph's zigzag stays] -> out of scope; D6's table is written for the follow-up.
- [Two eligible runs close together are both left] -> reported; a bend in a blind cave passage is such a case and needs
  the two-axis profile later.
- [Time] -> reference from points already in memory, sources from a few dozen windows per run; measured in task 3.6.
- [Capture format 3] -> formats 1 and 2 load; `tools/loop_graph.py`'s reader follows; Codex's scripts read format 2 files,
  which stay where they are.

## Migration Plan

Off by default: nothing to migrate. The capture reader accepts formats 1-3. Rollback: the switch.

## Open Questions

- Whether a contradicted or unconfirmed edge should later be dropped by the pruning (after the new recording).
- Whether the guard's constants should become settings (after a second sensor shows a need).
- The noise floor of the guard under the odometry perturbation family (after the parked corrected-arm family has run).
