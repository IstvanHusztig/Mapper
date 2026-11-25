# Design

## Context

See proposal.md for the motivation. The loop closure (`close_loops` in `src/application/loop_closure.cpp`) builds a pose graph
over the segment poses: one odometry step between consecutive segments with fixed sigmas
(`loop_closure_odometry_sigma_m` 0.1 m, `_deg` 0.5), one edge per kept registration with its NDT information scaled to
`loop_closure_edge_sigma_m`, and a pruning loop that drops the edge whose residual after the solve is beyond
`loop_closure_max_residual_m` (the candidate whose removal lowers the cost most). The back-end sees only poses and
timestamps of the odometry; the information ratio and the weak axis of a window exist only in the log and in
`window_features.csv`.

Both rules were prototyped on the corrected-arm Livox 4 loop capture with `tools/loop_graph.py`, which solves this graph to
0.005 mm of dMapper, and frozen in writing before their results were read:
`experiments/consistency/livox4_lever/odo_cov/RULE.md` (blind-stretch rule, code `scripts/odo_step_cov.py`) and
`.../livox4_lever/match_check/RULE.md` (match check, code `scripts/match_check.py`). Both were designed on one recording.

## Goals / Non-Goals

**Goals:**
- dMapper itself reproduces the prototype's result on Livox 4 (deadline 2026-11-26): the rules built as frozen, no retuning.
- Off by default; with both switches off every result bit-identical.
- A replay of a loop capture can run both rules (the capture carries the readings).

**Non-Goals:**
- The stall in the odometry itself (changes F2a/F2b), a physical verification of registrations (free space, reverse
  overlap), reduced edges, a calibrated loop edge covariance, the shared or hybrid form of a run's error (Codex's design;
  second arm on the next recording, Python only).
- Turning the switches on in `settings/` (after the user's decision on the maps), new baselines for recording 44.

## Decisions

**D1. The readings travel on the window.** `Window::readings` (optional): information ratio and weak axis of the window's
LiDAR translation information at its last linearization, the rest test, the placed move and the duration as solved (before
the tail re-propagation, as `window_features.csv`). Set once per window in `report_window`, with or without
`log_window_terms`. Written to the odometry snapshot and the loop capture (format 2; format 1 loads with the readings
absent). Alternative rejected: reading `window_features.csv` in the loop closure (a log file as an input, absent in a replay).

**D2. Blind-stretch rule, per-step form.** A pure function from the readings and the segments to one 6x6 information per
odometry step, used by the solve, the cost and the pruning alike. Weak window: ratio below
`loop_closure_odometry_weak_ratio` (0.05), not at rest. Run: consecutive weak windows with axes within
`_axis_tolerance_deg` (30), one other window bridged. sigma_run = hypot(S1, S2), S1 = 1/2 x `_accelerometer_error_mps2`
(0.05) x T^2, S2 = sum of |placed speed along the axis - reference pace| x duration. sigma_run^2 is shared among the steps
by the run's time in each and added along the axis (in the step's start frame) to the step's translation covariance.
The shared form (one error per run) was tried and does worse here; Codex's hybrid is indistinguishable on the checks
(`odo_cov/shared_form_finding.md`). Per-step needs no extra unknowns. A run's time in the last segment has no step and is
not represented (logged).

**D3. Match check after the pruning, by held-out innovation.** For each edge e still in the graph:
T(e) = cost(graph without e and its family, plus e) - cost(graph without e and its family), each solved to its minimum.
Family: edges that reuse e's scans in either role (same source segment; targets within 2 segments; one's source within
1 segment of the other's target). While the largest T / limit is above 1 (limit: chi-square 99.9 % quantile for the rank of
the edge's information), drop that edge, run the pruning again, test again. The pruning itself stays as it is.
Alternatives: replacing the pruning by the check (on Livox 4 the same set except one marginal edge; a larger change to
shipped behaviour); the covariance form of T from the inverse normal matrix (one factorization per round instead of two
solves per edge; an optimization for later, the definition by solves is the reference).

**D4. The match check needs the blind-stretch rule.** On uniform steps it rates true corridor-mouth edges as false (the
prototype drops 12, among them the four that measure the stall). With `loop_closure_match_check` on and
`loop_closure_odometry_from_readings` off the loop closure refuses to run (an error, no fallback).

**D5. Settings** (`[loop_closure]`, outside the odometry settings hash): `loop_closure_odometry_from_readings` (false),
`loop_closure_odometry_weak_ratio` (0.05), `loop_closure_odometry_axis_tolerance_deg` (30),
`loop_closure_odometry_accelerometer_error_mps2` (0.05), `loop_closure_match_check` (false),
`loop_closure_match_check_quantile` (0.999). With the rule on and readings absent: an error, the loop closure does not run,
and the CLI exits 1.

**D6. Confirmation.** One full Livox 4 run from the branch build writes a format 2 capture; replays with the switches on
are compared with the prototype on that capture (`tools/loop_graph.py check`: kept edges, trajectory). The user judges the
map from dMapper beside the Step A map.

## Risks / Trade-offs

- One recording. Both rules were designed knowing where the stall and the false edge are. The next recording is the first
  test, with both unchanged.
- The check is conservative: with 0.1 m per step the model's sigmas are generous (median T 0.3 where the model expects
  5.3), so it finds a 2 m contradiction and would pass a 1 m one. An edge no independent loop reaches passes unconfirmed.
- Time: two graph solves per edge and round (Livox 4: about 80 edges, 2 rounds). Not measured on a long recording; if it
  matters, D3's covariance form.
- A new full run is not the old run: its odometry may differ where windows stop at the time limit, so the edges and the map
  can differ slightly from the prototype's capture. The comparison is made on the new capture.
- The snapshot and capture formats change; older files load, but cannot run the rules.
