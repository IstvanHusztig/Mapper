# Design

## Context

See proposal.md for the motivation. The facts the design rests on, all from the saved stall
(`experiments/consistency/livox4_lever/fine_time/`, capture `stall_fmt3`, trace `rule/profile_trace.log`, results
`results.md`):

- **The stretch.** Weak run 482-506 (sigma 5.04 m), support 476-512, segments of 6 windows starting at 480, 486, 492,
  498, 504, 510. Windows with surfaces across the passage ("supported"): 476-481 and 496-512. Without: 482-495 (5 s).
- **The pose graph's correction along the axis** at the windows' middles: +0.05 m up to 479, +0.42 (480), +1.15 (481),
  then +0.73 m per window to **+4.25 (486)**, falling to **+2.41 (492)**, then rising to +5.80 (504) and flat. It
  changes slope only at segment first poses, because the node correction is interpolated between them.
- **Why it reverses.** Segment 81 (486-491) carries edge 71 -> 81, whose information along the axis lies entirely in
  windows without surfaces (informed share 0.00); it holds the segment's shift at +4.25 m. Segment 82 (492-497) carries
  edge 75 -> 82 (informed share 0.76, information in 496-497), which holds +2.41 m at the segment's first pose, 1.3 s
  before its information. The scans of 496 and 497 themselves put the correction at +1.80 and +2.20 m.
- **Today's rule** (`fine_time/RULE.md` section 6) fits offsets `d_w` with the cost
  `sum E_w(d_w) + 0.10 sum (d_{w+1} - d_w)^2`, zero at both pins, and moves each node by the offset interpolated between
  window middles. A window without surfaces has no `E` and follows its neighbours' offsets: it keeps the pose graph's
  shape. Result: offsets -1.0 m (481) to -1.8 m (496) across the gap, the sum falling from +3.25 (486) to +1.01 (492).
- **What is known and not known.** The scans say where 476-481 and 496-512 are. For 482-495 there is no evidence at
  all; Codex's independent prototype differs from ours there by up to 1.5 m, and window 481's own offset moves by 0.5 m
  with the point density (`results.md` section 6). The pose graph's placement in the gap rests on the two edges above.
- The fine-time step is on in `settings/livox-vertical.toml`; its capture format (4) stores each window's score curves,
  the pose graph's result and the eligible runs.

## Goals / Non-Goals

**Goals:**
- Inside an eligible run's stretch, along the axis, the correction does not reverse where no scan asks for it.
- The windows the scans place stay exactly where today's rule puts them.
- The rule is stated and frozen before its results are read; what it cannot know (the gap) is said with its size.
- A reading that shows the fault on any recording without points.

**Non-Goals:**
- Knowing where the windows of a gap really were. The line is a prior, not a measurement.
- Changing the pose graph, the registrations' information, or which edges are kept.
- Runs the fine-time step does not treat today (below 1 m sigma, overlapping supports, nothing to see across the
  passage): measured by the reading, not moved.

## Decisions

### D1 The fix is in the fine-time step, not in the pose graph

The correction reverses because of how two edges enter the graph (attached at the first pose; information from scans
that see nothing across the passage). Repairing that at the source means edges attached at the times of their
information (a three-node factor) or an edge's along-axis information reduced by its blind share (which needs the
fine-time scores before the graph, or a second graph pass), and a new capture format.

Inside the stretch none of that is needed for the result: along the axis the fine-time step can decide the nodes'
places from the scans and the two ends alone. The pose graph then matters there only through its correction at the pins
and across the axis. So the change is a different last step of the profile (D2), with no change to the graph, the
scores or the capture.

- *Alternative: the graph-level repair.* More right in principle, and the only one that also helps runs the fine-time
  step does not treat. Not chosen now: several times the work, a new capture format, and one stall to design it on. It
  is read in the prototype (D6, reading R1) so that the new recording can say whether it is needed.

### D2 The form: straight gaps

Supported windows are knots; windows without surfaces are not. The total correction along the axis is the line in time
through the pins and the knots.

- *Alternative F2: fit the profile to the total correction* (the same dynamic programme with the coupling on
  `c_w + d_w` instead of `d_w`). One principle for everything, but it changes the offsets of the supported windows too
  (the coupling then pulls toward an even slip everywhere), on the strength of one stall. Read beside the rule (D6); not
  the rule.
- *Alternative: keep today's interpolation of offsets and only clip the reversal.* A constraint without a model; the
  result would depend on where the clipping starts.

What the line assumes: between two places the scans fix, the odometry's error along the axis grew evenly in time. For a
stall (the scan matching does not hold the travel, the inertial terms carry it) the error's rate changes slowly, so an
even rate is the plain choice; it is still a choice. In the stall the scans show 0.11 m per window across the gap
(481 to 496) and 0.39 m per window after it (496 to 506): the rate is not the same over the whole run, so within the
gap it need not be even either.

### D3 The rule's text (to be frozen in `zigzag/RULE.md` as changes to `fine_time/RULE.md`; nothing else changes)

- **Correction.** `c(n) = a . (x_lc(n) - x_odo(n))` for a node `n`: its position after the loop closure minus its
  odometry position, along the run's axis `a` (section 2 of the rule). `c_w` = `c` at window w's middle node.
- **Knots.** (first pin, `c` at the first node at or after the pin), (the middle time of every supported window,
  `c_w + d_w`) with `d_w` from section 6 unchanged, (last pin, `c` at the last node before the pin's time or at it).
- **On the nodes** (replaces the last bullet of section 6): `T(t)` = linear interpolation in time through the knots. A
  node of the support at time `t` moves by `(T(t) - c(n)) a` from its place after the loop closure. Orientations do not
  change.
- **Effective offset** of a window: `e_w = T(middle time) - c_w`, which is `d_w` for a supported window. On the grid:
  `e_w` rounded to the nearest 0.1 m, a tie toward 0.
- **Section 7** (edges): `x_w` after = the middle node's position after this move. Blind windows and the informed share
  are as today.
- **Section 8** (when applied): condition 2 becomes "some window's effective offset is not 0 on the grid"; condition 3
  reads the held-out scores at the effective offsets on the grid. Everything else as today, including "lower", not
  "not higher": a profile that changes nothing a held-out patch can see is not applied.
- **The fall** of a weak run (every weak run, eligible or not): over the middle nodes of the run's windows in time
  order, with `s` = the sign of (`c` at the last minus `c` at the first; +1 when they are equal),
  `fall = max over w of ( max over v <= w of s c_v - s c_w )`. Given for the correction after the loop closure, and after
  the fine-time step for a run that was applied (`c` then includes the move). For a run that is not eligible the axis is
  the readings rule's axis turned as in section 2.
- The reported "largest offset" and "unsupported windows moved" use the effective offsets.

### D4 What is frozen, and when

`zigzag/RULE.md` is written from D3 and D5 and read by the conclusion reviewer against this design and the tool's code
before the first run of the new form. No constant of `fine_time/RULE.md` changes. If the rule misses a criterion of D5,
that is reported; the rule is not adjusted to pass.

(2026-11-25: frozen after the conclusion reviewer's reading. Its corrections, all before any run: knots at the middle
node's time, the rounding of the effective offset, criterion 1 taken over the gap and its two knots (the support's own
knots already fall by 0.10 m), criterion 2 on the offsets and not on every node, the by-hand residuals, a section on what
the rule assumes.)

What is already known when it is frozen, and therefore cannot count as a test: today's rule's offsets and the pose
graph's correction in every window of the stall (Context), from which the line's values in the gap follow by hand
(for example +0.70 m at window 486, where today's rule gives +3.25 m). The stall can show that the rule does what it
says and breaks nothing that is measured; not that the gap's windows are right.

### D5 Criteria on the stall capture (stated here, before any result of the new form is read)

Inputs as in `results.md` section 8: capture `stall_fmt3` (dMapper's own per-window information), its replay as the
trajectory, cache `fine_time/cache`. "Today's rule" = `rule_fmt3`.

1. **No reversal in the gap.** Over windows 481 to 496 the fall is at most 1e-6 m after the step (after the loop
   closure: 1.84 m; after today's rule: 2.24 m). True by construction: it checks the code. The fall over the whole
   support is reported; the supported windows' own knots already give 0.10 m at window 479.
2. **Supported windows unchanged.** The same windows are supported, each with today's rule's offset (the same grid
   index). Their nodes other than the middle node can differ from today's rule; the largest difference is reported.
3. **Applied**, with the held-out mean lower than at 0. Its value is reported beside today's 0.2745 m.
4. **Edges.** No edge is contradicted. 71 -> 84 and 74 -> 84 stay within their limits (0.221 and 0.211 m) at their time
   support; 75 -> 82 is reported beside today's -0.339 m (limit 0.315 m) and must not be classed contradicted;
   71 -> 81 stays unconfirmed and its residual is reported (by hand: about -3.5 m).
5. **Map checks** (the tolerances of `fine_time/RULE.md` section 9): the ghost line holds at most 50 returns in each of
   the bands -5..-4, -3..-2, -2..-1 and 0..+1 m; the 54 cm double within 5 mm of +0.015 m; facade out minus back within
   0.02 deg of -0.07; Wohnen hall minus room within 5 mm of +0.308 m; reference contacts >= 8 cm within one of 5, none
   >= 30 cm; far cross-storey faces >= 8 cm within one of 18, none >= 30 cm.
6. **The gap's own returns.** For the points of windows 482-495 (thinned to 0.07 m, placed), the share farther than
   0.20 m from the nearest reference point (the reference points of rule section 4 before the patches are taken): with
   the new form not more than one percentage point above today's rule's share. Reported for the loop closure's result,
   today's rule and the new form. These windows see nothing across the passage, so the share cannot confirm their place
   along it; a rise would show that the move put something where another visit saw nothing.

Read beside the rule; nothing in the rule depends on them:

- **F2** (D2's alternative): its offsets in the supported windows beside today's; its fall.
- **R1, the pose graph without the along-axis information of the unconfirmed edges** (`tools/loop_graph.py solve` with
  the information of each unconfirmed edge projected: `W - W u u' W / (u' W u)`, `u` the axis in the edge's frame): the
  correction at the pins and at every window of the support, its fall, and the largest change outside the support.
  This says whether the graph-level repair of D1 would change anything the fine-time step does not already decide.
- **R2, a stretch without scan evidence.** The line from pin to pin alone (as if no window were supported), beside the
  knots the scans give in 496-506: the difference per window. This is the size of the error the prior would make on a
  run the scans cannot time, on the one case where the answer is partly known.
- **R3**: window 481's knot at -0.5 m instead of -1.0 m (its value with every third point): the line in the gap.
- **R4**: the fall of every weak run of the capture (25 runs), after the loop closure.

On today's Livox 4 capture (`livox4_today`, run 481-496, sigma 1.46 m, applied by today's rule with 2 supported windows
of 16) and on the Livox 3 capture (`livox3_quiet`, run 585-601, not applied: edge 91 => 100 contradicted): the same
table. Required on both: no supported window's offset differs from today's rule; no edge that agrees today is
contradicted. Reported: whether the run is applied, the largest difference of a node's position to today's rule, the
falls, and on Livox 4 the map checks beside today's guard-on values. **If the Livox 3 run's status changes** (it would
move the Livox 3 result once the setting ships), the numbers go to the user before anything else.

On the synthetic corridor of `tests/test_loop_fine_time.py` (truth known), new cases:
- a correction that reverses inside the blind part, the true error growing evenly: every node of the gap within one grid
  step (0.1 m) of the truth after the step; the supported windows as without the setting;
- the true error growing only in the second half of the gap: the error of the line is reported and asserted to equal
  the analytic value (half the gap's error at its middle): the prior's known limit, held as a test;
- a right trajectory stays; the equal-jambs case is unchanged.

### D6 The prototype

`tools/loop_fine_time.py` gains `--straight-gaps` (the rule of D3), `--fit-total` (F2), and the fall in its summary and
windows table (correction after the loop closure, effective offset, total). `scripts/` in
`experiments/consistency/livox4_lever/zigzag/` holds the gap-returns measure (criterion 6) and the readings R1-R4. The
map checks reuse `fine_time/scripts/checks.py`. No dMapper run: the three captures, their replays and the point caches
exist (Livox 3's cache is the recording's own: no dMapper job may run on Livox 3 meanwhile).

### D7 In dMapper (after the user's go on the frozen rule's numbers)

- Setting `loop_closure_fine_time_straight_gaps` (bool, false) in `[loop_closure]`, outside the odometry settings hash;
  refused without `loop_closure_fine_time_placement`.
- `loop_fine_time_step`: the knots and the nodes' move of D3; the held-out read at the effective offsets; the report per
  run gains the falls; the fall of every weak run goes to the log and to the replay's JSON with the setting on or off.
  `loop_fine_time.csv` gains the correction and the effective offset per window.
- The score port, the curves and the capture format are untouched: `c` comes from the pose graph's result and the
  odometry poses, both present in a run and in a replay. A replay with the capture's settings stays bit for bit; a
  replay that only switches the setting is allowed (the curves fit either form).
- Tests: the corridor cases of D5 in `tests/loop_fine_time_test.cpp`, the prototype's numbers on them required exactly
  as in the existing port tests; with the setting off the existing cases unchanged (the check register shows additions
  only).
- If the setting is later shipped on, the old interpolation is removed with it (no second form kept switched off).

### D8 Order

1. Freeze the rule, prototype, read (now). Stop: numbers to the user.
2. The port, off by default (after the user's go).
3. The corridor recording: the recording check, a full run with the shipped settings and a capture, the fall of every
   weak run, the step with and without straight gaps from the snapshot; maps to the user.
4. The user's decision: on in the Livox file, off, or the graph-level repair.

## Risks / Trade-offs

- [The line is wrong where the error did not grow evenly] → it moves windows that have no evidence by up to 3.6 m from
  the pose graph's place on the stall (today's rule: 1.0 to 1.7 m). The size of the possible error is the gap's length
  times how uneven the rate is; reading R2 measures it where the answer is partly known, and the synthetic case holds the
  worst case as a number. A window in a gap sees nothing across the passage, so a wrong place along it shows little in
  the map; that also means the map cannot confirm it.
- [The knots carry the gap] → window 481's offset depends on the point density (0.5 m); the line in the gap moves with
  it (R3).
- [The held-out check is not independent and now decides a larger move] → unchanged from today's rule (master reviewer,
  2026-11-25); the reported held-out margin and the edge classes are the guard, with the limits said.
- [An edge the line leaves far behind] → 71 -> 81 stays a kept edge about 3.5 m from the trajectory along the axis. It
  is unconfirmed and stops nothing; it still shapes the pose graph outside the fine-time step (R1 measures by how much).
- [One recording, and the rule designed knowing it] → frozen before its results; the corridor recording is the test.
- [Runs the step does not treat keep the fault] → the fall is logged for every weak run, so the new recording shows it.

## Open Questions

- Whether a run in which no window has surfaces across the passage should get the line from pin to pin. Not in this
  change; R2 and the corridor recording's falls are the evidence for a later decision.
- Whether the graph-level repair is worth building. R1 on the stall and the corridor recording decide.
