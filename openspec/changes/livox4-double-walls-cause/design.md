# Design

## Context

Three stages can make a double wall: the odometry (two passes end up offset), loop closure (the graph moves poses), and
the consistency pass (it moves nodes inside windows). The measure that separates them follows the same wall returns
through the stages: O (poses of the odometry snapshot), L (after loop closure, consistency off), F (the final cloud).
Codex built that measure (`doubles/codex/compare_stages.py`, `phase_stage.py`, `stage_run.py`) and a graph tool that
re-solves the loop closure with one edge removed, in its own worktree (`C:/Repo/dMapper-worktrees/codex-doubles`,
`edge_diagnostic`); its all-edge control reproduces the native poses within nanometres.

What the code does today (read 2026-11-22): every loop edge, same-floor and cross-floor, is weighted by its registration's
normal matrix (`loop_edge_information`), falling back to the fixed sigmas (5 cm, 0.3 deg); only cross-floor candidates
are gated by an information ratio; edges are accepted by fitness (0.5) and correction (6 m) and pruned by residual. So an
edge that matches one family of parallel surfaces is not refused, and one that locks onto the wrong parallel surface is
well-constrained across the wall and wrong. The odometry's map is rebuilt from its newer half when the path passes
`sliding_window_trajectory_length_threshold` (200 m).

Existing odometries with snapshots: the reference (`robustness/off_v3/base`), six `off_v3` and seven `edges_v3` members,
Codex's `lever_loop` (first lever-arm vector), and the new base once it finishes. Reprocessing one takes minutes.

## Goals / Non-Goals

**Goals:**
- On the corrected-arm base: where the doubles are, at which stage each is made, and by which loop edges.
- For loop closure: the properties that tell the edges that make doubles from those that remove them, and a candidate rule
  tested across the family of odometries.
- The share of the map horizon and of the consistency pass, each measured on its own.

**Non-Goals:**
- No fix, settings or baseline change inside this change. No rule chosen on one run. No conclusion that an edge is false
  from its influence alone (an edge can be right and its correction spread over the wrong poses).

## Decisions

1. **Reproduce before building on it.** Codex's O / L / F numbers and the effect of removing the 45 s / 104 s edge are
   repeated on the new base with its tools, and one pair is checked by hand from the raw returns. Where the corrected arm
   changes the picture, the new base counts.
2. **Loop closure first, on a family.** For each of the existing odometries: the O -> L change of every measured wall pair,
   and the edges that move the pairs most (leave one out, other edges held; and with the pruning repeated, reported
   apart). The edges are then described: correction size against the path between the two segments, fitness, the spectrum
   of the registration's normal matrix, whether the matched surfaces span two directions, residual before pruning.
3. **Candidate rules, tested by reprocessing only** (each a question, none adopted here): refuse or free the direction a
   registration does not determine (the same-floor counterpart of the cross-floor information-ratio gate); bound a
   correction by what the odometry could have drifted over the path between the segments; verify an edge on geometry kept
   out of the registration; the order and criterion of pruning. A rule is reported by the doubles it makes and removes in
   every odometry of the family, beside the unchanged loop closure.
4. **Map horizon:** one odometry run with a longer horizon (memory and time cost reported); the O gaps of pairs with one
   visit before and one after the rebuild, against the base.
5. **Consistency pass:** reprocessing with it off and with equal axis weights; wall thickness, the within-pass spread and
   the pair gaps at L and F; the node path length as a check.
6. **The odometry's gaps last:** the corridor stretches, the carried gravity and velocity, the hand-off, with new captures
   (the old ones hold points without the lever arm).
7. **Research on request:** an agent that needs outside knowledge (loop verification, degenerate registration) asks in
   its hand-back; the orchestrator decides. A derivation is checked on a numeric case before code relies on it.
8. **Model and effort:** readings and rule design Fable/high; runs, reprocessing and tables Sonnet/medium; region jobs
   Opus/medium. The orchestrator says which before each phase.

## Risks / Trade-offs

- The measured wall pairs are a selection (plan walls of the start storey; Codex's face search on four storeys): counts
  describe coverage, and a rule tuned on them can fit the selection. Held-out walls and the user's eye are the check.
- A loop rule that refuses ambiguous edges also refuses corrections the odometry needs (the reference run's 4.3 m gaps
  are closed by loop closure): both directions are counted for every rule.
- With the corrected arm the odometry changes, and the leading edge of today's analysis may not exist: phase 1 of the
  tasks decides what the later phases look at.
- Precision before speed: a longer map horizon or more loop verification costs time and memory; the cost is reported, not
  avoided.
