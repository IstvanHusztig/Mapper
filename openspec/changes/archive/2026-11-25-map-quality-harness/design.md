# Design

## Context

- The analysis scripts of the last two days are in the job folder: `facade_lines.py`, `pair_reg.py`, `plan_anchor.py` (with `plan_reg.py`), `wall_np.py`, `yaw_diff.py`, `true_bias.py`, `build_cache.py`.
- `tools/evaluate_floors.py` already runs the stairwell, façade and railing checks.
- The diagnostics written during the horizon work are on branch `odometry-map-horizon-wip`, together with horizon code that must not come to master.

## Decisions

1. **Harness in Python with numpy.**
   - The user installed numpy. `CLAUDE.md` keeps repo tools pure Python unless a change decides otherwise, and this change decides it for `map_quality.py` and its helpers.
   - The harness reads result folders as the existing tools do: chunk LAZ through CloudCompare's ASCII export, `session_poses.mrp`, and the chunk CSVs.
   - It caches the placed points in a cache folder next to the result.
2. **Recording 44's geometry stays explicit.**
   - The façade band, the revisit pairs and the plan anchor are named constants for recording 44, as in `evaluate_floors.py`.
   - Other recordings get only the plan-free parts: revisit consistency, jitter and the spread.
3. **Diagnostics without cell ages.**
   - The heading diagnostic keeps the yaw correction against the guess and the window's total yaw stiffness.
   - The split by cell age needed the horizon's stamps, and it was uninformative: it measured a cell's last update, not how far back its information reaches.
4. **Silent events through the Logger port (`log_warning`).**
   - Counters live in the window loop's carry.
   - `optimize_lidar_odometry` already takes a Logger and logs "big jump"; it gains a counter it returns to the loop.

## Risks

- **Nondeterministic log order:** a log line written inside a parallel loop would appear in varying order. So events are counted per window and logged after the window, outside the parallel parts.
