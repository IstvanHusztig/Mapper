# Region: verification

<!-- docs-nav -->
> [dMapper docs](../README.md) › Region knowledge files · [All documents](../README.md) · [Regions](README.md)

Whether a result is what it claims: does a change change no number, did a run behave, is a recording usable, how far
do results move under meaningless perturbations, how two maps differ. The region owns the checks (the verification
kit, the baselines, the analysis tools in `tools/`, the four verify skills, the test helpers) and never edits `src/`.
It gives numbers with their coverage, never a verdict about the map: that is the user's, by eye.

## Contract

**Takes:** a worktree with its Release build (`out/build/x64-Release`), result folders (`scan_lio_*.laz`,
`trajectory_lio_*.csv`, `session_poses.mrp`, `dMapper_results_*.json`, `odometry_snapshot.bin`), run logs, window
captures and the replays' table files (`*_window_terms.csv`, `*_window_features.csv`), raw recording folders
(`imu*.csv`, `lidar*.laz`, `timing*.json`), and this workstation's paths (`tools/local_paths.toml`).

**Gives:**
- `step_checks.py`: one line per check and `ALL HOLD` / `FAILED: ...`, exit 0 only if all hold. "IDENTICAL" everywhere
  means the change changes no number; that is this command's word, not the change author's.
- `end_runs.py`: the full 44, Livox 3 and 44_short against their manifests, with odometry time and cost; exit 2 when a
  dMapper process runs that did not come through the gate (nothing is started), 1 when a run or comparison fails.
  `step_checks.py` covers 44_short only: its `ALL HOLD` says nothing about the full 44 and Livox 3, which a change needs
  once, from `end_runs.py`.
- `run_replay.py`: the replay's summary, its cost, and how many windows stopped at the time limit (above 0: the result
  depended on the machine's load, run again). Exit code: the replay's.
- Readings with coverage under the agreed contract (`openspec/changes/sensor-neutral-yardstick/criteria-proposal.md`,
  user, 2026-11-12): missing evidence is NOT MEASURED, never PASS; diagnostic triggers are not map verdicts; an
  improvement is resolved only when max(candidate) < min(reference) over the same perturbation family, overlap is
  inconclusive (not "equal"); geometry, revisits and sharpness are never averaged into one score; odometry and final
  stages are checked separately; no universal 1 m loop-correction fail rule.

**What the others rely on (and what it relies on in `src/`):**
- The gate (`kit_lib.Gate`, `tools/kit_lib.py:143`): at most `replay_slots` = 2 replays at once, none beside a full run
  (a "full" holder has the gate alone), a replay starts only when `memory_factor` (3.0) × capture size +
  `memory_reserve_gb` (2.0) is free; a job waits before it starts, a running one is never paused; a lock whose process
  has ended is ignored. Locks in `<guards>/gate`.
- What the gate does not cover: `perturbation_sweep.py` and a dMapper started by hand do not go through it, and only
  `end_runs.py` looks for a dMapper process that did not. A sweep beside a replay or a full run can push a window
  into the time limit: start sweeps only when nothing else runs.
- The references: `<guards>/refs` (guard outputs as exact hexadecimal doubles, the 44_short replay's term tables) with
  `recorded.json` naming the commit; `--record` refuses with uncommitted changes in `src/`, `tests/`, `CMakeLists.txt`
  and runs each guard twice. The manifests `tools/baseline/{44,44_short,livox3,livox4}.manifest.json`, compared by
  SHA-256 with only run-specific values normalized (`compare_baseline.py`; exit 0 identical, 1 different, 2 error).
- Exit codes the skills read: `run_health.py` 2 incomplete evidence, 1 triggers, 0 complete without triggers;
  `perturbation_sweep.py` 2 incomplete or invalid, 1 triggers or stability violations, 0 complete; `imu_health.py` 1 on
  a failed check.
- Formats owned by the orchestrator's code that the tools parse: the snapshot `DMSNAP01` v1
  (`snapshot_trajectory.py`; source `src/infrastructure/odometry_snapshot_file.cpp`), the log lines `window diagnostics
  N:`, `gyro bias [deg/s] window`, `loop closure: N loop(s) accepted, largest segment correction` (`run_health.py`), the
  verdict summary's `time limit` count (`kit_lib.time_limit_windows`), the table columns (`window_terms.py`), the
  output files. A change of one of these in `src/` breaks a tool silently unless its test catches it.

## Owns

All of `tools/**`, the verify skills and the test helpers (`docs/regions/owners.toml`), grouped by purpose:
- **The kit:** `step_checks.py` (build, unit tests, characterization guard, loop guard, the two capture replays, the
  44_short replay's term table, 44_short against its manifest; about 7 min), `end_runs.py` (about 50 min; `--debug`
  adds the Debug build and `ctest`), `run_replay.py` (the gate), `kit_lib.py` (paths, build through `vcvars64.bat`, the
  gate, `run_dmapper` with wall/CPU time and peak memory; Windows only), `local_paths.toml` (the one file to change on
  another machine), `region_owners.py` (the ownership list's check and `--diff`), `function_sizes.py`,
  `check_agent_setup.py` (Claude/Codex workflow parity).
- **Baselines:** `baseline/*.manifest.json`, `compare_baseline.py` (`--tolerance`, `--write-manifest`),
  `cut_recording/cut_recording.cpp` (the `cut_recording` target), `merge_chunks.sh`.
- **Recording checks:** `imu_health.py` (rate, gaps, rest at start, gravity, bias, false gyro offsets, noise),
  `imu_time_offset.py`, `restamp_imu.py`, `fix_imu_timestamps.py`, `stationary_plane_noise.py`.
- **Run checks:** `run_health.py` (bends, slides, bias, velocity chi², closure proxy, loop closure, from a log with
  `log_window_diagnostics`), `perturbation_sweep.py` (base plus six perturbations, identity and resume checks),
  `trajectory_seams.py`, `snapshot_trajectory.py`, `window_terms.py`, `convergence_summary.py`, `live_run_monitor.py`.
- **Map checks:** `map_quality.py` + `map_quality_lib.py` (recording 44's three measures), `wall_precision.py`,
  `holdout_sharpness.py`, `eval_odometry.py`, `spot_check.py`, `stairwell_check.py`, `facade_check.py`,
  `floor_shape_check.py`, `evaluate_floors.py`, `cloud_context.py`.
- **Plan overlay:** `plan_fit.py` then `plan_section.py` (lines at height H; `section_lib.py` shared with
  `wall_precision.py`). "Overlay" means these two, cut 2 m above the floor of the storey shown.
- **Trace tools left from the Livox 4 investigation:** `analyse_livox4_rotation_trace.py`, `analyse_ndt_cells.py`,
  `analyse_ndt_guard_trace.py`, `analyse_ndt_refresh_event.py`, `compare_window_traces.py`, `flyaway_timeline.py`,
  `ndt_cell_history.py`, `run_livox4_gain_comparison.py`, `run_livox4_rotation_trace.py`, `trace_extract_states.py`,
  `trace_range_summary.py`, `trace_summary.py`. None of them runs the mapper except the two `run_livox4_*`, which say
  they need explicit approval.
- **Refactor helper:** `rename_identifiers.py`. **Documentation:** `tools/README.md`.
- **Skills:** `.claude/skills/verify-recording`, `verify-run`, `verify-map`, `verify-robustness` and their Codex copies
  under `.agents/skills/` (identical today).
- **Test helpers:** `tests/synthetic_recording.{h,cpp}` (namespace `synthetic`, the two-lap loop recording),
  `tests/synthetic_recording_test.cpp`, `tests/serial_executor.h`, `tests/test_verification_tools.py`.
- Not owned: the characterization guard (`tests/scan_to_map_test.cpp:1011`) and the loop guard
  (`tests/window_capture_test.cpp:238`), in shared test files; capture and replay code in `src/` (orchestrator). The
  captures, references, gate locks and run folders live outside the repo, under `guards` of `local_paths.toml`.

## How to check your work

- **Python tool tests:** `python -B -m unittest discover -s tests -p test_verification_tools.py -v` (no data, no mapper):
  `trajectory_seams` refuses unequal or non-increasing timestamps and NaN, places chunk rows with `session_poses.mrp`;
  `snapshot_trajectory` reads world positions and refuses truncated, extended or corrupted files; `run_health` never
  passes with missing diagnostics, an incomplete or cancelled run, duplicate or malformed lines; `perturbation_sweep`
  keeps failed members' files, exits non-zero, runs seven members and resumes without relaunching, refuses a resume
  whose input, settings, executable or output changed.
- **The kit itself has no unit test**: `step_checks.py`, `kit_lib.py`, `run_replay.py` and `region_owners.py` are checked
  by the acceptance runs of `openspec/changes/region-agents/tasks.md` 2.1–3.1 (references re-recorded identical with
  stage 6's, a third replay waits while two run, a dead lock is ignored, a made-up branch outside its region fails).
  `python tools/region_owners.py` must exit 0 on the list.
- **C++ helpers:** `lidar_odometry_tests -tc="loop recording*"` (the synthetic recording); the back-end's and the
  scan-to-map tests use it too, so run the whole `lidar_odometry_tests` after a change to it.
- **A tool against a known answer:** each `tools/README.md` section records a reference reading (e.g. `stairwell_check.py`
  on `44_lc_s0`: floor −1 30 cm off floor 0; `holdout_sharpness.py` reproduces +1.85 / −0.87 mm within 0.05 mm; the
  snapshot reader matched 1517 logged window starts of a Livox 2 run within 0.5 mm). A changed tool reproduces its
  reference before its numbers are used.
- **How results are reported** (memory notes): every run that is compared, shown or reported goes through `verify-run`,
  map comparisons through `verify-map`, `verify-robustness` before a setting is called better, `verify-recording` for new
  data; show the numbers, then stop and ask. Never claim a direction from log proxies: hand over the LAZ paths and the
  overlay images for the user's look, and before a map-changing push the before/after images. Say before starting runs
  side by side that `real_time_threshold_seconds` (10 s) makes them load-dependent; baselines run one at a time. Check
  free disk before a batch and delete finished experiment folders (never inputs, baselines or snapshots in use).

## Known flaws

- **C38** (reported; also the front-end's): recording 44's gyro at 253–259.7 s turns 89.6° over 255.86–259.93 s against
  0.3–2.9° per window from raw-scan ICP, with the accelerometer still. The recording check shows it: `imu_health.py`'s
  false-offset check found 22–31 °/s on 44 at 252–261 s (`docs/sensor-neutral-retune-plan.md` F1). 44 and 44_short stay
  regression guards with this recorded defect, not a calibration source (agreed contract).
- Open reviewer findings on tools (master-reviewer 2026-10-18, memory note `map-precision-leaks-state.md`; not in the
  tracker): `wall_precision.py:152` takes the pair noise from raw point counts (correlated points make it near 0) and
  1.25 is the SD, not the expected |value|; `holdout_sharpness.py:81` checks the times with `np.allclose`, which can't fail
  on absolute times (`array_equal` intended).
- `map_quality.py`, `evaluate_floors.py`, `facade_check.py` and `plan_fit.py`'s defaults are tied to recording 44 (its
  constants at the top of `map_quality_lib.py`).

## To split in the first job

Not applicable to the Python tools. The region's C++ (`tools/cut_recording/cut_recording.cpp`,
`tests/synthetic_recording.cpp`) has no function of 80 lines or more (`function_sizes.py`, 2026-11-18).

## Experiments

- **2026-11-20, terms at the truth** (a5b422a; `consistency-harness` 5.1-5.2; `tests/terms_at_truth.{h,_test.cpp}`,
  `tests/accelerometer_noise_test.cpp`): accelerometer terms at the true state lie within their derived bounds in every
  motion (largest ratio 0.58; exactly 0 quadrature at rest and constant acceleration). The rotation edges' float floor
  in motion is 4.6e-7 rad, 1.6-1.9x the float32-storage bound (probably Fusion's float arithmetic; asserted below 0.1 of
  the edge sigma). The velocity prior at the truth leaves exactly |a| dt (F2a). D4: edge variance at ARW/sqrt(dt)
  measured 1.009 / 1.021 / 1.009 of assumed; joint F P F^T + Q exact to 1e-16; gap noise 0.998 of D^2 gap. F-b: in the
  sample-aligned fixture whitened triples have variance 0.997 and no correlation; with the Livox timing the triples sit
  about 5 % under 1 and the start term about 1.9, as the supports predict. Measured on the way: the anchor rule
  (`window_equations.cpp:27`, `>= span_s`) puts anchors 12 ms apart in 89 % of steps for a 10 ms span at 500 Hz
  (rounding of timestamps); the filter's start-up ramp levels on the accelerometer for 3 s even with `ahrs_gain = 0`.

- **2026-11-20, unit-level cases** (54c04db; `consistency-harness` 5.4; `tests/unit_cases_{inertial,ndt}_test.cpp`):
  all seven reproduce the audit's numbers and are isolated by a paired control: K3 the prior mean over the gap is the
  carried end velocity, -2 mm/s per m/s^2 over 2 ms; K4 the tail -0.75 mm at 1 m/s^2, the next first pose -1.664 mm at
  2 m/s^2 (shipped tail 30 / span 20: -6.0 / -6.3 mm); K6 rest lost from 1.7528 deg of filter tilt (= 2 asin(0.3 / 2g));
  K7 2 g/k at s = 0.5 (g / (s k) in general); K9 thinnest usable normal sigma 3.162 mm axis-aligned, 2.238 mm at 45 deg;
  K10 1e12 against 3.283e15. **A2 (new):** a constant-velocity window (2 m/s, no acceleration) passes the rest test;
  with `imu_always_on` it is solved as at rest: start 0.646, end 0.010 m/s, move 0.035 m against 0.716 m (the paired
  control without rest keeps 2 m/s). Characterisation only; markers are task 5.8. Hand-built analytic windows (not the
  generator), empty map.

- **2026-11-20, the model-world recording** (f758517, merged e9856e8; `consistency-harness` 4.1-4.2):
  `tests/synthetic_inertial.*`, `tests/synthetic_routes.*`. Gyro sample k = the interval ending at node k (sample 0 the
  5 ms step), accelerometer sample k = the interval starting at node k; a hit of node k is stamped `nextafter(node k)`.
  Window 0 starts after the 60 001st accepted point (stationary prefix of about 40 points per node, rests of 4 s; one
  IMU sample beyond the last window). Discretisation per 2 ms step: Magnus 2.3e-9 rad (bound 1.2e-8), left-end body
  force 4.4e-3 m/s^2 (1.55e-2), position quadrature 1.7e-8 m (5.5e-8); over 14 s of walking and turning a strap-down
  integration differs by 1.5e-6 rad, 2.5 mm/s, 1.7 cm (a property of the estimator's convention, mostly in the
  turn). Walking: force rms 0.52 / 0.49 / 0.89 m/s^2, rate rms 11.3 / 11.5 / 31.8 deg/s. Cut: R1 79, R2 124, R3 35
  windows; every eligible hit at its node (0 missing, 0 misplaced); R2's blind stretch 8 m with no structure hit.
  Decimation coverage of every node is a measurement on these densities, not a guarantee. New tests 4.1 s.

- **2026-11-20, the exact control** (c345927, merged d763f7f; `consistency-harness` task 2.1):
  `tests/truth_statistics.{h,cpp}` (`normalised_error`, chi-square cdf / quantile / interval,
  `power_of_variance_ratio`, `runs_for_power`, a platform-stable `Rng`) and `tests/truth_statistics_test.cpp` (a
  6-node, 2-state linear-Gaussian chain). Batch and Kalman elimination of the same factors agree to 2e-15; the honest
  sum lies inside the 1 % interval (2 of 200 seeds rejected); for a variance multiplier of 1.5, one comparison, 1 %,
  90 % power: M = 96 runs of 2 dof, measured rejection 88.5 % (claim x1.5) and 93 % (claim /1.5); the normalised move
  averages 1.002 (scalar) and 1.001 (3 unknowns), the old form 0.501. The 5 cases take 0.5 s; step checks ALL HOLD.
- **2026-11-18, the kit in the repo** (e17247e): a replay's peak memory is 2.9× its capture file on both captures (0.2 GB
  for 44_short 20–40 s, 1.6 GB and 5.5 cores for Livox 3 100–130 s). `docs/code-split-and-region-agents-plan.md` stage 7.
- **2026-11-12, tool validation:** the verification tests pass (10, then 11 with the shipped-TOML case); the snapshot
  reader on a Livox 2 snapshot (273,060 nodes, 1517 windows) matched the logged window starts within 0.0005 m; a Codex
  review found missing-window, cancellation, empty-accelerometer and frozen-denominator defects, all corrected with tests.
  No real seven-member sweep had run through the new orchestrator then.
  `openspec/changes/sensor-neutral-yardstick/tool-validation.md`.
- **2026-11-12, recording checks:** `imu_health.py` on Livox 1 and 44_short: exit 0 with WARN (handheld rest gives noise
  upper bounds); Livox 1 501 Hz, 6.5 s initial quiet span, gravity 1.0006 g. `sensor-neutral-yardstick/evidence.md`.
- **2026-11-10, sensor checks:** `imu_health.py` found the L2's false gyro offsets (11–32 °/s for 2.5–4 s) on 46, before,
  take2 and 44. `docs/sensor-neutral-retune-plan.md` F1.
- **2026-10-18, plan_fit corrected** (f8ce5e0): the first version (cost in whole pixels, turned about the map origin) gave
  a wrong start-room fit (−2.18°); fixed, gallery − start room is −0.04° (old baseline) and −0.46° (new).
  `tools/README.md`, memory note `corridor-state.md`.
- **2026-10-18, hold-out sharpness:** sharpness on all points rewarded consistency's overfitting (+1.85 mm on never-fitted
  points). `openspec/changes/map-precision-leaks/findings.md` F2.
- **2026-10-16, cut_recording:** a cut from 0 s drops about 3,900 points before the first IMU sample and shifts every
  window; a cut from −10 s reproduces the full run's trajectory. `tools/README.md`.
- **2026-10-15, timing dependence:** under memory pressure one window took 10.02 s (limit 10 s) and the odometry diverged
  from window 142; the reason for the gate's time-limit count and sequential baselines. Tracker C13.
- **2026-10-13, map-quality harness:** shipped final vs odometry only on 44: façade −0.40/−0.73° vs −3.87/−4.19°, revisit
  1.33°/2.29°/0.11 m vs 0.18°/1.52°/1.00 m, jitter 0.94/4.49° vs 0.40/1.29°. `tools/README.md`, `map-quality-harness`.
- **2026-10-10, trajectory_seams in world positions:** before, it compared chunk-relative rows and missed what loop
  closure moves; earlier loop-closure comparisons with it understate the differences. `tools/README.md`.
- **2026-10-09, IMU time offset:** 44's IMU was 79 ms behind the LiDAR at the start, 52 ms at the end; restamped.
  `tools/README.md`.
- **2026-10-01, golden baseline harness:** `compare_baseline.py` and the manifests.
  `openspec/changes/archive/2026-10-01-golden-baseline-test-harness/`.

## Ideas rejected

- Reading an overlap of envelopes as "no difference", a trigger as a map failure, and every loop correction over 1 m as a
  failure: replaced by the agreed contract (2026-11-12).
- Index-matched or truncated `trajectory_seams` comparisons with a timestamp warning: the tool now refuses unequal
  timestamps (`test_strict_seams`).
- An aggregate PASS from `run_health.py` with missing evidence; approximate bend sigmas and inferred stop causes are no
  longer reported (`verify-run` skill).
- Yaw-only `eval_odometry.py` for lagging stretches: it can't see them; `trajectory_seams.py` instead (CLAUDE.md).
- Sharpness on all points to judge a fitting stage: rewards overfitting; `holdout_sharpness.py` instead.
- `plan_fit.py`'s point images or the perimeter outline shown as "the overlay": not what the user means
  (memory note `feedback-overlay-tool.md`).
- A second size tool `function_lengths.py`: removed 2026-11-18, its reader moved into `function_sizes.py`.

## Open questions

- The kit's acceptance runs (`region-agents` tasks 2.1–2.4, 3.1) are not ticked yet; a unit test of `step_checks.py`'s
  paths reading is planned "if it fits".
- The two reviewer findings in `wall_precision.py` and `holdout_sharpness.py` (above) are unfixed.
- A general map-quality report without recording 44's constants (planned feature `map-quality-report`, tracker
  preamble); no absolute cave-wall limit is agreed until there is cave data.
- `run_replay.py` gates only `--replay`. A `--from-snapshot` run (the back-end's reprocessing) counts as a full run and
  is started by the orchestrator; a gated command for it is to be added with the back-end's first job.
- Whether the trace tools of the Livox 4 investigation are still needed, or can go once its evidence is archived.

## See also

- [Regions](README.md)
- [tools/README.md](../../tools/README.md)
- [The agreed evaluation contract](../../openspec/changes/sensor-neutral-yardstick/criteria-proposal.md)
- [Code-review tracker](../code-review-tracker.md)
- [Code-split plan](../code-split-and-region-agents-plan.md) (stage 7: the kit)
- [All documents](../README.md)
