# Tasks

> **Closed 2026-11-25 (housekeeping; the user: "all as suggested"). Parked 44 diagnosis; its snapshot reprocessing ships and is synced; information-weighted cross-floor edges shipped since (af7d538). findings.md is the record.**

## 1. Start from a committed state

- [x] 1.1 Commit `robust-loop-closure`'s uncommitted work as it stands, everything off by default: the loop-closure code (cross-floor pass, crop, time gap, information ratio, kernel, wrap fix), the tests, the golden TOML files, `tools/spot_check.py`, `tools/stairwell_check.py`, `tools/facade_check.py`, and the README entries. Then the two openspec changes. Verify: Release `ctest` passes; `44_short` bit-identical with the manifest apart from the parameters TOML; `git status` clean apart from openspec work in progress.

## 2. Odometry snapshot and reprocessing

- [x] 2.1 `save_odometry_snapshot` (`OdometryConfig`, `TomlIO`, golden files) and the snapshot writer and reader (`src/infrastructure`, design decision 1): window fields, run state, window count and timestamps, odometry-settings hash with the explicit list of post-processing settings. Verify: a unit test round-trips a synthetic run's windows bit for bit, and refuses a changed timestamp and a changed odometry setting.
- [x] 2.2 `OdometryPipeline::restore_odometry(snapshot)` in place of `run_odometry`, and `--from-snapshot <file>` in `run_cli` (design decision 2). Verify: a unit test on the synthetic loop recording — post-processing after a restore gives the same windows as after the odometry itself.
  - Done 2026-10-10: `OdometrySnapshot` (`src/domain/odometry_snapshot.h`), `src/infrastructure/odometry_snapshot_file.{h,cpp}` (format DMSNAP01 v1, FNV-1a settings hash over `TomlIO::POINTERS` minus the post-processing categories, plus the grid parameters and the motion-model correction), `OdometryPipeline::snapshot()` / `restore_odometry()`, `--from-snapshot` in `run_cli` / `main`, `save_odometry_snapshot` in `[lidar_odometry_debug_info]`. Tests: `tests/odometry_snapshot_test.cpp` (5 cases: round trip, settings, damaged file, restore checks, consistency after restore bit-identical). Golden files regenerated (one new key). Release `ctest` passes. CLAUDE.md's Run section documents it.
- [x] 2.3 Bit-identity on real data: 44_short and the full 44, a run with the snapshot and a reprocessing from it, compared with `compare_baseline.py`. Also a reprocessing with X3's loop-closure settings against the existing `44_x3_s0`. Verify: IDENTICAL apart from the parameters TOML; reprocessing time reported (target ≤ 5 min on 44).
  - Done 2026-10-10: 44_short and the full 44 with the snapshot are bit-identical with the manifests (apart from the parameters TOML: new keys); their reprocessings are bit-identical with the full runs (including the parameters TOML). The `44_s0` snapshot reprocessed with X3's settings is bit-identical with the old full X3 run. Times: 44_short 18 s against 179 s, 44 178 s against 1338 s, the X3 reprocessing 308 s.
- [x] 2.4 Snapshots of the four evaluation inputs (`44_s0/s1/s2`, `45f_p`) with the shipped odometry, kept in their result folders. Verify: four snapshot files; reprocessing each with shipped settings reproduces `44_lc_s*` / `45f_lc`.
  - Done 2026-10-10: `raw data/44_s0_snap`, `44_s1_snap`, `44_s2_snap`, `45f_snap` (`odometry_snapshot.bin`); each run is bit-identical with `44_lc_s*` / `45f_lc` apart from the parameters TOML.

## 3. Evaluation suite

- [x] 3.1 `tools/evaluate_floors.py` (design decision 4): one table over the four inputs against the shipped references; documented in `tools/README.md` with its fixed parameters. Verify: on the existing shipped, X3 and X4 results it reproduces the numbers in `robust-loop-closure` tasks 4.1–4.2.
  - Done 2026-10-10 (subagent, checked): every expected number reproduced on X3 against shipped. Storey-height columns added afterwards (finding 1.1). README entries for `evaluate_floors.py` and `facade_check.py`.
- [x] 3.2 `tools/trajectory_seams.py`: check whether it compares trajectories without the chunk poses (0.49 m against 0.94 m in world coordinates on `44_lc_s2` / `44_x3_s2`). Fix it to world coordinates, or document what it compares. Verify: the two numbers agree, or the README says why they don't.
  - Done 2026-10-10 (subagent, checked): the tool used the chunk-relative rows and never read `session_poses.mrp`. Now it compares world positions, with a warning if poses with the same index differ by more than 1 ms in time. `44_lc_s2` against `44_x3_s2`: 0.489 m before, **1.155 m at 178.1 s** (floor −2) after, matching an independent timestamp-matched comparison. `44_baseline` against `44_lc_s0`: 0.000 m. README notes the change.

## 4. Diagnosis 1: why cross-floor edges move the gallery

- [x] 4.1 A debug setting that logs the solved segment poses (read-only). With it, the per-segment correction difference X3 − shipped on all three inputs. Verify: a table of segments with the largest differences, in `findings.md`.
  - Done 2026-10-10: the segment poses are taken from the results' world trajectories at the segment start times instead of a new debug log (the same values, no code). Result in `findings.md` 1.1: X3 lifts the lower floors by 0.2–0.7 m and compresses the storeys (2.36–2.80 m against 2.67–3.15 m shipped); the likely cause is the vertical slip of shaft registrations.
- [ ] 4.2 A debug list of excluded segment pairs for cross-floor edges. Reprocessing X3 without each edge group (shaft, gallery, end-of-recording; design decision 5), judged by the suite. Verify: a results table in `findings.md`; the group responsible named with its measurement.
- [ ] 4.3 From 4.1–4.2, one or two candidate remedies, each a reprocessing judged by the suite (e.g. registration-covariance weights for cross-floor edges). Verify: a results table; a recommendation for a follow-up change, or "no remedy found" with what was ruled out.

## 5. Diagnosis 2: the gallery drift

- [x] 5.1 `facade_check` per 5 s time slice over 35–95 s on shipped `44_s0/s1/s2`, out and back separately. Verify: in `findings.md`, where and when the façade bends.
  - Done 2026-10-10 (subagent, checked): findings.md 2.1.
- [ ] 5.2 A diagnostic odometry run (`log_window_diagnostics`) on `44_s0` (diagnostics don't change the odometry; check against the snapshot). Compare the information ratio and weak direction along the gallery with 5.1. Verify: in `findings.md`, whether the bend coincides with weak windows, with confidence.

## 6. Diagnosis 3: the heading drift on the stairs

- [x] 6.1 Heading per window over 88–100 s and 155–166 s: the result's yaw increments against the integrated gyro, from `log_window_diagnostics` and the IMU CSV. Verify: in `findings.md`, the windows where the 2–6° appear, on all three inputs.
  - Done 2026-10-10: findings.md 3.1 — no turn on the stairs beyond the ±3° noise; the floors' 2–6° accumulates slowly where nothing known is in view. Computed from the shipped results' trajectories and bias JSON, not from `log_window_diagnostics`.
- [ ] 6.2 Single-change runs, each a full odometry run judged by `stairwell_check` on the three inputs (two at a time, memory): IMU shifted −6 ms; the gyro-bias prior during the descent; `rotation_sigma_inflation_k`. Verify: a results table; a change that moves the floor offset as predicted, or what was ruled out.
  - Set aside 2026-10-10 (user: concentrate on the loop-closure fix). Finding 3.2: these settings can only reduce the unmapped-stretch drift a little.

## 7. Close

- [ ] 7.1 `findings.md` complete: the three questions, measurements with reproducing commands, interpretations, confidence; the follow-up changes proposed (names and one-line goals). Verify: every claim in it cites a measurement.
- [ ] 7.2 Update `robust-loop-closure` with what this change found (keep, change or drop X3's design). Verify: its tasks reference `findings.md`.
