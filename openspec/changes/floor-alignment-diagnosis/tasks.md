# Tasks

## 1. Start from a committed state

- [ ] 1.1 Commit `robust-loop-closure`'s uncommitted work as it stands, everything off by default: the loop-closure code (cross-floor pass, crop, time gap, information ratio, kernel, wrap fix), the tests, the golden TOML files, `tools/spot_check.py`, `tools/stairwell_check.py`, `tools/facade_check.py`, and the README entries. Then the two openspec changes. Verify: Release `ctest` passes; `44_short` bit-identical with the manifest apart from the parameters TOML; `git status` clean apart from openspec work in progress.

## 2. Odometry snapshot and reprocessing

- [ ] 2.1 `save_odometry_snapshot` (`OdometryConfig`, `TomlIO`, golden files) and the snapshot writer and reader (`src/infrastructure`, design decision 1): window fields, run state, window count and timestamps, odometry-settings hash with the explicit list of post-processing settings. Verify: a unit test round-trips a synthetic run's windows bit for bit, and refuses a changed timestamp and a changed odometry setting.
- [ ] 2.2 `OdometryPipeline::restore_odometry(snapshot)` in place of `run_odometry`, and `--from-snapshot <file>` in `run_cli` (design decision 2). Verify: a unit test on the synthetic loop recording — post-processing after a restore gives the same windows as after the odometry itself.
- [ ] 2.3 Bit-identity on real data: 44_short and the full 44, a run with the snapshot and a reprocessing from it, compared with `compare_baseline.py`. Also a reprocessing with X3's loop-closure settings against the existing `44_x3_s0`. Verify: IDENTICAL apart from the parameters TOML; reprocessing time reported (target ≤ 5 min on 44).
- [ ] 2.4 Snapshots of the four evaluation inputs (`44_s0/s1/s2`, `45f_p`) with the shipped odometry, kept in their result folders. Verify: four snapshot files; reprocessing each with shipped settings reproduces `44_lc_s*` / `45f_lc`.

## 3. Evaluation suite

- [ ] 3.1 `tools/evaluate_floors.py` (design decision 4): one table over the four inputs against the shipped references; documented in `tools/README.md` with its fixed parameters. Verify: on the existing shipped, X3 and X4 results it reproduces the numbers in `robust-loop-closure` tasks 4.1–4.2.
- [ ] 3.2 `tools/trajectory_seams.py`: check whether it compares trajectories without the chunk poses (0.49 m against 0.94 m in world coordinates on `44_lc_s2` / `44_x3_s2`). Fix it to world coordinates, or document what it compares. Verify: the two numbers agree, or the README says why they don't.

## 4. Diagnosis 1: why cross-floor edges move the gallery

- [ ] 4.1 A debug setting that logs the solved segment poses (read-only). With it, the per-segment correction difference X3 − shipped on all three inputs. Verify: a table of segments with the largest differences, in `findings.md`.
- [ ] 4.2 A debug list of excluded segment pairs for cross-floor edges. Reprocessing X3 without each edge group (shaft, gallery, end-of-recording; design decision 5), judged by the suite. Verify: a results table in `findings.md`; the group responsible named with its measurement.
- [ ] 4.3 From 4.1–4.2, one or two candidate remedies, each a reprocessing judged by the suite (e.g. registration-covariance weights for cross-floor edges). Verify: a results table; a recommendation for a follow-up change, or "no remedy found" with what was ruled out.

## 5. Diagnosis 2: the gallery drift

- [ ] 5.1 `facade_check` per 5 s time slice over 35–95 s on shipped `44_s0/s1/s2`, out and back separately. Verify: in `findings.md`, where and when the façade bends.
- [ ] 5.2 A diagnostic odometry run (`log_window_diagnostics`) on `44_s0` (diagnostics don't change the odometry; check against the snapshot). Compare the information ratio and weak direction along the gallery with 5.1. Verify: in `findings.md`, whether the bend coincides with weak windows, with confidence.

## 6. Diagnosis 3: the heading drift on the stairs

- [ ] 6.1 Heading per window over 88–100 s and 155–166 s: the result's yaw increments against the integrated gyro, from `log_window_diagnostics` and the IMU CSV. Verify: in `findings.md`, the windows where the 2–6° appear, on all three inputs.
- [ ] 6.2 Single-change runs, each a full odometry run judged by `stairwell_check` on the three inputs (two at a time, memory): IMU shifted −6 ms; the gyro-bias prior during the descent; `rotation_sigma_inflation_k`. Verify: a results table; a change that moves the floor offset as predicted, or what was ruled out.

## 7. Close

- [ ] 7.1 `findings.md` complete: the three questions, measurements with reproducing commands, interpretations, confidence; the follow-up changes proposed (names and one-line goals). Verify: every claim in it cites a measurement.
- [ ] 7.2 Update `robust-loop-closure` with what this change found (keep, change or drop X3's design). Verify: its tasks reference `findings.md`.
