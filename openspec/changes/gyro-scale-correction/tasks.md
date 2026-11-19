# Tasks

## 1. The setting

- [x] 1.1 `ImuCalibration` (gyro scale, default ones) in `src/domain/imu_sample.h`; `RecordingSource::read_imu` and
  `load_imu` take it and scale the parsed double before the float store (both CSV paths, axes at 1.0 untouched);
  `load_data` passes it from the settings. Verify: builds, `region_owners.py` lists only the files named in design.
- [x] 1.2 Settings `gyro_scale_x/y/z` in `OdometryConfig`, `TomlIO::POINTERS` and the new `[sensor_calibration]`
  section; one log line with the factors when any differs from 1; the keys in `added_later` of the snapshot hash.
  Verify: the golden TOML tests save the section; the scale runs' log line shows the factor loaded from a TOML;
  snapshot and capture tests with defaults still match (step checks: replays identical).
- [x] 1.3 Unit test: `load_imu` on a small CSV written by the test, with scale 1 (values equal the unscaled read) and
  with z 1.004 (each z value equals `float(double * 1.004)`, x and y unchanged). Verify: `ctest` passes.
- [x] 1.4 Golden TOML files regenerated (`LO_UPDATE_GOLDEN=1`) in their own commit. Verify: `ctest` passes.
- [ ] 1.5 Defaults bit-identical: `tools/step_checks.py` (includes 44_short against its manifest). Verify: all pass.
  2026-11-19: build, 164 tests, guards, both replays and the term table IDENTICAL; 44_short differs only in the saved
  `dMapper_params_*.toml` (the new section). `tools/end_runs.py` then the params hash in the 44 / 44_short manifests
  (own commit), after checking that nothing else differs.
- [x] 1.6 Docs: the guide's orientation section and settings reference (new keys, what they do, where the Livox
  value comes from); front-end and inertial knowledge files (the replay limit, the clock-rate ambiguity). Verify:
  pages carry the nav line and "See also".

## 2. Confirmation runs (one at a time, background, the gate)

- [ ] 2.1 Livox 4 original with the stage 8 set (candidate + `ahrs_gain = 0` + `gyro_scale_z = 1.004`). Verify:
  windows stopped at the time limit = 0 in this run and in `candidate_gain0_livox4_gz0040`; `trajectory_seams.py`
  against that result (0 m only if the rare one-float-step differences of the CSV parser don't matter; otherwise it
  counts as a repeat member); wall measures, plan fit and the full sheet as in 2.3.
- [ ] 2.2 Livox 3: measure its yaw scale from the bias-against-rate slope of `candidate_livox3`
  (`s8_bias_compare.py`; done 2026-11-19: slope z -0.26 / -0.29 %, correlation -0.70 / -0.78, against Livox 4's
  -0.22 %, so the same direction and size: 1.004 is used), then a full run with the stage 8 set and the factor. Verify: run check
  (`verify-run`), start-to-end and loop corrections against `candidate_livox3`, full sheet for the user.
- [ ] 2.3 Repeat runs on Livox 4 with tiny changes (`verify-robustness`, two or three members). Verify: wall measures
  (`wall_precision.py --height 0.5`) and plan fit of every member, passage and end-stretch speeds, full sheets.
- [ ] 2.4 Results into a summary file and this file, no verdict; the user's look at the Livox 3 and repeat sheets.

## 3. Shipping (after the user's decision)

- [ ] 3.1 `settings/livox-vertical.toml`: the stage 8 set with a header line on each value's source; golden TOML
  regenerated in its own commit. Verify: `ctest` passes.
- [ ] 3.2 Livox baselines renewed (the old ones renamed with a date suffix), `tools/end_runs.py`. Verify: manifests
  written, L2 runs bit-identical.
- [ ] 3.3 Stage 8 (`livox4-passage-and-flyaway`) tasks pointed at this change; knowledge files' experiments entries;
  master-reviewer on the push.
