# Prepared short Livox4 gain-only comparison

User requested preparation and notification before any processing run. No
dMapper run has been launched. This is separate from the disabled AHRS-gravity
experiment and does not advance Charlie's larger roadmap gates.

## Scope

- Original input: `C:/Users/User/Downloads/dLidar/raw data/livox/4`.
- External bundle: `C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ahrs-gain-prefix`.
- Prepare prefix with existing `cut_recording`, start=-10 s, duration=135 s
  relative to the first IMU sample. This retains all original early points and
  startup, ending about 125 s into the recording. Starting at zero would risk
  dropping LiDAR points preceding the first IMU sample and changing initial map.
- Evaluate onset W298–302 (106.92–108.36 s) and velocity buildup through W325
  (about 117 s), with earlier trajectory as preservation control.
- Two sequential runs: frozen Huber3 control, then the gain-only correction.
  Estimate 15–30 minutes total processing/collection, subject to measured prefix
  timing. Build and input preparation are separate from this run estimate.

## Isolation and gates

Control uses the frozen `full-livox2-huber3` executable/runtime, source cbca36d.
Candidate source fc73404 (gain patch b62955b plus test include-path correction)
in separate `out/ahrs-gain-worktree` is cbca36d plus
only the two adapter runtime files; added regression test/registration do not
change runtime algorithms. The gravity-model experiment and stronger gyro
weighting are excluded. Release preset, AUTO CPU selection, precise floating
point, identical original Livox-vertical TOML, alpha0.1/beta0.1, Huber cutoff3.

The runner requires explicit `--approved`, prepared-file hashes, fresh output,
15 GiB available and exclusive dMapper cache ownership. It clears inherited
DMAPPER experiment flags and enables only Huber3. Control must finish normally
and its first325 full snapshot window payloads must match the original full
Livox4 baseline exactly (poses, times, IMU and carried state). If that fails,
stop before launching the candidate; do not reinterpret the changed prefix as
an equivalent experiment. Global postprocessing is not expected to match the
full route and is not used for that equivalence gate.

Candidate comparison requires matching trajectory timestamps/counts and reports
odometry/final stages separately. Preserve diagnostic gaps, all logs, hashes
and clouds. Compare early preservation, onset roll/pitch updates, speed growth,
and local map shape; this non-return prefix cannot measure full-route closure.
No acceptance/default promotion follows from a successful run alone.

Runner: `tools/run_livox4_gain_comparison.py`; external preparation.json records
input/build/settings/helper hashes and approval remains a separate explicit step.
No automatic retry or full-route extension.

Input prepared and verified: 62,500 IMU rows and 20,272,168 LiDAR points,
ending at absolute recording timestamp 125.002469950 s. Cutter verified retained
points against originals; IMU bytes are an exact prefix. All five original
recording hashes remain unchanged. See external input-verification.json.

Runner independent review found three gaps, now corrected: tracked child PID
and termination on interruption; mandatory hash coverage; nonempty cloud and
snapshot/log window agreement. Approval and omitted-identity guards were tested
without launching dMapper. Isolated Release build and its actual CMake adapter
test executable passed: 2 cases, 26406 assertions. The test include-path fix was
also mirrored into main repo; no runtime change beyond the adapter correction.

Final bundle is prepared with approved=false and no runs directory. All frozen
hashes and mandatory coverage verified. A copied TOML hash typo in the evidence
inventory was corrected from the existing reviewed reference manifest; no TOML
content changed. Original source hashes and verified prefix remain intact.

Final independent review also required identical runtime DLLs for both runs;
the frozen control DLLs are now used by both. Child cleanup covers PID-record
write failures as well as interruptions during processing.

Launch only after explicit user approval:

```powershell
python -B 'C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ahrs-gain-prefix/tools/run_livox4_gain_comparison.py' 'C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ahrs-gain-prefix' --approved
```
