# Prepared short Livox4 gain-only comparison

User requested preparation and notification before any processing run, then
explicitly approved both short runs and monitoring. The gated runner was
launched after that approval. This is separate from the disabled AHRS-gravity
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

Before launch the bundle was prepared with approved=false and no runs directory. All frozen
hashes and mandatory coverage verified. A copied TOML hash typo in the evidence
inventory was corrected from the existing reviewed reference manifest; no TOML
content changed. Original source hashes and verified prefix remain intact.

Final independent review also required identical runtime DLLs for both runs;
the frozen control DLLs are now used by both. Child cleanup covers PID-record
write failures as well as interruptions during processing.

Authorized command (user approval granted after preparation):

```powershell
python -B 'C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ahrs-gain-prefix/tools/run_livox4_gain_comparison.py' 'C:/Users/User/Downloads/dLidar/experiments/phase0b-2026-11-12/livox4-ahrs-gain-prefix' --approved
```

## Execution and collection recovery

Control completed successfully (native exit0, 566.58 s wall time, 346 windows).
Its first325 snapshot window payloads match the original full Livox4 baseline
exactly. The control cloud is present (111,481,109 bytes).

Collection initially stopped because `run_health.analyse()` returns a NumPy
speed array, which the runner's JSON writer did not serialize. This was a
reporting failure after successful mapping, not a solver failure. The original
runner, manifest and failure record are preserved in external
`collection-recovery/`. The repair serializes NumPy arrays/scalars while still
rejecting NaN/Inf; tested against the actual complete control health record.

The restricted `--resume-control-collection` option accepts only this recorded
serialization error, one exit-zero control, and no existing candidate folder.
It rechecks every frozen artifact and the exact325-window gate, collects the
existing control without rerunning dMapper, then launches only the already
approved gain-fix candidate. Only the runner hash changed in the preparation
manifest; mapping binaries, DLLs, settings and inputs stayed unchanged.
The gain-fix candidate completed successfully. No third run is authorized.

## Collected results

Both native runs exited0, with346 windows and62,280 poses per trajectory stage.
Wall times were566.58 s control and557.92 s candidate. Frozen identities and
output/log hashes verified; external `comparison.json` is `collected` with
`accuracy_accepted=false`. One independent evidence review confirmed the control
equivalence, unchanged experiment identities, measurements and recovery history.

| Diagnostic | Control | Gain-only fix |
|---|---:|---:|
| Maximum carried end speed | 5.086 m/s (W320) | 1.260 m/s (W319) |
| Windows above2.5 m/s | 8 | 0 |
| Maximum absolute yaw correction | 3.480 degrees (W301) | 16.959 degrees (W298) |
| Windows above3 /10 degrees | 4 /0 | 4 /1 |
| Largest loop-closure segment correction | 1.63 m | 4.83 m |
| Bend/speed/bias coverage | 346 /346 each | 346 /346 each |
| Missing eligible uncertainty diagnostics | W95 | none |

The candidate introduces yaw corrections of-16.959,-9.782,-4.269 degrees at
W298–300; W298 begins106.92 s into the recording. No earlier candidate window
exceeds the3-degree or2.5 m/s diagnostic triggers. In the predefined onset
interval106.9–117 s, the speed maximum changes5.086->1.260 m/s while the maximum
absolute yaw correction changes3.480->16.959 degrees. Thus the speed spike is
absent in this candidate, but this experiment does not establish a flyaway fix.
Attributing the remaining rotation jump requires further causal analysis at high
effort, using these existing results first; no new run is needed for that step.

Exact timestamp/count matching passed before comparison. Maximum position
disagreement is13.496 m in odometry and9.797 m after postprocessing. Odometry
first exceeds0.05/0.20/0.50 m disagreement at31.184/32.066/104.202 s, respectively:
the candidate changes the earlier trajectory too. These are differences from a
known-faulty control, not accuracy improvements or errors against truth.
Summed sampled path lengths are83.407/78.155 m for control/candidate odometry
and209.782/205.227 m for final output. Do not treat these as physical walking
distance; the much larger final sums require separate inspection of postprocessing
and chunk boundaries. This truncated prefix has no confirmed physical return.

### Descriptive map check, not an adoption gate

Existing `wall_precision.py` ran on both final clouds with identical defaults
and fixed z=+0.50 +/-0.05 m. Logs and CSV/PNG artifacts are under each run.

| Extracted-slice measure | Control | Gain-only fix |
|---|---:|---:|
| Slice points | 386,085 | 399,113 |
| Extracted pieces / line length | 332 /140 m | 292 /127 m |
| Robust wall spread p50 /p90 | 29 /95 mm | 35 /88 mm |
| Same-side visit difference p50 /p90 | 100 /193 mm | 82 /174 mm |
| Pieces with visit pairs | 163 | 137 |

Physical surfaces are not matched: the extraction retains different pieces,
range and incidence distributions, and its0.2 m band can omit displaced walls.
Therefore these values cannot rank map quality or prove an improvement. Matched
geometry/revisits, perturbation spread, full-route accuracy and indoor regression
are NOT MEASURED by this pair. No acceptance, default change or roadmap gate
completion follows. Charlie remains at2/19 tasks completed.

Clouds for the user's visual check, relative to the external bundle:

- `runs/control/result/scan_lio_merged.laz`
- `runs/gain-fix/result/scan_lio_merged.laz`

Follow-up high-effort existing-data analysis is in `gain-jump-107s-analysis.md`.
It locates the rotation within the joint optimizer before tail handoff, rules
out a direct AHRS heading jump, and records the remaining initiating-factor
evidence gap. No further mapping run accompanied that investigation.
