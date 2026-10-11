# Proposal

## Why

`44_s1`'s gallery is drawn up to 1.3 m off, because its odometry **stalls** at 68.5–72.5 s on the way out along a corridor. For 4 s it estimates 0.4 m/s instead of about 1 m/s and is left 1.69 m behind. Loop closure sees only that lag plus the gallery drift and can't tell them apart (floor-alignment-diagnosis findings 2.2).

The stall is the frontier drag of planar NDT cells: a cell pulls a point towards its mean within the plane too, so walls hold the window back along a passage. Pure point-to-plane removes that pull and shrank the stall from 1.69 to 0.48 m, but it ran away on the stairs (rejected change `2026-10-11-point-to-plane-ndt`):
- at window 212 (85.9 s) the window jumped by about one step (+32 cm up, then forward along the run, with the IMU strongly against it: rms 1.1–1.7 m/s²);
- without in-plane anchors, treads constrain only height and risers only the position along the run, so a shift by one step (rise and run) matches as well as the truth. The stairs' periodicity makes it an alias.

The information ratio can't separate the two cases: the stall corridor has 0.21–0.37, s1's stairs 0.25–0.36. So switching point-to-plane on per window can't work. What differs is the **direction**:
- in the corridor the false pull acts along the window's weak direction (the axis), within the walls and the floor;
- on the stairs the anchors that break the alias are the risers' in-plane (vertical) pull, which is across the weak direction.

The release has to remove the in-plane pull only along the weak direction.

## What Changes

- **Directional release:** for a window whose LiDAR translation information is weak (the previous window's ratio below `ndt_in_plane_release_ratio`, fully below `ndt_in_plane_release_ratio_full`), each planar cell's information loses its stiffness along the component of the weak direction that lies in the cell's plane. The stiffness along the normal and the in-plane stiffness across the weak direction stay.
  - In a corridor, walls and floor no longer hold the window back along the axis.
  - On stairs, risers (normal close to the run) keep their vertical in-plane anchors.
- **Settings in `[lidar_odometry_control]`:** `ndt_in_plane_release_ratio` and `ndt_in_plane_release_ratio_full`, 0 = off (the default; results bit-identical).
- **Go/no-go measurement first** (full odometry on `44_s0/s1/s2` and `45f_p`, plus the `/fp:fast` build on 44). On fail the code is removed again (no dead code) and the change archived as rejected.
- **On pass:** switch on in `settings/lidar-vertical.toml`, new 44 baselines, docs (EN/HU, `docs/degeneracy-information.html`).

## Capabilities

### New Capabilities

(none)

### Modified Capabilities

- `lidar-odometry-motion-model`: a requirement for the directional release of the in-plane NDT pull in weak windows (off by default, bit-identical when off).

## Impact

- `src/domain/scan_to_map.cpp`: the two NDT point lambdas use the released information.
- `src/domain/optimizer_options.h/.cpp`: the release direction and strength per window.
- `src/application/run_odometry.cpp`: the previous window's weak direction and ratio, already carried as `carry.weak_direction`, are handed on. Today the ratio is computed only with the accelerometer gate; it becomes needed whenever the release is on.
- `src/domain/odometry_config.h`, `src/infrastructure/toml_io.h`: two settings; the golden TOML files are regenerated in their own commit.
- `tests/scan_to_map_test.cpp`: corridor and room cases with the release on; the parallel-vs-sequential and cached-vs-on-the-spot bit-for-bit cases.
- **Runs:** each go/no-go run is a full odometry, 25–30 min with the diagnostics; about 2.5 h for the set, two at a time.
- **Interacts with the degeneracy projection** (`degenerate_projection_ratio` 0.1 / 0.2 shipped). It removes all NDT translation information along a weak direction below 0.2; the release only removes the false in-plane part, between 0.2 and the release ratio.
