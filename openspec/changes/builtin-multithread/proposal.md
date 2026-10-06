# Proposal

## Why

Since `deterministic-multithread`, multithreading gives the same bits as a single-threaded run, and it is on by default. There is no reason left to switch it off, and a setting that changes nothing is only a trap. The user asked on 2026-10-06 for it to become a built-in feature.

## What Changes

- **BREAKING (settings file):** the TOML key `useMultithread` is removed from `OdometryConfig`, the TOML table, `settings/default_settings.toml` and `settings/lidar-vertical.toml`. Older files that still contain it load as before, because loading reads only known keys, so the key is ignored.
- The window optimization always runs its node loop in parallel. `ScanToMapOptions::multithread` stays as an internal flag, `true` by default and not a setting, so the unit tests can still compare the parallel loop with the sequential one bit for bit. The robust odometry functions are called with `true`.
- The output params TOML loses the `useMultithread` line, so its hash in the manifests is updated. All other output files are unchanged.
- Docs: the setting's row in `docs/index.html` and the settings table in `docs/PERFORMANCE.md` go; `docs/ARCHITECTURE.md` is updated.

## Capabilities

### Modified Capabilities

- `lidar-odometry-motion-model`: "Reproducible results with or without multithreading" becomes "Reproducible results". Runs SHALL be byte-identical, and the parallel computation SHALL give the same output as a sequential one. There is no setting any more.

## Impact

- Code: `src/domain/odometry_config.h`, `src/domain/optimizer_options.{h,cpp}`, `src/infrastructure/toml_io.h`, `src/application/run_odometry.cpp`, `tests/optimizer_options_test.cpp`, `tests/scan_to_map_test.cpp`; the golden TOML files.
- Results: identical apart from the params TOML (precise and fast `44_short`).
