#pragma once

// The odometry snapshot on disk: odometry_snapshot.bin in a result folder (save_odometry_snapshot), read back by
// dMapper <input> <params.toml> <output> --from-snapshot <file>. Binary, in the machine's byte order (little-endian on
// x86/x64), so the doubles come back bit for bit.
//
// The file also holds a hash of the settings the odometry depends on: every TOML setting except those of the
// post-processing and output-only categories (loop_closure, consistency, lidar_odometry_debug_info, paths, misc,
// version_info). A new setting therefore counts as an odometry setting unless its category is listed there.

#include "odometry_config.h"
#include "odometry_snapshot.h"
#include <cstdint>
#include <filesystem>
#include <string>

std::uint64_t odometry_settings_hash(const OdometryConfig &config);

// false (and `why` says why) if the file can't be written
bool write_odometry_snapshot(const std::filesystem::path &file, const OdometrySnapshot &snapshot, const OdometryConfig &config, std::string &why);

// false (and `why` says why) if the file can't be read, isn't a snapshot of this format, or was made with other odometry
// settings than `config`
bool read_odometry_snapshot(const std::filesystem::path &file, const OdometryConfig &config, OdometrySnapshot &snapshot, std::string &why);
