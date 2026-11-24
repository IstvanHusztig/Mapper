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
#include <vector>

namespace binary_io
{
class Reader;
class Writer;
} // namespace binary_io

std::uint64_t odometry_settings_hash(const OdometryConfig &config);

// The windows' readings (Window::readings) as one block after the windows, in the snapshot (format 2 on) and the loop
// closure capture (format 2 on): the window count, then per window a presence flag and, if present, the information ratio,
// the weak axis, the rest flag, the placed move and the duration. Reading fails if the count isn't the windows' count.
void write_window_readings(binary_io::Writer &w, const std::vector<Window> &windows);
bool read_window_readings(binary_io::Reader &r, std::vector<Window> &windows);

// false (and `why` says why) if the file can't be written
bool write_odometry_snapshot(const std::filesystem::path &file, const OdometrySnapshot &snapshot, const OdometryConfig &config, std::string &why);

// false (and `why` says why) if the file can't be read, isn't a snapshot of format 1 or 2, or was made with other odometry
// settings than `config`. A format 1 file (written before the readings) gives windows without readings.
bool read_odometry_snapshot(const std::filesystem::path &file, const OdometryConfig &config, OdometrySnapshot &snapshot, std::string &why);
