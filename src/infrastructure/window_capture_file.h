#pragma once

// The window capture on disk (window_capture.h): window_capture_<from>-<to>s.bin in a CLI result folder, read back by
// dMapper --replay <file> <params.toml>. Binary, in the machine's byte order, so the doubles come back bit for bit. It
// holds the hash of the odometry settings it was made with (odometry_settings_hash), so a replay knows whether it runs
// with the same settings.

#include "odometry_config.h"
#include "window_capture.h"

#include <cstdint>
#include <filesystem>
#include <string>

// false (and `why` says why) if the file can't be written. format_version: 0 = the current format; an older one only for
// tests that check older files still read
bool write_window_capture(const std::filesystem::path &file, const WindowCapture &capture, const OdometryConfig &config, std::string &why,
                          std::uint32_t format_version = 0);

// false (and `why` says why) if the file can't be read or isn't a window capture of this format; settings_hash: the
// odometry settings hash of the run that made it
bool read_window_capture(const std::filesystem::path &file, WindowCapture &capture, std::uint64_t &settings_hash, std::string &why);

// writes the capture of a CLI run into its output folder
class FolderWindowCaptureSink : public WindowCaptureSink
{
public:
    FolderWindowCaptureSink(std::filesystem::path folder, const OdometryConfig &config) : folder_(std::move(folder)), config_(config) {}
    bool write(const WindowCapture &capture, std::string &why) override;
    // the file a capture with these settings is written to
    std::filesystem::path file() const;

private:
    std::filesystem::path folder_;
    const OdometryConfig &config_;
};
