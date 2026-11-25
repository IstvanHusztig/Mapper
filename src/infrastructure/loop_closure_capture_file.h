#pragma once

// The loop closure capture on disk (loop_closure_capture.h): loop_closure_capture.bin in a CLI result folder
// (capture_loop_closure), read back by dMapper --replay-loop-closure <file> <params.toml> <output_folder>. Binary, in the
// machine's byte order, so the doubles come back bit for bit. It holds every TOML setting of the run that made it (name
// and value as text), so a replay can name the settings it changes and refuse those it can't change.

#include "loop_closure_capture.h"
#include "loop_closure_replay.h"
#include "odometry_config.h"

#include <filesystem>
#include <map>
#include <string>
#include <vector>

// every TOML setting by name, its value as text (doubles to 17 digits, so two values are equal exactly when their texts are)
std::map<std::string, std::string> settings_as_text(const OdometryConfig &config);

// The settings a replay with `config` changes against the capture's (settings_as_text of the capturing run; a setting the
// capture doesn't list counts as at its default). refused: those a replay can't change, because they change the odometry
// (every setting outside [loop_closure], [consistency], [lidar_odometry_debug_info], [paths], [misc], [version_info]), the
// segments or a registration (loop_closure_segment_windows; loop_closure_max_correction_m, which is also the same-floor
// search range; loop_closure_cross_floor_search_m), and the odometry settings hash itself; changed: the other
// [loop_closure] settings that differ. Neither: the replay runs with the capture's own loop closure settings.
struct LoopClosureSettingsDifference
{
    std::vector<std::string> refused;
    std::vector<std::string> changed;
};
LoopClosureSettingsDifference loop_closure_replay_settings(const std::map<std::string, std::string> &captured, const OdometryConfig &config);

// false (and `why` says why) if the file can't be written
bool write_loop_closure_capture(const std::filesystem::path &file, const LoopClosureCapture &capture, const OdometryConfig &config, std::string &why);

// false (and `why` says why) if the file can't be read or isn't a loop closure capture of this format; settings: the
// capturing run's settings_as_text
bool read_loop_closure_capture(const std::filesystem::path &file, LoopClosureCapture &capture, std::map<std::string, std::string> &settings,
                               std::string &why);

// the replay's edges as JSON: the outcome, the edges kept and dropped (with their residuals), the missing tries, the
// comparison with the capturing run, the settings changed, and the kept edges read at the times of their information along
// each weak run's axis (edge_time_support, LoopClosureReport::edge_time_support; empty without the readings rule)
bool write_loop_closure_replay_json(const std::filesystem::path &file, const LoopClosureReplayResult &result,
                                    const LoopClosureSettingsDifference &settings, const OdometryConfig &config, std::string &why);

// the fine-time step's windows (loop_fine_time.csv, the columns of tools/loop_fine_time.py's windows.csv): per window of
// each scored eligible run its run, window, whether it is a run window, start and middle time from the first window's first
// node [s], offset [m], supported, training and held-out patches, the most training patches matched at one offset, the
// training and held-out RMS distance at 0 and at the profile's offset [m], and the offset its training curve prefers alone
bool write_loop_fine_time_csv(const std::filesystem::path &file, const LoopFineTimeReport &report, std::string &why);

// writes the capture of a CLI run into its output folder (loop_closure_capture.bin)
class FolderLoopClosureCaptureSink : public LoopClosureCaptureSink
{
public:
    FolderLoopClosureCaptureSink(std::filesystem::path folder, const OdometryConfig &config) : folder_(std::move(folder)), config_(config) {}
    bool write(const LoopClosureCapture &capture, std::string &why) override;
    std::filesystem::path file() const { return folder_ / "loop_closure_capture.bin"; }

private:
    std::filesystem::path folder_;
    const OdometryConfig &config_;
};
