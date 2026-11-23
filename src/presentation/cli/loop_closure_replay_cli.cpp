#include "cli.h"

#include "console_reporting.h"
#include "folder_result_sink.h"
#include "loop_closure_capture_file.h"
#include "loop_closure_replay.h"
#include "toml_config_repository.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace
{
    // the console, and a copy of every line in a file (the replay's loop log)
    class TeeLogger : public Logger
    {
    public:
        TeeLogger(Logger &console, const fs::path &file) : console_(console), file_(file) {}
        void write(LogLevel level, std::string_view message) override
        {
            console_.write(level, message);
            const std::lock_guard<std::mutex> lock(mutex_);
            file_ << message << "\n";
        }
        bool ok() const { return static_cast<bool>(file_); }

    private:
        Logger &console_;
        std::ofstream file_;
        std::mutex mutex_;
    };

    std::string joined(const std::vector<std::string> &names)
    {
        std::string s;
        for (const auto &n : names)
            s += (s.empty() ? "" : ", ") + n;
        return s;
    }
} // namespace

int run_loop_closure_replay(const char *capture_file, const char *params_file, const char *output_folder, const char *reject_tries)
{
    OdometryConfig config{};
    RunPaths paths;
    GuiPreferences gui;
    TomlConfigRepository config_repository;
    if (!fs::is_regular_file(params_file) || !config_repository.load(params_file, config, paths, gui))
    {
        std::cerr << "Cannot load parameters from: '" << params_file << "'" << std::endl;
        return 1;
    }
    ConsoleReporter reporter;
    std::vector<std::pair<int, int>> reject;
    std::string why;
    if (reject_tries && !parse_loop_try_names(reject_tries, reject, why))
    {
        log_error(reporter) << "--reject-tries: " << why;
        return 1;
    }
    LoopClosureCapture capture;
    std::map<std::string, std::string> captured_settings;
    if (!read_loop_closure_capture(capture_file, capture, captured_settings, why))
    {
        log_error(reporter) << "Cannot read the loop closure capture: " << why;
        return 1;
    }
    const LoopClosureSettingsDifference settings = loop_closure_replay_settings(captured_settings, config);
    if (!settings.refused.empty())
    {
        log_error(reporter) << "loop closure replay refused: these settings change the odometry, the segments or a registration, "
                               "which the capture can't replay: "
                            << joined(settings.refused);
        return 1;
    }
    const bool own_settings = settings.changed.empty();
    std::error_code ec;
    fs::create_directories(output_folder, ec);
    TeeLogger logger(reporter, fs::path(output_folder) / "loop_closure_log.txt");
    log_info(logger) << "loop closure replay of " << capture.windows.size() << " windows, " << capture.segments.size() << " segments, "
                     << capture.tries.size() << " captured registrations ("
                     << (own_settings ? "the capture's own loop closure settings" : "changed settings: " + joined(settings.changed)) << ")";
    if (!reject.empty())
    {
        std::vector<std::string> names;
        for (const auto &t : reject)
            names.push_back(loop_try_name(t));
        log_info(logger) << "loop closure replay: tries counted as rejected (--reject-tries): " << joined(names);
    }

    const auto start = std::chrono::steady_clock::now();
    const LoopClosureReplayResult r = replay_loop_closure(capture, config, logger, reject);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (!r.segments_match)
    {
        log_error(logger) << "loop closure replay: the capture's windows don't give the capture's segments with these settings";
        return 1;
    }
    if (!r.rejected_not_in_capture.empty())
    {
        log_error(logger) << "loop closure replay: --reject-tries names tries the capture doesn't hold, nothing run: " << joined(r.rejected_not_in_capture);
        return 1;
    }
    if (!reject.empty())
        log_info(logger) << "loop closure replay: rejected " << (r.rejected.empty() ? std::string("none") : joined(r.rejected))
                         << (r.rejected_not_asked.empty() ? std::string() : "; named but never asked for (no effect): " + joined(r.rejected_not_asked));
    if (!r.missing.empty())
    {
        log_error(logger) << "loop closure replay: the settings ask for " << r.missing.size()
                          << " registration(s) the capture doesn't hold, no trajectory written: " << joined(r.missing);
        return 1;
    }
    log_info(logger) << "loop closure replay: " << r.loops << " loop(s) in " << std::fixed << std::setprecision(2) << seconds
                     << " s; against the capturing run: identical nodes " << r.identical_nodes << " of " << r.nodes << ", largest difference "
                     << std::setprecision(6) << r.largest_position_difference_m << " m / " << r.largest_rotation_difference_deg << " deg";

    // the result's trajectory files, so that the tools reading a result folder read a replay
    FolderResultSink sink(output_folder, config_repository, reporter);
    sink.begin();
    const std::vector<TrajectoryChunk> chunks = trajectory_chunks(r.windows, config);
    std::vector<Eigen::Affine3d> first_poses;
    for (size_t k = 0; k < chunks.size(); k++)
    {
        if (!sink.chunk_trajectory(k, chunks[k].rows))
            return 1;
        first_poses.push_back(chunks[k].first_pose);
    }
    sink.chunk_poses(first_poses);
    sink.parameters(config, paths, gui);
    const fs::path json = fs::path(output_folder) / "loop_closure_edges.json";
    if (!write_loop_closure_replay_json(json, r, settings, config, why))
    {
        log_error(logger) << "loop closure replay: " << why;
        return 1;
    }
    if (!logger.ok())
        log_warning(reporter) << "loop closure replay: the loop log could not be written to " << output_folder;
    if (own_settings && reject.empty() && r.identical_nodes != r.nodes)
    {
        log_error(logger) << "loop closure replay: the capture's own settings did not reproduce the run";
        return 1;
    }
    return 0;
}
