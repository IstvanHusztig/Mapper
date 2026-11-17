#include "cli.h"

#include "console_reporting.h"
#include "folder_inputs.h"
#include "folder_result_sink.h"
#include "format_time.h"
#include "odometry_config.h"
#include "odometry_pipeline.h"
#include "odometry_snapshot_file.h"
#include "run_services.h"
#include "toml_config_repository.h"
#include "window_capture_file.h"
#include "window_term_csv.h"
#include "window_replay.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

int run_cli(int argc, char *argv[])
{
    // CLI settings: built-in defaults + the given file only (independent of the working directory)
    OdometryConfig config{};
    RunPaths paths;
    GuiPreferences gui;
    TomlConfigRepository config_repository;
    if (!fs::is_regular_file(argv[2]) || !config_repository.load(argv[2], config, paths, gui))
    {
        std::cerr << "Cannot load parameters from: '" << argv[2] << "'" << std::endl;
        return 1;
    }
    std::cout << "Parameters loaded OK from: " << argv[2] << std::endl;

    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    std::vector<std::string> input_file_names;
    for (const auto &entry : fs::directory_iterator(argv[1]))
    {
        if (!entry.is_directory())
        {
            std::cout << entry.path() << std::endl;
            input_file_names.push_back(entry.path().string());
        }
    }

    ConsoleReporter reporter;
    OdometryPipeline pipeline(config, reporter, reporter, make_folder_inputs(input_file_names, reporter), make_run_services(), paths);
    FolderWindowCaptureSink capture(argv[3], config);
    pipeline.set_window_capture(&capture);
    FolderWindowTermSink terms(argv[3]); // log_window_terms: window_terms.csv and window_features.csv in the output folder
    if (config.log_window_terms)
        pipeline.set_window_terms(&terms);
    if (pipeline.load())
        log_info(reporter) << "step_1_done";

    if (argc == 6)
    {
        // the odometry from an earlier run's snapshot
        OdometrySnapshot snapshot;
        std::string why;
        if (!read_odometry_snapshot(argv[5], config, snapshot, why) || !pipeline.restore_odometry(snapshot, why))
        {
            log_error(reporter) << "Cannot use the odometry snapshot '" << argv[5] << "': " << why;
            return 1;
        }
    }
    else
    {
        pipeline.run_odometry();
        if (config.log_window_terms && !terms.ok())
            log_warning(reporter) << "log_window_terms: " << terms.terms_file().string() << " could not be written";
        if (config.save_odometry_snapshot)
        {
            const fs::path file = fs::path(argv[3]) / "odometry_snapshot.bin";
            std::string why;
            if (write_odometry_snapshot(file, pipeline.snapshot(), config, why))
                log_info(reporter) << "odometry snapshot written to " << file.string();
            else
                log_error(reporter) << "odometry snapshot not written: " << why;
        }
    }
    pipeline.post_process();

    end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "calculations finished computation at "
              << std::ctime(&end_time)
              << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str() << "s\n";

    FolderResultSink sink(argv[3], config_repository, reporter);
    pipeline.save(sink, gui, elapsed_seconds.count());
    return 0;
}

int run_replay(const char *capture_file, const char *params_file)
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
    WindowCapture capture;
    std::uint64_t hash = 0;
    std::string why;
    if (!read_window_capture(capture_file, capture, hash, why))
    {
        log_error(reporter) << "Cannot read the window capture: " << why;
        return 1;
    }
    const bool same_settings = hash == odometry_settings_hash(config);
    const std::size_t n = capture.last - capture.first + 1;
    log_info(reporter) << "replay of windows " << capture.first + 1 << " to " << capture.last + 1 << " of " << capture.n_windows << " ("
                       << (same_settings ? "the capture's own odometry settings: iterations as captured" : "OTHER odometry settings than the capture's")
                       << ")";
    const auto start = std::chrono::steady_clock::now();
    // log_window_terms: <params name>_window_terms.csv and _window_features.csv next to the capture
    FolderWindowTermSink terms(fs::path(capture_file).parent_path(), fs::path(params_file).stem().string() + "_");
    const WindowReplayResult r = replay_window_capture(capture, config, same_settings, reporter, config.log_window_terms ? &terms : nullptr);
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    if (config.log_window_terms && !terms.ok())
        log_warning(reporter) << "log_window_terms: " << terms.terms_file().string() << " could not be written";

    std::size_t iterations_differ = 0;
    for (int d : r.iteration_differences)
        iterations_differ += d != 0;
    log_info(reporter) << "replay: " << r.outcomes.size() << " of " << n << " windows in " << std::fixed << std::setprecision(1) << seconds
                       << " s" << (r.cancelled ? " (cancelled)" : "") << "; identical with the capture: " << r.identical << " of " << n
                       << "; largest difference " << std::setprecision(6) << r.largest_position_difference_m << " m / "
                       << r.largest_rotation_difference_deg << " deg" << "; last iteration differs in " << iterations_differ << " windows";
    if (r.first_different_window >= 0)
        log_info(reporter) << "replay: first window that differs: " << r.first_different_window + 1;
    if (same_settings && (r.identical != n || r.cancelled))
    {
        log_error(reporter) << "replay: the capture's own settings did not reproduce the run";
        return 1;
    }
    return 0;
}
