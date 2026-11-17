#pragma once

// The GUI's state and actions (presentation layer), without ImGui, GLUT, OpenGL or file dialogs: the settings and
// where they came from, the current run (pipeline, recording folder, progress) and its worker thread. The view
// (lidar_odometry_gui.cpp) shows it, asks the user for paths and draws the run's data.

#include "console_reporting.h"
#include "gui_progress.h"
#include "odometry_config.h"
#include "odometry_pipeline.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

// what a run tells the view; called on the run's worker thread
struct RunNotifier
{
    virtual ~RunNotifier() = default;
    // the recording could not be loaded; the run ends here
    virtual void load_failed(const std::string &folder) = 0;
    // the run threw (e.g. a non-finite NDT distance): nothing is saved
    virtual void run_failed(const std::string &reason) = 0;
    // the result is saved next to the recording, in its next lio_result_<n>
    virtual void run_saved(const std::string &working_directory, double trajectory_length_m, double elapsed_seconds) = 0;
};

class GuiViewModel
{
public:
    explicit GuiViewModel(RunNotifier &notifier);
    ~GuiViewModel();
    GuiViewModel(const GuiViewModel &) = delete;
    GuiViewModel &operator=(const GuiViewModel &) = delete;

    // ---- settings ----
    // the GUI's defaults: a missing file leaves the built-in defaults
    void load_default_parameters(const std::filesystem::path &file);
    // Start -> Load parameters: a TOML file overlaid on the current settings; true if it was loaded
    bool load_parameters(const std::string &file);
    const OdometryConfig &config() const { return config_; }
    const GuiPreferences &preferences() const { return preferences_; }
    // where the current settings came from ("built-in defaults" or a file)
    const std::string &active_parameters() const { return active_parameters_; }
    // active_parameters() when the current run started
    const std::string &run_parameters() const { return run_parameters_; }

    // ---- a run ----
    // Start -> Load data and process: lists the folder's files, starts a fresh run on them (nothing from a previous
    // folder carries over; the [paths] loaded with the settings do) and loads, processes and saves it on a worker
    // thread. False if the folder doesn't exist (nothing starts).
    bool start_run(const std::string &folder);
    // waits until the worker thread has finished (tests)
    void wait_for_run();
    // a run is going (the progress window is shown)
    bool running() const { return running_.load(); }
    // seconds since the current run started
    double elapsed_seconds() const;
    GuiProgress &progress() { return progress_; }
    // the recording folder of the current run (empty before one)
    const std::string &working_directory() const { return working_directory_; }
    // the current run's data, for drawing
    const OdometryPipeline &run() const { return pipeline_; }
    // removes the run's stored points (at exit)
    void clear_point_store();
    // the console the run's messages go to
    Logger &console() { return console_; }

private:
    void process(const std::string &folder);

    RunNotifier *notifier_;
    // declared in this order: the progress and the pipeline report to the console, the pipeline reads the config
    ConsoleReporter console_;
    GuiProgress progress_{console_};
    OdometryConfig config_;
    GuiPreferences preferences_;
    OdometryPipeline pipeline_{config_, console_, progress_};
    std::string active_parameters_ = "built-in defaults";
    std::string run_parameters_;
    std::string working_directory_;
    std::atomic<bool> running_{false};
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    std::thread worker_;
};
