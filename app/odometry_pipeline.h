#pragma once

#include "lidar_odometry.h"
#include "output_ports.h"
#include "run_reporting.h"

#include <string>
#include <vector>

// One processing run, shared by the CLI and the GUI:
//   load (load_data -> calculate_trajectory -> compute_step_1)
//   -> run_odometry (compute_step_2)
//   -> post_process (loop closure if use_loop_closure, then consistency if use_consistency)
//   -> save (save_result)
// Each stage is reported to the ProgressReporter (with a pause checkpoint at its start); messages go to
// the Logger. The pipeline owns the run's inputs (recording, point store, preview). The config, logger and
// reporter must outlive the pipeline. The run's data members are public because
// the GUI views and edits them (trajectory gizmo, export).
class OdometryPipeline
{
public:
    // initial_paths: the [paths] section loaded with the settings; the run fills in the working folders
    // inputs: from make_folder_inputs; empty inputs make load() fail
    OdometryPipeline(const OdometryConfig &config, Logger &logger, ProgressReporter &progress, RunInputs inputs = {},
                     RunPaths initial_paths = {}, bool debug = false);

    // reads the recording, estimates the IMU orientation and cuts the windows; false if loading failed
    bool load();
    // LiDAR odometry over all windows; false on failure
    bool run_odometry();
    // loop closure and the consistency pass, each only when enabled in the config
    void post_process();
    // run_odometry + post_process; returns run_odometry's result (post-processing runs regardless, as in the CLI)
    bool run_all();
    // writes the result (session, trajectories, LAZ, parameters, results report) to the sink
    void save(ResultSink &sink, const GuiPreferences &gui, double elapsed_seconds);

    // removes the run's stored points (the GUI at exit)
    void clear_point_store();
    // a window's stored (decimated) points, e.g. for a preview; false without inputs or data
    bool load_window_points(std::size_t window, std::vector<Point3Di> &points) const;

    const OdometryConfig &config() const { return *config_; }

    std::vector<std::vector<Point3Di>> points_per_file;
    Imu imu;
    Trajectory trajectory;
    std::vector<Window> worker_data;
    OdometryState state;

private:
    const OdometryConfig *config_;
    Logger *logger_;
    ProgressReporter *progress_;
    bool debug_;
    RunInputs inputs_;
};
