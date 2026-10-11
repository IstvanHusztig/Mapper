#pragma once

#include "lidar_odometry.h"
#include "odometry_snapshot.h"
#include "output_ports.h"
#include "run_reporting.h"

#include <string>
#include <vector>

// One processing run, shared by the CLI and the GUI:
//   load (load_data -> calculate_trajectory -> compute_step_1)
//   -> run_odometry (optimize_windows)
//   -> post_process (loop closure if use_loop_closure, then consistency if use_consistency)
//   -> save (save_result)
// Each stage is reported to the ProgressReporter (with a pause checkpoint at its start); messages go to
// the Logger. The pipeline owns the run's inputs (recording, point store, preview). The config, logger and
// reporter must outlive the pipeline. The services (the AHRS, the parallel executor) come from make_run_services. The run's data members are public because
// the GUI views them (trajectory, point cloud preview).
class OdometryPipeline
{
public:
    // initial_paths: the [paths] section loaded with the settings; the run fills in the working folders
    // inputs: from make_folder_inputs; services: from make_run_services; empty inputs or services make load() fail
    OdometryPipeline(const OdometryConfig &config, Logger &logger, ProgressReporter &progress, RunInputs inputs = {},
                     RunServices services = {}, RunPaths initial_paths = {}, bool debug = false);

    // reads the recording, estimates the IMU orientation and cuts the windows; false if loading failed
    bool load();
    // LiDAR odometry over all windows; false on failure
    bool run_odometry();
    // what the later stages read from the odometry (the windows and three values of the run state)
    OdometrySnapshot snapshot() const;
    // in place of run_odometry, after load: takes the windows and run state from a snapshot of an earlier run with the same
    // recording; false (and `why` names the difference) if its window count or window timestamps differ from the loaded ones
    bool restore_odometry(const OdometrySnapshot &snapshot, std::string &why);
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
    ImuTrajectory imu_trajectory; // AHRS orientation per IMU sample
    std::vector<Window> windows;
    OdometryState state;

private:
    const OdometryConfig *config_;
    Logger *logger_;
    ProgressReporter *progress_;
    bool debug_;
    RunInputs inputs_;
    RunServices services_;
};
