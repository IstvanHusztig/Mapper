#ifndef _LIDAR_ODOMETRY_H_
#define _LIDAR_ODOMETRY_H_

#include "lidar_odometry_utils.h"
#include <session.h>
#include <HDMapping/Version.hpp>
#include <laszip/laszip_api.h>
#include <export_laz.h>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <sstream>
#include "toml_io.h"
#include "input_ports.h"

using Trajectory = std::map<double, std::pair<Eigen::Matrix4d, double>>;

bool load_data(RecordingSource &recording, PreviewSink &preview, const OdometryConfig &params, std::vector<std::vector<Point3Di>> &pointsPerFile, Imu &imu_data, bool debugMsg, Logger &logger);
void calculate_trajectory(
    Trajectory &trajectory, Imu &imu_data, bool fusionConventionNwu, bool fusionConventionEnu, bool fusionConventionNed, double ahrs_gain, bool debugMsg, bool use_removie_imu_bias_from_first_stationary_scan, Logger &logger);
bool compute_step_1(
    std::vector<std::vector<Point3Di>> &pointsPerFile, const OdometryConfig &params, OdometryState &state,
    Trajectory &trajectory, std::vector<WorkerData> &worker_data, PointStore &point_store, Logger &logger, ProgressReporter &progress, const Imu *imu_data = nullptr);
void run_consistency(std::vector<WorkerData>& worker_data, const OdometryConfig &params, PointStore &point_store, Logger &logger);
void filter_reference_buckets(const OdometryConfig &params, OdometryState& state);
void load_reference_point_clouds(std::vector<std::string> input_file_names, const OdometryConfig &params, OdometryState& state, Logger &logger);
void save_result(std::vector<WorkerData> &worker_data, const OdometryConfig &params, const OdometryState &state, const GuiPreferences &gui, fs::path outwd, double elapsed_seconds, PointStore &point_store, Logger &logger);
void save_parameters_toml(const OdometryConfig &params, const RunPaths &paths, const GuiPreferences &gui, const fs::path &outwd, double elapsed_seconds, Logger &logger);
void save_processing_results_json(const OdometryConfig &params, const OdometryState &state, const fs::path &outwd, double elapsed_seconds,
                                  Logger &logger, const std::vector<WorkerData> *worker_data = nullptr);
void save_trajectory_to_ascii(std::vector<WorkerData>& worker_data, std::string output_file_name);
#endif