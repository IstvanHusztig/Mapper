#ifndef _LIDAR_ODOMETRY_H_
#define _LIDAR_ODOMETRY_H_

#include "odometry_config.h"
#include "odometry_state.h"
#include "window.h"
#include "input_ports.h"
#include "output_ports.h"
#include "service_ports.h"


bool load_data(RecordingSource &recording, PreviewSink &preview, const OdometryConfig &params, std::vector<std::vector<Point3Di>> &pointsPerFile, Imu &imu_data, bool debugMsg, Logger &logger);
void calculate_trajectory(
    ImuTrajectory &trajectory, Imu &imu_data, bool fusionConventionNwu, bool fusionConventionEnu, bool fusionConventionNed, double ahrs_gain, bool debugMsg, bool use_removie_imu_bias_from_first_stationary_scan, OrientationEstimator &ahrs, Logger &logger);
bool compute_step_1(
    std::vector<std::vector<Point3Di>> &pointsPerFile, const OdometryConfig &params, OdometryState &state,
    ImuTrajectory &trajectory, std::vector<Window> &windows, PointStore &point_store, Logger &logger, ProgressReporter &progress, const Imu *imu_data = nullptr);
void run_consistency(std::vector<Window>& windows, const OdometryConfig &params, PointStore &point_store, Logger &logger);
void save_result(std::vector<Window> &windows, const OdometryConfig &params, const OdometryState &state, const GuiPreferences &gui, double elapsed_seconds, ResultSink &sink, PointStore &point_store, Logger &logger);
void save_trajectory_to_ascii(std::vector<Window>& windows, std::string output_file_name);
#endif