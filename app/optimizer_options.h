#pragma once

// Named options of optimize_lidar_odometry, built from OdometryConfig (replace its former positional parameters).

#include "lidar_odometry_utils.h"

// scan-to-map settings of the LiDAR odometry, fixed for a run
struct ScanToMapOptions
{
    NDT::GridParameters grid_indoor;
    NDT::GridParameters grid_outdoor;
    bool multithread = false;
    double min_distance = 0.0;              // points closer than this are not used [m]
    double min_distance_outdoor_grid = 0.0; // the same for the outdoor (coarse) grid [m]
    double max_distance = 0.0;              // points farther than this are not used [m]
    // ablation study switches
    bool use_planarity = false;
    bool use_norm = false;
    bool use_hierarchical_rgd = false;
    bool use_view_point_and_normal_vectors = false;
};

// motion model of a window: correction of the IMU frame and the sigmas of the consecutive-node and fix-origin priors
struct MotionModelOptions
{
    TaitBryanPose correction;
    double x_sigma_m = 0.0;
    double y_sigma_m = 0.0;
    double z_sigma_m = 0.0;
    double om_sigma_deg = 0.0;
    double fi_sigma_deg = 0.0;
    double ka_sigma_deg = 0.0;
    double fix_origin_x_sigma_m = 0.0;
    double fix_origin_y_sigma_m = 0.0;
    double fix_origin_z_sigma_m = 0.0;
    double fix_origin_om_sigma_deg = 0.0;
    double fix_origin_fi_sigma_deg = 0.0;
    double fix_origin_ka_sigma_deg = 0.0;
    double acc_sigma_mps2 = 0.0; // velocity-continuity prior (0 = off)
};

// the previous window's final velocity (world frame) as a prior for this one
struct VelocityPrior
{
    bool use = false;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // [m/s]
};

// rotation part of the motion model: time-scaled sigma and online gyro bias (see OdometryConfig)
struct MotionModelRotationOptions
{
    bool time_scaled_sigma = false;
    Eigen::Vector3d arw_rad_sqrt_s = Eigen::Vector3d::Zero(); // om/fi/ka axes
    double inflation_k = 1.0;
    double sigma_floor_rad = 0.0;

    bool estimate_bias = false;
    Eigen::Vector3d bias_prior_mean = Eigen::Vector3d::Zero();  // [rad/s]
    Eigen::Matrix3d bias_prior_cov = Eigen::Matrix3d::Identity(); // [(rad/s)^2]
    double bias_max_rad_s = 0.0;                                  // per-axis clamp, 0 = none
};

MotionModelRotationOptions get_motion_model_rotation_options(const OdometryConfig &params);

// translation part of the motion model: accelerometer as a soft measurement of the change of velocity (see OdometryConfig)
struct MotionModelTranslationImuOptions
{
    bool enabled = false;
    const std::vector<Eigen::Vector3d> *acc = nullptr;           // raw accelerometer of each node, body frame [accelerometer units]
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();           // gravity in the odometry frame [accelerometer units]
    double to_mps2 = 9.80665;                                    // [m/s^2] per accelerometer unit
    double sigma_mps2 = 0.5;
    double span_s = 0.01;
    double prev_velocity_age_s = 0.0;                            // time from the centre of prev_velocity to the first node
    Eigen::Vector3d offset_prior_mean = Eigen::Vector3d::Zero(); // [m/s^2]
    Eigen::Matrix3d offset_prior_cov = Eigen::Matrix3d::Identity();
};

ScanToMapOptions get_scan_to_map_options(const OdometryConfig &config);
// the position sigmas are the configured ones; optimize_windows may raise them for a degenerate window
MotionModelOptions get_motion_model_options(const OdometryConfig &config);
