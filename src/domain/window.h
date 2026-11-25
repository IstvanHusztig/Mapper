#pragma once

// Domain types of a run (Eigen and the standard library only).

#include <Eigen/Dense>

#include <map>
#include <optional>
#include <utility>
#include <vector>

// per-window gyro bias estimate [rad/s] and its covariance [(rad/s)^2], body frame
struct GyroBiasState
{
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

// world-frame velocity at the window's first and last node, estimated with imu_always_on; the end and its marginal covariance
// are the prior of the next window's start
struct VelocityState
{
    Eigen::Vector3d start = Eigen::Vector3d::Zero();              // [m/s]
    Eigen::Vector3d end = Eigen::Vector3d::Zero();                // [m/s]
    Eigen::Matrix3d end_covariance = Eigen::Matrix3d::Identity(); // [(m/s)^2]
};

// what the odometry read about a window when it solved it, before its tail was re-propagated (window_features.csv has the
// same quantities); the loop closure can weight its odometry steps from them (loop_closure_odometry_from_readings)
struct WindowReadings
{
    // smallest / largest eigenvalue of the window's LiDAR translation information at its last linearization (0 without any)
    double information_ratio = 0.0;
    Eigen::Vector3d weak_axis = Eigen::Vector3d::UnitX();  // world frame: that information's eigenvector of the smallest eigenvalue (sign arbitrary)
    bool at_rest = false;                                  // the window's rest test (imu_always_on; false without it)
    Eigen::Vector3d placed_move = Eigen::Vector3d::Zero(); // last minus first node as solved, world frame [m]
    double duration_s = 0.0;                               // first to last node [s]
};

// one optimization window: consecutive trajectory nodes (one per IMU sample) and what the odometry estimates for them
struct Window
{
    std::vector<Eigen::Affine3d> poses;                // optimized node poses (world)
    std::vector<Eigen::Affine3d> motion_model_poses;   // the motion-model prior of each node
    std::vector<std::pair<double, double>> timestamps; // (timestamp [s], unix timestamp [s]) per node
    std::vector<Eigen::Vector3d> imu_angles;           // IMU orientation (om, fi, ka) [rad] per node
    std::vector<Eigen::Vector3d> imu_acc;              // raw accelerometer of each node [g], body frame (empty if unknown)
    GyroBiasState gyro_bias;                           // estimated gyro bias of this window
    VelocityState velocity;                            // start and end velocity (imu_always_on)
    // the IMU's motion from the previous window's last node to this window's first node (one sample interval, or more
    // after a dropped window); identity for the first window
    Eigen::Affine3d imu_motion_from_previous = Eigen::Affine3d::Identity();
    // absent: the window was not solved, or it was read from a file written before the readings existed
    std::optional<WindowReadings> readings;
};

// the readings of a solved window from its LiDAR translation information and its rest test (WindowReadings)
inline WindowReadings window_readings(const Window &window, const Eigen::Matrix3d &lidar_translation_information, bool at_rest)
{
    WindowReadings r;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
    const double largest = eig.eigenvalues().maxCoeff();
    r.information_ratio = largest > 0.0 ? eig.eigenvalues().minCoeff() / largest : 0.0;
    r.weak_axis = eig.eigenvectors().col(0);
    r.at_rest = at_rest;
    if (!window.poses.empty())
        r.placed_move = window.poses.back().translation() - window.poses.front().translation();
    if (!window.timestamps.empty())
        r.duration_s = window.timestamps.back().first - window.timestamps.front().first;
    return r;
}

// AHRS orientation track: IMU timestamp -> (pose matrix, unix timestamp)
using ImuTrajectory = std::map<double, std::pair<Eigen::Matrix4d, double>>;
