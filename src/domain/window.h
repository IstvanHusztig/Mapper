#pragma once

// Domain types of a run (Eigen and the standard library only).

#include <Eigen/Dense>

#include <map>
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
    // covariance of the end velocity (rows) with the window's acceleration offset (columns) [m/s * m/s^2]; with
    // end_covariance and the offset's covariance the 6x6 joint block of (end velocity, offset)
    Eigen::Matrix3d end_offset_covariance = Eigen::Matrix3d::Zero();
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
};

// AHRS orientation track: IMU timestamp -> (pose matrix, unix timestamp)
using ImuTrajectory = std::map<double, std::pair<Eigen::Matrix4d, double>>;
