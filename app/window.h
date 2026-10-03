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

// one optimization window: consecutive trajectory nodes (one per IMU sample) and what the odometry estimates for them
struct Window
{
    std::vector<Eigen::Affine3d> poses;                // optimized node poses (world)
    std::vector<Eigen::Affine3d> motion_model_poses;   // the motion-model prior of each node
    std::vector<std::pair<double, double>> timestamps; // (timestamp [s], unix timestamp [s]) per node
    std::vector<Eigen::Vector3d> imu_angles;           // IMU orientation (om, fi, ka) [rad] per node
    std::vector<Eigen::Vector3d> imu_acc;              // raw accelerometer of each node [g], body frame (empty if unknown)
    GyroBiasState gyro_bias;                           // estimated gyro bias of this window
};

// AHRS orientation track: IMU timestamp -> (pose matrix, unix timestamp)
using ImuTrajectory = std::map<double, std::pair<Eigen::Matrix4d, double>>;
