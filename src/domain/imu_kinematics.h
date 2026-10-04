#pragma once

// IMU kinematics of the window loop: rotation chains, world-frame acceleration, offsets and rest detection.

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>
#include <vector>

// rotation by the vector v (axis * angle)
Eigen::Matrix3d exp_so3(const Eigen::Vector3d &v);

// orientation of a node as a normalized quaternion
Eigen::Quaterniond rotation_of(const Eigen::Affine3d &m);

// relative rotation from node a to node b
Eigen::Quaterniond relative_rotation(const Eigen::Affine3d &a, const Eigen::Affine3d &b);

// re-chain the node orientations with each relative rotation corrected by exp(-bias * dt); node 0 and all translations unchanged
void remove_gyro_bias_from_rotations(std::vector<Eigen::Affine3d> &trajectory,
                                     const std::vector<std::pair<double, double>> &timestamps,
                                     const Eigen::Vector3d &bias);

// world-frame acceleration of each node [m/s^2], averaged over +-half_width_s
std::vector<Eigen::Vector3d> world_acceleration(const std::vector<Eigen::Affine3d> &trajectory,
                                                const std::vector<std::pair<double, double>> &timestamps,
                                                const std::vector<Eigen::Vector3d> &acc,
                                                const std::vector<Eigen::Vector3d> &ahrs_om_fi_ka,
                                                double gravity_norm, double half_width_s);

// acceleration offset e [m/s^2] seen in an optimized window; false if the window is too short
bool observed_acceleration_offset(const std::vector<Eigen::Affine3d> &trajectory,
                                  const std::vector<std::pair<double, double>> &timestamps,
                                  const std::vector<Eigen::Vector3d> &acc, const Eigen::Vector3d &gravity,
                                  double to_mps2, int tail_nodes, Eigen::Vector3d &offset);

// true if the device rests during the whole window: small world acceleration and small rotation rate
bool is_window_stationary(const std::vector<Eigen::Affine3d> &trajectory,
                          const std::vector<std::pair<double, double>> &timestamps,
                          const std::vector<Eigen::Vector3d> &acceleration,
                          double acc_threshold_mps2, double gyro_threshold_rad_s);
