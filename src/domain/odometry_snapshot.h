#pragma once

// What the stages after the odometry read from it: every window (poses, motion-model poses, timestamps, IMU angles and
// accelerations, gyro bias, velocity) and three values of the run state. A run can save it (save_odometry_snapshot), and
// a later run can restore it instead of running the odometry again (dMapper ... --from-snapshot).

#include <Eigen/Dense>
#include <vector>
#include "window.h"

struct OdometrySnapshot
{
    std::vector<Window> windows;
    Eigen::Affine3d m_g = Eigen::Affine3d::Identity();
    double total_length_of_calculated_trajectory = 0.0;
    double consecutive_distance = 0.0;
};
