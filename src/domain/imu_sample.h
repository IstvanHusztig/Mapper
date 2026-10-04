#pragma once

// IMU samples as read from a recording (domain value type).

#include <Eigen/Dense>

#include <utility>
#include <vector>

// One IMU sample. The vectors are float, as the recording's values have always been narrowed on reading, so every
// later expression sees the same numbers.
struct ImuSample
{
    std::pair<double, double> timestamps; // the IMU timestamp and the unix timestamp [s]
    Eigen::Vector3f gyro;                 // angular velocity [rad/s], body frame
    Eigen::Vector3f acc;                  // acceleration [g], body frame
};

using Imu = std::vector<ImuSample>;
