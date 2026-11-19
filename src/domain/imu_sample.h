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

// Corrections applied to the IMU values as they are read (settings `gyro_scale_x/y/z`): each gyro axis is multiplied
// by its factor while the value is still the double parsed from the file, then narrowed to float once. That is the
// arithmetic of a recording copy whose gyro column was multiplied and written back with full precision, so a run
// with the factor gives that copy's result bit for bit. An axis whose factor is 1.0 is not touched.
struct ImuCalibration
{
    Eigen::Vector3d gyro_scale = Eigen::Vector3d::Ones();

    bool is_identity() const { return gyro_scale == Eigen::Vector3d::Ones(); }

    // the gyro value of one axis (0 x, 1 y, 2 z) as stored, from the value parsed from the file
    float gyro(int axis, double parsed) const
    {
        const double scale = gyro_scale[axis];
        return static_cast<float>(scale == 1.0 ? parsed : parsed * scale);
    }
};
