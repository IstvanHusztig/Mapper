#pragma once

#include <Eigen/Dense>

#include <string>

#include "imu_sample.h"
#include "odometry_config.h"

// Start-up gyro bias (use_remove_imu_bias_from_first_stationary_scan): the per-axis median of the gyro over the first
// window_s of the recording, removed from every sample before the orientation is estimated. It is accepted only when
// that stretch shows no real motion; a hand-held stand-still is noisy but zero-mean, so it is judged by motion, not by
// the scatter alone:
//  - turn:    the medians of the stretch's four quarters agree per axis within max_spread_deg_s;
//  - tilt:    the gravity direction (mean accelerometer of the first and of the last quarter) turns by at most max_tilt_deg;
//  - shaking: each axis' scatter is at most max_noise_ratio times the sensor's own noise, ARW x sqrt(IMU rate).
// A steady turn at the very start can't be told from a bias by the IMU alone; the odometry's bias estimate corrects it.
struct StartupBiasOptions
{
    double window_s = 1.0;
    double max_spread_deg_s = 3.0;
    double max_tilt_deg = 2.0;
    double max_noise_ratio = 3.0;
    Eigen::Vector3d arw_deg_sqrt_s = Eigen::Vector3d(0.028, 0.042, 0.057);
};

struct StartupBias
{
    bool accepted = false;
    Eigen::Vector3d bias_deg_s = Eigen::Vector3d::Zero(); // 0 when not accepted
    std::string reason;                                   // what was measured, and the failed test if rejected
};

StartupBiasOptions get_startup_bias_options(const OdometryConfig &params);
StartupBias startup_gyro_bias(const Imu &imu, const StartupBiasOptions &options);
