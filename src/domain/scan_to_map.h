#pragma once

// Scan-to-map optimization of one window against the NDT reference map.

#include <Eigen/Dense>
#include <vector>
#include <structures.h>
#include "ndt_map.h"
#include "optimizer_options.h"
#include "phase_timing.h"
#include "run_reporting.h"
#include "window.h"

// this function performs main LiDAR odometry calculations
// gyro_bias / gyro_bias_cov are updated only when rotation_options.estimate_bias is set
// timing (log_stage_timing): when not null, the time of each phase is added to it
void optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points, Window &window,
                             NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                             const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                             const MotionModelRotationOptions &rotation_options,
                             const MotionModelTranslationImuOptions &translation_imu_options,
                             const VelocityPrior &velocity_prior, double lm_factor, double &delta,
                             Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger,
                             Eigen::Matrix3d *lidar_translation_information, ScanToMapTiming *timing = nullptr);

