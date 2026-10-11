#pragma once

// Scan-to-map optimization of one window against the NDT reference map.

#include <Eigen/Dense>
#include <vector>
#include <structures.h>
#include "ndt_map.h"
#include "normal_equations.h"
#include "optimizer_options.h"
#include "phase_timing.h"
#include "run_reporting.h"
#include "window.h"

// What the optimizer keeps from one call to the next: the symbolic analysis of its two Cholesky factorizations (the
// covariance of the extra unknowns, and the damped solve), repeated only when the sparsity pattern changes. One per run.
struct ScanToMapWorkspace
{
    PatternCachedCholesky covariance;
    PatternCachedCholesky damped;
};

// The NDT cost of a window at the given node poses, as the point terms of optimize_lidar_odometry weigh it: the sum over the
// matched points of w r^T M r (r = the point placed with its node's pose minus its cell's mean, M the cell's information,
// w the planarity / norm weight when those are on), indoor and, with use_hierarchical_rgd, outdoor grid. Diagnostics only
// (log_window_diagnostics); sequential.
double ndt_window_cost(const std::vector<Point3Di> &intermediate_points, const std::vector<Eigen::Affine3d> &poses,
                       const NDTBucketMapType &buckets_indoor, const NDTBucketMapType &buckets_outdoor, const ScanToMapOptions &scan_to_map);

// this function performs main LiDAR odometry calculations
// gyro_bias / gyro_bias_cov are updated only when rotation_options.estimate_bias is set
// timing (log_stage_timing): when not null, the time of each phase is added to it
// workspace: when not null, the Cholesky analyses are kept in it between calls (the results are the same without it)
void optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points, Window &window,
                             NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                             const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                             const MotionModelRotationOptions &rotation_options,
                             const MotionModelTranslationImuOptions &translation_imu_options,
                             const VelocityPrior &velocity_prior, double lm_factor, double &delta,
                             Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger,
                             Eigen::Matrix3d *lidar_translation_information, ScanToMapTiming *timing = nullptr, ScanToMapWorkspace *workspace = nullptr);

