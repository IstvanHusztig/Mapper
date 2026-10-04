#pragma once

// Optimizers of use_robust_and_accurate_lidar_odometry.

#include <Eigen/Dense>
#include <vector>
#include <structures.h>
#include "ndt_map.h"

void optimize_sf2(std::vector<Point3Di> &intermediate_points, std::vector<Point3Di> &intermediate_points_sf, std::vector<Eigen::Affine3d> &trajectory,
                  const std::vector<Eigen::Affine3d> &motion_model_trajectory,
                  const NdtGridParameters &rgd_params, bool useMultithread, double wx, double wy, double wz, double wom, double wfi, double wka);

void optimize_rigid_sf(
    const std::vector<Point3Di> &intermediate_points,
    std::vector<Eigen::Affine3d> &trajectory,
    std::vector<Eigen::Affine3d> &motion_model_trajectory,
    const NDTBucketMapType &buckets,
    double distance_bucket,
    double polar_angle_deg,
    double azimutal_angle_deg,
    int robust_and_accurate_lidar_odometry_sf_rigid_iterations,
    double max_distance_lidar_rigid_sf,
    bool useMultithread,
    double rgd_sf_sigma_x_m,
    double rgd_sf_sigma_y_m,
    double rgd_sf_sigma_z_m,
    double rgd_sf_sigma_om_deg,
    double rgd_sf_sigma_fi_deg,
    double rgd_sf_sigma_ka_deg);
