#include <doctest/doctest.h>

#include "optimizer_options.h"

// Every mapped config field gets a distinct value, so a swapped or missing line in a builder fails.

TEST_CASE("get_scan_to_map_options copies each setting to its member")
{
    OdometryConfig config;
    config.in_out_params_indoor.resolution_X = 0.11;
    config.in_out_params_outdoor.resolution_X = 0.22;
    config.useMultithread = true;
    config.min_distance_lidar = 1.1;
    config.min_distance_lidar_outdoor_grid = 2.2;
    config.max_distance_lidar = 3.3;
    config.ablation_study_use_planarity = true;
    config.ablation_study_use_norm = false;
    config.ablation_study_use_hierarchical_rgd = true;
    config.ablation_study_use_view_point_and_normal_vectors = false;

    const ScanToMapOptions o = get_scan_to_map_options(config);
    CHECK(o.grid_indoor.resolution_X == 0.11);
    CHECK(o.grid_outdoor.resolution_X == 0.22);
    CHECK(o.multithread == true);
    CHECK(o.min_distance == 1.1);
    CHECK(o.min_distance_outdoor_grid == 2.2);
    CHECK(o.max_distance == 3.3);
    CHECK(o.use_planarity == true);
    CHECK(o.use_norm == false);
    CHECK(o.use_hierarchical_rgd == true);
    CHECK(o.use_view_point_and_normal_vectors == false);

    // the opposite flag values, so no flag can be hard-coded
    config.useMultithread = false;
    config.ablation_study_use_planarity = false;
    config.ablation_study_use_norm = true;
    config.ablation_study_use_hierarchical_rgd = false;
    config.ablation_study_use_view_point_and_normal_vectors = true;
    const ScanToMapOptions p = get_scan_to_map_options(config);
    CHECK(p.multithread == false);
    CHECK(p.use_planarity == false);
    CHECK(p.use_norm == true);
    CHECK(p.use_hierarchical_rgd == false);
    CHECK(p.use_view_point_and_normal_vectors == true);
}

TEST_CASE("get_motion_model_options copies each sigma to its member")
{
    OdometryConfig config;
    config.motion_model_correction.om = 0.1;
    config.motion_model_correction.fi = 0.2;
    config.motion_model_correction.ka = 0.3;
    config.lidar_odometry_motion_model_x_1_sigma_m = 1.0;
    config.lidar_odometry_motion_model_y_1_sigma_m = 2.0;
    config.lidar_odometry_motion_model_z_1_sigma_m = 3.0;
    config.lidar_odometry_motion_model_om_1_sigma_deg = 4.0;
    config.lidar_odometry_motion_model_fi_1_sigma_deg = 5.0;
    config.lidar_odometry_motion_model_ka_1_sigma_deg = 6.0;
    config.lidar_odometry_motion_model_fix_origin_x_1_sigma_m = 7.0;
    config.lidar_odometry_motion_model_fix_origin_y_1_sigma_m = 8.0;
    config.lidar_odometry_motion_model_fix_origin_z_1_sigma_m = 9.0;
    config.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg = 10.0;
    config.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg = 11.0;
    config.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg = 12.0;
    config.lidar_odometry_motion_model_acc_1_sigma_mps2 = 13.0;

    const MotionModelOptions o = get_motion_model_options(config);
    CHECK(o.correction.om == 0.1);
    CHECK(o.correction.fi == 0.2);
    CHECK(o.correction.ka == 0.3);
    CHECK(o.x_sigma_m == 1.0);
    CHECK(o.y_sigma_m == 2.0);
    CHECK(o.z_sigma_m == 3.0);
    CHECK(o.om_sigma_deg == 4.0);
    CHECK(o.fi_sigma_deg == 5.0);
    CHECK(o.ka_sigma_deg == 6.0);
    CHECK(o.fix_origin_x_sigma_m == 7.0);
    CHECK(o.fix_origin_y_sigma_m == 8.0);
    CHECK(o.fix_origin_z_sigma_m == 9.0);
    CHECK(o.fix_origin_om_sigma_deg == 10.0);
    CHECK(o.fix_origin_fi_sigma_deg == 11.0);
    CHECK(o.fix_origin_ka_sigma_deg == 12.0);
    CHECK(o.acc_sigma_mps2 == 13.0);
}

TEST_CASE("get_motion_model_rotation_options copies each setting and converts degrees to radians")
{
    const double deg = M_PI / 180.0;
    OdometryConfig config;
    config.use_time_scaled_rotation_sigma = true;
    config.gyro_arw_x_deg_sqrt_s = 1.0;
    config.gyro_arw_y_deg_sqrt_s = 2.0;
    config.gyro_arw_z_deg_sqrt_s = 3.0;
    config.rotation_sigma_inflation_k = 4.0;
    config.rotation_sigma_floor_deg = 5.0;
    config.use_gyro_bias_estimation = false;
    config.gyro_bias_max_deg_s = 6.0;

    const MotionModelRotationOptions o = get_motion_model_rotation_options(config);
    CHECK(o.time_scaled_sigma == true);
    CHECK(o.arw_rad_sqrt_s.x() == doctest::Approx(1.0 * deg));
    CHECK(o.arw_rad_sqrt_s.y() == doctest::Approx(2.0 * deg));
    CHECK(o.arw_rad_sqrt_s.z() == doctest::Approx(3.0 * deg));
    CHECK(o.inflation_k == 4.0);
    CHECK(o.sigma_floor_rad == doctest::Approx(5.0 * deg));
    CHECK(o.estimate_bias == false);
    CHECK(o.bias_max_rad_s == doctest::Approx(6.0 * deg));

    // the opposite flag values, so no flag can be hard-coded
    config.use_time_scaled_rotation_sigma = false;
    config.use_gyro_bias_estimation = true;
    const MotionModelRotationOptions p = get_motion_model_rotation_options(config);
    CHECK(p.time_scaled_sigma == false);
    CHECK(p.estimate_bias == true);
}
