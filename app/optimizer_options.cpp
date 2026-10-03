#include "optimizer_options.h"

ScanToMapOptions get_scan_to_map_options(const OdometryConfig &config)
{
    ScanToMapOptions o;
    o.grid_indoor = config.in_out_params_indoor;
    o.grid_outdoor = config.in_out_params_outdoor;
    o.multithread = config.useMultithread;
    o.min_distance = config.min_distance_lidar;
    o.min_distance_outdoor_grid = config.min_distance_lidar_outdoor_grid;
    o.max_distance = config.max_distance_lidar;
    o.use_planarity = config.ablation_study_use_planarity;
    o.use_norm = config.ablation_study_use_norm;
    o.use_hierarchical_rgd = config.ablation_study_use_hierarchical_rgd;
    o.use_view_point_and_normal_vectors = config.ablation_study_use_view_point_and_normal_vectors;
    return o;
}

MotionModelOptions get_motion_model_options(const OdometryConfig &config)
{
    MotionModelOptions o;
    o.correction = config.motion_model_correction;
    o.x_sigma_m = config.lidar_odometry_motion_model_x_1_sigma_m;
    o.y_sigma_m = config.lidar_odometry_motion_model_y_1_sigma_m;
    o.z_sigma_m = config.lidar_odometry_motion_model_z_1_sigma_m;
    o.om_sigma_deg = config.lidar_odometry_motion_model_om_1_sigma_deg;
    o.fi_sigma_deg = config.lidar_odometry_motion_model_fi_1_sigma_deg;
    o.ka_sigma_deg = config.lidar_odometry_motion_model_ka_1_sigma_deg;
    o.fix_origin_x_sigma_m = config.lidar_odometry_motion_model_fix_origin_x_1_sigma_m;
    o.fix_origin_y_sigma_m = config.lidar_odometry_motion_model_fix_origin_y_1_sigma_m;
    o.fix_origin_z_sigma_m = config.lidar_odometry_motion_model_fix_origin_z_1_sigma_m;
    o.fix_origin_om_sigma_deg = config.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg;
    o.fix_origin_fi_sigma_deg = config.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg;
    o.fix_origin_ka_sigma_deg = config.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg;
    o.acc_sigma_mps2 = config.lidar_odometry_motion_model_acc_1_sigma_mps2;
    return o;
}
