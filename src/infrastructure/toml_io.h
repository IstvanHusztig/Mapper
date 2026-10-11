#ifndef _TOML_IO_H_
#define _TOML_IO_H_

#include "toml.hpp"
#include <map>
#include <variant>

#include <string>
#include "odometry_config.h"

class TomlIO
{
public:
    TomlIO() { ; };
    ~TomlIO() { ; };

    using ParamValue = std::variant<
        bool OdometryConfig::*,
        double OdometryConfig::*,
        int OdometryConfig::*,
        std::string OdometryConfig::*>;

    const std::map<std::string, ParamValue> POINTERS = {
        // version information
        {"software_version", &OdometryConfig::software_version},
        {"config_version", &OdometryConfig::config_version},
        {"build_date", &OdometryConfig::build_date},
        
        {"real_time_threshold_seconds", &OdometryConfig::real_time_threshold_seconds},
        {"filter_threshold_xy_inner", &OdometryConfig::filter_threshold_xy_inner},
        {"filter_threshold_xy_outer", &OdometryConfig::filter_threshold_xy_outer},
        {"decimation", &OdometryConfig::decimation},
        {"threshould_output_filter", &OdometryConfig::threshould_output_filter},
        {"min_counter_concatenated_trajectory_nodes", &OdometryConfig::min_counter_concatenated_trajectory_nodes},
        {"fusionConventionNwu", &OdometryConfig::fusionConventionNwu},
        {"fusionConventionEnu", &OdometryConfig::fusionConventionEnu},
        {"fusionConventionNed", &OdometryConfig::fusionConventionNed},
        {"ahrs_gain", &OdometryConfig::ahrs_gain},
        {"use_motion_from_previous_step", &OdometryConfig::use_motion_from_previous_step},
        {"nr_iter", &OdometryConfig::nr_iter},
        {"convergence_delta_threshold", &OdometryConfig::convergence_delta_threshold},
        {"sliding_window_trajectory_length_threshold", &OdometryConfig::sliding_window_trajectory_length_threshold},
        {"max_distance_lidar", &OdometryConfig::max_distance_lidar},
        {"threshold_initial_points", &OdometryConfig::threshold_initial_points},
        {"threshold_nr_poses", &OdometryConfig::threshold_nr_poses},
        {"min_distance_lidar", &OdometryConfig::min_distance_lidar},
        {"min_distance_lidar_outdoor_grid", &OdometryConfig::min_distance_lidar_outdoor_grid},
        {"lm_factor_initial", &OdometryConfig::lm_factor_initial},
        {"use_exact_ndt_bucket_statistics", &OdometryConfig::use_exact_ndt_bucket_statistics},
        {"ndt_plane_in_plane_sigma_m", &OdometryConfig::ndt_plane_in_plane_sigma_m},
        {"consistency_min_distance", &OdometryConfig::consistency_min_distance},
        {"use_remove_imu_bias_from_first_stationary_scan", &OdometryConfig::use_remove_imu_bias_from_first_stationary_scan},
        {"lidar_imu_lever_arm_x_m", &OdometryConfig::lidar_imu_lever_arm_x_m},
        {"lidar_imu_lever_arm_y_m", &OdometryConfig::lidar_imu_lever_arm_y_m},
        {"lidar_imu_lever_arm_z_m", &OdometryConfig::lidar_imu_lever_arm_z_m},
        {"save_calibration_validation", &OdometryConfig::save_calibration_validation},
        {"log_iteration_convergence", &OdometryConfig::log_iteration_convergence},
        {"log_stage_timing", &OdometryConfig::log_stage_timing},
        {"log_window_diagnostics", &OdometryConfig::log_window_diagnostics},
        {"save_odometry_snapshot", &OdometryConfig::save_odometry_snapshot},
        {"calibration_validation_points", &OdometryConfig::calibration_validation_points},
        {"use_consistency", &OdometryConfig::use_consistency},
        {"num_constistency_iter", &OdometryConfig::num_constistency_iter},
        {"use_mutliple_gaussian", &OdometryConfig::use_mutliple_gaussian},
        {"lidar_odometry_motion_model_x_1_sigma_m", &OdometryConfig::lidar_odometry_motion_model_x_1_sigma_m},
        {"lidar_odometry_motion_model_y_1_sigma_m", &OdometryConfig::lidar_odometry_motion_model_y_1_sigma_m},
        {"lidar_odometry_motion_model_z_1_sigma_m", &OdometryConfig::lidar_odometry_motion_model_z_1_sigma_m},
        {"lidar_odometry_motion_model_om_1_sigma_deg", &OdometryConfig::lidar_odometry_motion_model_om_1_sigma_deg},
        {"lidar_odometry_motion_model_fi_1_sigma_deg", &OdometryConfig::lidar_odometry_motion_model_fi_1_sigma_deg},
        {"lidar_odometry_motion_model_ka_1_sigma_deg", &OdometryConfig::lidar_odometry_motion_model_ka_1_sigma_deg},
        {"lidar_odometry_motion_model_acc_1_sigma_mps2", &OdometryConfig::lidar_odometry_motion_model_acc_1_sigma_mps2},
        {"max_prior_velocity_mps", &OdometryConfig::max_prior_velocity_mps},
        {"use_time_scaled_rotation_sigma", &OdometryConfig::use_time_scaled_rotation_sigma},
        {"gyro_arw_x_deg_sqrt_s", &OdometryConfig::gyro_arw_x_deg_sqrt_s},
        {"gyro_arw_y_deg_sqrt_s", &OdometryConfig::gyro_arw_y_deg_sqrt_s},
        {"gyro_arw_z_deg_sqrt_s", &OdometryConfig::gyro_arw_z_deg_sqrt_s},
        {"rotation_sigma_inflation_k", &OdometryConfig::rotation_sigma_inflation_k},
        {"rotation_sigma_floor_deg", &OdometryConfig::rotation_sigma_floor_deg},
        {"rotation_sigma_inflation_k_turn", &OdometryConfig::rotation_sigma_inflation_k_turn},
        {"turn_rate_threshold_deg_s", &OdometryConfig::turn_rate_threshold_deg_s},
        {"export_merged_laz", &OdometryConfig::export_merged_laz},
        {"use_loop_closure", &OdometryConfig::use_loop_closure},
        {"loop_closure_segment_windows", &OdometryConfig::loop_closure_segment_windows},
        {"loop_closure_min_time_gap_s", &OdometryConfig::loop_closure_min_time_gap_s},
        {"loop_closure_search_radius_m", &OdometryConfig::loop_closure_search_radius_m},
        {"loop_closure_max_vertical_offset_m", &OdometryConfig::loop_closure_max_vertical_offset_m},
        {"loop_closure_max_correction_m", &OdometryConfig::loop_closure_max_correction_m},
        {"loop_closure_min_fitness", &OdometryConfig::loop_closure_min_fitness},
        {"loop_closure_odometry_sigma_m", &OdometryConfig::loop_closure_odometry_sigma_m},
        {"loop_closure_odometry_sigma_deg", &OdometryConfig::loop_closure_odometry_sigma_deg},
        {"loop_closure_edge_sigma_m", &OdometryConfig::loop_closure_edge_sigma_m},
        {"loop_closure_edge_sigma_deg", &OdometryConfig::loop_closure_edge_sigma_deg},
        {"loop_closure_candidates", &OdometryConfig::loop_closure_candidates},
        {"loop_closure_max_residual_m", &OdometryConfig::loop_closure_max_residual_m},
        {"loop_closure_max_residual_deg", &OdometryConfig::loop_closure_max_residual_deg},
        {"loop_closure_kernel_scale", &OdometryConfig::loop_closure_kernel_scale},
        {"loop_closure_cross_floor_max_vertical_m", &OdometryConfig::loop_closure_cross_floor_max_vertical_m},
        {"loop_closure_cross_floor_min_fitness", &OdometryConfig::loop_closure_cross_floor_min_fitness},
        {"loop_closure_cross_floor_max_angle_deg", &OdometryConfig::loop_closure_cross_floor_max_angle_deg},
        {"loop_closure_cross_floor_rotation_sigma_deg", &OdometryConfig::loop_closure_cross_floor_rotation_sigma_deg},
        {"loop_closure_cross_floor_min_time_gap_s", &OdometryConfig::loop_closure_cross_floor_min_time_gap_s},
        {"loop_closure_cross_floor_search_m", &OdometryConfig::loop_closure_cross_floor_search_m},
        {"loop_closure_cross_floor_min_information_ratio", &OdometryConfig::loop_closure_cross_floor_min_information_ratio},
        {"loop_closure_cross_floor_keep_height", &OdometryConfig::loop_closure_cross_floor_keep_height},
        {"loop_closure_cross_floor_exclude_segments", &OdometryConfig::loop_closure_cross_floor_exclude_segments},
        {"loop_closure_cross_floor_only_near", &OdometryConfig::loop_closure_cross_floor_only_near},
        {"loop_closure_cross_floor_point_edges", &OdometryConfig::loop_closure_cross_floor_point_edges},
        {"use_gyro_bias_estimation", &OdometryConfig::use_gyro_bias_estimation},
        {"gyro_bias_rw_deg_s_sqrt_s", &OdometryConfig::gyro_bias_rw_deg_s_sqrt_s},
        {"gyro_bias_initial_sigma_deg_s", &OdometryConfig::gyro_bias_initial_sigma_deg_s},
        {"gyro_bias_max_deg_s", &OdometryConfig::gyro_bias_max_deg_s},
        {"handoff_tail_nodes", &OdometryConfig::handoff_tail_nodes},
        {"use_imu_translation_prediction", &OdometryConfig::use_imu_translation_prediction},
        {"imu_acc_smoothing_s", &OdometryConfig::imu_acc_smoothing_s},
        {"use_zero_velocity_detection", &OdometryConfig::use_zero_velocity_detection},
        {"use_imu_acceleration_term", &OdometryConfig::use_imu_acceleration_term},
        {"imu_acceleration_sigma_mps2", &OdometryConfig::imu_acceleration_sigma_mps2},
        {"imu_acceleration_span_s", &OdometryConfig::imu_acceleration_span_s},
        {"imu_acceleration_offset_initial_sigma_mps2", &OdometryConfig::imu_acceleration_offset_initial_sigma_mps2},
        {"imu_acceleration_offset_rw_mps2_sqrt_s", &OdometryConfig::imu_acceleration_offset_rw_mps2_sqrt_s},
        {"imu_acceleration_only_when_degenerate", &OdometryConfig::imu_acceleration_only_when_degenerate},
        {"imu_degenerate_information_ratio", &OdometryConfig::imu_degenerate_information_ratio},
        {"imu_degenerate_weight_ratio_full", &OdometryConfig::imu_degenerate_weight_ratio_full},
        {"imu_degenerate_smoothing_windows", &OdometryConfig::imu_degenerate_smoothing_windows},
        {"imu_acceleration_offset_freeze_blind", &OdometryConfig::imu_acceleration_offset_freeze_blind},
        {"imu_always_on", &OdometryConfig::imu_always_on},
        {"imu_accel_noise_density_mps2_sqrt_hz", &OdometryConfig::imu_accel_noise_density_mps2_sqrt_hz},
        {"imu_accel_floor_sigma_mps2", &OdometryConfig::imu_accel_floor_sigma_mps2},
        {"imu_velocity_initial_sigma_mps", &OdometryConfig::imu_velocity_initial_sigma_mps},
        {"imu_zero_velocity_sigma_mps", &OdometryConfig::imu_zero_velocity_sigma_mps},
        {"degenerate_projection_ratio", &OdometryConfig::degenerate_projection_ratio},
        {"degenerate_projection_ratio_full", &OdometryConfig::degenerate_projection_ratio_full},
        {"imu_degenerate_max_duration_s", &OdometryConfig::imu_degenerate_max_duration_s},
        {"imu_degenerate_translation_sigma_m", &OdometryConfig::imu_degenerate_translation_sigma_m},
        {"zero_velocity_acc_threshold_mps2", &OdometryConfig::zero_velocity_acc_threshold_mps2},
        {"zero_velocity_gyro_threshold_deg_s", &OdometryConfig::zero_velocity_gyro_threshold_deg_s},
        {"lidar_odometry_motion_model_fix_origin_x_1_sigma_m", &OdometryConfig::lidar_odometry_motion_model_fix_origin_x_1_sigma_m},
        {"lidar_odometry_motion_model_fix_origin_y_1_sigma_m", &OdometryConfig::lidar_odometry_motion_model_fix_origin_y_1_sigma_m},
        {"lidar_odometry_motion_model_fix_origin_z_1_sigma_m", &OdometryConfig::lidar_odometry_motion_model_fix_origin_z_1_sigma_m},
        {"lidar_odometry_motion_model_fix_origin_om_1_sigma_deg", &OdometryConfig::lidar_odometry_motion_model_fix_origin_om_1_sigma_deg},
        {"lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg", &OdometryConfig::lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg},
        {"lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg", &OdometryConfig::lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg},
        {"use_robust_and_accurate_lidar_odometry", &OdometryConfig::use_robust_and_accurate_lidar_odometry},
        {"distance_bucket", &OdometryConfig::distance_bucket},
        {"polar_angle_deg", &OdometryConfig::polar_angle_deg},
        {"azimutal_angle_deg", &OdometryConfig::azimutal_angle_deg},
        {"robust_and_accurate_lidar_odometry_iterations", &OdometryConfig::robust_and_accurate_lidar_odometry_iterations},
        {"distance_bucket_rigid_sf", &OdometryConfig::distance_bucket_rigid_sf},
        {"polar_angle_deg_rigid_sf", &OdometryConfig::polar_angle_deg_rigid_sf},
        {"azimutal_angle_deg_rigid_sf", &OdometryConfig::azimutal_angle_deg_rigid_sf},
        {"robust_and_accurate_lidar_odometry_rigid_sf_iterations", &OdometryConfig::robust_and_accurate_lidar_odometry_rigid_sf_iterations},
        {"rgd_sf_sigma_x_m", &OdometryConfig::rgd_sf_sigma_x_m},
        {"rgd_sf_sigma_y_m", &OdometryConfig::rgd_sf_sigma_y_m},
        {"rgd_sf_sigma_z_m", &OdometryConfig::rgd_sf_sigma_z_m},
        {"rgd_sf_sigma_om_deg", &OdometryConfig::rgd_sf_sigma_om_deg},
        {"rgd_sf_sigma_fi_deg", &OdometryConfig::rgd_sf_sigma_fi_deg},
        {"rgd_sf_sigma_ka_deg", &OdometryConfig::rgd_sf_sigma_ka_deg},
        {"max_distance_lidar_rigid_sf", &OdometryConfig::max_distance_lidar_rigid_sf}};

    // [paths] keys, stored in RunPaths
    const std::map<std::string, std::string RunPaths::*> RUN_PATH_POINTERS = {
        {"current_output_dir", &RunPaths::current_output_dir},
        {"working_directory_preview", &RunPaths::working_directory_preview}};

    // Special handling for TaitBryanPose members
    const std::map<std::string, double TaitBryanPose::*> MOTION_MODEL_CORRECTION_POINTERS = {
        {"motion_model_correction_om", &TaitBryanPose::om},
        {"motion_model_correction_fi", &TaitBryanPose::fi},
        {"motion_model_correction_ka", &TaitBryanPose::ka}};

    std::map<std::string, std::vector<std::string>> CATEGORIES = {
        {"version_info", {"software_version", "config_version", "build_date"}},
        {"performance", {"real_time_threshold_seconds"}},
        {"filter_points", {"filter_threshold_xy_inner", "filter_threshold_xy_outer", "decimation", "threshould_output_filter", "min_counter_concatenated_trajectory_nodes", "export_merged_laz"}},
        {"madgwick_filter", {"fusionConventionNwu", "fusionConventionEnu", "fusionConventionNed", "ahrs_gain", "use_remove_imu_bias_from_first_stationary_scan"}},
        {"lidar_odometry_control", {"use_motion_from_previous_step", "nr_iter", "convergence_delta_threshold", "sliding_window_trajectory_length_threshold", "max_distance_lidar", "threshold_initial_points", "threshold_nr_poses", "min_distance_lidar", "min_distance_lidar_outdoor_grid", "lm_factor_initial", "use_exact_ndt_bucket_statistics", "ndt_plane_in_plane_sigma_m"}},
        {"lidar_imu_extrinsics", {"lidar_imu_lever_arm_x_m", "lidar_imu_lever_arm_y_m", "lidar_imu_lever_arm_z_m"}},
        {"lidar_odometry_debug_info", {"save_calibration_validation", "calibration_validation_points", "log_iteration_convergence", "log_stage_timing", "log_window_diagnostics", "save_odometry_snapshot"}},
        {"consistency", {"use_consistency", "num_constistency_iter", "use_mutliple_gaussian", "consistency_min_distance"}},
        {"motion_model_uncertainty", {"lidar_odometry_motion_model_x_1_sigma_m", "lidar_odometry_motion_model_y_1_sigma_m", "lidar_odometry_motion_model_z_1_sigma_m", "lidar_odometry_motion_model_om_1_sigma_deg", "lidar_odometry_motion_model_fi_1_sigma_deg", "lidar_odometry_motion_model_ka_1_sigma_deg", "lidar_odometry_motion_model_acc_1_sigma_mps2", "max_prior_velocity_mps", "use_time_scaled_rotation_sigma", "gyro_arw_x_deg_sqrt_s", "gyro_arw_y_deg_sqrt_s", "gyro_arw_z_deg_sqrt_s", "rotation_sigma_inflation_k", "rotation_sigma_floor_deg", "rotation_sigma_inflation_k_turn", "turn_rate_threshold_deg_s"}},
        {"gyro_bias_estimation", {"use_gyro_bias_estimation", "gyro_bias_rw_deg_s_sqrt_s", "gyro_bias_initial_sigma_deg_s", "gyro_bias_max_deg_s", "handoff_tail_nodes"}},
        {"loop_closure", {"use_loop_closure", "loop_closure_segment_windows", "loop_closure_min_time_gap_s", "loop_closure_search_radius_m", "loop_closure_max_vertical_offset_m", "loop_closure_max_correction_m", "loop_closure_min_fitness", "loop_closure_odometry_sigma_m", "loop_closure_odometry_sigma_deg", "loop_closure_edge_sigma_m", "loop_closure_edge_sigma_deg", "loop_closure_candidates", "loop_closure_max_residual_m", "loop_closure_max_residual_deg", "loop_closure_kernel_scale", "loop_closure_cross_floor_max_vertical_m", "loop_closure_cross_floor_min_fitness", "loop_closure_cross_floor_max_angle_deg", "loop_closure_cross_floor_rotation_sigma_deg", "loop_closure_cross_floor_min_time_gap_s", "loop_closure_cross_floor_search_m", "loop_closure_cross_floor_min_information_ratio", "loop_closure_cross_floor_keep_height", "loop_closure_cross_floor_exclude_segments", "loop_closure_cross_floor_only_near", "loop_closure_cross_floor_point_edges"}},
        {"imu_translation", {"use_imu_translation_prediction", "imu_acc_smoothing_s", "use_zero_velocity_detection", "zero_velocity_acc_threshold_mps2", "zero_velocity_gyro_threshold_deg_s", "use_imu_acceleration_term", "imu_acceleration_sigma_mps2", "imu_acceleration_span_s", "imu_acceleration_offset_initial_sigma_mps2", "imu_acceleration_offset_rw_mps2_sqrt_s", "imu_acceleration_only_when_degenerate", "imu_degenerate_information_ratio", "imu_degenerate_weight_ratio_full", "imu_degenerate_smoothing_windows", "imu_acceleration_offset_freeze_blind", "imu_always_on", "imu_accel_noise_density_mps2_sqrt_hz", "imu_accel_floor_sigma_mps2", "imu_velocity_initial_sigma_mps", "imu_zero_velocity_sigma_mps", "degenerate_projection_ratio", "degenerate_projection_ratio_full", "imu_degenerate_max_duration_s", "imu_degenerate_translation_sigma_m"}},
        {"motion_model_correction", {"motion_model_correction_om", "motion_model_correction_fi", "motion_model_correction_ka"}},
        {"motion_model_first_node_prior_uncertainty", {"lidar_odometry_motion_model_fix_origin_x_1_sigma_m", "lidar_odometry_motion_model_fix_origin_y_1_sigma_m", "lidar_odometry_motion_model_fix_origin_z_1_sigma_m", "lidar_odometry_motion_model_fix_origin_om_1_sigma_deg", "lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg", "lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg"}},
        {"robust_lidar_odometry", {"use_robust_and_accurate_lidar_odometry", "distance_bucket", "polar_angle_deg", "azimutal_angle_deg", "robust_and_accurate_lidar_odometry_iterations", "distance_bucket_rigid_sf", "polar_angle_deg_rigid_sf", "azimutal_angle_deg_rigid_sf", "robust_and_accurate_lidar_odometry_rigid_sf_iterations", "max_distance_lidar_rigid_sf", "rgd_sf_sigma_x_m", "rgd_sf_sigma_y_m", "rgd_sf_sigma_z_m", "rgd_sf_sigma_om_deg", "rgd_sf_sigma_fi_deg", "rgd_sf_sigma_ka_deg"}},
        {"paths", {"current_output_dir", "working_directory_preview"}},
        { "misc", {"clear_color"} } };

    bool SaveParametersToTomlFile(const std::string &filepath, const OdometryConfig &params, const RunPaths &paths, const GuiPreferences &gui);
    
    // Version validation and handling functions
    struct VersionInfo {
        std::string software_version;
        std::string config_version;
        std::string build_date;
        bool found = false;
    };
    
    VersionInfo CheckConfigVersion(const std::string &filepath);
    bool IsVersionCompatible(const std::string &file_version, const std::string &current_version);
    void HandleMissingVersion(OdometryConfig &params);
    
    template <typename T>
    void set_if_exists(NdtGridParameters &grid, const toml::table *tbl, const std::string &key, T NdtGridParameters::*member);
    void read_grid_params(NdtGridParameters &grid, const toml::table *grid_tbl);
    bool LoadParametersFromTomlFile(const std::string &filepath, OdometryConfig &params, RunPaths &paths, GuiPreferences &gui);
};

#endif