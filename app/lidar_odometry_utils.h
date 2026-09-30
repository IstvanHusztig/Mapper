#ifndef _LIDAR_ODOMETRY_UTILS_H_
#define _LIDAR_ODOMETRY_UTILS_H_

#include <laszip/laszip_api.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Fusion.h>
#include <map>
#include <filesystem>
#include <execution>
#include <structures.h>
#include <ndt.h>
#include <nlohmann/json.hpp>
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <chrono>
#include <python-scripts/constraints/smoothness_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
#include <common/include/cauchy.h>
#if WITH_GUI == 1
#include <imgui.h>
#endif

namespace fs = std::filesystem;

using NDTBucketMapType = std::unordered_map<uint64_t, NDT::Bucket>;
using NDTBucketMapType2 = std::unordered_map<uint64_t, NDT::Bucket2>;

// Helper function for getting software version from CMake macros
inline std::string get_software_version() {
    #ifdef HDMAPPING_VERSION_MAJOR
        return std::to_string(HDMAPPING_VERSION_MAJOR) + "." + 
               std::to_string(HDMAPPING_VERSION_MINOR) + "." + 
               std::to_string(HDMAPPING_VERSION_PATCH);
    #else
        return "0.84.0"; // fallback if CMake macros not available
    #endif
}

struct LidarOdometryParams
{
    // version information - automatically generated from CMake build system
    std::string software_version = get_software_version();
    std::string config_version = "1.0";
    std::string build_date = __DATE__;
    
    //performance
    bool useMultithread = false; // off: parallel summation of the NDT terms is not deterministic (float order); kept for a later rework
    double real_time_threshold_seconds = 10.0; // for realtime use: threshold_nr_poses * 0.005, where 0.005 is related with IMU frequency

    //filter points
    double filter_threshold_xy_inner = 0.3; //filtering points during load
    double filter_threshold_xy_outer = 70.0; //filtering points during load
    double decimation = 0.01;                // filtering points during load
    double threshould_output_filter = 0.5; //for export --> all points xyz.norm() < threshould_output_filter will be removed
    int min_counter_concatenated_trajectory_nodes = 0; // export: the last output chunk is written only if it has more windows than this (0 = always)

    //Madgwick filter
    bool fusionConventionNwu = true;
    bool fusionConventionEnu = false;
    bool fusionConventionNed = false;
    double ahrs_gain = 0.05;

    //lidar odometry control
    bool use_motion_from_previous_step = true;
    int nr_iter = 100;
    // a window stops iterating once the sum of |parameter updates| of an LM step falls below this value
    // (1e-12 is practically never reached, so all nr_iter iterations run)
    double convergence_delta_threshold = 1e-12;
    NDT::GridParameters in_out_params_indoor;
    NDT::GridParameters in_out_params_outdoor;
    double sliding_window_trajectory_length_threshold = 5.0;
    double max_distance_lidar = 30.0; // I am not processing data above dist distance in lidar odometry
    int threshold_initial_points = 10000;
    int threshold_nr_poses = 20;
    double min_distance_lidar = 0.06;              // blind zone: points closer (3D norm) are skipped by the fine (indoor) NDT grid
    double min_distance_lidar_outdoor_grid = 1.0;  // points closer (3D norm) are skipped by the coarse (outdoor) NDT grid
    double lm_factor_initial = 5.0;                // initial Levenberg-Marquardt damping (x10 every 10 iterations)

    //lidar-imu extrinsics (applied to raw points when no LiDAR calibration .json is present)
    double lidar_imu_lever_arm_x_m = 0.00667;
    double lidar_imu_lever_arm_y_m = -0.014655;
    double lidar_imu_lever_arm_z_m = 0.007698;

    //lidar odometry debug info
    bool save_calibration_validation = false;
    int calibration_validation_points = 1000000;

    //consistency
    bool use_consistency = false;   // run the whole-trajectory consistency pass after the odometry (CLI; the GUI has a button)
    int num_constistency_iter = 10;
    bool use_mutliple_gaussian = false;
    double consistency_min_distance = 0.4; // points closer (3D norm) are skipped in Consistency2 (masks the operator's body)

    // motion_model uncertainty
    double lidar_odometry_motion_model_x_1_sigma_m = 0.0001;
    double lidar_odometry_motion_model_y_1_sigma_m = 0.0001;
    double lidar_odometry_motion_model_z_1_sigma_m = 0.0001;

    double lidar_odometry_motion_model_om_1_sigma_deg = 0.3;
    double lidar_odometry_motion_model_fi_1_sigma_deg = 0.3;
    double lidar_odometry_motion_model_ka_1_sigma_deg = 0.3;

    // velocity-continuity prior: world-frame acceleration sigma [m/s^2] between consecutive nodes,
    // also applied across window boundaries (removes velocity kinks every threshold_nr_poses); 0 = disabled
    double lidar_odometry_motion_model_acc_1_sigma_mps2 = 0.0;

    // time-scaled rotation prior between consecutive nodes:
    // sigma_axis(dt) = max(k * gyro_arw_axis * sqrt(dt), rotation_sigma_floor_deg); replaces the constant om/fi/ka sigmas above
    bool use_time_scaled_rotation_sigma = true;
    double gyro_arw_x_deg_sqrt_s = 0.028;      // gyro angle random walk [deg/sqrt(s)], Unitree L2 static recording (43/static)
    double gyro_arw_y_deg_sqrt_s = 0.042;
    double gyro_arw_z_deg_sqrt_s = 0.057;
    double rotation_sigma_inflation_k = 4.0;   // covers model error beyond white gyro noise (timestamp jitter, vibration); tuned on datasets 42/43
    double rotation_sigma_floor_deg = 0.002;   // lower bound for near-zero dt

    // online gyro bias estimation (3 extra unknowns per window, carried across windows as a prior)
    bool use_gyro_bias_estimation = true;          // validated against a static recording (43/static)
    double gyro_bias_rw_deg_s_sqrt_s = 0.05;       // bias random walk: prior sigma grows by rw * sqrt(window duration)
    double gyro_bias_initial_sigma_deg_s = 10.0;   // prior sigma of the bias in the first window
    double gyro_bias_max_deg_s = 20.0;             // per-axis clamp of the estimated bias

    // window hand-off: the last handoff_tail_nodes nodes of each window are re-propagated from the IMU
    // before they seed the next window (they carry almost no points); 0 = disabled
    int handoff_tail_nodes = 30;

    // IMU-aided translation prediction: instead of carrying the previous window's constant velocity, the initial guess
    // (and with it the translation motion-model prior) of each window is integrated from the accelerometer;
    // gravity is removed per node with the AHRS tilt: a = R * (f - |g| * R_ahrs^T * z)
    bool use_imu_translation_prediction = false;
    double imu_acc_smoothing_s = 0.01; // moving-average width over the accelerometer (evens out IMU sample bursts)
    // zero-velocity update: a window in which the device rests is predicted without motion
    bool use_zero_velocity_detection = false;
    double zero_velocity_acc_threshold_mps2 = 0.3; // max |world acceleration| inside the window
    double zero_velocity_gyro_threshold_deg_s = 3.0; // max rotation rate inside the window

    // motion_model first trajectory node prior uncertainty
    double lidar_odometry_motion_model_fix_origin_x_1_sigma_m = 0.000001;
    double lidar_odometry_motion_model_fix_origin_y_1_sigma_m = 0.000001;
    double lidar_odometry_motion_model_fix_origin_z_1_sigma_m = 0.000001;
    double lidar_odometry_motion_model_fix_origin_om_1_sigma_deg = 0.000001;
    double lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg = 0.000001;
    double lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg = 0.000001;

    // motion model correction (experimental --> not recommended yet)
    TaitBryanPose motion_model_correction;

    // robust lidar odometry control (experimental --> not recommended yet)
    bool use_robust_and_accurate_lidar_odometry = false;
    double distance_bucket = 0.2;
    double polar_angle_deg = 10.0;
    double azimutal_angle_deg = 10.0;
    int robust_and_accurate_lidar_odometry_iterations = 20;
    double distance_bucket_rigid_sf = 0.5;
    double polar_angle_deg_rigid_sf = 10.0;
    double azimutal_angle_deg_rigid_sf = 10.0;
    int robust_and_accurate_lidar_odometry_rigid_sf_iterations = 30;
    double max_distance_lidar_rigid_sf = 70.0;
    double rgd_sf_sigma_x_m = 0.001;
    double rgd_sf_sigma_y_m = 0.001;
    double rgd_sf_sigma_z_m = 0.001;
    double rgd_sf_sigma_om_deg = 0.01;
    double rgd_sf_sigma_fi_deg = 0.01;
    double rgd_sf_sigma_ka_deg = 0.01;

    //paths
    std::string current_output_dir = "";
    std::string working_directory_preview = "";
    std::string working_directory_cache = "";
    
    //other
    Eigen::Affine3d m_g = Eigen::Affine3d::Identity();
    std::vector<Point3Di> initial_points;
    double consecutive_distance = 0.0;
    std::vector<Point3Di> reference_points;
    NDTBucketMapType reference_buckets;
    double total_length_of_calculated_trajectory = 0.0;
    NDTBucketMapType buckets_indoor;
    NDTBucketMapType buckets_outdoor;
#if WITH_GUI == 1
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
#endif

    bool use_remove_imu_bias_from_first_stationary_scan = true;

    //ablation study
    bool ablation_study_use_planarity = false;
    bool ablation_study_use_norm = false;
    bool ablation_study_use_hierarchical_rgd = true;
    bool ablation_study_use_view_point_and_normal_vectors = true;
    bool save_index_pose = false;
};

inline Eigen::Vector3d get_lidar_imu_lever_arm(const LidarOdometryParams &params)
{
    return Eigen::Vector3d(params.lidar_imu_lever_arm_x_m, params.lidar_imu_lever_arm_y_m, params.lidar_imu_lever_arm_z_m);
}

unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z);
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b);

// this function reduces number of points by preserving only first point for each bucket {bucket_x, bucket_y, bucket_z}
std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z);

// this function updates each bucket (mean value, covariance) in regular grid decomposition
void update_rgd(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global, Eigen::Vector3d viewport = Eigen::Vector3d(0, 0, 0));

void update_rgd_spherical_coordinates(NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                                      std::vector<Point3Di> &points_global, std::vector<Eigen::Vector3d> &points_global_spherical);

//! This function load inertial measurement unit data.
//! This function expects a file with the following format:
//! timestamp angular_velocity_x angular_velocity_y angular_velocity_z linear_acceleration_x linear_acceleration_y linear_acceleration_z imu_id
//! @note imu_id is an optional column, if not present, it is assumed that all data comes from the same IMU.
//! @param imu_file - path to file with IMU data
//! @param imuToUse - id number of IMU to use, the same index as in pointcloud return by @ref load_point_cloud
//! @return vector of tuples (std::pair<timestamp, timestampUnix>, angular_velocity, linear_acceleration)
std::vector<std::tuple<std::pair<double, double>, FusionVector, FusionVector>> load_imu(const std::string &imu_file, int imuToUse);

//! This function load point cloud from LAS/LAZ file.
//! Optionally it can apply extrinsic calibration to each point.
//! The calibration is stored in a map, where key is laser scanner id.
//! The id of the laser scanner is stored in LAS/LAZ file as `user_data` field.
//! @param lazFile - path to file with point cloud
//! @param ommit_points_with_timestamp_equals_zero - if true, points with timestamp == 0 will be omited
//! @param filter_threshold_xy - threshold for filtering points in xy plane
//! @param calibrations - map of calibrations for each scanner key is scanner id.
//! @return vector of points of @ref Point3Di type
std::vector<Point3Di> load_point_cloud(const std::string &lazFile, bool ommit_points_with_timestamp_equals_zero, double filter_threshold_xy_inner, double filter_threshold_xy_outer,
                                       const std::unordered_map<int, Eigen::Affine3d> &calibrations, const Eigen::Vector3d &lever_arm);

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames);

fs::path get_next_result_path(const std::string working_directory);

// rotation part of the motion model: time-scaled sigma and online gyro bias (see LidarOdometryParams)
struct MotionModelRotationOptions
{
    bool time_scaled_sigma = false;
    Eigen::Vector3d arw_rad_sqrt_s = Eigen::Vector3d::Zero(); // om/fi/ka axes
    double inflation_k = 1.0;
    double sigma_floor_rad = 0.0;

    bool estimate_bias = false;
    Eigen::Vector3d bias_prior_mean = Eigen::Vector3d::Zero();  // [rad/s]
    Eigen::Matrix3d bias_prior_cov = Eigen::Matrix3d::Identity(); // [(rad/s)^2]
    double bias_max_rad_s = 0.0;                                  // per-axis clamp, 0 = none
};

MotionModelRotationOptions get_motion_model_rotation_options(const LidarOdometryParams &params);

// this function performs main LiDAR odometry calculations
// gyro_bias / gyro_bias_cov are updated only when rotation_options.estimate_bias is set
void optimize_lidar_odometry(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                             std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                             NDT::GridParameters &rgd_params_indoor, NDTBucketMapType &buckets_indoor,
                             NDT::GridParameters &rgd_params_outdoor, NDTBucketMapType &buckets_outdoor,
                             bool useMultithread, double min_distance, double min_distance_outdoor_grid, double max_distance, double &delta, double lm_factor, TaitBryanPose motion_model_correction,
                             double lidar_odometry_motion_model_x_1_sigma_m,
                             double lidar_odometry_motion_model_y_1_sigma_m,
                             double lidar_odometry_motion_model_z_1_sigma_m,
                             double lidar_odometry_motion_model_om_1_sigma_deg,
                             double lidar_odometry_motion_model_fi_1_sigma_deg,
                             double lidar_odometry_motion_model_ka_1_sigma_deg,
                             double lidar_odometry_motion_model_fix_origin_x_1_sigma_m,
                             double lidar_odometry_motion_model_fix_origin_y_1_sigma_m,
                             double lidar_odometry_motion_model_fix_origin_z_1_sigma_m,
                             double lidar_odometry_motion_model_fix_origin_om_1_sigma_deg,
                             double lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg,
                             double lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg,
                             bool ablation_study_use_planarity,
                             bool ablation_study_use_norm,
                             bool ablation_study_use_hierarchical_rgd,
                             bool ablation_study_use_view_point_and_normal_vectors,
                             const std::vector<std::pair<double, double>> &timestamps,
                             double acc_sigma_mps2,
                             bool use_prev_velocity,
                             const Eigen::Vector3d &prev_velocity,
                             const MotionModelRotationOptions &rotation_options,
                             Eigen::Vector3d &gyro_bias,
                             Eigen::Matrix3d &gyro_bias_cov);

void optimize_sf2(std::vector<Point3Di> &intermediate_points, std::vector<Point3Di> &intermediate_points_sf, std::vector<Eigen::Affine3d> &intermediate_trajectory,
                  const std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
                  NDT::GridParameters &rgd_params, bool useMultithread, double wx, double wy, double wz, double wom, double wfi, double wka);

// this function registers initial point cloud to geoferenced point cloud
void align_to_reference(NDT::GridParameters &rgd_params, std::vector<Point3Di> &initial_points, Eigen::Affine3d &m_g, NDTBucketMapType &buckets);

bool compute_step_2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params, double &ts_failure, std::atomic<float> &loProgress, const std::atomic<bool> &pause, bool debugMsg);


namespace MLvxCalib
{

    //! Parse the calibration file and return a map from sensor id to serial number.
    //! Sensor id is the id of the point in laz file.
    //! Serial number is the serial number of the Livox.
    //! @param filename calibration file
    //! @return map of serial number, where key is sensor id.
    std::unordered_map<int, std::string> GetIdToSnMapping(const std::string &filename);

    //! Parse the calibration file and return a map from serial number to calibration.
    //! @param filename calibration file
    //! @return map of extrinsic calibration, where key is serial number of the lidar.
    std::unordered_map<std::string, Eigen::Affine3d> GetCalibrationFromFile(const std::string &filename);

    //! Parse the calibration file and return a serial number of the Livox to use for IMU.
    //! @param filename calibration file
    //! @return serial number of the Livox to use for IMU
    std::string GetImuSnToUse(const std::string &filename);

    //! Combine the id to serial number mapping and the calibration into a single map.
    //! The single map is from sensor id to calibration.
    //! @param idToSn mapping from serial number to Id number in pointcloud or IMU CSV
    //! @param calibration map of extrinsic calibration, where key is serial number of the lidar.
    //! @return map from sensor id to extrinsic calibration
    std::unordered_map<int, Eigen::Affine3d> CombineIntoCalibration(const std::unordered_map<int, std::string> &idToSn,
                                                                    const std::unordered_map<std::string, Eigen::Affine3d> &calibration);
    //! Get the id of the IMU to use.
    //! @param idToSn mapping from serial number to Id number in pointcloud or IMU CSV
    //! @param snToUse serial number of the Livox to use for IMU
    //! @return id of the IMU to use
    int GetImuIdToUse(const std::unordered_map<int, std::string> &idToSn, const std::string &snToUse);
}

void Consistency(std::vector<WorkerData> &worker_data, LidarOdometryParams &params);
void Consistency2(std::vector<WorkerData> &worker_data, LidarOdometryParams &params);
#endif
