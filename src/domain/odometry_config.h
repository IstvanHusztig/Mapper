#ifndef _ODOMETRY_CONFIG_H_
#define _ODOMETRY_CONFIG_H_

// the settings of a run (OdometryConfig), its folders (RunPaths) and the GUI-only settings (GuiPreferences)

#include <Eigen/Dense>
#include <string>
#include "dmapper_version.h"
#include <ndt_types.h>
#include <structures.h>

// Helper function for getting software version from CMake macros
inline std::string get_software_version() { return dmapper_version(); }

struct OdometryConfig
{
    // version information - automatically generated from CMake build system
    std::string software_version = get_software_version();
    std::string config_version = "1.0";
    std::string build_date = __DATE__;
    
    //performance
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
    NdtGridParameters in_out_params_indoor{}; // value-initialized: NdtGridParameters has no default member values
    NdtGridParameters in_out_params_outdoor{};
    double sliding_window_trajectory_length_threshold = 5.0;
    double max_distance_lidar = 30.0; // I am not processing data above dist distance in lidar odometry
    int threshold_initial_points = 10000;
    int threshold_nr_poses = 20;
    double min_distance_lidar = 0.06;              // blind zone: points closer (3D norm) are skipped by the fine (indoor) NDT grid
    double min_distance_lidar_outdoor_grid = 1.0;  // points closer (3D norm) are skipped by the coarse (outdoor) NDT grid
    double lm_factor_initial = 5.0;                // initial Levenberg-Marquardt damping (x10 every 10 iterations)
    // NDT map cells: true = mean, covariance and normal of all of a cell's points; false = the previous statistics
    // (first point as mean, normal fixed from the first 3 points)
    bool use_exact_ndt_bucket_statistics = false;

    //lidar-imu extrinsics (applied to raw points): the IMU position in the point frame, in the output axes; set per sensor in its TOML
    double lidar_imu_lever_arm_x_m = 0.00667;
    double lidar_imu_lever_arm_y_m = -0.014655;
    double lidar_imu_lever_arm_z_m = 0.007698;

    //lidar odometry debug info
    bool save_calibration_validation = false;
    // diagnostics only: log per window after how many iterations the poses settle (results unchanged)
    bool log_iteration_convergence = false;
    // diagnostics only: log the wall-clock time of each pipeline step and of the odometry's phases (results unchanged)
    bool log_stage_timing = false;
    // diagnostics only: one line per window with its time, iterations, displacement against the guess and the previous
    // velocity, and the state of the accelerometer term (results unchanged)
    bool log_window_diagnostics = false;
    // after the odometry the CLI writes odometry_snapshot.bin to the output folder, so that the post-processing can be run
    // again from it (dMapper <input> <params.toml> <output> --from-snapshot <file>) without running the odometry again
    bool save_odometry_snapshot = false;
    int calibration_validation_points = 1000000;

    //consistency
    bool use_consistency = false;   // run the whole-trajectory consistency pass after the odometry (CLI; the GUI has a button)
    int num_constistency_iter = 10;
    bool use_mutliple_gaussian = false;
    double consistency_min_distance = 0.4; // points closer (3D norm) are skipped in Consistency2 (masks the operator's body)
    // gyro rotation term in Consistency: consecutive nodes of a window keep the IMU's relative rotation (orientation
    // increments minus the window's gyro bias), sigma = consistency_gyro_sigma_scale x the odometry's time-scaled rotation
    // sigma; a window whose IMU rotation differs from its odometry result by more than consistency_gyro_gate_deg gets none
    bool consistency_gyro_rotation_term = false;
    double consistency_gyro_sigma_scale = 3.0;
    double consistency_gyro_gate_deg = 3.0;

    // evaluation: points in odd time blocks of holdout_block_s seconds are left out of every map and point term
    // (odometry, loop closure, consistency) but saved, for tools/holdout_sharpness.py (holdout.h); 0 = off
    double holdout_block_s = 0.0;

    // motion_model uncertainty
    double lidar_odometry_motion_model_x_1_sigma_m = 0.0001;
    double lidar_odometry_motion_model_y_1_sigma_m = 0.0001;
    double lidar_odometry_motion_model_z_1_sigma_m = 0.0001;

    double lidar_odometry_motion_model_om_1_sigma_deg = 0.53173616;
    double lidar_odometry_motion_model_fi_1_sigma_deg = 0.53173616;
    double lidar_odometry_motion_model_ka_1_sigma_deg = 0.53173616;

    // velocity-continuity prior: world-frame acceleration sigma [m/s^2] between consecutive nodes,
    // also applied across window boundaries (removes velocity kinks every threshold_nr_poses); 0 = disabled
    double lidar_odometry_motion_model_acc_1_sigma_mps2 = 0.0;
    // the previous window's final velocity (over its last 20 nodes) is used as a prior only up to this speed [m/s]
    double max_prior_velocity_mps = 10.0;

    // time-scaled rotation prior between consecutive nodes:
    // sigma_axis(dt) = max(k * gyro_arw_axis * sqrt(dt), rotation_sigma_floor_deg); replaces the constant om/fi/ka sigmas above
    bool use_time_scaled_rotation_sigma = true;
    double gyro_arw_x_deg_sqrt_s = 0.028;      // gyro angle random walk of the sensor's IMU [deg/sqrt(s)], measured at rest; set per sensor in its TOML
    double gyro_arw_y_deg_sqrt_s = 0.042;
    double gyro_arw_z_deg_sqrt_s = 0.057;
    double rotation_sigma_inflation_k = 7.0898154; // covers model error beyond white gyro noise (timestamp jitter, vibration); 4.0 x sqrt(pi), as the prior lost its 1/pi Cauchy factor
    double rotation_sigma_floor_deg = 0.0035449077; // lower bound for near-zero dt (0.002 x sqrt(pi))
    // in windows that turn faster than turn_rate_threshold_deg_s about the vertical (mean over the window, from the IMU),
    // rotation_sigma_inflation_k_turn replaces rotation_sigma_inflation_k: tight turns (stairwells) trust the gyro more,
    // since the LiDAR matching is weakest there; 0 = off
    double rotation_sigma_inflation_k_turn = 0.0;
    double turn_rate_threshold_deg_s = 30.0;

    // loop closure after the odometry (see loop_closure.h): segments of loop_closure_segment_windows windows,
    // revisits (same floor) = segments with at least loop_closure_min_path_m walked between them and centres within
    // min(loop_closure_search_radius_m, loop_closure_radius_per_path * that path) horizontally
    // (loop_closure_max_vertical_offset_m vertically); registered revisits with an overlap of at least
    // loop_closure_min_fitness become pose-graph edges
    // export: also write all chunks into one scan_lio_merged.laz in world coordinates
    bool export_merged_laz = false;

    bool use_loop_closure = false;
    int loop_closure_segment_windows = 6;
    double loop_closure_min_path_m = 8.0;
    double loop_closure_radius_per_path = 0.5;
    // same-floor loop edges kept per segment: 2 = the best near and the best far revisit, 1 (or less) = the best one
    int loop_closure_edges_per_segment = 2;
    double loop_closure_search_radius_m = 8.0;
    double loop_closure_max_vertical_offset_m = 1.5;
    double loop_closure_max_correction_m = 6.0;
    double loop_closure_min_fitness = 0.5;
    double loop_closure_odometry_sigma_m = 0.1;
    double loop_closure_odometry_sigma_deg = 0.5;
    double loop_closure_edge_sigma_m = 0.05;
    double loop_closure_edge_sigma_deg = 0.3;
    // same-floor registrations per segment (revisit_tries in loop_closure.h), and the largest disagreement between the
    // solved pose graph and a loop edge before that edge is dropped as a false match
    int loop_closure_candidates = 4;
    double loop_closure_max_residual_m = 0.15;
    double loop_closure_max_residual_deg = 2.0;
    // robust kernel on the loop edges of the pose graph: Cauchy with this scale, in units of the edge's sigmas (a loop
    // edge s sigmas off keeps the weight 1 / (1 + s^2 / scale^2)); 0 = plain least squares
    double loop_closure_kernel_scale = 0.0;
    // cross-floor loops: each segment also tries earlier segments between loop_closure_max_vertical_offset_m and this far
    // above or below it (in a stairwell they see the same shaft), as one more edge; accepted with an overlap of at least
    // loop_closure_cross_floor_min_fitness and a correction of at most 1 m and loop_closure_cross_floor_max_angle_deg.
    // Such an edge is weighted by what its registration determines (the NDT normal matrix at its result, mapped into the
    // edge's relative pose and scaled so that its best-determined translation gets loop_closure_edge_sigma_m), without the
    // height: a round shaft ties the floors horizontally and in tilt, but neither in height nor in the turn about its axis,
    // while stairs and walls in two directions tie the turn too. 0 = off
    double loop_closure_cross_floor_max_vertical_m = 0.0;
    double loop_closure_cross_floor_min_fitness = 0.3;
    double loop_closure_cross_floor_max_angle_deg = 1.5;
    // a cross-floor pair needs only this time gap (there is always a staircase between two floors, so even a short gap
    // spans the drift of the way down; the same-floor path rule would exclude the view of the shaft from the floor
    // just above), and is registered on the footprint both segments share (the shaft), searched within this range
    double loop_closure_cross_floor_min_time_gap_s = 5.0;
    double loop_closure_cross_floor_search_m = 1.0;
    // a cross-floor pair is accepted only if the region both segments share constrains every direction: smallest over
    // largest eigenvalue of sum(n n^T) over its planar 0.2 m cells (a single wall such as a facade gives ~0, a round shaft
    // with floor and ceiling much more); 0 = no check
    double loop_closure_cross_floor_min_information_ratio = 0.0;
    // diagnostics: cross-floor pairs touching these segments are not tried ("8-15,45-57": segment index ranges, inclusive)
    std::string loop_closure_cross_floor_exclude_segments;
    // diagnostics: "x y r" accepts a cross-floor edge only if the centre of its shared region lies within r m of (x, y)
    std::string loop_closure_cross_floor_only_near;

    // online gyro bias estimation (3 extra unknowns per window, carried across windows as a prior)
    bool use_gyro_bias_estimation = true;
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

    // accelerometer as a soft measurement inside the window optimization: the change of world-frame velocity between
    // nodes ~imu_acceleration_span_s apart must match the measured acceleration R * f - g minus an offset e;
    // e (3 unknowns per window, absorbs tilt and accelerometer bias errors) is carried across windows as a prior.
    // The prediction stays constant velocity. Usually combined with looser lidar_odometry_motion_model_x/y/z_1_sigma_m.
    bool use_imu_acceleration_term = false;
    double imu_acceleration_sigma_mps2 = 0.5;                 // accuracy of the measured acceleration
    double imu_acceleration_span_s = 0.01;                    // node spacing of the velocity-change terms
    double imu_acceleration_offset_initial_sigma_mps2 = 0.5;  // prior sigma of e in the first window
    double imu_acceleration_offset_rw_mps2_sqrt_s = 0.05;     // e prior sigma grows by rw * sqrt(window duration)
    // IMU samples the trajectory velocities of the gravity calibration are averaged over (observed_acceleration_offset);
    // counted in samples like threshold_nr_poses, so set per sensor for its IMU rate (20 = 20 ms at 1 kHz)
    int imu_acceleration_offset_span_nr_samples = 20;
    // use the accelerometer term only where the LiDAR geometry does not constrain the translation in some direction
    // (smallest / largest eigenvalue of sum(n * n^T) over the surface normals the points are matched to, below
    // imu_degenerate_information_ratio; decided from the previous window),
    // for at most imu_degenerate_max_duration_s per such stretch; outside, e is calibrated from the LiDAR trajectory
    bool imu_acceleration_only_when_degenerate = false;
    double imu_degenerate_information_ratio = 0.05;
    double imu_degenerate_max_duration_s = 15.0;
    double imu_degenerate_translation_sigma_m = 3e-4; // lidar_odometry_motion_model_x/y/z_1_sigma_m while the term is active
    // continuous weight instead of the on/off gate: with imu_degenerate_weight_ratio_full above imu_degenerate_information_ratio,
    // the term's weight is 1 at or below imu_degenerate_information_ratio, 0 at or above imu_degenerate_weight_ratio_full and
    // linear in between, from the median ratio of the last imu_degenerate_smoothing_windows windows; the accelerometer sigma is
    // divided by sqrt(weight) and the translation sigma moves from the normal one towards imu_degenerate_translation_sigma_m.
    // 0 = the on/off gate (previous window's ratio below imu_degenerate_information_ratio)
    double imu_degenerate_weight_ratio_full = 0.0;
    int imu_degenerate_smoothing_windows = 5;
    // keep the acceleration offset where it is along the window's weakest LiDAR direction while the term is active (it is not
    // observable there: the LiDAR can't see the motion it would explain)
    bool imu_acceleration_offset_freeze_blind = false;
    // the accelerometer in every window after the first (implies use_imu_acceleration_term; the degenerate gate, its time limit,
    // its loosened translation sigma and the gravity calibration are not used): each term weighted by the sensor's noise,
    // sigma^2 = density^2 / span + floor^2; the window's start and end velocities are unknowns, the start with a prior carried from
    // the previous window's end (its marginal covariance, grown over the gap); a window at rest gets a zero-velocity measurement
    bool imu_always_on = false;
    double imu_accel_noise_density_mps2_sqrt_hz = 0.0013; // the sensor's accelerometer noise density, measured at rest (largest axis); set per sensor in its TOML
    double imu_accel_floor_sigma_mps2 = 0.1;               // walking vibration and unmodelled effects; set per sensor in its TOML
    double imu_velocity_initial_sigma_mps = 1.0;           // velocity prior of the first window with the term, if not at rest
    double imu_zero_velocity_sigma_mps = 0.01;             // velocity measurement of a window at rest
    // degeneracy-aware scan-to-map (solution remapping): along a direction whose share of the window's geometric
    // information (eigenvalue / largest eigenvalue of sum(n * n^T) over the matched surface normals) is below
    // degenerate_projection_ratio, the NDT terms carry no translation information, so the motion model (prediction,
    // velocity continuity, accelerometer) decides there and the map can't drag the pose along a blind passage; between
    // it and degenerate_projection_ratio_full the NDT information is scaled linearly. degenerate_projection_ratio_full = 0: off
    double degenerate_projection_ratio = 0.0;
    double degenerate_projection_ratio_full = 0.0;

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

    bool use_remove_imu_bias_from_first_stationary_scan = true;
    // start-up gyro bias: the per-axis median of the first N IMU samples (set N for the sensor's IMU rate, about 1 s of
    // data); rejected as movement (bias 0) when the summed per-axis variance of those samples is above the gate
    int startup_gyro_bias_nr_samples = 1000;
    double startup_gyro_bias_max_variance_deg2_s2 = 5.0;

    //ablation study
    bool ablation_study_use_planarity = false;
    bool ablation_study_use_norm = false;
    bool ablation_study_use_hierarchical_rgd = true;
    bool ablation_study_use_view_point_and_normal_vectors = true;
    bool save_index_pose = false;
};

// folders of one run; current_output_dir and working_directory_preview are the [paths] section of the TOML
struct RunPaths
{
    std::string current_output_dir = "";
    std::string working_directory_preview = ""; // <input folder>/preview
    std::string working_directory_cache = "";   // <input folder>/cache: per-window point caches written by step 1
};

// GUI-only settings, stored in the [misc] section of the TOML
struct GuiPreferences
{
    float clear_color[4] = {0.45f, 0.55f, 0.60f, 1.00f}; // background RGBA
};

inline Eigen::Vector3d get_lidar_imu_lever_arm(const OdometryConfig &params)
{
    return Eigen::Vector3d(params.lidar_imu_lever_arm_x_m, params.lidar_imu_lever_arm_y_m, params.lidar_imu_lever_arm_z_m);
}

#endif
