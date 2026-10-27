#include "run_odometry.h"
#include "ndt_map.h"
#include "odometry_state.h"

#include "angle_units.h"
#include "holdout.h"
#include "imu_kinematics.h"
#include "optimizer_options.h"
#include "phase_timing.h"
#include "robust_lidar_odometry.h"
#include "scan_to_map.h"
#include "window_steps.h"
#include <point_placement.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace
{
    // settings of the window loop, fixed for a run
    struct OdometryRunSetup
    {
        MotionModelRotationOptions rotation_options; // bias_prior_* are set per window
        ScanToMapOptions scan_to_map;
        MotionModelOptions motion_model;
        double bias_rw_rad = 0.0;
        double bias_sigma0_rad = 0.0;
        bool have_acc = false;     // raw accelerometer and AHRS orientation of every node
        bool use_imu_pred = false; // IMU-aided translation prediction
        bool use_zupt = false;     // zero-velocity detection
        double gravity_norm = 1.0; // accelerometer reading of gravity [g]
        bool use_acc_term = false; // accelerometer term in the optimization
        bool acc_gated = false;    // ... only in LiDAR-degenerate stretches
        bool always_on = false;    // ... in every window after the first, with velocity states (imu_always_on)
        double acc_offset_sigma0 = 0.0;
        bool use_acc_prior = false;               // velocity-continuity prior
        bool uses_previous_velocity = false;      // the acc prior, IMU translation or the accelerometer term need it
        PosePredictionOptions prediction;         // how the initial guess of a window is predicted
    };

    // log_stage_timing: where the window loop spends its time, summed over all windows
    struct OdometryTiming
    {
        ScanToMapTiming lm;      // inside optimize_lidar_odometry
        PhaseTime point_load;    // reading the window's points from the PointStore
        PhaseTime robust;        // run_robust_odometry (use_robust_and_accurate_lidar_odometry)
        PhaseTime lm_iterations; // run_lm_iterations as a whole
        PhaseTime preview;       // send_preview
        PhaseTime add_to_map;    // add_window_to_map (update_rgd, sliding-map rebuild)
        PhaseTime window_total;  // one window, from loading its points to adding it to the map
        std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); // the breakdown is relative to the time since
    };

    // what the window loop carries from one window to the next
    struct WindowLoopCarry
    {
        GyroBiasState bias;                                     // the previous window's estimate, the prior of the next one
        Eigen::Vector3d acc_gravity = Eigen::Vector3d::Zero();  // gravity in the odometry frame [accelerometer units]
        Eigen::Matrix3d acc_offset_cov = Eigen::Matrix3d::Zero(); // prior of the next window's acceleration offset
        bool previous_window_degenerate = false;                // decides about the accelerometer term in the next window
        double acc_weight = 0.0;                                // continuous weight of the term in the next window (imu_degenerate_weight_ratio_full)
        std::vector<double> ratio_history;                      // the last windows' information ratios (for the median)
        Eigen::Vector3d weak_direction = Eigen::Vector3d::Zero(); // the last window's weakest LiDAR direction
        double degenerate_time_s = 0.0;                         // time spent in the current degenerate stretch
        double acc_distance = 0.0;                              // trajectory length since the last sliding-window reset [m]
        std::vector<Point3Di> points_global;                    // map points since the last reset
        int stationary_windows = 0;
        int turn_windows = 0;
        int map_rebuilds = 0;                                   // sliding_window_trajectory_length_threshold reached
        int acc_term_windows = 0;
        int rest_windows = 0;                                        // imu_always_on: windows at rest
        bool velocity_valid = false;                                 // imu_always_on: the previous window estimated its end velocity
        Eigen::Vector3d velocity_mean = Eigen::Vector3d::Zero();     // ... the prior of the next window's start velocity
        Eigen::Matrix3d velocity_cov = Eigen::Matrix3d::Identity();
        double velocity_time_s = 0.0;                                // ... at the previous window's last node
        bool first_window_at_rest = false;
        std::vector<IterationConvergence> convergence; // per window, with log_iteration_convergence
        OdometryTiming timing;                         // with log_stage_timing
    };

    OdometryRunSetup prepare_run(const std::vector<Window> &windows, const OdometryConfig &params, Logger &logger)
    {
        OdometryRunSetup s;
        s.rotation_options = get_motion_model_rotation_options(params);
        s.scan_to_map = get_scan_to_map_options(params);
        s.motion_model = get_motion_model_options(params);
        if (s.rotation_options.estimate_bias && params.use_robust_and_accurate_lidar_odometry)
        {
            log_info(logger) << "gyro bias estimation is not supported with use_robust_and_accurate_lidar_odometry, disabled for this run";
            s.rotation_options.estimate_bias = false;
        }
        s.bias_rw_rad = params.gyro_bias_rw_deg_s_sqrt_s * DEG_TO_RAD;
        s.bias_sigma0_rad = params.gyro_bias_initial_sigma_deg_s * DEG_TO_RAD;

        // IMU-aided translation prediction: raw accelerometer and AHRS orientation of every node are needed
        s.have_acc = !windows.empty() && windows[0].imu_acc.size() == windows[0].poses.size() &&
                     windows[0].imu_angles.size() == windows[0].poses.size();
        s.use_imu_pred = params.use_imu_translation_prediction && s.have_acc;
        s.use_zupt = params.use_zero_velocity_detection && s.have_acc;
        if ((params.use_imu_translation_prediction || params.use_zero_velocity_detection) && !s.have_acc)
            log_info(logger) << "no accelerometer data per node: IMU translation prediction / zero-velocity detection disabled";
        // accelerometer reading of gravity [g]: mean magnitude over the first window (the recording starts at rest)
        if (s.have_acc)
        {
            double sum = 0.0;
            for (const auto &a : windows[0].imu_acc)
                sum += a.norm();
            s.gravity_norm = sum / double(windows[0].imu_acc.size());
        }

        s.use_acc_term = (params.use_imu_acceleration_term || params.imu_always_on) && s.have_acc;
        s.always_on = params.imu_always_on && s.have_acc;
        s.acc_offset_sigma0 = params.imu_acceleration_offset_initial_sigma_mps2;
        // gating to LiDAR-degenerate stretches: degeneracy of the previous window, time spent in the current stretch
        s.acc_gated = s.use_acc_term && params.imu_acceleration_only_when_degenerate && !s.always_on;

        s.use_acc_prior = params.lidar_odometry_motion_model_acc_1_sigma_mps2 > 0.0;
        const bool imu_translation = s.use_imu_pred || s.use_zupt;
        s.uses_previous_velocity = s.use_acc_prior || imu_translation || s.use_acc_term;
        s.prediction.use_imu_pred = s.use_imu_pred;
        s.prediction.use_zupt = s.use_zupt || s.always_on; // imu_always_on: a window at rest is also predicted without motion
        s.prediction.constant_velocity = s.use_acc_prior || s.use_acc_term;
        s.prediction.gravity_norm = s.gravity_norm;
        s.prediction.acc_smoothing_half_width_s = 0.5 * params.imu_acc_smoothing_s;
        s.prediction.zupt_acc_threshold_mps2 = params.zero_velocity_acc_threshold_mps2;
        s.prediction.zupt_gyro_threshold_rad_s = params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD;
        return s;
    }

    // the carried state before the first window (window 0 already placed)
    WindowLoopCarry start_carry(const OdometryRunSetup &setup, const std::vector<Window> &windows)
    {
        WindowLoopCarry c;
        c.bias.covariance = Eigen::Matrix3d::Identity() * (setup.bias_sigma0_rad * setup.bias_sigma0_rad);
        // accelerometer term: gravity in the odometry frame [accelerometer units], taken from the first window
        // (the recording starts at rest); the acceleration offset e is folded into it after every window
        if (setup.use_acc_term)
        {
            for (size_t k = 0; k < windows[0].imu_acc.size(); k++)
                c.acc_gravity += rotation_of(windows[0].poses[k]) * windows[0].imu_acc[k];
            c.acc_gravity /= double(windows[0].imu_acc.size());
        }
        c.acc_offset_cov = Eigen::Matrix3d::Identity() * (setup.acc_offset_sigma0 * setup.acc_offset_sigma0);
        return c;
    }

    // the accelerometer term of one window and the motion-model options that depend on it
    struct AccTerm
    {
        MotionModelTranslationImuOptions options; // enabled only in windows that use the term
        MotionModelOptions motion_model;          // the window's translation sigmas (loosened while the term is active)
        Eigen::Matrix3d offset_cov;               // acceleration-offset covariance, updated by the optimization
        double window_duration_s = 0.0;
    };

    // accelerometer term (from the second window on, like the bias): offset prior grown by its random walk;
    // with gating, only in a degenerate stretch that is not too long yet (updates carry.degenerate_time_s)
    AccTerm prepare_acc_term(const Window &window, bool after_first_window, double prev_velocity_age_s,
                             const OdometryRunSetup &setup, WindowLoopCarry &carry, const OdometryConfig &params)
    {
        AccTerm t;
        t.offset_cov = carry.acc_offset_cov;
        t.window_duration_s = window.timestamps.size() > 1
                                  ? window.timestamps.back().first - window.timestamps.front().first
                                  : 0.0;
        bool acc_term_active = setup.use_acc_term && after_first_window;
        if (setup.acc_gated)
        {
            // a new degenerate stretch starts with the calibrated gravity and a fresh offset prior
            if (carry.previous_window_degenerate && carry.degenerate_time_s == 0.0)
                carry.acc_offset_cov = Eigen::Matrix3d::Identity() * (setup.acc_offset_sigma0 * setup.acc_offset_sigma0);
            carry.degenerate_time_s = carry.previous_window_degenerate ? carry.degenerate_time_s + t.window_duration_s : 0.0;
            acc_term_active = acc_term_active && carry.previous_window_degenerate && carry.degenerate_time_s <= params.imu_degenerate_max_duration_s;
        }
        double sigma_x_m = params.lidar_odometry_motion_model_x_1_sigma_m;
        double sigma_y_m = params.lidar_odometry_motion_model_y_1_sigma_m;
        double sigma_z_m = params.lidar_odometry_motion_model_z_1_sigma_m;
        const bool weighted = params.imu_degenerate_weight_ratio_full > params.imu_degenerate_information_ratio;
        if (setup.acc_gated && acc_term_active)
        {
            if (weighted)
            {
                const double w = carry.acc_weight;
                sigma_x_m += w * (params.imu_degenerate_translation_sigma_m - sigma_x_m);
                sigma_y_m += w * (params.imu_degenerate_translation_sigma_m - sigma_y_m);
                sigma_z_m += w * (params.imu_degenerate_translation_sigma_m - sigma_z_m);
            }
            else
                sigma_x_m = sigma_y_m = sigma_z_m = params.imu_degenerate_translation_sigma_m;
        }
        t.motion_model = setup.motion_model;
        t.motion_model.x_sigma_m = sigma_x_m;
        t.motion_model.y_sigma_m = sigma_y_m;
        t.motion_model.z_sigma_m = sigma_z_m;

        if (acc_term_active && window.imu_acc.size() == window.poses.size())
        {
            const double window_duration = t.window_duration_s;
            const double rw = params.imu_acceleration_offset_rw_mps2_sqrt_s;
            t.options.enabled = true;
            t.options.acc = &window.imu_acc;
            t.options.gravity = carry.acc_gravity;
            t.options.to_mps2 = 9.80665 / carry.acc_gravity.norm();
            t.options.sigma_mps2 = params.imu_acceleration_sigma_mps2;
            if (setup.acc_gated && weighted)
                t.options.sigma_mps2 /= std::sqrt(std::max(carry.acc_weight, 1e-6));
            t.options.span_s = params.imu_acceleration_span_s;
            t.options.prev_velocity_age_s = prev_velocity_age_s;
            t.options.offset_prior_cov = carry.acc_offset_cov + Eigen::Matrix3d::Identity() * (rw * rw * window_duration);
            if (params.imu_acceleration_offset_freeze_blind && carry.weak_direction.squaredNorm() > 0.5)
            {
                // the offset's spread along the weakest LiDAR direction shrunk to 1 %: it stays where it was there
                const Eigen::Matrix3d A = Eigen::Matrix3d::Identity() - 0.99 * carry.weak_direction * carry.weak_direction.transpose();
                t.options.offset_prior_cov = A * t.options.offset_prior_cov * A.transpose();
            }
            t.offset_cov = t.options.offset_prior_cov;
            if (setup.always_on)
            {
                t.options.always_on = true;
                t.options.noise_density_mps2_sqrt_hz = params.imu_accel_noise_density_mps2_sqrt_hz;
                t.options.floor_sigma_mps2 = params.imu_accel_floor_sigma_mps2;
                t.options.zero_velocity_sigma_mps = params.imu_zero_velocity_sigma_mps;
            }
        }
        return t;
    }

    // imu_always_on: is the window at rest? The zero-velocity detection of predict_window_poses on the window's guess
    bool window_at_rest(const Window &window, const OdometryRunSetup &setup, const OdometryConfig &params)
    {
        if (window.imu_acc.size() != window.poses.size() || window.imu_angles.size() != window.poses.size() ||
            window.timestamps.size() != window.poses.size())
            return false;
        constexpr double zupt_smoothing_half_width_s = 0.025; // as predict_window_poses
        return is_window_stationary(window.poses, window.timestamps,
                                    world_acceleration(window.poses, window.timestamps, window.imu_acc, window.imu_angles, setup.gravity_norm,
                                                       zupt_smoothing_half_width_s),
                                    params.zero_velocity_acc_threshold_mps2, params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD);
    }

    // imu_always_on: the start velocity prior (the previous window's end velocity, its covariance grown over the gap; in the first
    // window with the term zero if the recording starts at rest, else the previous window's velocity with imu_velocity_initial_sigma_mps)
    // and the window's velocity unknowns started from it
    void prepare_velocity_state(Window &window, const PreviousVelocity &previous, bool at_rest, const OdometryRunSetup &setup,
                                const WindowLoopCarry &carry, AccTerm &t, const OdometryConfig &params)
    {
        if (!setup.always_on || !t.options.enabled)
            return;
        const double initial = params.imu_velocity_initial_sigma_mps, rest = params.imu_zero_velocity_sigma_mps;
        Eigen::Vector3d mean = previous.use ? previous.velocity : Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * (initial * initial);
        if (carry.velocity_valid && carry.velocity_mean.norm() <= params.max_prior_velocity_mps)
        {
            const double gap = std::max(window.timestamps.front().first - carry.velocity_time_s, 0.0);
            const double density = params.imu_accel_noise_density_mps2_sqrt_hz, floor = params.imu_accel_floor_sigma_mps2;
            mean = carry.velocity_mean;
            cov = carry.velocity_cov + Eigen::Matrix3d::Identity() * (density * density * gap + floor * floor * gap * gap);
        }
        else if (!carry.velocity_valid && carry.first_window_at_rest)
        {
            mean.setZero();
            cov = Eigen::Matrix3d::Identity() * (rest * rest);
        }
        t.options.velocity_prior_mean = mean;
        t.options.velocity_prior_cov = cov;
        t.options.at_rest = at_rest;
        window.velocity.start = mean;
        window.velocity.end = mean;
    }

    // after the window: its degeneracy decides about the term in the next window, gravity is calibrated outside
    // degenerate stretches, and an estimated offset is folded into the gravity vector
    void update_acc_term_after_window(const Window &window, int i, const AccTerm &t, const Eigen::Matrix3d &lidar_translation_information,
                                      const Eigen::Vector3d &acc_offset, const OdometryRunSetup &setup, WindowLoopCarry &carry,
                                      const OdometryConfig &params, Logger &logger)
    {
        constexpr double gravity_calibration_time_constant_s = 2.0;
        if (setup.acc_gated)
        {
            // degeneracy of this window decides about the term in the next one
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
            const double ratio = eig.eigenvalues().maxCoeff() > 0.0 ? eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() : 0.0;
            const Eigen::Vector3d weak = eig.eigenvectors().col(0);
            carry.weak_direction = weak;
            const bool weighted = params.imu_degenerate_weight_ratio_full > params.imu_degenerate_information_ratio;
            if (weighted)
            {
                // the median of the last windows' ratios, so one window's noise doesn't flip the term
                carry.ratio_history.push_back(ratio);
                const size_t keep = static_cast<size_t>(std::max(1, params.imu_degenerate_smoothing_windows));
                if (carry.ratio_history.size() > keep)
                    carry.ratio_history.erase(carry.ratio_history.begin());
                std::vector<double> sorted = carry.ratio_history;
                std::sort(sorted.begin(), sorted.end());
                const double median = sorted[sorted.size() / 2];
                const double r0 = params.imu_degenerate_information_ratio, r1 = params.imu_degenerate_weight_ratio_full;
                carry.acc_weight = std::clamp((r1 - median) / (r1 - r0), 0.0, 1.0);
                carry.previous_window_degenerate = carry.acc_weight > 0.0;
            }
            else
                carry.previous_window_degenerate = ratio < params.imu_degenerate_information_ratio;
            log_info(logger) << "lidar translation information window " << i + 1 << ": ratio " << std::fixed << std::setprecision(4) << ratio
                             << " weakest direction " << std::setprecision(2) << weak.x() << " " << weak.y() << " " << weak.z()
                             << (t.options.enabled ? " | accelerometer term active" : "")
                             << (weighted ? " | next weight " : "") << (weighted ? std::to_string(carry.acc_weight).substr(0, 4) : std::string());

            // outside degenerate stretches: calibrate the gravity vector from the LiDAR-constrained trajectory
            Eigen::Vector3d observed;
            if (!t.options.enabled && !carry.previous_window_degenerate &&
                observed_acceleration_offset(window.poses, window.timestamps,
                                             window.imu_acc, carry.acc_gravity, 9.80665 / carry.acc_gravity.norm(), params.handoff_tail_nodes,
                                             params.imu_acceleration_offset_span_nr_samples, observed))
            {
                const double alpha = 1.0 - std::exp(-t.window_duration_s / gravity_calibration_time_constant_s);
                carry.acc_gravity += alpha * observed / (9.80665 / carry.acc_gravity.norm());
            }
        }

        if (t.options.enabled && t.options.always_on)
        {
            carry.velocity_valid = true;
            carry.velocity_mean = window.velocity.end;
            carry.velocity_cov = window.velocity.end_covariance;
            carry.velocity_time_s = window.timestamps.back().first;
        }
        if (t.options.enabled)
        {
            carry.acc_term_windows++;
            // fold the estimated offset into the gravity vector; its uncertainty carries over as the next prior
            carry.acc_gravity += acc_offset / t.options.to_mps2;
            carry.acc_offset_cov = t.offset_cov;
            {
                const Eigen::Vector3d s = t.offset_cov.diagonal().cwiseMax(0.0).cwiseSqrt();
                log_info(logger) << "acceleration offset [m/s^2] window " << i + 1 << ": " << std::fixed << std::setprecision(3)
                                 << acc_offset.x() << " " << acc_offset.y() << " " << acc_offset.z() << " (1-sigma "
                                 << s.x() << " " << s.y() << " " << s.z() << ")";
            }
        }
    }

    // use_robust_and_accurate_lidar_odometry: optimize_sf2 in spherical coordinates, then optimize_rigid_sf against the map
    void run_robust_odometry(Window &window, const std::vector<Point3Di> &intermediate_points, OdometryState &state,
                             const OdometryConfig &params, Logger &logger)
    {
        auto tr = window.poses;
        auto trmm = window.motion_model_poses;

        auto firstm = tr[0];

        for (auto &t : tr)
            t.translation() -= firstm.translation();

        for (auto &t : trmm)
            t.translation() -= firstm.translation();

        NdtGridParameters rgd_params_sc;

        rgd_params_sc.resolution_X = params.distance_bucket;
        rgd_params_sc.resolution_Y = params.polar_angle_deg;
        rgd_params_sc.resolution_Z = params.azimutal_angle_deg;

        std::vector<Point3Di> points_local_sf;
        std::vector<Point3Di> points_local;

        for (int ii = 0; ii < intermediate_points.size(); ii++)
        {
            double r_l = intermediate_points[ii].point.norm();

            if (r_l > 0.5 && intermediate_points[ii].index_pose != -1 && r_l < params.max_distance_lidar_rigid_sf)
            {
                double polar_angle_deg_l = atan2(intermediate_points[ii].point.y(), intermediate_points[ii].point.x()) * RAD_TO_DEG;
                double azimutal_angle_deg_l = acos(intermediate_points[ii].point.z() / r_l) * RAD_TO_DEG;

                points_local.push_back(intermediate_points[ii]);

                Point3Di p_sl = intermediate_points[ii];
                p_sl.point.x() = r_l;
                p_sl.point.y() = polar_angle_deg_l;
                p_sl.point.z() = azimutal_angle_deg_l;

                points_local_sf.push_back(p_sl);
            }
        }
        log_info(logger) << "optimize_sf2";

        double wx = 1000000;
        double wy = 1000000;
        double wz = 1000000;
        double angle_sigma_rad = 0.1 * DEG_TO_RAD;
        double wom = 1.0 / (angle_sigma_rad * angle_sigma_rad);
        double wfi = 1.0 / (angle_sigma_rad * angle_sigma_rad);
        double wka = 1.0 / (angle_sigma_rad * angle_sigma_rad);

        for (int iter = 0; iter < params.robust_and_accurate_lidar_odometry_iterations; iter++)
            optimize_sf2(points_local, points_local_sf, tr, trmm, rgd_params_sc, true, wx, wy, wz, wom, wfi, wka);

        for (auto &t : tr)
            t.translation() += firstm.translation();

        for (auto &t : trmm)
            t.translation() += firstm.translation();

        window.poses = tr;
        window.motion_model_poses = tr;

        optimize_rigid_sf(intermediate_points,
                          window.poses,
                          window.motion_model_poses,
                          state.buckets_indoor,
                          params.distance_bucket_rigid_sf,
                          params.polar_angle_deg_rigid_sf,
                          params.azimutal_angle_deg_rigid_sf,
                          params.robust_and_accurate_lidar_odometry_rigid_sf_iterations,
                          params.max_distance_lidar_rigid_sf,
                          true,
                          params.rgd_sf_sigma_x_m,
                          params.rgd_sf_sigma_y_m,
                          params.rgd_sf_sigma_z_m,
                          params.rgd_sf_sigma_om_deg,
                          params.rgd_sf_sigma_fi_deg,
                          params.rgd_sf_sigma_ka_deg);
    }

    struct WindowOptimization
    {
        int last_iteration = 0; // index of the last Levenberg-Marquardt iteration
        double delta = 100000.0; // sum of the parameter updates of that iteration
        IterationConvergence convergence; // only with log_iteration_convergence
    };

    // limits of log_iteration_convergence: poses count as settled within these of the last result; the fast-mode threshold
    constexpr double convergence_settle_position_m = 1e-4;
    constexpr double convergence_settle_rotation_deg = 0.001;
    constexpr double convergence_fast_delta = 1e-2;

    // Levenberg-Marquardt iterations of optimize_lidar_odometry: until the update falls below convergence_delta_threshold,
    // nr_iter iterations ran, or the window took longer than real_time_threshold_seconds; the damping grows every 10
    // iterations and after a step that was not applied (next_lm_factor)
    WindowOptimization run_lm_iterations(std::vector<Point3Di> &intermediate_points, Window &window, OdometryState &state,
                                         const OdometryRunSetup &setup, AccTerm &acc_term, const MotionModelRotationOptions &window_rotation_options,
                                         const VelocityPrior &velocity_prior, Eigen::Vector3d &acc_offset, Eigen::Matrix3d &lidar_translation_information,
                                         const OdometryConfig &params, bool debugMsg, Logger &logger, ScanToMapTiming *timing,
                                         ScanToMapWorkspace &workspace)
    {
        double delta = 100000.0;
        double lm_factor = params.lm_factor_initial;

        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();

        int iter_end = 0;

        // log_iteration_convergence: the poses and delta after each iteration (only read, the iterations are unchanged)
        std::vector<std::vector<Eigen::Affine3d>> poses_after;
        std::vector<double> deltas;

        for (int iter = 0; iter < params.nr_iter; iter++)
        {
            iter_end = iter;
            delta = 100000.0;
            const bool applied = optimize_lidar_odometry(intermediate_points, window, state.buckets_indoor, state.buckets_outdoor,
                                    setup.scan_to_map, acc_term.motion_model, window_rotation_options, acc_term.options,
                                    velocity_prior, lm_factor, delta, acc_offset, acc_term.offset_cov, logger,
                                    setup.acc_gated || params.log_window_diagnostics ? &lidar_translation_information : nullptr, timing, &workspace);
            if (params.log_iteration_convergence)
            {
                poses_after.push_back(window.poses);
                deltas.push_back(delta);
            }
            if (delta < params.convergence_delta_threshold)
                break;

            const double next_factor = next_lm_factor(lm_factor, iter, applied);
            if (debugMsg && next_factor != lm_factor)
                log_debug(logger) << "\nlm_factor " << lm_factor << ", delta " << std::setprecision(10) << delta << "\n";
            lm_factor = next_factor;

            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;

            if (elapsed_seconds.count() > params.real_time_threshold_seconds)
                break;
        }

        WindowOptimization result;
        result.last_iteration = iter_end;
        result.delta = delta;
        if (params.log_iteration_convergence)
            result.convergence = convergence_of(poses_after, deltas, convergence_settle_position_m,
                                                convergence_settle_rotation_deg * DEG_TO_RAD, convergence_fast_delta);
        return result;
    }

    void log_window_result(int i, size_t n_windows, const WindowOptimization &optimization, double acc_distance, double elapsed_s,
                           const OdometryConfig &params, Logger &logger)
    {
        auto msg = log_info(logger);
        msg << "finished at iteration " << optimization.last_iteration + 1 << "/" << params.nr_iter;
        msg << " optimizing window " << i + 1 << "/" << n_windows
            << " with acc_distance " << std::fixed << std::setprecision(2) << acc_distance << "[m] in "
            << std::fixed << std::setprecision(2) << elapsed_s
            << "[s], delta ";
        if (optimization.delta >= params.convergence_delta_threshold)
            msg << std::setprecision(10) << optimization.delta << "!!!";
        else
            msg << "< " << std::setprecision(1) << std::scientific << params.convergence_delta_threshold;
    }

    // log_window_diagnostics: what one window did, to see a window that stalls or leaps. All displacements are first node to last
    // node in the world frame: the guess before the iterations (prediction), the result, and what the previous window's final
    // velocity would give over the same time. time_s counts from the first window's first node.
    void log_window_diagnostics(int i, const std::vector<Window> &windows, const std::vector<Eigen::Affine3d> &guess, const WindowOptimization &optimization,
                                const PreviousVelocity &previous, const AccTerm &acc_term, const Eigen::Matrix3d &lidar_translation_information,
                                const OdometryConfig &params, Logger &logger)
    {
        if (!params.log_window_diagnostics)
            return;
        const Window &window = windows[i];
        const double time_s = window.timestamps.front().first - windows[0].timestamps.front().first;
        const double dt = window.timestamps.back().first - window.timestamps.front().first;
        const Eigen::Vector3d guess_move = guess.back().translation() - guess.front().translation();
        const Eigen::Vector3d result_move = window.poses.back().translation() - window.poses.front().translation();
        const Eigen::Vector3d velocity_move = previous.velocity * dt;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
        const double ratio = eig.eigenvalues().maxCoeff() > 0.0 ? eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() : 0.0;
        const auto v = [](const Eigen::Vector3d &x) { std::ostringstream o; o << std::fixed << std::setprecision(3) << x.x() << " " << x.y() << " " << x.z(); return o.str(); };
        log_info(logger) << "window diagnostics " << i + 1 << ": t " << std::fixed << std::setprecision(2) << time_s << " s, dt " << dt << " s, iterations "
                         << optimization.last_iteration + 1 << ", delta " << std::scientific << std::setprecision(2) << optimization.delta
                         << std::fixed << ", start " << v(window.poses.front().translation()) << ", guess move " << v(guess_move)
                         << ", result move " << v(result_move) << ", previous-velocity move " << (previous.use ? v(velocity_move) : std::string("none"))
                         << ", result-guess " << v(result_move - guess_move) << ", info ratio " << std::setprecision(3) << ratio << ", weak " << v(eig.eigenvectors().col(0))
                         << ", acc term " << (acc_term.options.enabled ? "on" : "off");
        if (params.imu_always_on)
            log_info(logger) << "window diagnostics " << i + 1 << ": velocity start " << v(window.velocity.start) << ", end " << v(window.velocity.end)
                             << ", end sigma " << std::setprecision(3) << std::sqrt(window.velocity.end_covariance.trace() / 3.0) << " m/s";
    }

    // log_window_diagnostics, windows with an information ratio below 0.45: what holds the window along its weakest direction w.
    //  - prior: the start velocity's jump against its prior (the previous window's end velocity), its chi^2, and the prior's
    //    sigma along w;
    //  - stiffness against a velocity offset along w (node positions moved by d (t - t0) w): the NDT cost's second derivative
    //    (central differences, d = 0.01 m/s), against the velocity prior's 2 w^T C^-1 w and the consecutive-node motion
    //    model's; the accelerometer terms don't see such an offset;
    //  - profile: the NDT cost with the whole window moved along w by -2 .. +2 m in 5 cm steps, and its local minima.
    void log_weak_axis_diagnostics(int i, const Window &window, const std::vector<Point3Di> &intermediate_points, const OdometryState &state,
                                   const ScanToMapOptions &scan_to_map, const AccTerm &acc_term, const Eigen::Matrix3d &lidar_translation_information,
                                   const OdometryConfig &params, Logger &logger)
    {
        if (!params.log_window_diagnostics || window.poses.size() < 2)
            return;
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
        const double largest = eig.eigenvalues().maxCoeff();
        const double ratio = largest > 0.0 ? eig.eigenvalues().minCoeff() / largest : 0.0;
        if (!(largest > 0.0) || ratio >= 0.45)
            return;
        const Eigen::Vector3d w = eig.eigenvectors().col(0);
        const auto cost_at = [&](const std::vector<Eigen::Affine3d> &poses)
        { return ndt_window_cost(intermediate_points, poses, state.buckets_indoor, state.buckets_outdoor, scan_to_map); };
        const double c0 = cost_at(window.poses);

        // velocity prior
        const Eigen::Matrix3d C = acc_term.options.velocity_prior_cov;
        const Eigen::Vector3d jump = window.velocity.start - acc_term.options.velocity_prior_mean;
        const Eigen::Matrix3d Ci = C.inverse();
        const double chi2 = jump.dot(Ci * jump);
        const double prior_sigma_w = std::sqrt(std::max(w.dot(C * w), 0.0));

        // stiffness against a velocity offset along w
        const double t0 = window.timestamps.front().first, d = 0.01;
        const auto stretched = [&](double dv)
        {
            std::vector<Eigen::Affine3d> poses = window.poses;
            for (size_t k = 0; k < poses.size(); k++)
                poses[k].translation() += dv * (window.timestamps[k].first - t0) * w;
            return poses;
        };
        const double ndt_stiffness = (cost_at(stretched(d)) + cost_at(stretched(-d)) - 2.0 * c0) / (d * d);
        const double prior_stiffness = 2.0 * w.dot(Ci * w);
        const MotionModelOptions &mm = acc_term.motion_model;
        double motion_stiffness = 0.0;
        if (mm.x_sigma_m > 0.0 && mm.y_sigma_m > 0.0 && mm.z_sigma_m > 0.0)
            for (size_t k = 1; k < window.poses.size(); k++)
            {
                const double dt = window.timestamps[k].first - window.timestamps[k - 1].first;
                const Eigen::Vector3d local = window.poses[k - 1].linear().transpose() * w;
                motion_stiffness += 2.0 * dt * dt *
                                    (local.x() * local.x() / (mm.x_sigma_m * mm.x_sigma_m) + local.y() * local.y() / (mm.y_sigma_m * mm.y_sigma_m) +
                                     local.z() * local.z() / (mm.z_sigma_m * mm.z_sigma_m));
            }

        // cost profile along w
        std::vector<double> profile;
        for (int k = -40; k <= 40; k++)
        {
            std::vector<Eigen::Affine3d> poses = window.poses;
            for (auto &p : poses)
                p.translation() += 0.05 * k * w;
            profile.push_back(cost_at(poses));
        }
        auto line = log_info(logger);
        line << "window diagnostics " << i + 1 << ": weak axis " << std::fixed << std::setprecision(2) << w.x() << " " << w.y() << " " << w.z()
             << " (ratio " << std::setprecision(3) << ratio << "); start-velocity jump " << std::setprecision(3) << jump.x() << " " << jump.y() << " "
             << jump.z() << " m/s, chi2 " << std::setprecision(1) << chi2 << ", prior sigma along it " << std::setprecision(4) << prior_sigma_w
             << " m/s; stiffness against a velocity offset along it [cost/(m/s)^2]: ndt " << std::scientific << std::setprecision(2) << ndt_stiffness
             << ", velocity prior " << prior_stiffness << ", motion model " << motion_stiffness << std::fixed << "; ndt cost " << std::setprecision(1) << c0
             << ", profile minima [m: cost - cost(0)]:";
        for (int k = 0; k < static_cast<int>(profile.size()); k++)
        {
            const bool left = k == 0 || profile[k] < profile[k - 1];
            const bool right = k + 1 == static_cast<int>(profile.size()) || profile[k] <= profile[k + 1];
            if (left && right)
                line << " " << std::setprecision(2) << 0.05 * (k - 40) << ":" << std::setprecision(1) << profile[k] - c0;
        }
    }

    // log_window_diagnostics: what holds the window's heading.
    //  - yaw correction: the rotation about the world z axis from the guess (the prediction before the iterations) to the
    //    result, at the first and the last node [deg]; also the last node's heading [deg];
    //  - yaw stiffness: the NDT cost's second derivative against a rotation of the whole window about the world z axis
    //    through the mean node position (central differences, 0.1 deg) [cost/deg^2].
    void log_heading_diagnostics(int i, const Window &window, const std::vector<Eigen::Affine3d> &guess, const std::vector<Point3Di> &intermediate_points,
                                 const OdometryState &state, const ScanToMapOptions &scan_to_map, const OdometryConfig &params, Logger &logger)
    {
        if (!params.log_window_diagnostics || window.poses.empty() || guess.size() != window.poses.size())
            return;
        const auto yaw_between = [](const Eigen::Matrix3d &from, const Eigen::Matrix3d &to)
        {
            const Eigen::AngleAxisd aa(to * from.transpose());
            return aa.angle() * aa.axis().z() * RAD_TO_DEG;
        };
        const double yaw_first = yaw_between(guess.front().linear(), window.poses.front().linear());
        const double yaw_last = yaw_between(guess.back().linear(), window.poses.back().linear());
        const Eigen::Matrix3d &r = window.poses.back().linear();
        const double heading = std::atan2(r(1, 0), r(0, 0)) * RAD_TO_DEG;

        Eigen::Vector3d centre = Eigen::Vector3d::Zero();
        for (const auto &p : window.poses)
            centre += p.translation();
        centre /= static_cast<double>(window.poses.size());
        const double step_deg = 0.1;
        const auto rotated = [&](double deg)
        {
            const Eigen::Matrix3d rz = Eigen::AngleAxisd(deg * DEG_TO_RAD, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            std::vector<Eigen::Affine3d> poses = window.poses;
            for (auto &p : poses)
            {
                p.linear() = rz * p.linear();
                p.translation() = rz * (p.translation() - centre) + centre;
            }
            return poses;
        };
        const auto cost = [&](const std::vector<Eigen::Affine3d> &poses)
        { return ndt_window_cost(intermediate_points, poses, state.buckets_indoor, state.buckets_outdoor, scan_to_map); };
        const double stiffness = (cost(rotated(step_deg)) + cost(rotated(-step_deg)) - 2.0 * cost(window.poses)) / (step_deg * step_deg);
        log_info(logger) << "window diagnostics " << i + 1 << ": heading " << std::fixed << std::setprecision(3) << heading << " deg, yaw correction first "
                         << yaw_first << " last " << yaw_last << " deg; yaw stiffness " << std::scientific << std::setprecision(3) << stiffness
                         << " cost/deg^2";
    }

    // log_window_diagnostics: how the accelerometer fits the optimized trajectory. Anchor nodes ~50 ms apart; per triple the
    // change of the trajectory's velocity is compared with the mean of R f - g over the same time, as an acceleration
    // residual (IMU minus LiDAR) [m/s^2]. Logged: its mean in the world and in the body frame (at the middle anchor), the rms
    // around the mean, the longest IMU sample spacing, and whether the window rests. The world mean's horizontal part is
    // what a tilt of |h| / g would leak; a body-frame mean that stays put from window to window is a bias.
    void log_accelerometer_fit(int i, const Window &window, const Eigen::Matrix3d &lidar_translation_information, const AccTerm &acc_term,
                               const OdometryRunSetup &setup, const WindowLoopCarry &carry, const OdometryConfig &params, Logger &logger)
    {
        const int n = static_cast<int>(window.poses.size());
        if (!params.log_window_diagnostics || !setup.have_acc || carry.acc_gravity.squaredNorm() == 0.0 ||
            window.timestamps.size() != window.poses.size() || window.imu_acc.size() != window.poses.size() || n < 3)
            return;
        constexpr double span_s = 0.05;
        const double to_mps2 = 9.80665 / carry.acc_gravity.norm();
        std::vector<Eigen::Vector3d> a(n);
        for (int k = 0; k < n; k++)
            a[k] = (rotation_of(window.poses[k]) * window.imu_acc[k] - carry.acc_gravity) * to_mps2;
        std::vector<int> anchors{0};
        for (int k = 1; k < n; k++)
            if (window.timestamps[k].first - window.timestamps[anchors.back()].first >= span_s)
                anchors.push_back(k);
        std::vector<Eigen::Vector3d> world, body;
        for (size_t j = 1; j + 1 < anchors.size(); j++)
        {
            const int A = anchors[j - 1], B = anchors[j], C = anchors[j + 1];
            const double tA = window.timestamps[A].first, tB = window.timestamps[B].first, tC = window.timestamps[C].first;
            const double m1 = 0.5 * (tA + tB), m2 = 0.5 * (tB + tC), tau = m2 - m1;
            const Eigen::Vector3d dv = (window.poses[C].translation() - window.poses[B].translation()) / (tC - tB) -
                                       (window.poses[B].translation() - window.poses[A].translation()) / (tB - tA);
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            double weight = 0.0;
            for (int k = A; k < C; k++)
            {
                const double s0 = std::max(window.timestamps[k].first, m1), s1 = std::min(window.timestamps[k + 1].first, m2);
                if (s1 > s0)
                {
                    sum += a[k] * (s1 - s0);
                    weight += s1 - s0;
                }
            }
            if (weight <= 0.0 || tau <= 0.0)
                continue;
            world.push_back(sum / weight - dv / tau);
            body.push_back(rotation_of(window.poses[B]).conjugate() * world.back());
        }
        if (world.empty())
            return;
        Eigen::Vector3d world_mean = Eigen::Vector3d::Zero(), body_mean = Eigen::Vector3d::Zero();
        for (size_t k = 0; k < world.size(); k++)
        {
            world_mean += world[k] / double(world.size());
            body_mean += body[k] / double(world.size());
        }
        double rest = 0.0;
        for (const auto &w : world)
            rest += (w - world_mean).squaredNorm() / double(world.size());
        double max_gap = 0.0;
        for (int k = 1; k < n; k++)
            max_gap = std::max(max_gap, window.timestamps[k].first - window.timestamps[k - 1].first);
        constexpr double zupt_smoothing_half_width_s = 0.025; // as predict_window_poses
        const bool at_rest = is_window_stationary(window.poses, window.timestamps,
                                                  world_acceleration(window.poses, window.timestamps, window.imu_acc, window.imu_angles,
                                                                     setup.gravity_norm, zupt_smoothing_half_width_s),
                                                  params.zero_velocity_acc_threshold_mps2, params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD);
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
        const double ratio = eig.eigenvalues().maxCoeff() > 0.0 ? eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() : 0.0;
        const auto v = [](const Eigen::Vector3d &x) { std::ostringstream o; o << std::fixed << std::setprecision(4) << x.x() << " " << x.y() << " " << x.z(); return o.str(); };
        log_info(logger) << "accelerometer fit " << i + 1 << ": ratio " << std::fixed << std::setprecision(3) << ratio << ", acc term "
                         << (acc_term.options.enabled ? 1 : 0) << ", triples " << world.size() << ", world mean " << v(world_mean)
                         << ", body mean " << v(body_mean) << ", rms " << std::setprecision(4) << std::sqrt(rest) << ", max gap ms "
                         << std::setprecision(2) << max_gap * 1000.0 << ", rest " << (at_rest ? 1 : 0);
    }

    // log_iteration_convergence: after how many iterations the window's poses settled, and where fast mode would have stopped
    void log_iteration_convergence(int i, const OdometryConfig &params, const WindowOptimization &optimization, WindowLoopCarry &carry, Logger &logger)
    {
        if (!params.log_iteration_convergence)
            return;
        const IterationConvergence &c = optimization.convergence;
        carry.convergence.push_back(c);
        auto msg = log_info(logger);
        msg << "convergence window " << i + 1 << ": settled after " << c.settled_after << "/" << c.iterations << " iterations ("
            << convergence_settle_position_m * 1000.0 << " mm / " << convergence_settle_rotation_deg << " deg), delta < "
            << convergence_fast_delta << " after ";
        if (c.fast_stop_after > 0)
            msg << c.fast_stop_after << ", error then " << std::fixed << std::setprecision(4) << c.fast_stop_position_error_m * 1000.0
                << " mm / " << c.fast_stop_rotation_error_rad * RAD_TO_DEG << " deg";
        else
            msg << "never";
        if (c.iterations < params.nr_iter)
            msg << " (stopped at " << c.iterations << ")";
    }

    // after the window: log the estimated gyro bias (and a clamp), and carry it as the next window's prior
    void update_gyro_bias_after_window(const Window &window, int i, const OdometryRunSetup &setup, WindowLoopCarry &carry,
                                       const OdometryConfig &params, Logger &logger)
    {
        if (setup.rotation_options.estimate_bias)
        {
            const Eigen::Vector3d b_deg = window.gyro_bias.mean * RAD_TO_DEG;
            const Eigen::Vector3d s_deg = window.gyro_bias.covariance.diagonal().cwiseMax(0.0).cwiseSqrt() * RAD_TO_DEG;
            log_info(logger) << "gyro bias [deg/s] window " << i + 1 << ": " << std::fixed << std::setprecision(3)
                             << b_deg.x() << " " << b_deg.y() << " " << b_deg.z() << " (1-sigma "
                             << s_deg.x() << " " << s_deg.y() << " " << s_deg.z() << ")";
            if ((window.gyro_bias.mean.cwiseAbs().array() >= setup.rotation_options.bias_max_rad_s * (1.0 - 1e-9)).any())
                log_info(logger) << "WARNING: gyro bias clamped to +-" << params.gyro_bias_max_deg_s << " deg/s in window " << i + 1 << "\n";

            carry.bias = window.gyro_bias;
        }
    }

    // the reference map before the first window: the initial points, placed with m_g
    void seed_map(OdometryState &state, const OdometryConfig &params)
    {
        auto pp = state.initial_points;
        remove_held_out(pp, params.holdout_block_s);
        for (int i = 0; i < pp.size(); i++)
            pp[i].point = state.m_g * pp[i].point;

        update_rgd(params.in_out_params_indoor, state.buckets_indoor, pp, state.m_g.translation(), params.use_exact_ndt_bucket_statistics);

        if (params.ablation_study_use_hierarchical_rgd){
            update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, pp, state.m_g.translation(), params.use_exact_ndt_bucket_statistics);
        }
    }

    // preview of window i's points, placed with its optimized poses
    // (window is not const: points_to_vector takes the poses by non-const reference)
    void send_preview(int i, const std::vector<Point3Di> &intermediate_points, Window &window, const OdometryConfig &params, PreviewSink &preview)
    {
        std::vector<Eigen::Vector3d> global_pointcloud;
        std::vector<unsigned short> intensity;
        std::vector<double> timestamps;
        points_to_vector(
            intermediate_points, window.poses,
            0, nullptr, global_pointcloud, intensity, timestamps, false, params.save_index_pose);
        preview.window_cloud(i, global_pointcloud, intensity, timestamps);
    }

    // adds the window's travelled distance to carry.acc_distance; if it is NaN, the window's poses are restored,
    // ts_failure is set and false is returned (the run is cancelled)
    bool add_window_distance(Window &window, const std::vector<Eigen::Affine3d> &poses_before_optimization, WindowLoopCarry &carry,
                             double &ts_failure, Logger &logger)
    {
        auto acc_distance_tmp = carry.acc_distance;
        carry.acc_distance += ((window.poses[0].inverse()) *
                         window.poses[window.poses.size() - 1])
                            .translation()
                            .norm();

        if (!(carry.acc_distance == carry.acc_distance)) // NaN check
        {
            window.poses = poses_before_optimization;
            log_info(logger) << "CHALLENGING DATA OCCURED!!!";
            carry.acc_distance = acc_distance_tmp;
            log_info(logger) << "please split data set into subsets";
            ts_failure = window.timestamps[0].first;
            log_info(logger) << "calculations canceled for TIMESTAMP: " << (long long int)window.timestamps[0].first;
            return false;
        }
        return true;
    }

    // the window's points go into the reference map; after sliding_window_trajectory_length_threshold metres the map
    // is rebuilt from the newer half of the points since the last reset (decimated). Returns whether it was rebuilt.
    bool add_window_to_map(const std::vector<Point3Di> &intermediate_points, const Window &window, WindowLoopCarry &carry,
                           OdometryState &state, const OdometryConfig &params)
    {
        for (int j = 0; j < intermediate_points.size(); j++)
        {
            Point3Di pp = intermediate_points[j];
            pp.point = window.poses[intermediate_points[j].index_pose] * pp.point;
            carry.points_global.push_back(pp);
        }

        const bool rebuild = carry.acc_distance > params.sliding_window_trajectory_length_threshold;
        if (rebuild)
        {
            carry.map_rebuilds++;
            if (state.reference_points.size() == 0)
            {
                state.buckets_indoor.clear();
                state.buckets_outdoor.clear();
            }

            std::vector<Point3Di> points_global_new;
            points_global_new.reserve(carry.points_global.size() / 2 + 1);
            for (int k = carry.points_global.size() / 2; k < carry.points_global.size(); k++)
                points_global_new.emplace_back(carry.points_global[k]);

            carry.acc_distance = 0;
            carry.points_global = points_global_new;

            // decimate (decimate() returns the reduced cloud; its result used to be discarded)
            if (params.decimation > 0){
                carry.points_global = decimate(carry.points_global, params.decimation, params.decimation, params.decimation);
            }

            update_rgd(params.in_out_params_indoor, state.buckets_indoor, carry.points_global, window.poses[0].translation(), params.use_exact_ndt_bucket_statistics);
            if (params.ablation_study_use_hierarchical_rgd)
            {
                update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, carry.points_global, window.poses[0].translation(), params.use_exact_ndt_bucket_statistics);
            }
        }
        else
        {
            std::vector<Point3Di> pg;
            for (int j = 0; j < intermediate_points.size(); j++)
            {
                Point3Di pp = intermediate_points[j];
                pp.point = window.poses[intermediate_points[j].index_pose] * pp.point;
                pg.push_back(pp);
            }
            update_rgd(params.in_out_params_indoor, state.buckets_indoor, pg, window.poses[0].translation(), params.use_exact_ndt_bucket_statistics);
            if (params.ablation_study_use_hierarchical_rgd)
            {
                update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, pg, window.poses[0].translation(), params.use_exact_ndt_bucket_statistics);
            }
        }
        return rebuild;
    }

    // nearest-rank percentile (0..100) of values, which must not be empty
    template <typename T>
    T percentile(std::vector<T> values, double pct)
    {
        std::sort(values.begin(), values.end());
        const size_t rank = static_cast<size_t>(std::ceil(pct / 100.0 * values.size()));
        return values[std::clamp<size_t>(rank, 1, values.size()) - 1];
    }

    // log_iteration_convergence: the distribution over all windows
    void log_convergence_summary(const std::vector<IterationConvergence> &convergence, Logger &logger)
    {
        std::vector<int> settled, fast;
        std::vector<double> fast_error_mm;
        for (const IterationConvergence &c : convergence)
        {
            settled.push_back(c.settled_after);
            if (c.fast_stop_after > 0)
            {
                fast.push_back(c.fast_stop_after);
                fast_error_mm.push_back(c.fast_stop_position_error_m * 1000.0);
            }
        }
        auto msg = log_info(logger);
        msg << "iteration convergence over " << convergence.size() << " windows: settled after median " << percentile(settled, 50.0)
            << ", p90 " << percentile(settled, 90.0) << ", max " << percentile(settled, 100.0);
        if (fast.empty())
        {
            msg << "; delta never below " << convergence_fast_delta;
            return;
        }
        msg << "; delta < " << convergence_fast_delta << " after median " << percentile(fast, 50.0) << ", p90 " << percentile(fast, 90.0)
            << " (in " << fast.size() << " windows); error at that point median " << std::fixed << std::setprecision(4)
            << percentile(fast_error_mm, 50.0) << " mm, p90 " << percentile(fast_error_mm, 90.0) << " mm, max "
            << percentile(fast_error_mm, 100.0) << " mm";
    }

    // log_stage_timing: one line per phase, with its share of the odometry time
    void log_phase_time(Logger &logger, const char *name, const PhaseTime &t, double total_s)
    {
        auto msg = log_info(logger);
        msg << "timing   " << std::left << std::setw(20) << name << std::right << std::fixed << std::setprecision(1) << std::setw(8) << t.seconds
            << " s " << std::setw(5) << (total_s > 0.0 ? 100.0 * t.seconds / total_s : 0.0) << " %";
        if (t.calls > 0)
            msg << " " << std::setw(8) << t.calls << " calls " << std::setprecision(3) << std::setw(9) << 1000.0 * t.seconds / t.calls << " ms";
    }

    void log_odometry_timing(const OdometryTiming &t, double total_s, size_t n_windows, Logger &logger)
    {
        log_info(logger) << "timing odometry breakdown over " << n_windows << " windows, " << std::fixed << std::setprecision(1) << total_s << " s:";
        const ScanToMapTiming &lm = t.lm;
        log_phase_time(logger, "lm setup", lm.setup, total_s);
        log_phase_time(logger, "lm ndt_indoor", lm.ndt_indoor, total_s);
        log_phase_time(logger, "lm ndt_outdoor", lm.ndt_outdoor, total_s);
        log_phase_time(logger, "lm motion_terms", lm.motion_terms, total_s);
        log_phase_time(logger, "lm assembly", lm.assembly, total_s);
        log_phase_time(logger, "lm covariance_solve", lm.covariance_solve, total_s);
        log_phase_time(logger, "lm solve", lm.solve, total_s);
        log_phase_time(logger, "lm update", lm.update, total_s);
        const double lm_named = lm.setup.seconds + lm.ndt_indoor.seconds + lm.ndt_outdoor.seconds + lm.motion_terms.seconds +
                                lm.assembly.seconds + lm.covariance_solve.seconds + lm.solve.seconds + lm.update.seconds;
        // the rest of run_lm_iterations: the loop around the calls and the end of each call, where its matrices are freed
        log_phase_time(logger, "lm other", PhaseTime{t.lm_iterations.seconds - lm_named, 0}, total_s);
        log_phase_time(logger, "lm_iterations", t.lm_iterations, total_s);
        log_phase_time(logger, "point_load", t.point_load, total_s);
        log_phase_time(logger, "robust", t.robust, total_s);
        log_phase_time(logger, "preview", t.preview, total_s);
        log_phase_time(logger, "add_to_map", t.add_to_map, total_s);
        log_phase_time(logger, "window_total", t.window_total, total_s);
        const double named = t.lm_iterations.seconds + t.point_load.seconds + t.robust.seconds + t.preview.seconds + t.add_to_map.seconds;
        log_phase_time(logger, "other", PhaseTime{total_s - named, 0}, total_s);
    }

    // log_stage_timing: how often the Cholesky analyses had to be repeated
    void log_solver_analyses(const ScanToMapWorkspace &workspace, Logger &logger)
    {
        log_info(logger) << "timing   cholesky analyses: covariance " << workspace.covariance.analyses() << " in " << workspace.covariance.calls()
                         << " solves, damped " << workspace.damped.analyses() << " in " << workspace.damped.calls() << " solves";
    }

    // after the last window: the motion-model poses follow the result, the trajectory length and the summary
    void finish_run(std::vector<Window> &windows, const std::chrono::time_point<std::chrono::system_clock> &start,
                    const OdometryRunSetup &setup, const WindowLoopCarry &carry, OdometryState &state,
                    const OdometryConfig &params, const ScanToMapWorkspace &workspace, Logger &logger)
    {
        for (int i = 0; i < windows.size(); i++)
            windows[i].motion_model_poses = windows[i].poses;

        const auto end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);

        std::tm local_tm = *std::localtime(&end_time);
        log_info(logger) << "finished computation at " << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                         << ", elapsed time: " << std::fixed << std::setprecision(2) << elapsed_seconds.count() << "s";

        state.total_length_of_calculated_trajectory = 0;
        for (int i = 1; i < windows.size(); i++)
            state.total_length_of_calculated_trajectory += (windows[i].poses[0].translation() - windows[i - 1].poses[0].translation()).norm();

        log_info(logger) << "total_length_of_calculated_trajectory: " << state.total_length_of_calculated_trajectory << " [m]";
        if (setup.use_zupt)
            log_info(logger) << "zero-velocity detection: " << carry.stationary_windows << " of " << windows.size() << " windows stationary";
        if (setup.use_acc_term)
            log_info(logger) << "accelerometer term: active in " << carry.acc_term_windows << " of " << windows.size() << " windows";
        if (setup.always_on)
            log_info(logger) << "accelerometer always on: " << carry.rest_windows << " of " << windows.size() << " windows at rest";
        if (params.rotation_sigma_inflation_k_turn > 0.0)
            log_info(logger) << "turn windows (gyro trusted more): " << carry.turn_windows << " of " << windows.size();
        log_info(logger) << "silent events: map rebuilds " << carry.map_rebuilds << ", solver steps skipped " << workspace.steps_skipped
                         << ", big-jump nodes (their steps not applied) " << workspace.big_jump_nodes;
        if (params.log_iteration_convergence && !carry.convergence.empty())
            log_convergence_summary(carry.convergence, logger);
        if (params.log_stage_timing)
        {
            log_odometry_timing(carry.timing, std::chrono::duration<double>(std::chrono::steady_clock::now() - carry.timing.start).count(), windows.size(), logger);
            log_solver_analyses(workspace, logger);
        }
    }
} // namespace

bool optimize_windows(std::vector<Window> &windows, const OdometryConfig &params, OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger, ProgressReporter &progress)
{
    if (windows.size() != 0)
    {
        const auto start = std::chrono::system_clock::now();

        place_window_after(windows[0], state.m_g);
        windows[0].motion_model_poses = windows[0].poses;
        seed_map(state, params);

        OdometryRunSetup setup = prepare_run(windows, params, logger);
        WindowLoopCarry carry = start_carry(setup, windows);
        ScanToMapWorkspace workspace; // the Cholesky analyses, kept from window to window

        // log_stage_timing: the timers get a target only when the setting is on
        const auto timed = [&params](PhaseTime &t) { return params.log_stage_timing ? &t : nullptr; };

        for (int i = 0; i < windows.size(); i++)
        {
            PhaseTimer window_timer(timed(carry.timing.window_total));
            std::vector<Point3Di> intermediate_points;
            PhaseTimer point_load_timer(timed(carry.timing.point_load));
            point_store.load(PointSet::WindowIntermediate, i, intermediate_points);
            remove_held_out(intermediate_points, params.holdout_block_s); // also out of the map update below
            point_load_timer.stop();

            progress.checkpoint();

            // final world-frame velocity of the previous window (for the velocity-continuity prior)
            PreviousVelocity previous;
            if (setup.uses_previous_velocity && i > 0)
            {
                constexpr int velocity_span_nodes = 20;
                previous = estimate_previous_velocity(windows[i - 1], windows[i], velocity_span_nodes, params.max_prior_velocity_mps);
                if (previous.dropped)
                    log_info(logger) << "window " << i + 1 << ": previous window velocity " << previous.speed
                                     << " m/s > max_prior_velocity_mps " << params.max_prior_velocity_mps << ": not used as prior";
            }

            // initial guess: continue from the previous window's end
            if ((i > 1 || (setup.uses_previous_velocity && i > 0)) && params.use_motion_from_previous_step)
            {
                const PosePrediction prediction = predict_window_poses(windows[i], windows[i - 1], i > 1 ? &windows[i - 2] : nullptr,
                                                                       previous.velocity, setup.prediction);
                if (prediction.mean_shift_dropped)
                    log_info(logger) << "!!!mean_shift.norm() > 1.0!!!";
                if (prediction.stationary)
                {
                    carry.stationary_windows++;
                    if (debugMsg)
                        log_debug(logger) << "window " << i + 1 << " stationary (zero-velocity detection)\n";
                }
            }

            // gyro bias: prior from the previous window, grown by the bias random walk over this window
            if (setup.rotation_options.estimate_bias)
            {
                const GyroBiasState prior = apply_gyro_bias_prior(windows[i], carry.bias, setup.bias_rw_rad);
                setup.rotation_options.bias_prior_mean = prior.mean;
                setup.rotation_options.bias_prior_cov = prior.covariance;
            }

            // the first window is registered against the initial map, which is built from not yet optimized points
            // (motion during the first threshold_initial_points is not compensated): its misfit must not enter the bias,
            // so the bias is only estimated from the second window on
            MotionModelRotationOptions window_rotation_options = setup.rotation_options;
            window_rotation_options.estimate_bias = setup.rotation_options.estimate_bias && i > 0;

            // tight turns: trust the gyro more (rotation about the vertical from the IMU-based initial guess)
            if (params.rotation_sigma_inflation_k_turn > 0.0 && is_turn_window(windows[i], params.turn_rate_threshold_deg_s * DEG_TO_RAD))
            {
                window_rotation_options.inflation_k = params.rotation_sigma_inflation_k_turn;
                carry.turn_windows++;
            }

            // accelerometer term (from the second window on, like the bias) and the window's translation sigmas
            AccTerm acc_term = prepare_acc_term(windows[i], i > 0, previous.age_s, setup, carry, params);
            if (setup.always_on)
            {
                const bool at_rest = window_at_rest(windows[i], setup, params);
                if (at_rest)
                    carry.rest_windows++;
                if (i == 0)
                    carry.first_window_at_rest = at_rest;
                prepare_velocity_state(windows[i], previous, at_rest, setup, carry, acc_term, params);
                // the guess (and so the motion model) follows the accelerometer from the start velocity prior; a window at rest
                // keeps its motionless guess
                if (acc_term.options.enabled && !at_rest)
                    integrate_window_translations(windows[i], windows[i].velocity.start, carry.acc_gravity, acc_term.options.to_mps2);
            }
            Eigen::Matrix3d lidar_translation_information = Eigen::Matrix3d::Zero();
            Eigen::Vector3d acc_offset = Eigen::Vector3d::Zero();
            const VelocityPrior velocity_prior{previous.use, previous.velocity};

            const auto window_start = std::chrono::system_clock::now();
            const std::vector<Eigen::Affine3d> poses_before_optimization = windows[i].poses;

            if (params.use_robust_and_accurate_lidar_odometry)
            {
                PhaseTimer robust_timer(timed(carry.timing.robust));
                run_robust_odometry(windows[i], intermediate_points, state, params, logger);
            }

            windows[i].motion_model_poses = windows[i].poses;

            PhaseTimer lm_timer(timed(carry.timing.lm_iterations));
            const int steps_skipped_before = workspace.steps_skipped;
            const int big_jump_nodes_before = workspace.big_jump_nodes;
            const WindowOptimization optimization = run_lm_iterations(intermediate_points, windows[i], state, setup, acc_term, window_rotation_options,
                                                                      velocity_prior, acc_offset, lidar_translation_information, params, debugMsg, logger,
                                                                      params.log_stage_timing ? &carry.timing.lm : nullptr, workspace);
            lm_timer.stop();
            if (workspace.steps_skipped > steps_skipped_before)
                log_warning(logger) << "window " << i + 1 << ": " << workspace.steps_skipped - steps_skipped_before
                                    << " solver step(s) not applied (the solve failed)";
            if (workspace.big_jump_nodes > big_jump_nodes_before)
                log_warning(logger) << "window " << i + 1 << ": " << workspace.big_jump_nodes - big_jump_nodes_before
                                    << " node(s) would have moved 1 m or more from their guess: those steps were not applied (big jump)";

            const std::chrono::duration<double> window_seconds = std::chrono::system_clock::now() - window_start;
            log_window_result(i, windows.size(), optimization, carry.acc_distance, window_seconds.count(), params, logger);
            log_iteration_convergence(i, params, optimization, carry, logger);
            log_window_diagnostics(i, windows, poses_before_optimization, optimization, previous, acc_term, lidar_translation_information, params, logger);
            log_accelerometer_fit(i, windows[i], lidar_translation_information, acc_term, setup, carry, params, logger);
            log_weak_axis_diagnostics(i, windows[i], intermediate_points, state, setup.scan_to_map, acc_term, lidar_translation_information, params,
                                      logger);
            log_heading_diagnostics(i, windows[i], poses_before_optimization, intermediate_points, state, setup.scan_to_map, params, logger);

            update_gyro_bias_after_window(windows[i], i, setup, carry, params, logger);
            update_acc_term_after_window(windows[i], i, acc_term, lidar_translation_information, acc_offset, setup, carry, params, logger);

            // window hand-off; the initial guess was corrected with the prior bias, so the tail gets the update estimated in this window
            {
                const Eigen::Vector3d bias_update = setup.rotation_options.estimate_bias
                                                        ? Eigen::Vector3d(windows[i].gyro_bias.mean - setup.rotation_options.bias_prior_mean)
                                                        : Eigen::Vector3d::Zero();
                repropagate_window_tail(windows[i], params.handoff_tail_nodes, bias_update);
            }

            if (i % 100 == 0)
            {
                PhaseTimer preview_timer(timed(carry.timing.preview));
                send_preview(i, intermediate_points, windows[i], params, preview);
            }

            if (!add_window_distance(windows[i], poses_before_optimization, carry, ts_failure, logger))
            {
                // the run stops here but is still saved: the remaining windows continue from this one's restored end
                place_windows_after(windows, i + 1);
                return false;
            }

            // reported only now: from here on the odometry doesn't change this window again (a viewer may read it)
            progress.progress(i + 1, windows.size());

            // the next window continues from the new end of this one; the windows after it are placed in their own turn,
            // since no step reads a window's placement before the window before it is done
            if (i + 1 < windows.size())
                place_window_after(windows[i + 1], windows[i].poses[windows[i].poses.size() - 1]);

            PhaseTimer add_to_map_timer(timed(carry.timing.add_to_map));
            if (add_window_to_map(intermediate_points, windows[i], carry, state, params))
                log_warning(logger) << "window " << i + 1 << ": reference map rebuilt from the newer half of its points "
                                    << "(sliding_window_trajectory_length_threshold " << params.sliding_window_trajectory_length_threshold << " m reached)";
            add_to_map_timer.stop();

            if (i > 1)
            {
                double translation = (windows[i - 1].poses[0].translation() -
                                      windows[i - 2].poses[0].translation())
                                         .norm();
                state.consecutive_distance += translation;
            }
        }

        finish_run(windows, start, setup, carry, state, params, workspace, logger);
    }

    return true;
}
