#include "run_odometry.h"
#include "ndt_map.h"
#include "odometry_state.h"

#include "angle_units.h"
#include "imu_kinematics.h"
#include "optimizer_options.h"
#include "phase_timing.h"
#include "robust_lidar_odometry.h"
#include "scan_to_map.h"
#include "window_steps.h"
#include <point_placement.h>

#include <algorithm>
#include <cmath>

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
        double degenerate_time_s = 0.0;                         // time spent in the current degenerate stretch
        double acc_distance = 0.0;                              // trajectory length since the last sliding-window reset [m]
        std::vector<Point3Di> points_global;                    // map points since the last reset
        int stationary_windows = 0;
        int turn_windows = 0;
        int acc_term_windows = 0;
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

        s.use_acc_term = params.use_imu_acceleration_term && s.have_acc;
        s.acc_offset_sigma0 = params.imu_acceleration_offset_initial_sigma_mps2;
        // gating to LiDAR-degenerate stretches: degeneracy of the previous window, time spent in the current stretch
        s.acc_gated = s.use_acc_term && params.imu_acceleration_only_when_degenerate;

        s.use_acc_prior = params.lidar_odometry_motion_model_acc_1_sigma_mps2 > 0.0;
        const bool imu_translation = s.use_imu_pred || s.use_zupt;
        s.uses_previous_velocity = s.use_acc_prior || imu_translation || s.use_acc_term;
        s.prediction.use_imu_pred = s.use_imu_pred;
        s.prediction.use_zupt = s.use_zupt;
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
        if (setup.acc_gated && acc_term_active)
            sigma_x_m = sigma_y_m = sigma_z_m = params.imu_degenerate_translation_sigma_m;
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
            t.options.span_s = params.imu_acceleration_span_s;
            t.options.prev_velocity_age_s = prev_velocity_age_s;
            t.options.offset_prior_cov = carry.acc_offset_cov + Eigen::Matrix3d::Identity() * (rw * rw * window_duration);
            t.offset_cov = t.options.offset_prior_cov;
        }
        return t;
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
            carry.previous_window_degenerate = ratio < params.imu_degenerate_information_ratio;
            const Eigen::Vector3d weak = eig.eigenvectors().col(0);
            log_info(logger) << "lidar translation information window " << i + 1 << ": ratio " << std::fixed << std::setprecision(4) << ratio
                             << " weakest direction " << std::setprecision(2) << weak.x() << " " << weak.y() << " " << weak.z()
                             << (t.options.enabled ? " | accelerometer term active" : "");

            // outside degenerate stretches: calibrate the gravity vector from the LiDAR-constrained trajectory
            Eigen::Vector3d observed;
            if (!t.options.enabled && !carry.previous_window_degenerate &&
                observed_acceleration_offset(window.poses, window.timestamps,
                                             window.imu_acc, carry.acc_gravity, 9.80665 / carry.acc_gravity.norm(), params.handoff_tail_nodes, observed))
            {
                const double alpha = 1.0 - std::exp(-t.window_duration_s / gravity_calibration_time_constant_s);
                carry.acc_gravity += alpha * observed / (9.80665 / carry.acc_gravity.norm());
            }
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
    // nr_iter iterations ran, or the window took longer than real_time_threshold_seconds; the damping grows every 10 iterations
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
            optimize_lidar_odometry(intermediate_points, window, state.buckets_indoor, state.buckets_outdoor,
                                    setup.scan_to_map, acc_term.motion_model, window_rotation_options, acc_term.options,
                                    velocity_prior, lm_factor, delta, acc_offset, acc_term.offset_cov, logger,
                                    setup.acc_gated ? &lidar_translation_information : nullptr, timing, &workspace);
            if (params.log_iteration_convergence)
            {
                poses_after.push_back(window.poses);
                deltas.push_back(delta);
            }
            if (delta < params.convergence_delta_threshold)
                break;

            if (iter % 10 == 0 && iter > 0)
            {
                if (debugMsg)
                    log_debug(logger) << "\nlm_factor " << lm_factor << ", delta " << std::setprecision(10) << delta << "\n";

                lm_factor *= 10.0;
            }

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
    // is rebuilt from the newer half of the points since the last reset (decimated)
    void add_window_to_map(const std::vector<Point3Di> &intermediate_points, const Window &window, WindowLoopCarry &carry,
                           OdometryState &state, const OdometryConfig &params)
    {
        for (int j = 0; j < intermediate_points.size(); j++)
        {
            Point3Di pp = intermediate_points[j];
            pp.point = window.poses[intermediate_points[j].index_pose] * pp.point;
            carry.points_global.push_back(pp);
        }

        if (carry.acc_distance > params.sliding_window_trajectory_length_threshold)
        {
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
        if (params.rotation_sigma_inflation_k_turn > 0.0)
            log_info(logger) << "turn windows (gyro trusted more): " << carry.turn_windows << " of " << windows.size();
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
            const WindowOptimization optimization = run_lm_iterations(intermediate_points, windows[i], state, setup, acc_term, window_rotation_options,
                                                                      velocity_prior, acc_offset, lidar_translation_information, params, debugMsg, logger,
                                                                      params.log_stage_timing ? &carry.timing.lm : nullptr, workspace);
            lm_timer.stop();

            const std::chrono::duration<double> window_seconds = std::chrono::system_clock::now() - window_start;
            log_window_result(i, windows.size(), optimization, carry.acc_distance, window_seconds.count(), params, logger);
            log_iteration_convergence(i, params, optimization, carry, logger);

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
            add_window_to_map(intermediate_points, windows[i], carry, state, params);
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
