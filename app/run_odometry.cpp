#include "run_odometry.h"

#include "angle_units.h"
#include "imu_kinematics.h"
#include "optimizer_options.h"
#include "robust_lidar_odometry.h"
#include "scan_to_map.h"
#include <export_laz.h>

bool compute_step_2(std::vector<Window> &windows, const OdometryConfig &params, OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger, ProgressReporter &progress)
{
    if (windows.size() != 0)
    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        double acc_distance = 0.0;
        std::vector<Point3Di> points_global;

        Eigen::Affine3d m_last = state.m_g;
        auto tmp = windows[0].poses;

        windows[0].poses[0] = m_last;
        for (int k = 1; k < tmp.size(); k++)
        {
            Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
            m_last = m_last * m_update;
            windows[0].poses[k] = m_last;
        }
        windows[0].motion_model_poses = windows[0].poses;

        auto pp = state.initial_points;
        for (int i = 0; i < pp.size(); i++)
            pp[i].point = state.m_g * pp[i].point;

        update_rgd(params.in_out_params_indoor, state.buckets_indoor, pp, state.m_g.translation());

        if (params.ablation_study_use_hierarchical_rgd){
            update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, pp, state.m_g.translation());
        }

        MotionModelRotationOptions rotation_options = get_motion_model_rotation_options(params);
        const ScanToMapOptions scan_to_map = get_scan_to_map_options(params);
        const MotionModelOptions motion_model = get_motion_model_options(params);
        if (rotation_options.estimate_bias && params.use_robust_and_accurate_lidar_odometry)
        {
            log_info(logger) << "gyro bias estimation is not supported with use_robust_and_accurate_lidar_odometry, disabled for this run";
            rotation_options.estimate_bias = false;
        }
        const double bias_rw_rad = params.gyro_bias_rw_deg_s_sqrt_s * DEG_TO_RAD;
        const double bias_sigma0_rad = params.gyro_bias_initial_sigma_deg_s * DEG_TO_RAD;
        GyroBiasState carried_bias; // the previous window's estimate, the prior of the next one
        carried_bias.covariance = Eigen::Matrix3d::Identity() * (bias_sigma0_rad * bias_sigma0_rad);

        // IMU-aided translation prediction: raw accelerometer and AHRS orientation of every node are needed
        const bool have_acc = !windows.empty() && windows[0].imu_acc.size() == windows[0].poses.size() &&
                              windows[0].imu_angles.size() == windows[0].poses.size();
        const bool use_imu_pred = params.use_imu_translation_prediction && have_acc;
        const bool use_zupt = params.use_zero_velocity_detection && have_acc;
        if ((params.use_imu_translation_prediction || params.use_zero_velocity_detection) && !have_acc)
            log_info(logger) << "no accelerometer data per node: IMU translation prediction / zero-velocity detection disabled";
        // accelerometer reading of gravity [g]: mean magnitude over the first window (the recording starts at rest)
        double gravity_norm = 1.0;
        if (have_acc)
        {
            double sum = 0.0;
            for (const auto &a : windows[0].imu_acc)
                sum += a.norm();
            gravity_norm = sum / double(windows[0].imu_acc.size());
        }
        int stationary_windows = 0;
        int turn_windows = 0;

        // accelerometer term: gravity in the odometry frame [accelerometer units], taken from the first window
        // (the recording starts at rest); the acceleration offset e is folded into it after every window
        const bool use_acc_term = params.use_imu_acceleration_term && have_acc;
        Eigen::Vector3d acc_gravity = Eigen::Vector3d::Zero();
        if (use_acc_term)
        {
            for (size_t k = 0; k < windows[0].imu_acc.size(); k++)
                acc_gravity += rotation_of(windows[0].poses[k]) * windows[0].imu_acc[k];
            acc_gravity /= double(windows[0].imu_acc.size());
        }
        const double acc_offset_sigma0 = params.imu_acceleration_offset_initial_sigma_mps2;
        Eigen::Matrix3d carried_acc_offset_cov = Eigen::Matrix3d::Identity() * (acc_offset_sigma0 * acc_offset_sigma0);
        // gating to LiDAR-degenerate stretches: degeneracy of the previous window, time spent in the current stretch
        const bool acc_gated = use_acc_term && params.imu_acceleration_only_when_degenerate;
        bool previous_window_degenerate = false;
        double degenerate_time_s = 0.0;
        int acc_term_windows = 0;
        constexpr double gravity_calibration_time_constant_s = 2.0;

        for (int i = 0; i < windows.size(); i++)
        {
            std::vector<Point3Di> intermediate_points;
            point_store.load(PointSet::WindowIntermediate, i, intermediate_points);

            progress.checkpoint();

            // final world-frame velocity of the previous window (for the velocity-continuity prior)
            const bool use_acc_prior = params.lidar_odometry_motion_model_acc_1_sigma_mps2 > 0.0;
            bool use_prev_velocity = false;
            Eigen::Vector3d prev_velocity(0.0, 0.0, 0.0);
            const bool imu_translation = use_imu_pred || use_zupt;
            double prev_velocity_age_s = 0.0;
            if ((use_acc_prior || imu_translation || use_acc_term) && i > 0)
            {
                const auto &prev_tr = windows[i - 1].poses;
                const auto &prev_ts = windows[i - 1].timestamps;
                const int n = prev_tr.size();
                const int k = std::min(20, n - 1);
                if (k > 0 && prev_ts.size() == prev_tr.size())
                {
                    const double dt = prev_ts[n - 1].first - prev_ts[n - 1 - k].first;
                    if (dt > 0.0)
                    {
                        prev_velocity = (prev_tr[n - 1].translation() - prev_tr[n - 1 - k].translation()) / dt;
                        use_prev_velocity = true;
                        const auto &ts_i = windows[i].timestamps;
                        if (!ts_i.empty())
                            prev_velocity_age_s = std::max(ts_i[0].first - 0.5 * (prev_ts[n - 1].first + prev_ts[n - 1 - k].first), 0.0);
                    }
                }

                if (prev_velocity.norm() > params.max_prior_velocity_mps)
                {
                    log_info(logger) << "window " << i + 1 << ": previous window velocity " << prev_velocity.norm()
                                     << " m/s > max_prior_velocity_mps " << params.max_prior_velocity_mps << ": not used as prior";
                    prev_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
                    use_prev_velocity = false;
                }
            }

            Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
            bool stationary = false;
            if ((i > 1 || ((use_acc_prior || imu_translation || use_acc_term) && i > 0)) && params.use_motion_from_previous_step)
            {
                if (i > 1)
                {
                    mean_shift = windows[i - 1].poses[windows[i - 1].poses.size() - 1].translation() -
                                 windows[i - 2].poses[windows[i - 2].poses.size() - 1].translation();
                    mean_shift /= (windows[i - 1].poses.size());
                }

                if (mean_shift.norm() > 1.0)
                {
                    log_info(logger) << "!!!mean_shift.norm() > 1.0!!!";
                    mean_shift = Eigen::Vector3d(0.0, 0.0, 0.0);
                }

                std::vector<Eigen::Affine3d> new_trajectory;
                Eigen::Affine3d current_node = windows[i - 1].poses[windows[i - 1].poses.size() - 1];
                new_trajectory.push_back(current_node);

                for (int tr = 1; tr < windows[i].poses.size(); tr++)
                {
                    auto update = windows[i].poses[tr - 1].inverse() * windows[i].poses[tr];
                    current_node = current_node * update;
                    new_trajectory.push_back(current_node);
                }

                const auto &ts = windows[i].timestamps;
                const auto &acc = windows[i].imu_acc;
                const bool imu_sizes_ok = imu_translation && ts.size() == new_trajectory.size() && acc.size() == new_trajectory.size() &&
                                          windows[i].imu_angles.size() == new_trajectory.size();
                std::vector<Eigen::Vector3d> a;
                if (imu_sizes_ok)
                {
                    a = world_acceleration(new_trajectory, ts, acc, windows[i].imu_angles, gravity_norm, 0.5 * params.imu_acc_smoothing_s);
                    // rest detection on a 50 ms average: the accelerometer noise of a 10 ms average already reaches ~0.3 m/s^2
                    // at rest, while walking accelerations (1-3 m/s^2 at ~2 Hz) survive the longer average
                    constexpr double zupt_smoothing_half_width_s = 0.025;
                    stationary = use_zupt && is_window_stationary(new_trajectory, ts,
                                                                  world_acceleration(new_trajectory, ts, acc, windows[i].imu_angles, gravity_norm, zupt_smoothing_half_width_s),
                                                                  params.zero_velocity_acc_threshold_mps2,
                                                                  params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD);
                }

                if (imu_sizes_ok && (use_imu_pred || stationary))
                {
                    // integrate the accelerometer from the previous window's final velocity;
                    // a window at rest (zero-velocity detection) is predicted without motion
                    Eigen::Vector3d v = stationary ? Eigen::Vector3d::Zero() : prev_velocity;
                    Eigen::Vector3d p = new_trajectory[0].translation();
                    for (size_t tr = 1; tr < new_trajectory.size(); tr++)
                    {
                        const double dt = std::max(ts[tr].first - ts[tr - 1].first, 0.0);
                        const Eigen::Vector3d a_k = (use_imu_pred && !stationary) ? a[tr - 1] : Eigen::Vector3d::Zero();
                        p += v * dt + 0.5 * a_k * dt * dt;
                        v += a_k * dt;
                        new_trajectory[tr].translation() = p;
                    }
                    if (stationary)
                    {
                        stationary_windows++;
                        if (debugMsg)
                            log_debug(logger) << "window " << i + 1 << " stationary (zero-velocity detection)\n";
                    }
                }
                else if ((use_acc_prior || use_acc_term) && ts.size() == new_trajectory.size())
                {
                    // timestamp-aware constant-velocity prediction from the previous window's final velocity
                    for (int tr = 0; tr < new_trajectory.size(); tr++)
                        new_trajectory[tr].translation() += prev_velocity * (ts[tr].first - ts[0].first);
                }
                else
                {
                    for (int tr = 0; tr < new_trajectory.size(); tr++)
                        new_trajectory[tr].translation() += mean_shift * tr;
                }

                windows[i].poses = new_trajectory;
                windows[i].motion_model_poses = new_trajectory;
            }

            // gyro bias: prior from the previous window, grown by the bias random walk over this window
            if (rotation_options.estimate_bias)
            {
                const auto &ts = windows[i].timestamps;
                const double window_duration = ts.size() > 1 ? ts.back().first - ts.front().first : 0.0;
                rotation_options.bias_prior_mean = carried_bias.mean;
                rotation_options.bias_prior_cov = carried_bias.covariance + Eigen::Matrix3d::Identity() * (bias_rw_rad * bias_rw_rad * window_duration);
                windows[i].gyro_bias.mean = carried_bias.mean;
                windows[i].gyro_bias.covariance = rotation_options.bias_prior_cov;

                // remove the carried bias from the IMU rotation increments of the initial guess (translations unchanged)
                remove_gyro_bias_from_rotations(windows[i].poses, ts, carried_bias.mean);
                windows[i].motion_model_poses = windows[i].poses;
            }

            // the first window is registered against the initial map, which is built from not yet optimized points
            // (motion during the first threshold_initial_points is not compensated): its misfit must not enter the bias,
            // so the bias is only estimated from the second window on
            MotionModelRotationOptions window_rotation_options = rotation_options;
            window_rotation_options.estimate_bias = rotation_options.estimate_bias && i > 0;

            // tight turns: trust the gyro more (rotation about the vertical from the IMU-based initial guess)
            if (params.rotation_sigma_inflation_k_turn > 0.0)
            {
                const auto &tr = windows[i].poses;
                const auto &ts = windows[i].timestamps;
                if (tr.size() > 1 && ts.size() == tr.size())
                {
                    double yaw = 0.0;
                    for (size_t k = 1; k < tr.size(); k++)
                    {
                        // world-frame rotation increment, its component about the vertical
                        const Eigen::AngleAxisd aa(Eigen::Matrix3d(tr[k].linear() * tr[k - 1].linear().transpose()));
                        yaw += aa.angle() * aa.axis().z();
                    }
                    const double duration = ts.back().first - ts.front().first;
                    if (duration > 0.0 && std::abs(yaw) / duration > params.turn_rate_threshold_deg_s * DEG_TO_RAD)
                    {
                        window_rotation_options.inflation_k = params.rotation_sigma_inflation_k_turn;
                        turn_windows++;
                    }
                }
            }

            // accelerometer term (from the second window on, like the bias): offset prior grown by its random walk
            MotionModelTranslationImuOptions translation_imu_options;
            Eigen::Matrix3d lidar_translation_information = Eigen::Matrix3d::Zero();
            Eigen::Vector3d acc_offset = Eigen::Vector3d::Zero();
            Eigen::Matrix3d acc_offset_cov = carried_acc_offset_cov;
            const double window_duration_s = windows[i].timestamps.size() > 1
                                                 ? windows[i].timestamps.back().first - windows[i].timestamps.front().first
                                                 : 0.0;
            bool acc_term_active = use_acc_term && i > 0;
            if (acc_gated)
            {
                // a new degenerate stretch starts with the calibrated gravity and a fresh offset prior
                if (previous_window_degenerate && degenerate_time_s == 0.0)
                    carried_acc_offset_cov = Eigen::Matrix3d::Identity() * (acc_offset_sigma0 * acc_offset_sigma0);
                degenerate_time_s = previous_window_degenerate ? degenerate_time_s + window_duration_s : 0.0;
                acc_term_active = acc_term_active && previous_window_degenerate && degenerate_time_s <= params.imu_degenerate_max_duration_s;
            }
            double sigma_x_m = params.lidar_odometry_motion_model_x_1_sigma_m;
            double sigma_y_m = params.lidar_odometry_motion_model_y_1_sigma_m;
            double sigma_z_m = params.lidar_odometry_motion_model_z_1_sigma_m;
            if (acc_gated && acc_term_active)
                sigma_x_m = sigma_y_m = sigma_z_m = params.imu_degenerate_translation_sigma_m;
            MotionModelOptions window_motion_model = motion_model;
            window_motion_model.x_sigma_m = sigma_x_m;
            window_motion_model.y_sigma_m = sigma_y_m;
            window_motion_model.z_sigma_m = sigma_z_m;
            const VelocityPrior velocity_prior{use_prev_velocity, prev_velocity};

            if (acc_term_active && windows[i].imu_acc.size() == windows[i].poses.size())
            {
                const double window_duration = window_duration_s;
                const double rw = params.imu_acceleration_offset_rw_mps2_sqrt_s;
                translation_imu_options.enabled = true;
                translation_imu_options.acc = &windows[i].imu_acc;
                translation_imu_options.gravity = acc_gravity;
                translation_imu_options.to_mps2 = 9.80665 / acc_gravity.norm();
                translation_imu_options.sigma_mps2 = params.imu_acceleration_sigma_mps2;
                translation_imu_options.span_s = params.imu_acceleration_span_s;
                translation_imu_options.prev_velocity_age_s = prev_velocity_age_s;
                translation_imu_options.offset_prior_cov = carried_acc_offset_cov + Eigen::Matrix3d::Identity() * (rw * rw * window_duration);
                acc_offset_cov = translation_imu_options.offset_prior_cov;
            }

            std::chrono::time_point<std::chrono::system_clock> start1, end1;
            start1 = std::chrono::system_clock::now();

            auto tmp_worker_data = windows[i].poses;

            if (params.use_robust_and_accurate_lidar_odometry)
            {
                auto tr = windows[i].poses;
                auto trmm = windows[i].motion_model_poses;

                auto firstm = tr[0];

                for (auto &t : tr)
                    t.translation() -= firstm.translation();

                for (auto &t : trmm)
                    t.translation() -= firstm.translation();

                NDT::GridParameters rgd_params_sc;

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
                    optimize_sf2(points_local, points_local_sf, tr, trmm, rgd_params_sc, params.useMultithread, wx, wy, wz, wom, wfi, wka);

                for (auto &t : tr)
                    t.translation() += firstm.translation();

                for (auto &t : trmm)
                    t.translation() += firstm.translation();

                windows[i].poses = tr;
                windows[i].motion_model_poses = tr;

                optimize_rigid_sf(intermediate_points,
                                  windows[i].poses,
                                  windows[i].motion_model_poses,
                                  state.buckets_indoor,
                                  params.distance_bucket_rigid_sf,
                                  params.polar_angle_deg_rigid_sf,
                                  params.azimutal_angle_deg_rigid_sf,
                                  params.robust_and_accurate_lidar_odometry_rigid_sf_iterations,
                                  params.max_distance_lidar_rigid_sf,
                                  params.useMultithread,
                                  params.rgd_sf_sigma_x_m,
                                  params.rgd_sf_sigma_y_m,
                                  params.rgd_sf_sigma_z_m,
                                  params.rgd_sf_sigma_om_deg,
                                  params.rgd_sf_sigma_fi_deg,
                                  params.rgd_sf_sigma_ka_deg);
            }

            windows[i].motion_model_poses = windows[i].poses;

            double delta = 100000.0;
            double lm_factor = params.lm_factor_initial;

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            int iter_end = 0;

            for (int iter = 0; iter < params.nr_iter; iter++)
            {
                iter_end = iter;
                delta = 100000.0;
                optimize_lidar_odometry(intermediate_points, windows[i], state.buckets_indoor, state.buckets_outdoor,
                                        scan_to_map, window_motion_model, window_rotation_options, translation_imu_options,
                                        velocity_prior, lm_factor, delta, acc_offset, acc_offset_cov, logger,
                                        acc_gated ? &lidar_translation_information : nullptr);
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

            end1 = std::chrono::system_clock::now();

            std::chrono::duration<double> elapsed_seconds1 = end1 - start1;

            {
                auto msg = log_info(logger);
                msg << "finished at iteration " << iter_end + 1 << "/" << params.nr_iter;
                msg << " optimizing window " << i + 1 << "/" << windows.size()
                    << " with acc_distance " << fixed << std::setprecision(2) << acc_distance << "[m] in "
                    << fixed << std::setprecision(2) << elapsed_seconds1.count()
                    << "[s], delta ";
                if (delta >= params.convergence_delta_threshold)
                    msg << std::setprecision(10) << delta << "!!!";
                else
                    msg << "< " << std::setprecision(1) << std::scientific << params.convergence_delta_threshold;
            }

            if (rotation_options.estimate_bias)
            {
                const Eigen::Vector3d b_deg = windows[i].gyro_bias.mean * RAD_TO_DEG;
                const Eigen::Vector3d s_deg = windows[i].gyro_bias.covariance.diagonal().cwiseMax(0.0).cwiseSqrt() * RAD_TO_DEG;
                log_info(logger) << "gyro bias [deg/s] window " << i + 1 << ": " << std::fixed << std::setprecision(3)
                                 << b_deg.x() << " " << b_deg.y() << " " << b_deg.z() << " (1-sigma "
                                 << s_deg.x() << " " << s_deg.y() << " " << s_deg.z() << ")";
                if ((windows[i].gyro_bias.mean.cwiseAbs().array() >= rotation_options.bias_max_rad_s * (1.0 - 1e-9)).any())
                    log_info(logger) << "WARNING: gyro bias clamped to +-" << params.gyro_bias_max_deg_s << " deg/s in window " << i + 1 << "\n";

                carried_bias = windows[i].gyro_bias;
            }

            if (acc_gated)
            {
                // degeneracy of this window decides about the term in the next one
                const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
                const double ratio = eig.eigenvalues().maxCoeff() > 0.0 ? eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() : 0.0;
                previous_window_degenerate = ratio < params.imu_degenerate_information_ratio;
                const Eigen::Vector3d weak = eig.eigenvectors().col(0);
                log_info(logger) << "lidar translation information window " << i + 1 << ": ratio " << std::fixed << std::setprecision(4) << ratio
                                 << " weakest direction " << std::setprecision(2) << weak.x() << " " << weak.y() << " " << weak.z()
                                 << (translation_imu_options.enabled ? " | accelerometer term active" : "");

                // outside degenerate stretches: calibrate the gravity vector from the LiDAR-constrained trajectory
                Eigen::Vector3d observed;
                if (!translation_imu_options.enabled && !previous_window_degenerate &&
                    observed_acceleration_offset(windows[i].poses, windows[i].timestamps,
                                                 windows[i].imu_acc, acc_gravity, 9.80665 / acc_gravity.norm(), params.handoff_tail_nodes, observed))
                {
                    const double alpha = 1.0 - std::exp(-window_duration_s / gravity_calibration_time_constant_s);
                    acc_gravity += alpha * observed / (9.80665 / acc_gravity.norm());
                }
            }

            if (translation_imu_options.enabled)
            {
                acc_term_windows++;
                // fold the estimated offset into the gravity vector; its uncertainty carries over as the next prior
                acc_gravity += acc_offset / translation_imu_options.to_mps2;
                carried_acc_offset_cov = acc_offset_cov;
                {
                    const Eigen::Vector3d s = acc_offset_cov.diagonal().cwiseMax(0.0).cwiseSqrt();
                    log_info(logger) << "acceleration offset [m/s^2] window " << i + 1 << ": " << std::fixed << std::setprecision(3)
                                     << acc_offset.x() << " " << acc_offset.y() << " " << acc_offset.z() << " (1-sigma "
                                     << s.x() << " " << s.y() << " " << s.z() << ")";
                }
            }

            // window hand-off: the last nodes carry almost no points (the last one none), so their pose is
            // re-propagated from the last well-constrained node with the IMU rotation and constant velocity;
            // the next window starts from the re-propagated last node
            {
                auto &tr = windows[i].poses;
                const auto &imu = windows[i].motion_model_poses;
                const auto &ts = windows[i].timestamps;
                const int n = static_cast<int>(tr.size());
                const int m = params.handoff_tail_nodes;
                const int a = n - 1 - m;          // last node kept from the optimization
                constexpr int velocity_nodes = 20; // same span as the prev_velocity estimate
                if (m > 0 && a >= velocity_nodes && ts.size() == tr.size() && imu.size() == tr.size())
                {
                    const double dt_v = ts[a].first - ts[a - velocity_nodes].first;
                    const Eigen::Vector3d v = dt_v > 0.0 ? Eigen::Vector3d((tr[a].translation() - tr[a - velocity_nodes].translation()) / dt_v)
                                                         : Eigen::Vector3d::Zero();
                    // the initial guess was corrected with the prior bias; apply the update estimated in this window
                    const Eigen::Vector3d db = rotation_options.estimate_bias ? Eigen::Vector3d(windows[i].gyro_bias.mean - rotation_options.bias_prior_mean)
                                                                              : Eigen::Vector3d::Zero();
                    Eigen::Quaterniond q = rotation_of(tr[a]);
                    for (int k = a + 1; k < n; k++)
                    {
                        const double dt = std::max(ts[k].first - ts[k - 1].first, 0.0);
                        q = (q * relative_rotation(imu[k - 1], imu[k]) * Eigen::Quaterniond(exp_so3(-db * dt))).normalized();
                        const Eigen::Vector3d t_new = tr[k - 1].translation() + v * dt;
                        tr[k].linear() = q.toRotationMatrix();
                        tr[k].translation() = t_new;
                    }
                }
            }

            progress.progress(i + 1, windows.size());

            // temp save
            if (i % 100 == 0)
            {
                std::vector<Eigen::Vector3d> global_pointcloud;
                std::vector<unsigned short> intensity;
                std::vector<double> timestamps;
                points_to_vector(
                    intermediate_points, windows[i].poses,
                    0, nullptr, global_pointcloud, intensity, timestamps, false, params.save_index_pose);
                preview.window_cloud(i, global_pointcloud, intensity, timestamps);
            }
            auto acc_distance_tmp = acc_distance;
            acc_distance += ((windows[i].poses[0].inverse()) *
                             windows[i].poses[windows[i].poses.size() - 1])
                                .translation()
                                .norm();

            if (!(acc_distance == acc_distance)) // NaN check
            {
                windows[i].poses = tmp_worker_data;
                log_info(logger) << "CHALLENGING DATA OCCURED!!!";
                acc_distance = acc_distance_tmp;
                log_info(logger) << "please split data set into subsets";
                ts_failure = windows[i].timestamps[0].first;
                log_info(logger) << "calculations canceled for TIMESTAMP: " << (long long int)windows[i].timestamps[0].first;
                return false;
            }

            for (int j = i + 1; j < windows.size(); j++)
            {
                Eigen::Affine3d m_last = windows[j - 1].poses[windows[j - 1].poses.size() - 1];
                auto tmp = windows[j].poses;

                windows[j].poses[0] = m_last;

                for (int k = 1; k < tmp.size(); k++)
                {
                    Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
                    m_last = m_last * m_update;
                    windows[j].poses[k] = m_last;
                }
            }

            for (int j = 0; j < intermediate_points.size(); j++)
            {
                Point3Di pp = intermediate_points[j];
                pp.point = windows[i].poses[intermediate_points[j].index_pose] * pp.point;
                points_global.push_back(pp);
            }

            if (acc_distance > params.sliding_window_trajectory_length_threshold)
            {
                if (state.reference_points.size() == 0)
                {
                    state.buckets_indoor.clear();
                    state.buckets_outdoor.clear();
                }

                std::vector<Point3Di> points_global_new;
                points_global_new.reserve(points_global.size() / 2 + 1);
                for (int k = points_global.size() / 2; k < points_global.size(); k++)
                    points_global_new.emplace_back(points_global[k]);

                acc_distance = 0;
                points_global = points_global_new;

                // decimate (decimate() returns the reduced cloud; its result used to be discarded)
                if (params.decimation > 0){
                    points_global = decimate(points_global, params.decimation, params.decimation, params.decimation);
                }

                update_rgd(params.in_out_params_indoor, state.buckets_indoor, points_global, windows[i].poses[0].translation());
                if (params.ablation_study_use_hierarchical_rgd)
                {
                    update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, points_global, windows[i].poses[0].translation());
                }
            }
            else
            {
                std::vector<Point3Di> pg;
                for (int j = 0; j < intermediate_points.size(); j++)
                {
                    Point3Di pp = intermediate_points[j];
                    pp.point = windows[i].poses[intermediate_points[j].index_pose] * pp.point;
                    pg.push_back(pp);
                }
                update_rgd(params.in_out_params_indoor, state.buckets_indoor, pg, windows[i].poses[0].translation());
                if (params.ablation_study_use_hierarchical_rgd)
                {
                    update_rgd(params.in_out_params_outdoor, state.buckets_outdoor, pg, windows[i].poses[0].translation());
                }
            }

            if (i > 1)
            {
                double translation = (windows[i - 1].poses[0].translation() -
                                      windows[i - 2].poses[0].translation())
                                         .norm();
                state.consecutive_distance += translation;
            }
        }

        for (int i = 0; i < windows.size(); i++)
            windows[i].motion_model_poses = windows[i].poses;

        end = std::chrono::system_clock::now();

        std::chrono::duration<double> elapsed_seconds = end - start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);

        std::tm local_tm = *std::localtime(&end_time);
        log_info(logger) << "finished computation at " << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
                         << ", elapsed time: " << std::fixed << std::setprecision(2) << elapsed_seconds.count() << "s";

        state.total_length_of_calculated_trajectory = 0;
        for (int i = 1; i < windows.size(); i++)
            state.total_length_of_calculated_trajectory += (windows[i].poses[0].translation() - windows[i - 1].poses[0].translation()).norm();

        log_info(logger) << "total_length_of_calculated_trajectory: " << state.total_length_of_calculated_trajectory << " [m]";
        if (use_zupt)
            log_info(logger) << "zero-velocity detection: " << stationary_windows << " of " << windows.size() << " windows stationary";
        if (use_acc_term)
            log_info(logger) << "accelerometer term: active in " << acc_term_windows << " of " << windows.size() << " windows";
        if (params.rotation_sigma_inflation_k_turn > 0.0)
            log_info(logger) << "turn windows (gyro trusted more): " << turn_windows << " of " << windows.size();
    }

    return true;
}
