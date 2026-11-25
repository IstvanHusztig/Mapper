#include "run_odometry.h"
#include "ndt_map.h"
#include "odometry_state.h"

#include "angle_units.h"
#include "holdout.h"
#include "imu_kinematics.h"
#include "optimizer_options.h"
#include "phase_timing.h"
#include "scan_to_map.h"
#include "window_loop.h"
#include "window_steps.h"
#include "window_problem.h"
#include "window_term_table.h"
#include <point_placement.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

namespace
{
    // first: the run's first window (only its IMU data is read)
    OdometryRunSetup prepare_run(const Window &first, const OdometryConfig &params, Logger &logger)
    {
        OdometryRunSetup s;
        s.rotation_options = get_motion_model_rotation_options(params);
        s.scan_to_map = get_scan_to_map_options(params);
        s.motion_model = get_motion_model_options(params);
        s.bias_rw_rad = params.gyro_bias_rw_deg_s_sqrt_s * DEG_TO_RAD;
        s.bias_sigma0_rad = params.gyro_bias_initial_sigma_deg_s * DEG_TO_RAD;

        // IMU-aided translation prediction: raw accelerometer and AHRS orientation of every node are needed
        s.have_acc = first.imu_acc.size() == first.poses.size() && first.imu_angles.size() == first.poses.size();
        s.use_imu_pred = params.use_imu_translation_prediction && s.have_acc;
        s.use_zupt = params.use_zero_velocity_detection && s.have_acc;
        if ((params.use_imu_translation_prediction || params.use_zero_velocity_detection) && !s.have_acc)
            log_info(logger) << "no accelerometer data per node: IMU translation prediction / zero-velocity detection disabled";
        // accelerometer reading of gravity [g]: mean magnitude over the first window (the recording starts at rest)
        if (s.have_acc)
        {
            double sum = 0.0;
            for (const auto &a : first.imu_acc)
                sum += a.norm();
            s.gravity_norm = sum / double(first.imu_acc.size());
        }

        s.always_on = params.imu_always_on && s.have_acc;
        s.acc_offset_sigma0 = params.imu_acceleration_offset_initial_sigma_mps2;

        s.use_acc_prior = params.lidar_odometry_motion_model_acc_1_sigma_mps2 > 0.0;
        const bool imu_translation = s.use_imu_pred || s.use_zupt;
        s.uses_previous_velocity = s.use_acc_prior || imu_translation || s.always_on;
        s.prediction.use_imu_pred = s.use_imu_pred;
        s.prediction.use_zupt = s.use_zupt || s.always_on; // imu_always_on: a window at rest is also predicted without motion
        s.prediction.constant_velocity = s.use_acc_prior || s.always_on;
        s.prediction.gravity_norm = s.gravity_norm;
        s.prediction.acc_smoothing_half_width_s = 0.5 * params.imu_acc_smoothing_s;
        s.prediction.zupt_acc_threshold_mps2 = params.zero_velocity_acc_threshold_mps2;
        s.prediction.zupt_gyro_threshold_rad_s = params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD;

        // spans counted in IMU samples that don't fit a full window (a setting meant for another IMU rate)
        const int kept_nodes = params.threshold_nr_poses - 1 - params.handoff_tail_nodes;
        if (params.handoff_tail_nodes > 0 && params.handoff_tail_velocity_span_nr_samples > kept_nodes)
            log_warning(logger) << "handoff_tail_velocity_span_nr_samples " << params.handoff_tail_velocity_span_nr_samples << " > "
                                << kept_nodes << " kept nodes per window: the tail is not re-propagated";
        if (s.uses_previous_velocity && params.prior_velocity_span_nr_samples > params.threshold_nr_poses - 1)
            log_warning(logger) << "prior_velocity_span_nr_samples " << params.prior_velocity_span_nr_samples
                                << " > threshold_nr_poses - 1: the previous velocity is measured over the whole window";
        return s;
    }

    // the carried state before the first window (window 0 already placed)
    WindowLoopCarry start_carry(const OdometryRunSetup &setup, const Window &first)
    {
        WindowLoopCarry c;
        c.bias.covariance = Eigen::Matrix3d::Identity() * (setup.bias_sigma0_rad * setup.bias_sigma0_rad);
        // accelerometer term: gravity in the odometry frame [accelerometer units], taken from the first window
        // (the recording starts at rest); the acceleration offset e is folded into it after every window
        if (setup.always_on)
        {
            for (size_t k = 0; k < first.imu_acc.size(); k++)
                c.acc_gravity += rotation_of(first.poses[k]) * first.imu_acc[k];
            c.acc_gravity /= double(first.imu_acc.size());
        }
        c.acc_offset_cov = Eigen::Matrix3d::Identity() * (setup.acc_offset_sigma0 * setup.acc_offset_sigma0);
        return c;
    }

    // accelerometer term (imu_always_on, from the second window on, like the bias): offset prior grown by its random walk
    AccTerm prepare_acc_term(const Window &window, bool after_first_window, const OdometryRunSetup &setup, const WindowLoopCarry &carry,
                             const OdometryConfig &params)
    {
        AccTerm t;
        t.offset_cov = carry.acc_offset_cov;
        t.window_duration_s = window.timestamps.size() > 1
                                  ? window.timestamps.back().first - window.timestamps.front().first
                                  : 0.0;
        if (setup.always_on && after_first_window && window.imu_acc.size() == window.poses.size())
        {
            const double rw = params.imu_acceleration_offset_rw_mps2_sqrt_s;
            t.options.enabled = true;
            t.options.acc = &window.imu_acc;
            t.options.gravity = carry.acc_gravity;
            t.options.to_mps2 = 9.80665 / carry.acc_gravity.norm();
            t.options.span_s = params.imu_acceleration_span_s;
            t.options.offset_prior_cov = random_walk_prior_cov(carry.acc_offset_cov, rw, t.window_duration_s);
            t.offset_cov = t.options.offset_prior_cov;
            t.options.noise_density_mps2_sqrt_hz = params.imu_accel_noise_density_mps2_sqrt_hz;
            t.options.floor_sigma_mps2 = params.imu_accel_floor_sigma_mps2;
            t.options.position_information_scale = params.imu_position_information_scale;
            t.options.zero_velocity_sigma_mps = params.imu_zero_velocity_sigma_mps;
            t.options.edges_follow_state = params.imu_edges_follow_state;
            t.options.joint_covariance = params.imu_carry_joint_covariance;
            t.options.window_sigma_mps2 = params.imu_accel_window_sigma_mps2;
        }
        return t;
    }

    // imu_always_on: is the window at rest? The rest test of predict_window_poses (window_rests) on the window's guess
    bool window_at_rest(const Window &window, const OdometryRunSetup &setup, const OdometryConfig &params)
    {
        if (window.imu_acc.size() != window.poses.size() || window.imu_angles.size() != window.poses.size() ||
            window.timestamps.size() != window.poses.size())
            return false;
        return window_rests(window.poses, window.timestamps, window.imu_acc, window.imu_angles, setup.gravity_norm,
                            params.zero_velocity_acc_threshold_mps2, params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD);
    }

    // imu_always_on: the start velocity prior (the previous window's end velocity, its covariance grown over the gap; in the first
    // window with the term zero if the recording starts at rest, else the previous window's velocity with imu_velocity_initial_sigma_mps)
    // and the window's velocity unknowns started from it
    void prepare_velocity_state(Window &window, const PreviousVelocity &previous, bool at_rest, const OdometryRunSetup &setup,
                                const WindowLoopCarry &carry, AccTerm &t, const OdometryConfig &params)
    {
        if (!t.options.enabled)
            return;
        const double initial = params.imu_velocity_initial_sigma_mps, rest = params.imu_zero_velocity_sigma_mps;
        Eigen::Vector3d mean = previous.use ? previous.velocity : Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * (initial * initial);
        if (carry.velocity_valid && carry.velocity_mean.norm() <= params.max_prior_velocity_mps)
        {
            const double gap = std::max(window.timestamps.front().first - carry.velocity_time_s, 0.0);
            const double density = params.imu_accel_noise_density_mps2_sqrt_hz, floor = params.imu_accel_floor_sigma_mps2;
            mean = carry.velocity_mean;
            // with imu_carry_joint_covariance also the prior's covariance with the offset; it stays zero (the AccTerm's
            // default) where the prior does not come from the previous window
            if (t.options.joint_covariance)
            {
                // imu_carry_joint_covariance: (end velocity, offset) carried together, moved over the gap (the velocity
                // changes by -offset x gap) and grown as below; the helper grows the offset by its random walk from the
                // carried covariance once, as prepare_acc_term did (same function, same arguments: the same value)
                const JointPriorCov p =
                    joint_prior_cov_over_gap(joint_prior_cov(carry.velocity_cov, carry.velocity_offset_cov, carry.acc_offset_cov), density,
                                             floor, gap, params.imu_acceleration_offset_rw_mps2_sqrt_s, t.window_duration_s);
                cov = p.topLeftCorner<3, 3>();
                t.options.velocity_offset_prior_cov = p.topRightCorner<3, 3>();
                t.options.offset_prior_cov = p.bottomRightCorner<3, 3>();
                t.offset_cov = t.options.offset_prior_cov;
            }
            else
                cov = velocity_prior_cov_over_gap(carry.velocity_cov, density, floor, gap);
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

    // after the window: the end velocity becomes the next window's prior, and an estimated offset is folded into the gravity vector
    void update_acc_term_after_window(const Window &window, int i, const AccTerm &t, const Eigen::Vector3d &acc_offset, WindowLoopCarry &carry,
                                      Logger &logger)
    {
        if (t.options.enabled)
        {
            carry.velocity_valid = true;
            carry.velocity_mean = window.velocity.end;
            carry.velocity_cov = window.velocity.end_covariance;
            carry.velocity_time_s = window.timestamps.back().first;
            carry.acc_term_windows++;
            // fold the estimated offset into the gravity vector; its uncertainty carries over as the next prior
            carry.acc_gravity += acc_offset / t.options.to_mps2;
            carry.acc_offset_cov = t.offset_cov;
            // imu_carry_joint_covariance: the solve's cov(end velocity, offset). The offset is folded into the gravity vector, so
            // the next window's offset (prior mean zero) is the error left in the fold, minus that of this window's estimate:
            // its prior error is this window's offset error, and its covariance with the velocity's error is this block as it
            // is (the to_mps2 rescale of the fold, a factor 1 - O(|offset| / 9.8), is neglected)
            if (t.options.joint_covariance)
                carry.velocity_offset_cov = window.velocity.end_offset_covariance;
            {
                const Eigen::Vector3d s = t.offset_cov.diagonal().cwiseMax(0.0).cwiseSqrt();
                log_info(logger) << "acceleration offset [m/s^2] window " << i + 1 << ": " << std::fixed << std::setprecision(3)
                                 << acc_offset.x() << " " << acc_offset.y() << " " << acc_offset.z() << " (1-sigma "
                                 << s.x() << " " << s.y() << " " << s.z() << ")";
            }
        }
    }

    // what the iterations of one window did: the solver's result (iterations, delta, damping, rejected steps, why they
    // stopped), and for the window the cost of its final state (lm_step_check) and the convergence record
    struct WindowOptimization : LmResult
    {
        ScanToMapCost cost;               // lm_step_check: the cost of the final state
        IterationConvergence convergence; // only with log_iteration_convergence
        HonestyInputs honesty;            // the window problem's part of the honesty readings, taken before it goes out of scope
    };

    // limits of log_iteration_convergence: poses count as settled within these of the last result; the fast-mode threshold
    constexpr double convergence_settle_position_m = 1e-4;
    constexpr double convergence_settle_rotation_deg = 0.001;
    constexpr double convergence_fast_delta = 1e-2;

    // The Levenberg-Marquardt iterations of a window: lm_solve (lm_solver.h) on the window's WindowProblem, with the run's
    // settings. forced_last_iteration >= 0 (a replay, window_replay.h): the iterations end after that one, as they did in
    // the run that was captured, and the time limit is not checked, so the result doesn't depend on the computer's speed.
    WindowOptimization run_lm_iterations(std::vector<Point3Di> &intermediate_points, Window &window, OdometryState &state,
                                         const OdometryRunSetup &setup, AccTerm &acc_term, const MotionModelRotationOptions &window_rotation_options,
                                         const WindowNoiseModel &noise, Eigen::Vector3d &acc_offset, Eigen::Matrix3d &lidar_translation_information,
                                         const OdometryConfig &params, bool debugMsg, Logger &logger, ScanToMapTiming *timing,
                                         ScanToMapWorkspace &workspace, int forced_last_iteration)
    {
        const std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

        // log_iteration_convergence: the poses and delta after each iteration (only read, the iterations are unchanged)
        std::vector<std::vector<Eigen::Affine3d>> poses_after;
        std::vector<double> deltas;

        // the window's translation information, in every run (the window verdict's information ratio); it enters no matrix
        Eigen::Matrix3d *translation_information = &lidar_translation_information;
        WindowProblem problem(intermediate_points, window, state.buckets_indoor, state.buckets_outdoor, setup.scan_to_map, setup.motion_model,
                              window_rotation_options, acc_term.options, noise, acc_offset, acc_term.offset_cov, logger,
                              translation_information, timing, workspace);

        LmOptions options;
        options.forced = forced_last_iteration >= 0;
        options.last_iteration = options.forced ? forced_last_iteration : params.nr_iter - 1;
        options.initial_factor = params.lm_factor_initial;
        options.convergence_delta_threshold = params.convergence_delta_threshold;
        options.step_check = params.lm_step_check;
        options.out_of_time = [&]()
        {
            const std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - start;
            return elapsed_seconds.count() > params.real_time_threshold_seconds;
        };
        if (params.log_iteration_convergence)
            options.after_iteration = [&](double delta)
            {
                poses_after.push_back(window.poses);
                deltas.push_back(delta);
            };
        if (debugMsg)
            options.factor_change = [&](double factor, double delta)
            { log_debug(logger) << "\nlm_factor " << factor << ", delta " << std::setprecision(10) << delta << "\n"; };

        const auto phase = [timing](PhaseTime ScanToMapTiming::*p) { return timing ? &(timing->*p) : nullptr; };
        WindowOptimization result;
        static_cast<LmResult &>(result) =
            lm_solve(problem, options, LmWorkspace{workspace.covariance, workspace.damped},
                     LmTiming{phase(&ScanToMapTiming::covariance_solve), phase(&ScanToMapTiming::solve), phase(&ScanToMapTiming::update)});
        if (params.lm_step_check)
            result.cost = problem.cost();
        result.honesty = problem.honesty_inputs();
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
        if (params.lm_step_check)
            msg << " (" << optimization.rejected_steps << " steps rejected, cost " << std::scientific << std::setprecision(4)
                << optimization.cost.total << std::defaultfloat << ", matched " << optimization.cost.matched << ", unmatched "
                << optimization.cost.unmatched << ")";
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
                                   const ScanToMapOptions &scan_to_map, const MotionModelOptions &mm, const AccTerm &acc_term,
                                   const Eigen::Matrix3d &lidar_translation_information,
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
        { return ndt_window_cost(intermediate_points, poses, state.buckets_indoor, state.buckets_outdoor, scan_to_map, ndt_noise(scan_to_map)); };
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
        { return ndt_window_cost(intermediate_points, poses, state.buckets_indoor, state.buckets_outdoor, scan_to_map, ndt_noise(scan_to_map)); };
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
        const bool at_rest = window_rests(window.poses, window.timestamps, window.imu_acc, window.imu_angles, setup.gravity_norm,
                                          params.zero_velocity_acc_threshold_mps2, params.zero_velocity_gyro_threshold_deg_s * DEG_TO_RAD);
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(lidar_translation_information);
        const double ratio = eig.eigenvalues().maxCoeff() > 0.0 ? eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff() : 0.0;
        const auto v = [](const Eigen::Vector3d &x) { std::ostringstream o; o << std::fixed << std::setprecision(4) << x.x() << " " << x.y() << " " << x.z(); return o.str(); };
        log_info(logger) << "accelerometer fit " << i + 1 << ": ratio " << std::fixed << std::setprecision(3) << ratio << ", acc term "
                         << (acc_term.options.enabled ? 1 : 0) << ", triples " << world.size() << ", world mean " << v(world_mean)
                         << ", body mean " << v(body_mean) << ", rms " << std::setprecision(4) << std::sqrt(rest) << ", max gap ms "
                         << std::setprecision(2) << max_gap * 1000.0 << ", rest " << (at_rest ? 1 : 0);
    }

    // log_window_terms: one motion of the table in a few words: the three terms that hold it most (as a sigma), the largest
    // pull, and the step all terms together still want (the total's pull)
    std::string motion_summary(const WindowTermTable &t, int motion, int direction, const char *unit)
    {
        std::vector<const WindowTermRow *> rows;
        const WindowTermRow *total = nullptr;
        for (const WindowTermRow &r : t.rows)
        {
            if (r.name == "total")
                total = &r;
            else if (r.in_total && r.weight[motion][direction] > 0.0)
                rows.push_back(&r);
        }
        std::sort(rows.begin(), rows.end(), [&](const WindowTermRow *a, const WindowTermRow *b)
                  { return a->weight[motion][direction] > b->weight[motion][direction]; });
        std::ostringstream o;
        o << term_motion_name(motion) << " " << term_direction_name(direction) << ": hold";
        for (size_t k = 0; k < rows.size() && k < 3; k++)
            o << " " << rows[k]->name << " " << std::scientific << std::setprecision(1) << 1.0 / std::sqrt(rows[k]->weight[motion][direction]);
        const WindowTermRow *largest = nullptr;
        for (const WindowTermRow *r : rows)
            if (std::isfinite(r->pull[motion][direction]) &&
                (largest == nullptr || std::abs(r->pull[motion][direction]) > std::abs(largest->pull[motion][direction])))
                largest = r;
        o << std::fixed << std::setprecision(4);
        if (largest)
            o << ", largest pull " << largest->name << " " << largest->pull[motion][direction];
        if (total)
            o << ", remaining " << total->pull[motion][direction];
        o << " " << unit;
        return o.str();
    }

    // log_window_terms: cost per residual of every term, guess -> result
    std::string cost_summary(const WindowTermTable &guess, const WindowTermTable &result)
    {
        std::ostringstream o;
        o << std::setprecision(3);
        for (const WindowTermRow &r : result.rows)
        {
            if (!r.in_total || r.residuals == 0)
                continue;
            double before = std::numeric_limits<double>::quiet_NaN();
            for (const WindowTermRow &g : guess.rows)
                if (g.name == r.name && g.residuals > 0)
                    before = g.cost / g.residuals;
            o << " " << r.name << " " << before << "->" << r.cost / r.residuals;
        }
        return o.str();
    }

    // what the window loop knows of one window for its term record
    struct WindowTermSources
    {
        const std::vector<Point3Di> &points;
        const Window &guess;      // the window as the first iteration found it
        const OdometryState &state;
        const OdometryRunSetup &setup;
        const AccTerm &acc_term;
        const MotionModelRotationOptions &rotation;
        const WindowNoiseModel &noise;
        const Eigen::Vector3d &acc_offset;
        const WindowOptimization &optimization;
        bool at_rest;
    };

    // log_window_terms: the per-term table at the guess and at the result, one log line, and the record for the sink.
    // Only reads: nothing here changes the window, the maps or the loop's state. last_along: the travel direction of the
    // previous window, used when this one moves less than 1 cm, and updated.
    std::optional<WindowTermRecord> record_window_terms(int i, const std::vector<Window> &windows, const WindowTermSources &s,
                                                        const OdometryConfig &params, Eigen::Vector3d &last_along, Logger &logger)
    {
        if (!params.log_window_terms)
            return std::nullopt;
        const Window &window = windows[i];
        last_along = window_travel_direction(window.poses, last_along);
        const Eigen::Vector3d no_offset = Eigen::Vector3d::Zero(); // the acceleration offset starts at zero in every window
        const WindowTermInputs at_guess{s.points, s.guess, s.state.buckets_indoor, s.state.buckets_outdoor, s.setup.scan_to_map,
                                        s.setup.motion_model, s.rotation, s.acc_term.options, s.noise, no_offset,
                                        params.lm_factor_initial, last_along};
        const WindowTermInputs at_result{s.points, window, s.state.buckets_indoor, s.state.buckets_outdoor, s.setup.scan_to_map,
                                         s.setup.motion_model, s.rotation, s.acc_term.options, s.noise, s.acc_offset,
                                         s.optimization.lm_factor, last_along};
        WindowTermRecord r;
        r.window = i + 1;
        if (!window.timestamps.empty() && !windows[0].timestamps.empty())
        {
            r.time_s = window.timestamps.front().first - windows[0].timestamps.front().first;
            r.duration_s = window.timestamps.back().first - window.timestamps.front().first;
        }
        r.iterations = s.optimization.last_iteration + 1;
        r.delta = s.optimization.delta;
        r.lm_factor = s.optimization.lm_factor;
        r.at_rest = s.at_rest;
        r.acc_term = s.acc_term.options.enabled;
        if (!window.poses.empty())
        {
            r.guess_move = s.guess.poses.back().translation() - s.guess.poses.front().translation();
            r.result_move = window.poses.back().translation() - window.poses.front().translation();
        }
        r.velocity_start = window.velocity.start;
        r.velocity_end = window.velocity.end;
        r.gyro_bias_deg_s = window.gyro_bias.mean * RAD_TO_DEG;
        r.acc_offset_mps2 = s.acc_offset;
        r.guess = window_term_table(at_guess);
        r.result = window_term_table(at_result);
        r.change = window_motion_change(s.guess, window, r.result.directions);

        for (const WindowTermTable *t : {&r.guess, &r.result})
            if (!t->failure.empty())
                log_warning(logger) << "window terms " << i + 1 << ": " << (t == &r.guess ? "guess" : "result") << " table not computed: " << t->failure;
        if (r.result.failure.empty())
        {
            auto line = log_info(logger);
            line << "window terms " << i + 1 << ": along " << std::fixed << std::setprecision(2) << last_along.x() << " " << last_along.y()
                 << "; cost/n" << cost_summary(r.guess, r.result) << "; " << motion_summary(r.result, 1, 0, "m") << "; "
                 << motion_summary(r.result, 2, 0, "m") << "; " << motion_summary(r.result, 2, 2, "m");
            if (r.result.turn_defined)
                line << "; " << motion_summary(r.result, 3, 2, "deg");
        }
        return r;
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
        log_phase_time(logger, "preview", t.preview, total_s);
        log_phase_time(logger, "add_to_map", t.add_to_map, total_s);
        log_phase_time(logger, "window_total", t.window_total, total_s);
        const double named = t.lm_iterations.seconds + t.point_load.seconds + t.preview.seconds + t.add_to_map.seconds;
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
        if (setup.always_on)
            log_info(logger) << "accelerometer term: active in " << carry.acc_term_windows << " of " << windows.size() << " windows";
        if (setup.always_on)
            log_info(logger) << "accelerometer always on: " << carry.rest_windows << " of " << windows.size() << " windows at rest";
        log_info(logger) << "silent events: map rebuilds " << carry.map_rebuilds << ", solver steps skipped " << workspace.steps_skipped
                         << ", big-jump nodes (their steps not applied) " << workspace.big_jump_nodes;
        if (params.log_iteration_convergence && !carry.convergence.empty())
            log_convergence_summary(carry.convergence, logger);
        log_run_verdict(summarize_run(carry.verdicts, true), VerdictLimits{}, logger);
        if (params.log_stage_timing)
        {
            log_odometry_timing(carry.timing, std::chrono::duration<double>(std::chrono::steady_clock::now() - carry.timing.start).count(), windows.size(), logger);
            log_solver_analyses(workspace, logger);
        }
    }
} // namespace

void start_window_loop(WindowLoop &loop, const Window &first, const OdometryConfig &params, Logger &logger)
{
    loop.setup = prepare_run(first, params, logger);
    log_noise_budget(params, logger);
    loop.carry = start_carry(loop.setup, first);
}

namespace
{
    // what process_windows hands to its four steps
    struct LoopContext
    {
        WindowLoop &loop;
        std::vector<Window> &windows;
        size_t first; // the range's first window (index of the forced iteration counts)
        const OdometryConfig &params;
        OdometryState &state;
        double &ts_failure;
        bool debugMsg;
        PointStore &point_store;
        PreviewSink &preview;
        Logger &logger;
        ProgressReporter &progress;
        const WindowRange &range;
        Eigen::Vector3d last_along = Eigen::Vector3d::UnitX(); // log_window_terms: the travel direction of the window before

        // log_stage_timing: the timers get a target only when the setting is on
        PhaseTime *timed(PhaseTime &t) const { return params.log_stage_timing ? &t : nullptr; }
    };

    // one window's values that pass from step to step
    struct WindowWork
    {
        size_t window_index = 0;
        int i = 0;
        std::vector<Point3Di> intermediate_points;
        PreviousVelocity previous;                   // the previous window's final velocity
        MotionModelRotationOptions rotation_options; // this window's (bias estimation from the second window on)
        AccTerm acc_term;
        bool at_rest = false;
        Eigen::Matrix3d lidar_translation_information = Eigen::Matrix3d::Zero();
        Eigen::Vector3d acc_offset = Eigen::Vector3d::Zero();
        VelocityPrior velocity_prior;
        WindowNoiseModel noise; // every weight of the window (window_noise_model.h)
        std::chrono::time_point<std::chrono::system_clock> window_start;
        std::vector<Eigen::Affine3d> poses_before_optimization; // the guess
        Window guess;                                           // log_window_terms: the window as the first iteration finds it
        WindowOptimization optimization;
        int steps_not_applied = 0; // failed solves of this window
        int big_jump_nodes = 0;    // nodes whose step was refused (big jump)
        // log_window_terms: the state the window was prepared with and the two features (consistency-harness D9); the
        // record is completed after the hand-off and written once, at the end of finish_window
        WindowPreparedState prepared;
        std::optional<double> rotation_rate_rad_s;
        std::optional<Eigen::Vector3d> start_acceleration_mps2;
        std::optional<WindowTermRecord> record;
    };

    // log_window_terms: what the window was prepared with (taken at the end of prepare_window, on its own row)
    void record_prepared_state(const Window &window, const WindowLoopCarry &carry, WindowWork &w)
    {
        WindowPreparedState &p = w.prepared;
        if (!window.poses.empty())
            p.first_pose = window.poses.front();
        const MotionModelTranslationImuOptions &o = w.acc_term.options;
        if (o.enabled)
        {
            p.velocity_prior_mean = o.velocity_prior_mean;
            p.velocity_prior_cov = o.velocity_prior_cov;
            p.offset_prior_mean = Eigen::Vector3d::Zero(); // the offset is folded into gravity after every window
            p.offset_prior_cov = o.offset_prior_cov;
            p.velocity_offset_prior_cov = o.velocity_offset_prior_cov;
            p.gravity = o.gravity;
        }
        if (w.rotation_options.estimate_bias)
        {
            p.bias_prior_mean_rad_s = w.rotation_options.bias_prior_mean;
            p.bias_prior_cov = w.rotation_options.bias_prior_cov;
        }
        w.rotation_rate_rad_s = mean_rotation_rate(window.poses, window.timestamps);
        if (carry.acc_gravity.norm() > 0.0)
            w.start_acceleration_mps2 = start_world_acceleration(window.poses, window.timestamps, window.imu_acc, carry.acc_gravity);
    }

    // the window's points, the previous window's velocity and the guess continued from the previous window's end
    void load_and_predict(LoopContext &c, WindowWork &w)
    {
        OdometryRunSetup &setup = c.loop.setup;
        const int i = w.i;
        PhaseTimer point_load_timer(c.timed(c.loop.carry.timing.point_load));
        c.point_store.load(PointSet::WindowIntermediate, i, w.intermediate_points);
        remove_held_out(w.intermediate_points, c.params.holdout_block_s); // also out of the map update below
        point_load_timer.stop();

        c.progress.checkpoint();

        // final world-frame velocity of the previous window (for the velocity-continuity prior)
        if (setup.uses_previous_velocity && i > 0)
        {
            w.previous = estimate_previous_velocity(c.windows[i - 1], c.params.prior_velocity_span_nr_samples, c.params.max_prior_velocity_mps);
            if (w.previous.dropped)
                log_info(c.logger) << "window " << i + 1 << ": previous window velocity " << w.previous.speed
                                   << " m/s > max_prior_velocity_mps " << c.params.max_prior_velocity_mps << ": not used as prior";
        }

        // initial guess: continue from the previous window's end
        if ((i > 1 || (setup.uses_previous_velocity && i > 0)) && c.params.use_motion_from_previous_step)
        {
            const PosePrediction prediction = predict_window_poses(c.windows[i], c.windows[i - 1], i > 1 ? &c.windows[i - 2] : nullptr,
                                                                   w.previous.velocity, setup.prediction);
            if (prediction.mean_shift_dropped)
                log_info(c.logger) << "!!!mean_shift.norm() > 1.0!!!";
            if (prediction.stationary)
            {
                c.loop.carry.stationary_windows++;
                if (c.debugMsg)
                    log_debug(c.logger) << "window " << i + 1 << " stationary (zero-velocity detection)\n";
            }
        }
    }

    // step 1: everything the iterations start from: the points, the guess, the gyro-bias prior, this window's rotation
    // options, the accelerometer term and the velocity state; the window's motion model is the guess
    WindowWork prepare_window(LoopContext &c, size_t window_index)
    {
        OdometryRunSetup &setup = c.loop.setup;
        WindowLoopCarry &carry = c.loop.carry;
        const OdometryConfig &params = c.params;
        WindowWork w;
        w.window_index = window_index;
        w.i = static_cast<int>(window_index);
        const int i = w.i;
        Window &window = c.windows[i];
        load_and_predict(c, w);

        // gyro bias: prior from the previous window, grown by the bias random walk over this window
        if (setup.rotation_options.estimate_bias)
        {
            const GyroBiasState prior = apply_gyro_bias_prior(window, carry.bias, setup.bias_rw_rad);
            setup.rotation_options.bias_prior_mean = prior.mean;
            setup.rotation_options.bias_prior_cov = prior.covariance;
        }

        // the first window is registered against the initial map, which is built from not yet optimized points
        // (motion during the first threshold_initial_points is not compensated): its misfit must not enter the bias,
        // so the bias is only estimated from the second window on
        w.rotation_options = setup.rotation_options;
        w.rotation_options.estimate_bias = setup.rotation_options.estimate_bias && i > 0;

        // accelerometer term (from the second window on, like the bias) and the window's translation sigmas
        w.acc_term = prepare_acc_term(window, i > 0, setup, carry, params);
        if (setup.always_on)
        {
            w.at_rest = window_at_rest(window, setup, params);
            if (w.at_rest)
                carry.rest_windows++;
            if (i == 0)
                carry.first_window_at_rest = w.at_rest;
            prepare_velocity_state(window, w.previous, w.at_rest, setup, carry, w.acc_term, params);
            // the guess (and so the motion model) follows the accelerometer from the start velocity prior; a window at rest
            // keeps its motionless guess
            if (w.acc_term.options.enabled && !w.at_rest)
            {
                w.acc_term.options.velocity_guess = window.velocity.start;
                w.acc_term.options.guess_integrated =
                    integrate_window_translations(window, window.velocity.start, carry.acc_gravity, w.acc_term.options.to_mps2);
            }
        }
        w.velocity_prior = VelocityPrior{w.previous.use, w.previous.velocity};

        w.window_start = std::chrono::system_clock::now();
        w.poses_before_optimization = window.poses;

        window.motion_model_poses = window.poses;
        if (params.log_window_terms)
            w.guess = window;
        w.noise = build_window_noise_model(window, setup.scan_to_map, setup.motion_model, w.rotation_options, w.acc_term.options, w.velocity_prior);
        if (params.log_window_terms)
            record_prepared_state(window, carry, w);
        return w;
    }

    // step 2: the iterations, and the warnings about steps the solver could not take
    void solve_window(LoopContext &c, WindowWork &w)
    {
        WindowLoopCarry &carry = c.loop.carry;
        ScanToMapWorkspace &workspace = c.loop.workspace;
        PhaseTimer lm_timer(c.timed(carry.timing.lm_iterations));
        const int steps_skipped_before = workspace.steps_skipped;
        const int big_jump_nodes_before = workspace.big_jump_nodes;
        const int forced = c.range.forced_last_iteration ? (*c.range.forced_last_iteration)[w.window_index - c.first] : -1;
        w.optimization = run_lm_iterations(w.intermediate_points, c.windows[w.i], c.state, c.loop.setup, w.acc_term, w.rotation_options,
                                           w.noise, w.acc_offset, w.lidar_translation_information, c.params, c.debugMsg, c.logger,
                                           c.params.log_stage_timing ? &carry.timing.lm : nullptr, workspace, forced);
        lm_timer.stop();
        w.steps_not_applied = workspace.steps_skipped - steps_skipped_before;
        w.big_jump_nodes = workspace.big_jump_nodes - big_jump_nodes_before;
        if (workspace.steps_skipped > steps_skipped_before)
            log_warning(c.logger) << "window " << w.i + 1 << ": " << workspace.steps_skipped - steps_skipped_before
                                  << " solver step(s) not applied (the solve failed)";
        if (workspace.big_jump_nodes > big_jump_nodes_before)
            log_warning(c.logger) << "window " << w.i + 1 << ": " << workspace.big_jump_nodes - big_jump_nodes_before
                                  << " node(s) would have moved 1 m or more from their guess: those steps were not applied (big jump)";
    }

    // what the run knows about window w for its verdict (window_verdict.h); read before the tail is re-propagated. A reading
    // the window doesn't have stays empty: speed and chi2 without the accelerometer term, the bias without its estimation
    WindowEvidence window_evidence(const LoopContext &c, const WindowWork &w, const WindowTermRecord *record)
    {
        const Window &window = c.windows[w.i];
        WindowEvidence e;
        e.window = w.i + 1;
        if (!window.timestamps.empty() && !c.windows[0].timestamps.empty())
            e.time_s = window.timestamps.front().first - c.windows[0].timestamps.front().first;
        if (!window.poses.empty() && w.poses_before_optimization.size() == window.poses.size())
        {
            // as log_heading_diagnostics' "yaw correction last", and its horizontal components
            const Eigen::AngleAxisd aa(window.poses.back().linear() * w.poses_before_optimization.back().linear().transpose());
            e.heading_bend_deg = aa.angle() * aa.axis().z() * RAD_TO_DEG;
            e.roll_bend_deg = aa.angle() * aa.axis().x() * RAD_TO_DEG;
            e.pitch_bend_deg = aa.angle() * aa.axis().y() * RAD_TO_DEG;
        }
        if (w.acc_term.options.enabled)
        {
            e.speed_mps = window.velocity.end.norm();
            const Eigen::Vector3d jump = window.velocity.start - w.acc_term.options.velocity_prior_mean;
            const Eigen::Matrix3d Ci = w.acc_term.options.velocity_prior_cov.inverse();
            e.velocity_chi2 = jump.dot(Ci * jump);
        }
        if (w.rotation_options.estimate_bias)
            e.max_bias_deg_s = (window.gyro_bias.mean * RAD_TO_DEG).cwiseAbs().maxCoeff();
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(w.lidar_translation_information);
        if (eig.eigenvalues().maxCoeff() > 0.0)
            e.information_ratio = eig.eigenvalues().minCoeff() / eig.eigenvalues().maxCoeff();
        e.stop = w.optimization.stop;
        e.iterations = w.optimization.last_iteration + 1;
        e.lm_factor = w.optimization.lm_factor;
        e.rejected_steps = w.optimization.rejected_steps;
        e.steps_not_applied = w.steps_not_applied;
        e.big_jump_nodes = w.big_jump_nodes;
        e.moves = honesty_readings(w.optimization.honesty);
        if (record && record->result.failure.empty())
            for (const WindowTermRow &row : record->result.rows)
            {
                if (row.name == "total")
                {
                    double largest = 0.0;
                    for (int m = 0; m < 3; m++)
                        for (int d = 0; d < kTermDirections; d++)
                            if (std::isfinite(row.pull[m][d]))
                                largest = std::max(largest, std::abs(row.pull[m][d]));
                    e.remaining_step_m = largest;
                }
            }
        if (record && record->result.failure.empty())
        {
            double ndt = 0.0, total = 0.0;
            for (const WindowTermRow &row : record->result.rows)
            {
                if (row.name == "ndt_indoor" || row.name == "ndt_outdoor")
                    ndt += row.weight[1][0];
                if (row.name == "total")
                    total = row.weight[1][0];
            }
            if (total > 0.0)
                e.lidar_share_along = ndt / total;
        }
        return e;
    }

    // step 3: what the window did, in the log (and the per-term table); reads the window before its tail is re-propagated
    void report_window(LoopContext &c, WindowWork &w)
    {
        const OdometryConfig &params = c.params;
        WindowLoopCarry &carry = c.loop.carry;
        const OdometryRunSetup &setup = c.loop.setup;
        const int i = w.i;
        const std::chrono::duration<double> window_seconds = std::chrono::system_clock::now() - w.window_start;
        log_window_result(i, c.windows.size(), w.optimization, carry.acc_distance, window_seconds.count(), params, c.logger);
        log_iteration_convergence(i, params, w.optimization, carry, c.logger);
        log_window_diagnostics(i, c.windows, w.poses_before_optimization, w.optimization, w.previous, w.acc_term, w.lidar_translation_information,
                               params, c.logger);
        log_accelerometer_fit(i, c.windows[i], w.lidar_translation_information, w.acc_term, setup, carry, params, c.logger);
        log_weak_axis_diagnostics(i, c.windows[i], w.intermediate_points, c.state, setup.scan_to_map, setup.motion_model, w.acc_term, w.lidar_translation_information,
                                  params, c.logger);
        log_heading_diagnostics(i, c.windows[i], w.poses_before_optimization, w.intermediate_points, c.state, setup.scan_to_map, params, c.logger);
        std::optional<WindowTermRecord> record =
            record_window_terms(i, c.windows,
                                WindowTermSources{w.intermediate_points, w.guess, c.state, setup, w.acc_term, w.rotation_options, w.noise,
                                                  w.acc_offset, w.optimization, w.at_rest},
                                params, c.last_along, c.logger);

        if (params.log_window_noise)
            log_window_noise(i + 1, window_noise_sigmas(w.noise), c.logger);
        c.windows[i].readings = window_readings(c.windows[i], w.lidar_translation_information, w.at_rest); // for the loop closure
        const WindowVerdict verdict = judge_window(window_evidence(c, w, record ? &*record : nullptr));
        carry.verdicts.push_back(verdict);
        if (params.log_window_verdict && verdict.any())
            log_window_verdict(verdict, c.logger);
        if (record && c.range.terms)
        {
            record->verdict = verdict;
            record->noise = window_noise_sigmas(w.noise);
            record->prepared = w.prepared;
            record->rotation_rate_rad_s = w.rotation_rate_rad_s;
            record->start_acceleration_mps2 = w.start_acceleration_mps2;
            w.record = std::move(record); // completed and written at the end of finish_window
        }
    }

    // the record's handed-on part, and the record written (consistency-harness D9); nothing here changes the odometry
    void write_window_record(LoopContext &c, WindowWork &w, WindowHandOff handoff)
    {
        if (!w.record || !c.range.terms)
            return;
        w.record->handoff = handoff;
        c.range.terms->write(*w.record);
        w.record.reset();
    }

    WindowNode node_of(const Window &window, const std::vector<Window> &windows)
    {
        return WindowNode{window.poses.back(), window.timestamps.back().first - windows[0].timestamps.front().first};
    }

    // step 4: the hand-off (bias and offset carried, the tail re-propagated), the distance check, the next window placed
    // and this one added to the map. false: the run is cancelled (the window's distance was not a number)
    bool finish_window(LoopContext &c, WindowWork &w)
    {
        const OdometryConfig &params = c.params;
        OdometryRunSetup &setup = c.loop.setup;
        WindowLoopCarry &carry = c.loop.carry;
        std::vector<Window> &windows = c.windows;
        const int i = w.i;
        update_gyro_bias_after_window(windows[i], i, setup, carry, params, c.logger);
        update_acc_term_after_window(windows[i], i, w.acc_term, w.acc_offset, carry, c.logger);

        // log_window_terms: what is handed on (the offset is folded into gravity by now; the tail is re-propagated below)
        WindowHandedOnState *handed = w.record ? &w.record->handed_on : nullptr;
        if (handed)
        {
            if (setup.rotation_options.estimate_bias)
            {
                handed->bias_mean_rad_s = carry.bias.mean;
                handed->bias_cov = carry.bias.covariance;
            }
            if (w.acc_term.options.enabled)
            {
                handed->velocity_end = carry.velocity_mean;
                handed->velocity_end_cov = carry.velocity_cov;
                handed->gravity = carry.acc_gravity;
                handed->to_mps2_before_fold = w.acc_term.options.to_mps2;
                handed->offset_cov = carry.acc_offset_cov;
                if (w.acc_term.options.joint_covariance)
                    handed->velocity_offset_cov = carry.velocity_offset_cov;
            }
            if (!windows[i].poses.empty() && !windows[i].timestamps.empty())
                handed->last_node_solved = node_of(windows[i], windows);
        }

        // window hand-off; the initial guess was corrected with the prior bias, so the tail gets the update estimated in this window
        const Eigen::Vector3d bias_update = setup.rotation_options.estimate_bias
                                                ? Eigen::Vector3d(windows[i].gyro_bias.mean - setup.rotation_options.bias_prior_mean)
                                                : Eigen::Vector3d::Zero();
        repropagate_window_tail(windows[i], params.handoff_tail_nodes, params.handoff_tail_velocity_span_nr_samples, bias_update);
        if (handed && !windows[i].poses.empty() && !windows[i].timestamps.empty())
            handed->last_node_handed_on = node_of(windows[i], windows);

        if (i % 100 == 0)
        {
            PhaseTimer preview_timer(c.timed(carry.timing.preview));
            send_preview(i, w.intermediate_points, windows[i], params, c.preview);
        }

        if (!add_window_distance(windows[i], w.poses_before_optimization, carry, c.ts_failure, c.logger))
        {
            // the run stops here but is still saved: the remaining windows continue from this one's restored end
            windows[i].readings.reset(); // they describe the discarded solve; the loop closure then counts this window as unread
            place_windows_after(windows, i + 1);
            write_window_record(c, w, WindowHandOff::cancelled);
            return false;
        }

        if (c.range.outcomes)
            c.range.outcomes->push_back(WindowOutcome{w.optimization.last_iteration, windows[i].poses, w.optimization.rejected_steps});

        // reported only now: from here on the odometry doesn't change this window again (a viewer may read it)
        c.progress.progress(i + 1, windows.size());

        // the next window continues from the new end of this one; the windows after it are placed in their own turn,
        // since no step reads a window's placement before the window before it is done
        if (i + 1 < windows.size())
            place_window_after(windows[i + 1], windows[i].poses[windows[i].poses.size() - 1]);

        PhaseTimer add_to_map_timer(c.timed(carry.timing.add_to_map));
        if (add_window_to_map(w.intermediate_points, windows[i], carry, c.state, params))
            log_warning(c.logger) << "window " << i + 1 << ": reference map rebuilt from the newer half of its points "
                                  << "(sliding_window_trajectory_length_threshold " << params.sliding_window_trajectory_length_threshold << " m reached)";
        add_to_map_timer.stop();

        if (i > 1)
        {
            double translation = (windows[i - 1].poses[0].translation() -
                                  windows[i - 2].poses[0].translation())
                                     .norm();
            c.state.consecutive_distance += translation;
        }
        write_window_record(c, w, WindowHandOff::done);
        return true;
    }
}

bool process_windows(WindowLoop &loop, std::vector<Window> &windows, size_t first, size_t last, const OdometryConfig &params,
                     OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger,
                     ProgressReporter &progress, const WindowRange &range)
{
    LoopContext c{loop, windows, first, params, state, ts_failure, debugMsg, point_store, preview, logger, progress, range};
    for (size_t window_index = first; window_index <= last; window_index++)
    {
        PhaseTimer window_timer(c.timed(loop.carry.timing.window_total));
        WindowWork w = prepare_window(c, window_index);
        solve_window(c, w);
        report_window(c, w);
        if (!finish_window(c, w))
            return false;
    }
    return true;
}

bool capture_range(const std::vector<Window> &windows, double from_s, double to_s, std::size_t &first, std::size_t &last)
{
    if (windows.empty() || from_s < 0.0 || to_s < from_s)
        return false;
    const double t0 = windows[0].timestamps.front().first;
    bool found = false;
    for (std::size_t i = 0; i < windows.size(); i++)
    {
        const double t = windows[i].timestamps.front().first - t0;
        if (t < from_s || t > to_s)
            continue;
        if (!found)
            first = i;
        last = i;
        found = true;
    }
    return found;
}

namespace
{
    // the capture of windows first..last before the first of them is processed (the outcomes are added as they come)
    WindowCapture start_capture(const std::vector<Window> &windows, std::size_t first, std::size_t last, const WindowLoop &loop,
                                const OdometryState &state, PointStore &point_store)
    {
        WindowCapture c;
        c.n_windows = windows.size();
        c.first = first;
        c.last = last;
        c.first_window = windows[0];
        c.windows_begin = first >= 2 ? first - 2 : 0;
        const std::size_t end = std::min(last + 1, windows.size() - 1);
        c.windows.assign(windows.begin() + c.windows_begin, windows.begin() + end + 1);
        c.points.resize(last - first + 1);
        for (std::size_t i = first; i <= last; i++)
            point_store.load(PointSet::WindowIntermediate, i, c.points[i - first]);
        c.state.m_g = state.m_g;
        c.state.consecutive_distance = state.consecutive_distance;
        c.state.reference_points = state.reference_points;
        c.state.total_length_of_calculated_trajectory = state.total_length_of_calculated_trajectory;
        c.state.buckets_indoor = state.buckets_indoor;
        c.state.buckets_outdoor = state.buckets_outdoor;
        c.carry = loop.carry;
        return c;
    }
}

bool optimize_windows(std::vector<Window> &windows, const OdometryConfig &params, OdometryState &state, double &ts_failure, bool debugMsg, PointStore &point_store, PreviewSink &preview, Logger &logger, ProgressReporter &progress,
                      WindowCaptureSink *capture, WindowTermSink *terms)
{
    if (windows.size() != 0)
    {
        const auto start = std::chrono::system_clock::now();

        place_window_after(windows[0], state.m_g);
        windows[0].motion_model_poses = windows[0].poses;
        seed_map(state, params);

        WindowLoop loop;
        start_window_loop(loop, windows[0], params, logger);
        const std::size_t n = windows.size();
        const auto run = [&](std::size_t first, std::size_t last, const WindowRange &range)
        { return first > last || process_windows(loop, windows, first, last, params, state, ts_failure, debugMsg, point_store, preview, logger, progress, range); };

        std::size_t capture_first = 0, capture_last = 0;
        const bool capturing = capture && capture_range(windows, params.capture_windows_from_s, params.capture_windows_to_s, capture_first, capture_last);
        if (capture && !capturing && params.capture_windows_from_s >= 0.0)
            log_warning(logger) << "window capture: no window starts between " << params.capture_windows_from_s << " and "
                                << params.capture_windows_to_s << " s";
        WindowRange plain; // every window outside a capture
        plain.terms = terms;
        // a cancelled run logs the verdict summary of the windows it finished
        const auto cancelled = [&]()
        {
            log_run_verdict(summarize_run(loop.carry.verdicts, false), VerdictLimits{}, logger);
            return false;
        };
        if (!capturing)
        {
            if (!run(0, n - 1, plain))
                return cancelled();
        }
        else
        {
            // the same windows in the same order as without the capture, in three parts: before, in and after the range
            if (capture_first > 0 && !run(0, capture_first - 1, plain))
                return cancelled();
            WindowCapture c = start_capture(windows, capture_first, capture_last, loop, state, point_store);
            WindowRange range = plain;
            range.outcomes = &c.outcomes;
            if (!run(capture_first, capture_last, range))
                return cancelled();
            std::string why;
            if (capture->write(c, why))
                log_info(logger) << "window capture written: windows " << capture_first + 1 << " to " << capture_last + 1;
            else
                log_error(logger) << "window capture not written: " << why;
            if (capture_last + 1 < n && !run(capture_last + 1, n - 1, plain))
                return cancelled();
        }

        finish_run(windows, start, loop.setup, loop.carry, state, params, loop.workspace, logger);
    }

    return true;
}
