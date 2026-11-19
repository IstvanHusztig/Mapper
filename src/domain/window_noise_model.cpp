#include "window_noise_model.h"
#include "angle_units.h"
#include "window_verdict.h"

#include <iomanip>
#include <sstream>
#include <string>

#include <algorithm>
#include <cmath>

double time_scaled_rotation_sigma(double inflation_k, double arw_rad_sqrt_s, double dt, double floor_rad)
{
    return std::max(inflation_k * arw_rad_sqrt_s * std::sqrt(dt), floor_rad);
}

MotionEdgeWeights motion_edge_weights(const WindowUnknowns &unknowns, const Window &window, const MotionModelOptions &motion_model,
                                      const MotionModelRotationOptions &rotation_options,
                                      const MotionModelTranslationImuOptions &translation_imu_options)
{
    MotionEdgeWeights weights;
    double sigma_motion_model_om = motion_model.om_sigma_deg * DEG_TO_RAD;
    double sigma_motion_model_fi = motion_model.fi_sigma_deg * DEG_TO_RAD;
    double sigma_motion_model_ka = motion_model.ka_sigma_deg * DEG_TO_RAD;

    double w_motion_model_om = 1.0 / (sigma_motion_model_om * sigma_motion_model_om);
    double w_motion_model_fi = 1.0 / (sigma_motion_model_fi * sigma_motion_model_fi);
    double w_motion_model_ka = 1.0 / (sigma_motion_model_ka * sigma_motion_model_ka);

    weights.x = 1.0 / (motion_model.x_sigma_m * motion_model.x_sigma_m);
    weights.y = 1.0 / (motion_model.y_sigma_m * motion_model.y_sigma_m);
    weights.z = 1.0 / (motion_model.z_sigma_m * motion_model.z_sigma_m);
    // imu_position_information_scale: the x/y/z rows (with their attitude coupling) weaker while the accelerometer term is
    // active (Codex's "beta", DMAPPER_EXPERIMENT_MOTION_XYZ_SCALE in cbca36d)
    if (unknowns.acc_term && translation_imu_options.position_information_scale != 1.0)
    {
        weights.x *= translation_imu_options.position_information_scale;
        weights.y *= translation_imu_options.position_information_scale;
        weights.z *= translation_imu_options.position_information_scale;
    }

    for (size_t i = 1; i < window.poses.size(); i++)
    {
        double w_om_i = w_motion_model_om;
        double w_fi_i = w_motion_model_fi;
        double w_ka_i = w_motion_model_ka;
        double dt_i = 0.0;
        if (unknowns.time_scaled_sigma || unknowns.estimate_bias)
            dt_i = std::max(window.timestamps[i].first - window.timestamps[i - 1].first, 0.0);

        if (unknowns.time_scaled_sigma)
        {
            // gyro angle random walk over this edge, inflated by k, floored for near-zero dt
            const double s_om = time_scaled_rotation_sigma(rotation_options.inflation_k, rotation_options.arw_rad_sqrt_s.x(), dt_i, rotation_options.sigma_floor_rad);
            const double s_fi = time_scaled_rotation_sigma(rotation_options.inflation_k, rotation_options.arw_rad_sqrt_s.y(), dt_i, rotation_options.sigma_floor_rad);
            const double s_ka = time_scaled_rotation_sigma(rotation_options.inflation_k, rotation_options.arw_rad_sqrt_s.z(), dt_i, rotation_options.sigma_floor_rad);
            w_om_i = 1.0 / (s_om * s_om);
            w_fi_i = 1.0 / (s_fi * s_fi);
            w_ka_i = 1.0 / (s_ka * s_ka);
        }
        weights.rotation.emplace_back(w_om_i, w_fi_i, w_ka_i);
        weights.dt.push_back(dt_i);
    }
    return weights;
}

NdtNoise ndt_noise(const ScanToMapOptions &scan_to_map)
{
    NdtNoise n;
    n.planarity = scan_to_map.use_planarity;
    n.indoor = NdtGridNoise{scan_to_map.use_norm, true};
    n.outdoor = NdtGridNoise{false, false};
    n.huber_cutoff = scan_to_map.huber_cutoff;
    n.information_scale = scan_to_map.information_scale;
    n.projection_ratio = scan_to_map.degenerate_projection_ratio;
    n.projection_ratio_full = scan_to_map.degenerate_projection_ratio_full;
    return n;
}

Eigen::Matrix3d random_walk_prior_cov(const Eigen::Matrix3d &carried, double rw, double duration_s)
{
    return carried + Eigen::Matrix3d::Identity() * (rw * rw * duration_s);
}

Eigen::Matrix3d velocity_prior_cov_over_gap(const Eigen::Matrix3d &carried, double density, double floor, double gap_s)
{
    return carried + Eigen::Matrix3d::Identity() * (density * density * gap_s + floor * floor * gap_s * gap_s);
}

JointPriorCov joint_prior_cov(const Eigen::Matrix3d &velocity, const Eigen::Matrix3d &cross, const Eigen::Matrix3d &offset)
{
    JointPriorCov cov;
    cov.topLeftCorner<3, 3>() = velocity;
    cov.topRightCorner<3, 3>() = cross;
    cov.bottomLeftCorner<3, 3>() = cross.transpose();
    cov.bottomRightCorner<3, 3>() = offset;
    return cov;
}

JointPriorCov joint_prior_cov_over_gap(const JointPriorCov &carried, double density, double floor, double gap_s, double offset_rw,
                                       double duration_s)
{
    const Eigen::Matrix3d vv = carried.topLeftCorner<3, 3>();
    const Eigen::Matrix3d ve = carried.topRightCorner<3, 3>();
    const Eigen::Matrix3d ee = carried.bottomRightCorner<3, 3>();
    // F carried F^T with F = [[I, -gap I], [0, I]]
    const Eigen::Matrix3d vv_moved = vv - (ve + ve.transpose()) * gap_s + ee * (gap_s * gap_s);
    const Eigen::Matrix3d ve_moved = ve - ee * gap_s;
    return joint_prior_cov(velocity_prior_cov_over_gap(vv_moved, density, floor, gap_s), ve_moved,
                           random_walk_prior_cov(ee, offset_rw, duration_s));
}

namespace
{
    // velocity continuity (only without the accelerometer term, with timestamps): sigma = acc_sigma * mean dt of the two
    // steps around a node, the first step's sigma = acc_sigma * dt; dt clamped at 0.1 ms
    void velocity_continuity_weights(WindowNoiseModel &m, const Window &window, const MotionModelOptions &motion_model,
                                     const VelocityPrior &velocity_prior)
    {
        if (!(motion_model.acc_sigma_mps2 > 0.0 && !m.unknowns.acc_term && window.timestamps.size() == window.poses.size()))
            return;
        constexpr double min_dt = 1e-4;
        m.velocity_continuity = true;
        for (size_t i = 1; i + 1 < window.poses.size(); i++)
        {
            const double dt1 = std::max(window.timestamps[i].first - window.timestamps[i - 1].first, min_dt);
            const double dt2 = std::max(window.timestamps[i + 1].first - window.timestamps[i].first, min_dt);
            const double sigma_dv = motion_model.acc_sigma_mps2 * 0.5 * (dt1 + dt2);
            m.velocity_continuity_weights.push_back(1.0 / (sigma_dv * sigma_dv));
        }
        if (velocity_prior.use && window.poses.size() > 1)
        {
            const double dt = std::max(window.timestamps[1].first - window.timestamps[0].first, min_dt);
            const double sigma_dv = motion_model.acc_sigma_mps2 * dt;
            m.first_step_prior = true;
            m.first_step_weight = 1.0 / (sigma_dv * sigma_dv);
            m.first_step_velocity = velocity_prior.velocity;
        }
    }

    // the accelerometer terms over the anchors, in the order they are added: the anchor triples (the change of velocity
    // between consecutive anchor pairs against the mean acceleration between their midpoints), the start velocity over
    // [node 0, first anchor], the end velocity over the last two anchors to the last node; a term whose stretch has an IMU
    // gap (longer than 2 span_s) is left out
    void accelerometer_terms(WindowNoiseModel &m, const Window &window, const MotionModelTranslationImuOptions &imu)
    {
        if (!m.unknowns.acc_term)
            return;
        const std::vector<int> &anchors = m.unknowns.anchors;
        const double density2 = imu.noise_density_mps2_sqrt_hz * imu.noise_density_mps2_sqrt_hz;
        const double floor2 = imu.floor_sigma_mps2 * imu.floor_sigma_mps2;
        const double max_dt = 2.0 * imu.span_s;
        const auto add = [&](AccelerometerTermKind kind, int a, int b, int c, double tau)
        { m.accelerometer_terms.push_back({kind, a, b, c, tau, 1.0 / ((density2 / tau + floor2) * tau * tau)}); };

        for (size_t j = 1; j + 1 < anchors.size(); j++)
        {
            const int A = anchors[j - 1], B = anchors[j], C = anchors[j + 1];
            const double tA = window.timestamps[A].first, tB = window.timestamps[B].first, tC = window.timestamps[C].first;
            const double dt1 = tB - tA, dt2 = tC - tB;
            if (dt1 > max_dt || dt2 > max_dt)
                continue;
            const double m1 = 0.5 * (tA + tB), m2 = 0.5 * (tB + tC);
            add(AccelerometerTermKind::anchor_triple, A, B, C, m2 - m1);
        }
        if (anchors.size() <= 1)
            return;
        {
            const int B = anchors[1];
            const double t0 = window.timestamps[0].first, tB = window.timestamps[B].first, dt = tB - t0;
            if (dt <= max_dt)
                add(AccelerometerTermKind::start_velocity, 0, B, 0, 0.5 * dt);
        }
        {
            const int n = static_cast<int>(window.poses.size());
            const int P = anchors[anchors.size() - 2], Q = anchors.back(), N = n - 1;
            const double tP = window.timestamps[P].first, tQ = window.timestamps[Q].first, tN = window.timestamps[N].first, dt = tQ - tP;
            const double mid = 0.5 * (tP + tQ);
            if (dt <= max_dt && tN - tQ <= max_dt)
            {
                add(AccelerometerTermKind::end_velocity, P, Q, N, tN - mid);
                m.end_velocity_tied = true;
            }
        }
    }
}

WindowNoiseModel build_window_noise_model(const Window &window, const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model,
                                          const MotionModelRotationOptions &rotation_options,
                                          const MotionModelTranslationImuOptions &translation_imu_options, const VelocityPrior &velocity_prior)
{
    WindowNoiseModel m;
    m.unknowns = layout_window_unknowns(window, rotation_options, translation_imu_options);
    m.edges = motion_edge_weights(m.unknowns, window, motion_model, rotation_options, translation_imu_options);
    velocity_continuity_weights(m, window, motion_model, velocity_prior);

    if (m.unknowns.estimate_bias)
    {
        m.bias_prior_mean = rotation_options.bias_prior_mean;
        m.bias_prior_cov = rotation_options.bias_prior_cov;
        m.bias_prior_information = rotation_options.bias_prior_cov.inverse();
    }

    accelerometer_terms(m, window, translation_imu_options);
    if (m.unknowns.acc_term)
    {
        m.velocity_prior_mean = translation_imu_options.velocity_prior_mean;
        m.velocity_prior_cov = translation_imu_options.velocity_prior_cov;
        m.velocity_prior_information = translation_imu_options.velocity_prior_cov.inverse();
        m.at_rest = translation_imu_options.at_rest;
        if (m.at_rest)
            m.zero_velocity_weight =
                1.0 / (translation_imu_options.zero_velocity_sigma_mps * translation_imu_options.zero_velocity_sigma_mps);
        m.offset_prior_mean = translation_imu_options.offset_prior_mean;
        m.offset_prior_cov = translation_imu_options.offset_prior_cov;
        m.offset_prior_information = translation_imu_options.offset_prior_cov.inverse();
        if (translation_imu_options.joint_covariance)
        {
            m.joint_prior = true;
            m.joint_prior_cov = joint_prior_cov(m.velocity_prior_cov, translation_imu_options.velocity_offset_prior_cov, m.offset_prior_cov);
            m.joint_prior_information = m.joint_prior_cov.inverse();
        }
    }

    m.origin = {1.0 / (motion_model.fix_origin_x_sigma_m * motion_model.fix_origin_x_sigma_m),
                1.0 / (motion_model.fix_origin_y_sigma_m * motion_model.fix_origin_y_sigma_m),
                1.0 / (motion_model.fix_origin_z_sigma_m * motion_model.fix_origin_z_sigma_m),
                1.0 / (motion_model.fix_origin_om_sigma_deg * motion_model.fix_origin_om_sigma_deg),
                1.0 / (motion_model.fix_origin_fi_sigma_deg * motion_model.fix_origin_fi_sigma_deg),
                1.0 / (motion_model.fix_origin_ka_sigma_deg * motion_model.fix_origin_ka_sigma_deg)};

    m.ndt = ndt_noise(scan_to_map);
    return m;
}

namespace
{
    double sigma_of(double weight) { return weight > 0.0 ? 1.0 / std::sqrt(weight) : kNoiseNaN; }

    // square roots of a covariance's eigenvalues, smallest first
    Eigen::Vector3d sigmas_of(const Eigen::Matrix3d &cov)
    {
        const Eigen::Vector3d ev = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(cov).eigenvalues();
        return ev.cwiseMax(0.0).cwiseSqrt();
    }

    std::string text(double v, int precision)
    {
        if (!std::isfinite(v))
            return "-";
        std::ostringstream o;
        o << std::setprecision(precision) << v;
        return o.str();
    }

    std::string triple(const Eigen::Vector3d &v, int precision)
    {
        return text(v.x(), precision) + " " + text(v.y(), precision) + " " + text(v.z(), precision);
    }
}

WindowNoiseSigmas window_noise_sigmas(const WindowNoiseModel &m)
{
    WindowNoiseSigmas s;
    s.translation_m = Eigen::Vector3d(sigma_of(m.edges.x), sigma_of(m.edges.y), sigma_of(m.edges.z));
    if (!m.edges.rotation.empty())
        for (int axis = 0; axis < 3; axis++)
        {
            std::vector<double> deg;
            for (const Eigen::Vector3d &w : m.edges.rotation)
                deg.push_back(sigma_of(w(axis)) / DEG_TO_RAD);
            s.rotation_min_deg(axis) = *std::min_element(deg.begin(), deg.end());
            s.rotation_median_deg(axis) = percentile_linear(deg, 50.0);
            s.rotation_max_deg(axis) = *std::max_element(deg.begin(), deg.end());
        }
    if (!m.velocity_continuity_weights.empty())
    {
        const auto [lo, hi] = std::minmax_element(m.velocity_continuity_weights.begin(), m.velocity_continuity_weights.end());
        s.velocity_continuity_min_mps = sigma_of(*hi); // the largest weight is the smallest sigma
        s.velocity_continuity_max_mps = sigma_of(*lo);
    }
    for (const AccelerometerTermSpan &t : m.accelerometer_terms)
    {
        const double sigma = sigma_of(t.weight);
        if (!std::isfinite(sigma))
            continue;
        s.accelerometer_min_mps = std::isfinite(s.accelerometer_min_mps) ? std::min(s.accelerometer_min_mps, sigma) : sigma;
        s.accelerometer_max_mps = std::isfinite(s.accelerometer_max_mps) ? std::max(s.accelerometer_max_mps, sigma) : sigma;
    }
    if (m.unknowns.acc_term)
    {
        s.velocity_prior_mps = sigmas_of(m.velocity_prior_cov);
        s.offset_prior_mps2 = sigmas_of(m.offset_prior_cov);
        if (m.at_rest)
            s.zero_velocity_mps = sigma_of(m.zero_velocity_weight);
    }
    if (m.unknowns.estimate_bias)
        s.bias_prior_deg_s = sigmas_of(m.bias_prior_cov) / DEG_TO_RAD;
    s.ndt_information_scale = m.ndt.information_scale;
    s.ndt_huber_cutoff = m.ndt.huber_cutoff;
    return s;
}

void log_window_noise(int window, const WindowNoiseSigmas &s, Logger &logger)
{
    log_info(logger) << "window noise " << window << ": edge translation sigma " << triple(s.translation_m, 4)
                     << " m, edge rotation sigma om/fi/ka [deg] min " << triple(s.rotation_min_deg, 4) << ", median " << triple(s.rotation_median_deg, 4)
                     << ", max " << triple(s.rotation_max_deg, 4) << "; velocity continuity sigma " << text(s.velocity_continuity_min_mps, 4) << ".."
                     << text(s.velocity_continuity_max_mps, 4) << " m/s; accelerometer sigma " << text(s.accelerometer_min_mps, 4) << ".."
                     << text(s.accelerometer_max_mps, 4) << " m/s; start velocity prior sigma " << triple(s.velocity_prior_mps, 4)
                     << " m/s; gyro bias prior sigma " << triple(s.bias_prior_deg_s, 4) << " deg/s; acceleration offset prior sigma "
                     << triple(s.offset_prior_mps2, 4) << " m/s^2; zero velocity sigma " << text(s.zero_velocity_mps, 4)
                     << " m/s; ndt information scale " << text(s.ndt_information_scale, 4) << ", huber cutoff " << text(s.ndt_huber_cutoff, 4);
}

void log_noise_budget(const OdometryConfig &p, Logger &logger)
{
    const auto v = [](double x) { return text(x, 6); };
    std::ostringstream rotation;
    if (p.use_time_scaled_rotation_sigma)
        rotation << "time-scaled max(k * ARW * sqrt(dt), floor), k " << v(p.rotation_sigma_inflation_k) << " (turn windows "
                 << (p.rotation_sigma_inflation_k_turn > 0.0 ? v(p.rotation_sigma_inflation_k_turn) : std::string("off")) << "), ARW x/y/z " << v(p.gyro_arw_x_deg_sqrt_s) << " " << v(p.gyro_arw_y_deg_sqrt_s) << " "
                 << v(p.gyro_arw_z_deg_sqrt_s) << " deg/sqrt(s), floor " << v(p.rotation_sigma_floor_deg) << " deg";
    else
        rotation << "om/fi/ka " << v(p.lidar_odometry_motion_model_om_1_sigma_deg) << " " << v(p.lidar_odometry_motion_model_fi_1_sigma_deg) << " "
                 << v(p.lidar_odometry_motion_model_ka_1_sigma_deg) << " deg";
    log_info(logger) << "noise budget: edge translation sigma x/y/z " << v(p.lidar_odometry_motion_model_x_1_sigma_m) << " "
                     << v(p.lidar_odometry_motion_model_y_1_sigma_m) << " " << v(p.lidar_odometry_motion_model_z_1_sigma_m) << " m (weight x "
                     << v(p.imu_position_information_scale) << " with the accelerometer term); edge rotation sigma " << rotation.str()
                     << "; velocity continuity " << v(p.lidar_odometry_motion_model_acc_1_sigma_mps2) << " m/s^2 (without the accelerometer term); "
                     << "accelerometer term " << (p.imu_always_on ? "on" : "off") << ", density " << v(p.imu_accel_noise_density_mps2_sqrt_hz)
                     << " m/s^2/sqrt(Hz), floor " << v(p.imu_accel_floor_sigma_mps2) << " m/s^2, span " << v(p.imu_acceleration_span_s)
                     << " s, window error " << v(p.imu_accel_window_sigma_mps2) << " m/s^2; edges follow the state "
                     << (p.imu_edges_follow_state ? "yes" : "no") << ", joint velocity-offset carry "
                     << (p.imu_carry_joint_covariance ? "yes" : "no") << "; zero velocity sigma " << v(p.imu_zero_velocity_sigma_mps) << " m/s; first start velocity sigma "
                     << v(p.imu_velocity_initial_sigma_mps) << " m/s; gyro bias " << (p.use_gyro_bias_estimation ? "estimated" : "off")
                     << ", initial sigma " << v(p.gyro_bias_initial_sigma_deg_s) << " deg/s, random walk " << v(p.gyro_bias_rw_deg_s_sqrt_s)
                     << " deg/s/sqrt(s); acceleration offset initial sigma " << v(p.imu_acceleration_offset_initial_sigma_mps2)
                     << " m/s^2, random walk " << v(p.imu_acceleration_offset_rw_mps2_sqrt_s) << " m/s^2/sqrt(s); ndt information scale "
                     << v(p.ndt_information_scale) << ", huber cutoff " << v(p.ndt_huber_cutoff) << "; origin fix sigma x/y/z "
                     << v(p.lidar_odometry_motion_model_fix_origin_x_1_sigma_m) << " " << v(p.lidar_odometry_motion_model_fix_origin_y_1_sigma_m)
                     << " " << v(p.lidar_odometry_motion_model_fix_origin_z_1_sigma_m) << " m, om/fi/ka "
                     << v(p.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg) << " " << v(p.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg)
                     << " " << v(p.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg) << " (used as radians, tracker C37)";
}
