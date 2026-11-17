#include "window_noise_model.h"
#include "angle_units.h"

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
    }

    m.origin = {1.0 / (motion_model.fix_origin_x_sigma_m * motion_model.fix_origin_x_sigma_m),
                1.0 / (motion_model.fix_origin_y_sigma_m * motion_model.fix_origin_y_sigma_m),
                1.0 / (motion_model.fix_origin_z_sigma_m * motion_model.fix_origin_z_sigma_m),
                1.0 / (motion_model.fix_origin_om_sigma_deg * motion_model.fix_origin_om_sigma_deg),
                1.0 / (motion_model.fix_origin_fi_sigma_deg * motion_model.fix_origin_fi_sigma_deg),
                1.0 / (motion_model.fix_origin_ka_sigma_deg * motion_model.fix_origin_ka_sigma_deg)};

    m.ndt.planarity = scan_to_map.use_planarity;
    m.ndt.indoor = NdtGridNoise{scan_to_map.use_norm, true};
    m.ndt.outdoor = NdtGridNoise{false, false};
    m.ndt.huber_cutoff = scan_to_map.huber_cutoff;
    m.ndt.information_scale = scan_to_map.information_scale;
    m.ndt.projection_ratio = scan_to_map.degenerate_projection_ratio;
    m.ndt.projection_ratio_full = scan_to_map.degenerate_projection_ratio_full;
    return m;
}
