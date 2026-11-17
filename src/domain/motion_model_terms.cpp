#include "motion_model_terms.h"
#include "angle_units.h"
#include <transformations.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

#include <algorithm>

void add_gyro_bias_prior(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                         const MotionModelRotationOptions &rotation_options)
{
    if (!unknowns.estimate_bias)
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    const int ib = unknowns.ib;
    // bias prior carried over from the previous window
    const Eigen::Matrix3d prior_inf = rotation_options.bias_prior_cov.inverse();
    const Eigen::Vector3d prior_delta = rotation_options.bias_prior_mean - window.gyro_bias.mean;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            AtPAndt.add(ib + r, ib + c, prior_inf(r, c));
    AtPBndt.block<3, 1>(ib, 0) += prior_inf * prior_delta;
    equations.motion_cost += prior_delta.dot(prior_inf * prior_delta);
    equations.residuals += 3;
}

void add_velocity_continuity(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                             const MotionModelOptions &motion_model, const VelocityPrior &velocity_prior)
{
    if (!(motion_model.acc_sigma_mps2 > 0.0 && !unknowns.acc_term && window.timestamps.size() == window.poses.size()))
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    double &motion_cost = equations.motion_cost;
    constexpr double min_dt = 1e-4;

    for (size_t i = 1; i + 1 < window.poses.size(); i++)
    {
        const double dt1 = std::max(window.timestamps[i].first - window.timestamps[i - 1].first, min_dt);
        const double dt2 = std::max(window.timestamps[i + 1].first - window.timestamps[i].first, min_dt);
        const double sigma_dv = motion_model.acc_sigma_mps2 * 0.5 * (dt1 + dt2);
        const double w = 1.0 / (sigma_dv * sigma_dv);

        const Eigen::Vector3d r = (window.poses[i + 1].translation() - window.poses[i].translation()) / dt2 -
                                  (window.poses[i].translation() - window.poses[i - 1].translation()) / dt1;

        const int nodes[3] = {int(i) - 1, int(i), int(i) + 1};
        const double c[3] = {1.0 / dt1, -(1.0 / dt1 + 1.0 / dt2), 1.0 / dt2};
        motion_cost += w * r.squaredNorm();
        equations.residuals += 3;

        for (int k = 0; k < 3; k++)
        {
            for (int a = 0; a < 3; a++)
            {
                for (int b = 0; b < 3; b++)
                    AtPAndt.add(nodes[a] * 6 + k, nodes[b] * 6 + k, w * c[a] * c[b]);
                AtPBndt(nodes[a] * 6 + k, 0) -= w * c[a] * r(k);
            }
        }
    }

    // tie the first step of this window to the final velocity of the previous window
    if (velocity_prior.use && window.poses.size() > 1)
    {
        const double dt = std::max(window.timestamps[1].first - window.timestamps[0].first, min_dt);
        const double sigma_dv = motion_model.acc_sigma_mps2 * dt;
        const double w = 1.0 / (sigma_dv * sigma_dv);

        const Eigen::Vector3d r = (window.poses[1].translation() - window.poses[0].translation()) / dt - velocity_prior.velocity;

        const int nodes[2] = {0, 1};
        const double c[2] = {-1.0 / dt, 1.0 / dt};
        motion_cost += w * r.squaredNorm();
        equations.residuals += 3;

        for (int k = 0; k < 3; k++)
        {
            for (int a = 0; a < 2; a++)
            {
                for (int b = 0; b < 2; b++)
                    AtPAndt.add(nodes[a] * 6 + k, nodes[b] * 6 + k, w * c[a] * c[b]);
                AtPBndt(nodes[a] * 6 + k, 0) -= w * c[a] * r(k);
            }
        }
    }
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
            const double sqrt_dt = std::sqrt(dt_i);
            const double s_om = std::max(rotation_options.inflation_k * rotation_options.arw_rad_sqrt_s.x() * sqrt_dt, rotation_options.sigma_floor_rad);
            const double s_fi = std::max(rotation_options.inflation_k * rotation_options.arw_rad_sqrt_s.y() * sqrt_dt, rotation_options.sigma_floor_rad);
            const double s_ka = std::max(rotation_options.inflation_k * rotation_options.arw_rad_sqrt_s.z() * sqrt_dt, rotation_options.sigma_floor_rad);
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
    // the motion model's relative pose from node a to node b (the IMU's), with the frame correction and, with bias
    // estimation, minus the bias update over the edge's duration
    Eigen::Matrix<double, 6, 1> edge_measurement(const TaitBryanPose &a, const TaitBryanPose &b, const MotionModelOptions &motion_model,
                                                 bool estimate_bias, const Eigen::Vector3d &bias_update, double dt)
    {
        Eigen::Matrix<double, 6, 1> m;
        relative_pose_tait_bryan_wc_case1_simplified_1(m, a.px, a.py, a.pz, a.om, a.fi, a.ka, b.px, b.py, b.pz, b.om, b.fi, b.ka);

        m(3, 0) += motion_model.correction.om * DEG_TO_RAD;
        m(4, 0) += motion_model.correction.fi * DEG_TO_RAD;
        m(5, 0) += motion_model.correction.ka * DEG_TO_RAD;

        if (estimate_bias)
        {
            // the IMU increment contains bias * dt (body frame of node i, small angle); apply_gyro_bias_prior already
            // removed the carried bias (the prior mean) from motion_model_poses, so only the update is subtracted here
            m(3, 0) -= bias_update.x() * dt;
            m(4, 0) -= bias_update.y() * dt;
            m(5, 0) -= bias_update.z() * dt;
        }
        return m;
    }

    // the edge's 12x12 block and right-hand side between the columns of nodes a (ic_1) and b (ic_2)
    void add_edge_block(WindowEquations &equations, const TaitBryanPose &a, const TaitBryanPose &b, int ic_1, int ic_2,
                        const Eigen::Matrix<double, 6, 1> &m, const MotionEdgeWeights &weights, const Eigen::Vector3d &w_rot)
    {
        Eigen::Matrix<double, 12, 12> AtPAodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPA_simplified(AtPAodo, a.px, a.py, a.pz, a.om, a.fi, a.ka, b.px, b.py, b.pz, b.om, b.fi, b.ka,
                                                                 weights.x, weights.y, weights.z, w_rot.x(), w_rot.y(), w_rot.z());
        Eigen::Matrix<double, 12, 1> AtPBodo;
        relative_pose_obs_eq_tait_bryan_wc_case1_AtPB_simplified(AtPBodo, a.px, a.py, a.pz, a.om, a.fi, a.ka, b.px, b.py, b.pz, b.om, b.fi, b.ka,
                                                                 m(0, 0), m(1, 0), m(2, 0), m(3, 0), m(4, 0), m(5, 0),
                                                                 weights.x, weights.y, weights.z, w_rot.x(), w_rot.y(), w_rot.z());
        for (int row = 0; row < 6; row++)
        {
            for (int col = 0; col < 6; col++)
            {
                equations.matrix.add(ic_1 + row, ic_1 + col, AtPAodo(row, col));
                equations.matrix.add(ic_1 + row, ic_2 + col, AtPAodo(row, col + 6));
                equations.matrix.add(ic_2 + row, ic_1 + col, AtPAodo(row + 6, col));
                equations.matrix.add(ic_2 + row, ic_2 + col, AtPAodo(row + 6, col + 6));
            }
        }
        for (int row = 0; row < 6; row++)
        {
            equations.rhs(ic_1 + row, 0) -= AtPBodo(row, 0);
            equations.rhs(ic_2 + row, 0) -= AtPBodo(row + 6, 0);
        }
    }

    // the edge's cost: its weighted squared residual
    double edge_cost(const TaitBryanPose &p1, const TaitBryanPose &p2, const Eigen::Matrix<double, 6, 1> &m, const MotionEdgeWeights &weights,
                     const Eigen::Vector3d &w_rot)
    {
        Eigen::Matrix<double, 6, 1> d;
        relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(d, p1.px, p1.py, p1.pz, p1.om, p1.fi, p1.ka, p2.px, p2.py, p2.pz, p2.om, p2.fi, p2.ka,
                                                              m(0, 0), m(1, 0), m(2, 0), m(3, 0), m(4, 0), m(5, 0));
        return weights.x * d(0) * d(0) + weights.y * d(1) * d(1) + weights.z * d(2) * d(2) + w_rot.x() * d(3) * d(3) + w_rot.y() * d(4) * d(4) +
               w_rot.z() * d(5) * d(5);
    }

    // with bias estimation: the rotation rows of delta = (m - (b - b0)*dt) - f(pose_1, pose_2), with d(delta)/d(pose) = J
    // and d(delta)/d(b) = -dt * I, couple the edge's poses to the bias unknowns at column ib
    void add_edge_bias_coupling(WindowEquations &equations, int ib, const TaitBryanPose &a, const TaitBryanPose &b, int ic_1, int ic_2,
                                const Eigen::Matrix<double, 6, 1> &m, const Eigen::Vector3d &w_rot, double dt_i)
    {
        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> J;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian_simplified_1(J, a.px, a.py, a.pz, a.om, a.fi, a.ka, b.px, b.py, b.pz, b.om, b.fi, b.ka);
        Eigen::Matrix<double, 6, 1> delta_odo;
        relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(delta_odo, a.px, a.py, a.pz, a.om, a.fi, a.ka, b.px, b.py, b.pz, b.om, b.fi, b.ka,
                                                              m(0, 0), m(1, 0), m(2, 0), m(3, 0), m(4, 0), m(5, 0));
        for (int r = 0; r < 3; r++)
        {
            // pose/bias cross terms: J_rot^T * P_rot * (-dt)
            for (int col = 0; col < 6; col++)
            {
                const double v1 = J(3 + r, col) * w_rot(r) * (-dt_i);
                const double v2 = J(3 + r, col + 6) * w_rot(r) * (-dt_i);
                equations.matrix.add(ic_1 + col, ib + r, v1);
                equations.matrix.add(ib + r, ic_1 + col, v1);
                equations.matrix.add(ic_2 + col, ib + r, v2);
                equations.matrix.add(ib + r, ic_2 + col, v2);
            }
            equations.matrix.add(ib + r, ib + r, dt_i * dt_i * w_rot(r));
            equations.rhs(ib + r, 0) -= (-dt_i) * w_rot(r) * delta_odo(3 + r, 0);
        }
    }
}

void add_motion_edges(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window, const MotionModelOptions &motion_model,
                      const MotionModelRotationOptions &rotation_options, const MotionEdgeWeights &weights)
{
    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;
    for (size_t i = 0; i < window.poses.size(); i++)
        poses.push_back(pose_tait_bryan_from_affine_matrix(window.poses[i]));
    for (size_t i = 0; i < window.motion_model_poses.size(); i++)
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(window.motion_model_poses[i]));

    for (size_t i = 0; i + 1 < window.poses.size(); i++)
    {
        const size_t first = i, second = i + 1;
        const Eigen::Vector3d bias_update = window.gyro_bias.mean - rotation_options.bias_prior_mean;
        const Eigen::Matrix<double, 6, 1> m =
            edge_measurement(poses_desired[first], poses_desired[second], motion_model, unknowns.estimate_bias, bias_update, weights.dt[i]);
        const Eigen::Vector3d &w_rot = weights.rotation[i];
        const int ic_1 = static_cast<int>(first) * 6;
        const int ic_2 = static_cast<int>(second) * 6;

        add_edge_block(equations, poses[first], poses[second], ic_1, ic_2, m, weights, w_rot);
        equations.residuals += 6;
        if (equations.with_cost)
            equations.motion_cost += edge_cost(poses[first], poses[second], m, weights, w_rot);
        if (unknowns.estimate_bias)
            add_edge_bias_coupling(equations, unknowns.ib, poses[first], poses[second], ic_1, ic_2, m, w_rot, weights.dt[i]);
    }
}

void add_motion_edge_terms(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window, const MotionModelOptions &motion_model,
                           const MotionModelRotationOptions &rotation_options, const MotionModelTranslationImuOptions &translation_imu_options)
{
    add_motion_edges(equations, unknowns, window, motion_model, rotation_options,
                     motion_edge_weights(unknowns, window, motion_model, rotation_options, translation_imu_options));
}
