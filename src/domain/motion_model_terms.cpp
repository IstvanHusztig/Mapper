#include "motion_model_terms.h"
#include "angle_units.h"
#include <transformations.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>

#include <algorithm>

void add_gyro_bias_prior(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window)
{
    if (!noise.unknowns.estimate_bias)
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    const int ib = noise.unknowns.ib;
    // bias prior carried over from the previous window
    const Eigen::Matrix3d &prior_inf = noise.bias_prior_information;
    const Eigen::Vector3d prior_delta = noise.bias_prior_mean - window.gyro_bias.mean;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            AtPAndt.add(ib + r, ib + c, prior_inf(r, c));
    AtPBndt.block<3, 1>(ib, 0) += prior_inf * prior_delta;
    equations.motion_cost += prior_delta.dot(prior_inf * prior_delta);
    equations.residuals += 3;
}

void add_velocity_continuity(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window)
{
    if (!noise.velocity_continuity)
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    double &motion_cost = equations.motion_cost;
    constexpr double min_dt = 1e-4;

    for (size_t i = 1; i + 1 < window.poses.size(); i++)
    {
        const double dt1 = std::max(window.timestamps[i].first - window.timestamps[i - 1].first, min_dt);
        const double dt2 = std::max(window.timestamps[i + 1].first - window.timestamps[i].first, min_dt);
        const double w = noise.velocity_continuity_weights[i - 1];

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
    if (noise.first_step_prior)
    {
        const double dt = std::max(window.timestamps[1].first - window.timestamps[0].first, min_dt);
        const double w = noise.first_step_weight;

        const Eigen::Vector3d r = (window.poses[1].translation() - window.poses[0].translation()) / dt - noise.first_step_velocity;

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

    // imu_edges_follow_state (change honest-inertial-carry, design D1), in a window whose guess was integrated from the
    // accelerometer: the guess is linear in the start velocity v and the acceleration offset e (integrate_window_translations;
    // the accelerometer term reads mean acceleration - e), and for another v, e node k moves by (v - v_guess)(t_k - t_0) -
    // e (t_k - t_0)^2 / 2. So edge k-1 -> k, whose position rows measure R_k-1^T (p_k - p_k-1) of the guess, measures
    //   m_k(v, e) = m_k(guess) + R_guess,k-1^T [(v - v_guess) dt_k - e ((t_k - t_0)^2 - (t_k-1 - t_0)^2) / 2]
    struct EdgeStateFollow
    {
        bool active = false;
        int iv = 0, io = 0;
        Eigen::Vector3d dv = Eigen::Vector3d::Zero(); // v - v_guess [m/s]
        Eigen::Vector3d e = Eigen::Vector3d::Zero();  // [m/s^2]
    };

    EdgeStateFollow edge_state_follow(const WindowUnknowns &unknowns, const Window &window, const MotionModelTranslationImuOptions &translation,
                                      const Eigen::Vector3d &acc_offset)
    {
        EdgeStateFollow f;
        f.active = unknowns.acc_term && translation.enabled && translation.edges_follow_state && translation.guess_integrated &&
                   !translation.at_rest && window.timestamps.size() == window.poses.size() &&
                   window.motion_model_poses.size() == window.poses.size();
        if (f.active)
        {
            f.iv = unknowns.iv;
            f.io = unknowns.io;
            f.dv = window.velocity.start - translation.velocity_guess;
            f.e = acc_offset;
        }
        return f;
    }

    // d(m_k position)/dv and d(m_k position)/de of edge `first` -> first + 1 (the formula above)
    struct EdgeStateJacobian
    {
        Eigen::Matrix3d v, e;
    };

    EdgeStateJacobian edge_state_jacobian(const Window &window, size_t first)
    {
        const double t0 = window.timestamps.front().first;
        const double ta = window.timestamps[first].first - t0, tb = window.timestamps[first + 1].first - t0;
        const double dt = std::max(tb - ta, 0.0);
        const double s = 0.5 * (tb * tb - ta * ta);
        const Eigen::Matrix3d Rt = window.motion_model_poses[first].linear().transpose();
        return {Rt * dt, -Rt * s};
    }

    // the edge's position rows against the v and e columns: delta = m(v, e) - f(pose_1, pose_2), d(delta)/d(pose) = J,
    // d(delta)/d(v, e) = the edge's state Jacobian
    void add_edge_state_coupling(WindowEquations &equations, const EdgeStateFollow &follow, const EdgeStateJacobian &js, const TaitBryanPose &a,
                                 const TaitBryanPose &b, int ic_1, int ic_2, const Eigen::Matrix<double, 6, 1> &m, const MotionEdgeWeights &weights)
    {
        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> J;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian_simplified_1(J, a.px, a.py, a.pz, a.om, a.fi, a.ka, b.px, b.py, b.pz, b.om, b.fi, b.ka);
        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1_simplified_1(delta, a.px, a.py, a.pz, a.om, a.fi, a.ka, b.px, b.py, b.pz, b.om, b.fi, b.ka,
                                                              m(0, 0), m(1, 0), m(2, 0), m(3, 0), m(4, 0), m(5, 0));
        const Eigen::Vector3d w(weights.x, weights.y, weights.z);
        Eigen::Matrix<double, 3, 6> Js;
        Js << js.v, js.e;
        const int columns[6] = {follow.iv, follow.iv + 1, follow.iv + 2, follow.io, follow.io + 1, follow.io + 2};
        const Eigen::Matrix<double, 3, 6> WJs = w.asDiagonal() * Js;
        for (int c = 0; c < 6; c++)
        {
            for (int col = 0; col < 6; col++)
            {
                const double v1 = J.block<3, 1>(0, col).dot(WJs.col(c));
                const double v2 = J.block<3, 1>(0, col + 6).dot(WJs.col(c));
                equations.matrix.add(ic_1 + col, columns[c], v1);
                equations.matrix.add(columns[c], ic_1 + col, v1);
                equations.matrix.add(ic_2 + col, columns[c], v2);
                equations.matrix.add(columns[c], ic_2 + col, v2);
            }
            for (int c2 = 0; c2 < 6; c2++)
                equations.matrix.add(columns[c], columns[c2], Js.col(c).dot(WJs.col(c2)));
            equations.rhs(columns[c], 0) -= WJs.col(c).dot(delta.head<3>());
        }
    }
}

void add_motion_edges(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window, const MotionModelOptions &motion_model,
                      const MotionModelRotationOptions &rotation_options, const MotionModelTranslationImuOptions &translation_imu_options,
                      const Eigen::Vector3d &acc_offset, const MotionEdgeWeights &weights)
{
    const EdgeStateFollow follow = edge_state_follow(unknowns, window, translation_imu_options, acc_offset);
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
        Eigen::Matrix<double, 6, 1> m =
            edge_measurement(poses_desired[first], poses_desired[second], motion_model, unknowns.estimate_bias, bias_update, weights.dt[i]);
        EdgeStateJacobian js;
        if (follow.active)
        {
            js = edge_state_jacobian(window, first);
            m.head<3>() += js.v * follow.dv + js.e * follow.e;
        }
        const Eigen::Vector3d &w_rot = weights.rotation[i];
        const int ic_1 = static_cast<int>(first) * 6;
        const int ic_2 = static_cast<int>(second) * 6;

        add_edge_block(equations, poses[first], poses[second], ic_1, ic_2, m, weights, w_rot);
        equations.residuals += 6;
        if (equations.with_cost)
            equations.motion_cost += edge_cost(poses[first], poses[second], m, weights, w_rot);
        if (unknowns.estimate_bias)
            add_edge_bias_coupling(equations, unknowns.ib, poses[first], poses[second], ic_1, ic_2, m, w_rot, weights.dt[i]);
        if (follow.active)
            add_edge_state_coupling(equations, follow, js, poses[first], poses[second], ic_1, ic_2, m, weights);
    }
}

void add_motion_edge_terms(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window, const MotionModelOptions &motion_model,
                           const MotionModelRotationOptions &rotation_options, const MotionModelTranslationImuOptions &translation_imu_options,
                           const Eigen::Vector3d &acc_offset)
{
    add_motion_edges(equations, noise.unknowns, window, motion_model, rotation_options, translation_imu_options, acc_offset, noise.edges);
}
