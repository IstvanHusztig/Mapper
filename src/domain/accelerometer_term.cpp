#include "accelerometer_term.h"
#include <transformations.h>

#include <algorithm>
#include <utility>
#include <vector>

Eigen::Matrix3d tait_bryan_rotation_jacobian(const Eigen::Affine3d &m)
{
    const TaitBryanPose p = pose_tait_bryan_from_affine_matrix(m);
    constexpr double h = 1e-5;
    Eigen::Matrix3d e;
    for (int j = 0; j < 3; j++)
    {
        TaitBryanPose plus = p, minus = p;
        double *const angle_plus[3] = {&plus.om, &plus.fi, &plus.ka};
        double *const angle_minus[3] = {&minus.om, &minus.fi, &minus.ka};
        *angle_plus[j] += h;
        *angle_minus[j] -= h;
        const Eigen::Matrix3d d = affine_matrix_from_pose_tait_bryan(plus).linear() * affine_matrix_from_pose_tait_bryan(minus).linear().transpose();
        e.col(j) = Eigen::Vector3d(d(2, 1) - d(1, 2), d(0, 2) - d(2, 0), d(1, 0) - d(0, 1)) * 0.5 / (2.0 * h);
    }
    return e;
}

namespace
{
    Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d s;
        s << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
        return s;
    }

    // per node: the specific force in the world frame (R f), the acceleration (R f - g) * to_mps2, and the rotation Jacobian
    struct AccelerometerSamples
    {
        std::vector<Eigen::Vector3d> rf, a;
        std::vector<Eigen::Matrix3d> e_rot;
    };

    AccelerometerSamples accelerometer_samples(const Window &window, const MotionModelTranslationImuOptions &translation_imu_options)
    {
        const auto &acc = *translation_imu_options.acc;
        const int n = static_cast<int>(window.poses.size());
        const double s = translation_imu_options.to_mps2;
        AccelerometerSamples samples{std::vector<Eigen::Vector3d>(n), std::vector<Eigen::Vector3d>(n), std::vector<Eigen::Matrix3d>(n)};
        for (int k = 0; k < n; k++)
        {
            samples.rf[k] = Eigen::Quaterniond(window.poses[k].linear()).normalized() * acc[k];
            samples.a[k] = (samples.rf[k] - translation_imu_options.gravity) * s;
            samples.e_rot[k] = tait_bryan_rotation_jacobian(window.poses[k]);
        }
        return samples;
    }

    // what every acceleration term of a window shares
    struct AccTermContext
    {
        WindowEquations &equations;
        const WindowUnknowns &unknowns;
        const Window &window;
        const AccelerometerSamples &samples;
        const Eigen::Vector3d &acc_offset;
        double to_mps2;
    };

    // one term: residual r = sum(c_j * p_j) + c_v * v - sign * (mean acceleration - e) * tau [m/s] over [t0, t1), with the
    // node rotations of nodes first..last
    struct AccTermShape
    {
        std::vector<std::pair<int, double>> translation; // node, coefficient
        int velocity_column;                             // iv, ie, or -1
        double c_v;
        double t0, t1;
        int first, last;
        double sign, tau;
        double w; // the term's weight, from the noise model
    };

    void add_acceleration_term(const AccTermContext &x, const AccTermShape &shape)
    {
        const Window &window = x.window;
        const int n = static_cast<int>(window.poses.size());
        // time weights of the nodes over [t0, t1)
        std::vector<std::pair<int, double>> weights;
        double weight = 0.0;
        for (int k = shape.first; k <= shape.last && k + 1 < n; k++)
        {
            const double s0 = std::max(window.timestamps[k].first, shape.t0);
            const double s1 = std::min(window.timestamps[k + 1].first, shape.t1);
            if (s1 > s0)
            {
                weights.push_back({k, s1 - s0});
                weight += s1 - s0;
            }
        }
        if (weight <= 0.0 || shape.tau <= 0.0)
            return;
        Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
        for (auto &[k, wk] : weights)
        {
            wk /= weight;
            mean_acc += x.samples.a[k] * wk;
        }
        Eigen::Vector3d r = -shape.sign * (mean_acc - x.acc_offset) * shape.tau;
        for (const auto &[node, c] : shape.translation)
            r += c * window.poses[node].translation();
        if (shape.velocity_column >= 0)
            r += shape.c_v * (shape.velocity_column == x.unknowns.iv ? window.velocity.start : window.velocity.end);

        // Jacobian blocks (3x3 each, by parameter group)
        std::vector<std::pair<int, Eigen::Matrix3d>> blocks;
        for (const auto &[node, c] : shape.translation)
            blocks.push_back({node * 6, Eigen::Matrix3d::Identity() * c});
        for (const auto &[k, wk] : weights)
            blocks.push_back({k * 6 + 3, shape.sign * shape.tau * wk * x.to_mps2 * skew(x.samples.rf[k]) * x.samples.e_rot[k]});
        blocks.push_back({x.unknowns.io, Eigen::Matrix3d::Identity() * (shape.sign * shape.tau)});
        if (shape.velocity_column >= 0)
            blocks.push_back({shape.velocity_column, Eigen::Matrix3d::Identity() * shape.c_v});

        const double w = shape.w;
        x.equations.motion_cost += w * r.squaredNorm();
        x.equations.residuals += 3;
        for (const auto &[ca, ja] : blocks)
        {
            const Eigen::Vector3d g = w * ja.transpose() * r;
            for (int p = 0; p < 3; p++)
                x.equations.rhs(ca + p, 0) -= g(p);
            for (const auto &[cb, jb] : blocks)
            {
                const Eigen::Matrix3d h = w * ja.transpose() * jb;
                for (int p = 0; p < 3; p++)
                    for (int q = 0; q < 3; q++)
                        x.equations.matrix.add(ca + p, cb + q, h(p, q));
            }
        }
    }

    // one listed term as its shape: the anchor triples (the change of velocity between consecutive anchor pairs against
    // the mean acceleration between their midpoints), the start velocity (the mean velocity over [node 0, first anchor] is
    // v_start + the mean acceleration over its first half * dt / 2), the end velocity (the mean velocity over the last two
    // anchors is v_end - the mean acceleration from their midpoint to the last node * that time)
    AccTermShape shape_of(const AccTermContext &x, const AccelerometerTermSpan &span)
    {
        const Window &window = x.window;
        switch (span.kind)
        {
        case AccelerometerTermKind::anchor_triple:
        {
            const int A = span.a, B = span.b, C = span.c;
            const double tA = window.timestamps[A].first, tB = window.timestamps[B].first, tC = window.timestamps[C].first;
            const double dt1 = tB - tA, dt2 = tC - tB;
            const double m1 = 0.5 * (tA + tB), m2 = 0.5 * (tB + tC);
            return {{{A, 1.0 / dt1}, {B, -(1.0 / dt1 + 1.0 / dt2)}, {C, 1.0 / dt2}}, -1, 0.0, m1, m2, A, C, 1.0, span.tau, span.weight};
        }
        case AccelerometerTermKind::start_velocity:
        {
            const int B = span.b;
            const double t0 = window.timestamps[0].first, tB = window.timestamps[B].first, dt = tB - t0;
            return {{{0, -1.0 / dt}, {B, 1.0 / dt}}, x.unknowns.iv, -1.0, t0, 0.5 * (t0 + tB), 0, B, 1.0, span.tau, span.weight};
        }
        case AccelerometerTermKind::end_velocity:
        default:
        {
            const int P = span.a, Q = span.b, N = span.c;
            const double tP = window.timestamps[P].first, tQ = window.timestamps[Q].first, tN = window.timestamps[N].first, dt = tQ - tP;
            const double mid = 0.5 * (tP + tQ);
            return {{{P, -1.0 / dt}, {Q, 1.0 / dt}}, x.unknowns.ie, -1.0, mid, tN, P, N, -1.0, span.tau, span.weight};
        }
        }
    }
}

// the accelerometer term with the node rotations, noise from the sensor and the start and end velocity as
// unknowns. One term: residual r = sum(c_j * p_j) + c_v * v - sign * (mean acceleration - e) * tau [m/s], with mean
// acceleration = time-weighted mean of (R_k f_k - g) * to_mps2 over the term's interval; the node rotations enter through
// d(R f) = -[R f]x d(theta). Weight 1 / (sigma^2 tau^2) with sigma^2 = density^2 / tau + floor^2.
bool add_acceleration_terms(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window,
                            const MotionModelTranslationImuOptions &translation_imu_options, const Eigen::Vector3d &acc_offset)
{
    if (!noise.unknowns.acc_term)
        return false;
    const AccelerometerSamples samples = accelerometer_samples(window, translation_imu_options);
    const AccTermContext x{equations, noise.unknowns, window, samples, acc_offset, translation_imu_options.to_mps2};
    for (const AccelerometerTermSpan &span : noise.accelerometer_terms)
        add_acceleration_term(x, shape_of(x, span));
    return noise.end_velocity_tied;
}

void add_start_velocity_prior(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window)
{
    if (!noise.unknowns.acc_term)
        return;
    const int iv = noise.unknowns.iv;
    const Eigen::Matrix3d &velocity_inf = noise.velocity_prior_information;
    const Eigen::Vector3d velocity_rhs = velocity_inf * (noise.velocity_prior_mean - window.velocity.start);
    equations.motion_cost += (noise.velocity_prior_mean - window.velocity.start).dot(velocity_rhs);
    equations.residuals += 3;
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
            equations.matrix.add(iv + r, iv + c, velocity_inf(r, c));
        equations.rhs(iv + r, 0) += velocity_rhs(r);
    }
}

void add_zero_velocity(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window)
{
    if (!noise.unknowns.acc_term || !noise.at_rest)
        return;
    const int iv = noise.unknowns.iv, ie = noise.unknowns.ie;
    const double w = noise.zero_velocity_weight;
    for (int r = 0; r < 3; r++)
    {
        equations.matrix.add(iv + r, iv + r, w);
        equations.rhs(iv + r, 0) -= w * window.velocity.start(r);
        equations.matrix.add(ie + r, ie + r, w);
        equations.rhs(ie + r, 0) -= w * window.velocity.end(r);
    }
    equations.motion_cost += w * (window.velocity.start.squaredNorm() + window.velocity.end.squaredNorm());
    equations.residuals += 6;
}

void add_weak_end_velocity_tie(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window, bool end_tied)
{
    if (!noise.unknowns.acc_term || end_tied)
        return;
    const int ie = noise.unknowns.ie;
    // weight weak_end_velocity_tie_weight = 1: the right-hand side and the cost are the unweighted residual
    for (int r = 0; r < 3; r++)
    {
        equations.matrix.add(ie + r, ie + r, weak_end_velocity_tie_weight);
        equations.rhs(ie + r, 0) += window.velocity.start(r) - window.velocity.end(r);
    }
    equations.motion_cost += (window.velocity.start - window.velocity.end).squaredNorm();
    equations.residuals += 3;
}

void add_velocity_priors(WindowEquations &equations, const WindowNoiseModel &noise, const Window &window, bool end_tied)
{
    add_start_velocity_prior(equations, noise, window);
    add_zero_velocity(equations, noise, window);
    add_weak_end_velocity_tie(equations, noise, window, end_tied);
}

void add_acceleration_offset_prior(WindowEquations &equations, const WindowNoiseModel &noise, const Eigen::Vector3d &acc_offset)
{
    if (!noise.unknowns.acc_term)
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    double &motion_cost = equations.motion_cost;
    const int io = noise.unknowns.io;
    // offset prior carried over from the previous window
    const Eigen::Matrix3d &prior_inf = noise.offset_prior_information;
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            AtPAndt.add(io + r, io + c, prior_inf(r, c));
    AtPBndt.block<3, 1>(io, 0) += prior_inf * (noise.offset_prior_mean - acc_offset);
    motion_cost += (noise.offset_prior_mean - acc_offset).dot(prior_inf * (noise.offset_prior_mean - acc_offset));
    equations.residuals += 3;
}
