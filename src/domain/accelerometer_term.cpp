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
        double density2; // noise density^2
        double floor2;   // vibration floor^2
        double max_dt;   // a longer stretch has a gap in the IMU samples: its terms are dropped
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

        const double w = 1.0 / ((x.density2 / shape.tau + x.floor2) * shape.tau * shape.tau);
        x.equations.motion_cost += w * r.squaredNorm();
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

    // the change of velocity between consecutive anchor pairs against the mean acceleration between their midpoints
    void add_anchor_triples(const AccTermContext &x)
    {
        const std::vector<int> &anchors = x.unknowns.anchors;
        const Window &window = x.window;
        for (size_t j = 1; j + 1 < anchors.size(); j++)
        {
            const int A = anchors[j - 1], B = anchors[j], C = anchors[j + 1];
            const double tA = window.timestamps[A].first, tB = window.timestamps[B].first, tC = window.timestamps[C].first;
            const double dt1 = tB - tA, dt2 = tC - tB;
            if (dt1 > x.max_dt || dt2 > x.max_dt)
                continue;
            const double m1 = 0.5 * (tA + tB), m2 = 0.5 * (tB + tC);
            add_acceleration_term(x, {{{A, 1.0 / dt1}, {B, -(1.0 / dt1 + 1.0 / dt2)}, {C, 1.0 / dt2}}, -1, 0.0, m1, m2, A, C, 1.0, m2 - m1});
        }
    }

    // start velocity: the mean velocity over [node 0, first anchor] is v_start + (mean acceleration over its first half) * dt / 2
    void add_start_velocity_term(const AccTermContext &x)
    {
        const std::vector<int> &anchors = x.unknowns.anchors;
        if (anchors.size() <= 1)
            return;
        const int B = anchors[1];
        const double t0 = x.window.timestamps[0].first, tB = x.window.timestamps[B].first, dt = tB - t0;
        if (dt <= x.max_dt)
            add_acceleration_term(x, {{{0, -1.0 / dt}, {B, 1.0 / dt}}, x.unknowns.iv, -1.0, t0, 0.5 * (t0 + tB), 0, B, 1.0, 0.5 * dt});
    }

    // end velocity: the mean velocity over the last two anchors is v_end - (mean acceleration from their midpoint to the last
    // node) * (time from the midpoint to the last node). Returns whether the term was added.
    bool add_end_velocity_term(const AccTermContext &x)
    {
        const std::vector<int> &anchors = x.unknowns.anchors;
        if (anchors.size() <= 1)
            return false;
        const int n = static_cast<int>(x.window.poses.size());
        const int P = anchors[anchors.size() - 2], Q = anchors.back(), N = n - 1;
        const double tP = x.window.timestamps[P].first, tQ = x.window.timestamps[Q].first, tN = x.window.timestamps[N].first, dt = tQ - tP;
        const double mid = 0.5 * (tP + tQ);
        if (!(dt <= x.max_dt && tN - tQ <= x.max_dt))
            return false;
        add_acceleration_term(x, {{{P, -1.0 / dt}, {Q, 1.0 / dt}}, x.unknowns.ie, -1.0, mid, tN, P, N, -1.0, tN - mid});
        return true;
    }
}

// the accelerometer term with the node rotations, noise from the sensor and the start and end velocity as
// unknowns. One term: residual r = sum(c_j * p_j) + c_v * v - sign * (mean acceleration - e) * tau [m/s], with mean
// acceleration = time-weighted mean of (R_k f_k - g) * to_mps2 over the term's interval; the node rotations enter through
// d(R f) = -[R f]x d(theta). Weight 1 / (sigma^2 tau^2) with sigma^2 = density^2 / tau + floor^2.
bool add_acceleration_terms(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                            const MotionModelTranslationImuOptions &translation_imu_options, const Eigen::Vector3d &acc_offset)
{
    if (!unknowns.acc_term)
        return false;
    const AccelerometerSamples samples = accelerometer_samples(window, translation_imu_options);
    const AccTermContext x{equations,
                           unknowns,
                           window,
                           samples,
                           acc_offset,
                           translation_imu_options.to_mps2,
                           translation_imu_options.noise_density_mps2_sqrt_hz * translation_imu_options.noise_density_mps2_sqrt_hz,
                           translation_imu_options.floor_sigma_mps2 * translation_imu_options.floor_sigma_mps2,
                           2.0 * translation_imu_options.span_s};
    add_anchor_triples(x);
    add_start_velocity_term(x);
    return add_end_velocity_term(x);
}

void add_start_velocity_prior(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                              const MotionModelTranslationImuOptions &translation_imu_options)
{
    if (!unknowns.acc_term)
        return;
    const int iv = unknowns.iv;
    const Eigen::Matrix3d velocity_inf = translation_imu_options.velocity_prior_cov.inverse();
    const Eigen::Vector3d velocity_rhs = velocity_inf * (translation_imu_options.velocity_prior_mean - window.velocity.start);
    equations.motion_cost += (translation_imu_options.velocity_prior_mean - window.velocity.start).dot(velocity_rhs);
    for (int r = 0; r < 3; r++)
    {
        for (int c = 0; c < 3; c++)
            equations.matrix.add(iv + r, iv + c, velocity_inf(r, c));
        equations.rhs(iv + r, 0) += velocity_rhs(r);
    }
}

void add_zero_velocity(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                       const MotionModelTranslationImuOptions &translation_imu_options)
{
    if (!unknowns.acc_term || !translation_imu_options.at_rest)
        return;
    const int iv = unknowns.iv, ie = unknowns.ie;
    const double w = 1.0 / (translation_imu_options.zero_velocity_sigma_mps * translation_imu_options.zero_velocity_sigma_mps);
    for (int r = 0; r < 3; r++)
    {
        equations.matrix.add(iv + r, iv + r, w);
        equations.rhs(iv + r, 0) -= w * window.velocity.start(r);
        equations.matrix.add(ie + r, ie + r, w);
        equations.rhs(ie + r, 0) -= w * window.velocity.end(r);
    }
    equations.motion_cost += w * (window.velocity.start.squaredNorm() + window.velocity.end.squaredNorm());
}

void add_weak_end_velocity_tie(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window, bool end_tied)
{
    if (!unknowns.acc_term || end_tied)
        return;
    const int ie = unknowns.ie;
    for (int r = 0; r < 3; r++)
    {
        equations.matrix.add(ie + r, ie + r, 1.0);
        equations.rhs(ie + r, 0) += window.velocity.start(r) - window.velocity.end(r);
    }
    equations.motion_cost += (window.velocity.start - window.velocity.end).squaredNorm();
}

void add_velocity_priors(WindowEquations &equations, const WindowUnknowns &unknowns, const Window &window,
                         const MotionModelTranslationImuOptions &translation_imu_options, bool end_tied)
{
    add_start_velocity_prior(equations, unknowns, window, translation_imu_options);
    add_zero_velocity(equations, unknowns, window, translation_imu_options);
    add_weak_end_velocity_tie(equations, unknowns, window, end_tied);
}

void add_acceleration_offset_prior(WindowEquations &equations, const WindowUnknowns &unknowns,
                                   const MotionModelTranslationImuOptions &translation_imu_options, const Eigen::Vector3d &acc_offset)
{
    if (!unknowns.acc_term)
        return;
    BandedNormalMatrix &AtPAndt = equations.matrix;
    Eigen::MatrixXd &AtPBndt = equations.rhs;
    double &motion_cost = equations.motion_cost;
    const int io = unknowns.io;
    // offset prior carried over from the previous window
    const Eigen::Matrix3d prior_inf = translation_imu_options.offset_prior_cov.inverse();
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            AtPAndt.add(io + r, io + c, prior_inf(r, c));
    AtPBndt.block<3, 1>(io, 0) += prior_inf * (translation_imu_options.offset_prior_mean - acc_offset);
    motion_cost += (translation_imu_options.offset_prior_mean - acc_offset).dot(prior_inf * (translation_imu_options.offset_prior_mean - acc_offset));
}
