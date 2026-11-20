#pragma once

// Helpers for evaluating production terms at the TRUE state (openspec change consistency-harness, design D5, tasks 5.1 and
// 5.2). Test helpers only, no production code is changed or copied: a term is called as the window problem calls it, on a
// window whose state the test sets, and its residual is read from what it accumulated.
//
// How a production residual is observed (design D5, "How a production residual is observed"):
//  - an accelerometer term only accumulates equations; its residual is local. One production span is passed at a time, on
//    a sub-window that holds exactly the nodes the span touches (re-indexed), with a fresh WindowEquations. The offset's
//    right-hand side of that call is  rhs_o = -w * sign * tau * r  (the offset's Jacobian is sign * tau * I, accelerometer_term.cpp
//    add_acceleration_term), so the signed residual is  r = -rhs_o / (w * sign * tau), sign = -1 for the end velocity term
//    and +1 for the others. (A test below checks that the isolation does not change the observation: the sum over the
//    isolated spans equals the offset right-hand side of the whole window.)
//  - a rotation edge: with bias estimation the edge's rotation rows couple to the bias columns, rhs_b = dt * w_rot * delta,
//    so the signed rotation residual (om, fi, ka) is  delta = rhs_b / (dt * w_rot)  (motion_model_terms.cpp
//    add_edge_bias_coupling). The edge is passed on a two-node window.
//  - a prior: rhs = information * (mean - state), so the residual is  covariance * rhs.

#include "accelerometer_term.h"
#include "motion_model_terms.h"
#include "optimizer_options.h"
#include "synthetic_inertial.h"
#include "synthetic_routes.h"
#include "window.h"
#include "window_equations.h"
#include "window_noise_model.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace terms_truth
{
// the settings the terms see: the stage 8 values where they matter to the terms
struct Options
{
    OdometryConfig config;
    ScanToMapOptions scan;
    MotionModelOptions motion;
    MotionModelRotationOptions rotation;
    MotionModelTranslationImuOptions imu; // acc is set at each use (it points into the window)
};

inline Options make_options(double span_s, const Eigen::Vector3d &arw_deg_sqrt_s)
{
    Options o;
    o.config = synthetic::front_end_config();
    OdometryConfig &c = o.config;
    c.use_time_scaled_rotation_sigma = true;
    c.gyro_arw_x_deg_sqrt_s = arw_deg_sqrt_s.x();
    c.gyro_arw_y_deg_sqrt_s = arw_deg_sqrt_s.y();
    c.gyro_arw_z_deg_sqrt_s = arw_deg_sqrt_s.z();
    c.rotation_sigma_inflation_k = 1.0;
    c.rotation_sigma_floor_deg = 0.0;
    c.use_gyro_bias_estimation = true;
    c.imu_always_on = true;
    c.imu_acceleration_span_s = span_s;
    c.imu_accel_noise_density_mps2_sqrt_hz = 0.0013;
    c.imu_accel_floor_sigma_mps2 = 0.1;
    o.scan = get_scan_to_map_options(c);
    o.motion = get_motion_model_options(c);
    o.rotation = get_motion_model_rotation_options(c);
    o.rotation.bias_prior_mean.setZero();
    o.rotation.bias_prior_cov = Eigen::Matrix3d::Identity() * 1e-4;
    MotionModelTranslationImuOptions &i = o.imu;
    i.enabled = true;
    i.gravity = Eigen::Vector3d(0.0, 0.0, 1.0);
    i.to_mps2 = synthetic::standard_gravity;
    i.span_s = span_s;
    i.offset_prior_mean.setZero();
    i.offset_prior_cov = Eigen::Matrix3d::Identity() * 0.01;
    i.noise_density_mps2_sqrt_hz = c.imu_accel_noise_density_mps2_sqrt_hz;
    i.floor_sigma_mps2 = c.imu_accel_floor_sigma_mps2;
    i.position_information_scale = 0.1;
    i.velocity_prior_mean.setZero();
    i.velocity_prior_cov = Eigen::Matrix3d::Identity();
    i.zero_velocity_sigma_mps = 0.01;
    return o;
}

// a window with its state set to the truth, and what it takes to read the truth back at any instant
struct TermWindow
{
    Window window;
    Options options;
    Eigen::Vector3d offset = Eigen::Vector3d::Zero(); // the acceleration offset the terms are evaluated at, world frame [m/s2]
    WindowNoiseModel noise;
    // the truth: the motion (unaligned), the alignment of its frame to the window's frame, and the timestamp of motion time 0
    const synthetic::Motion *motion = nullptr;
    Eigen::Matrix3d align = Eigen::Matrix3d::Identity();
    double time_offset_s = 0.0; // timestamp [s] = motion time + time_offset_s
    double dt_s = 0.002;
    Eigen::Vector3d offset_body = Eigen::Vector3d::Zero(); // the generator's body-frame offset [m/s2]

    MotionModelTranslationImuOptions imu() const
    {
        MotionModelTranslationImuOptions i = options.imu;
        i.acc = &window.imu_acc;
        return i;
    }
    void build_noise()
    {
        noise = build_window_noise_model(window, options.scan, options.motion, options.rotation, imu(), VelocityPrior{});
    }
    double motion_time(double timestamp) const { return timestamp - time_offset_s; }
    Eigen::Vector3d velocity_at(double timestamp) const { return align * motion->state(motion_time(timestamp)).velocity; }
};

// ---- the accelerometer terms ----

// the interval a term averages the acceleration over [t0, t1], and the support of its position/velocity kernel [lo, hi]
struct Support
{
    double t0, t1, lo, hi;
};

inline Support support_of(const Window &w, const AccelerometerTermSpan &s)
{
    const auto t = [&](int k) { return w.timestamps[k].first; };
    switch (s.kind)
    {
    case AccelerometerTermKind::anchor_triple:
        return {0.5 * (t(s.a) + t(s.b)), 0.5 * (t(s.b) + t(s.c)), t(s.a), t(s.c)};
    case AccelerometerTermKind::start_velocity:
        return {t(0), 0.5 * (t(0) + t(s.b)), t(0), t(s.b)};
    case AccelerometerTermKind::end_velocity:
    default:
        return {0.5 * (t(s.a) + t(s.b)), t(s.c), t(s.a), t(s.c)};
    }
}

inline double term_sign(const AccelerometerTermSpan &s) { return s.kind == AccelerometerTermKind::end_velocity ? -1.0 : 1.0; }

// per sample k (the accelerometer's interval [t_k, t_k+1)): the length of its overlap with [t0, t1]; the last node has no sample
inline std::vector<double> sample_overlaps(const Window &w, double t0, double t1)
{
    const int n = static_cast<int>(w.poses.size());
    std::vector<double> l(n, 0.0);
    for (int k = 0; k + 1 < n; k++)
        l[k] = std::max(0.0, std::min(w.timestamps[k + 1].first, t1) - std::max(w.timestamps[k].first, t0));
    return l;
}

// the signed residual of one production span, observed as the header describes
inline Eigen::Vector3d observe_span(const TermWindow &tw, size_t index)
{
    const AccelerometerTermSpan &s = tw.noise.accelerometer_terms[index];
    const int n = static_cast<int>(tw.window.poses.size());
    int first = s.a, last = s.c;
    if (s.kind == AccelerometerTermKind::start_velocity)
    {
        first = 0;
        last = s.b;
    }
    const int hi = std::min(last + 1, n - 1);
    Window sub;
    for (int k = first; k <= hi; k++)
    {
        sub.poses.push_back(tw.window.poses[k]);
        sub.motion_model_poses.push_back(tw.window.poses[k]);
        sub.timestamps.push_back(tw.window.timestamps[k]);
    }
    sub.velocity = tw.window.velocity;
    std::vector<Eigen::Vector3d> sub_acc(tw.window.imu_acc.begin() + first, tw.window.imu_acc.begin() + hi + 1);
    MotionModelTranslationImuOptions imu = tw.options.imu;
    imu.acc = &sub_acc;
    WindowUnknowns u = layout_window_unknowns(sub, tw.options.rotation, imu);
    u.pose_band_offset = std::max(17, (u.n_nodes - 1) * 6 + 5);
    WindowNoiseModel noise = tw.noise;
    noise.unknowns = u;
    AccelerometerTermSpan shifted = s;
    shifted.a -= first;
    shifted.b -= first;
    shifted.c -= first;
    noise.accelerometer_terms = {shifted};
    WindowEquations eq(u);
    add_acceleration_terms(eq, noise, sub, imu, tw.offset);
    return -eq.rhs.block<3, 1>(u.io, 0) / (s.weight * term_sign(s) * s.tau);
}

// the part of the residual that is the term's own approximation of the continuous motion: the position / velocity side of
// the residual at the true positions and velocities, against the exact change of the true velocity over the term's interval
// (no samples involved). Triple: second difference of the positions minus v(m2) - v(m1); start: (p_b - p_0)/dt minus v(mid);
// end: (p_q - p_p)/dt minus v(mid). (The remainder of the residual, r - this, is what the samples add.)
inline Eigen::Vector3d quadrature_part(const TermWindow &tw, const AccelerometerTermSpan &s)
{
    const Window &w = tw.window;
    const auto p = [&](int k) { return w.poses[k].translation(); };
    const auto t = [&](int k) { return w.timestamps[k].first; };
    const Support sup = support_of(w, s);
    switch (s.kind)
    {
    case AccelerometerTermKind::anchor_triple:
        return (p(s.c) - p(s.b)) / (t(s.c) - t(s.b)) - (p(s.b) - p(s.a)) / (t(s.b) - t(s.a)) - (tw.velocity_at(sup.t1) - tw.velocity_at(sup.t0));
    case AccelerometerTermKind::start_velocity:
        return (p(s.b) - p(0)) / (t(s.b) - t(0)) - tw.velocity_at(sup.t1);
    case AccelerometerTermKind::end_velocity:
    default:
        return (p(s.b) - p(s.a)) / (t(s.b) - t(s.a)) - tw.velocity_at(sup.t0);
    }
}

// L1 = integral of |u - c| |K - U| du over the kernel's support (c its middle), K the kernel with which the position /
// velocity side of the term weighs the true acceleration, U the box with which the term's mean does:
//   triple: K the tent (u - tA)/dt1 up to tB, (tC - u)/dt2 down to tC;     U = 1 on [m1, m2]
//   start:  K = (tB - u)/dt on [t0, tB];                                     U = 1 on [t0, mid]
//   end:    K = (u - tP)/dt up to tQ, then 1 to tN;                          U = 1 on [mid, tN]
// (mean velocity over an interval = v(end) - integral of the acceleration against a ramp). Both integrate to tau, so
// Q = integral a (K - U) = integral (a - a(c)) (K - U) and |Q| <= max|jerk| * L1 (a is Lipschitz with the jerk).
inline double kernel_l1(const Window &w, const AccelerometerTermSpan &s)
{
    const auto t = [&](int k) { return w.timestamps[k].first; };
    const Support sup = support_of(w, s);
    const double c = 0.5 * (sup.lo + sup.hi);
    const auto kernel = [&](double u)
    {
        switch (s.kind)
        {
        case AccelerometerTermKind::anchor_triple:
            return u < t(s.b) ? (u - t(s.a)) / (t(s.b) - t(s.a)) : (t(s.c) - u) / (t(s.c) - t(s.b));
        case AccelerometerTermKind::start_velocity:
            return (t(s.b) - u) / (t(s.b) - t(0));
        case AccelerometerTermKind::end_velocity:
        default:
            return u < t(s.b) ? (u - t(s.a)) / (t(s.b) - t(s.a)) : 1.0;
        }
    };
    constexpr int steps = 8000;
    const double h = (sup.hi - sup.lo) / steps;
    double sum = 0.0;
    for (int i = 0; i < steps; i++)
    {
        const double u = sup.lo + (i + 0.5) * h;
        const double box = (u >= sup.t0 && u <= sup.t1) ? 1.0 : 0.0;
        sum += std::abs(u - c) * std::abs(kernel(u) - box) * h;
    }
    return sum * 1.01; // the midpoint rule on kernels with jumps
}

// extremes of the truth over [from, to] (motion times)
struct Extremes
{
    double rate = 0.0;      // |body rate| [rad/s]
    double force = 0.0;     // |specific force| + |offset| [m/s2]
    double jerk = 0.0;      // |d acceleration / dt| [m/s3]
    double rotation = 0.0;  // the largest angle between the attitude and the attitude at motion time `reference` [rad]
};

inline Extremes extremes_of(const synthetic::Motion &motion, double from, double to, double reference, double offset_magnitude)
{
    Extremes e;
    const Eigen::Matrix3d r0 = motion.state(std::max(reference, 0.0)).rotation;
    constexpr double step = 0.00025, d = 2e-4;
    for (double t = std::max(from, d); t <= to + 1e-12; t += step)
    {
        const synthetic::MotionState s = motion.state(t);
        e.rate = std::max(e.rate, s.body_rate.norm());
        e.force = std::max(e.force, s.specific_force.norm() + offset_magnitude);
        e.jerk = std::max(e.jerk, (motion.state(t + d).acceleration - motion.state(t - d).acceleration).norm() / (2.0 * d));
        e.rotation = std::max(e.rotation, Eigen::AngleAxisd(r0.transpose() * s.rotation).angle());
    }
    return e;
}

// The bounds derived for one term (the derivation is in terms_at_truth_test.cpp), split as the residual is:
//   quadrature: |Q| <= 1.01 * jerk * L1                       (the term's own approximation of the continuous motion)
//   samples:    tau * [ dt/2 * rate * force                    (a sample's rotation is the node's, at the interval's start)
//                       + 2^-24 * force                         (the accelerometer narrowed to float)
//                       + |offset| * rotation ]                 (the generator's offset is fixed in the body, the term's in the world)
//               + jerk * dt^2                                   (the two samples a term boundary cuts: whole-sample mean vs the part)
struct Bounds
{
    double quadrature = 0.0, samples = 0.0;
    double total() const { return quadrature + samples; }
};

inline Bounds derive_bounds(const TermWindow &tw, const AccelerometerTermSpan &s)
{
    const Support sup = support_of(tw.window, s);
    const double dt = tw.dt_s;
    const Extremes e = extremes_of(*tw.motion, tw.motion_time(sup.lo) - dt, tw.motion_time(sup.hi) + dt, tw.motion_time(tw.window.timestamps[0].first),
                                   tw.offset_body.norm());
    Bounds b;
    b.quadrature = 1.01 * e.jerk * kernel_l1(tw.window, s);
    b.samples = s.tau * (0.5 * dt * e.rate * e.force + std::ldexp(1.0, -24) * e.force + tw.offset_body.norm() * e.rotation) + e.jerk * dt * dt;
    return b;
}

// ---- the rotation edges ----

// The signed rotation residual (om, fi, ka) of the edge between nodes k and k + 1 [rad], observed through the bias coupling
inline Eigen::Vector3d observe_edge_rotation(const TermWindow &tw, int k, Eigen::Vector3d *weight = nullptr, double *dt = nullptr)
{
    Window sub;
    sub.poses = {tw.window.poses[k], tw.window.poses[k + 1]};
    sub.motion_model_poses = {tw.window.motion_model_poses[k], tw.window.motion_model_poses[k + 1]};
    sub.timestamps = {tw.window.timestamps[k], tw.window.timestamps[k + 1]};
    sub.gyro_bias = tw.window.gyro_bias;
    const MotionModelTranslationImuOptions no_imu;
    const WindowNoiseModel nm = build_window_noise_model(sub, tw.options.scan, tw.options.motion, tw.options.rotation, no_imu, VelocityPrior{});
    WindowEquations eq(nm.unknowns);
    add_motion_edge_terms(eq, nm, sub, tw.options.motion, tw.options.rotation, no_imu, Eigen::Vector3d::Zero());
    const double edge_dt = nm.edges.dt[0];
    const Eigen::Vector3d w = nm.edges.rotation[0];
    if (weight)
        *weight = w;
    if (dt)
        *dt = edge_dt;
    Eigen::Vector3d d;
    for (int r = 0; r < 3; r++)
        d(r) = eq.rhs(nm.unknowns.ib + r, 0) / (edge_dt * w(r));
    return d;
}
}
