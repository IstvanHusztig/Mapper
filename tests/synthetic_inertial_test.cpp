#include <doctest/doctest.h>

#include "synthetic_inertial.h"
#include "truth_statistics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

// The generator of the model world's motion, truth and IMU samples (synthetic_inertial.h), tested on its own before
// any fixture relies on it: the motion is smooth and its derivatives agree, the walking has the stated dynamics, an
// independent strap-down integration of the noise-free samples reproduces the truth within bounds derived below, the
// time conventions are those of the estimator, and the noise statistics match the parameters.
//
// Derivation of the discretisation bounds (dt the node interval, omega(t) the body rate, F = a + g e_z the world
// specific force, J the largest jerk, wmax / wdmax / Fmax the largest |omega|, |d omega / dt|, |F|):
//
//  1. A mean body rate integrated as one rotation. The gyro value is the mean of omega over the interval ending at a
//     node, so the estimator's rotation exp(mean(omega) dt) differs from the true R_k^T R_{k+1}, whose rotation vector
//     is the Magnus series: first term the integral of omega (the gyro value, no error), second term
//     1/2 int int [omega(s1), omega(s2)] ds2 ds1. With omega(s2) = omega(s1) - wdot (s1 - s2) + ... the commutator is at
//     most |omega| |wdot| (s1 - s2), and the double integral of (s1 - s2) is dt^3 / 6, so |second term| <=
//     dt^3 / 12 * wmax * wdmax. The later terms are smaller by a factor of the order of wmax dt (here 2.4e-3); the bound
//     uses 1.5 times the second term.
//  2. The body force rotated at the left end. The accelerometer sample holds the mean of the body force f(t) =
//     R(t)^T F(t) over [t_k, t_{k+1}), and the estimator rotates it with R_k. Writing R_k f(t) = W(s) F(t) with W(s) the
//     rotation of the body between t_k and t_k + s, the error against the mean true force is mean((I - W(s)) F(t)),
//     and |(I - W) F| <= angle(W) |F| <= wmax s |F| (a chord is shorter than its arc), whose mean over s in [0, dt]
//     is wmax Fmax dt / 2. No second-order term: the bound is exact in dt.
//  3. The position quadrature. The estimator moves a window's position by v_k dt + a dt^2 / 2 with a the (mean)
//     acceleration; the truth moves by v_k dt + int (dt - s) a(s) ds. The difference int (dt - s) (a_mean - a(s)) ds is
//     at most J dt^3 / 4 (|dt/2 - s| integrates to dt^2 / 4, |a - a_mean| <= J dt), and exactly J dt^3 / 12 for a
//     linearly changing acceleration.
//
// The strap-down check accumulates them: attitude bound B_th[k+1] = B_th[k] + step bound 1; the specific force's
// error B_a[k] = wmax Fmax dt / 2 + B_th[k] Fmax; B_v[k+1] = B_v[k] + dt B_a[k]; B_p[k+1] = B_p[k] + dt B_v[k] +
// dt^2 B_a[k] / 2 + J dt^3 / 4. Every node's error must stay below its bound.

namespace
{
    using namespace synthetic;
    constexpr double pi = 3.14159265358979323846;

    Eigen::Matrix3d exp_so3(const Eigen::Vector3d &w)
    {
        const double a = w.norm();
        Eigen::Matrix3d K;
        K << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
        if (a < 1e-12)
            return Eigen::Matrix3d::Identity() + K;
        return Eigen::Matrix3d::Identity() + std::sin(a) / a * K + (1.0 - std::cos(a)) / (a * a) * K * K;
    }

    // the rotation angle of R, from the skew part and the trace (acos of the trace loses everything below 1e-8 rad)
    double angle_of(const Eigen::Matrix3d &R)
    {
        const Eigen::Vector3d skew(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        return std::atan2(0.5 * skew.norm(), 0.5 * (R.trace() - 1.0));
    }

    // every kind of segment: rest, speeding up, constant velocity, a turn with centripetal acceleration, walking
    // (speeding up, constant, slowing down), a turn in place
    Motion mixed_motion()
    {
        Motion m(Eigen::Vector3d(1.0, 2.0, 1.5), 0.3);
        m.rest(1.0).change_speed(1.5, 2.0).cruise(1.0).turn(0.5 * pi, 2.0).cruise(2.0, true).change_speed(-1.5, 2.0, 0.5, true).rest(0.5).turn(-pi, 3.0).rest(0.5);
        return m;
    }
}

TEST_CASE("synthetic motion: derivatives agree and segments join smoothly")
{
    const Motion m = mixed_motion();
    const double h = 1e-5;
    double worst_v = 0, worst_a = 0, worst_w = 0, worst_join = 0;
    for (double t = 0.01; t < m.duration() - 0.01; t += 0.00731)
    {
        const MotionState a = m.state(t - h), c = m.state(t), b = m.state(t + h);
        worst_v = std::max(worst_v, ((b.position - a.position) / (2 * h) - c.velocity).norm());
        worst_a = std::max(worst_a, ((b.velocity - a.velocity) / (2 * h) - c.acceleration).norm());
        const Eigen::Matrix3d d = a.rotation.transpose() * b.rotation;
        const Eigen::AngleAxisd aa(d);
        worst_w = std::max(worst_w, (aa.axis() * aa.angle() / (2 * h) - c.body_rate).norm());
    }
    // a jump across any boundary between segments, measured as the change within 1e-9 s
    for (double t : {1.0, 3.0, 4.0, 6.0, 8.0, 10.0, 10.5, 13.5})
    {
        const MotionState a = m.state(t - 1e-9), b = m.state(t + 1e-9);
        worst_join = std::max({worst_join, (a.position - b.position).norm(), (a.velocity - b.velocity).norm() * 1e3, (a.acceleration - b.acceleration).norm() * 1e3,
                               (a.body_rate - b.body_rate).norm() * 1e3});
    }
    MESSAGE("position derivative " << worst_v << ", velocity derivative " << worst_a << ", body rate " << worst_w << ", join jump " << worst_join);
    CHECK(worst_v < 1e-7);
    CHECK(worst_a < 1e-6);
    CHECK(worst_w < 1e-6);
    CHECK(worst_join < 1e-4); // 1e-9 s apart: position moves by v dt <= 2e-9, the rest by the (bounded) derivative times 1e3
    CHECK(m.at_rest(0.0, 1.0));
    CHECK_FALSE(m.at_rest(0.0, 1.5));
    CHECK(m.first_motion_time() == doctest::Approx(1.0));
    CHECK(m.end_speed() == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("synthetic motion: the walking has the stated dynamics")
{
    Motion m(Eigen::Vector3d(0, 0, 1.5), 0.0);
    m.rest(1.0).change_speed(1.2, 2.0, 0.5, true).cruise(40.0, true).change_speed(-1.2, 2.0, 0.5, true).rest(1.0);
    // the steady stretch, 5 s to 40 s in 1 ms steps
    Eigen::Vector3d sum_f = Eigen::Vector3d::Zero(), sum_ff = Eigen::Vector3d::Zero(), sum_ww = Eigen::Vector3d::Zero(), sum_w = Eigen::Vector3d::Zero();
    int n = 0;
    for (double t = 5.0; t < 40.0; t += 0.001, n++)
    {
        const MotionState s = m.state(t);
        sum_f += s.specific_force;
        sum_ff += s.specific_force.cwiseProduct(s.specific_force);
        sum_ww += s.body_rate.cwiseProduct(s.body_rate);
        sum_w += s.body_rate;
    }
    Eigen::Vector3d f_rms, w_rms;
    for (int a = 0; a < 3; a++)
    {
        f_rms[a] = std::sqrt(sum_ff[a] / n - std::pow(sum_f[a] / n, 2));
        w_rms[a] = std::sqrt(sum_ww[a] / n - std::pow(sum_w[a] / n, 2)) * 180.0 / pi;
    }
    MESSAGE("force rms [m/s2] " << f_rms.transpose() << ", rate rms [deg/s] " << w_rms.transpose());
    const Gait gait;
    for (int a = 0; a < 3; a++)
    {
        CHECK(f_rms[a] == doctest::Approx(gait.accel_rms_mps2[a]).epsilon(0.10));
        CHECK(w_rms[a] == doctest::Approx(gait.rate_rms_deg_s[a]).epsilon(0.10));
    }
}

TEST_CASE("synthetic IMU: the estimator's time conventions")
{
    ImuOptions o;
    o.noise = ImuNoise::none();
    // gyro sample k is the mean rate over the interval ENDING at node k: in a turn in place, gyro_z[k] dt is the yaw
    // change from node k-1 to node k
    Motion turn(Eigen::Vector3d(0, 0, 1.5), 0.0);
    turn.rest(0.5).turn(1.2, 2.0, 0.5).rest(0.5);
    const ImuRecording rt = generate_imu(turn, o);
    const double dt = rt.dt_s();
    const auto yaw = [&](double t) { return std::atan2(turn.state(t).rotation(1, 0), turn.state(t).rotation(0, 0)); };
    double worst_gyro = 0;
    for (int k = 1; k < rt.n_nodes(); k++)
        worst_gyro = std::max(worst_gyro, std::abs(rt.gyro_rad_s[k].z() * dt - (yaw(k * dt) - yaw((k - 1) * dt))));
    // the first sample covers the 5 ms before node 0
    CHECK(rt.gyro_rad_s[0].norm() == 0.0);
    // accelerometer sample k is the mean force over the interval STARTING at node k: on a straight line the body x
    // force is the velocity change from node k to node k+1
    Motion line(Eigen::Vector3d(0, 0, 1.5), 0.0);
    line.rest(0.5).change_speed(1.5, 2.0, 0.5).cruise(1.0).change_speed(-1.5, 2.0, 0.5).rest(0.5);
    const ImuRecording rl = generate_imu(line, o);
    double worst_acc = 0;
    for (int k = 0; k < rl.n_nodes(); k++)
        worst_acc = std::max(worst_acc, std::abs(rl.acc_g[k].x() * standard_gravity - (line.state((k + 1) * dt).velocity.x() - line.state(k * dt).velocity.x()) / dt));
    MESSAGE("gyro: " << worst_gyro << " rad (yaw change), accelerometer: " << worst_acc << " m/s2");
    CHECK(worst_gyro < 1e-12);
    CHECK(worst_acc < 1e-9);
    CHECK(rl.n_nodes() == static_cast<int>(std::floor(line.duration() / dt + 1e-9)));
    // at rest the accelerometer reads +1 g up and the gyro nothing
    CHECK(rl.acc_g[0].z() == doctest::Approx(1.0).epsilon(1e-12));
    CHECK(rl.acc_g[0].head<2>().norm() < 1e-15);
    // the times are those of the source: nanoseconds as read, divided by 1e9
    CHECK(rl.t_s[3] == static_cast<double>(rl.t_ns[3]) / 1e9);
    CHECK(rl.t_ns[3] - rl.t_ns[2] == o.dt_ns);
}

TEST_CASE("synthetic IMU: an independent strap-down integration reproduces the truth within the derived bounds")
{
    const Motion m = mixed_motion();
    ImuOptions o;
    o.noise = ImuNoise::none();
    const ImuRecording rec = generate_imu(m, o);
    const double dt = rec.dt_s(), g = standard_gravity;
    const int n = rec.n_nodes();

    // the largest rates, accelerations and jerks on a 0.5 ms grid, with a 5 % margin for what falls between
    double wmax = 0, wdmax = 0, Fmax = 0, Jmax = 0;
    const double h = 1e-4;
    for (double t = h; t < n * dt; t += 5e-4)
    {
        const MotionState s = m.state(t), a = m.state(t - h), b = m.state(t + h);
        wmax = std::max(wmax, s.body_rate.norm());
        wdmax = std::max(wdmax, ((b.body_rate - a.body_rate) / (2 * h)).norm());
        Fmax = std::max(Fmax, (s.acceleration + Eigen::Vector3d(0, 0, g)).norm());
        Jmax = std::max(Jmax, ((b.acceleration - a.acceleration) / (2 * h)).norm());
    }
    wmax *= 1.05;
    wdmax *= 1.05;
    Fmax *= 1.05;
    Jmax *= 1.05;
    const double magnus = 1.5 * dt * dt * dt / 12.0 * wmax * wdmax;
    const double force_term = wmax * Fmax * dt / 2.0;
    const double quad_term = Jmax * dt * dt * dt / 4.0;

    // per-step terms
    double worst_rot = 0, worst_force = 0, worst_quad = 0;
    for (int k = 0; k + 1 < n; k++)
    {
        const MotionState s0 = m.state(k * dt), s1 = m.state((k + 1) * dt);
        const Eigen::Matrix3d integrated = exp_so3(rec.gyro_rad_s[k + 1] * dt);
        worst_rot = std::max(worst_rot, angle_of(integrated.transpose() * (s0.rotation.transpose() * s1.rotation)));
        const Eigen::Vector3d mean_a = (s1.velocity - s0.velocity) / dt;
        const Eigen::Vector3d est_a = s0.rotation * (rec.acc_g[k] * g) - Eigen::Vector3d(0, 0, g);
        worst_force = std::max(worst_force, (est_a - mean_a).norm());
        worst_quad = std::max(worst_quad, (s1.position - s0.position - s0.velocity * dt - 0.5 * mean_a * dt * dt).norm());
    }
    MESSAGE("per step: rotation " << worst_rot << " (bound " << magnus << "), force " << worst_force << " (bound " << force_term << "), quadrature " << worst_quad
                                  << " (bound " << quad_term << ")");
    CHECK(worst_rot <= magnus);
    CHECK(worst_force <= force_term);
    CHECK(worst_quad <= quad_term);

    // the strap-down integration: from node 0's truth, using only the samples
    Eigen::Matrix3d R = m.state(0.0).rotation;
    Eigen::Vector3d v = m.state(0.0).velocity, p = m.state(0.0).position;
    double Bth = 0, Bv = 0, Bp = 0;
    double max_th = 0, max_v = 0, max_p = 0, worst_ratio = 0;
    bool within = true;
    for (int k = 0; k + 1 < n; k++)
    {
        const Eigen::Vector3d a = R * (rec.acc_g[k] * g) - Eigen::Vector3d(0, 0, g);
        const double Ba = force_term + Bth * Fmax;
        p += v * dt + 0.5 * a * dt * dt;
        v += a * dt;
        R = R * exp_so3(rec.gyro_rad_s[k + 1] * dt);
        Bp += dt * Bv + 0.5 * dt * dt * Ba + quad_term;
        Bv += dt * Ba;
        Bth += magnus;
        const MotionState s = m.state((k + 1) * dt);
        const double eth = angle_of(s.rotation.transpose() * R), ev = (v - s.velocity).norm(), ep = (p - s.position).norm();
        max_th = std::max(max_th, eth);
        max_v = std::max(max_v, ev);
        max_p = std::max(max_p, ep);
        worst_ratio = std::max({worst_ratio, eth / Bth, ev / Bv, ep / Bp});
        within = within && eth <= Bth && ev <= Bv && ep <= Bp;
    }
    MESSAGE("strap-down after " << n << " nodes: attitude " << max_th << " rad (bound " << Bth << "), velocity " << max_v << " m/s (bound " << Bv << "), position " << max_p
                                << " m (bound " << Bp << "); largest error / bound " << worst_ratio);
    CHECK(within);
    // measured, then pinned with a factor of 4 (the bounds above are worst-case sums, not the errors)
    CHECK(max_th < 6e-6);
    CHECK(max_v < 1e-2);
    CHECK(max_p < 7e-2);
}

TEST_CASE("synthetic IMU: the noise statistics match the parameters")
{
    Motion m(Eigen::Vector3d(0, 0, 1.5), 0.0);
    m.rest(160.0);
    ImuOptions o;
    o.seed = 5;
    o.window_origin_node = 0;
    const ImuRecording rec = generate_imu(m, o);
    const double dt = rec.dt_s(), deg = pi / 180.0, g = standard_gravity;
    const int n = rec.n_nodes();
    const double alpha = 1e-4;

    for (int a = 0; a < 3; a++)
    {
        // gyro: white noise of sigma ARW / sqrt(dt) around the window's bias (k = 0 has the 5 ms step)
        double sum = 0, sum2 = 0;
        for (int k = 1; k < n; k++)
        {
            const double e = rec.gyro_rad_s[k][a] - rec.gyro_bias_rad_s[rec.bias_index(k)][a];
            sum += e;
            sum2 += e * e;
        }
        const double sigma = o.noise.gyro_arw_deg_sqrt_s[a] * deg / std::sqrt(dt), count = n - 1;
        const double mean = sum / count, var = (sum2 - count * mean * mean) / (count - 1);
        const auto interval = truth_stats::chi2_interval(count - 1, alpha);
        MESSAGE("gyro axis " << a << ": std ratio " << std::sqrt(var) / sigma << ", mean / sigma " << mean / sigma);
        CHECK((count - 1) * var / (sigma * sigma) > interval.first);
        CHECK((count - 1) * var / (sigma * sigma) < interval.second);
        CHECK(std::abs(mean) < 4.5 * sigma / std::sqrt(count));

        // accelerometer: white noise of D / sqrt(dt) around gravity and the window's offset
        sum = sum2 = 0;
        for (int k = 0; k < n; k++)
        {
            const double expected = (a == 2 ? g : 0.0) + rec.accel_offset_mps2[rec.bias_index(k)][a];
            const double e = rec.acc_g[k][a] * g - expected;
            sum += e;
            sum2 += e * e;
        }
        const double sigma_a = o.noise.accel_density_mps2_sqrt_hz / std::sqrt(dt), count_a = n;
        const double mean_a = sum / count_a, var_a = (sum2 - count_a * mean_a * mean_a) / (count_a - 1);
        const auto interval_a = truth_stats::chi2_interval(count_a - 1, alpha);
        MESSAGE("accelerometer axis " << a << ": std ratio " << std::sqrt(var_a) / sigma_a << ", mean / sigma " << mean_a / sigma_a);
        CHECK((count_a - 1) * var_a / (sigma_a * sigma_a) > interval_a.first);
        CHECK((count_a - 1) * var_a / (sigma_a * sigma_a) < interval_a.second);
        CHECK(std::abs(mean_a) < 4.5 * sigma_a / std::sqrt(count_a));
    }

    // the random walks: the step between windows has the variance rw^2 T (three axes pooled)
    const int windows = static_cast<int>(rec.gyro_bias_rad_s.size()) - 2; // [0] and [1] are the first window's
    double sum_gyro = 0, sum_acc = 0;
    for (int w = 2; w < static_cast<int>(rec.gyro_bias_rad_s.size()); w++)
    {
        sum_gyro += (rec.gyro_bias_rad_s[w] - rec.gyro_bias_rad_s[w - 1]).squaredNorm();
        sum_acc += (rec.accel_offset_mps2[w] - rec.accel_offset_mps2[w - 1]).squaredNorm();
    }
    const double T = o.nodes_per_window * dt;
    const double var_gyro = std::pow(o.noise.gyro_bias_rw_deg_s_sqrt_s * deg, 2) * T, var_acc = std::pow(o.noise.accel_offset_rw_mps2_sqrt_s, 2) * T;
    const auto interval = truth_stats::chi2_interval(3.0 * windows, alpha);
    MESSAGE(windows << " window steps: gyro bias variance ratio " << sum_gyro / (3.0 * windows) / var_gyro << ", offset " << sum_acc / (3.0 * windows) / var_acc);
    CHECK(sum_gyro / var_gyro > interval.first);
    CHECK(sum_gyro / var_gyro < interval.second);
    CHECK(sum_acc / var_acc > interval.first);
    CHECK(sum_acc / var_acc < interval.second);
}

TEST_CASE("synthetic IMU: bias and offset are constant inside a window, step between windows, and the seed decides")
{
    Motion m(Eigen::Vector3d(0, 0, 1.5), 0.0);
    m.rest(5.0);
    ImuOptions o;
    o.noise = ImuNoise::none();
    o.noise.gyro_bias_rw_deg_s_sqrt_s = 0.05;
    o.noise.accel_offset_rw_mps2_sqrt_s = 0.05;
    o.noise.gyro_bias0_deg_s = Eigen::Vector3d(0.1, -0.2, 0.3);
    o.window_origin_node = 100;
    o.nodes_per_window = 180;
    const ImuRecording rec = generate_imu(m, o);
    const double deg = pi / 180.0;
    int steps = 0;
    for (int k = 1; k < rec.n_nodes(); k++)
    {
        const bool same = rec.gyro_rad_s[k] == rec.gyro_rad_s[k - 1] && rec.acc_g[k] == rec.acc_g[k - 1];
        const bool boundary = k > o.window_origin_node && (k - o.window_origin_node) % o.nodes_per_window == 0;
        CHECK(same == !boundary);
        steps += boundary;
    }
    CHECK(steps > 10);
    // the first window has the initial bias, and so has everything before it
    CHECK((rec.gyro_rad_s[50] - Eigen::Vector3d(0.1, -0.2, 0.3) * deg).norm() < 1e-15);
    CHECK((rec.gyro_rad_s[150] - Eigen::Vector3d(0.1, -0.2, 0.3) * deg).norm() < 1e-15);
    CHECK(rec.truth_at_node(150).gyro_bias == rec.truth_at_node(100).gyro_bias);
    CHECK(rec.truth_at_node(300).gyro_bias != rec.truth_at_node(100).gyro_bias);
    // the same seed gives the same recording, another seed another one
    const ImuRecording again = generate_imu(m, o);
    CHECK(again.gyro_rad_s == rec.gyro_rad_s);
    ImuOptions o2 = o;
    o2.seed = 2;
    CHECK(generate_imu(m, o2).gyro_rad_s != rec.gyro_rad_s);
}
