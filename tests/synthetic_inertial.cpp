#include "synthetic_inertial.h"

#include "truth_statistics.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace synthetic
{
namespace
{
    constexpr double pi = 3.14159265358979323846;

    // 8-point Gauss-Legendre on [-1, 1]
    constexpr std::array<double, 4> gl_x = {0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363};
    constexpr std::array<double, 4> gl_w = {0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763};

    template <class F>
    auto gauss_legendre(double a, double b, F &&f) -> decltype(f(a))
    {
        const double mid = 0.5 * (a + b), half = 0.5 * (b - a);
        std::decay_t<decltype(f(a))> sum = f(mid) * 0.0;
        for (int i = 0; i < 4; i++)
            sum += gl_w[i] * (f(mid - half * gl_x[i]) + f(mid + half * gl_x[i]));
        return sum * half;
    }

    // mean of f over [t0, t1] with panels of at most 1 ms
    template <class F>
    auto interval_mean(double t0, double t1, F &&f) -> decltype(f(t0))
    {
        const int n = std::max(1, static_cast<int>(std::ceil((t1 - t0) / 1e-3 - 1e-9)));
        std::decay_t<decltype(f(t0))> sum = f(t0) * 0.0;
        for (int i = 0; i < n; i++)
            sum += gauss_legendre(t0 + (t1 - t0) * i / n, t0 + (t1 - t0) * (i + 1) / n, f);
        return sum / (t1 - t0);
    }

    // quintic smoothstep S(u) = 10u^3 - 15u^4 + 6u^5, its derivatives and the integral of S
    double S(double u) { return u * u * u * (10.0 + u * (-15.0 + 6.0 * u)); }
    double dS(double u) { return u * u * (30.0 + u * (-60.0 + 30.0 * u)); }
    double ddS(double u) { return u * (60.0 + u * (-180.0 + 120.0 * u)); }
    double intS(double u) { return u * u * u * u * (2.5 + u * (-3.0 + u)); }

    struct Env
    {
        double e, de, dde; // the envelope and its first two derivatives
        double integral;   // the integral of e from 0
    };

    Env envelope(double tau, double T, double ramp)
    {
        tau = std::clamp(tau, 0.0, T);
        if (tau < ramp)
        {
            const double u = tau / ramp;
            return {S(u), dS(u) / ramp, ddS(u) / (ramp * ramp), ramp * intS(u)};
        }
        if (tau > T - ramp)
        {
            const double u = (T - tau) / ramp;
            return {S(u), -dS(u) / ramp, ddS(u) / (ramp * ramp), (T - ramp) - ramp * intS(u)};
        }
        return {1.0, 0.0, 0.0, 0.5 * ramp + (tau - ramp)};
    }

    // amplitude * sin(w tau + phase) * envelope, with its first two derivatives
    std::array<double, 3> oscillation(double amplitude, double w, double phase, double tau, const Env &env)
    {
        const double s = std::sin(w * tau + phase), c = std::cos(w * tau + phase);
        const double g = amplitude * s, dg = amplitude * w * c, ddg = -amplitude * w * w * s;
        return {g * env.e, dg * env.e + g * env.de, ddg * env.e + 2.0 * dg * env.de + g * env.dde};
    }

    Eigen::Matrix3d rot_z(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }
    Eigen::Matrix3d rot_y(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
    Eigen::Matrix3d rot_x(double a) { return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
}

Motion::Motion(const Eigen::Vector3d &start_position, double start_heading_rad, const Gait &gait)
    : gait_(gait), z_(start_position.z()), v_end_(0.0), psi_end_(start_heading_rad), t_end_(0.0), p_end_(start_position.head<2>())
{
    const double deg = pi / 180.0, w1 = 2.0 * pi * 0.5 * gait.step_hz, w2 = 2.0 * pi * gait.step_hz;
    const double s2 = std::sqrt(2.0);
    ang_amp_ = {s2 * gait.rate_rms_deg_s.x() * deg / w1, s2 * gait.rate_rms_deg_s.y() * deg / w2, s2 * gait.rate_rms_deg_s.z() * deg / w1};
    const double a_surge = s2 * gait.accel_rms_mps2.x(), a_sway = s2 * gait.accel_rms_mps2.y();
    const double surge = std::sqrt(std::max(a_surge * a_surge - std::pow(standard_gravity * ang_amp_.y(), 2), 0.0));
    const double sway = std::sqrt(std::max(a_sway * a_sway - std::pow(standard_gravity * ang_amp_.x(), 2), 0.0));
    pos_amp_ = {surge / (w2 * w2), sway / (w1 * w1), s2 * gait.accel_rms_mps2.z() / (w2 * w2)};
}

void Motion::add(double duration, double ramp, double accel, double yaw_rate, bool walking)
{
    if (duration < 2.0 * ramp || ramp <= 0.0)
        throw std::invalid_argument("Motion: a segment needs duration >= 2 ramp > 0");
    if (walking && yaw_rate != 0.0)
        throw std::invalid_argument("Motion: walking segments are straight");
    Seg s;
    s.t0 = t_end_;
    s.duration = duration;
    s.ramp = ramp;
    s.accel = accel;
    s.yaw_rate = yaw_rate;
    s.walking = walking;
    s.walk_in = walking && (segs_.empty() || !segs_.back().walking);
    s.walk_out = walking; // until a walking segment follows
    if (walking && !segs_.empty())
        segs_.back().walk_out = false;
    s.v0 = v_end_;
    s.psi0 = psi_end_;
    s.p0 = p_end_;
    const int panels = std::max(1, static_cast<int>(std::ceil(duration / 0.02)));
    s.panel = duration / panels;
    s.cumulative.assign(1, Eigen::Vector2d::Zero());
    for (int i = 0; i < panels; i++)
    {
        const auto integrand = [&](double tau)
        {
            const Env e = envelope(tau, duration, ramp);
            const double v = s.v0 + accel * e.integral, psi = s.psi0 + yaw_rate * e.integral;
            return Eigen::Vector2d(v * std::cos(psi), v * std::sin(psi));
        };
        s.cumulative.push_back(s.cumulative.back() + gauss_legendre(i * s.panel, (i + 1) * s.panel, integrand));
    }
    v_end_ = s.v0 + accel * (duration - ramp);
    psi_end_ = s.psi0 + yaw_rate * (duration - ramp);
    p_end_ = s.p0 + s.cumulative.back();
    t_end_ += duration;
    segs_.push_back(std::move(s));
}

Motion &Motion::rest(double duration_s)
{
    if (std::abs(v_end_) > 1e-12)
        throw std::invalid_argument("Motion::rest: the speed is not zero");
    add(duration_s, std::min(0.5, 0.5 * duration_s), 0.0, 0.0, false);
    return *this;
}

Motion &Motion::cruise(double duration_s, bool walking)
{
    add(duration_s, std::min(0.5, 0.5 * duration_s), 0.0, 0.0, walking);
    return *this;
}

Motion &Motion::change_speed(double dv, double duration_s, double ramp_s, bool walking)
{
    add(duration_s, ramp_s, dv / (duration_s - ramp_s), 0.0, walking);
    return *this;
}

Motion &Motion::turn(double dpsi, double duration_s, double ramp_s)
{
    add(duration_s, ramp_s, 0.0, dpsi / (duration_s - ramp_s), false);
    return *this;
}

double Motion::duration() const { return t_end_; }
double Motion::end_speed() const { return v_end_; }
double Motion::end_heading() const { return psi_end_; }
Eigen::Vector3d Motion::end_position() const { return {p_end_.x(), p_end_.y(), z_}; }

double Motion::first_motion_time() const
{
    for (const Seg &s : segs_)
        if (s.v0 != 0.0 || s.accel != 0.0 || s.yaw_rate != 0.0)
            return s.t0;
    return t_end_;
}

bool Motion::at_rest(double t0, double t1) const
{
    for (const Seg &s : segs_)
        if (s.t0 < t1 && s.t0 + s.duration > t0 && (s.v0 != 0.0 || s.accel != 0.0 || s.yaw_rate != 0.0 || s.walking))
            return false;
    return true;
}

const Motion::Seg &Motion::segment_at(double t) const
{
    if (segs_.empty() || t > t_end_ + 1e-9)
        throw std::out_of_range("Motion: time beyond the end of the motion");
    // before the start the body stays in its start state
    const auto it = std::upper_bound(segs_.begin(), segs_.end(), t, [](double v, const Seg &s) { return v < s.t0; });
    return it == segs_.begin() ? segs_.front() : *(it - 1);
}

Eigen::Vector2d Motion::planar_position(const Seg &s, double tau) const
{
    const int panels = static_cast<int>(s.cumulative.size()) - 1;
    const int i = std::clamp(static_cast<int>(tau / s.panel), 0, panels - 1);
    const auto integrand = [&](double u)
    {
        const Env e = envelope(u, s.duration, s.ramp);
        const double v = s.v0 + s.accel * e.integral, psi = s.psi0 + s.yaw_rate * e.integral;
        return Eigen::Vector2d(v * std::cos(psi), v * std::sin(psi));
    };
    return s.p0 + s.cumulative[i] + gauss_legendre(i * s.panel, tau, integrand);
}

MotionState Motion::state(double t) const
{
    const Seg &s = segment_at(t);
    const double tau = std::clamp(t - s.t0, 0.0, s.duration);
    const Env e = envelope(tau, s.duration, s.ramp);
    const double v = s.v0 + s.accel * e.integral, psi = s.psi0 + s.yaw_rate * e.integral;
    const double a_t = s.accel * e.e, r = s.yaw_rate * e.e;
    const double c = std::cos(psi), sn = std::sin(psi);

    const Eigen::Vector2d p = planar_position(s, tau);
    Eigen::Vector3d pos(p.x(), p.y(), z_), vel(v * c, v * sn, 0.0), acc(a_t * c - v * r * sn, a_t * sn + v * r * c, 0.0);
    double heading = psi, heading_rate = r, pitch = 0.0, pitch_rate = 0.0, roll = 0.0, roll_rate = 0.0;

    if (s.walking)
    {
        // the gait fades in at the first of a run of walking segments and out at the last; its phase runs on the global time
        Env g{1.0, 0.0, 0.0, 0.0};
        if (s.walk_in && tau < s.ramp)
            g = envelope(tau, 2.0 * s.ramp, s.ramp);
        else if (s.walk_out && tau > s.duration - s.ramp)
            g = envelope(tau - (s.duration - 2.0 * s.ramp), 2.0 * s.ramp, s.ramp);
        const double w1 = pi * gait_.step_hz, w2 = 2.0 * pi * gait_.step_hz;
        const double tg = s.t0 + tau;
        const auto surge = oscillation(pos_amp_.x(), w2, 0.3 + 0.5 * pi, tg, g);
        const auto sway = oscillation(pos_amp_.y(), w1, 0.5 * pi, tg, g);
        const auto bob = oscillation(pos_amp_.z(), w2, 0.7, tg, g);
        const Eigen::Matrix3d heading_frame = rot_z(s.psi0);
        pos += heading_frame * Eigen::Vector3d(surge[0], sway[0], bob[0]);
        vel += heading_frame * Eigen::Vector3d(surge[1], sway[1], bob[1]);
        acc += heading_frame * Eigen::Vector3d(surge[2], sway[2], bob[2]);
        const auto d_roll = oscillation(ang_amp_.x(), w1, 0.0, tg, g);
        const auto d_pitch = oscillation(ang_amp_.y(), w2, 0.3, tg, g);
        const auto d_yaw = oscillation(ang_amp_.z(), w1, 1.1, tg, g);
        roll = d_roll[0];
        roll_rate = d_roll[1];
        pitch = d_pitch[0];
        pitch_rate = d_pitch[1];
        heading += d_yaw[0];
        heading_rate += d_yaw[1];
    }

    MotionState out;
    out.position = pos;
    out.velocity = vel;
    out.acceleration = acc;
    out.rotation = rot_z(heading) * rot_y(pitch) * rot_x(roll);
    const double sp = std::sin(pitch), cp = std::cos(pitch), sr = std::sin(roll), cr = std::cos(roll);
    // the body rate of a Z-Y-X rotation sequence (yaw, pitch, roll)
    out.body_rate = Eigen::Vector3d(roll_rate - heading_rate * sp, pitch_rate * cr + heading_rate * sr * cp, heading_rate * cr * cp - pitch_rate * sr);
    out.specific_force = out.rotation.transpose() * (acc + Eigen::Vector3d(0.0, 0.0, standard_gravity));
    return out;
}

Eigen::Vector3d Motion::mean_body_rate(double t0, double t1) const
{
    return interval_mean(t0, t1, [&](double t) { return state(t).body_rate; });
}

Eigen::Vector3d Motion::mean_specific_force(double t0, double t1) const
{
    return interval_mean(t0, t1, [&](double t) { return state(t).specific_force; });
}

ImuNoise ImuNoise::none()
{
    ImuNoise n;
    n.gyro_arw_deg_sqrt_s.setZero();
    n.accel_density_mps2_sqrt_hz = 0.0;
    n.gyro_bias_rw_deg_s_sqrt_s = 0.0;
    n.accel_offset_rw_mps2_sqrt_s = 0.0;
    return n;
}

int ImuRecording::bias_index(int node) const
{
    if (node < options.window_origin_node)
        return 0;
    return 1 + (node - options.window_origin_node) / options.nodes_per_window;
}

Truth ImuRecording::truth_at_node(int k) const
{
    Truth t;
    t.motion = motion.state(node_time(k));
    t.gyro_bias = gyro_bias_rad_s[bias_index(k)];
    t.accel_offset = accel_offset_mps2[bias_index(k)];
    return t;
}

Truth ImuRecording::truth_at(double motion_time) const
{
    const int k = std::clamp(static_cast<int>(std::floor(motion_time / dt_s() + 1e-9)), 0, n_nodes() - 1);
    Truth t = truth_at_node(k);
    t.motion = motion.state(motion_time);
    return t;
}

ImuRecording generate_imu(const Motion &motion, const ImuOptions &options)
{
    ImuRecording r(motion);
    r.options = options;
    const double dt = options.dt_ns * 1e-9;
    const int fits = static_cast<int>(std::floor(motion.duration() / dt + 1e-9));
    const int n = options.n_samples > 0 ? options.n_samples : fits;
    if (n > fits || n < 1)
        throw std::invalid_argument("generate_imu: the motion is too short for the samples (the last accelerometer interval must end inside it)");

    const std::uint64_t mix = 0x9E3779B97F4A7C15ULL;
    truth_stats::Rng gyro_rng(options.seed * mix + 1), acc_rng(options.seed * mix + 2), bias_rng(options.seed * mix + 3), offset_rng(options.seed * mix + 4);
    const ImuNoise &noise = options.noise;
    const double deg = pi / 180.0;

    // the bias and offset tables: [0] before window 0, [1] window 0 (the same), then one step per window
    const int windows = std::max(0, (n - options.window_origin_node + options.nodes_per_window - 1) / options.nodes_per_window);
    const double window_s = options.nodes_per_window * dt;
    r.gyro_bias_rad_s.assign(1, noise.gyro_bias0_deg_s * deg);
    r.accel_offset_mps2.assign(1, noise.accel_offset0_mps2);
    for (int w = 0; w < windows; w++)
    {
        Eigen::Vector3d b = r.gyro_bias_rad_s.back(), o = r.accel_offset_mps2.back();
        if (w > 0)
        {
            b += noise.gyro_bias_rw_deg_s_sqrt_s * deg * std::sqrt(window_s) * bias_rng.normal_vector(3);
            o += noise.accel_offset_rw_mps2_sqrt_s * std::sqrt(window_s) * offset_rng.normal_vector(3);
        }
        r.gyro_bias_rad_s.push_back(b);
        r.accel_offset_mps2.push_back(o);
    }

    r.t_ns.resize(n);
    r.t_s.resize(n);
    r.gyro_rad_s.resize(n);
    r.acc_g.resize(n);
    for (int k = 0; k < n; k++)
    {
        r.t_ns[k] = options.t0_ns + static_cast<std::int64_t>(k) * options.dt_ns;
        r.t_s[k] = static_cast<double>(r.t_ns[k]) / 1e9;
        const double t = k * dt;
        const double step = k == 0 ? options.first_step_s : dt;
        const int bi = r.bias_index(k);
        Eigen::Vector3d gyro = motion.mean_body_rate(t - step, t) + r.gyro_bias_rad_s[bi];
        Eigen::Vector3d acc = motion.mean_specific_force(t, t + dt) + r.accel_offset_mps2[bi];
        for (int a = 0; a < 3; a++)
        {
            gyro[a] += noise.gyro_arw_deg_sqrt_s[a] * deg / std::sqrt(step) * gyro_rng.normal();
            acc[a] += noise.accel_density_mps2_sqrt_hz / std::sqrt(dt) * acc_rng.normal();
        }
        r.gyro_rad_s[k] = gyro;
        r.acc_g[k] = acc / standard_gravity;
    }
    return r;
}
}
