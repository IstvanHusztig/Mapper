#include "window_steps.h"
#include "window_noise_model.h"

#include "imu_kinematics.h"

#include <algorithm>

void place_window_after(Window &window, const Eigen::Affine3d &start)
{
    Eigen::Affine3d m_last = start * window.imu_motion_from_previous;
    auto tmp = window.poses;

    window.poses[0] = m_last;
    for (int k = 1; k < tmp.size(); k++)
    {
        Eigen::Affine3d m_update = tmp[k - 1].inverse() * tmp[k];
        m_last = m_last * m_update;
        window.poses[k] = m_last;
    }
}

void place_windows_after(std::vector<Window> &windows, size_t first)
{
    if (first == 0)
        return;
    for (size_t j = first; j < windows.size(); j++)
        place_window_after(windows[j], windows[j - 1].poses[windows[j - 1].poses.size() - 1]);
}

PreviousVelocity estimate_previous_velocity(const Window &previous, int span_nodes, double max_speed_mps)
{
    PreviousVelocity r;
    const auto &prev_tr = previous.poses;
    const auto &prev_ts = previous.timestamps;
    const int n = prev_tr.size();
    const int k = std::min(span_nodes, n - 1);
    if (k > 0 && prev_ts.size() == prev_tr.size())
    {
        const double dt = prev_ts[n - 1].first - prev_ts[n - 1 - k].first;
        if (dt > 0.0)
        {
            r.velocity = (prev_tr[n - 1].translation() - prev_tr[n - 1 - k].translation()) / dt;
            r.use = true;
        }
    }

    if (r.velocity.norm() > max_speed_mps)
    {
        r.dropped = true;
        r.speed = r.velocity.norm();
        r.velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
        r.use = false;
    }
    return r;
}

bool window_rests(const std::vector<Eigen::Affine3d> &poses, const std::vector<std::pair<double, double>> &timestamps,
                  const std::vector<Eigen::Vector3d> &imu_acc, const std::vector<Eigen::Vector3d> &imu_angles, double gravity_norm,
                  double acc_threshold_mps2, double gyro_threshold_rad_s)
{
    return is_window_stationary(poses, timestamps,
                                world_acceleration(poses, timestamps, imu_acc, imu_angles, gravity_norm, rest_smoothing_half_width_s),
                                acc_threshold_mps2, gyro_threshold_rad_s);
}

Eigen::Vector3d mean_shift_of(const Window &previous, const Window *before_previous, bool &dropped)
{
    Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
    if (before_previous)
    {
        mean_shift = previous.poses[previous.poses.size() - 1].translation() -
                     before_previous->poses[before_previous->poses.size() - 1].translation();
        mean_shift /= (previous.poses.size());
    }
    dropped = mean_shift.norm() > 1.0;
    if (dropped)
        mean_shift = Eigen::Vector3d(0.0, 0.0, 0.0);
    return mean_shift;
}

void guess_window_translations(std::vector<Eigen::Affine3d> &poses, const std::vector<std::pair<double, double>> &ts,
                               const std::vector<Eigen::Vector3d> &a, bool imu_sizes_ok, bool stationary, const Eigen::Vector3d &prev_velocity,
                               const Eigen::Vector3d &mean_shift, double gap_s, const PosePredictionOptions &options)
{
    if (imu_sizes_ok && (options.use_imu_pred || stationary))
    {
        // integrate the accelerometer from the previous window's final velocity;
        // a window at rest (zero-velocity detection) is predicted without motion
        Eigen::Vector3d v = stationary ? Eigen::Vector3d::Zero() : prev_velocity;
        Eigen::Vector3d p = poses[0].translation() + v * gap_s;
        poses[0].translation() = p;
        for (size_t tr = 1; tr < poses.size(); tr++)
        {
            const double dt = std::max(ts[tr].first - ts[tr - 1].first, 0.0);
            const Eigen::Vector3d a_k = (options.use_imu_pred && !stationary) ? a[tr - 1] : Eigen::Vector3d::Zero();
            p += v * dt + 0.5 * a_k * dt * dt;
            v += a_k * dt;
            poses[tr].translation() = p;
        }
    }
    else if (options.constant_velocity && ts.size() == poses.size())
    {
        // timestamp-aware constant-velocity prediction from the previous window's final velocity
        for (int tr = 0; tr < poses.size(); tr++)
            poses[tr].translation() += prev_velocity * (ts[tr].first - ts[0].first + gap_s);
    }
    else
    {
        for (int tr = 0; tr < poses.size(); tr++)
            poses[tr].translation() += mean_shift * tr;
    }
}

PosePrediction predict_window_poses(Window &window, const Window &previous, const Window *before_previous,
                                    const Eigen::Vector3d &prev_velocity, const PosePredictionOptions &options)
{
    PosePrediction r;
    const Eigen::Vector3d mean_shift = mean_shift_of(previous, before_previous, r.mean_shift_dropped);

    // the window starts after the IMU's motion since the previous window's last node (one sample interval); its first
    // position follows the prediction over that time too. An empty window (never cut) gets one node there, as before.
    if (window.poses.empty())
        window.poses.resize(1, Eigen::Affine3d::Identity());
    place_window_after(window, previous.poses[previous.poses.size() - 1]);
    const double gap_s = !window.timestamps.empty() && !previous.timestamps.empty()
                             ? std::max(window.timestamps[0].first - previous.timestamps.back().first, 0.0)
                             : 0.0;

    const auto &ts = window.timestamps;
    const auto &acc = window.imu_acc;
    const bool imu_sizes_ok = (options.use_imu_pred || options.use_zupt) && ts.size() == window.poses.size() &&
                              acc.size() == window.poses.size() && window.imu_angles.size() == window.poses.size();
    std::vector<Eigen::Vector3d> a;
    if (imu_sizes_ok)
    {
        a = world_acceleration(window.poses, ts, acc, window.imu_angles, options.gravity_norm, options.acc_smoothing_half_width_s);
        r.stationary = options.use_zupt && window_rests(window.poses, ts, acc, window.imu_angles, options.gravity_norm,
                                                        options.zupt_acc_threshold_mps2, options.zupt_gyro_threshold_rad_s);
    }

    guess_window_translations(window.poses, ts, a, imu_sizes_ok, r.stationary, prev_velocity, mean_shift, gap_s, options);
    window.motion_model_poses = window.poses;
    return r;
}

GyroBiasState apply_gyro_bias_prior(Window &window, const GyroBiasState &carried, double bias_rw_rad)
{
    const auto &ts = window.timestamps;
    const double window_duration = ts.size() > 1 ? ts.back().first - ts.front().first : 0.0;
    GyroBiasState prior;
    prior.mean = carried.mean;
    prior.covariance = random_walk_prior_cov(carried.covariance, bias_rw_rad, window_duration);
    window.gyro_bias.mean = carried.mean;
    window.gyro_bias.covariance = prior.covariance;

    // remove the carried bias from the IMU rotation increments of the initial guess (translations unchanged)
    remove_gyro_bias_from_rotations(window.poses, ts, carried.mean);
    window.motion_model_poses = window.poses;
    return prior;
}

bool integrate_window_translations(Window &window, const Eigen::Vector3d &start_velocity, const Eigen::Vector3d &gravity, double to_mps2)
{
    const size_t n = window.poses.size();
    if (n == 0 || window.imu_acc.size() != n || window.timestamps.size() != n)
        return false;
    Eigen::Vector3d p = window.poses[0].translation();
    Eigen::Vector3d v = start_velocity;
    for (size_t k = 1; k < n; k++)
    {
        const double dt = std::max(window.timestamps[k].first - window.timestamps[k - 1].first, 0.0);
        const Eigen::Vector3d a = (rotation_of(window.poses[k - 1]) * window.imu_acc[k - 1] - gravity) * to_mps2;
        p += v * dt + 0.5 * a * dt * dt;
        v += a * dt;
        window.poses[k].translation() = p;
    }
    window.motion_model_poses = window.poses;
    return true;
}

void repropagate_window_tail(Window &window, int tail_nodes, int velocity_nodes, const Eigen::Vector3d &bias_update)
{
    auto &tr = window.poses;
    const auto &imu = window.motion_model_poses;
    const auto &ts = window.timestamps;
    const int n = static_cast<int>(tr.size());
    const int m = tail_nodes;
    const int a = n - 1 - m;          // last node kept from the optimization
    if (m > 0 && velocity_nodes > 0 && a >= velocity_nodes && ts.size() == tr.size() && imu.size() == tr.size())
    {
        const double dt_v = ts[a].first - ts[a - velocity_nodes].first;
        const Eigen::Vector3d v = dt_v > 0.0 ? Eigen::Vector3d((tr[a].translation() - tr[a - velocity_nodes].translation()) / dt_v)
                                             : Eigen::Vector3d::Zero();
        const Eigen::Vector3d &db = bias_update;
        Eigen::Quaterniond q = rotation_of(tr[a]);
        for (int k = a + 1; k < n; k++)
        {
            const double dt = std::max(ts[k].first - ts[k - 1].first, 0.0);
            q = (q * relative_rotation(imu[k - 1], imu[k]) * Eigen::Quaterniond(exp_so3(-db * dt))).normalized();
            const Eigen::Vector3d t_new = tr[k - 1].translation() + v * dt;
            tr[k].linear() = q.toRotationMatrix();
            tr[k].translation() = t_new;
        }
    }
}

IterationConvergence convergence_of(const std::vector<std::vector<Eigen::Affine3d>> &poses_after, const std::vector<double> &deltas,
                                    double settle_position_m, double settle_rotation_rad, double fast_delta)
{
    IterationConvergence r;
    r.iterations = static_cast<int>(poses_after.size());
    if (poses_after.empty())
        return r;
    const std::vector<Eigen::Affine3d> &last = poses_after.back();

    // largest position and rotation difference of a pose list from the last result
    const auto difference = [&](const std::vector<Eigen::Affine3d> &poses)
    {
        std::pair<double, double> d{0.0, 0.0};
        for (size_t k = 0; k < poses.size() && k < last.size(); k++)
        {
            d.first = std::max(d.first, (poses[k].translation() - last[k].translation()).norm());
            d.second = std::max(d.second, Eigen::AngleAxisd(poses[k].linear() * last[k].linear().transpose()).angle());
        }
        return d;
    };

    r.settled_after = r.iterations;
    for (int i = r.iterations - 1; i >= 0; i--)
    {
        const auto d = difference(poses_after[i]);
        if (d.first > settle_position_m || d.second > settle_rotation_rad)
            break;
        r.settled_after = i + 1;
    }

    for (size_t i = 0; i < deltas.size() && i < poses_after.size(); i++)
        if (deltas[i] < fast_delta)
        {
            r.fast_stop_after = static_cast<int>(i) + 1;
            const auto d = difference(poses_after[i]);
            r.fast_stop_position_error_m = d.first;
            r.fast_stop_rotation_error_rad = d.second;
            break;
        }
    return r;
}
