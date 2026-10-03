#include "window_steps.h"

#include "imu_kinematics.h"

#include <algorithm>

void place_window_after(Window &window, const Eigen::Affine3d &start)
{
    Eigen::Affine3d m_last = start;
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

PreviousVelocity estimate_previous_velocity(const Window &previous, const Window &window, int span_nodes, double max_speed_mps)
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
            const auto &ts_i = window.timestamps;
            if (!ts_i.empty())
                r.age_s = std::max(ts_i[0].first - 0.5 * (prev_ts[n - 1].first + prev_ts[n - 1 - k].first), 0.0);
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

PosePrediction predict_window_poses(Window &window, const Window &previous, const Window *before_previous,
                                    const Eigen::Vector3d &prev_velocity, const PosePredictionOptions &options)
{
    PosePrediction r;
    const bool imu_translation = options.use_imu_pred || options.use_zupt;
    Eigen::Vector3d mean_shift(0.0, 0.0, 0.0);
    if (before_previous)
    {
        mean_shift = previous.poses[previous.poses.size() - 1].translation() -
                     before_previous->poses[before_previous->poses.size() - 1].translation();
        mean_shift /= (previous.poses.size());
    }

    if (mean_shift.norm() > 1.0)
    {
        r.mean_shift_dropped = true;
        mean_shift = Eigen::Vector3d(0.0, 0.0, 0.0);
    }

    std::vector<Eigen::Affine3d> new_trajectory;
    Eigen::Affine3d current_node = previous.poses[previous.poses.size() - 1];
    new_trajectory.push_back(current_node);

    for (int tr = 1; tr < window.poses.size(); tr++)
    {
        auto update = window.poses[tr - 1].inverse() * window.poses[tr];
        current_node = current_node * update;
        new_trajectory.push_back(current_node);
    }

    const auto &ts = window.timestamps;
    const auto &acc = window.imu_acc;
    const bool imu_sizes_ok = imu_translation && ts.size() == new_trajectory.size() && acc.size() == new_trajectory.size() &&
                              window.imu_angles.size() == new_trajectory.size();
    std::vector<Eigen::Vector3d> a;
    if (imu_sizes_ok)
    {
        a = world_acceleration(new_trajectory, ts, acc, window.imu_angles, options.gravity_norm, options.acc_smoothing_half_width_s);
        // rest detection on a 50 ms average: the accelerometer noise of a 10 ms average already reaches ~0.3 m/s^2
        // at rest, while walking accelerations (1-3 m/s^2 at ~2 Hz) survive the longer average
        constexpr double zupt_smoothing_half_width_s = 0.025;
        r.stationary = options.use_zupt && is_window_stationary(new_trajectory, ts,
                                                                world_acceleration(new_trajectory, ts, acc, window.imu_angles, options.gravity_norm, zupt_smoothing_half_width_s),
                                                                options.zupt_acc_threshold_mps2,
                                                                options.zupt_gyro_threshold_rad_s);
    }

    if (imu_sizes_ok && (options.use_imu_pred || r.stationary))
    {
        // integrate the accelerometer from the previous window's final velocity;
        // a window at rest (zero-velocity detection) is predicted without motion
        Eigen::Vector3d v = r.stationary ? Eigen::Vector3d::Zero() : prev_velocity;
        Eigen::Vector3d p = new_trajectory[0].translation();
        for (size_t tr = 1; tr < new_trajectory.size(); tr++)
        {
            const double dt = std::max(ts[tr].first - ts[tr - 1].first, 0.0);
            const Eigen::Vector3d a_k = (options.use_imu_pred && !r.stationary) ? a[tr - 1] : Eigen::Vector3d::Zero();
            p += v * dt + 0.5 * a_k * dt * dt;
            v += a_k * dt;
            new_trajectory[tr].translation() = p;
        }
    }
    else if (options.constant_velocity && ts.size() == new_trajectory.size())
    {
        // timestamp-aware constant-velocity prediction from the previous window's final velocity
        for (int tr = 0; tr < new_trajectory.size(); tr++)
            new_trajectory[tr].translation() += prev_velocity * (ts[tr].first - ts[0].first);
    }
    else
    {
        for (int tr = 0; tr < new_trajectory.size(); tr++)
            new_trajectory[tr].translation() += mean_shift * tr;
    }

    window.poses = new_trajectory;
    window.motion_model_poses = new_trajectory;
    return r;
}

GyroBiasState apply_gyro_bias_prior(Window &window, const GyroBiasState &carried, double bias_rw_rad)
{
    const auto &ts = window.timestamps;
    const double window_duration = ts.size() > 1 ? ts.back().first - ts.front().first : 0.0;
    GyroBiasState prior;
    prior.mean = carried.mean;
    prior.covariance = carried.covariance + Eigen::Matrix3d::Identity() * (bias_rw_rad * bias_rw_rad * window_duration);
    window.gyro_bias.mean = carried.mean;
    window.gyro_bias.covariance = prior.covariance;

    // remove the carried bias from the IMU rotation increments of the initial guess (translations unchanged)
    remove_gyro_bias_from_rotations(window.poses, ts, carried.mean);
    window.motion_model_poses = window.poses;
    return prior;
}

bool is_turn_window(const Window &window, double threshold_rad_s)
{
    const auto &tr = window.poses;
    const auto &ts = window.timestamps;
    if (tr.size() > 1 && ts.size() == tr.size())
    {
        double yaw = 0.0;
        for (size_t k = 1; k < tr.size(); k++)
        {
            // world-frame rotation increment, its component about the vertical
            const Eigen::AngleAxisd aa(Eigen::Matrix3d(tr[k].linear() * tr[k - 1].linear().transpose()));
            yaw += aa.angle() * aa.axis().z();
        }
        const double duration = ts.back().first - ts.front().first;
        if (duration > 0.0 && std::abs(yaw) / duration > threshold_rad_s)
            return true;
    }
    return false;
}

void repropagate_window_tail(Window &window, int tail_nodes, const Eigen::Vector3d &bias_update)
{
    auto &tr = window.poses;
    const auto &imu = window.motion_model_poses;
    const auto &ts = window.timestamps;
    const int n = static_cast<int>(tr.size());
    const int m = tail_nodes;
    const int a = n - 1 - m;          // last node kept from the optimization
    constexpr int velocity_nodes = 20; // same span as the prev_velocity estimate
    if (m > 0 && a >= velocity_nodes && ts.size() == tr.size() && imu.size() == tr.size())
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
