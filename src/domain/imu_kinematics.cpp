#include "imu_kinematics.h"

#include <transformations.h>

// rotation by the vector v (axis * angle)
Eigen::Matrix3d exp_so3(const Eigen::Vector3d &v)
{
    const double angle = v.norm();
    if (angle < 1e-15)
        return Eigen::Matrix3d::Identity();
    return Eigen::AngleAxisd(angle, v / angle).toRotationMatrix();
}

// node orientations come from single-precision AHRS quaternions and are not exactly orthonormal;
// chains built here must stay on SO(3), otherwise the scale error compounds from node to node
Eigen::Quaterniond rotation_of(const Eigen::Affine3d &m)
{
    return Eigen::Quaterniond(m.linear()).normalized();
}

// relative rotation from node a to node b
Eigen::Quaterniond relative_rotation(const Eigen::Affine3d &a, const Eigen::Affine3d &b)
{
    return (rotation_of(a).conjugate() * rotation_of(b)).normalized();
}

// re-chain the node orientations with each relative rotation corrected by exp(-bias * dt) (body frame);
// node 0 and all translations stay unchanged
void remove_gyro_bias_from_rotations(std::vector<Eigen::Affine3d> &trajectory,
                                     const std::vector<std::pair<double, double>> &timestamps,
                                     const Eigen::Vector3d &bias)
{
    if (bias.isZero() || trajectory.size() != timestamps.size())
        return;
    const std::vector<Eigen::Affine3d> original = trajectory;
    Eigen::Quaterniond q = rotation_of(original[0]);
    for (size_t k = 1; k < trajectory.size(); k++)
    {
        const double dt = std::max(timestamps[k].first - timestamps[k - 1].first, 0.0);
        q = (q * relative_rotation(original[k - 1], original[k]) * Eigen::Quaterniond(exp_so3(-bias * dt))).normalized();
        trajectory[k].linear() = q.toRotationMatrix();
    }
}

// world-frame acceleration of each node [m/s^2]: gravity is removed in the sensor frame with the node's AHRS tilt,
// a = R * (f - |g| * R_ahrs^T * z), so a slow tilt drift of the odometry frame does not leak gravity into it;
// averaged over +-half_width_s (IMU samples arrive in bursts)
std::vector<Eigen::Vector3d> world_acceleration(const std::vector<Eigen::Affine3d> &trajectory,
                                                const std::vector<std::pair<double, double>> &timestamps,
                                                const std::vector<Eigen::Vector3d> &acc,
                                                const std::vector<Eigen::Vector3d> &ahrs_om_fi_ka,
                                                double gravity_norm, double half_width_s)
{
    const size_t n = trajectory.size();
    const double to_mps2 = 9.80665 / gravity_norm;
    std::vector<Eigen::Vector3d> a(n);
    for (size_t k = 0; k < n; k++)
    {
        TaitBryanPose tb;
        tb.om = ahrs_om_fi_ka[k].x();
        tb.fi = ahrs_om_fi_ka[k].y();
        tb.ka = ahrs_om_fi_ka[k].z();
        const Eigen::Matrix3d r_ahrs = affine_matrix_from_pose_tait_bryan(tb).linear();
        const Eigen::Vector3d gravity_body = gravity_norm * (r_ahrs.transpose() * Eigen::Vector3d::UnitZ());
        a[k] = rotation_of(trajectory[k]) * (acc[k] - gravity_body) * to_mps2;
    }
    if (half_width_s <= 0.0)
        return a;

    std::vector<Eigen::Vector3d> prefix(n + 1, Eigen::Vector3d::Zero());
    for (size_t k = 0; k < n; k++)
        prefix[k + 1] = prefix[k] + a[k];
    std::vector<Eigen::Vector3d> smoothed(n);
    size_t lo = 0, hi = 0;
    for (size_t k = 0; k < n; k++)
    {
        while (timestamps[lo].first < timestamps[k].first - half_width_s)
            lo++;
        while (hi < n && timestamps[hi].first <= timestamps[k].first + half_width_s)
            hi++;
        smoothed[k] = (prefix[hi] - prefix[lo]) / double(hi - lo);
    }
    return smoothed;
}

// acceleration offset e [m/s^2] seen in an optimized window: (integral of the IMU acceleration - change of the
// trajectory velocity) / T, both over the nodes k1..k2; the velocities are averaged over span nodes (false if span < 1)
bool observed_acceleration_offset(const std::vector<Eigen::Affine3d> &trajectory,
                                  const std::vector<std::pair<double, double>> &timestamps,
                                  const std::vector<Eigen::Vector3d> &acc, const Eigen::Vector3d &gravity,
                                  double to_mps2, int tail_nodes, int span, Eigen::Vector3d &offset)
{
    if (span < 1)
        return false;
    const int n = static_cast<int>(trajectory.size());
    const int k1 = span;
    const int k2 = n - 1 - std::max(tail_nodes, 0); // tail nodes are re-propagated, not optimized
    if (k2 - k1 < 2 * span || static_cast<int>(timestamps.size()) != n || static_cast<int>(acc.size()) != n)
        return false;
    auto velocity_at = [&](int k)
    {
        const double dt = timestamps[k].first - timestamps[k - span].first;
        return dt > 0.0 ? Eigen::Vector3d((trajectory[k].translation() - trajectory[k - span].translation()) / dt) : Eigen::Vector3d::Zero();
    };
    // velocities are averages over [k - span, k]: integrate the acceleration between their centres
    Eigen::Vector3d dv_imu = Eigen::Vector3d::Zero();
    for (int k = k1 - span / 2; k < k2 - span / 2; k++)
    {
        const double dt = timestamps[k + 1].first - timestamps[k].first;
        dv_imu += (Eigen::Quaterniond(trajectory[k].linear()).normalized() * acc[k] - gravity) * to_mps2 * dt;
    }
    const double T = timestamps[k2 - span / 2].first - timestamps[k1 - span / 2].first;
    if (T <= 0.0)
        return false;
    offset = (dv_imu - (velocity_at(k2) - velocity_at(k1))) / T;
    return true;
}

// true if the device rests during the whole window: small world acceleration and small rotation rate
bool is_window_stationary(const std::vector<Eigen::Affine3d> &trajectory,
                          const std::vector<std::pair<double, double>> &timestamps,
                          const std::vector<Eigen::Vector3d> &acceleration,
                          double acc_threshold_mps2, double gyro_threshold_rad_s)
{
    for (const auto &a : acceleration)
        if (a.norm() > acc_threshold_mps2)
            return false;

    // rotation rate over spans of >= 20 ms (single-sample rates are dominated by timestamp jitter)
    size_t a = 0;
    for (size_t k = 1; k < trajectory.size(); k++)
    {
        const double dt = timestamps[k].first - timestamps[a].first;
        if (dt < 0.02)
            continue;
        const double angle = Eigen::AngleAxisd(relative_rotation(trajectory[a], trajectory[k])).angle();
        if (angle / dt > gyro_threshold_rad_s)
            return false;
        a = k;
    }
    return true;
}
