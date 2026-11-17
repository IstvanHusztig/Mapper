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
