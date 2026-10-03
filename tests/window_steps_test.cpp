#include <doctest/doctest.h>

#include "window_steps.h"

#include <cmath>

// Synthetic windows: nodes every 10 ms, built in code.

namespace
{
    constexpr double dt = 0.01;

    Eigen::Affine3d pose(double x, double y, double z, double yaw_rad)
    {
        Eigen::Affine3d m = Eigen::Affine3d::Identity();
        m.linear() = Eigen::AngleAxisd(yaw_rad, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        m.translation() = Eigen::Vector3d(x, y, z);
        return m;
    }

    // n nodes moving along x at speed_mps and turning about z at rate_rad_s, starting at origin_pose
    Window moving_window(int n, double speed_mps, double rate_rad_s, const Eigen::Affine3d &origin_pose = Eigen::Affine3d::Identity())
    {
        Window w;
        for (int k = 0; k < n; k++)
        {
            w.poses.push_back(origin_pose * pose(speed_mps * k * dt, 0.0, 0.0, rate_rad_s * k * dt));
            w.timestamps.push_back({k * dt, 0.0});
        }
        w.motion_model_poses = w.poses;
        return w;
    }
}

TEST_CASE("place_window_after starts the window at the given pose and keeps its relative poses")
{
    const Window original = moving_window(30, 1.5, 0.3, pose(5.0, -2.0, 1.0, 0.7));
    Window w = original;
    const Eigen::Affine3d start = pose(-1.0, 4.0, 0.5, -0.2);

    place_window_after(w, start);

    REQUIRE(w.poses.size() == original.poses.size());
    CHECK(w.poses[0].isApprox(start, 1e-12));
    for (size_t k = 1; k < w.poses.size(); k++)
    {
        const Eigen::Affine3d rel_new = w.poses[k - 1].inverse() * w.poses[k];
        const Eigen::Affine3d rel_old = original.poses[k - 1].inverse() * original.poses[k];
        CHECK(rel_new.isApprox(rel_old, 1e-12));
    }
}

TEST_CASE("estimate_previous_velocity measures the previous window's final velocity")
{
    const Window previous = moving_window(50, 1.5, 0.0);
    Window window = moving_window(50, 1.5, 0.0);
    for (auto &t : window.timestamps)
        t.first += 0.5; // the window starts 0.5 s after the previous one

    const PreviousVelocity v = estimate_previous_velocity(previous, window, 20, 5.0);
    CHECK(v.use);
    CHECK_FALSE(v.dropped);
    CHECK(v.velocity.isApprox(Eigen::Vector3d(1.5, 0.0, 0.0), 1e-9));
    // centre of the span: nodes 29..49 -> 0.39 s
    CHECK(v.age_s == doctest::Approx(0.5 - 0.39));

    // faster than the limit: dropped, and the speed is reported
    const PreviousVelocity fast = estimate_previous_velocity(moving_window(50, 7.0, 0.0), window, 20, 5.0);
    CHECK_FALSE(fast.use);
    CHECK(fast.dropped);
    CHECK(fast.speed == doctest::Approx(7.0));
    CHECK(fast.velocity == Eigen::Vector3d::Zero());

    // no usable span: a single node, or timestamps that don't match the poses
    CHECK_FALSE(estimate_previous_velocity(moving_window(1, 1.5, 0.0), window, 20, 5.0).use);
    Window mismatched = moving_window(50, 1.5, 0.0);
    mismatched.timestamps.pop_back();
    CHECK_FALSE(estimate_previous_velocity(mismatched, window, 20, 5.0).use);
}
