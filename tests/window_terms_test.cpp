#include <doctest/doctest.h>

#include "window_equations.h"
#include "window_solver.h"

#include <cmath>

// The term functions of optimize_lidar_odometry (stage 2 of docs/code-split-and-region-agents-plan.md), each on its own:
// the layout of the unknowns, the origin fix, and (as they are moved) every term: its right-hand side must be minus half
// the numeric gradient of its cost.

namespace
{
    // a window of n nodes, dt apart, along x at 1 m/s
    Window straight_window(int n, double dt)
    {
        Window w;
        for (int k = 0; k < n; k++)
        {
            Eigen::Affine3d p = Eigen::Affine3d::Identity();
            p.translation() = Eigen::Vector3d(k * dt, 0.0, 0.0);
            w.poses.push_back(p);
            w.timestamps.push_back({k * dt, k * dt});
        }
        w.motion_model_poses = w.poses;
        return w;
    }
}

TEST_CASE("window terms: the layout of the unknowns")
{
    const Window w = straight_window(20, 0.002);
    MotionModelRotationOptions rotation;
    MotionModelTranslationImuOptions imu;
    SUBCASE("poses only")
    {
        rotation.estimate_bias = false;
        const WindowUnknowns u = layout_window_unknowns(w, rotation, imu);
        CHECK(u.n_nodes == 20);
        CHECK(u.n_pose_params == 120);
        CHECK(u.n_params == 120);
        CHECK(u.pose_band_offset == 17);
        CHECK(u.anchors.empty());
    }
    SUBCASE("gyro bias and the accelerometer term")
    {
        rotation.estimate_bias = true;
        std::vector<Eigen::Vector3d> acc(20, Eigen::Vector3d(0.0, 0.0, 1.0));
        imu.enabled = true;
        imu.acc = &acc;
        imu.span_s = 0.005; // a node every 2 ms: anchors at 0, 3, 6, ... (6 ms apart)
        const WindowUnknowns u = layout_window_unknowns(w, rotation, imu);
        CHECK(u.n_params == 120 + 3 + 9);
        CHECK(u.ib == 120);
        CHECK(u.io == 123);
        CHECK(u.iv == 126);
        CHECK(u.ie == 129);
        REQUIRE(u.anchors.size() >= 3);
        CHECK(u.anchors[0] == 0);
        CHECK(u.anchors[1] == 3);
        CHECK(u.pose_band_offset == 6 * 6 + 5); // the widest acceleration term spans two anchor gaps
    }
    SUBCASE("without timestamps there is no bias and no accelerometer term")
    {
        Window no_time = w;
        no_time.timestamps.clear();
        rotation.estimate_bias = true;
        std::vector<Eigen::Vector3d> acc(20, Eigen::Vector3d(0.0, 0.0, 1.0));
        imu.enabled = true;
        imu.acc = &acc;
        const WindowUnknowns u = layout_window_unknowns(no_time, rotation, imu);
        CHECK_FALSE(u.estimate_bias);
        CHECK_FALSE(u.acc_term);
        CHECK(u.n_params == 120);
    }
}

TEST_CASE("window terms: the origin fix weighs the first node's six parameters and nothing else")
{
    const Window w = straight_window(5, 0.002);
    MotionModelRotationOptions rotation;
    rotation.estimate_bias = false;
    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
    WindowEquations e(u);
    MotionModelOptions m;
    m.fix_origin_x_sigma_m = m.fix_origin_y_sigma_m = m.fix_origin_z_sigma_m = 1e-3;
    m.fix_origin_om_sigma_deg = m.fix_origin_fi_sigma_deg = m.fix_origin_ka_sigma_deg = 1e-2;
    fix_origin(e, m);
    for (int r = 0; r < u.n_params; r++)
        for (int c = 0; c < u.n_params; c++)
        {
            const double expected = r == c && r < 3 ? 1e6 : (r == c && r < 6 ? 1e4 : 0.0);
            CHECK(std::abs(e.matrix(r, c) - expected) <= 1e-9 * expected);
        }
    CHECK(e.rhs.isZero());
    CHECK(e.motion_cost == 0.0);
}
