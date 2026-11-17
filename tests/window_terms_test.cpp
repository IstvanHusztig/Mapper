#include <doctest/doctest.h>

#include "accelerometer_term.h"
#include "motion_model_terms.h"
#include "ndt_term.h"
#include "window_equations.h"
#include "window_solver.h"
#include <transformations.h>

#include <cmath>
#include <functional>
#include <random>

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

    // a term as a function of the window: it adds to fresh equations
    using Term = std::function<void(WindowEquations &, const Window &)>;
    // moves unknown `column` of the window by h
    using Shift = std::function<void(Window &, int column, double h)>;

    void shift_pose_or_bias(Window &w, int column, double h, const WindowUnknowns &u)
    {
        if (column >= u.ib && u.estimate_bias && column < u.ib + 3)
        {
            w.gyro_bias.mean(column - u.ib) += h;
            return;
        }
        // the optimizer's parameters of a node: x, y, z, om, fi, ka (Tait-Bryan)
        TaitBryanPose p = pose_tait_bryan_from_affine_matrix(w.poses[column / 6]);
        double *const parameter[6] = {&p.px, &p.py, &p.pz, &p.om, &p.fi, &p.ka};
        *parameter[column % 6] += h;
        w.poses[column / 6] = affine_matrix_from_pose_tait_bryan(p);
    }

    // The term's right-hand side must be minus half the gradient of its cost (central differences, step h), and with
    // linear = true its matrix diagonal half the second derivative.
    void check_gradient(const WindowUnknowns &u, const Window &w, const Term &term, const std::vector<int> &columns, const Shift &shift,
                        double h, bool linear)
    {
        WindowEquations e0(u);
        term(e0, w);
        const auto cost_at = [&](int column, double step)
        {
            Window moved = w;
            shift(moved, column, step);
            WindowEquations e(u);
            term(e, moved);
            return e.motion_cost;
        };
        for (int c : columns)
        {
            CAPTURE(c);
            const double plus = cost_at(c, h), minus = cost_at(c, -h);
            const double gradient = (plus - minus) / (2.0 * h);
            const double scale = std::max({std::abs(gradient), std::abs(e0.rhs(c, 0)), 1e-9 * e0.motion_cost, 1e-12});
            CHECK(std::abs(e0.rhs(c, 0) + 0.5 * gradient) <= 1e-5 * scale);
            if (linear)
            {
                const double curvature = (plus + minus - 2.0 * e0.motion_cost) / (2.0 * h * h);
                CHECK(std::abs(e0.matrix(c, c) - curvature) <= 1e-4 * std::max(std::abs(curvature), 1e-12));
            }
        }
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

TEST_CASE("window terms: the gyro-bias prior's right-hand side is minus half its cost's gradient")
{
    Window w = straight_window(5, 0.002);
    w.gyro_bias.mean = Eigen::Vector3d(0.01, -0.02, 0.005);
    MotionModelRotationOptions rotation;
    rotation.estimate_bias = true;
    rotation.bias_prior_mean = Eigen::Vector3d(0.012, -0.018, 0.0);
    Eigen::Matrix3d a;
    a << 1.0, 0.2, 0.0, 0.2, 2.0, 0.1, 0.0, 0.1, 1.5;
    rotation.bias_prior_cov = a * 1e-4;
    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
    REQUIRE(u.estimate_bias);
    const Term term = [&](WindowEquations &e, const Window &win) { add_gyro_bias_prior(e, u, win, rotation); };
    check_gradient(u, w, term, {u.ib, u.ib + 1, u.ib + 2}, [&](Window &win, int c, double h) { shift_pose_or_bias(win, c, h, u); }, 1e-4, true);
}

TEST_CASE("window terms: the velocity continuity's right-hand side is minus half its cost's gradient")
{
    // uneven timestamps and a wobbling path, so every residual is non-zero
    Window w = straight_window(12, 0.002);
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> wobble(-0.002, 0.002);
    for (int k = 0; k < 12; k++)
    {
        const double t = k * 0.002 + (k % 3) * 0.0003;
        w.timestamps[k] = {t, t};
        w.poses[k].translation() += Eigen::Vector3d(wobble(rng), wobble(rng), wobble(rng));
    }
    MotionModelRotationOptions rotation;
    rotation.estimate_bias = false;
    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
    MotionModelOptions m;
    m.acc_sigma_mps2 = 1.0;
    std::vector<int> columns;
    for (int k = 0; k < 12; k++)
        for (int axis = 0; axis < 3; axis++)
            columns.push_back(k * 6 + axis);
    const Shift shift = [&](Window &win, int c, double h) { shift_pose_or_bias(win, c, h, u); };
    for (bool with_prior : {false, true})
    {
        CAPTURE(with_prior);
        const VelocityPrior prior{with_prior, Eigen::Vector3d(1.1, -0.2, 0.05)};
        const Term term = [&](WindowEquations &e, const Window &win) { add_velocity_continuity(e, u, win, m, prior); };
        check_gradient(u, w, term, columns, shift, 1e-5, true);
    }
}

TEST_CASE("window terms: the motion-model edges' right-hand side is minus half their cost's gradient, with the bias coupling")
{
    // a turning window whose poses differ from the motion model by small random offsets, with uneven timestamps
    constexpr int n = 8;
    Window w = straight_window(n, 0.002);
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> offset(-0.002, 0.002), angle(-0.003, 0.003);
    for (int k = 0; k < n; k++)
    {
        const double t = k * 0.002 + (k % 2) * 0.0004;
        w.timestamps[k] = {t, t};
        w.motion_model_poses[k].linear() = Eigen::AngleAxisd(0.05 * k, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        w.poses[k] = w.motion_model_poses[k];
        w.poses[k].translation() += Eigen::Vector3d(offset(rng), offset(rng), offset(rng));
        w.poses[k].linear() = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitZ())).toRotationMatrix() * w.poses[k].linear();
    }
    w.gyro_bias.mean = Eigen::Vector3d(0.02, -0.01, 0.03);
    MotionModelOptions m;
    m.x_sigma_m = m.y_sigma_m = m.z_sigma_m = 1e-3;
    m.om_sigma_deg = m.fi_sigma_deg = m.ka_sigma_deg = 0.1;
    m.correction.om = 0.001;
    MotionModelRotationOptions rotation;
    rotation.inflation_k = 7.0;
    rotation.arw_rad_sqrt_s = Eigen::Vector3d(0.028, 0.042, 0.057) * (M_PI / 180.0);
    rotation.sigma_floor_rad = 0.0035 * (M_PI / 180.0);
    rotation.bias_prior_mean = Eigen::Vector3d(0.01, -0.012, 0.025);
    for (bool time_scaled : {false, true})
        for (bool bias : {false, true})
        {
            CAPTURE(time_scaled);
            CAPTURE(bias);
            rotation.time_scaled_sigma = time_scaled;
            rotation.estimate_bias = bias;
            const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
            std::vector<int> columns;
            for (int c = 0; c < u.n_pose_params; c++)
                columns.push_back(c);
            if (bias)
                for (int r = 0; r < 3; r++)
                    columns.push_back(u.ib + r);
            const Term term = [&](WindowEquations &e, const Window &win)
            {
                e.with_cost = true;
                add_motion_edge_terms(e, u, win, m, rotation, {});
            };
            check_gradient(u, w, term, columns, [&](Window &win, int c, double h) { shift_pose_or_bias(win, c, h, u); }, 1e-7, false);
        }
}

namespace
{
    // the accelerometer functions read the window and the acceleration offset
    struct AccState
    {
        Window window;
        Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    };

    void shift_acc_state(AccState &s, int column, double h, const WindowUnknowns &u)
    {
        if (column >= u.io && column < u.io + 3)
            s.offset(column - u.io) += h;
        else if (column >= u.iv && column < u.iv + 3)
            s.window.velocity.start(column - u.iv) += h;
        else if (column >= u.ie && column < u.ie + 3)
            s.window.velocity.end(column - u.ie) += h;
        else
            shift_pose_or_bias(s.window, column, h, u);
    }

    // as check_gradient, on an AccState; tolerance relative to the larger of the two sides
    void check_acc_gradient(const WindowUnknowns &u, const AccState &state, const std::function<void(WindowEquations &, const AccState &)> &term,
                            const std::vector<int> &columns, double h, double tolerance)
    {
        WindowEquations e0(u);
        term(e0, state);
        for (int c : columns)
        {
            CAPTURE(c);
            AccState plus = state, minus = state;
            shift_acc_state(plus, c, h, u);
            shift_acc_state(minus, c, -h, u);
            WindowEquations ep(u), em(u);
            term(ep, plus);
            term(em, minus);
            const double gradient = (ep.motion_cost - em.motion_cost) / (2.0 * h);
            const double scale = std::max({std::abs(gradient), std::abs(e0.rhs(c, 0)), 1e-9 * e0.motion_cost, 1e-12});
            CHECK(std::abs(e0.rhs(c, 0) + 0.5 * gradient) <= tolerance * scale);
        }
    }

    // a window of 30 nodes 2 ms apart with a measured specific force that doesn't quite match the motion, a start and end
    // velocity and an offset away from their priors
    struct AccCase
    {
        AccState state;
        std::vector<Eigen::Vector3d> acc;
        MotionModelTranslationImuOptions imu;
        WindowUnknowns unknowns;
    };

    void make_acc_case(AccCase &k)
    {
        constexpr int n = 30;
        k.state.window = straight_window(n, 0.002);
        std::mt19937 rng(5);
        std::uniform_real_distribution<double> noise(-1.0, 1.0);
        for (int i = 0; i < n; i++)
        {
            Window &w = k.state.window;
            w.poses[i].linear() = (Eigen::AngleAxisd(0.3 * i * 0.002, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX())).toRotationMatrix();
            w.poses[i].translation() += Eigen::Vector3d(noise(rng), noise(rng), noise(rng)) * 1e-4;
            k.acc.push_back(Eigen::Vector3d(0.02 * noise(rng), 0.02 * noise(rng), 1.0 + 0.01 * noise(rng)));
        }
        k.state.window.velocity.start = Eigen::Vector3d(0.95, 0.03, -0.01);
        k.state.window.velocity.end = Eigen::Vector3d(1.04, -0.02, 0.02);
        k.state.offset = Eigen::Vector3d(0.01, -0.02, 0.005);
        k.imu.enabled = true;
        k.imu.acc = &k.acc;
        k.imu.gravity = Eigen::Vector3d(0.0, 0.0, 1.0);
        k.imu.to_mps2 = 9.80665;
        k.imu.span_s = 0.005;
        k.imu.noise_density_mps2_sqrt_hz = 0.0013;
        k.imu.floor_sigma_mps2 = 0.1;
        k.imu.velocity_prior_mean = Eigen::Vector3d(1.0, 0.0, 0.0);
        Eigen::Matrix3d a;
        a << 1.0, 0.1, 0.0, 0.1, 1.2, 0.05, 0.0, 0.05, 0.8;
        k.imu.velocity_prior_cov = a * 4e-4;
        k.imu.offset_prior_mean = Eigen::Vector3d(0.0, -0.01, 0.0);
        k.imu.offset_prior_cov = a * 1e-4;
        k.imu.zero_velocity_sigma_mps = 0.01;
        MotionModelRotationOptions rotation;
        rotation.estimate_bias = false;
        k.unknowns = layout_window_unknowns(k.state.window, rotation, k.imu);
    }
}

TEST_CASE("window terms: the acceleration terms' right-hand side is minus half their cost's gradient")
{
    AccCase k;
    make_acc_case(k);
    const WindowUnknowns &u = k.unknowns;
    REQUIRE(u.acc_term);
    const auto term = [&](WindowEquations &e, const AccState &s) { add_acceleration_terms(e, u, s.window, k.imu, s.offset); };
    std::vector<int> linear, rotation;
    for (int node = 0; node < u.n_nodes; node++)
        for (int p = 0; p < 6; p++)
            (p < 3 ? linear : rotation).push_back(node * 6 + p);
    for (int r = 0; r < 3; r++)
    {
        linear.push_back(u.io + r);
        linear.push_back(u.iv + r);
        linear.push_back(u.ie + r);
    }
    check_acc_gradient(u, k.state, term, linear, 1e-6, 1e-5);
    // the rotation part of the Jacobian is itself a central difference (tait_bryan_rotation_jacobian), and d(R f) = -[R f]x
    // d(theta) is first order: a looser tolerance
    check_acc_gradient(u, k.state, term, rotation, 1e-6, 1e-3);
}

TEST_CASE("window terms: the velocity priors' right-hand side is minus half their cost's gradient")
{
    AccCase k;
    make_acc_case(k);
    const WindowUnknowns &u = k.unknowns;
    SUBCASE("start prior and rest, end velocity tied by its term")
    {
        k.imu.at_rest = true;
        const auto term = [&](WindowEquations &e, const AccState &s) { add_velocity_priors(e, u, s.window, k.imu, true); };
        check_acc_gradient(u, k.state, term, {u.iv, u.iv + 1, u.iv + 2, u.ie, u.ie + 1, u.ie + 2}, 1e-6, 1e-5);
    }
    SUBCASE("weak end tie: it treats the start velocity as fixed, so only the end velocity is checked")
    {
        const auto term = [&](WindowEquations &e, const AccState &s)
        {
            // the tie alone: its cost and right-hand side without the start prior's
            WindowEquations with(u), without(u);
            add_velocity_priors(with, u, s.window, k.imu, false);
            add_velocity_priors(without, u, s.window, k.imu, true);
            e.motion_cost = with.motion_cost - without.motion_cost;
            e.rhs = with.rhs - without.rhs;
        };
        check_acc_gradient(u, k.state, term, {u.ie, u.ie + 1, u.ie + 2}, 1e-6, 1e-5);
    }
}

TEST_CASE("window terms: the acceleration-offset prior's right-hand side is minus half its cost's gradient")
{
    AccCase k;
    make_acc_case(k);
    const WindowUnknowns &u = k.unknowns;
    const auto term = [&](WindowEquations &e, const AccState &s) { add_acceleration_offset_prior(e, u, k.imu, s.offset); };
    check_acc_gradient(u, k.state, term, {u.io, u.io + 1, u.io + 2}, 1e-6, 1e-5);
}

TEST_CASE("window terms: the degeneracy projection passes strong directions and removes weak ones")
{
    const Window w = straight_window(3, 0.002);
    MotionModelRotationOptions rotation;
    rotation.estimate_bias = false;
    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> value(-1.0, 1.0);
    NdtSums sums;
    for (int k = 0; k < 3; k++)
    {
        Eigen::Matrix<double, 6, 6> a;
        for (int i = 0; i < 36; i++)
            a.data()[i] = value(rng);
        sums.blocks.push_back(a * a.transpose() + Eigen::Matrix<double, 6, 6>::Identity());
        Eigen::Matrix<double, 6, 1> g;
        for (int i = 0; i < 6; i++)
            g(i) = value(rng);
        sums.rhs.push_back(g);
    }
    ScanToMapOptions options;
    options.degenerate_projection_ratio = 0.1;
    options.degenerate_projection_ratio_full = 0.2;
    SUBCASE("every direction strong: the blocks unchanged")
    {
        sums.normal_information.assign(3, Eigen::Vector3d(1.0, 0.8, 0.6).asDiagonal());
        WindowEquations e(u);
        add_projected_ndt_blocks(e, sums, options);
        for (int k = 0; k < 3; k++)
            for (int r = 0; r < 6; r++)
            {
                CHECK(e.rhs(k * 6 + r, 0) == sums.rhs[k](r));
                for (int c = 0; c < 6; c++)
                    CHECK(e.matrix(k * 6 + r, k * 6 + c) == sums.blocks[k](r, c));
            }
    }
    SUBCASE("z weak (share 0.05 < 0.1): its translation row and column removed, rotation untouched")
    {
        sums.normal_information.assign(3, Eigen::Vector3d(1.0, 0.8, 0.05).asDiagonal());
        WindowEquations e(u);
        add_projected_ndt_blocks(e, sums, options);
        Eigen::Matrix<double, 6, 6> M = Eigen::Matrix<double, 6, 6>::Identity();
        M(2, 2) = 0.0;
        for (int k = 0; k < 3; k++)
        {
            const Eigen::Matrix<double, 6, 6> expected = M * sums.blocks[k] * M;
            const Eigen::Matrix<double, 6, 1> expected_rhs = M * sums.rhs[k];
            for (int r = 0; r < 6; r++)
            {
                CHECK(std::abs(e.rhs(k * 6 + r, 0) - expected_rhs(r)) < 1e-12);
                for (int c = 0; c < 6; c++)
                    CHECK(std::abs(e.matrix(k * 6 + r, k * 6 + c) - expected(r, c)) < 1e-12);
            }
        }
    }
    SUBCASE("projection off: nothing added")
    {
        sums.normal_information.assign(3, Eigen::Matrix3d::Identity());
        options.degenerate_projection_ratio_full = 0.0;
        WindowEquations e(u);
        add_projected_ndt_blocks(e, sums, options);
        CHECK(e.rhs.isZero());
        CHECK(e.matrix(0, 0) == 0.0);
    }
    CHECK(window_translation_information(sums).isApprox(3.0 * sums.normal_information[0]));
}
