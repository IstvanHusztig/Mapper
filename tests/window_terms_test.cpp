#include <doctest/doctest.h>

#include "accelerometer_term.h"
#include "motion_model_terms.h"
#include "ndt_term.h"
#include "window_equations.h"
#include "window_solver.h"
#include "window_steps.h"
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
    fix_origin(e, build_window_noise_model(w, ScanToMapOptions{}, m, rotation, {}, VelocityPrior{}));
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
    const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, MotionModelOptions{}, rotation, {}, VelocityPrior{});
    const WindowUnknowns &u = noise.unknowns;
    REQUIRE(u.estimate_bias);
    const Term term = [&](WindowEquations &e, const Window &win) { add_gyro_bias_prior(e, noise, win); };
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
        const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, rotation, {}, prior);
        const Term term = [&](WindowEquations &e, const Window &win) { add_velocity_continuity(e, noise, win); };
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
            const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, rotation, {}, VelocityPrior{});
            const WindowUnknowns &u = noise.unknowns;
            std::vector<int> columns;
            for (int c = 0; c < u.n_pose_params; c++)
                columns.push_back(c);
            if (bias)
                for (int r = 0; r < 3; r++)
                    columns.push_back(u.ib + r);
            const Term term = [&](WindowEquations &e, const Window &win)
            {
                e.with_cost = true;
                add_motion_edge_terms(e, noise, win, m, rotation, {}, Eigen::Vector3d::Zero());
            };
            check_gradient(u, w, term, columns, [&](Window &win, int c, double h) { shift_pose_or_bias(win, c, h, u); }, 1e-7, false);
        }

    // the edge block is linear in its six weights: the position rows alone (rotation weights zero) plus the rotation rows
    // alone (position weights zero) give the full edges, matrix, right-hand side and cost (the per-term table's two rows)
    rotation.time_scaled_sigma = true;
    rotation.estimate_bias = true;
    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
    const MotionEdgeWeights full = motion_edge_weights(u, w, m, rotation, {});
    MotionEdgeWeights position = full;
    for (Eigen::Vector3d &r : position.rotation)
        r.setZero();
    MotionEdgeWeights rotation_only = full;
    rotation_only.x = rotation_only.y = rotation_only.z = 0.0;
    const auto edges = [&](const MotionEdgeWeights &weights)
    {
        WindowEquations e(u);
        e.with_cost = true;
        add_motion_edges(e, u, w, m, rotation, {}, Eigen::Vector3d::Zero(), weights);
        return e;
    };
    const WindowEquations all = edges(full), pos = edges(position), rot = edges(rotation_only);
    const Eigen::MatrixXd A_all = Eigen::MatrixXd(all.matrix.to_sparse());
    const Eigen::MatrixXd A_sum = Eigen::MatrixXd(pos.matrix.to_sparse()) + Eigen::MatrixXd(rot.matrix.to_sparse());
    CHECK((A_all - A_sum).norm() <= 1e-10 * A_all.norm());
    CHECK((all.rhs - pos.rhs - rot.rhs).norm() <= 1e-10 * all.rhs.norm());
    CHECK(std::abs(all.motion_cost - pos.motion_cost - rot.motion_cost) <= 1e-10 * all.motion_cost);
    CHECK(pos.motion_cost > 0.0);
    CHECK(rot.motion_cost > 0.0);
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
        WindowNoiseModel noise;
    };

    // the case's noise model (after a change of its options)
    void rebuild_noise(AccCase &k)
    {
        MotionModelRotationOptions rotation;
        rotation.estimate_bias = false;
        k.noise = build_window_noise_model(k.state.window, ScanToMapOptions{}, MotionModelOptions{}, rotation, k.imu, VelocityPrior{});
        k.unknowns = k.noise.unknowns;
    }

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
        rebuild_noise(k);
    }
}

TEST_CASE("window terms: the acceleration terms' right-hand side is minus half their cost's gradient")
{
    AccCase k;
    make_acc_case(k);
    const WindowUnknowns &u = k.unknowns;
    REQUIRE(u.acc_term);
    const auto term = [&](WindowEquations &e, const AccState &s) { add_acceleration_terms(e, k.noise, s.window, k.imu, s.offset); };
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
        rebuild_noise(k);
        const auto term = [&](WindowEquations &e, const AccState &s) { add_velocity_priors(e, k.noise, s.window, true); };
        check_acc_gradient(u, k.state, term, {u.iv, u.iv + 1, u.iv + 2, u.ie, u.ie + 1, u.ie + 2}, 1e-6, 1e-5);
    }
    SUBCASE("weak end tie: it treats the start velocity as fixed, so only the end velocity is checked")
    {
        const auto term = [&](WindowEquations &e, const AccState &s)
        {
            // the tie alone: its cost and right-hand side without the start prior's
            WindowEquations with(u), without(u);
            add_velocity_priors(with, k.noise, s.window, false);
            add_velocity_priors(without, k.noise, s.window, true);
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
    const auto term = [&](WindowEquations &e, const AccState &s) { add_acceleration_offset_prior(e, k.noise, s.offset); };
    check_acc_gradient(u, k.state, term, {u.io, u.io + 1, u.io + 2}, 1e-6, 1e-5);
}

// imu_carry_joint_covariance (change honest-inertial-carry, design D2): one prior on (start velocity, offset) together
TEST_CASE("window terms: the joint velocity-offset prior's right-hand side is minus half its cost's gradient")
{
    AccCase k;
    make_acc_case(k);
    // a cross block that keeps the 6x6 covariance positive definite ([[4, 0.5], [0.5, 1]] x a, x 1e-4) and is not symmetric
    Eigen::Matrix3d a;
    a << 1.0, 0.1, 0.0, 0.1, 1.2, 0.05, 0.0, 0.05, 0.8;
    Eigen::Matrix3d asymmetric;
    asymmetric << 0.0, 0.2, -0.1, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0;
    k.imu.velocity_offset_prior_cov = (0.5 * a + 0.1 * asymmetric) * 1e-4;
    k.imu.joint_covariance = true;
    rebuild_noise(k);
    REQUIRE(k.noise.joint_prior);
    const WindowUnknowns &u = k.unknowns;
    const auto term = [&](WindowEquations &e, const AccState &s) { add_joint_velocity_offset_prior(e, k.noise, s.window, s.offset); };
    const std::vector<int> columns = {u.iv, u.iv + 1, u.iv + 2, u.io, u.io + 1, u.io + 2};
    check_acc_gradient(u, k.state, term, columns, 1e-6, 1e-5);

    // the matrix on (iv, io) is the joint information; the cost is the residual's quadratic form
    WindowEquations joint(u);
    term(joint, k.state);
    Eigen::Matrix<double, 6, 1> r;
    r << k.noise.velocity_prior_mean - k.state.window.velocity.start, k.noise.offset_prior_mean - k.state.offset;
    CHECK(joint.motion_cost == doctest::Approx(r.dot(k.noise.joint_prior_information * r)).epsilon(1e-12));
    CHECK(joint.residuals == 6);
    for (int p = 0; p < 6; p++)
        for (int q = 0; q < 6; q++)
            CHECK(joint.matrix(columns[p], columns[q]) == k.noise.joint_prior_information(p, q));
    // the cross block matters: the velocity's right-hand side depends on the offset's residual
    REQUIRE(k.noise.joint_prior_information.topRightCorner<3, 3>().cwiseAbs().maxCoeff() > 0.0);

    // with the joint prior the two separate priors add nothing
    WindowEquations separate(u);
    add_start_velocity_prior(separate, k.noise, k.state.window);
    add_acceleration_offset_prior(separate, k.noise, k.state.offset);
    CHECK(separate.motion_cost == 0.0);
    CHECK(separate.residuals == 0);
    CHECK(separate.rhs.cwiseAbs().maxCoeff() == 0.0);
}

TEST_CASE("window terms: the joint velocity-offset prior with a zero cross block equals the two separate priors")
{
    AccCase k;
    make_acc_case(k);
    const WindowUnknowns &u = k.unknowns;
    const auto all_priors = [&](WindowEquations &e)
    {
        add_start_velocity_prior(e, k.noise, k.state.window);
        add_acceleration_offset_prior(e, k.noise, k.state.offset);
        add_joint_velocity_offset_prior(e, k.noise, k.state.window, k.state.offset);
    };
    WindowEquations separate(u);
    all_priors(separate);
    REQUIRE_FALSE(k.noise.joint_prior);
    k.imu.joint_covariance = true; // velocity_offset_prior_cov stays zero
    rebuild_noise(k);
    REQUIRE(k.noise.joint_prior);
    WindowEquations joint(u);
    all_priors(joint);

    CHECK(joint.residuals == separate.residuals);
    CHECK(joint.motion_cost == doctest::Approx(separate.motion_cost).epsilon(1e-9));
    const double rhs_scale = separate.rhs.cwiseAbs().maxCoeff();
    REQUIRE(rhs_scale > 0.0);
    CHECK((joint.rhs - separate.rhs).cwiseAbs().maxCoeff() <= 1e-9 * rhs_scale);
    const std::vector<int> columns = {u.iv, u.iv + 1, u.iv + 2, u.io, u.io + 1, u.io + 2};
    double matrix_scale = 0.0, matrix_difference = 0.0;
    for (int p : columns)
        for (int q : columns)
        {
            matrix_scale = std::max(matrix_scale, std::abs(separate.matrix(p, q)));
            matrix_difference = std::max(matrix_difference, std::abs(joint.matrix(p, q) - separate.matrix(p, q)));
        }
    CHECK(matrix_difference <= 1e-9 * matrix_scale);
}

namespace
{
    // imu_edges_follow_state: a turning window of 30 nodes about 2 ms apart (uneven), with the accelerometer, whose guess
    // is integrated from velocity_guess (integrate_window_translations), as prepare_window makes it
    struct FollowCase
    {
        Window guess;
        std::vector<Eigen::Vector3d> acc;
        MotionModelTranslationImuOptions imu;
        MotionModelOptions m;
        MotionModelRotationOptions rotation;
        WindowNoiseModel noise;
    };

    void rebuild_follow_noise(FollowCase &k)
    {
        k.noise = build_window_noise_model(k.guess, ScanToMapOptions{}, k.m, k.rotation, k.imu, VelocityPrior{});
    }

    void make_follow_case(FollowCase &k, bool bias)
    {
        constexpr int n = 30;
        k.guess = straight_window(n, 0.002);
        std::mt19937 rng(13);
        std::uniform_real_distribution<double> noise(-1.0, 1.0);
        for (int i = 0; i < n; i++)
        {
            const double t = i * 0.002 + (i % 3) * 0.0003;
            k.guess.timestamps[i] = {t, t};
            k.guess.poses[i].linear() = (Eigen::AngleAxisd(0.5 * t, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX()) *
                                         Eigen::AngleAxisd(-0.03, Eigen::Vector3d::UnitY())).toRotationMatrix();
            k.acc.push_back(Eigen::Vector3d(0.05 * noise(rng), 0.05 * noise(rng), 1.0 + 0.02 * noise(rng)));
        }
        k.guess.imu_acc = k.acc;
        k.imu.enabled = true;
        k.imu.acc = &k.acc;
        k.imu.gravity = Eigen::Vector3d(0.0, 0.0, 1.0);
        k.imu.to_mps2 = 9.80665;
        k.imu.span_s = 0.005;
        k.imu.noise_density_mps2_sqrt_hz = 0.0013;
        k.imu.floor_sigma_mps2 = 0.1;
        k.imu.velocity_prior_mean = Eigen::Vector3d(0.8, 0.1, -0.05);
        k.imu.velocity_prior_cov = Eigen::Matrix3d::Identity() * 1e-2;
        k.imu.offset_prior_cov = Eigen::Matrix3d::Identity() * 1e-2;
        k.imu.edges_follow_state = true;
        k.imu.velocity_guess = k.imu.velocity_prior_mean;
        k.guess.velocity.start = k.guess.velocity.end = k.imu.velocity_guess;
        k.imu.guess_integrated = integrate_window_translations(k.guess, k.imu.velocity_guess, k.imu.gravity, k.imu.to_mps2);
        k.m.x_sigma_m = k.m.y_sigma_m = k.m.z_sigma_m = 1e-3;
        k.m.om_sigma_deg = k.m.fi_sigma_deg = k.m.ka_sigma_deg = 0.1;
        k.rotation.estimate_bias = bias;
        k.rotation.bias_prior_mean = Eigen::Vector3d(0.01, -0.012, 0.025);
        k.guess.gyro_bias.mean = Eigen::Vector3d(0.02, -0.01, 0.03);
        rebuild_follow_noise(k);
    }

    // the node translations of the guess's path for start velocity v and acceleration offset e: the guess integrated again
    // with mean acceleration - e (the gravity vector + e in accelerometer units)
    std::vector<Eigen::Vector3d> path_of(const FollowCase &k, const Eigen::Vector3d &v, const Eigen::Vector3d &e)
    {
        Window w = k.guess;
        integrate_window_translations(w, v, k.imu.gravity + e / k.imu.to_mps2, k.imu.to_mps2);
        std::vector<Eigen::Vector3d> p;
        for (const Eigen::Affine3d &pose : w.poses)
            p.push_back(pose.translation());
        return p;
    }

    WindowEquations follow_edges(const FollowCase &k, const AccState &s)
    {
        WindowEquations e(k.noise.unknowns);
        e.with_cost = true;
        add_motion_edge_terms(e, k.noise, s.window, k.m, k.rotation, k.imu, s.offset);
        return e;
    }

    // a state near the guess: node translations off by up to 0.1 mm, at the guess's start velocity and zero offset
    AccState near_guess(const FollowCase &k)
    {
        AccState s{k.guess, Eigen::Vector3d::Zero()};
        std::mt19937 rng(17);
        std::uniform_real_distribution<double> offset(-1e-4, 1e-4);
        for (Eigen::Affine3d &p : s.window.poses)
            p.translation() += Eigen::Vector3d(offset(rng), offset(rng), offset(rng));
        return s;
    }

    // s moved along the path from (v_guess, 0) to (v_guess + dv, e): the nodes, the start velocity and the offset
    AccState moved_along_path(const FollowCase &k, const AccState &s, const Eigen::Vector3d &dv, const Eigen::Vector3d &e)
    {
        const std::vector<Eigen::Vector3d> from = path_of(k, k.imu.velocity_guess, Eigen::Vector3d::Zero());
        const std::vector<Eigen::Vector3d> to = path_of(k, k.imu.velocity_guess + dv, e);
        AccState moved = s;
        for (size_t i = 0; i < moved.window.poses.size(); i++)
            moved.window.poses[i].translation() += to[i] - from[i];
        moved.window.velocity.start += dv;
        moved.offset += e;
        return moved;
    }

    // the same cost, and the same right-hand side in the translation, velocity and offset columns; with rotations = true
    // also in the rotation columns. (Off the guess the rotation columns differ: d(R_k-1^T (p_k - p_k-1))/d(theta) grows
    // with the step p_k - p_k-1, which the other path changes; at the guess the position residuals are zero and so is
    // that part of the rotation columns.)
    void check_same_edges(const WindowUnknowns &u, const WindowEquations &a, const WindowEquations &b, bool rotations)
    {
        CHECK(std::abs(a.motion_cost - b.motion_cost) <= 1e-8 * std::max(a.motion_cost, 1e-12));
        double scale = 1e-3, diff = 0.0; // [weight x m]: the edges' right-hand side off the path is ~1e5 here
        for (int c = 0; c < u.n_params; c++)
        {
            if (c < u.n_pose_params && c % 6 >= 3 && !rotations)
                continue;
            scale = std::max(scale, std::abs(a.rhs(c, 0)));
            diff = std::max(diff, std::abs(a.rhs(c, 0) - b.rhs(c, 0)));
        }
        CHECK(diff <= 1e-7 * scale);
        CHECK(a.residuals == b.residuals);
    }
}

TEST_CASE("window terms: with imu_edges_follow_state the edges follow the start velocity and the acceleration offset")
{
    for (bool bias : {false, true})
    {
        CAPTURE(bias);
        FollowCase k;
        make_follow_case(k, bias);
        REQUIRE(k.imu.guess_integrated);
        REQUIRE(k.noise.unknowns.acc_term);
        const WindowUnknowns &u = k.noise.unknowns;
        const AccState guess{k.guess, Eigen::Vector3d::Zero()};
        const WindowEquations at_guess = follow_edges(k, guess);
        const AccState s = near_guess(k);
        const WindowEquations at = follow_edges(k, s);
        REQUIRE(at.motion_cost > 0.0);
        FollowCase off = k;
        off.imu.edges_follow_state = false;

        SUBCASE("a start velocity 0.5 m/s larger, the nodes moved along its path: cost and right-hand side unchanged")
        {
            for (int axis = 0; axis < 3; axis++)
            {
                CAPTURE(axis);
                const Eigen::Vector3d dv = 0.5 * Eigen::Vector3d::Unit(axis);
                const AccState moved = moved_along_path(k, s, dv, Eigen::Vector3d::Zero());
                check_same_edges(u, at, follow_edges(k, moved), false);
                check_same_edges(u, at_guess, follow_edges(k, moved_along_path(k, guess, dv, Eigen::Vector3d::Zero())), true);
                CHECK(follow_edges(off, moved).motion_cost > 10.0 * follow_edges(off, s).motion_cost); // without the setting it costs
            }
        }
        SUBCASE("an acceleration offset of 0.1 m/s^2, the nodes moved along its path: cost and right-hand side unchanged")
        {
            for (int axis = 0; axis < 3; axis++)
            {
                CAPTURE(axis);
                const Eigen::Vector3d e = 0.1 * Eigen::Vector3d::Unit(axis);
                const AccState moved = moved_along_path(k, s, Eigen::Vector3d::Zero(), e);
                check_same_edges(u, at, follow_edges(k, moved), false);
                check_same_edges(u, at_guess, follow_edges(k, moved_along_path(k, guess, Eigen::Vector3d::Zero(), e)), true);
                // without the setting the move costs (from the guess: 2 ms edges, so a small cost)
                CHECK(follow_edges(off, moved_along_path(k, guess, Eigen::Vector3d::Zero(), e)).motion_cost > at_guess.motion_cost);
            }
            const AccState both = moved_along_path(k, s, Eigen::Vector3d(0.3, -0.2, 0.1), Eigen::Vector3d(0.05, 0.08, -0.06));
            check_same_edges(u, at, follow_edges(k, both), false);
        }
        SUBCASE("the right-hand side is minus half the cost's gradient, with the velocity and offset columns")
        {
            // rotations off the guess too, the velocity and the offset away from it: every residual non-zero
            AccState x = moved_along_path(k, s, Eigen::Vector3d(0.2, -0.1, 0.05), Eigen::Vector3d(0.03, -0.02, 0.04));
            x.window.velocity.start += Eigen::Vector3d(0.01, 0.02, -0.01);
            x.offset += Eigen::Vector3d(-0.02, 0.01, 0.03);
            std::mt19937 rng(19);
            std::uniform_real_distribution<double> angle(-0.003, 0.003);
            for (Eigen::Affine3d &p : x.window.poses)
                p.linear() = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitZ())).toRotationMatrix() * p.linear();
            std::vector<int> columns;
            for (int c = 0; c < u.n_pose_params; c++)
                columns.push_back(c);
            for (int r = 0; r < 3; r++)
            {
                columns.push_back(u.iv + r);
                columns.push_back(u.io + r);
                if (bias)
                    columns.push_back(u.ib + r);
            }
            const auto term = [&](WindowEquations &e, const AccState &state)
            {
                e.with_cost = true;
                add_motion_edge_terms(e, k.noise, state.window, k.m, k.rotation, k.imu, state.offset);
            };
            check_acc_gradient(u, x, term, columns, 1e-7, 1e-5);

            // the residual is linear in v and e and the pose Jacobian does not depend on them: the matrix's v and e columns
            // are exactly minus the change of the right-hand side
            const Eigen::MatrixXd A = Eigen::MatrixXd(follow_edges(k, x).matrix.to_sparse());
            for (int c : {u.iv, u.iv + 1, u.iv + 2, u.io, u.io + 1, u.io + 2})
            {
                CAPTURE(c);
                constexpr double h = 1e-3;
                AccState plus = x, minus = x;
                shift_acc_state(plus, c, h, u);
                shift_acc_state(minus, c, -h, u);
                const Eigen::VectorXd column = -(follow_edges(k, plus).rhs - follow_edges(k, minus).rhs) / (2.0 * h);
                CHECK((A.col(c) - column).norm() <= 1e-6 * column.norm());
                CHECK(A(c, c) > 0.0);
            }
        }
        SUBCASE("a window at rest or a guess not integrated: the edges as without the setting, bit for bit")
        {
            AccState x = s;
            x.window.velocity.start += Eigen::Vector3d(0.2, -0.1, 0.05);
            x.offset = Eigen::Vector3d(0.03, -0.02, 0.04);
            FollowCase rest = k, not_integrated = k;
            rest.imu.at_rest = true;
            rebuild_follow_noise(rest);
            not_integrated.imu.guess_integrated = false;
            for (const FollowCase *c : {&rest, &not_integrated})
            {
                FollowCase c_off = *c;
                c_off.imu.edges_follow_state = false;
                const WindowEquations before = follow_edges(c_off, x);
                const WindowEquations e = follow_edges(*c, x);
                CHECK(e.motion_cost == before.motion_cost);
                CHECK(e.rhs == before.rhs);
                CHECK(Eigen::MatrixXd(e.matrix.to_sparse()) == Eigen::MatrixXd(before.matrix.to_sparse()));
                CHECK(e.residuals == before.residuals);
            }
        }
    }
}

TEST_CASE("window terms: each term counts the scalar residuals it adds")
{
    SUBCASE("accelerometer term, velocity priors, offset prior")
    {
        AccCase k;
        make_acc_case(k);
        const WindowUnknowns &u = k.unknowns;
        REQUIRE(u.anchors.size() >= 3);
        WindowEquations acc(u);
        const bool end_tied = add_acceleration_terms(acc, k.noise, k.state.window, k.imu, k.state.offset);
        REQUIRE(end_tied);
        // a triple per inner anchor, the start and the end velocity term: 3 residuals each
        CHECK(acc.residuals == 3 * static_cast<int>(u.anchors.size() - 2 + 2));
        k.imu.at_rest = true;
        rebuild_noise(k);
        WindowEquations rest(u);
        add_velocity_priors(rest, k.noise, k.state.window, true);
        CHECK(rest.residuals == 3 + 6); // start prior, both velocities zero
        k.imu.at_rest = false;
        rebuild_noise(k);
        WindowEquations untied(u);
        add_velocity_priors(untied, k.noise, k.state.window, false);
        CHECK(untied.residuals == 3 + 3); // start prior, weak end tie
        WindowEquations offset(u);
        add_acceleration_offset_prior(offset, k.noise, k.state.offset);
        CHECK(offset.residuals == 3);
    }
    SUBCASE("edges, gyro-bias prior, velocity continuity, origin fix")
    {
        const Window w = straight_window(12, 0.002);
        MotionModelRotationOptions rotation;
        rotation.estimate_bias = true;
        const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
        MotionModelOptions m;
        m.x_sigma_m = m.y_sigma_m = m.z_sigma_m = 1e-3;
        m.om_sigma_deg = m.fi_sigma_deg = m.ka_sigma_deg = 0.1;
        m.fix_origin_x_sigma_m = m.fix_origin_y_sigma_m = m.fix_origin_z_sigma_m = 1e-3;
        m.fix_origin_om_sigma_deg = m.fix_origin_fi_sigma_deg = m.fix_origin_ka_sigma_deg = 1e-2;
        m.acc_sigma_mps2 = 1.0;
        const VelocityPrior previous{true, Eigen::Vector3d(1.0, 0.0, 0.0)};
        const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, rotation, {}, previous);
        WindowEquations edges(u);
        add_motion_edge_terms(edges, noise, w, m, rotation, {}, Eigen::Vector3d::Zero());
        CHECK(edges.residuals == 6 * 11); // position and rotation, per edge
        WindowEquations bias(u);
        add_gyro_bias_prior(bias, noise, w);
        CHECK(bias.residuals == 3);
        WindowEquations continuity(u);
        add_velocity_continuity(continuity, noise, w);
        CHECK(continuity.residuals == 3 * 10 + 3); // one per inner node, and the tie to the previous window
        WindowEquations origin(u);
        fix_origin(origin, noise);
        CHECK(origin.residuals == 6);
    }
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
    NdtNoise options;
    options.projection_ratio = 0.1;
    options.projection_ratio_full = 0.2;
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
        options.projection_ratio_full = 0.0;
        WindowEquations e(u);
        add_projected_ndt_blocks(e, sums, options);
        CHECK(e.rhs.isZero());
        CHECK(e.matrix(0, 0) == 0.0);
    }
    CHECK(window_translation_information(sums).isApprox(3.0 * sums.normal_information[0]));
}
