#include <doctest/doctest.h>

#include "accelerometer_term.h"
#include "angle_units.h"
#include "motion_model_terms.h"
#include "window_noise_model.h"
#include "window_solver.h"
#include "window_term_table.h"

#include <cmath>
#include <vector>

// The window noise model (window_noise_model.h, stage 5): each weight against its formula, the conditions under which a
// weight exists, and the same values the window's terms put into the normal equations.

namespace
{
    // n nodes along x at 1 m/s, the steps alternating between dt and 1.5 dt (so each edge has its own weight)
    Window uneven_window(int n, double dt)
    {
        Window w;
        double t = 0.0;
        for (int k = 0; k < n; k++)
        {
            Eigen::Affine3d p = Eigen::Affine3d::Identity();
            p.translation() = Eigen::Vector3d(t, 0.0, 0.0);
            w.poses.push_back(p);
            w.timestamps.push_back({t, t});
            t += (k % 2 == 0) ? dt : 1.5 * dt;
        }
        w.motion_model_poses = w.poses;
        return w;
    }

    MotionModelOptions motion_model()
    {
        MotionModelOptions m;
        m.x_sigma_m = 0.002;
        m.y_sigma_m = 0.003;
        m.z_sigma_m = 0.004;
        m.om_sigma_deg = 0.05;
        m.fi_sigma_deg = 0.06;
        m.ka_sigma_deg = 0.07;
        m.fix_origin_x_sigma_m = 1e-6;
        m.fix_origin_y_sigma_m = 2e-6;
        m.fix_origin_z_sigma_m = 3e-6;
        m.fix_origin_om_sigma_deg = 1e-6;
        m.fix_origin_fi_sigma_deg = 2e-6;
        m.fix_origin_ka_sigma_deg = 3e-6;
        m.acc_sigma_mps2 = 2.0;
        return m;
    }

    MotionModelRotationOptions rotation(bool estimate_bias)
    {
        MotionModelRotationOptions r;
        r.time_scaled_sigma = true;
        r.arw_rad_sqrt_s = Eigen::Vector3d(1e-4, 2e-4, 3e-4);
        r.inflation_k = 4.0;
        r.sigma_floor_rad = 2e-6;
        r.estimate_bias = estimate_bias;
        r.bias_prior_mean = Eigen::Vector3d(1e-3, -2e-3, 3e-3);
        Eigen::Matrix3d a;
        a << 1.0, 0.1, 0.0, 0.1, 1.2, 0.05, 0.0, 0.05, 0.8;
        r.bias_prior_cov = a * 1e-6;
        return r;
    }

    struct Imu
    {
        std::vector<Eigen::Vector3d> acc;
        MotionModelTranslationImuOptions options;
    };

    void imu_term(Imu &imu, int n, bool at_rest)
    {
        imu.acc.assign(n, Eigen::Vector3d(0.0, 0.0, 1.0));
        imu.options.enabled = true;
        imu.options.acc = &imu.acc;
        imu.options.span_s = 0.005;
        imu.options.noise_density_mps2_sqrt_hz = 0.0013;
        imu.options.floor_sigma_mps2 = 0.1;
        imu.options.position_information_scale = 0.1;
        Eigen::Matrix3d a;
        a << 1.0, 0.1, 0.0, 0.1, 1.2, 0.05, 0.0, 0.05, 0.8;
        imu.options.velocity_prior_mean = Eigen::Vector3d(1.0, 0.0, 0.0);
        imu.options.velocity_prior_cov = a * 4e-4;
        imu.options.offset_prior_mean = Eigen::Vector3d(0.0, -0.01, 0.0);
        imu.options.offset_prior_cov = a * 1e-4;
        imu.options.at_rest = at_rest;
        imu.options.zero_velocity_sigma_mps = 0.01;
    }
}

TEST_CASE("window noise model: the motion edges' weights follow their formulas")
{
    const Window w = uneven_window(12, 0.002);
    const MotionModelOptions m = motion_model();
    for (const double k : {4.0, 9.0}) // the run's k and a turn window's
    {
        CAPTURE(k);
        MotionModelRotationOptions r = rotation(true);
        r.inflation_k = k;
        const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, r, {}, VelocityPrior{});
        REQUIRE(noise.edges.rotation.size() == 11);
        for (size_t i = 0; i < noise.edges.rotation.size(); i++)
        {
            const double dt = w.timestamps[i + 1].first - w.timestamps[i].first;
            CHECK(noise.edges.dt[i] == dt);
            for (int axis = 0; axis < 3; axis++)
            {
                const double sigma = std::max(k * r.arw_rad_sqrt_s(axis) * std::sqrt(dt), r.sigma_floor_rad);
                CHECK(noise.edges.rotation[i](axis) == 1.0 / (sigma * sigma));
            }
        }
        CHECK(noise.edges.x == 1.0 / (m.x_sigma_m * m.x_sigma_m)); // no accelerometer term: no position scale
    }
    SUBCASE("the floor holds at zero dt")
    {
        CHECK(time_scaled_rotation_sigma(4.0, 1e-4, 0.0, 2e-6) == 2e-6);
    }
    SUBCASE("the position scale only with the accelerometer term")
    {
        Imu imu;
        imu_term(imu, 12, false);
        const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, rotation(true), imu.options, VelocityPrior{});
        REQUIRE(noise.unknowns.acc_term);
        double x = 1.0 / (m.x_sigma_m * m.x_sigma_m);
        x *= 0.1;
        CHECK(noise.edges.x == x);
    }
}

TEST_CASE("window noise model: the velocity continuity and the first step, only without the accelerometer term")
{
    const Window w = uneven_window(10, 0.002);
    const MotionModelOptions m = motion_model();
    const VelocityPrior prior{true, Eigen::Vector3d(0.8, 0.1, 0.0)};
    const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, rotation(false), {}, prior);
    REQUIRE(noise.velocity_continuity);
    REQUIRE(noise.velocity_continuity_weights.size() == 8);
    for (size_t i = 1; i + 1 < w.poses.size(); i++)
    {
        const double dt1 = w.timestamps[i].first - w.timestamps[i - 1].first, dt2 = w.timestamps[i + 1].first - w.timestamps[i].first;
        const double sigma = m.acc_sigma_mps2 * 0.5 * (dt1 + dt2);
        CHECK(noise.velocity_continuity_weights[i - 1] == 1.0 / (sigma * sigma));
    }
    REQUIRE(noise.first_step_prior);
    const double sigma = m.acc_sigma_mps2 * (w.timestamps[1].first - w.timestamps[0].first);
    CHECK(noise.first_step_weight == 1.0 / (sigma * sigma));
    CHECK(noise.first_step_velocity == prior.velocity);

    Imu imu;
    imu_term(imu, 10, false);
    const WindowNoiseModel with_acc = build_window_noise_model(w, ScanToMapOptions{}, m, rotation(false), imu.options, prior);
    CHECK_FALSE(with_acc.velocity_continuity);
    CHECK(with_acc.velocity_continuity_weights.empty());
    CHECK_FALSE(with_acc.first_step_prior);
}

TEST_CASE("window noise model: the accelerometer terms' spans and weights")
{
    const Window w = Window(uneven_window(30, 0.002));
    Imu imu;
    imu_term(imu, 30, false);
    const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, motion_model(), rotation(true), imu.options, VelocityPrior{});
    REQUIRE(noise.unknowns.acc_term);
    const std::vector<int> &anchors = noise.unknowns.anchors;
    REQUIRE(anchors.size() >= 4);
    // anchor triples, then the start velocity, then the end velocity
    REQUIRE(noise.accelerometer_terms.size() == anchors.size() - 2 + 2);
    const double d2 = 0.0013 * 0.0013, f2 = 0.1 * 0.1;
    for (size_t j = 0; j < noise.accelerometer_terms.size(); j++)
    {
        const AccelerometerTermSpan &s = noise.accelerometer_terms[j];
        CAPTURE(j);
        CHECK(s.weight == 1.0 / ((d2 / s.tau + f2) * s.tau * s.tau));
        CHECK(s.tau > 0.0);
    }
    const AccelerometerTermSpan &first = noise.accelerometer_terms.front();
    CHECK(first.kind == AccelerometerTermKind::anchor_triple);
    CHECK(first.tau == 0.5 * (w.timestamps[anchors[1]].first + w.timestamps[anchors[2]].first) -
                           0.5 * (w.timestamps[anchors[0]].first + w.timestamps[anchors[1]].first));
    const AccelerometerTermSpan &start = noise.accelerometer_terms[noise.accelerometer_terms.size() - 2];
    CHECK(start.kind == AccelerometerTermKind::start_velocity);
    CHECK(start.tau == 0.5 * (w.timestamps[anchors[1]].first - w.timestamps[0].first));
    const AccelerometerTermSpan &end = noise.accelerometer_terms.back();
    CHECK(end.kind == AccelerometerTermKind::end_velocity);
    CHECK(end.c == 29);
    CHECK(noise.end_velocity_tied);

    SUBCASE("a gap leaves the terms across it out")
    {
        Window gap = w;
        for (size_t k = 15; k < gap.timestamps.size(); k++)
        {
            gap.timestamps[k].first += 0.05; // 50 ms between nodes 14 and 15: more than 2 span_s
            gap.timestamps[k].second += 0.05;
        }
        const WindowNoiseModel g = build_window_noise_model(gap, ScanToMapOptions{}, motion_model(), rotation(true), imu.options, VelocityPrior{});
        CHECK(g.accelerometer_terms.size() < noise.accelerometer_terms.size());
    }
}

TEST_CASE("window noise model: what exists in the first window, at rest and without timestamps")
{
    const Window w = uneven_window(20, 0.002);
    const MotionModelOptions m = motion_model();
    SUBCASE("the first window: no bias estimation, no accelerometer term")
    {
        const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, rotation(false), {}, VelocityPrior{});
        CHECK_FALSE(noise.unknowns.estimate_bias);
        CHECK(noise.bias_prior_information.isZero());
        CHECK(noise.accelerometer_terms.empty());
        CHECK(noise.velocity_prior_information.isZero());
        CHECK(noise.offset_prior_information.isZero());
        CHECK(noise.zero_velocity_weight == 0.0);
    }
    SUBCASE("a window at rest gets the zero-velocity weight")
    {
        Imu imu;
        imu_term(imu, 20, true);
        const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, rotation(true), imu.options, VelocityPrior{});
        CHECK(noise.at_rest);
        CHECK(noise.zero_velocity_weight == 1.0 / (0.01 * 0.01));
    }
    SUBCASE("without timestamps: no time scaling, no bias, no accelerometer term, no velocity continuity")
    {
        Window no_time = w;
        no_time.timestamps.clear();
        Imu imu;
        imu_term(imu, 20, false);
        const WindowNoiseModel noise = build_window_noise_model(no_time, ScanToMapOptions{}, m, rotation(true), imu.options, VelocityPrior{});
        CHECK_FALSE(noise.unknowns.time_scaled_sigma);
        CHECK_FALSE(noise.unknowns.acc_term);
        CHECK_FALSE(noise.velocity_continuity);
        const double sigma = m.om_sigma_deg * DEG_TO_RAD;
        CHECK(noise.edges.rotation[0](0) == 1.0 / (sigma * sigma));
    }
}

TEST_CASE("window noise model: the same values the terms put into the normal equations")
{
    const Window w = uneven_window(30, 0.002);
    const MotionModelOptions m = motion_model();
    const MotionModelRotationOptions r = rotation(true);
    Imu imu;
    imu_term(imu, 30, true);
    const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, m, r, imu.options, VelocityPrior{});
    const WindowUnknowns &u = noise.unknowns;
    REQUIRE(u.estimate_bias);
    REQUIRE(u.acc_term);

    WindowEquations bias(u);
    add_gyro_bias_prior(bias, noise, w);
    WindowEquations velocity(u);
    add_start_velocity_prior(velocity, noise, w);
    WindowEquations rest(u);
    add_zero_velocity(rest, noise, w);
    WindowEquations offset(u);
    add_acceleration_offset_prior(offset, noise, Eigen::Vector3d::Zero());
    WindowEquations origin(u);
    fix_origin(origin, noise);
    for (int a = 0; a < 3; a++)
    {
        for (int b = 0; b < 3; b++)
        {
            CHECK(bias.matrix(u.ib + a, u.ib + b) == noise.bias_prior_information(a, b));
            CHECK(velocity.matrix(u.iv + a, u.iv + b) == noise.velocity_prior_information(a, b));
            CHECK(offset.matrix(u.io + a, u.io + b) == noise.offset_prior_information(a, b));
        }
        CHECK(rest.matrix(u.iv + a, u.iv + a) == noise.zero_velocity_weight);
        CHECK(rest.matrix(u.ie + a, u.ie + a) == noise.zero_velocity_weight);
    }
    for (int k = 0; k < 6; k++)
        CHECK(origin.matrix(k, k) == noise.origin[size_t(k)]);
}

TEST_CASE("window noise model: the priors grow by their random walk over the window and by the noise over the gap")
{
    Eigen::Matrix3d carried;
    carried << 4.0, 0.5, 0.0, 0.5, 3.0, 0.2, 0.0, 0.2, 2.0;
    carried *= 1e-6;
    const double rw = 2e-4, duration = 0.4;
    const Eigen::Matrix3d grown = random_walk_prior_cov(carried, rw, duration);
    CHECK(grown == carried + Eigen::Matrix3d::Identity() * (rw * rw * duration));
    CHECK((grown - carried).isDiagonal());
    CHECK(random_walk_prior_cov(carried, rw, 0.0) == carried);

    const double density = 0.0013, floor = 0.1, gap = 0.002;
    const Eigen::Matrix3d v = velocity_prior_cov_over_gap(carried, density, floor, gap);
    CHECK(v == carried + Eigen::Matrix3d::Identity() * (density * density * gap + floor * floor * gap * gap));
    CHECK(v(0, 0) > carried(0, 0));
    CHECK(v(0, 1) == carried(0, 1));
}

// consistency-harness D9: the two features of the per-window record (window_term_table.h)
TEST_CASE("window term table: mean rotation rate from the attitude increments")
{
    // 11 nodes, 0.1 s apart, turning about z by 2 deg per step, then back by 1 deg per step: angles add, signs don't cancel
    std::vector<Eigen::Affine3d> poses;
    std::vector<std::pair<double, double>> timestamps;
    double heading = 0.0;
    for (int k = 0; k <= 10; k++)
    {
        Eigen::Affine3d p = Eigen::Affine3d::Identity();
        p.linear() = (Eigen::AngleAxisd(heading * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX())).toRotationMatrix();
        poses.push_back(p);
        timestamps.push_back({100.0 + 0.1 * k, 0.0});
        heading += k < 5 ? 2.0 : -1.0;
    }
    // 5 steps of 2 deg and 5 of 1 deg over 1 s; the increments are in each node's frame, so the tilt doesn't change the angle
    const std::optional<double> rate = mean_rotation_rate(poses, timestamps);
    REQUIRE(rate);
    CHECK(*rate * RAD_TO_DEG == doctest::Approx(15.0).epsilon(1e-9));

    CHECK_FALSE(mean_rotation_rate({poses[0]}, {timestamps[0]}));
    CHECK_FALSE(mean_rotation_rate(poses, std::vector<std::pair<double, double>>(timestamps.begin(), timestamps.end() - 1)));
    CHECK_FALSE(mean_rotation_rate(poses, std::vector<std::pair<double, double>>(poses.size(), {5.0, 0.0})));
}

TEST_CASE("window term table: world-frame acceleration at the window's start")
{
    // the body is turned 90 deg about z; the accelerometer reads gravity (1.02 units) plus 0.1 units along body x, i.e. world y,
    // for the first 30 ms, then something else; nodes every 10 ms
    const Eigen::Vector3d gravity(0.0, 0.0, 1.02);
    const double to_mps2 = 9.80665 / 1.02;
    Eigen::Affine3d turned = Eigen::Affine3d::Identity();
    turned.linear() = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    std::vector<Eigen::Affine3d> poses(8, turned);
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Eigen::Vector3d> acc;
    for (int k = 0; k < 8; k++)
    {
        timestamps.push_back({50.0 + 0.01 * k, 0.0});
        acc.push_back(k <= 2 ? Eigen::Vector3d(0.1, 0.0, 1.02) : Eigen::Vector3d(-5.0, 3.0, 0.0));
    }
    // the span holds the nodes at 0, 10, 20 ms (30 ms lies at the edge in floating point: take a span that is clearly inside)
    const std::optional<Eigen::Vector3d> a = start_world_acceleration(poses, timestamps, acc, gravity, 0.025);
    REQUIRE(a);
    CHECK(a->x() == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(a->y() == doctest::Approx(0.1 * to_mps2).epsilon(1e-12));
    CHECK(a->z() == doctest::Approx(0.0).epsilon(1e-12));
    // a span shorter than one step: node 0 alone
    const std::optional<Eigen::Vector3d> first = start_world_acceleration(poses, timestamps, acc, gravity, 0.0);
    REQUIRE(first);
    CHECK(first->y() == doctest::Approx(0.1 * to_mps2).epsilon(1e-12));
    // the default span, 30 ms, also takes the node at 30 ms when it falls inside
    std::vector<std::pair<double, double>> exact = timestamps;
    exact[3].first = exact[0].first + 0.029;
    const std::optional<Eigen::Vector3d> wider = start_world_acceleration(poses, exact, acc, gravity);
    REQUIRE(wider);
    CHECK(wider->y() != doctest::Approx(0.1 * to_mps2));

    CHECK_FALSE(start_world_acceleration(poses, timestamps, {}, gravity));
    CHECK_FALSE(start_world_acceleration(poses, timestamps, acc, Eigen::Vector3d::Zero()));
    CHECK_FALSE(start_world_acceleration({}, {}, {}, gravity));
}
