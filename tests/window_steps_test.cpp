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

TEST_CASE("place_windows_after chains the windows from first on, each after its predecessor's end")
{
    // windows at unrelated places, as cut from the data before the odometry
    std::vector<Window> original;
    for (int i = 0; i < 6; i++)
        original.push_back(moving_window(25, 1.0 + 0.2 * i, 0.1 * i, pose(10.0 * i, -3.0 * i, 0.5 * i, 0.4 * i)));

    std::vector<Window> windows = original;
    place_windows_after(windows, 2);

    for (size_t i = 0; i < 2; i++)
        for (size_t k = 0; k < windows[i].poses.size(); k++)
            CHECK(windows[i].poses[k].isApprox(original[i].poses[k], 0.0));
    for (size_t i = 2; i < windows.size(); i++)
    {
        CHECK(windows[i].poses[0].isApprox(windows[i - 1].poses.back(), 1e-12));
        for (size_t k = 1; k < windows[i].poses.size(); k++)
        {
            const Eigen::Affine3d rel_new = windows[i].poses[k - 1].inverse() * windows[i].poses[k];
            const Eigen::Affine3d rel_old = original[i].poses[k - 1].inverse() * original[i].poses[k];
            CHECK(rel_new.isApprox(rel_old, 1e-12));
        }
    }

    SUBCASE("one call matches the old placement of all later windows after every window")
    {
        std::vector<Window> repeated = original;
        for (size_t i = 0; i < repeated.size(); i++)
            for (size_t j = i + 1; j < repeated.size(); j++)
                place_window_after(repeated[j], repeated[j - 1].poses.back());

        std::vector<Window> once = original;
        place_windows_after(once, 1);
        for (size_t i = 0; i < once.size(); i++)
            for (size_t k = 0; k < once[i].poses.size(); k++)
                CHECK((once[i].poses[k].matrix() - repeated[i].poses[k].matrix()).cwiseAbs().maxCoeff() < 1e-9);
    }

    SUBCASE("first = 0 and first = size change nothing")
    {
        for (size_t first : {size_t(0), windows.size()})
        {
            std::vector<Window> w = original;
            place_windows_after(w, first);
            for (size_t i = 0; i < w.size(); i++)
                for (size_t k = 0; k < w[i].poses.size(); k++)
                    CHECK(w[i].poses[k].isApprox(original[i].poses[k], 0.0));
        }
    }
}

TEST_CASE("estimate_previous_velocity measures the previous window's final velocity")
{
    const Window previous = moving_window(50, 1.5, 0.0);

    const PreviousVelocity v = estimate_previous_velocity(previous, 20, 5.0);
    CHECK(v.use);
    CHECK_FALSE(v.dropped);
    CHECK(v.velocity.isApprox(Eigen::Vector3d(1.5, 0.0, 0.0), 1e-9));

    // faster than the limit: dropped, and the speed is reported
    const PreviousVelocity fast = estimate_previous_velocity(moving_window(50, 7.0, 0.0), 20, 5.0);
    CHECK_FALSE(fast.use);
    CHECK(fast.dropped);
    CHECK(fast.speed == doctest::Approx(7.0));
    CHECK(fast.velocity == Eigen::Vector3d::Zero());

    // no usable span: a single node, or timestamps that don't match the poses
    CHECK_FALSE(estimate_previous_velocity(moving_window(1, 1.5, 0.0), 20, 5.0).use);
    Window mismatched = moving_window(50, 1.5, 0.0);
    mismatched.timestamps.pop_back();
    CHECK_FALSE(estimate_previous_velocity(mismatched, 20, 5.0).use);
}

namespace
{
    // accelerometer [g] and level AHRS angles for a window with identity orientations and world acceleration a_mps2
    void add_imu(Window &w, const Eigen::Vector3d &a_mps2)
    {
        w.imu_acc.assign(w.poses.size(), Eigen::Vector3d::UnitZ() + a_mps2 / 9.80665);
        w.imu_angles.assign(w.poses.size(), Eigen::Vector3d::Zero());
    }
}

TEST_CASE("predict_window_poses guesses the window's motion")
{
    const int n = 40;
    const Window previous = moving_window(n, 1.0, 0.0, pose(3.0, 1.0, 0.0, 0.0));
    const Eigen::Vector3d end = previous.poses.back().translation();
    const Eigen::Vector3d v0(1.0, 0.0, 0.0);

    SUBCASE("constant velocity from the previous window")
    {
        Window w = moving_window(n, 0.0, 0.0);
        PosePredictionOptions o;
        o.constant_velocity = true;
        const PosePrediction r = predict_window_poses(w, previous, nullptr, v0, o);
        CHECK_FALSE(r.stationary);
        for (int k = 0; k < n; k++)
            CHECK(w.poses[k].translation().isApprox(end + v0 * (k * dt), 1e-12));
        CHECK(w.motion_model_poses.size() == w.poses.size());
    }

    SUBCASE("IMU integration: p = v t + a t^2 / 2")
    {
        Window w = moving_window(n, 0.0, 0.0);
        const Eigen::Vector3d a(0.5, -0.2, 0.0);
        add_imu(w, a);
        PosePredictionOptions o;
        o.use_imu_pred = true;
        predict_window_poses(w, previous, nullptr, v0, o);
        for (int k = 0; k < n; k++)
        {
            const double t = k * dt;
            CHECK(w.poses[k].translation().isApprox(end + v0 * t + 0.5 * a * t * t, 1e-9));
        }
    }

    SUBCASE("a window at rest is predicted without motion")
    {
        Window w = moving_window(n, 0.0, 0.0);
        add_imu(w, Eigen::Vector3d::Zero());
        PosePredictionOptions o;
        o.use_zupt = true;
        o.zupt_acc_threshold_mps2 = 0.3;
        o.zupt_gyro_threshold_rad_s = 0.05;
        const PosePrediction r = predict_window_poses(w, previous, nullptr, v0, o);
        CHECK(r.stationary);
        for (int k = 0; k < n; k++)
            CHECK(w.poses[k].translation().isApprox(end, 1e-12));
    }

    SUBCASE("mean shift between the two previous windows, and its plausibility limit")
    {
        const Window before = moving_window(n, 1.0, 0.0, pose(3.0 - n * dt, 1.0, 0.0, 0.0));
        Window w = moving_window(n, 0.0, 0.0);
        const PosePrediction r = predict_window_poses(w, previous, &before, v0, PosePredictionOptions());
        CHECK_FALSE(r.mean_shift_dropped);
        const Eigen::Vector3d shift = (end - before.poses.back().translation()) / double(n);
        for (int k = 0; k < n; k++)
            CHECK(w.poses[k].translation().isApprox(end + shift * k, 1e-12));

        const Window far = moving_window(n, 1.0, 0.0, pose(-100.0, 1.0, 0.0, 0.0));
        Window w2 = moving_window(n, 0.0, 0.0);
        CHECK(predict_window_poses(w2, previous, &far, v0, PosePredictionOptions()).mean_shift_dropped);
        for (int k = 0; k < n; k++)
            CHECK(w2.poses[k].translation().isApprox(end, 1e-12));
    }
}

TEST_CASE("apply_gyro_bias_prior grows the prior and removes the bias from the guess")
{
    const int n = 50;
    const double bias = 0.1; // rad/s about z: the guess turns only because of it
    Window w = moving_window(n, 1.0, bias);
    GyroBiasState carried;
    carried.mean = Eigen::Vector3d(0.0, 0.0, bias);
    carried.covariance = Eigen::Matrix3d::Identity() * 1e-4;
    const double rw = 0.01;

    const GyroBiasState prior = apply_gyro_bias_prior(w, carried, rw);
    const double duration = (n - 1) * dt;
    CHECK(prior.mean == carried.mean);
    CHECK(prior.covariance.isApprox(carried.covariance + Eigen::Matrix3d::Identity() * (rw * rw * duration), 1e-15));
    CHECK(w.gyro_bias.mean == prior.mean);
    CHECK(w.gyro_bias.covariance == prior.covariance);
    for (int k = 0; k < n; k++)
    {
        CHECK(w.poses[k].linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
        CHECK(w.motion_model_poses[k].matrix() == w.poses[k].matrix());
    }
}

TEST_CASE("is_turn_window compares the mean yaw rate with the threshold")
{
    const double threshold = 30.0 * 3.14159265358979323846 / 180.0; // 30 deg/s
    CHECK(is_turn_window(moving_window(40, 1.0, 1.0), threshold));         // 57 deg/s
    CHECK(is_turn_window(moving_window(40, 1.0, -1.0), threshold));        // either direction
    CHECK_FALSE(is_turn_window(moving_window(40, 1.0, 0.2), threshold));   // 11 deg/s
    CHECK_FALSE(is_turn_window(moving_window(1, 1.0, 1.0), threshold));    // a single node
}

TEST_CASE("repropagate_window_tail continues the tail with the kept velocity and the IMU rotation")
{
    const int n = 60;
    const int tail = 5;
    // optimized poses: along x at 1 m/s, but the tail went astray (sideways jump)
    Window w = moving_window(n, 1.0, 0.0);
    const Window imu = moving_window(n, 1.0, 0.4); // the IMU says the device turns at 0.4 rad/s
    w.motion_model_poses = imu.poses;
    for (int k = n - tail; k < n; k++)
        w.poses[k].translation().y() += 0.3;
    const Window before = w;

    SUBCASE("no tail nodes: nothing changes")
    {
        repropagate_window_tail(w, 0, 20, Eigen::Vector3d::Zero());
        for (int k = 0; k < n; k++)
            CHECK(w.poses[k].matrix() == before.poses[k].matrix());
    }

    SUBCASE("the tail continues from the last kept node")
    {
        repropagate_window_tail(w, tail, 20, Eigen::Vector3d::Zero());
        const int a = n - 1 - tail;
        for (int k = 0; k <= a; k++)
            CHECK(w.poses[k].matrix() == before.poses[k].matrix());
        for (int k = a + 1; k < n; k++)
        {
            CHECK(w.poses[k].translation().isApprox(Eigen::Vector3d(k * dt, 0.0, 0.0), 1e-9));
            // rotation: the kept node's rotation (identity) followed by the IMU increments
            const Eigen::Matrix3d expected = Eigen::AngleAxisd(0.4 * (k - a) * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            CHECK(w.poses[k].linear().isApprox(expected, 1e-9));
        }
    }

    SUBCASE("a short velocity span (a slower IMU) gives the same tail at constant velocity")
    {
        repropagate_window_tail(w, tail, 4, Eigen::Vector3d::Zero());
        for (int k = n - tail; k < n; k++)
            CHECK(w.poses[k].translation().isApprox(Eigen::Vector3d(k * dt, 0.0, 0.0), 1e-9));
    }

    SUBCASE("no velocity span, or one longer than the kept nodes: nothing changes")
    {
        repropagate_window_tail(w, tail, 0, Eigen::Vector3d::Zero());
        repropagate_window_tail(w, tail, n - tail, Eigen::Vector3d::Zero());
        for (int k = 0; k < n; k++)
            CHECK(w.poses[k].matrix() == before.poses[k].matrix());
    }

    SUBCASE("a bias update removes rotation from the IMU increments")
    {
        repropagate_window_tail(w, tail, 20, Eigen::Vector3d(0.0, 0.0, 0.4));
        for (int k = n - tail; k < n; k++)
            CHECK(w.poses[k].linear().isApprox(Eigen::Matrix3d::Identity(), 1e-9));
    }
}

TEST_CASE("convergence_of finds when the poses settle and when delta drops below the fast threshold")
{
    // 10 iterations of a 3-node window: node 2 moves by 1 cm, 1 mm, 0.1 mm, ... until iteration 6, then stays
    std::vector<std::vector<Eigen::Affine3d>> poses_after;
    std::vector<double> deltas;
    double x = 0.0;
    for (int i = 0; i < 10; i++)
    {
        if (i < 6)
            x += 0.01 * std::pow(0.1, i);
        poses_after.push_back({pose(0, 0, 0, 0), pose(1, 0, 0, 0), pose(2 + x, 0, 0, 0)});
        deltas.push_back(0.5 * std::pow(0.1, i)); // 0.5, 0.05, 0.005 (< 1e-2 after 3 iterations), ...
    }

    const IterationConvergence c = convergence_of(poses_after, deltas, 1e-4, 0.001 * 3.14159265358979 / 180.0, 1e-2);
    CHECK(c.iterations == 10);
    // after iteration 3 (index 2) node 2 is 1e-5 + 1e-6 + 1e-7 short of the end: within 1e-4; after iteration 2 it is 1.1e-4 short
    CHECK(c.settled_after == 3);
    CHECK(c.fast_stop_after == 3);
    CHECK(c.fast_stop_position_error_m == doctest::Approx(1.11e-5).epsilon(1e-6));
    CHECK(c.fast_stop_rotation_error_rad == doctest::Approx(0.0));

    SUBCASE("a rotation that settles late")
    {
        poses_after[7][1] = pose(1, 0, 0, 0.01); // iteration 8 rotates node 1 by 0.57 deg
        const IterationConvergence r = convergence_of(poses_after, deltas, 1e-4, 0.001 * 3.14159265358979 / 180.0, 1e-2);
        CHECK(r.settled_after == 9);
    }

    SUBCASE("a run cut short is measured against its last iteration")
    {
        poses_after.resize(2);
        deltas.resize(2);
        const IterationConvergence r = convergence_of(poses_after, deltas, 1e-4, 0.001 * 3.14159265358979 / 180.0, 1e-2);
        CHECK(r.iterations == 2);
        CHECK(r.settled_after == 2);
        CHECK(r.fast_stop_after == 0);
    }
}

TEST_CASE("integrate_window_translations follows the accelerometer from the start velocity")
{
    // turning at 0.3 rad/s with a constant world acceleration; the accelerometer reads R^T (a + g) in g. The integration is exact
    // for a constant acceleration, whatever the node spacing (here uneven, as IMU bursts are)
    const double g = 9.80665;
    const Eigen::Vector3d a(1.0, -0.5, 0.2), v0(1.2, 0.3, 0.0), start(2.0, -1.0, 0.5);
    Window w;
    double t = 0.0;
    for (int k = 0; k < 40; k++)
    {
        w.poses.push_back(pose(0.0, 0.0, 0.0, 0.3 * t)); // translations wrong on purpose (all at the origin)
        w.timestamps.push_back({t, 0.0});
        w.imu_acc.push_back(w.poses.back().linear().transpose() * (a + Eigen::Vector3d(0.0, 0.0, g)) / g);
        t += (k % 3 == 0) ? 0.0002 : 0.0015;
    }
    w.poses[0].translation() = start;
    REQUIRE(integrate_window_translations(w, v0, Eigen::Vector3d(0.0, 0.0, 1.0), g));
    double worst = 0.0;
    for (size_t k = 0; k < w.poses.size(); k++)
    {
        const double tk = w.timestamps[k].first;
        worst = std::max(worst, (w.poses[k].translation() - (start + v0 * tk + 0.5 * a * tk * tk)).norm());
        CHECK(w.motion_model_poses[k].isApprox(w.poses[k]));
    }
    CHECK(worst < 1e-12);
    CHECK(Eigen::AngleAxisd(w.poses.back().linear()).angle() == doctest::Approx(0.3 * w.timestamps.back().first)); // rotations kept

    Window no_imu = moving_window(5, 1.0, 0.0);
    CHECK_FALSE(integrate_window_translations(no_imu, v0, Eigen::Vector3d(0.0, 0.0, 1.0), g));
}

TEST_CASE("window_rests: still with gravity only, not while walking")
{
    // 200 nodes 2 ms apart, level and not turning; the accelerometer reads gravity (1 g up), and in the second case a
    // walking acceleration of 2 m/s^2 at 2 Hz along x on top
    constexpr int n = 200;
    std::vector<Eigen::Affine3d> poses(n, Eigen::Affine3d::Identity());
    std::vector<std::pair<double, double>> ts;
    std::vector<Eigen::Vector3d> angles(n, Eigen::Vector3d::Zero()), still, walking;
    for (int k = 0; k < n; k++)
    {
        const double t = 0.002 * k;
        ts.push_back({t, t});
        still.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
        walking.push_back(Eigen::Vector3d(2.0 / 9.80665 * std::sin(2.0 * 3.14159265358979323846 * 2.0 * t), 0.0, 1.0));
    }
    CHECK(window_rests(poses, ts, still, angles, 1.0, 0.1, 0.01));
    CHECK_FALSE(window_rests(poses, ts, walking, angles, 1.0, 0.1, 0.01));
}

namespace
{
    // n nodes 2 ms apart turning about z and x, starting at the origin
    Window turning_window(int n)
    {
        Window w;
        for (int k = 0; k < n; k++)
        {
            Eigen::Affine3d p = Eigen::Affine3d::Identity();
            p.linear() = (Eigen::AngleAxisd(0.01 * k, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.003 * k, Eigen::Vector3d::UnitX())).toRotationMatrix();
            p.translation() = Eigen::Vector3d(0.002 * k, 0.0005 * k, 0.0);
            w.poses.push_back(p);
            w.timestamps.push_back({1.0 + 0.002 * k, 1.0 + 0.002 * k});
        }
        w.imu_motion_from_previous.linear() = Eigen::AngleAxisd(0.002, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return w;
    }
}

TEST_CASE("predict_window_poses steps: the mean shift per node and its limit")
{
    Window before = turning_window(360), previous = turning_window(360);
    before.poses.back().translation() = Eigen::Vector3d(1.0, 2.0, 0.0);
    previous.poses.back().translation() = Eigen::Vector3d(1.36, 2.0, 0.0);
    bool dropped = true;
    const Eigen::Vector3d shift = mean_shift_of(previous, &before, dropped);
    CHECK_FALSE(dropped);
    CHECK(shift.x() == doctest::Approx(0.001));
    CHECK(shift.y() == doctest::Approx(0.0));
    previous.poses.back().translation() = Eigen::Vector3d(501.0, 2.0, 0.0);
    CHECK(mean_shift_of(previous, &before, dropped).isZero());
    CHECK(dropped);
    CHECK(mean_shift_of(previous, nullptr, dropped).isZero());
    CHECK_FALSE(dropped);
}

TEST_CASE("predict_window_poses steps: the chained guess equals place_window_after")
{
    Window previous = turning_window(20);
    previous.poses.back().translation() = Eigen::Vector3d(3.0, -1.0, 0.5);
    Window placed = turning_window(30), predicted = turning_window(30);
    place_window_after(placed, previous.poses.back());
    // no velocity and no mean shift: the translations stay as chained (+ 0)
    predict_window_poses(predicted, previous, nullptr, Eigen::Vector3d::Zero(), PosePredictionOptions());
    REQUIRE(predicted.poses.size() == placed.poses.size());
    for (size_t k = 0; k < placed.poses.size(); k++)
    {
        CAPTURE(k);
        CHECK(predicted.poses[k].linear() == placed.poses[k].linear());
        CHECK(predicted.poses[k].translation() == placed.poses[k].translation());
    }
    CHECK(predicted.motion_model_poses.size() == predicted.poses.size());
}

TEST_CASE("predict_window_poses steps: the four ways to the translations")
{
    const Window w = turning_window(11);
    const std::vector<Eigen::Vector3d> a(11, Eigen::Vector3d(0.5, 0.0, 0.0));
    const Eigen::Vector3d v0(1.0, 0.2, 0.0), shift(0.001, 0.0, 0.0);
    const double gap = 0.002;
    const auto run = [&](bool imu_ok, bool stationary, const PosePredictionOptions &o)
    {
        std::vector<Eigen::Affine3d> poses = w.poses;
        guess_window_translations(poses, w.timestamps, a, imu_ok, stationary, v0, shift, gap, o);
        return poses;
    };
    const Eigen::Vector3d start = w.poses[0].translation();
    SUBCASE("accelerometer integration: p = v t + a t^2 / 2 from the gap on")
    {
        PosePredictionOptions o;
        o.use_imu_pred = true;
        const auto poses = run(true, false, o);
        const double t = 0.02; // the last node, 10 steps of 2 ms
        const Eigen::Vector3d expected = start + v0 * gap + v0 * t + 0.5 * a[0] * t * t;
        CHECK((poses.back().translation() - expected).norm() < 1e-12);
    }
    SUBCASE("at rest: no motion")
    {
        PosePredictionOptions o;
        o.use_zupt = true;
        const auto poses = run(true, true, o);
        for (const Eigen::Affine3d &p : poses)
            CHECK(p.translation() == start);
    }
    SUBCASE("constant velocity over the node times and the gap")
    {
        PosePredictionOptions o;
        o.constant_velocity = true;
        const auto poses = run(false, false, o);
        for (size_t k = 0; k < poses.size(); k++)
            CHECK((poses[k].translation() - (w.poses[k].translation() + v0 * (0.002 * k + gap))).norm() < 1e-12);
    }
    SUBCASE("the mean shift per node")
    {
        const auto poses = run(false, false, PosePredictionOptions());
        for (size_t k = 0; k < poses.size(); k++)
            CHECK((poses[k].translation() - (w.poses[k].translation() + shift * double(k))).norm() < 1e-15);
    }
}
