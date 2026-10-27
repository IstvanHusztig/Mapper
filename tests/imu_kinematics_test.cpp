#include <doctest/doctest.h>

#include "imu_kinematics.h"

#include <cmath>

// Synthetic trajectories: nodes every 10 ms, built in code.

namespace
{
    constexpr double dt = 0.01;
    constexpr double pi = 3.14159265358979323846;

    std::vector<std::pair<double, double>> timestamps(int n)
    {
        std::vector<std::pair<double, double>> ts(n);
        for (int k = 0; k < n; k++)
            ts[k] = {k * dt, 0.0};
        return ts;
    }

    // turning about z at rate_rad_s, moving along x at speed_mps
    std::vector<Eigen::Affine3d> trajectory(int n, double rate_rad_s, double speed_mps)
    {
        std::vector<Eigen::Affine3d> tr(n);
        for (int k = 0; k < n; k++)
        {
            tr[k] = Eigen::Affine3d::Identity();
            tr[k].linear() = Eigen::AngleAxisd(rate_rad_s * k * dt, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            tr[k].translation() = Eigen::Vector3d(speed_mps * k * dt, 0.0, 0.0);
        }
        return tr;
    }
}

TEST_CASE("exp_so3 turns about the vector by its length")
{
    CHECK(exp_so3(Eigen::Vector3d::Zero()).isApprox(Eigen::Matrix3d::Identity()));
    const Eigen::Vector3d x = exp_so3(Eigen::Vector3d(0.0, 0.0, pi / 2)) * Eigen::Vector3d::UnitX();
    CHECK(x.isApprox(Eigen::Vector3d::UnitY(), 1e-12));
}

TEST_CASE("remove_gyro_bias_from_rotations removes a constant rotation rate")
{
    const int n = 50;
    const double bias = 0.1; // rad/s about z
    const auto ts = timestamps(n);
    const auto original = trajectory(n, bias, 1.0);

    auto tr = original;
    remove_gyro_bias_from_rotations(tr, ts, Eigen::Vector3d(0.0, 0.0, bias));
    for (int k = 0; k < n; k++)
    {
        CHECK(tr[k].linear().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
        CHECK(tr[k].translation() == original[k].translation());
    }
    CHECK(tr[0].matrix() == original[0].matrix());

    auto unchanged = original;
    remove_gyro_bias_from_rotations(unchanged, ts, Eigen::Vector3d::Zero());
    for (int k = 0; k < n; k++)
        CHECK(unchanged[k].matrix() == original[k].matrix());
}

TEST_CASE("world_acceleration removes gravity and converts to m/s^2")
{
    const int n = 11;
    const auto ts = timestamps(n);
    const auto tr = trajectory(n, 0.0, 0.0);
    const std::vector<Eigen::Vector3d> level(n, Eigen::Vector3d::Zero()); // AHRS om, fi, ka
    const double gravity_norm = 1.0;                                      // accelerometer in g
    const double g = 9.80665;

    // at rest: the accelerometer reads gravity only
    const std::vector<Eigen::Vector3d> at_rest(n, Eigen::Vector3d::UnitZ());
    for (const auto &a : world_acceleration(tr, ts, at_rest, level, gravity_norm, 0.0))
        CHECK(a.norm() < 1e-12);

    // a known acceleration on top of gravity, and its average over +-15 ms (3 nodes)
    std::vector<Eigen::Vector3d> acc(n);
    std::vector<Eigen::Vector3d> expected(n);
    for (int k = 0; k < n; k++)
    {
        expected[k] = Eigen::Vector3d(0.5 * k, -1.0, 2.0);
        acc[k] = Eigen::Vector3d::UnitZ() + expected[k] / g;
    }
    const auto a = world_acceleration(tr, ts, acc, level, gravity_norm, 0.0);
    for (int k = 0; k < n; k++)
        CHECK(a[k].isApprox(expected[k], 1e-12));
    const auto smoothed = world_acceleration(tr, ts, acc, level, gravity_norm, 0.0151);
    CHECK(smoothed[5].isApprox((expected[4] + expected[5] + expected[6]) / 3.0, 1e-12));
}

TEST_CASE("observed_acceleration_offset recovers an accelerometer offset")
{
    const Eigen::Vector3d gravity = Eigen::Vector3d::UnitZ();
    const double to_mps2 = 9.80665;
    const Eigen::Vector3d e(0.1, -0.2, 0.05); // [m/s^2]

    const int n = 100;
    const auto ts = timestamps(n);
    const auto tr = trajectory(n, 0.0, 1.0); // constant velocity
    const std::vector<Eigen::Vector3d> acc(n, gravity + e / to_mps2);

    Eigen::Vector3d offset = Eigen::Vector3d::Zero();
    REQUIRE(observed_acceleration_offset(tr, ts, acc, gravity, to_mps2, 0, 20, offset));
    CHECK(offset.isApprox(e, 1e-9));

    // too short: fewer than 2 * 20 nodes between the two velocity estimates
    const int short_n = 60;
    CHECK_FALSE(observed_acceleration_offset(trajectory(short_n, 0.0, 1.0), timestamps(short_n),
                                             std::vector<Eigen::Vector3d>(short_n, gravity), gravity, to_mps2, 0, 20, offset));

    // a shorter span (a slower IMU) fits the same 60 nodes and still recovers the offset
    const std::vector<Eigen::Vector3d> short_acc(short_n, gravity + e / to_mps2);
    offset = Eigen::Vector3d::Zero();
    REQUIRE(observed_acceleration_offset(trajectory(short_n, 0.0, 1.0), timestamps(short_n), short_acc, gravity, to_mps2, 0, 4, offset));
    CHECK(offset.isApprox(e, 1e-9));
    CHECK_FALSE(observed_acceleration_offset(tr, ts, acc, gravity, to_mps2, 0, 0, offset));
}

TEST_CASE("is_window_stationary detects rest")
{
    const int n = 30;
    const auto ts = timestamps(n);
    const double acc_threshold = 0.3;   // m/s^2
    const double gyro_threshold = 0.05; // rad/s
    std::vector<Eigen::Vector3d> acc(n, Eigen::Vector3d::Zero());

    CHECK(is_window_stationary(trajectory(n, 0.0, 0.0), ts, acc, acc_threshold, gyro_threshold));

    auto one_spike = acc;
    one_spike[10] = Eigen::Vector3d(0.0, 0.4, 0.0);
    CHECK_FALSE(is_window_stationary(trajectory(n, 0.0, 0.0), ts, one_spike, acc_threshold, gyro_threshold));

    CHECK_FALSE(is_window_stationary(trajectory(n, 0.1, 0.0), ts, acc, acc_threshold, gyro_threshold));
}
