#include <doctest/doctest.h>

#include "angle_units.h"
#include "startup_gyro_bias.h"

#include <cmath>
#include <random>

// startup_gyro_bias on synthetic first seconds at 890 Hz: the sensor's own noise is ARW x sqrt(rate) per axis
// (0.84 / 1.25 / 1.70 deg/s with the shipped ARW).

namespace
{
    constexpr double rate = 890.0;

    // 1.2 s of samples: bias + noise_factor x the sensor noise + extra(t) [deg/s], gravity turned by tilt(t) [deg] about x
    template <class Extra, class Tilt>
    Imu samples(const Eigen::Vector3d &bias_deg_s, double noise_factor, Extra extra, Tilt tilt)
    {
        const StartupBiasOptions o;
        std::mt19937 rng(11);
        std::normal_distribution<double> n01(0.0, 1.0);
        Imu imu;
        for (int i = 0; i < static_cast<int>(1.2 * rate); i++)
        {
            const double t = i / rate;
            Eigen::Vector3d g = bias_deg_s + extra(t);
            for (int a = 0; a < 3; a++)
                g[a] += noise_factor * o.arw_deg_sqrt_s[a] * std::sqrt(rate) * n01(rng);
            const double tl = tilt(t) * DEG_TO_RAD;
            imu.push_back({{t, 1000.0 + t}, (g * DEG_TO_RAD).cast<float>(), Eigen::Vector3f(0.0f, -std::sin(tl), std::cos(tl))});
        }
        return imu;
    }

    const auto none = [](double) { return Eigen::Vector3d::Zero().eval(); };
    const auto level = [](double) { return 0.0; };
    const Eigen::Vector3d bias(0.05, -0.1, -0.8);
}

TEST_CASE("start-up gyro bias: a still sensor gives its bias")
{
    const StartupBias b = startup_gyro_bias(samples(bias, 1.0, none, level), StartupBiasOptions{});
    MESSAGE(b.reason);
    REQUIRE(b.accepted);
    for (int a = 0; a < 3; a++)
        CHECK(std::abs(b.bias_deg_s[a] - bias[a]) < 0.2);
}

TEST_CASE("start-up gyro bias: a hand-held stand-still is accepted")
{
    // twice the sensor noise and a 3 Hz tremor of 1.5 deg/s on every axis (no net turn)
    const auto tremor = [](double t) { return Eigen::Vector3d::Constant(1.5 * std::sin(2.0 * 3.14159265358979 * 3.0 * t)).eval(); };
    const StartupBias b = startup_gyro_bias(samples(bias, 2.0, tremor, level), StartupBiasOptions{});
    MESSAGE(b.reason);
    REQUIRE(b.accepted);
    for (int a = 0; a < 3; a++)
        CHECK(std::abs(b.bias_deg_s[a] - bias[a]) < 0.5);
}

TEST_CASE("start-up gyro bias: motion is rejected, with the reason")
{
    SUBCASE("a turn: 30 deg/s about z from 0.5 s on")
    {
        const auto turn = [](double t) { return Eigen::Vector3d(0.0, 0.0, t > 0.5 ? 30.0 : 0.0); };
        const StartupBias b = startup_gyro_bias(samples(bias, 1.0, turn, level), StartupBiasOptions{});
        CHECK_FALSE(b.accepted);
        CHECK(b.reason.find("turn") != std::string::npos);
        CHECK(b.bias_deg_s.norm() == 0.0);
    }
    SUBCASE("a tilt: gravity turns by 5 deg over the second")
    {
        const auto tilt = [](double t) { return 5.0 * t; };
        const StartupBias b = startup_gyro_bias(samples(bias, 1.0, none, tilt), StartupBiasOptions{});
        CHECK_FALSE(b.accepted);
        CHECK(b.reason.find("tilt") != std::string::npos);
    }
    SUBCASE("shaking: five times the sensor noise")
    {
        const StartupBias b = startup_gyro_bias(samples(bias, 5.0, none, level), StartupBiasOptions{});
        CHECK_FALSE(b.accepted);
        CHECK(b.reason.find("shaking") != std::string::npos);
    }
}
