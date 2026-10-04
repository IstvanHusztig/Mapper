#include <doctest/doctest.h>

#include "lidar_odometry.h"
#include "serial_executor.h"
#include "synthetic_recording.h"

#include <cmath>
#include <vector>

// calculate_trajectory (the IMU orientation step) through the OrientationEstimator port, with a fake estimator that
// records every call: which convention and gain it starts with, the gyroscope it gets (bias removed, in degrees), when
// the gain drops, the time steps, and that its orientation becomes the trajectory.

namespace
{
    const double rad_to_deg = 180.0 / 3.14159265358979323846;

    struct Update
    {
        Eigen::Vector3f gyro_deg_s, acc_g;
        float dt_s;
        float gain; // the gain in force for this update
    };

    struct RecordingEstimator : OrientationEstimator
    {
        int resets = 0;
        AxesConvention convention = AxesConvention::Nwu;
        float initial_gain = -1.0f;
        float gain = -1.0f;
        std::vector<Update> updates;
        Eigen::Quaternionf fixed{Eigen::AngleAxisf(0.5f, Eigen::Vector3f::UnitZ())};

        void reset(AxesConvention c, float g) override
        {
            resets++;
            convention = c;
            initial_gain = gain = g;
        }
        void set_gain(float g) override { gain = g; }
        void update(const Eigen::Vector3f &gyro, const Eigen::Vector3f &acc, float dt) override { updates.push_back({gyro, acc, dt, gain}); }
        Eigen::Quaternionf orientation() const override { return fixed; }
    };

    // n samples 5 ms apart, at rest (1 g along z), with a constant gyroscope [rad/s]
    Imu still_imu(int n, const Eigen::Vector3f &gyro)
    {
        Imu imu;
        for (int i = 0; i < n; i++)
            imu.push_back({{1.0 + 0.005 * i, 1000.0 + 0.005 * i}, gyro, Eigen::Vector3f(0.0f, 0.0f, 1.0f)});
        return imu;
    }

    struct Flags
    {
        bool nwu = true, enu = false, ned = false;
        bool remove_bias = false;
    };

    RecordingEstimator run(Imu &imu, ImuTrajectory &trajectory, const Flags &f = {}, double gain = 0.5)
    {
        RecordingEstimator estimator;
        synthetic::NullLogger logger;
        calculate_trajectory(trajectory, imu, f.nwu, f.enu, f.ned, gain, false, f.remove_bias, estimator, logger);
        return estimator;
    }
}

TEST_CASE("orientation: the filter starts with the first convention set and the configured gain")
{
    const struct
    {
        Flags flags;
        AxesConvention expected;
    } cases[] = {
        {{true, true, true}, AxesConvention::Nwu},
        {{false, true, true}, AxesConvention::Enu},
        {{false, false, true}, AxesConvention::Ned},
        {{false, false, false}, AxesConvention::Nwu},
    };
    for (const auto &c : cases)
    {
        Imu imu = still_imu(3, Eigen::Vector3f::Zero());
        ImuTrajectory trajectory;
        const RecordingEstimator e = run(imu, trajectory, c.flags, 0.25);
        CHECK(e.resets == 1);
        CHECK(e.convention == c.expected);
        CHECK(e.initial_gain == 0.25f);
    }
}

TEST_CASE("orientation: the gyroscope goes in degrees, with the bias of the first stationary samples removed")
{
    const Eigen::Vector3f bias(0.01f, -0.02f, 0.03f); // [rad/s]

    SUBCASE("without bias removal")
    {
        Imu imu = still_imu(1200, bias);
        ImuTrajectory trajectory;
        const RecordingEstimator e = run(imu, trajectory);
        REQUIRE(e.updates.size() == 1200);
        for (int k = 0; k < 3; k++)
            CHECK(e.updates.back().gyro_deg_s[k] == doctest::Approx(bias[k] * rad_to_deg).epsilon(1e-6));
    }
    SUBCASE("at rest: the median of the first 1000 samples is removed")
    {
        Imu imu = still_imu(1200, bias);
        ImuTrajectory trajectory;
        Flags f;
        f.remove_bias = true;
        const RecordingEstimator e = run(imu, trajectory, f);
        for (const Update &u : e.updates)
            CHECK(u.gyro_deg_s.norm() < 1e-6f);
    }
    SUBCASE("moving: no bias is removed when the first 1000 samples vary too much")
    {
        Imu imu = still_imu(1200, bias);
        for (int i = 0; i < 1000; i += 2)
            imu[i].gyro.x() = 1.0f; // 57 deg/s every other sample: variance far above 5
        ImuTrajectory trajectory;
        Flags f;
        f.remove_bias = true;
        const RecordingEstimator e = run(imu, trajectory, f);
        CHECK(e.updates.back().gyro_deg_s.y() == doctest::Approx(bias.y() * rad_to_deg).epsilon(1e-6));
    }
}

TEST_CASE("orientation: the gain drops to 0 while the acceleration is away from 1 g")
{
    Imu imu = still_imu(4, Eigen::Vector3f::Zero());
    imu[1].acc = Eigen::Vector3f(0.0f, 0.0f, 1.5f);
    imu[2].acc = Eigen::Vector3f(0.0f, 0.0f, 0.8f);
    ImuTrajectory trajectory;
    const RecordingEstimator e = run(imu, trajectory, {}, 0.5);
    REQUIRE(e.updates.size() == 4);
    CHECK(e.updates[0].gain == 0.5f);
    CHECK(e.updates[1].gain == 0.0f);
    CHECK(e.updates[2].gain == 0.0f);
    CHECK(e.updates[3].gain == 0.5f);
    CHECK(e.updates[1].acc_g.z() == 1.5f);
}

TEST_CASE("orientation: time steps, and one trajectory node per timestamp with the filter's orientation")
{
    Imu imu = still_imu(3, Eigen::Vector3f::Zero());
    imu.push_back(imu.back()); // a repeated timestamp
    ImuTrajectory trajectory;
    const RecordingEstimator e = run(imu, trajectory);
    REQUIRE(e.updates.size() == 4);
    CHECK(e.updates[0].dt_s == static_cast<float>(1.0 / 200.0));
    CHECK(e.updates[1].dt_s == doctest::Approx(0.005).epsilon(1e-6));
    CHECK(e.updates[3].dt_s == static_cast<float>(0.002));

    CHECK(trajectory.size() == 3); // keyed by timestamp: the repeated one replaces its node
    const Eigen::Matrix3d expected = e.fixed.cast<double>().toRotationMatrix();
    for (const auto &[time, node] : trajectory)
    {
        CHECK((node.first.topLeftCorner<3, 3>() - expected).norm() < 1e-6);
        CHECK(node.first.topRightCorner<3, 1>().norm() == 0.0);
        CHECK(node.second == doctest::Approx(time + 999.0));
    }
}

TEST_CASE("serial executor: every index once, in order")
{
    SerialExecutor executor;
    std::vector<std::size_t> seen;
    executor.for_each_index(5, [&](std::size_t i) { seen.push_back(i); });
    CHECK(seen == std::vector<std::size_t>{0, 1, 2, 3, 4});
    executor.for_each_index(0, [&](std::size_t) { seen.push_back(99); });
    CHECK(seen.size() == 5);
}
