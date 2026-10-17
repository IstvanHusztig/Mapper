#include <doctest/doctest.h>

#include "lidar_odometry.h"
#include "memory_point_store.h"
#include "synthetic_recording.h"
#include "window_steps.h"

#include <cmath>
#include <vector>

// compute_step_1 (cutting the recording into windows) and the hand-off between windows (place_windows_after) on a
// synthetic recording: IMU samples every 1 ms turning at a constant yaw rate, and a point every 10 us. The windows are
// disjoint, so between one window's last sample and the next window's first lies one sample interval (finding C15).

namespace
{
    using namespace synthetic;

    struct SilentProgress : ProgressReporter
    {
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };

    constexpr double imu_dt = 0.001;
    constexpr double point_dt = 1e-5;
    constexpr double yaw_rate = 1.0; // [rad/s]
    constexpr int n_samples = 130;

    Eigen::Matrix4d imu_pose(double t)
    {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        m.topLeftCorner<3, 3>() = Eigen::AngleAxisd(yaw_rate * t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return m;
    }

    double yaw_of(const Eigen::Affine3d &m) { return std::atan2(m.linear()(1, 0), m.linear()(0, 0)); }

    struct Cut
    {
        std::vector<Window> windows;
        MemoryPointStore store;
        std::vector<double> point_times; // every point of the recording
    };

    // 20 samples per window (2000 points each, so none is dropped), the first 100 points for the initial map, no decimation
    void cut(Cut &c)
    {
        OdometryConfig params;
        params.threshold_initial_points = 100;
        params.threshold_nr_poses = 20;
        params.decimation = 0.0;
        ImuTrajectory trajectory;
        for (int k = 0; k < n_samples; k++)
            trajectory[k * imu_dt] = {imu_pose(k * imu_dt), 1000.0 + k * imu_dt};
        // points half-way between the 10 us grid lines, so that none falls exactly on a sample time
        std::vector<std::vector<Point3Di>> points_per_file(1);
        for (int j = 0; (j + 0.5) * point_dt < n_samples * imu_dt; j++)
        {
            Point3Di p{};
            p.point = Eigen::Vector3d(2.0, 0.0, 0.0);
            p.timestamp = (j + 0.5) * point_dt;
            p.index_point = j;
            points_per_file[0].push_back(p);
            c.point_times.push_back(p.timestamp);
        }
        OdometryState state;
        NullLogger logger;
        SilentProgress progress;
        REQUIRE(compute_step_1(points_per_file, params, state, trajectory, c.windows, c.store, logger, progress));
        REQUIRE(c.windows.size() >= 5);
    }
}

TEST_CASE("window hand-off: chained windows keep the rotation between one window's last sample and the next one's first")
{
    // place_windows_after starts each window after its predecessor's last node. Starting it at that node's pose lost the
    // IMU rotation of one sample interval per window (1/20 of the rotation here, 1/360 with lidar-vertical.toml).
    Cut c;
    cut(c);
    place_windows_after(c.windows, 1);
    double worst = 0.0;
    for (const Window &w : c.windows)
        for (size_t k = 0; k < w.poses.size(); k++)
            worst = std::max(worst, std::abs(yaw_of(w.poses[k]) - yaw_rate * w.timestamps[k].first));
    MESSAGE("largest yaw error of a node after chaining ", c.windows.size(), " windows: ", worst, " rad");
    CHECK(worst < 1e-6);
}

TEST_CASE("window hand-off: the predicted guess of the next window keeps the motion between the two samples")
{
    // optimize_windows predicts each window's guess with predict_window_poses (use_motion_from_previous_step), which
    // rebuilds it from the previous window's last node: that is where the hand-off was lost again after D5 fixed
    // place_window_after. Here at a constant yaw rate and a constant velocity, without IMU translation.
    Cut c;
    cut(c);
    const Eigen::Vector3d velocity(0.8, -0.3, 0.1); // [m/s]
    PosePredictionOptions options;
    options.constant_velocity = true;
    double worst_yaw = 0.0, worst_position = 0.0;
    for (size_t i = 1; i < c.windows.size(); i++)
    {
        predict_window_poses(c.windows[i], c.windows[i - 1], nullptr, velocity, options);
        const Window &w = c.windows[i];
        const Window &prev = c.windows[i - 1];
        for (size_t k = 0; k < w.poses.size(); k++)
            worst_yaw = std::max(worst_yaw, std::abs(yaw_of(w.poses[k]) - yaw_rate * w.timestamps[k].first));
        const Eigen::Vector3d expected = prev.poses.back().translation() + velocity * (w.timestamps[0].first - prev.timestamps.back().first);
        worst_position = std::max(worst_position, (w.poses[0].translation() - expected).norm());
    }
    MESSAGE("predicted windows: largest yaw error ", worst_yaw, " rad, largest first-node position error ", worst_position, " m");
    CHECK(worst_yaw < 1e-6);
    CHECK(worst_position < 1e-9);
}

TEST_CASE("window hand-off: the points between two windows belong to the earlier window's last node")
{
    Cut c;
    cut(c);
    // up to the IMU sample after the last window (the samples left over after the last full window): the last window
    // keeps the points up to it too
    const double first = c.windows.front().timestamps.front().first;
    const double end = c.windows.back().timestamps.back().first + imu_dt;
    size_t expected = 0;
    for (double t : c.point_times)
        if (t > first && t < end)
            expected++;

    size_t found = 0, in_gaps = 0, gaps_at_last_node = 0;
    for (size_t i = 0; i < c.windows.size(); i++)
    {
        std::vector<Point3Di> points;
        REQUIRE(c.store.load(PointSet::WindowOriginal, i, points));
        found += points.size();
        const auto &ts = c.windows[i].timestamps;
        for (const Point3Di &p : points)
        {
            CHECK(p.timestamp > ts.front().first);
            if (p.timestamp > ts.back().first)
            {
                in_gaps++;
                if (p.index_pose == static_cast<int>(ts.size()) - 1)
                    gaps_at_last_node++;
            }
        }
    }
    MESSAGE("points from the first window's first sample to the sample after the last window: ", expected, ", in the windows: ",
            found, ", of them after a window's last sample: ", in_gaps);
    CHECK(found == expected);
    CHECK(in_gaps == c.windows.size() * 100); // one sample interval of points after each window's last node
    CHECK(gaps_at_last_node == in_gaps);
}
