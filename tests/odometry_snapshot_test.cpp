#include <doctest/doctest.h>
#include "consistency.h"
#include "memory_point_store.h"
#include "odometry_pipeline.h"
#include "odometry_snapshot_file.h"
#include "synthetic_recording.h"
#include <filesystem>
#include <fstream>
#include <random>

// The odometry snapshot (save_odometry_snapshot, dMapper ... --from-snapshot): written and read back bit for bit, refused
// for other odometry settings or another recording, and the post-processing after a restore gives the same windows as
// after the odometry itself (on the synthetic loop recording, see synthetic_recording.h).

using namespace synthetic;

namespace
{
    struct SilentProgress : ProgressReporter
    {
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };

    // a temporary file, removed when the test ends
    struct TempFile
    {
        std::filesystem::path path;
        explicit TempFile(const char *name) : path(std::filesystem::temp_directory_path() / name) {}
        ~TempFile()
        {
            std::error_code ec;
            std::filesystem::remove(path, ec);
        }
    };

    // the loop recording's windows with every snapshot field filled (the bias, velocity and accelerometer values too)
    OdometrySnapshot sample_snapshot()
    {
        OdometrySnapshot s;
        s.windows = loop_recording(1).windows;
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        for (Window &w : s.windows)
        {
            w.gyro_bias.mean = Eigen::Vector3d(u(rng), u(rng), u(rng)) * 1e-3;
            w.gyro_bias.covariance = Eigen::Matrix3d::Identity() * (1e-6 + u(rng) * 1e-7);
            w.velocity.start = Eigen::Vector3d(u(rng), u(rng), u(rng));
            w.velocity.end = Eigen::Vector3d(u(rng), u(rng), u(rng));
            w.velocity.end_covariance(0, 1) = w.velocity.end_covariance(1, 0) = u(rng) * 1e-3;
            w.imu_acc.assign(w.poses.size(), Eigen::Vector3d(u(rng), u(rng), 1.0 + u(rng) * 0.01));
            w.imu_angles.assign(w.poses.size(), Eigen::Vector3d(u(rng), u(rng), u(rng)));
        }
        s.m_g = s.windows.front().poses.front();
        s.total_length_of_calculated_trajectory = 44.4 + u(rng);
        s.consecutive_distance = 12.3 + u(rng);
        return s;
    }

    bool same(const Eigen::Affine3d &a, const Eigen::Affine3d &b) { return a.matrix() == b.matrix(); }

    bool same(const Window &a, const Window &b)
    {
        if (a.poses.size() != b.poses.size() || a.motion_model_poses.size() != b.motion_model_poses.size() || a.timestamps != b.timestamps ||
            a.imu_angles != b.imu_angles || a.imu_acc != b.imu_acc)
            return false;
        for (size_t k = 0; k < a.poses.size(); k++)
            if (!same(a.poses[k], b.poses[k]))
                return false;
        for (size_t k = 0; k < a.motion_model_poses.size(); k++)
            if (!same(a.motion_model_poses[k], b.motion_model_poses[k]))
                return false;
        return a.gyro_bias.mean == b.gyro_bias.mean && a.gyro_bias.covariance == b.gyro_bias.covariance &&
               a.velocity.start == b.velocity.start && a.velocity.end == b.velocity.end && a.velocity.end_covariance == b.velocity.end_covariance;
    }

    bool same(const std::vector<Window> &a, const std::vector<Window> &b)
    {
        if (a.size() != b.size())
            return false;
        for (size_t i = 0; i < a.size(); i++)
            if (!same(a[i], b[i]))
                return false;
        return true;
    }
}

TEST_CASE("odometry snapshot: written and read back bit for bit")
{
    const OdometrySnapshot written = sample_snapshot();
    const OdometryConfig config;
    TempFile file("dmapper_snapshot_roundtrip.bin");
    std::string why;
    REQUIRE(write_odometry_snapshot(file.path, written, config, why));
    OdometrySnapshot read;
    REQUIRE(read_odometry_snapshot(file.path, config, read, why));
    CHECK(same(read.windows, written.windows));
    CHECK(same(read.m_g, written.m_g));
    CHECK(read.total_length_of_calculated_trajectory == written.total_length_of_calculated_trajectory);
    CHECK(read.consecutive_distance == written.consecutive_distance);
}

TEST_CASE("odometry snapshot: refused for other odometry settings, accepted for other post-processing settings")
{
    const OdometryConfig config;
    TempFile file("dmapper_snapshot_settings.bin");
    std::string why;
    REQUIRE(write_odometry_snapshot(file.path, sample_snapshot(), config, why));
    OdometrySnapshot read;

    OdometryConfig odometry = config;
    odometry.nr_iter += 1;
    CHECK_FALSE(read_odometry_snapshot(file.path, odometry, read, why));
    CHECK(why.find("odometry settings") != std::string::npos);
    OdometryConfig grid = config;
    grid.in_out_params_indoor.resolution_X += 0.01;
    CHECK_FALSE(read_odometry_snapshot(file.path, grid, read, why));

    OdometryConfig post = config;
    post.use_loop_closure = !post.use_loop_closure;
    post.loop_closure_cross_floor_max_vertical_m = 6.0;
    post.use_consistency = !post.use_consistency;
    post.log_window_diagnostics = !post.log_window_diagnostics;
    post.save_odometry_snapshot = !post.save_odometry_snapshot;
    CHECK(read_odometry_snapshot(file.path, post, read, why));
}

TEST_CASE("odometry snapshot: a file that isn't a snapshot is refused")
{
    TempFile file("dmapper_snapshot_damaged.bin");
    {
        std::ofstream out(file.path, std::ios::binary);
        out << "not a snapshot";
    }
    OdometrySnapshot read;
    std::string why;
    CHECK_FALSE(read_odometry_snapshot(file.path, OdometryConfig{}, read, why));
    CHECK_FALSE(read_odometry_snapshot(std::filesystem::temp_directory_path() / "dmapper_no_such_snapshot.bin", OdometryConfig{}, read, why));
}

TEST_CASE("odometry snapshot: restore refuses another recording and takes the windows and run state otherwise")
{
    const OdometryConfig config;
    NullLogger logger;
    SilentProgress progress;
    const OdometrySnapshot snapshot = sample_snapshot();
    std::string why;

    OdometryPipeline fewer(config, logger, progress);
    fewer.windows.assign(snapshot.windows.begin(), snapshot.windows.end() - 1);
    CHECK_FALSE(fewer.restore_odometry(snapshot, why));
    CHECK(why.find("windows") != std::string::npos);

    OdometryPipeline other_time(config, logger, progress);
    other_time.windows = snapshot.windows;
    other_time.windows[3].timestamps[2].first += 1e-6;
    CHECK_FALSE(other_time.restore_odometry(snapshot, why));
    CHECK(why.find("window 4") != std::string::npos);

    // the recording's windows as load() cuts them (the truth here), then the odometry's result from the snapshot
    OdometryPipeline pipeline(config, logger, progress);
    pipeline.windows = loop_recording(1).windows;
    REQUIRE(pipeline.restore_odometry(snapshot, why));
    CHECK(same(pipeline.windows, snapshot.windows));
    CHECK(same(pipeline.state.m_g, snapshot.m_g));
    CHECK(pipeline.state.total_length_of_calculated_trajectory == snapshot.total_length_of_calculated_trajectory);
    CHECK(pipeline.state.consecutive_distance == snapshot.consecutive_distance);
    CHECK(same(pipeline.snapshot().windows, snapshot.windows));
}

TEST_CASE("odometry snapshot: the post-processing after a restore gives the same windows as after the odometry")
{
    // the odometry's result: the loop recording's windows, every window after the first moved by a small offset
    Recording r = loop_recording(1);
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> shift(-0.02, 0.02);
    for (size_t w = 1; w < r.windows.size(); w++)
        for (auto &pose : r.windows[w].poses)
            pose.translation() += Eigen::Vector3d(shift(rng), shift(rng), shift(rng));

    OdometryConfig config;
    config.use_consistency = true;
    config.num_constistency_iter = 2;
    config.in_out_params_indoor.resolution_X = config.in_out_params_indoor.resolution_Y = config.in_out_params_indoor.resolution_Z = 0.15;
    MemoryPointStore store;
    for (size_t w = 0; w < r.points.size(); w++)
        store.save(PointSet::WindowIntermediate, w, r.points[w]);
    NullLogger logger;

    // directly after the odometry
    std::vector<Window> direct = r.windows;
    run_consistency(direct, config, store, logger);

    // through the snapshot file
    OdometrySnapshot written;
    written.windows = r.windows;
    TempFile file("dmapper_snapshot_postprocess.bin");
    std::string why;
    REQUIRE(write_odometry_snapshot(file.path, written, config, why));
    OdometrySnapshot read;
    REQUIRE(read_odometry_snapshot(file.path, config, read, why));
    std::vector<Window> restored = read.windows;
    run_consistency(restored, config, store, logger);

    CHECK(same(restored, direct));
}
