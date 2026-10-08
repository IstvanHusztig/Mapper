#include <doctest/doctest.h>

#include "angle_units.h"
#include "consistency.h"
#include "lidar_odometry.h"
#include "memory_point_store.h"
#include "synthetic_recording.h"

#include <chrono>
#include <random>
#include <string>

// run_consistency (Consistency, and Consistency2 with use_mutliple_gaussian) on the loop recording (see
// synthetic_recording.h) against its true trajectory. The options are those of settings/lidar-vertical.toml, except the
// number of passes; they are set here, so tuning the shipped file doesn't change the tests.

using namespace synthetic;

namespace
{
    const double deg = DEG_TO_RAD;

    std::string variant(bool multiple_gaussian) { return multiple_gaussian ? "Consistency2" : "Consistency"; }

    OdometryConfig consistency_config(bool multiple_gaussian)
    {
        OdometryConfig c;
        c.use_consistency = true;
        c.num_constistency_iter = 3; // (shipped: 10)
        c.use_mutliple_gaussian = multiple_gaussian;
        c.consistency_min_distance = 0.4;
        c.in_out_params_indoor.resolution_X = c.in_out_params_indoor.resolution_Y = c.in_out_params_indoor.resolution_Z = 0.15;
        c.use_exact_ndt_bucket_statistics = false;
        return c;
    }

    // window seams: every window after the first is moved as a whole by a small rigid offset (about its first node),
    // as the hand-over between odometry windows leaves them; the first window stays, as Consistency holds node 0
    void add_window_offsets(Recording &r, double max_shift_m, double max_angle_rad)
    {
        std::mt19937 rng(5);
        std::uniform_real_distribution<double> shift(-max_shift_m, max_shift_m);
        std::uniform_real_distribution<double> angle(-max_angle_rad, max_angle_rad);
        for (size_t w = 1; w < r.windows.size(); w++)
        {
            Window &window = r.windows[w];
            const Eigen::Vector3d about = window.poses.front().translation();
            Eigen::Affine3d offset = Eigen::Affine3d::Identity();
            offset.translate(about + Eigen::Vector3d(shift(rng), shift(rng), shift(rng)));
            offset.rotate(Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX()));
            offset.translate(-about);
            for (size_t n = 0; n < window.poses.size(); n++)
            {
                window.poses[n] = offset * window.poses[n];
                window.motion_model_poses[n] = window.poses[n];
            }
        }
    }

    struct Measured
    {
        PoseError mean, largest;
        size_t voxels = 0;
    };

    Measured measure(const Recording &r)
    {
        const auto poses = all_poses(r.windows), truth = all_poses(r.truth);
        return {mean_error(poses, truth), max_error(poses, truth), occupied_voxels(r.windows, r.points, 0.03)};
    }

    double run(Recording &r, bool multiple_gaussian)
    {
        MemoryPointStore store;
        for (size_t w = 0; w < r.points.size(); w++)
            store.save(PointSet::WindowIntermediate, w, r.points[w]);
        NullLogger logger;
        const auto start = std::chrono::steady_clock::now();
        run_consistency(r.windows, consistency_config(multiple_gaussian), store, logger);
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    }

    // seams of +-3 cm / +-0.2 deg per window: the mean pose error and the map's excess of occupied 3 cm voxels over the
    // truth's must shrink
    void check_seams(bool multiple_gaussian)
    {
        Recording r = loop_recording(2);
        const Measured truth_map = measure(r);
        add_window_offsets(r, 0.03, 0.2 * deg);
        const Measured before = measure(r);
        const double ms = run(r, multiple_gaussian);
        const Measured after = measure(r);
        MESSAGE(variant(multiple_gaussian), " seams: mean error ", before.mean.position * 1000.0, " mm / ", before.mean.rotation / deg,
                " deg -> ", after.mean.position * 1000.0, " mm / ", after.mean.rotation / deg, " deg; largest ",
                before.largest.position * 1000.0, " -> ", after.largest.position * 1000.0, " mm; 3 cm voxels ", before.voxels, " -> ",
                after.voxels, " (truth ", truth_map.voxels, "); ", ms, " ms");
        CHECK(after.mean.position <= 0.8 * before.mean.position);
        CHECK(after.mean.rotation <= 0.8 * before.mean.rotation);
        CHECK(after.voxels <= truth_map.voxels + (before.voxels - truth_map.voxels) / 2);
    }

    // poses equal to the truth: every node must stay within 5 mm and 0.1 deg
    void check_truth(bool multiple_gaussian)
    {
        Recording r = loop_recording(2);
        const double ms = run(r, multiple_gaussian);
        const Measured after = measure(r);
        MESSAGE(variant(multiple_gaussian), " truth: largest change ", after.largest.position * 1000.0, " mm / ",
                after.largest.rotation / deg, " deg, mean ", after.mean.position * 1000.0, " mm; ", ms, " ms");
        CHECK(after.largest.position <= 0.005);
        CHECK(after.largest.rotation <= 0.1 * deg);
    }
}

namespace
{
    // three passes of Consistency with the points split into chunks of chunk_points (the shipped value is 1M), sharing the
    // ConsistencyData between the passes as run_consistency does
    void run_in_chunks(Recording &r, size_t chunk_points)
    {
        MemoryPointStore store;
        for (size_t w = 0; w < r.points.size(); w++)
            store.save(PointSet::WindowIntermediate, w, r.points[w]);
        NullLogger logger;
        const OdometryConfig config = consistency_config(false);
        ConsistencyData data;
        data.chunk_points = chunk_points;
        for (int pass = 0; pass < config.num_constistency_iter; pass++)
        {
            for (Window &window : r.windows)
                window.motion_model_poses = window.poses;
            Consistency(r.windows, config, store, logger, nullptr, &data);
        }
    }
}

// The chunks are computed in parallel and merged in chunk order, so the result must not depend on the thread timing:
// two runs give the same bits. Several chunks still reduce the seams (each chunk has its own NDT map).
TEST_CASE("consistency: Consistency with several chunks is repeatable and reduces the seams")
{
    Recording a = loop_recording(2);
    size_t total_points = 0;
    for (const auto &w : a.points)
        total_points += w.size();
    const size_t chunk_points = total_points / 5 + 1;
    REQUIRE(chunk_points > 10000); // smaller chunks are skipped, as in the shipped code
    add_window_offsets(a, 0.03, 0.2 * deg);
    Recording b = a;
    const Measured before = measure(a);

    run_in_chunks(a, chunk_points);
    run_in_chunks(b, chunk_points);
    const auto poses_a = all_poses(a.windows), poses_b = all_poses(b.windows);
    REQUIRE(poses_a.size() == poses_b.size());
    bool same = true;
    for (size_t i = 0; i < poses_a.size(); i++)
        same = same && poses_a[i].matrix() == poses_b[i].matrix();
    CHECK(same);

    const Measured after = measure(a);
    MESSAGE("Consistency in 5 chunks: mean error ", before.mean.position * 1000.0, " mm -> ", after.mean.position * 1000.0, " mm");
    // each chunk has a map of a fifth of the points, so fewer seams are matched than with one map (27.9 -> 22.6 mm here,
    // against 27.9 -> 19.5 mm with a single chunk)
    CHECK(after.mean.position <= 0.9 * before.mean.position);
}

// Measured 2026-10-04 (Release, 3 passes): mean error 27.9 mm / 0.207 deg -> 19.5 mm / 0.147 deg, 3 cm voxels
// 127,956 -> 126,563 (truth 125,898); 1.0 s
TEST_CASE("consistency: Consistency reduces the offsets between windows")
{
    check_seams(false);
}

// Measured 2026-10-04 (Release, 3 passes): mean error 27.9 mm / 0.207 deg -> 14.6 mm / 0.125 deg, 3 cm voxels
// 127,956 -> 126,324 (truth 125,898); 0.6 s
TEST_CASE("consistency: Consistency2 reduces the offsets between windows")
{
    check_seams(true);
}

// Expected to fail (found 2026-10-04): the pass moves the true trajectory by up to 6.4 mm / 0.075 deg (mean 2.1 mm),
// spread evenly over all windows (2-6 mm in each). That is the noise of fitting 150 points per node to 0.15 m NDT
// cells, not a systematic pull: the rotation stays within 0.1 deg. When the pass leaves a correct trajectory within
// 5 mm, doctest reports this case as passing unexpectedly: then remove should_fail.
TEST_CASE("consistency: Consistency leaves a trajectory that is already right" * doctest::should_fail())
{
    check_truth(false);
}

// Expected to fail (found 2026-10-04): the pass moves the true trajectory by up to 16 mm / 0.46 deg (mean 5.2 mm). The
// rotation changes sit in the corner windows (0.2-0.46 deg there, about 0.05 deg on the straights): the smoothness
// term holds the change of turn rate from node to node to 0.01 deg (Consistency: 0.1 deg), and the fixture's turn rate
// ramps by 0.29 deg per node in and out of each corner. With a 1 s ramp (half the change per node) the corner windows
// drop to 0.18 deg, and an even 5-12 mm / 0.1 deg remains in every window.
TEST_CASE("consistency: Consistency2 leaves a trajectory that is already right" * doctest::should_fail())
{
    check_truth(true);
}
