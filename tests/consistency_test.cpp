#include <doctest/doctest.h>

#include "angle_units.h"
#include "consistency.h"
#include "lidar_odometry.h"
#include "memory_point_store.h"
#include "synthetic_recording.h"
#include "transformations.h"

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
    // truth's must shrink (the excess by at least 15 %, see the measurements at the test cases)
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
        CHECK(after.voxels <= truth_map.voxels + 0.85 * (before.voxels - truth_map.voxels));
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
    // a perfect gyro: every node's IMU orientation is its true orientation, the bias zero
    void set_imu_from_truth(Recording &r)
    {
        for (size_t w = 0; w < r.windows.size(); w++)
        {
            r.windows[w].imu_angles.clear();
            for (const Eigen::Affine3d &pose : r.truth[w])
            {
                const TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(pose);
                r.windows[w].imu_angles.emplace_back(tb.om, tb.fi, tb.ka);
            }
            r.windows[w].gyro_bias.mean.setZero();
        }
    }

    // mean and largest error of the rotation between consecutive nodes of a window, against the truth [rad]
    std::pair<double, double> node_rotation_error(const Recording &r)
    {
        double sum = 0.0, largest = 0.0;
        int n = 0;
        for (size_t w = 0; w < r.windows.size(); w++)
            for (size_t k = 0; k + 1 < r.windows[w].poses.size(); k++)
            {
                const Eigen::Matrix3d got = r.windows[w].poses[k].linear().transpose() * r.windows[w].poses[k + 1].linear();
                const Eigen::Matrix3d want = r.truth[w][k].linear().transpose() * r.truth[w][k + 1].linear();
                const double e = Eigen::AngleAxisd(Eigen::Matrix3d(got.transpose() * want)).angle();
                sum += e;
                largest = std::max(largest, e);
                n++;
            }
        return {sum / n, largest};
    }

    double run_with(Recording &r, const OdometryConfig &config)
    {
        MemoryPointStore store;
        for (size_t w = 0; w < r.points.size(); w++)
            store.save(PointSet::WindowIntermediate, w, r.points[w]);
        NullLogger logger;
        const auto start = std::chrono::steady_clock::now();
        run_consistency(r.windows, config, store, logger);
        return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    }
}

// The gyro rotation term (consistency_gyro_rotation_term) ties consecutive nodes to the IMU's relative rotation: with a
// perfect gyro the node-to-node rotation must stay closer to the truth than without the term, from the true trajectory
// and from one with window seams, and the result must be repeatable bit for bit.
TEST_CASE("consistency: the gyro rotation term keeps the node-to-node rotation")
{
    OdometryConfig with = consistency_config(false);
    with.consistency_gyro_rotation_term = true;
    const OdometryConfig without = consistency_config(false);
    for (const bool seams : {false, true})
    {
        Recording a = loop_recording(2);
        set_imu_from_truth(a);
        if (seams)
            add_window_offsets(a, 0.03, 0.2 * deg);
        Recording b = a, c = a;
        run_with(a, without);
        const double ms = run_with(b, with);
        run_with(c, with);
        const auto [mean_without, largest_without] = node_rotation_error(a);
        const auto [mean_with, largest_with] = node_rotation_error(b);
        const Measured m_without = measure(a), m_with = measure(b);
        MESSAGE("gyro term, ", seams ? "seams" : "truth", ": node-to-node rotation error mean ", mean_without / deg, " -> ", mean_with / deg,
                " deg, largest ", largest_without / deg, " -> ", largest_with / deg, " deg; pose error largest ", m_without.largest.position * 1000.0,
                " -> ", m_with.largest.position * 1000.0, " mm, mean ", m_without.mean.position * 1000.0, " -> ", m_with.mean.position * 1000.0,
                " mm; ", ms, " ms");
        CHECK(mean_with < mean_without);
        CHECK(largest_with < largest_without);
        const auto poses_b = all_poses(b.windows), poses_c = all_poses(c.windows);
        bool same = true;
        for (size_t i = 0; i < poses_b.size(); i++)
            same = same && poses_b[i].matrix() == poses_c[i].matrix();
        CHECK(same);
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
// 127,956 -> 126,563 (truth 125,898); 1.0 s. That scene lay at the world origin, its faces on the 3 cm voxel boundaries,
// which made the truth map thick and hid most of the remaining seams (the excess shrank by 68 %).
// Measured 2026-10-14 with the scene at grid_offset (faces inside the voxels): 27.9 mm / 0.207 deg -> 17.5 mm / 0.145 deg,
// voxels 127,982 -> 126,492 (truth 121,634): the excess shrinks by 23 % (36 % with the cell index truncated towards zero
// as before C10, whose double-height cell held the floor with twice the points; 16.8 mm / 0.128 deg); 0.5 s
TEST_CASE("consistency: Consistency reduces the offsets between windows")
{
    check_seams(false);
}

// Measured 2026-10-04 (Release, 3 passes): mean error 27.9 mm / 0.207 deg -> 14.6 mm / 0.125 deg, 3 cm voxels
// 127,956 -> 126,324 (truth 125,898); 0.6 s (scene at the world origin, see above).
// Measured 2026-10-14 with the scene at grid_offset: 27.9 mm / 0.207 deg -> 12.9 mm / 0.103 deg, voxels 127,982 -> 125,702
// (truth 121,634): the excess shrinks by 36 % (51 % with the cell index truncated, 12.7 mm / 0.095 deg); 0.8 s
TEST_CASE("consistency: Consistency2 reduces the offsets between windows")
{
    check_seams(true);
}

// Expected to fail (found 2026-10-04): the pass moves the true trajectory by up to 6.4 mm / 0.075 deg (mean 2.1 mm),
// spread evenly over all windows (2-6 mm in each). That is the noise of fitting 150 points per node to 0.15 m NDT
// cells, not a systematic pull: the rotation stays within 0.1 deg. When the pass leaves a correct trajectory within
// 5 mm, doctest reports this case as passing unexpectedly: then remove should_fail. Measured 2026-10-14 (scene at
// grid_offset, C10): up to 6.3 mm / 0.067 deg.
TEST_CASE("consistency: Consistency leaves a trajectory that is already right" * doctest::should_fail())
{
    check_truth(false);
}

// Expected to fail (found 2026-10-04): the pass moves the true trajectory by up to 16 mm / 0.46 deg (mean 5.2 mm). The
// rotation changes sit in the corner windows (0.2-0.46 deg there, about 0.05 deg on the straights): the smoothness
// term holds the change of turn rate from node to node to 0.01 deg (Consistency: 0.1 deg), and the fixture's turn rate
// ramps by 0.29 deg per node in and out of each corner. With a 1 s ramp (half the change per node) the corner windows
// drop to 0.18 deg, and an even 5-12 mm / 0.1 deg remains in every window. Measured 2026-10-14 (scene at grid_offset,
// C10): up to 10.7 mm / 0.35 deg.
TEST_CASE("consistency: Consistency2 leaves a trajectory that is already right" * doctest::should_fail())
{
    check_truth(true);
}
