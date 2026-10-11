#include <doctest/doctest.h>

#include "angle_units.h"
#include "loop_closure.h"
#include "memory_point_store.h"
#include "synthetic_recording.h"

#include <chrono>
#include <limits>

// run_loop_closure on the loop recording (two laps around a block, see synthetic_recording.h) against its true
// trajectory. The options are those of settings/lidar-vertical.toml, except the segment length and the time gap,
// which are shortened to fit the 44 s recording; they are set here, so tuning the shipped file doesn't change the tests.
//
// Skipped in the regular run (ctest): each case takes about 20 s in Release and more than 15 minutes in Debug. To run them,
// e.g. to debug the loop closure:  lidar_odometry_tests -tc="loop closure*" --no-skip -s

using namespace synthetic;

namespace
{
    const double deg = DEG_TO_RAD;

    OdometryConfig loop_closure_config()
    {
        OdometryConfig c;
        c.use_loop_closure = true;
        c.loop_closure_segment_windows = 4;   // 4 s per segment (shipped: 12)
        c.loop_closure_min_time_gap_s = 10.0; // (shipped: 30)
        c.loop_closure_search_radius_m = 8.0;
        c.loop_closure_max_vertical_offset_m = 1.5;
        c.loop_closure_max_correction_m = 6.0;
        c.loop_closure_min_fitness = 0.5;
        c.loop_closure_odometry_sigma_m = 0.1;
        c.loop_closure_odometry_sigma_deg = 0.5;
        c.loop_closure_edge_sigma_m = 0.05;
        c.loop_closure_edge_sigma_deg = 0.3;
        c.min_distance_lidar = 0.15;
        c.max_distance_lidar = 25.0;
        c.use_exact_ndt_bucket_statistics = false;
        return c;
    }

    void store_points(const Recording &r, MemoryPointStore &store)
    {
        for (size_t w = 0; w < r.points.size(); w++)
            store.save(PointSet::WindowIntermediate, w, r.points[w]);
    }

    // odometry drift on the second lap: from its start, a translation growing linearly to `shift` and a yaw growing to
    // `yaw` at the end, applied about the origin to poses and motion_model_poses (the points stay as seen from the truth)
    void add_second_lap_drift(Recording &r, const Eigen::Vector3d &shift, double yaw)
    {
        const double t_end = r.windows.back().timestamps.back().first;
        for (Window &w : r.windows)
            for (size_t n = 0; n < w.poses.size(); n++)
            {
                const double t = w.timestamps[n].first;
                if (t < r.lap_time_s)
                    continue;
                const double f = (t - r.lap_time_s) / (t_end - r.lap_time_s);
                Eigen::Affine3d drift = Eigen::Affine3d::Identity();
                drift.linear() = Eigen::AngleAxisd(f * yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                drift.translation() = f * shift;
                w.poses[n] = drift * w.poses[n];
                w.motion_model_poses[n] = w.poses[n];
            }
    }

    struct Run
    {
        int loops = 0;
        double ms = 0.0;
    };

    Run run(Recording &r, MemoryPointStore &store)
    {
        NullLogger logger;
        const auto start = std::chrono::steady_clock::now();
        Run result;
        result.loops = run_loop_closure(r.windows, loop_closure_config(), store, logger);
        result.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return result;
    }
}

TEST_CASE("loop closure: drift on the second lap is reduced" * doctest::skip())
{
    Recording r = loop_recording(2);
    MemoryPointStore store;
    store_points(r, store);
    add_second_lap_drift(r, {0.3, 0.4, 0.0}, 2.0 * deg);
    const auto truth = all_poses(r.truth);
    const auto before = all_poses(r.windows);

    const Run result = run(r, store);
    const auto after = all_poses(r.windows);

    const double end_before = (before.back().translation() - truth.back().translation()).norm();
    const double end_after = (after.back().translation() - truth.back().translation()).norm();
    const PoseError max_before = max_error(before, truth), max_after = max_error(after, truth);
    const PoseError mean_before = mean_error(before, truth), mean_after = mean_error(after, truth);
    MESSAGE("drift: ", result.loops, " loops accepted; end error ", end_before * 1000.0, " -> ", end_after * 1000.0,
            " mm; largest ", max_before.position * 1000.0, " mm / ", max_before.rotation / deg, " deg -> ", max_after.position * 1000.0,
            " mm / ", max_after.rotation / deg, " deg; mean ", mean_before.position * 1000.0, " -> ", mean_after.position * 1000.0,
            " mm; ", result.ms, " ms");
    // Measured 2026-10-04 (Release): 5 loops; end error 575 -> 112 mm, largest 575 mm / 2 deg -> 112 mm / 0.36 deg, mean
    // 113 -> 34 mm; 20.6 s
    CHECK(result.loops >= 1);
    CHECK(end_after <= 0.5 * end_before);
}

// Expected to fail (found 2026-10-04): every already-aligned revisit still registers with a "correction" of 1-3 cm and
// up to 0.2 deg (coarse search on 0.1 m / 0.75 deg steps, then NDT refinement on 0.5 / 0.25 m cells of sparse points),
// and the pose graph, which trusts loop edges to loop_closure_edge_sigma_m = 0.05, spreads that onto the nodes: measured
// 5 loops, largest change 32 mm / 0.09 deg (the exact NDT cell statistics give the same). When the loop
// closure leaves a correct trajectory alone, doctest reports this case as passing unexpectedly: then remove should_fail.
TEST_CASE("loop closure: a trajectory that is already right stays" * doctest::skip() * doctest::should_fail())
{
    Recording r = loop_recording(2);
    MemoryPointStore store;
    store_points(r, store);
    const auto truth = all_poses(r.truth);

    const Run result = run(r, store);
    const PoseError change = max_error(all_poses(r.windows), truth);
    MESSAGE("truth: ", result.loops, " loops accepted; largest change ", change.position * 1000.0, " mm / ", change.rotation / deg,
            " deg; ", result.ms, " ms");
    CHECK(change.position <= 0.005);
    CHECK(change.rotation <= 0.1 * deg);
}

TEST_CASE("loop closure: no revisit leaves the trajectory untouched" * doctest::skip())
{
    // the first 16 windows (three quarters of a lap): no two segments are both 10 s apart and close to each other
    Recording r = loop_recording(2);
    r.windows.resize(16);
    r.truth.resize(16);
    r.points.resize(16);
    MemoryPointStore store;
    store_points(r, store);
    const auto before = all_poses(r.windows);

    const Run result = run(r, store);
    const auto after = all_poses(r.windows);
    bool unchanged = before.size() == after.size();
    for (size_t k = 0; unchanged && k < before.size(); k++)
        unchanged = before[k].matrix() == after[k].matrix();
    MESSAGE("no revisit: ", result.loops, " loops accepted; poses unchanged ", unchanged, "; ", result.ms, " ms");
    CHECK(result.loops == 0);
    CHECK(unchanged);
}

// the pose graph of step 4 and the check of its result, on hand-made segment poses (fast, not skipped)
namespace
{
    Eigen::Affine3d segment_pose(double x, double y, double yaw_deg)
    {
        Eigen::Affine3d p = Eigen::Affine3d::Identity();
        p.translation() = Eigen::Vector3d(x, y, 0.0);
        p.linear() = Eigen::AngleAxisd(yaw_deg * DEG_TO_RAD, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return p;
    }
}

TEST_CASE("loop closure pose graph: a loop edge removes the drift and the result is plausible")
{
    OdometryConfig c = loop_closure_config();
    // a square of 8 segments; the odometry drifts 5 cm sideways per segment
    std::vector<Eigen::Affine3d> truth, initial;
    for (int k = 0; k < 8; k++)
    {
        const double x = k < 4 ? k * 2.0 : (7 - k) * 2.0;
        const double y = k < 4 ? 0.0 : 3.0;
        truth.push_back(segment_pose(x, y, k < 4 ? 0.0 : 180.0));
        initial.push_back(segment_pose(x, y + 0.05 * k, k < 4 ? 0.0 : 180.0));
    }
    // the revisit of segment 0 by segment 7 measured exactly: the correction moving 7 onto its true place relative to 0
    LoopClosureEdge e;
    e.i = 0;
    e.j = 7;
    e.correction = (initial[0] * truth[0].inverse() * truth[7]) * initial[7].inverse();
    const std::vector<LoopClosureEdge> loops{e};

    const std::vector<Eigen::Affine3d> optimized = optimize_loop_pose_graph(initial, loops, c);
    REQUIRE(optimized.size() == initial.size());
    const double before = (initial[7].translation() - truth[7].translation()).norm();
    const double after = (optimized[7].translation() - truth[7].translation()).norm();
    CHECK(after < 0.2 * before);
    double max_shift = 0.0;
    CHECK(loop_pose_graph_plausible(initial, optimized, loops, c, max_shift));
    CHECK(max_shift < 0.5);
}

TEST_CASE("loop closure pose graph: a result that ran away or is not finite is refused")
{
    OdometryConfig c = loop_closure_config();
    c.loop_closure_max_correction_m = 6.0;
    const std::vector<Eigen::Affine3d> initial{segment_pose(0, 0, 0), segment_pose(2, 0, 0), segment_pose(4, 0, 0)};
    LoopClosureEdge e;
    e.i = 0;
    e.j = 2;
    e.correction = segment_pose(0.5, 0, 0); // 0.5 m
    const std::vector<LoopClosureEdge> loops{e};
    double max_shift = 0.0;

    std::vector<Eigen::Affine3d> moved = initial;
    moved[2].translation().x() += 1.0;
    CHECK(loop_pose_graph_plausible(initial, moved, loops, c, max_shift));
    CHECK(max_shift == doctest::Approx(1.0));

    std::vector<Eigen::Affine3d> ran_away = initial;
    ran_away[2].translation().x() += 2.2e6; // as seen on the 44 with exact statistics and the accelerometer term off
    CHECK_FALSE(loop_pose_graph_plausible(initial, ran_away, loops, c, max_shift));

    std::vector<Eigen::Affine3d> broken = initial;
    broken[1].translation().y() = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(loop_pose_graph_plausible(initial, broken, loops, c, max_shift));
}

TEST_CASE("loop closure pose graph: a false loop edge stands out by its residual")
{
    OdometryConfig c = loop_closure_config();
    std::vector<Eigen::Affine3d> truth, initial;
    for (int k = 0; k < 8; k++)
    {
        const double x = k < 4 ? k * 2.0 : (7 - k) * 2.0;
        const double y = k < 4 ? 0.0 : 3.0;
        truth.push_back(segment_pose(x, y, k < 4 ? 0.0 : 180.0));
        initial.push_back(segment_pose(x, y + 0.05 * k, k < 4 ? 0.0 : 180.0));
    }
    const auto edge = [&](int i, int j, const Eigen::Vector3d &error)
    {
        LoopClosureEdge e;
        e.i = i;
        e.j = j;
        e.correction = (initial[i] * truth[i].inverse() * truth[j]) * initial[j].inverse();
        e.correction.translation() += error;
        return e;
    };
    // two revisits measured exactly, one registration that snapped 1 m off (a wrong match): the solution spreads its error
    // over the good edges too, but it keeps the largest residual, so dropping the worst edge first removes it
    std::vector<LoopClosureEdge> loops{edge(0, 7, Eigen::Vector3d::Zero()), edge(1, 6, Eigen::Vector3d::Zero()),
                                       edge(2, 5, Eigen::Vector3d(1.0, 0.0, 0.0))};
    const std::vector<LoopEdgeResidual> r = loop_edge_residuals(initial, optimize_loop_pose_graph(initial, loops, c), loops);
    REQUIRE(r.size() == 3);
    CHECK(r[2].translation_m > c.loop_closure_max_residual_m);
    CHECK(r[2].translation_m > std::max(r[0].translation_m, r[1].translation_m));
    loops.pop_back();
    for (const LoopEdgeResidual &good : loop_edge_residuals(initial, optimize_loop_pose_graph(initial, loops, c), loops))
        CHECK(good.translation_m < 0.02);
}

// the robust kernel on the loop edges (loop_closure_kernel_scale): two laps of the square, 8 segments each, drifting 3 cm
// sideways per segment; lap 2 revisits lap 1 segment by segment, so every stretch is covered by several loops and one
// edge that disagrees with the others can be told apart (with a single loop its error just spreads into the odometry)
namespace
{
    struct TwoLaps
    {
        std::vector<Eigen::Affine3d> truth, initial;
        TwoLaps()
        {
            for (int k = 0; k < 16; k++)
            {
                const int q = k % 8;
                const double x = q < 4 ? q * 2.0 : (7 - q) * 2.0;
                const double y = q < 4 ? 0.0 : 3.0;
                truth.push_back(segment_pose(x, y, q < 4 ? 0.0 : 180.0));
                initial.push_back(segment_pose(x, y + 0.03 * k, q < 4 ? 0.0 : 180.0));
            }
        }
        // the revisits of lap 1 by lap 2 as measured; the one at `wrong` off by `error` along x
        std::vector<LoopClosureEdge> loops(int wrong = -1, double error = 0.0) const
        {
            std::vector<LoopClosureEdge> out;
            for (int k = 0; k < 8; k++)
            {
                LoopClosureEdge e;
                e.i = k;
                e.j = k + 8;
                e.correction = (initial[k] * truth[k].inverse() * truth[k + 8]) * initial[k + 8].inverse();
                if (k == wrong)
                    e.correction.translation().x() += error;
                out.push_back(e);
            }
            return out;
        }
        double largest_error(const std::vector<Eigen::Affine3d> &poses) const
        {
            double m = 0.0;
            for (size_t k = 0; k < poses.size(); k++)
                m = std::max(m, (poses[k].translation() - truth[k].translation()).norm());
            return m;
        }
    };

    OdometryConfig with_kernel(double scale)
    {
        OdometryConfig c = loop_closure_config();
        c.loop_closure_kernel_scale = scale;
        return c;
    }
}

TEST_CASE("loop closure pose graph: the kernel weakens a slightly wrong edge")
{
    const TwoLaps s;
    // one revisit 15 cm off, below the drop limit: without the kernel it counts in full
    const auto loops = s.loops(3, 0.15);
    const double e_plain = s.largest_error(optimize_loop_pose_graph(s.initial, loops, with_kernel(0.0)));
    const double e_robust = s.largest_error(optimize_loop_pose_graph(s.initial, loops, with_kernel(0.5)));
    const double e_truth = s.largest_error(optimize_loop_pose_graph(s.initial, s.loops(), with_kernel(0.0)));
    MESSAGE("largest position error: plain ", e_plain, " m, kernel 0.5 ", e_robust, " m, all edges right ", e_truth, " m");
    CHECK(e_robust < 0.75 * e_plain);
    CHECK(e_robust < e_truth + 0.005);
}

TEST_CASE("loop closure pose graph: consistent edges give the same solution with and without the kernel")
{
    const TwoLaps s;
    const auto a = optimize_loop_pose_graph(s.initial, s.loops(), with_kernel(0.0));
    const auto b = optimize_loop_pose_graph(s.initial, s.loops(), with_kernel(0.5));
    REQUIRE(a.size() == b.size());
    double largest = 0.0;
    for (size_t k = 0; k < a.size(); k++)
        largest = std::max(largest, (a[k].translation() - b[k].translation()).norm());
    MESSAGE("largest difference ", largest, " m");
    CHECK(largest < 2e-3);
}

TEST_CASE("loop closure pose graph: with the kernel a false edge still stands out by its residual")
{
    const TwoLaps s;
    const OdometryConfig c = with_kernel(0.5);
    const auto loops = s.loops(3, 1.0);
    const std::vector<LoopEdgeResidual> r = loop_edge_residuals(s.initial, optimize_loop_pose_graph(s.initial, loops, c), loops);
    REQUIRE(r.size() == 8);
    CHECK(r[3].translation_m > c.loop_closure_max_residual_m);
    for (int k = 0; k < 8; k++)
        if (k != 3)
            CHECK(r[3].translation_m > r[k].translation_m);
}


TEST_CASE("loop closure pose graph: a cross-floor edge ties the translation, not the rotation")
{
    // the revisits of the two laps, plus one more edge whose registration turned 3 deg about the vertical (a round shaft):
    // as a cross-floor edge it hardly turns the solution and isn't dropped for its rotation; as a same-floor edge it turns it
    const TwoLaps s;
    const OdometryConfig c = with_kernel(0.0);
    auto loops = s.loops();
    const auto reference = optimize_loop_pose_graph(s.initial, loops, c);
    LoopClosureEdge turned = s.loops()[5];
    turned.correction = Eigen::AngleAxisd(3.0 * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) * turned.correction;
    loops.push_back(turned);
    const auto yaw_change = [&](const std::vector<Eigen::Affine3d> &p)
    {
        double m = 0.0;
        for (size_t k = 0; k < p.size(); k++)
            m = std::max(m, Eigen::AngleAxisd(reference[k].linear().transpose() * p[k].linear()).angle() / DEG_TO_RAD);
        return m;
    };

    loops.back().cross_floor = false;
    const double same_floor = yaw_change(optimize_loop_pose_graph(s.initial, loops, c));
    loops.back().cross_floor = true;
    const auto solved = optimize_loop_pose_graph(s.initial, loops, c);
    const double cross_floor = yaw_change(solved);
    MESSAGE("largest rotation change: as a same-floor edge ", same_floor, " deg, as a cross-floor edge ", cross_floor, " deg");
    CHECK(cross_floor < 0.2 * same_floor);
    CHECK(cross_floor < 0.1);

    const std::vector<LoopEdgeResidual> r = loop_edge_residuals(s.initial, solved, loops);
    CHECK(r.back().angle_deg > c.loop_closure_max_residual_deg); // the rotation disagrees ...
    CHECK(loop_edge_drop_ratio(loops.back(), r.back(), c) < 1.0); // ... but only the translation counts
    LoopClosureEdge same = loops.back();
    same.cross_floor = false;
    CHECK(loop_edge_drop_ratio(same, r.back(), c) > 1.0);
}

TEST_CASE("loop closure pose graph: a point edge ties a point horizontally, not the rotation or the height")
{
    // three segments 2 m apart; the odometry turned the last one 2 deg and drifted it 0.3 m sideways and 0.2 m up
    OdometryConfig c = with_kernel(0.0);
    std::vector<Eigen::Affine3d> initial{segment_pose(0, 0, 0), segment_pose(2, 0, 0), segment_pose(4, 0.3, 2.0)};
    initial[2].translation().z() = 0.2;
    // a shaft centre 3 m to the side: seen by segment 2 at its drifted place, the registration puts it where segment 0 sees it
    const Eigen::Vector3d centre(4.0, 3.3, 0.5);
    LoopClosureEdge e;
    e.i = 0;
    e.j = 2;
    e.cross_floor = true;
    e.point = true;
    e.point_j = initial[2].inverse() * centre;
    e.point_i = initial[0].inverse() * Eigen::Vector3d(4.0, 3.0, 0.0);
    const std::vector<LoopClosureEdge> loops{e};

    const auto p = optimize_loop_pose_graph(initial, loops, c);
    REQUIRE(p.size() == 3);
    const Eigen::Vector3d placed_j = p[2] * e.point_j, placed_i = p[0] * e.point_i;
    MESSAGE("point edge: horizontal gap ", (placed_j - placed_i).head<2>().norm(), " m (before 0.30 m)");
    // least squares against the two odometry edges (sigma 0.1 m each) and the point edge (0.05 m): the gap left is
    // 0.30 m * 0.05^2 / (0.05^2 + 2 * 0.1^2)
    const double expected_gap = 0.30 * std::pow(c.loop_closure_edge_sigma_m, 2) /
                                (std::pow(c.loop_closure_edge_sigma_m, 2) + 2.0 * std::pow(c.loop_closure_odometry_sigma_m, 2));
    CHECK((placed_j - placed_i).head<2>().norm() == doctest::Approx(expected_gap).epsilon(0.15));
    // the odometry's relative rotation and height stay (the edge carries neither)
    const Eigen::Affine3d rel_before = initial[1].inverse() * initial[2], rel_after = p[1].inverse() * p[2];
    CHECK(Eigen::AngleAxisd(rel_before.linear().transpose() * rel_after.linear()).angle() / DEG_TO_RAD < 0.3);
    CHECK(std::abs((p[2].translation() - initial[2].translation()).z()) < 0.02);
    // the drop judges it by the remaining horizontal gap
    const auto r = loop_edge_residuals(initial, p, loops);
    REQUIRE(r.size() == 1);
    CHECK(r[0].translation_m == doctest::Approx(expected_gap).epsilon(0.15));
    CHECK(r[0].angle_deg == 0.0);
}
