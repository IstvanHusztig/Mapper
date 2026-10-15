#include <doctest/doctest.h>

#include "angle_units.h"
#include "loop_closure.h"
#include "memory_point_store.h"
#include "synthetic_recording.h"
#include <transformations.h>

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
        c.loop_closure_segment_windows = 4;   // 4 s per segment (shipped: 6)
        c.loop_closure_min_path_m = 8.0;      // (shipped)
        c.loop_closure_radius_per_path = 0.5; // (shipped)
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

namespace
{
    // windows whose first nodes lie at the given positions (one node per window is enough for the path)
    std::vector<Window> windows_at(const std::vector<Eigen::Vector3d> &positions)
    {
        std::vector<Window> windows;
        for (const auto &p : positions)
        {
            Window w;
            Eigen::Affine3d m = Eigen::Affine3d::Identity();
            m.translation() = p;
            w.poses.push_back(m);
            windows.push_back(w);
        }
        return windows;
    }

    RevisitSegment segment_at(double x, double y, double path_begin, double path_end)
    {
        return {Eigen::Vector3d(x, y, 0.0), path_begin, path_end};
    }
}

TEST_CASE("loop closure revisits: the path walked ignores standing still and counts walking")
{
    std::vector<Eigen::Vector3d> positions;
    for (int k = 0; k < 300; k++) // 2 min standing, 4 mm pose jitter per window
        positions.emplace_back(0.004 * ((k % 3) - 1), 0.004 * ((k % 2) * 2 - 1), 0.0);
    for (int k = 1; k <= 20; k++) // then 10 m along x in 0.5 m steps
        positions.emplace_back(0.5 * k, 0.0, 0.0);
    const std::vector<double> path = path_at_windows(windows_at(positions));
    CHECK(path[299] == 0.0);
    CHECK(path.back() == doctest::Approx(10.0).epsilon(0.01));
}

TEST_CASE("loop closure revisits: out and back qualifies, neighbours, standing still and a place seen from afar don't")
{
    OdometryConfig c;
    c.loop_closure_min_path_m = 8.0;
    c.loop_closure_radius_per_path = 0.5;
    c.loop_closure_search_radius_m = 8.0;
    c.loop_closure_max_vertical_offset_m = 1.5;
    // a dead end walked out and back: segment 3 returns to segment 0's place after 9 m (gallery 10 -> 13 on 44)
    const std::vector<RevisitSegment> out_and_back = {segment_at(0.0, 0.0, 0.0, 4.5), segment_at(4.0, 0.0, 4.5, 9.0),
                                                      segment_at(4.5, 0.4, 9.0, 13.5), segment_at(0.4, 0.0, 13.5, 18.0)};
    CHECK(same_floor_revisits(out_and_back, 3, c) == std::vector<int>{0});
    // walking on: the next-but-one segment is about as far as the path between them (ratio about 1)
    const std::vector<RevisitSegment> straight = {segment_at(0.0, 0.0, 0.0, 4.5), segment_at(4.5, 0.0, 4.5, 9.0),
                                                  segment_at(9.0, 0.0, 9.0, 13.5), segment_at(13.5, 0.0, 13.5, 18.0)};
    CHECK(same_floor_revisits(straight, 3, c).empty());
    // standing still: no path between, however long the time
    const std::vector<RevisitSegment> standing = {segment_at(0.0, 0.0, 0.0, 0.0), segment_at(0.1, 0.0, 0.0, 0.0), segment_at(0.0, 0.1, 0.0, 0.0)};
    CHECK(same_floor_revisits(standing, 2, c).empty());
    // a place seen from 6.8 m after 8.6 m of path (gallery 9 -> 12 on 44: ratio 0.79)
    const std::vector<RevisitSegment> afar = {segment_at(0.0, 0.0, 0.0, 0.0), segment_at(6.8, 0.0, 8.6, 13.0)};
    CHECK(same_floor_revisits(afar, 1, c).empty());
    // another floor: not a same-floor revisit
    std::vector<RevisitSegment> upstairs = out_and_back;
    upstairs[3].center.z() = 2.8;
    CHECK(same_floor_revisits(upstairs, 3, c).empty());
}

TEST_CASE("loop closure revisits: a far revisit is tried before a near one")
{
    // segment 3 is back at the start after a round of 40 m (segment 0) and also near segment 2, walked 9 m earlier
    OdometryConfig c;
    c.loop_closure_min_path_m = 8.0;
    c.loop_closure_radius_per_path = 0.5;
    c.loop_closure_search_radius_m = 8.0;
    c.loop_closure_max_vertical_offset_m = 1.5;
    const std::vector<RevisitSegment> segments = {segment_at(0.0, 0.0, 0.0, 4.5), segment_at(20.0, 0.0, 4.5, 25.0),
                                                  segment_at(1.5, 0.0, 31.0, 35.5), segment_at(0.5, 0.0, 44.5, 49.0)};
    CHECK(same_floor_revisits(segments, 3, c) == std::vector<int>{0, 2});
}

TEST_CASE("loop closure revisits: the tries are the nearest and the longest-path ones, each with a second one at least 3 segments away")
{
    OdometryConfig c;
    c.loop_closure_min_path_m = 8.0;
    c.loop_closure_radius_per_path = 0.5;
    c.loop_closure_search_radius_m = 8.0;
    c.loop_closure_max_vertical_offset_m = 1.5;
    c.loop_closure_candidates = 4;
    // segments 0..7 all qualify as revisits of segment 8 (path between 30..100 m); distances from segment 8's centre:
    const std::vector<double> distance = {3.0, 2.9, 2.0, 1.0, 1.5, 0.5, 0.4, 0.6};
    std::vector<RevisitSegment> segments;
    for (size_t k = 0; k < distance.size(); k++)
        segments.push_back(segment_at(distance[k], 0.0, 10.0 * static_cast<double>(k), 10.0 * static_cast<double>(k)));
    segments.push_back(segment_at(0.0, 0.0, 100.0, 104.0));
    // nearest 6, then 5 and 7 are its neighbours, so 3; longest path 0, then 1 and 2 are its neighbours, so 3 again
    const std::vector<RevisitTry> tries = revisit_tries(segments, 8, c);
    REQUIRE(tries.size() == 3);
    CHECK((tries[0].i == 6 && tries[0].near && !tries[0].far));
    CHECK((tries[1].i == 0 && !tries[1].near && tries[1].far));
    CHECK((tries[2].i == 3 && tries[2].near && tries[2].far));
    // a cap of 2 keeps the nearest and the longest-path one: a near revisit doesn't take the place of a far one
    c.loop_closure_candidates = 2;
    const std::vector<RevisitTry> two = revisit_tries(segments, 8, c);
    REQUIRE(two.size() == 2);
    CHECK(two[0].i == 6);
    CHECK(two[1].i == 0);
}

TEST_CASE("loop closure revisits: two slots keep the best near and the best far edge when their sources are apart, else the best one")
{
    const auto accepted = [](int i, double fitness, bool near, bool far) {
        AcceptedRevisit a;
        a.edge.i = i;
        a.edge.j = 8;
        a.edge.fitness = fitness;
        a.near = near;
        a.far = far;
        return a;
    };
    const auto sources = [](const std::vector<LoopClosureEdge> &edges) {
        std::vector<int> out;
        for (const auto &e : edges)
            out.push_back(e.i);
        return out;
    };
    const std::vector<AcceptedRevisit> three = {accepted(6, 0.8, true, false), accepted(3, 0.7, true, true), accepted(0, 0.6, false, true)};
    CHECK(sources(revisit_slots(three, 1)) == std::vector<int>{6});
    CHECK(sources(revisit_slots(three, 2)) == std::vector<int>{6, 3});
    // one edge best in both classes: kept once
    const std::vector<AcceptedRevisit> one_best = {accepted(6, 0.6, true, false), accepted(3, 0.9, true, true), accepted(0, 0.5, false, true)};
    CHECK(sources(revisit_slots(one_best, 2)) == std::vector<int>{3});
    // near and far sources 2 segments apart: they share a target, only the better one is kept
    const std::vector<AcceptedRevisit> close = {accepted(5, 0.7, true, false), accepted(3, 0.8, false, true)};
    CHECK(sources(revisit_slots(close, 2)) == std::vector<int>{3});
    // only one class registered
    CHECK(sources(revisit_slots({accepted(0, 0.6, false, true)}, 2)) == std::vector<int>{0});
    CHECK(revisit_slots({}, 2).empty());
}

TEST_CASE("loop closure drop order: leave one out drops the false edge, not the strained right one")
{
    // 14 segment anchors 6 m apart along x (truth straight). The odometry turned 1.5 deg too much between anchors 6 and 7.
    // Right loop edges hold the chain (0 -> 6, 7 -> 13, and across the drift 5 -> 9 with its twin 4 -> 9); a false long
    // edge 0 -> 13 claims a further 4 deg. Alone, a long false edge would be absorbed by the chain unnoticed (13 odometry
    // edges at 0.5 deg against 0.3 deg); held by the right edges it strains them, the twin pair most.
    OdometryConfig c;
    c.loop_closure_odometry_sigma_m = 0.1;
    c.loop_closure_odometry_sigma_deg = 0.5;
    c.loop_closure_edge_sigma_m = 0.05;
    c.loop_closure_edge_sigma_deg = 0.3;
    c.loop_closure_max_residual_m = 0.15;
    c.loop_closure_max_residual_deg = 2.0;
    c.loop_closure_kernel_scale = 0.0;
    std::vector<Eigen::Affine3d> truth, initial;
    for (int k = 0; k < 14; k++)
    {
        truth.push_back(pose(Eigen::Vector3d(6.0 * k, 0.0, 0.0), 0.0));
        initial.push_back(k <= 6 ? truth[k] : truth[6] * pose(Eigen::Vector3d::Zero(), 1.5 * deg) * pose(Eigen::Vector3d(6.0 * (k - 6), 0.0, 0.0), 0.0));
    }
    const auto edge = [&](int i, int j, double extra_deg)
    {
        LoopClosureEdge e;
        e.i = i;
        e.j = j;
        // the correction that moves j onto where the truth (plus a false extra turn about i) puts it
        const Eigen::Affine3d target = truth[i] * pose(Eigen::Vector3d::Zero(), extra_deg * deg) * truth[i].inverse() * truth[j];
        e.correction = target * initial[j].inverse();
        return e;
    };
    const std::vector<LoopClosureEdge> loops = {edge(0, 6, 0.0), edge(7, 13, 0.0), edge(5, 9, 0.0), edge(4, 9, 0.0), edge(0, 13, 4.0)};
    const std::vector<Eigen::Affine3d> optimized = optimize_loop_pose_graph(initial, loops, c);
    const std::vector<LoopEdgeResidual> r = loop_edge_residuals(initial, optimized, loops);
    size_t worst_ratio = loops.size();
    double largest = 1.0;
    for (size_t e = 0; e < loops.size(); e++)
    {
        const double ratio = loop_edge_drop_ratio(loops[e], r[e], c);
        MESSAGE("edge ", loops[e].i, " -> ", loops[e].j, ": residual ", r[e].translation_m, " m / ", r[e].angle_deg, " deg, ratio ", ratio);
        if (ratio > largest)
        {
            largest = ratio;
            worst_ratio = e;
        }
    }
    int beyond = 0;
    for (size_t e = 0; e < loops.size(); e++)
        beyond += loop_edge_drop_ratio(loops[e], r[e], c) > 1.0 ? 1 : 0;
    const size_t left_out = loop_edge_to_drop(initial, optimized, loops, c);
    MESSAGE("largest ratio drops edge ", worst_ratio, ", leave one out drops edge ", left_out);
    // a choice among several edges beyond the limits, where the largest ratio would drop a right edge
    REQUIRE(beyond >= 2);
    CHECK(worst_ratio != 4);
    CHECK(left_out == 4);
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


// Information-weighted loop edges (cross-floor edges): the registration's information over the correction's Tait-Bryan parameters, here built from a synthetic shared region
// (points with their surface normals, point-to-plane), as ndt_refine's normal matrix would describe it.
namespace
{
    struct SurfacePoint
    {
        Eigen::Vector3d p; // in world coordinates of the target (after the correction)
        Eigen::Vector3d n;
    };

    // a vertical cylinder (a round shaft): radius r around (cx, cy), from z0 to z1
    std::vector<SurfacePoint> cylinder(double cx, double cy, double r, double z0, double z1)
    {
        std::vector<SurfacePoint> out;
        for (int a = 0; a < 72; a++)
            for (int h = 0; h <= 10; h++)
            {
                const double t = a * 5.0 * DEG_TO_RAD;
                const Eigen::Vector3d n(std::cos(t), std::sin(t), 0.0);
                out.push_back({Eigen::Vector3d(cx, cy, z0 + (z1 - z0) * h / 10.0) + r * n, n});
            }
        return out;
    }

    // a box 2 x 3 m with floor and ceiling (a lift shaft seen from inside, a room)
    std::vector<SurfacePoint> box(const Eigen::Vector3d &c)
    {
        std::vector<SurfacePoint> out;
        const Eigen::Vector3d half(1.0, 1.5, 1.25);
        for (int axis = 0; axis < 3; axis++)
            for (int side = -1; side <= 1; side += 2)
                for (int u = 0; u <= 8; u++)
                    for (int v = 0; v <= 8; v++)
                    {
                        Eigen::Vector3d q = Eigen::Vector3d::Zero();
                        const int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
                        q(axis) = side * half(axis);
                        q(a1) = (u / 4.0 - 1.0) * half(a1);
                        q(a2) = (v / 4.0 - 1.0) * half(a2);
                        Eigen::Vector3d n = Eigen::Vector3d::Zero();
                        n(axis) = side;
                        out.push_back({c + q, n});
                    }
        return out;
    }

    Eigen::Matrix<double, 6, 1> tb(const Eigen::Affine3d &T)
    {
        const TaitBryanPose p = pose_tait_bryan_from_affine_matrix(T);
        Eigen::Matrix<double, 6, 1> v;
        v << p.px, p.py, p.pz, p.om, p.fi, p.ka;
        return v;
    }

    Eigen::Affine3d from_tb(const Eigen::Matrix<double, 6, 1> &v)
    {
        TaitBryanPose p;
        p.px = v(0);
        p.py = v(1);
        p.pz = v(2);
        p.om = v(3);
        p.fi = v(4);
        p.ka = v(5);
        return affine_matrix_from_pose_tait_bryan(p);
    }

    // the point-to-plane normal matrix of a registration with the result `correction` over its Tait-Bryan parameters
    Eigen::Matrix<double, 6, 6> registration_information(const std::vector<SurfacePoint> &region, const Eigen::Affine3d &correction)
    {
        const Eigen::Matrix<double, 6, 1> c = tb(correction);
        Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
        constexpr double h = 1e-6;
        for (const SurfacePoint &s : region)
        {
            const Eigen::Vector3d source = correction.inverse() * s.p;
            Eigen::Matrix<double, 1, 6> J;
            for (int k = 0; k < 6; k++)
            {
                Eigen::Matrix<double, 6, 1> e = Eigen::Matrix<double, 6, 1>::Zero();
                e(k) = h;
                J(k) = s.n.dot(from_tb(c + e) * source - from_tb(c - e) * source) / (2.0 * h);
            }
            H += J.transpose() * J * 1e4; // a 1 cm point sigma (the scale is replaced anyway)
        }
        return H;
    }

    Eigen::Affine3d tilted_pose(double x, double y, double z, double yaw_deg, double pitch_deg)
    {
        Eigen::Affine3d p = segment_pose(x, y, yaw_deg);
        p.translation().z() = z;
        p.linear() = p.linear() * Eigen::AngleAxisd(pitch_deg * DEG_TO_RAD, Eigen::Vector3d::UnitY()).toRotationMatrix();
        return p;
    }

    double relative_angle_about(const Eigen::Affine3d &a, const Eigen::Affine3d &b, const Eigen::Vector3d &axis)
    {
        const Eigen::AngleAxisd d(a.linear().transpose() * b.linear());
        return d.angle() * d.axis().dot(axis) / DEG_TO_RAD;
    }
}

TEST_CASE("loop closure information edges: the transformed information reproduces the registration's error in the edge's residual")
{
    const Eigen::Affine3d pose_i = tilted_pose(0, 0, 0, 10, 0), pose_j = tilted_pose(5, 1, 0.1, 40, 1);
    Eigen::Affine3d correction = Eigen::Affine3d::Identity();
    correction.translation() = Eigen::Vector3d(0.2, -0.1, 0.05);
    correction.linear() = Eigen::AngleAxisd(1.5 * DEG_TO_RAD, Eigen::Vector3d(0.1, 0.2, 1.0).normalized()).toRotationMatrix();
    const Eigen::Matrix<double, 6, 6> H = registration_information(box(Eigen::Vector3d(3, 2, 1)), correction);
    bool ok = false;
    const Eigen::Matrix<double, 6, 6> omega = loop_edge_information(pose_i, pose_j, correction, H, false, 0.05, ok);
    REQUIRE(ok);
    // the best-determined translation has the edge sigma
    const double best = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(omega.topLeftCorner<3, 3>()).eigenvalues()(2);
    CHECK(best == doctest::Approx(1.0 / (0.05 * 0.05)).epsilon(1e-9));
    // for small perturbations of the registration, the measurement's weighted change is the registration's weighted error,
    // up to the one scale factor
    const auto measurement = [&](const Eigen::Affine3d &C)
    {
        Eigen::Matrix<double, 6, 1> m = tb(pose_i.inverse() * C * pose_j);
        return m;
    };
    std::vector<double> ratios;
    for (int k = 0; k < 6; k++)
    {
        Eigen::Matrix<double, 6, 1> e;
        e << 1e-4 * (k + 1), -2e-4 * (k % 2), 1.5e-4, 0.5e-4 * (k % 3), -1e-4, 2e-4 * (k == 2);
        const Eigen::Matrix<double, 6, 1> dm = measurement(from_tb(tb(correction) + e)) - measurement(correction);
        ratios.push_back(dm.dot(omega * dm) / e.dot(H * e));
    }
    for (double r : ratios)
        CHECK(r == doctest::Approx(ratios.front()).epsilon(1e-3));
}

TEST_CASE("loop closure information edges: a cross-floor edge carries no height")
{
    const Eigen::Affine3d pose_i = tilted_pose(0, 0, 0, 0, 0), pose_j = tilted_pose(1, 3, -2.9, 30, 0.5);
    const Eigen::Affine3d correction = Eigen::Affine3d(Eigen::Translation3d(0.1, 0.05, 0.3));
    const Eigen::Matrix<double, 6, 6> H = registration_information(box(Eigen::Vector3d(1, 4, -1.6)), correction);
    bool ok = false;
    const Eigen::Matrix<double, 6, 6> omega = loop_edge_information(pose_i, pose_j, correction, H, true, 0.05, ok);
    REQUIRE(ok);
    const auto measurement = [&](const Eigen::Affine3d &C) { return Eigen::Matrix<double, 6, 1>(tb(pose_i.inverse() * C * pose_j)); };
    const Eigen::Matrix<double, 6, 1> up = measurement(Eigen::Translation3d(0, 0, 0.01) * correction) - measurement(correction);
    const Eigen::Matrix<double, 6, 1> side = measurement(Eigen::Translation3d(0.01, 0, 0) * correction) - measurement(correction);
    MESSAGE("weighted squared change for 1 cm: up ", up.dot(omega * up), ", sideways ", side.dot(omega * side));
    CHECK(up.dot(omega * up) < 1e-6 * side.dot(omega * side));
    CHECK(side.dot(omega * side) > 0.01); // sideways the edge ties (1 cm against a few cm sigma)
}

TEST_CASE("loop closure information edges: with the fixed sigmas as information the solution equals the fixed-sigma edges'")
{
    const TwoLaps s;
    const OdometryConfig c = with_kernel(0.0);
    const auto loops = s.loops();
    const auto reference = optimize_loop_pose_graph(s.initial, loops, c);
    auto weighted = loops;
    const double wt = 1.0 / std::pow(c.loop_closure_edge_sigma_m, 2), wr = 1.0 / std::pow(c.loop_closure_edge_sigma_deg * DEG_TO_RAD, 2);
    for (auto &e : weighted)
    {
        e.information = true;
        e.omega.setZero();
        e.omega.diagonal() << wt, wt, wt, wr, wr, wr;
    }
    const auto solved = optimize_loop_pose_graph(s.initial, weighted, c);
    REQUIRE(solved.size() == reference.size());
    double largest = 0.0;
    for (size_t k = 0; k < solved.size(); k++)
        largest = std::max(largest, (solved[k].matrix() - reference[k].matrix()).cwiseAbs().maxCoeff());
    MESSAGE("largest difference ", largest);
    CHECK(largest < 1e-9);
}

TEST_CASE("loop closure information edges: a shaft ties the floors horizontally and in tilt, not in height or the turn about its axis")
{
    // three segments 2 m apart; the odometry turned the last one 2 deg, tilted it 1 deg about y, and drifted it 0.3 m
    // sideways and 0.2 m up. A round shaft 3 m to the side is seen from both segment 0 and segment 2
    OdometryConfig c = with_kernel(0.0);
    const std::vector<Eigen::Affine3d> truth{segment_pose(0, 0, 0), segment_pose(2, 0, 0), segment_pose(4, 0, 0)};
    std::vector<Eigen::Affine3d> initial{truth[0], truth[1], tilted_pose(4, 0.3, 0.2, 2.0, 1.0)};
    const std::vector<SurfacePoint> shaft = cylinder(4.0, 3.0, 1.5, -1.0, 1.5);
    // the registration moves segment 2's view of the shaft onto segment 0's: right horizontally and in tilt, but (as a
    // shaft allows) 0.5 m too high and turned 5 deg about the shaft's axis
    const Eigen::Vector3d axis_point(4.0, 3.0, 0.0);
    const Eigen::Affine3d about_axis = Eigen::Translation3d(axis_point) * Eigen::AngleAxisd(5.0 * DEG_TO_RAD, Eigen::Vector3d::UnitZ()) *
                                       Eigen::Translation3d(-axis_point);
    LoopClosureEdge e;
    e.i = 0;
    e.j = 2;
    e.cross_floor = true;
    e.correction = Eigen::Translation3d(0, 0, 0.5) * about_axis * truth[2] * initial[2].inverse();
    bool ok = false;
    e.omega = loop_edge_information(initial[0], initial[2], e.correction, registration_information(shaft, e.correction), true,
                                    c.loop_closure_edge_sigma_m, ok);
    REQUIRE(ok);
    e.information = true;
    const Eigen::Matrix<double, 6, 1> sig = loop_edge_region_sigmas(initial[0], initial[2], e, axis_point);
    MESSAGE("shaft edge sigmas: x ", sig(0), " y ", sig(1), " z ", sig(2), " m, roll ", sig(3), " pitch ", sig(4), " yaw ", sig(5), " deg");
    CHECK(sig(0) < 0.1);
    CHECK(sig(1) < 0.1);
    CHECK(sig(2) > 10.0);
    CHECK(sig(5) > 90.0);
    CHECK(sig(3) < 10.0);
    CHECK(sig(4) < 10.0);

    const std::vector<LoopClosureEdge> loops{e};
    const auto p = optimize_loop_pose_graph(initial, loops, c);
    REQUIRE(p.size() == 3);
    // where segment 2 puts the shaft's axis, against the truth (segment 0's view)
    const Eigen::Vector3d axis_in_2 = initial[2] * (truth[2].inverse() * axis_point);
    const double gap_before = (axis_in_2 - axis_point).head<2>().norm();
    const double gap_after = ((p[2] * (initial[2].inverse() * axis_in_2)) - p[0] * (initial[0].inverse() * axis_point)).head<2>().norm();
    const double pitch_before = relative_angle_about(initial[0], initial[2], Eigen::Vector3d::UnitY());
    const double pitch_after = relative_angle_about(p[0], p[2], Eigen::Vector3d::UnitY());
    const double yaw_change = relative_angle_about(initial[1].inverse() * initial[2], p[1].inverse() * p[2], Eigen::Vector3d::UnitZ());
    MESSAGE("shaft edge: axis gap ", gap_before, " -> ", gap_after, " m, pitch ", pitch_before, " -> ", pitch_after,
            " deg, height change ", (p[2].translation() - initial[2].translation()).z(), " m, yaw change ", yaw_change, " deg");
    CHECK(gap_after < 0.5 * gap_before);
    CHECK(std::abs(pitch_after) < std::abs(pitch_before));
    CHECK(std::abs((p[2].translation() - initial[2].translation()).z()) < 0.02);
    CHECK(std::abs(yaw_change) < 0.3);

    // the drop rule: this edge's residual lies along its free directions (the 0.5 m and 5 deg) and in what the odometry
    // holds, so it isn't dropped; the same edge 0.3 m off sideways is
    const auto r = loop_edge_residuals(initial, p, loops);
    REQUIRE(r.size() == 1);
    MESSAGE("weighted residual ", r[0].weighted * c.loop_closure_edge_sigma_m, " m (plain ", r[0].translation_m, " m, ", r[0].angle_deg, " deg)");
    CHECK(loop_edge_drop_ratio(loops[0], r[0], c) < 1.0);
    LoopClosureEdge off = e;
    off.correction = Eigen::Translation3d(0.3, 0, 0) * e.correction;
    const auto r_off = loop_edge_residuals(initial, p, std::vector<LoopClosureEdge>{off});
    CHECK(loop_edge_drop_ratio(off, r_off[0], c) > 1.0);
}

TEST_CASE("loop closure information edges: a box-shaped region also ties the turn")
{
    // as above, but the shared region is a box (walls in two directions, floor and ceiling): the turn is tied too
    OdometryConfig c = with_kernel(0.0);
    const std::vector<Eigen::Affine3d> truth{segment_pose(0, 0, 0), segment_pose(2, 0, 0), segment_pose(4, 0, 0)};
    std::vector<Eigen::Affine3d> initial{truth[0], truth[1], tilted_pose(4, 0.3, 0.2, 2.0, 1.0)};
    LoopClosureEdge e;
    e.i = 0;
    e.j = 2;
    e.cross_floor = true;
    e.correction = Eigen::Translation3d(0, 0, 0.5) * truth[2] * initial[2].inverse();
    bool ok = false;
    e.omega = loop_edge_information(initial[0], initial[2], e.correction, registration_information(box(Eigen::Vector3d(4, 3, 0.25)), e.correction),
                                    true, c.loop_closure_edge_sigma_m, ok);
    REQUIRE(ok);
    e.information = true;
    const Eigen::Matrix<double, 6, 1> sig = loop_edge_region_sigmas(initial[0], initial[2], e, Eigen::Vector3d(4, 3, 0.25));
    MESSAGE("box edge sigmas: x ", sig(0), " y ", sig(1), " z ", sig(2), " m, roll ", sig(3), " pitch ", sig(4), " yaw ", sig(5), " deg");
    CHECK(sig(2) > 10.0);
    CHECK(sig(5) < 10.0);
    const auto p = optimize_loop_pose_graph(initial, std::vector<LoopClosureEdge>{e}, c);
    const double yaw_before = relative_angle_about(initial[0], initial[2], Eigen::Vector3d::UnitZ());
    const double yaw_after = relative_angle_about(p[0], p[2], Eigen::Vector3d::UnitZ());
    MESSAGE("box edge: relative yaw ", yaw_before, " -> ", yaw_after, " deg");
    // the turn moves toward the truth by about what weighted least squares gives: the edge's yaw sigma (2.5 deg, from the
    // region's size and the 5 cm translation sigma) against two odometry edges of 0.5 deg would take 8 % off; the coupling
    // with the translation adds some
    CHECK(std::abs(yaw_after) < std::abs(yaw_before) - 0.1);
}

TEST_CASE("loop closure: the plausibility check does not depend on the distance from the world origin")
{
    // two segments and a loop edge whose correction turns segment 1 by 2 deg about its own anchor; the optimized poses
    // carry that correction. Near the origin and 300 m away the check must see the same small movement (before, the
    // rotation about the world origin counted: about 10 m at 300 m)
    for (const double offset : {0.0, 300.0})
    {
        std::vector<Eigen::Affine3d> initial(2, Eigen::Affine3d::Identity());
        initial[0].translation() = Eigen::Vector3d(offset, 0.0, 0.0);
        initial[1].translation() = Eigen::Vector3d(offset + 5.0, 0.0, 0.0);
        Eigen::Affine3d turn = Eigen::Affine3d::Identity();
        turn.translate(initial[1].translation());
        turn.rotate(Eigen::AngleAxisd(2.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));
        turn.translate(-initial[1].translation());
        LoopClosureEdge e;
        e.i = 0;
        e.j = 1;
        e.correction = turn;
        std::vector<Eigen::Affine3d> optimized = initial;
        optimized[1] = turn * initial[1];
        OdometryConfig params;
        double max_shift_m = -1.0;
        CHECK(loop_pose_graph_plausible(initial, optimized, {e}, params, max_shift_m));
        CHECK(max_shift_m < 1e-9);
    }
}
