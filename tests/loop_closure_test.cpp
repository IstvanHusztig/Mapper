#include <doctest/doctest.h>

#include "angle_units.h"
#include "lidar_odometry.h"
#include "loop_closure.h"
#include "loop_closure_capture.h"
#include "loop_closure_capture_file.h"
#include "loop_closure_replay.h"
#include "memory_point_store.h"
#include "synthetic_recording.h"
#include "chi_square.h"
#include <transformations.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <string>
#include <limits>

// run_loop_closure on the loop recording (two laps around a block, see synthetic_recording.h) against its true
// trajectory. The options are those of settings/l2-vertical.toml, except the segment length and the time gap,
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
    // a dead end walked out and back: segment 3 returns to segment 0's place after 9 m
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
    // a place seen from 6.8 m after 8.6 m of path (ratio 0.79)
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
    ran_away[2].translation().x() += 2.2e6; // a run-away window, as seen with exact statistics and the accelerometer term off
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

namespace
{
    // walking at 1 m/s along x, 0.4 s windows, seen well (ratio 0.5)
    WindowReadings seen_window()
    {
        return WindowReadings{0.5, Eigen::Vector3d::UnitX(), false, Eigen::Vector3d(0.4, 0.0, 0.0), 0.4};
    }

    WindowReadings weak_window(const Eigen::Vector3d &axis, const Eigen::Vector3d &move)
    {
        return WindowReadings{0.01, axis, false, move, 0.4};
    }

    // every window with its readings
    std::vector<std::optional<WindowReadings>> all_read(const std::vector<WindowReadings> &rd)
    {
        return std::vector<std::optional<WindowReadings>>(rd.begin(), rd.end());
    }

    // keeps the warnings
    struct WarningLogger : Logger
    {
        std::vector<std::string> warnings;
        void write(LogLevel level, std::string_view message) override
        {
            if (level == LogLevel::Warning)
                warnings.emplace_back(message);
        }
    };

    // segments of 3 windows each; segment k turned by 30 k deg about z (the steps' frames differ from the world's)
    std::vector<LoopClosureSegment> segments_of_three(int windows)
    {
        std::vector<LoopClosureSegment> segments;
        for (int w0 = 0; w0 < windows; w0 += 3)
        {
            LoopClosureSegment s;
            s.first_window = w0;
            s.last_window = std::min(windows, w0 + 3) - 1;
            s.pose = Eigen::Affine3d(Eigen::AngleAxisd(30.0 * deg * (w0 / 3), Eigen::Vector3d::UnitZ()));
            s.pose.translation() = Eigen::Vector3d(0.4 * w0, 0.0, 0.0);
            segments.push_back(s);
        }
        return segments;
    }
}

TEST_CASE("loop closure odometry steps: weak runs, their sigmas and their shares among the steps")
{
    // 30 windows, 10 segments (9 steps):
    //  - windows 4-9 weak along x and stalled (0.1 m per window), window 6 seen in between (bridged): steps 1, 2, 3
    //  - window 13 weak along y, alone: step 4
    //  - windows 19 (weak along x) and 20 (weak along y): two runs, their axes differ
    //  - window 22 weak but at rest: no run
    //  - windows 26-29 weak along -x (the axis turns to the travel), in segments 8 and 9: step 8 and the last segment
    std::vector<WindowReadings> rd(30, seen_window());
    for (int w : {4, 5, 7, 8, 9})
        rd[w] = weak_window(w == 7 ? Eigen::Vector3d(-Eigen::Vector3d::UnitX()) : Eigen::Vector3d(Eigen::Vector3d::UnitX()), Eigen::Vector3d(0.1, 0.0, 0.0));
    rd[13] = weak_window(Eigen::Vector3d::UnitY(), Eigen::Vector3d(0.4, 0.1, 0.0));
    rd[19] = weak_window(Eigen::Vector3d::UnitX(), Eigen::Vector3d(0.4, 0.0, 0.0));
    rd[20] = weak_window(Eigen::Vector3d::UnitY(), Eigen::Vector3d(0.4, 0.0, 0.0));
    rd[22] = weak_window(Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
    rd[22].at_rest = true;
    for (int w = 26; w < 30; w++)
        rd[w] = weak_window(-Eigen::Vector3d::UnitX(), Eigen::Vector3d(0.2, 0.0, 0.0));
    const std::vector<LoopClosureSegment> segments = segments_of_three(30);
    const LoopOdometryRule rule; // 0.05, 30 deg, 0.05 m/s^2, 0.1 m, 0.5 deg
    const LoopOdometryInformation out = loop_odometry_step_information(all_read(rd), segments, rule);
    REQUIRE(out.information.size() == 9);
    REQUIRE(out.runs.size() == 5);

    const LoopOdometryRun &a = out.runs[0];
    CHECK(a.first_window == 4);
    CHECK(a.last_window == 9);
    CHECK(a.weak_windows == 5);
    CHECK(a.duration_s == doctest::Approx(2.4));
    CHECK(a.axis.isApprox(Eigen::Vector3d::UnitX()));
    CHECK(a.pace_before_mps == doctest::Approx(1.0));
    CHECK(a.pace_after_mps == doctest::Approx(1.0));
    CHECK(a.s1_m == doctest::Approx(0.5 * 0.05 * 2.4 * 2.4));
    CHECK(a.s2_m == doctest::Approx(5 * 0.75 * 0.4)); // the bridged window walks at the pace and adds nothing
    CHECK(a.sigma_m == doctest::Approx(std::hypot(a.s1_m, a.s2_m)));
    REQUIRE(a.step_shares.size() == 3);
    CHECK(a.step_shares[0].first == 1);
    CHECK(a.step_shares[0].second == doctest::Approx(2.0 / 6.0));
    CHECK(a.step_shares[1].second == doctest::Approx(3.0 / 6.0));
    CHECK(a.step_shares[2].second == doctest::Approx(1.0 / 6.0));

    const LoopOdometryRun &b = out.runs[1];
    CHECK(b.first_window == 13);
    CHECK(b.last_window == 13);
    CHECK(b.axis.isApprox(Eigen::Vector3d::UnitY()));
    CHECK(b.s1_m == doctest::Approx(0.5 * 0.05 * 0.4 * 0.4));
    CHECK(b.s2_m == doctest::Approx(0.75 * 0.4));
    REQUIRE(b.step_shares.size() == 1);
    CHECK(b.step_shares[0].first == 4);
    CHECK(b.step_shares[0].second == doctest::Approx(1.0));

    CHECK(out.runs[2].first_window == 19);
    CHECK(out.runs[2].last_window == 19);
    CHECK(out.runs[3].first_window == 20);
    CHECK(out.runs[3].last_window == 20);

    const LoopOdometryRun &e = out.runs[4];
    CHECK(e.first_window == 26);
    CHECK(e.last_window == 29);
    CHECK(e.axis.isApprox(Eigen::Vector3d::UnitX())); // turned to the placed travel
    CHECK(e.pace_after_mps == doctest::Approx(1.0)); // no window after it: the pace before
    CHECK(e.s2_m == doctest::Approx(4 * 0.5 * 0.4));
    REQUIRE(e.step_shares.size() == 1);
    CHECK(e.step_shares[0].first == 8);
    CHECK(e.step_shares[0].second == doctest::Approx(0.25));
    CHECK(e.share_after_last_step == doctest::Approx(0.75));

    // the shares of a run sum to 1, its added variance over the steps to sigma^2 (less the share no step covers)
    double expected = 0.0;
    for (const LoopOdometryRun &r : out.runs)
    {
        double shares = r.share_after_last_step;
        for (const auto &[k, share] : r.step_shares)
            shares += share;
        CHECK(shares == doctest::Approx(1.0));
        expected += r.sigma_m * r.sigma_m * (1.0 - r.share_after_last_step);
    }
    double added = 0.0;
    const double base = rule.sigma_m * rule.sigma_m;
    const double w_r = 1.0 / std::pow(rule.sigma_deg * deg, 2);
    for (size_t k = 0; k < out.information.size(); k++)
    {
        const Eigen::Matrix<double, 6, 6> &omega = out.information[k];
        CHECK(omega.topRightCorner<3, 3>().isZero());
        CHECK(omega.bottomRightCorner<3, 3>().isApprox(Eigen::Matrix3d::Identity() * w_r));
        const Eigen::Matrix3d extra = Eigen::Matrix3d(omega.topLeftCorner<3, 3>().inverse()) - Eigen::Matrix3d::Identity() * base;
        added += extra.trace();
        if (k == 0 || k == 5 || k == 7)
            CHECK(extra.norm() < 1e-12); // no run in segments 0, 5 (windows 15-17) and 7 (21-23)
    }
    CHECK(added == doctest::Approx(expected).epsilon(1e-9));

    // along the run's axis in the step's frame: step 2 (segment 2, turned by 60 deg) gets half of run a's variance along
    // the world's x, which is (cos 60, -sin 60, 0) in its frame
    const Eigen::Matrix3d extra2 = Eigen::Matrix3d(out.information[2].topLeftCorner<3, 3>().inverse()) - Eigen::Matrix3d::Identity() * base;
    const Eigen::Vector3d along(std::cos(60.0 * deg), -std::sin(60.0 * deg), 0.0);
    CHECK((extra2 - a.sigma_m * a.sigma_m * 0.5 * along * along.transpose()).norm() < 1e-9);
}

TEST_CASE("loop closure odometry steps: windows without readings are not read, the rule applies to the others")
{
    // 30 windows, 10 segments: windows 4-9 weak and stalled (window 6 seen, bridged), window 19 weak; windows 20-29 not solved
    std::vector<WindowReadings> full(30, seen_window());
    for (int w : {4, 5, 7, 8, 9, 19})
        full[w] = weak_window(Eigen::Vector3d::UnitX(), Eigen::Vector3d(0.1, 0.0, 0.0));
    const std::vector<LoopClosureSegment> segments = segments_of_three(30);
    const LoopOdometryRule rule;
    std::vector<std::optional<WindowReadings>> first_part = all_read(full);
    for (int w = 20; w < 30; w++)
        first_part[w].reset();
    const LoopOdometryInformation all = loop_odometry_step_information(all_read(full), segments, rule);
    const LoopOdometryInformation part = loop_odometry_step_information(first_part, segments, rule);
    REQUIRE(part.runs.size() == 2);
    // the run in the solved part is the one the full readings give
    CHECK(part.runs[0].first_window == 4);
    CHECK(part.runs[0].last_window == 9);
    CHECK(part.runs[0].sigma_m == all.runs[0].sigma_m);
    CHECK(part.information[1] == all.information[1]);
    // window 19: nothing after it to bridge to or take the pace from, the pace before stands for both
    CHECK(part.runs[1].first_window == 19);
    CHECK(part.runs[1].last_window == 19);
    CHECK(part.runs[1].pace_after_mps == doctest::Approx(1.0));
    // the steps of the unsolved windows keep the fixed sigmas
    const double wt = 1.0 / (rule.sigma_m * rule.sigma_m);
    for (int k = 7; k < 9; k++)
        CHECK(part.information[k].topLeftCorner<3, 3>().isApprox(Eigen::Matrix3d::Identity() * wt));

    // readings missing in the middle: the weak windows before and after them are two runs, not one bridged over the gap
    std::vector<WindowReadings> rd(30, seen_window());
    for (int w : {10, 11, 13, 14})
        rd[w] = weak_window(Eigen::Vector3d::UnitX(), Eigen::Vector3d(0.1, 0.0, 0.0));
    std::vector<std::optional<WindowReadings>> gap = all_read(rd);
    CHECK(loop_odometry_step_information(gap, segments, rule).runs.size() == 1); // window 12 seen: bridged
    gap[12].reset();
    const LoopOdometryInformation split = loop_odometry_step_information(gap, segments, rule);
    REQUIRE(split.runs.size() == 2);
    CHECK(split.runs[0].last_window == 11);
    CHECK(split.runs[1].first_window == 13);
}

TEST_CASE("loop closure odometry steps: the pace line runs from the pace before to the pace after, by the windows' mid times")
{
    // windows 3-4 weak and stalled; before 0.5 m/s, after 1.5 m/s: v_ref 0.75 and 1.25 at the two mid times
    std::vector<WindowReadings> rd(9, seen_window());
    for (int w = 0; w < 3; w++)
        rd[w].placed_move = Eigen::Vector3d(0.2, 0.0, 0.0);
    for (int w = 5; w < 9; w++)
        rd[w].placed_move = Eigen::Vector3d(0.0, 0.6, 0.0); // a turn after the run: the pace ignores the direction
    rd[3] = weak_window(Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
    rd[4] = weak_window(Eigen::Vector3d::UnitX(), Eigen::Vector3d(0.04, 0.0, 0.0));
    const LoopOdometryInformation out = loop_odometry_step_information(all_read(rd), segments_of_three(9), LoopOdometryRule{});
    REQUIRE(out.runs.size() == 1);
    const LoopOdometryRun &r = out.runs[0];
    CHECK(r.pace_before_mps == doctest::Approx(0.5));
    CHECK(r.pace_after_mps == doctest::Approx(1.5));
    CHECK(r.s2_m == doctest::Approx((0.75 - 0.0) * 0.4 + (1.25 - 0.1) * 0.4));
}

TEST_CASE("loop closure odometry steps: the fixed sigmas as information give the fixed-sigma solution")
{
    const TwoLaps s;
    const OdometryConfig c = with_kernel(0.0);
    const auto loops = s.loops();
    const auto reference = optimize_loop_pose_graph(s.initial, loops, c);
    std::vector<WindowReadings> rd(3 * s.initial.size(), seen_window()); // no weak window: every step at the fixed sigmas
    std::vector<LoopClosureSegment> segments = segments_of_three(static_cast<int>(rd.size()));
    for (size_t k = 0; k < segments.size(); k++)
        segments[k].pose = s.initial[k];
    const LoopOdometryInformation steps = loop_odometry_step_information(all_read(rd), segments, loop_odometry_rule(c));
    REQUIRE(steps.runs.empty());
    REQUIRE(steps.information.size() + 1 == s.initial.size());
    const auto solved = optimize_loop_pose_graph(s.initial, loops, c, &steps.information);
    REQUIRE(solved.size() == reference.size());
    double largest = 0.0;
    for (size_t k = 0; k < solved.size(); k++)
        largest = std::max(largest, (solved[k].matrix() - reference[k].matrix()).cwiseAbs().maxCoeff());
    MESSAGE("largest difference ", largest);
    CHECK(largest < 1e-9);
    CHECK(loop_pose_graph_cost(s.initial, solved, loops, c, &steps.information) ==
          doctest::Approx(loop_pose_graph_cost(s.initial, reference, loops, c)).epsilon(1e-9));
}

namespace
{
    // out along x and back 2 m to the side: segments 0..n-1 at x = k, segments n..2n-1 at x = 2n-1-k, y = 2, turned by 180 deg.
    // stall: the odometry placed steps from..to-1 of the way out `short_m` shorter each (a blind stretch)
    struct OutAndBack
    {
        std::vector<Eigen::Affine3d> truth, initial;
        OutAndBack(int n, int stall_from = 0, int stall_to = 0, double short_m = 0.0)
        {
            double lag = 0.0;
            for (int k = 0; k < 2 * n; k++)
            {
                const bool out = k < n;
                const double x = out ? k : 2 * n - 1 - k;
                truth.push_back(segment_pose(x, out ? 0.0 : 2.0, out ? 0.0 : 180.0));
                if (k > stall_from && k <= stall_to)
                    lag += short_m;
                Eigen::Affine3d p = truth.back();
                p.translation().x() -= lag; // every pose after the stall stays behind
                initial.push_back(p);
            }
        }
        // the revisit of segment i from segment j as measured, its translation off by `error` [m] along x
        LoopClosureEdge edge(int i, int j, double error = 0.0, double sigma_m = 0.05) const
        {
            LoopClosureEdge e;
            e.i = i;
            e.j = j;
            e.correction = (initial[i] * truth[i].inverse() * truth[j]) * initial[j].inverse();
            e.correction.translation().x() += error;
            e.information = true;
            e.omega.setZero();
            const double wt = 1.0 / (sigma_m * sigma_m), wr = 1.0 / std::pow(0.3 * DEG_TO_RAD, 2);
            e.omega.diagonal() << wt, wt, wt, wr, wr, wr;
            return e;
        }
        // one window per segment, 1 s each, walking at 1 m/s; the windows of segments from..to-1 weak along x, placed as the
        // odometry placed them
        std::vector<LoopClosureSegment> segments() const
        {
            std::vector<LoopClosureSegment> out;
            for (size_t k = 0; k < initial.size(); k++)
            {
                LoopClosureSegment s;
                s.first_window = s.last_window = static_cast<int>(k);
                s.pose = initial[k];
                out.push_back(s);
            }
            return out;
        }
        std::vector<WindowReadings> readings(int weak_from, int weak_to) const
        {
            std::vector<WindowReadings> rd;
            for (size_t k = 0; k < initial.size(); k++)
            {
                const Eigen::Vector3d move = k + 1 < initial.size() ? Eigen::Vector3d(initial[k + 1].translation() - initial[k].translation())
                                                                     : Eigen::Vector3d(-1.0, 0.0, 0.0);
                const bool weak = static_cast<int>(k) >= weak_from && static_cast<int>(k) < weak_to;
                rd.push_back(WindowReadings{weak ? 0.01 : 0.5, Eigen::Vector3d::UnitX(), false, move, 1.0});
            }
            return rd;
        }
    };

    LoopOdometryEdgeInformation uniform_steps(size_t segments, const OdometryConfig &c)
    {
        Eigen::Matrix<double, 6, 6> omega = Eigen::Matrix<double, 6, 6>::Zero();
        const double wt = 1.0 / std::pow(c.loop_closure_odometry_sigma_m, 2), wr = 1.0 / std::pow(c.loop_closure_odometry_sigma_deg * DEG_TO_RAD, 2);
        omega.diagonal() << wt, wt, wt, wr, wr, wr;
        return LoopOdometryEdgeInformation(segments - 1, omega);
    }
}

TEST_CASE("loop closure match check: the chi-square quantile")
{
    const double expected[6] = {10.828, 13.816, 16.266, 18.467, 20.515, 22.458};
    for (int dof = 1; dof <= 6; dof++)
        CHECK(chi_square_quantile(0.999, dof) == doctest::Approx(expected[dof - 1]).epsilon(5e-5));
    CHECK(chi_square_quantile(0.5, 2) == doctest::Approx(2.0 * std::log(2.0)));
    CHECK(chi_square_quantile(0.95, 1) == doctest::Approx(3.841459).epsilon(1e-6));
    CHECK(std::isnan(chi_square_quantile(1.0, 6)));
    CHECK(std::isnan(chi_square_quantile(0.999, 0)));
}

TEST_CASE("loop closure match check: edges share scans in either role, the degrees of freedom are the information's rank")
{
    LoopClosureEdge e;
    e.i = 10;
    e.j = 40;
    const auto with = [](int i, int j)
    {
        LoopClosureEdge f;
        f.i = i;
        f.j = j;
        return f;
    };
    CHECK(loop_edges_share_scans(e, with(20, 40))); // the same source
    CHECK(loop_edges_share_scans(e, with(12, 30))); // a target within 2 segments
    CHECK(loop_edges_share_scans(e, with(25, 11))); // its source within 1 segment of e's target
    CHECK(loop_edges_share_scans(e, with(39, 60))); // e's source within 1 segment of its target
    CHECK_FALSE(loop_edges_share_scans(e, with(13, 30)));
    CHECK_FALSE(loop_edges_share_scans(e, with(38, 60)));
    CHECK(loop_edge_dof(e) == 6); // the fixed sigmas
    e.information = true;
    e.omega = Eigen::Matrix<double, 6, 6>::Identity() * 400.0;
    CHECK(loop_edge_dof(e) == 6);
    e.omega(2, 2) = 0.0; // a cross-floor edge: no height
    CHECK(loop_edge_dof(e) == 5);
}

TEST_CASE("loop closure match check: an influential false edge inside the pruning limit is dropped, the true ones stay")
{
    // true revisits at segments 3, 13, 23, 33 (none shares scans with another); a false one at 8, 2 m off and claimed at 1 cm
    const OutAndBack g(40);
    std::vector<LoopClosureEdge> loops;
    for (int i : {3, 13, 23, 33})
        loops.push_back(g.edge(i, 79 - i));
    loops.push_back(g.edge(8, 71, 2.0, 0.01));
    OdometryConfig c = with_kernel(0.0);
    const LoopOdometryEdgeInformation steps = uniform_steps(g.initial.size(), c);
    NullLogger logger;

    // the pruning alone keeps it: it bends the graph towards itself
    {
        std::vector<LoopClosureEdge> kept = loops;
        LoopClosureReport rep;
        std::vector<Eigen::Affine3d> optimized;
        std::vector<LoopEdgeResidual> residuals;
        REQUIRE(solve_and_prune_loop_pose_graph(g.initial, kept, c, &steps, logger, rep, optimized, residuals));
        REQUIRE(kept.size() == 5);
        MESSAGE("false edge weighted residual ", residuals[4].weighted * c.loop_closure_edge_sigma_m, " m");
        CHECK(loop_edge_drop_ratio(kept[4], residuals[4], c) < 1.0);
    }

    c.loop_closure_match_check = true;
    std::vector<LoopClosureEdge> kept = loops;
    LoopClosureReport rep;
    std::vector<Eigen::Affine3d> optimized;
    std::vector<LoopEdgeResidual> residuals;
    REQUIRE(solve_and_prune_loop_pose_graph(g.initial, kept, c, &steps, logger, rep, optimized, residuals));
    REQUIRE(rep.dropped.size() == 1);
    CHECK(rep.dropped[0].i == 8);
    REQUIRE(rep.dropped_match_tests[0].has_value());
    const LoopMatchTest &t = *rep.dropped_match_tests[0];
    MESSAGE("T ", t.t, " limit ", t.limit, " innovation ", t.innovation_m, " m");
    CHECK(t.t > t.limit);
    CHECK(t.dof == 6);
    CHECK(t.held_out == 0);
    CHECK(t.innovation_m == doctest::Approx(2.0).epsilon(0.05));
    CHECK(kept.size() == 4);
    REQUIRE(rep.kept_match_tests.size() == 4);
    for (const LoopMatchTest &k : rep.kept_match_tests)
        CHECK(k.t < 1.0); // consistent with each other
    double largest = 0.0;
    for (size_t k = 0; k < optimized.size(); k++)
        largest = std::max(largest, (optimized[k].translation() - g.truth[k].translation()).norm());
    CHECK(largest < 1e-6);
}

TEST_CASE("loop closure match check: edges across a blind stretch are kept with the readings' steps, rated false on uniform steps")
{
    // the way out stalled in segments 5..9 (0.8 m short per step, 4 m in all); revisits at 3 and 12 on either side
    const OutAndBack g(20, 4, 9, 0.8);
    const std::vector<LoopClosureEdge> loops{g.edge(3, 36), g.edge(12, 27)};
    const OdometryConfig c = with_kernel(0.0);

    const LoopOdometryInformation rule = loop_odometry_step_information(all_read(g.readings(4, 9)), g.segments(), loop_odometry_rule(c));
    REQUIRE(rule.runs.size() == 1);
    MESSAGE("run sigma ", rule.runs[0].sigma_m, " m");
    for (const LoopMatchTest &t : loop_match_tests(g.initial, loops, c, &rule.information))
    {
        MESSAGE("rule: T ", t.t, " limit ", t.limit);
        CHECK(t.t < t.limit);
    }
    const LoopOdometryEdgeInformation uniform = uniform_steps(g.initial.size(), c);
    for (const LoopMatchTest &t : loop_match_tests(g.initial, loops, c, &uniform))
    {
        MESSAGE("uniform: T ", t.t, " limit ", t.limit);
        CHECK(t.t > t.limit);
    }
}

TEST_CASE("loop closure match check: an edge without a degree of freedom is not tested and doesn't decide by its place")
{
    // the influential false edge of the case above, with an edge that carries no information first in the list
    const OutAndBack g(40);
    LoopClosureEdge empty = g.edge(18, 61);
    empty.omega.setZero();
    std::vector<LoopClosureEdge> loops{empty};
    for (int i : {3, 13, 23, 33})
        loops.push_back(g.edge(i, 79 - i));
    loops.push_back(g.edge(8, 71, 2.0, 0.01));
    OdometryConfig c = with_kernel(0.0);
    c.loop_closure_match_check = true;
    const LoopOdometryEdgeInformation steps = uniform_steps(g.initial.size(), c);
    NullLogger logger;
    LoopClosureReport rep;
    std::vector<Eigen::Affine3d> optimized;
    std::vector<LoopEdgeResidual> residuals;
    REQUIRE(solve_and_prune_loop_pose_graph(g.initial, loops, c, &steps, logger, rep, optimized, residuals));
    REQUIRE(rep.dropped.size() == 1);
    CHECK(rep.dropped[0].i == 8);
    REQUIRE(loops.size() == 5);
    CHECK(loops[0].i == 18);
    REQUIRE(rep.kept_match_tests.size() == 5);
    CHECK(rep.kept_match_tests[0].dof == 0);
    for (size_t k = 1; k < 5; k++)
        CHECK(rep.kept_match_tests[k].dof == 6);
}

TEST_CASE("loop closure match check: an edge no other loop reaches is kept")
{
    // the only revisit, 1 m off after 76 steps of odometry: the rest of the graph predicts it only loosely
    const OutAndBack g(40);
    std::vector<LoopClosureEdge> loops{g.edge(1, 78, 1.0)};
    OdometryConfig c = with_kernel(0.0);
    c.loop_closure_match_check = true;
    c.loop_closure_max_residual_m = 10.0; // the pruning out of the way
    const LoopOdometryEdgeInformation steps = uniform_steps(g.initial.size(), c);
    NullLogger logger;
    LoopClosureReport rep;
    std::vector<Eigen::Affine3d> optimized;
    std::vector<LoopEdgeResidual> residuals;
    REQUIRE(solve_and_prune_loop_pose_graph(g.initial, loops, c, &steps, logger, rep, optimized, residuals));
    CHECK(loops.size() == 1);
    REQUIRE(rep.kept_match_tests.size() == 1);
    MESSAGE("T ", rep.kept_match_tests[0].t, " limit ", rep.kept_match_tests[0].limit);
    CHECK(rep.kept_match_tests[0].held_out == 0);
    CHECK(rep.kept_match_tests[0].t < rep.kept_match_tests[0].limit);
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

// ---- loop closure capture and replay (change faster-experiments, tasks 1.1 to 1.5) ----
namespace
{
    struct MemoryLoopClosureCaptureSink : LoopClosureCaptureSink
    {
        LoopClosureCapture capture;
        int writes = 0;
        bool write(const LoopClosureCapture &c, std::string &) override
        {
            capture = c;
            writes++;
            return true;
        }
    };

    // a capture of the loop recording's windows without any registration
    LoopClosureCapture capture_without_tries(const OdometryConfig &config)
    {
        LoopClosureCapture c;
        c.windows = loop_recording(2).windows;
        c.segments = loop_closure_segments(c.windows, config);
        for (const auto &w : c.windows)
            c.result_poses.push_back(w.poses);
        return c;
    }

    bool same_bits(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b)
    {
        if (a.size() != b.size())
            return false;
        for (size_t k = 0; k < a.size(); k++)
            if (a[k].matrix() != b[k].matrix())
                return false;
        return true;
    }

    bool contains(const std::vector<std::string> &names, const std::string &name)
    {
        return std::find(names.begin(), names.end(), name) != names.end();
    }

    // a ResultSink that keeps the trajectory rows and chunk poses (save_result's other output is dropped)
    struct TrajectorySink : ResultSink
    {
        std::vector<std::vector<TrajectoryRow>> rows;
        std::vector<Eigen::Affine3d> first_poses;
        bool begin() override { return true; }
        void point_sizes(const std::vector<int> &) override {}
        bool open_merged_cloud() override { return true; }
        void chunk_cloud(std::size_t, const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &, const std::vector<double> &) override {}
        void merged_cloud(const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &, const std::vector<double> &) override {}
        bool chunk_trajectory(std::size_t, const std::vector<TrajectoryRow> &r) override
        {
            rows.push_back(r);
            return true;
        }
        void close_merged_cloud() override {}
        void chunk_poses(const std::vector<Eigen::Affine3d> &p) override { first_poses = p; }
        void index_poses(const std::vector<std::vector<int>> &) override {}
        void session(const SessionSummary &) override {}
        void parameters(const OdometryConfig &, const RunPaths &, const GuiPreferences &) override {}
        void processing_results(const OdometryConfig &, const OdometryState &, const std::vector<Window> &, double) override {}
    };
}

TEST_CASE("loop closure replay settings: odometry and registration settings are refused by name, rules are changes, diagnostics don't count")
{
    const OdometryConfig captured = loop_closure_config();
    const auto text = settings_as_text(captured);

    const LoopClosureSettingsDifference same = loop_closure_replay_settings(text, captured);
    CHECK(same.refused.empty());
    CHECK(same.changed.empty());

    OdometryConfig c = captured;
    c.loop_closure_min_fitness = 0.6;
    c.loop_closure_max_residual_m = 0.1;
    c.capture_loop_closure = true; // diagnostics
    c.capture_loop_closure_max_tries = 5;
    c.num_constistency_iter = 3; // consistency
    const LoopClosureSettingsDifference rules = loop_closure_replay_settings(text, c);
    CHECK(rules.refused.empty());
    CHECK(rules.changed == std::vector<std::string>{"loop_closure_max_residual_m", "loop_closure_min_fitness"});

    c.loop_closure_segment_windows = 5;
    c.loop_closure_max_correction_m = 4.0;
    c.loop_closure_cross_floor_search_m = 2.0;
    c.max_distance_lidar = 20.0; // an odometry setting (it also changes the segments' points)
    const LoopClosureSettingsDifference refused = loop_closure_replay_settings(text, c);
    CHECK(contains(refused.refused, "loop_closure_segment_windows"));
    CHECK(contains(refused.refused, "loop_closure_max_correction_m"));
    CHECK(contains(refused.refused, "loop_closure_cross_floor_search_m"));
    CHECK(contains(refused.refused, "max_distance_lidar"));
    CHECK(contains(refused.refused, "odometry_settings_hash"));
    CHECK(!contains(refused.refused, "loop_closure_min_fitness"));

    // a setting the capture doesn't list counts as at its default
    auto older = text;
    older.erase("loop_closure_kernel_scale");
    CHECK(loop_closure_replay_settings(older, captured).changed.empty());
    OdometryConfig kernel = captured;
    kernel.loop_closure_kernel_scale = 3.0;
    CHECK(loop_closure_replay_settings(older, kernel).changed == std::vector<std::string>{"loop_closure_kernel_scale"});
}

TEST_CASE("loop closure replay: the tries the settings ask for and the capture doesn't hold are named, and nothing is applied")
{
    const OdometryConfig config = loop_closure_config();
    const LoopClosureCapture capture = capture_without_tries(config);
    NullLogger logger;
    const LoopClosureReplayResult r = replay_loop_closure(capture, config, logger);
    CHECK(r.segments_match);
    REQUIRE(!r.missing.empty());
    MESSAGE("missing: ", r.missing.size(), ", first ", r.missing.front());
    CHECK(r.missing.front().rfind("segments ", 0) == 0);
    CHECK(r.loops == 0);
    CHECK(r.report.outcome == "no_loops");
    CHECK(r.identical_nodes == r.nodes);

    // other segment settings than the capture's: the segments differ and nothing runs
    OdometryConfig other = config;
    other.loop_closure_segment_windows = 5;
    CHECK(!replay_loop_closure(capture, other, logger).segments_match);
}

TEST_CASE("loop closure: readings for the first windows only, the loop closure runs and names the windows without them")
{
    OdometryConfig config = loop_closure_config();
    LoopClosureCapture capture = capture_without_tries(config);
    const int n = static_cast<int>(capture.windows.size());
    for (int w = 0; w < n; w++)
        if (w < n / 2)
            capture.windows[w].readings = WindowReadings{0.5, Eigen::Vector3d::UnitX(), false, Eigen::Vector3d(0.4, 0.0, 0.0), 0.4};
        else
            capture.windows[w].readings.reset();
    config.loop_closure_odometry_from_readings = true;
    config.loop_closure_match_check = true;
    WarningLogger logger;
    const LoopClosureReplayResult r = replay_loop_closure(capture, config, logger);
    CHECK(r.report.outcome == "no_loops"); // it ran (and asked for the registrations the capture doesn't hold)
    CHECK(!r.missing.empty());
    CHECK(r.report.windows_without_readings == n - n / 2);
    REQUIRE(logger.warnings.size() == 1);
    MESSAGE(logger.warnings[0]);
    CHECK(logger.warnings[0].find("windows " + std::to_string(n / 2) + "-" + std::to_string(n - 1)) != std::string::npos);

    // none with readings: refused as before
    for (Window &w : capture.windows)
        w.readings.reset();
    CHECK(replay_loop_closure(capture, config, logger).report.outcome == "readings_missing");
}

TEST_CASE("loop closure replay: the match check without the readings' steps is refused, nothing is registered or applied")
{
    OdometryConfig config = loop_closure_config();
    LoopClosureCapture capture = capture_without_tries(config);
    for (Window &w : capture.windows)
        w.readings = WindowReadings{0.5, Eigen::Vector3d::UnitX(), false, Eigen::Vector3d(0.4, 0.0, 0.0), 0.4};
    config.loop_closure_match_check = true;
    NullLogger logger;
    const LoopClosureReplayResult r = replay_loop_closure(capture, config, logger);
    CHECK(r.report.outcome == "match_check_without_readings");
    CHECK(r.loops == 0);
    CHECK(r.missing.empty());
    CHECK(r.identical_nodes == r.nodes);
    config.loop_closure_odometry_from_readings = true;
    CHECK(replay_loop_closure(capture, config, logger).report.outcome == "no_loops");
}

TEST_CASE("loop closure match check: a quantile outside (0, 1) or the robust kernel is refused, nothing is registered or applied")
{
    OdometryConfig config = loop_closure_config();
    LoopClosureCapture capture = capture_without_tries(config);
    for (Window &w : capture.windows)
        w.readings = WindowReadings{0.5, Eigen::Vector3d::UnitX(), false, Eigen::Vector3d(0.4, 0.0, 0.0), 0.4};
    config.loop_closure_odometry_from_readings = true;
    config.loop_closure_match_check = true;
    NullLogger logger;
    for (double q : {0.0, 1.0, -0.5, 1.5, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
        OdometryConfig c = config;
        c.loop_closure_match_check_quantile = q;
        const LoopClosureReplayResult r = replay_loop_closure(capture, c, logger);
        CHECK(r.report.outcome == "match_check_bad_quantile");
        CHECK(r.missing.empty()); // nothing was asked for
        CHECK(r.identical_nodes == r.nodes);
    }
    OdometryConfig kernel = config;
    kernel.loop_closure_kernel_scale = 3.0;
    const LoopClosureReplayResult k = replay_loop_closure(capture, kernel, logger);
    CHECK(k.report.outcome == "match_check_with_kernel");
    CHECK(k.missing.empty());
    CHECK(k.identical_nodes == k.nodes);
    kernel.loop_closure_match_check = false; // the kernel alone runs
    CHECK(replay_loop_closure(capture, kernel, logger).report.outcome == "no_loops");

    // the pose-graph step refuses them too
    const OutAndBack g(10);
    std::vector<LoopClosureEdge> loops{g.edge(1, 18)};
    OdometryConfig c = with_kernel(0.0);
    c.loop_closure_match_check = true;
    c.loop_closure_match_check_quantile = 1.0;
    const LoopOdometryEdgeInformation steps = uniform_steps(g.initial.size(), c);
    LoopClosureReport rep;
    std::vector<Eigen::Affine3d> optimized;
    std::vector<LoopEdgeResidual> residuals;
    CHECK_FALSE(solve_and_prune_loop_pose_graph(g.initial, loops, c, &steps, logger, rep, optimized, residuals));
    CHECK(rep.outcome == "match_check_bad_quantile");
    c = with_kernel(0.5);
    c.loop_closure_match_check = true;
    rep = LoopClosureReport{};
    CHECK_FALSE(solve_and_prune_loop_pose_graph(g.initial, loops, c, &steps, logger, rep, optimized, residuals));
    CHECK(rep.outcome == "match_check_with_kernel");
}

TEST_CASE("loop closure capture file: written and read back bit for bit; another file is refused")
{
    const OdometryConfig config = loop_closure_config();
    LoopClosureCapture c = capture_without_tries(config);
    CapturedLoopTry t;
    t.registration.i = 2;
    t.registration.j = 9;
    t.registration.cross_floor = true;
    t.registration.registered = true;
    t.registration.overlap_before = 0.123456789;
    t.registration.overlap_after = 0.7;
    t.registration.correction.translate(Eigen::Vector3d(0.1, -0.2, 0.3));
    t.registration.correction.rotate(Eigen::AngleAxisd(0.01, Eigen::Vector3d(1, 2, 3).normalized()));
    for (int k = 0; k < 36; k++)
        t.registration.information.data()[k] = 1.0 / (k + 3.0);
    t.registration.shift_m = 0.37;
    t.registration.angle_deg = 0.57;
    t.registration.information_ratio = 0.31;
    t.registration.centre = Eigen::Vector3d(1.0 / 3.0, 2.0, -7.0);
    t.accepted = true;
    t.requested_by_run = false;
    c.tries = {t, CapturedLoopTry{}};
    c.result_poses.back().back().translation().x() += 1e-3;
    c.loops = 3;
    for (size_t w = 0; w < c.windows.size(); w++)
    {
        WindowReadings r;
        r.information_ratio = 1.0 / (w + 3.0);
        r.weak_axis = Eigen::Vector3d(1.0, static_cast<double>(w), -2.0).normalized();
        r.at_rest = w % 3 == 0;
        r.placed_move = Eigen::Vector3d(0.1 * w, 1.0 / 7.0, -0.3);
        r.duration_s = 0.4 + 1e-3 * w;
        c.windows[w].readings = r;
    }
    c.windows[1].readings.reset(); // a window without readings

    const auto file = std::filesystem::temp_directory_path() / "lidar_odometry_tests_loop_closure_capture.bin";
    std::string why;
    REQUIRE(write_loop_closure_capture(file, c, config, why));
    LoopClosureCapture back;
    std::map<std::string, std::string> settings;
    REQUIRE(read_loop_closure_capture(file, back, settings, why));
    CHECK(settings == settings_as_text(config));
    REQUIRE(back.windows.size() == c.windows.size());
    for (size_t w = 0; w < c.windows.size(); w++)
        CHECK(same_bits(back.windows[w].poses, c.windows[w].poses));
    REQUIRE(back.segments.size() == c.segments.size());
    for (size_t k = 0; k < c.segments.size(); k++)
    {
        CHECK(back.segments[k].pose.matrix() == c.segments[k].pose.matrix());
        CHECK(back.segments[k].center == c.segments[k].center);
        CHECK(back.segments[k].t_end == c.segments[k].t_end);
        CHECK(back.segments[k].path_end == c.segments[k].path_end);
        CHECK(back.segments[k].last_window == c.segments[k].last_window);
    }
    REQUIRE(back.tries.size() == 2);
    const LoopClosureTry &b = back.tries[0].registration;
    CHECK(b.i == 2);
    CHECK(b.j == 9);
    CHECK(b.cross_floor);
    CHECK(b.registered);
    CHECK(b.overlap_before == t.registration.overlap_before);
    CHECK(b.correction.matrix() == t.registration.correction.matrix());
    CHECK(b.information == t.registration.information);
    CHECK(b.information_ratio == t.registration.information_ratio);
    CHECK(b.centre == t.registration.centre);
    CHECK(back.tries[0].accepted);
    CHECK(!back.tries[0].requested_by_run);
    CHECK(!back.tries[1].registration.registered);
    REQUIRE(back.result_poses.size() == c.result_poses.size());
    CHECK(same_bits(back.result_poses.back(), c.result_poses.back()));
    CHECK(back.loops == 3);
    size_t readings_bytes = 8;
    for (size_t w = 0; w < c.windows.size(); w++)
    {
        const auto &a = c.windows[w].readings;
        const auto &r = back.windows[w].readings;
        readings_bytes += 1 + (a ? 8 + 24 + 1 + 24 + 8 : 0);
        REQUIRE(a.has_value() == r.has_value());
        if (!a)
            continue;
        CHECK(r->information_ratio == a->information_ratio);
        CHECK(r->weak_axis == a->weak_axis);
        CHECK(r->at_rest == a->at_rest);
        CHECK(r->placed_move == a->placed_move);
        CHECK(r->duration_s == a->duration_s);
    }

    // format 1 (written before the readings: format 2 without the readings block at the end) is read without readings
    {
        std::string bytes;
        {
            std::ifstream in(file, std::ios::binary);
            bytes.assign(std::istreambuf_iterator<char>(in), {});
        }
        REQUIRE(bytes.size() > readings_bytes + 12);
        bytes.resize(bytes.size() - readings_bytes);
        const std::uint32_t version = 1;
        std::memcpy(&bytes[8], &version, sizeof(version));
        std::ofstream out(file, std::ios::binary | std::ios::trunc);
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    }
    REQUIRE(read_loop_closure_capture(file, back, settings, why));
    CHECK(back.loops == 3);
    for (const Window &w : back.windows)
        CHECK_FALSE(w.readings.has_value());

    // another file (here the start of a window capture) is not a loop closure capture
    {
        std::ofstream out(file, std::ios::binary | std::ios::trunc);
        out << "DMCAPT01 something else";
    }
    CHECK(!read_loop_closure_capture(file, back, settings, why));
    std::filesystem::remove(file);
}

TEST_CASE("loop closure replay: the trajectory is cut into output chunks as save_result cuts it")
{
    const OdometryConfig config = loop_closure_config();
    std::vector<Window> windows;
    for (int copy = 0; copy < 3; copy++) // 132 windows: chunks of 51, 51 and 30
    {
        const auto r = loop_recording(2);
        windows.insert(windows.end(), r.windows.begin(), r.windows.end());
    }
    for (Window &w : windows) // an orientation per node, as the odometry gives them
        for (size_t n = w.imu_angles.size(); n < w.poses.size(); n++)
            w.imu_angles.push_back(Eigen::Vector3d(0.001 * n, -0.002, 0.5));
    TrajectorySink sink;
    MemoryPointStore store;
    NullLogger logger;
    save_result(windows, config, OdometryState{}, GuiPreferences{}, 0.0, sink, store, logger);
    const std::vector<TrajectoryChunk> chunks = trajectory_chunks(windows, config);
    REQUIRE(chunks.size() == 3);
    REQUIRE(sink.rows.size() == chunks.size());
    REQUIRE(sink.first_poses.size() == chunks.size());
    for (size_t k = 0; k < chunks.size(); k++)
    {
        CHECK(chunks[k].first_pose.matrix() == sink.first_poses[k].matrix());
        REQUIRE(chunks[k].rows.size() == sink.rows[k].size());
        bool same = true;
        for (size_t n = 0; n < chunks[k].rows.size(); n++)
            same = same && chunks[k].rows[n].pose.matrix() == sink.rows[k][n].pose.matrix() &&
                   chunks[k].rows[n].timestamps == sink.rows[k][n].timestamps && chunks[k].rows[n].om_fi_ka == sink.rows[k][n].om_fi_ka;
        CHECK(same);
    }
}

// The capture of a run on the drifting loop recording, replayed (skipped like the other end-to-end cases: the run registers)
TEST_CASE("loop closure capture: capturing changes nothing, the own settings replay bit for bit, a stricter acceptance re-solves, a try the capture lacks is named" *
          doctest::skip())
{
    Recording r = loop_recording(2);
    MemoryPointStore store;
    store_points(r, store);
    add_second_lap_drift(r, {0.3, 0.4, 0.0}, 2.0 * deg);
    const OdometryConfig config = loop_closure_config();
    NullLogger logger;

    std::vector<Window> plain = r.windows;
    const int loops_plain = run_loop_closure(plain, config, store, logger);
    std::vector<Window> captured_run = r.windows;
    MemoryLoopClosureCaptureSink sink;
    const int loops = run_loop_closure(captured_run, config, store, logger, &sink);
    CHECK(loops == loops_plain);
    CHECK(same_bits(all_poses(captured_run), all_poses(plain)));
    REQUIRE(sink.writes == 1);
    const LoopClosureCapture &capture = sink.capture;
    CHECK(capture.loops == loops);
    CHECK(same_bits(all_poses(capture.windows), all_poses(r.windows)));
    int accepted = 0;
    for (const auto &t : capture.tries)
    {
        CHECK(t.requested_by_run);
        accepted += t.accepted;
    }
    MESSAGE("capture: ", capture.segments.size(), " segments, ", capture.tries.size(), " registrations, ", accepted, " accepted, ", loops, " loops");
    REQUIRE(loops >= 1);

    // the own settings, from memory and through the file
    const auto start = std::chrono::steady_clock::now();
    const LoopClosureReplayResult own = replay_loop_closure(capture, config, logger);
    const double replay_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    MESSAGE("replay with the own settings: ", own.identical_nodes, " of ", own.nodes, " nodes identical, ", replay_ms, " ms");
    CHECK(own.missing.empty());
    CHECK(own.loops == loops);
    CHECK(own.identical_nodes == own.nodes);
    CHECK(same_bits(all_poses(own.windows), all_poses(plain)));
    const auto file = std::filesystem::temp_directory_path() / "lidar_odometry_tests_loop_closure_replay.bin";
    std::string why;
    REQUIRE(write_loop_closure_capture(file, capture, config, why));
    LoopClosureCapture back;
    std::map<std::string, std::string> settings;
    REQUIRE(read_loop_closure_capture(file, back, settings, why));
    std::filesystem::remove(file);
    const LoopClosureSettingsDifference own_difference = loop_closure_replay_settings(settings, config);
    CHECK(own_difference.refused.empty());
    CHECK(own_difference.changed.empty());
    const LoopClosureReplayResult from_file = replay_loop_closure(back, config, logger);
    CHECK(from_file.identical_nodes == from_file.nodes);

    // a stricter acceptance: the median overlap of the accepted registrations
    std::vector<double> fitness;
    for (const auto &t : capture.tries)
        if (t.accepted)
            fitness.push_back(t.registration.overlap_after);
    REQUIRE(!fitness.empty());
    std::sort(fitness.begin(), fitness.end());
    OdometryConfig stricter = config;
    stricter.loop_closure_min_fitness = fitness[fitness.size() / 2];
    CHECK(loop_closure_replay_settings(settings, stricter).changed == std::vector<std::string>{"loop_closure_min_fitness"});
    const LoopClosureReplayResult strict = replay_loop_closure(capture, stricter, logger);
    MESSAGE("min fitness ", stricter.loop_closure_min_fitness, ": ", strict.loops, " loops (", loops, " with ", config.loop_closure_min_fitness,
            "), ", strict.report.dropped.size(), " dropped, nodes identical with the run ", strict.identical_nodes, " of ", strict.nodes);
    CHECK(strict.missing.empty());
    for (const auto &e : strict.report.kept)
        CHECK(e.fitness >= stricter.loop_closure_min_fitness);
    for (const auto &e : strict.report.dropped)
        CHECK(e.fitness >= stricter.loop_closure_min_fitness);

    // a try the run made but the capture lacks (as a changed rule would ask for one never registered): named, not applied
    LoopClosureCapture lacking = capture;
    const LoopClosureTry dropped = lacking.tries.front().registration;
    lacking.tries.erase(lacking.tries.begin());
    const LoopClosureReplayResult missing = replay_loop_closure(lacking, config, logger);
    const std::string name = "segments " + std::to_string(dropped.i) + " -> " + std::to_string(dropped.j);
    MESSAGE("without the try ", name, ": ", missing.missing.size(), " missing, first ", missing.missing.empty() ? std::string() : missing.missing.front());
    CHECK(missing.missing == std::vector<std::string>{name});
}

TEST_CASE("loop closure capture: registrations made only for the capture change nothing and cover the qualifying revisits" * doctest::skip())
{
    Recording r = loop_recording(2);
    MemoryPointStore store;
    store_points(r, store);
    add_second_lap_drift(r, {0.3, 0.4, 0.0}, 2.0 * deg);
    const OdometryConfig config = loop_closure_config();
    NullLogger logger;

    std::vector<Window> plain = r.windows;
    MemoryLoopClosureCaptureSink run_only;
    run_loop_closure(plain, config, store, logger, &run_only);
    std::vector<Window> with_extra = r.windows;
    MemoryLoopClosureCaptureSink sink;
    OdometryConfig capturing = config;
    capturing.capture_loop_closure_max_tries = 8;
    run_loop_closure(with_extra, capturing, store, logger, &sink);
    CHECK(same_bits(all_poses(with_extra), all_poses(plain)));
    int extra = 0;
    for (const auto &t : sink.capture.tries)
        extra += !t.requested_by_run;
    MESSAGE("registrations: ", run_only.capture.tries.size(), " by the run, ", extra, " more for the capture");
    CHECK(extra > 0);
    const LoopClosureReplayResult own = replay_loop_closure(sink.capture, capturing, logger);
    CHECK(own.identical_nodes == own.nodes);

    // a stricter selection (a smaller search radius or radius per path): every try it asks for is among the qualifying
    // ones, so the extra registrations never add a missing one (on this recording the run's own tries already suffice)
    for (const auto &[radius, per_path] : std::vector<std::pair<double, double>>{{3.0, 0.5}, {2.0, 0.5}, {8.0, 0.25}, {8.0, 0.1}})
    {
        OdometryConfig stricter = config;
        stricter.loop_closure_search_radius_m = radius;
        stricter.loop_closure_radius_per_path = per_path;
        const LoopClosureReplayResult without = replay_loop_closure(run_only.capture, stricter, logger);
        const LoopClosureReplayResult with = replay_loop_closure(sink.capture, stricter, logger);
        MESSAGE("search radius ", radius, " m, radius per path ", per_path, ": ", without.missing.size(), " tries missing without the extra registrations, ",
                with.missing.size(), " with them; ", with.loops, " loops");
        CHECK(with.missing.size() <= without.missing.size());
    }

    // with a cap above every segment's qualifying count the capture holds every qualifying same-floor pair
    std::vector<Window> all = r.windows;
    MemoryLoopClosureCaptureSink complete;
    OdometryConfig every = config;
    every.capture_loop_closure_max_tries = 1000;
    run_loop_closure(all, every, store, logger, &complete);
    CHECK(same_bits(all_poses(all), all_poses(plain)));
    std::vector<RevisitSegment> revisit_segments;
    for (const auto &s : complete.capture.segments)
        revisit_segments.push_back({s.center, s.path_begin, s.path_end});
    size_t qualifying = 0, held = 0;
    for (int j = 0; j < static_cast<int>(revisit_segments.size()); j++)
        for (const int i : same_floor_revisits(revisit_segments, j, config))
        {
            qualifying++;
            for (const auto &t : complete.capture.tries)
                if (t.registration.i == i && t.registration.j == j && !t.registration.cross_floor)
                {
                    held++;
                    break;
                }
        }
    MESSAGE("cap 1000: ", complete.capture.tries.size(), " registrations, ", held, " of ", qualifying, " qualifying pairs held");
    CHECK(held == qualifying);
}

// ---- --reject-tries (change livox4-double-walls-cause, task 2.3) ----
namespace
{
    // a capture of the drifting loop recording with a registration for every same-floor try close_loops asks for: each
    // accepted, with fixed sigmas (no information), its correction a small shift that differs from try to try
    LoopClosureCapture capture_with_tries(const OdometryConfig &config)
    {
        Recording r = loop_recording(2);
        add_second_lap_drift(r, {0.3, 0.4, 0.0}, 2.0 * deg);
        LoopClosureCapture c;
        c.windows = r.windows;
        c.segments = loop_closure_segments(c.windows, config);
        std::vector<RevisitSegment> revisit_segments;
        for (const auto &s : c.segments)
            revisit_segments.push_back({s.center, s.path_begin, s.path_end});
        for (int j = 0; j < static_cast<int>(c.segments.size()); j++)
            for (const auto &t : revisit_tries(revisit_segments, j, config))
            {
                CapturedLoopTry k;
                k.registration.i = t.i;
                k.registration.j = j;
                k.registration.registered = true;
                k.registration.overlap_before = 0.6;
                k.registration.overlap_after = 0.8 - 0.01 * (c.tries.size() % 7);
                k.registration.correction.translation() = Eigen::Vector3d(-0.2 + 0.03 * (c.tries.size() % 5), -0.3, 0.0);
                k.registration.shift_m = k.registration.correction.translation().norm();
                k.accepted = loop_try_accepted(k.registration, config);
                k.requested_by_run = true;
                c.tries.push_back(k);
            }
        for (const auto &w : c.windows)
            c.result_poses.push_back(w.poses);
        return c;
    }
}

TEST_CASE("loop closure replay: a rejected try changes the edges and the result, an unknown one is named, none changes nothing")
{
    const OdometryConfig config = loop_closure_config();
    const LoopClosureCapture capture = capture_with_tries(config);
    NullLogger logger;
    const LoopClosureReplayResult plain = replay_loop_closure(capture, config, logger);
    REQUIRE(plain.missing.empty());
    REQUIRE(plain.loops >= 2);
    MESSAGE(capture.tries.size(), " tries, ", plain.loops, " loops, ", plain.report.dropped.size(), " dropped");
    CHECK(plain.report.first_solve.size() == plain.report.kept.size() + plain.report.dropped.size());

    // no rejection named: the same as without the option
    const LoopClosureReplayResult none = replay_loop_closure(capture, config, logger, {});
    CHECK(none.loops == plain.loops);
    CHECK(same_bits(all_poses(none.windows), all_poses(plain.windows)));
    CHECK(none.rejected.empty());

    // a kept edge's try rejected: that edge is gone, and the trajectory moves
    const LoopClosureEdge &kept = plain.report.kept.front();
    const std::pair<int, int> named{kept.i, kept.j};
    const LoopClosureReplayResult without = replay_loop_closure(capture, config, logger, {named});
    CHECK(without.rejected_not_in_capture.empty());
    CHECK(without.rejected == std::vector<std::string>{loop_try_name(named)});
    CHECK(without.rejected_not_asked.empty());
    for (const auto &e : without.report.first_solve)
        CHECK(!(e.i == kept.i && e.j == kept.j));
    CHECK(!same_bits(all_poses(without.windows), all_poses(plain.windows)));

    // a try the capture doesn't hold: named, nothing run
    const std::pair<int, int> unknown{0, 1};
    const LoopClosureReplayResult bad = replay_loop_closure(capture, config, logger, {named, unknown});
    CHECK(bad.rejected_not_in_capture == std::vector<std::string>{"0-1"});
    CHECK(bad.loops == 0);
    CHECK(same_bits(all_poses(bad.windows), all_poses(capture.windows)));

    // the edge list carries the edges' registration and first-solve readings and the tries rejected
    const auto file = std::filesystem::temp_directory_path() / "lidar_odometry_tests_loop_closure_edges.json";
    std::string why;
    REQUIRE(write_loop_closure_replay_json(file, without, LoopClosureSettingsDifference{}, config, why));
    std::ifstream in(file);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    std::filesystem::remove(file);
    for (const char *field : {"\"shift_m\"", "\"angle_deg\"", "\"first_solve_residual_m\"", "\"rejected_tries\"", "\"edges_kept\""})
        CHECK_MESSAGE(text.find(field) != std::string::npos, field);
    CHECK(text.find(loop_try_name(named)) != std::string::npos);
}

TEST_CASE("loop closure replay: the tries to reject are read as i-j[,i-j...]")
{
    std::vector<std::pair<int, int>> tries;
    std::string why;
    REQUIRE(parse_loop_try_names("3-17,12-40", tries, why));
    CHECK(tries == std::vector<std::pair<int, int>>{{3, 17}, {12, 40}});
    CHECK(loop_try_name(tries[1]) == "12-40");
    CHECK(!parse_loop_try_names("3-17,x", tries, why));
    CHECK(why.find("'x'") != std::string::npos);
    CHECK(!parse_loop_try_names("3:17", tries, why));
    CHECK(!parse_loop_try_names("3-17-2", tries, why));
    CHECK(!parse_loop_try_names("", tries, why));
}
