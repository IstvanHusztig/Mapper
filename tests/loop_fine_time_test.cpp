#include <doctest/doctest.h>

#include "loop_closure.h"
#include "loop_closure_capture.h"
#include "loop_closure_capture_file.h"
#include "loop_closure_replay.h"
#include "loop_fine_time.h"
#include "memory_point_store.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <random>
#include <string>
#include <vector>

// The fine-time placement of a blind stretch (change loop-closure-fine-time-guard; loop_fine_time.h, loop_fine_time_step in
// loop_closure.h): its pure parts on constructed numbers, the step on the synthetic corridor of tests/test_loop_fine_time.py
// (the same scene, built with the same formulas; the expected offsets are the Python prototype's on it), and the step
// through close_loops and the loop closure replay.

namespace
{
    using V3 = Eigen::Vector3d;
    const V3 X = V3::UnitX();

    struct QuietLogger : Logger
    {
        std::vector<std::string> errors;
        std::vector<std::string> infos;
        void write(LogLevel level, std::string_view message) override
        {
            if (level == LogLevel::Error)
                errors.emplace_back(message);
            else if (level == LogLevel::Info)
                infos.emplace_back(message);
        }
    };

    // numpy.arange's values: start, start + step, then start + i ((start + step) - start); its length ceil((stop - start) / step)
    std::vector<double> numpy_arange(double start, double stop, double step)
    {
        const size_t n = static_cast<size_t>(std::ceil((stop - start) / step));
        std::vector<double> v(n);
        if (n > 0)
            v[0] = start;
        if (n > 1)
            v[1] = start + step;
        const double delta = n > 1 ? v[1] - v[0] : 0.0;
        for (size_t i = 2; i < n; i++)
            v[i] = start + static_cast<double>(i) * delta;
        return v;
    }

    LoopFineTimeRule rule() { return LoopFineTimeRule{}; }
} // namespace

// ---- the pure parts --------------------------------------------------------------------------------------------------

TEST_CASE("loop fine time: sums are made in numpy's order")
{
    const double p[7] = {1.0, 10.0, 100.0, 1000.0, 10000.0, 100000.0, 1000000.0};
    const auto values = [&](int n)
    {
        std::vector<double> a;
        for (int i = 0; i < n; i++)
            a.push_back(((i % 13) - 6.0) * p[i % 7] / 3.0);
        return a;
    };
    // numpy 2.5.4's np.sum of these arrays (17 digits): sequential below 8, eight running sums up to 128, halves above
    const std::vector<double> a7 = values(7), a100 = values(100), a300 = values(300);
    CHECK(fine_time_pairwise_sum(a7.data(), a7.size()) == -41152.0);
    CHECK(fine_time_pairwise_sum(a100.data(), a100.size()) == -41144.99999999938);
    CHECK(fine_time_pairwise_sum(a300.data(), a300.size()) == -1672015.9999999984);
    CHECK(fine_time_pairwise_sum(a300.data(), a300.size()) / 300.0 == -5573.386666666661); // np.mean
    CHECK(fine_time_pairwise_sum(nullptr, 0) == 0.0);
}

TEST_CASE("loop fine time: thinning keeps the first point of a voxel in input order")
{
    const std::vector<V3> x{{0.51, 0, 0}, {0.01, 0, 0}, {0.02, 0, 0}, {0.55, 0, 0}, {-0.01, 0, 0}};
    CHECK(fine_time_thin(x, 0.1) == std::vector<size_t>{0, 1, 4});
    CHECK(fine_time_thin({}, 0.1).empty());
}

TEST_CASE("loop fine time: the nearest neighbours on the grid index equal brute force, ties to the lower index")
{
    std::mt19937_64 rng(5);
    std::uniform_real_distribution<double> u(-3.0, 3.0);
    std::vector<V3> points;
    for (int k = 0; k < 2000; k++)
        points.emplace_back(u(rng), u(rng), 0.5 * u(rng));
    // points at equal distances: a lattice of 0.25 m, and duplicates
    for (int a = -4; a <= 4; a++)
        for (int b = -4; b <= 4; b++)
            points.emplace_back(0.25 * a, 0.25 * b, 1.0);
    points.push_back(points[17]);
    points.push_back(points[17]);
    for (const double cell : {0.5, 0.65, 0.2})
    {
        const FineTimeIndex index(points, cell);
        std::vector<std::pair<double, size_t>> got;
        int queries = 0, mismatches = 0;
        for (int q = 0; q < 300; q++)
        {
            const V3 at = q < 100 ? V3(0.25 * (q % 9 - 4), 0.25 * (q / 9 % 9 - 4), 1.0) : (q < 110 ? points[17] : V3(u(rng), u(rng), 0.5 * u(rng)));
            for (const auto &[k, reach] : std::vector<std::pair<int, double>>{{4, 0.5}, {14, 0.65}, {1, 0.3}, {30, 1.2}})
            {
                std::vector<std::pair<double, size_t>> brute;
                for (size_t i = 0; i < points.size(); i++)
                {
                    const double dx = points[i].x() - at.x(), dy = points[i].y() - at.y(), dz = points[i].z() - at.z();
                    const double s = ((0.0 + dx * dx) + dy * dy) + dz * dz;
                    if (s < reach * reach)
                        brute.emplace_back(s, i);
                }
                std::sort(brute.begin(), brute.end());
                brute.resize(std::min(brute.size(), static_cast<size_t>(k)));
                index.nearest(at, k, reach, got);
                queries++;
                mismatches += got != brute;
            }
        }
        CHECK_MESSAGE(mismatches == 0, "cell ", cell, ": ", mismatches, " of ", queries, " queries differ from brute force");
    }
}

TEST_CASE("loop fine time: patches of a wall across the axis have its normal, a wall along it has none")
{
    const std::vector<double> g = numpy_arange(0.0, 2.0, 0.05);
    REQUIRE(g.size() == 40);
    std::vector<V3> wall, along;
    for (const double z : g) // np.meshgrid(g, g) raveled: rows by the second, columns by the first
        for (const double y : g)
        {
            wall.emplace_back(3.0, y, z);
            along.emplace_back(y, 1.0, z);
        }
    const FineTimePatches p = fine_time_patches(wall, X, 0.07, rule());
    CHECK(p.point.size() > 100);
    for (const V3 &n : p.normal)
        CHECK(std::abs(n.dot(X)) > 0.999);
    CHECK(std::is_sorted(p.point.begin(), p.point.end()));
    CHECK(fine_time_patches(along, X, 0.07, rule()).point.empty());
    CHECK(fine_time_patches(std::vector<V3>(wall.begin(), wall.begin() + 10), X, 0.07, rule()).point.empty()); // fewer than 16
}

TEST_CASE("loop fine time: the held-out hash holds out a fifth, fixed by the position's cell, with a non-negative remainder")
{
    std::mt19937_64 rng(1);
    std::uniform_real_distribution<double> u(-20.0, 20.0);
    int held = 0;
    for (int k = 0; k < 5000; k++)
    {
        const V3 q(u(rng), u(rng), u(rng));
        held += fine_time_held_out(q, rule());
        CHECK(fine_time_held_out(q, rule()) == fine_time_held_out(V3(std::floor(q.x()) + 0.5, std::floor(q.y()) + 0.5, std::floor(q.z()) + 0.5), rule()));
    }
    CHECK(held > 750);
    CHECK(held < 1250);
    CHECK(fine_time_held_out(V3(0.2, 0.7, 0.1), rule()));                  // cell 0: h = 0
    CHECK(!fine_time_held_out(V3(-0.5, -0.5, -0.5), rule()));              // cell -1: h = -176698547, remainder 3 (Python's %)
    CHECK(fine_time_held_out(V3(-0.5, -0.5, -0.5), rule()) == fine_time_held_out(V3(-0.9, -0.1, -0.3), rule()));
}

TEST_CASE("loop fine time: at most 256 patches, spread over all; the offset grid; the order of preference")
{
    CHECK(fine_time_take(5, 256) == std::vector<size_t>{0, 1, 2, 3, 4});
    const std::vector<size_t> t = fine_time_take(1000, 256);
    REQUIRE(t.size() == 256);
    CHECK(t.front() == 0);
    CHECK(t.back() == 999);
    CHECK(std::adjacent_find(t.begin(), t.end(), [](size_t a, size_t b) { return b <= a; }) == t.end());
    CHECK(t[1] == 3); // floor(1 * 999 / 255)

    double bound = 0.0;
    const std::vector<double> grid = fine_time_offset_grid(5.04, rule(), bound);
    CHECK(bound == doctest::Approx(7.56));
    REQUIRE(grid.size() == 151);
    CHECK(grid.front() == -75 * 0.1);
    CHECK(grid.back() == 75 * 0.1);
    CHECK(grid[75] == 0.0);
    CHECK(fine_time_offset_grid(100.0, rule(), bound).size() == 161);
    CHECK(bound == 8.0);

    const std::vector<double> small = {-0.2, -0.1, 0.0, 0.1, 0.2};
    CHECK(fine_time_preference(small) == std::vector<int>{2, 1, 3, 0, 4});
}

namespace
{
    // every combination of offsets: the lowest cost of rule 6
    double brute_profile_cost(const std::vector<std::vector<double>> &E, const std::vector<double> &grid, double coupling)
    {
        const size_t W = E.size(), G = grid.size();
        std::vector<size_t> path(W, 0);
        double best = std::numeric_limits<double>::infinity();
        for (;;)
        {
            double cost = 0.0, before = 0.0;
            for (size_t w = 0; w < W; w++)
            {
                cost += E[w][path[w]] + coupling * (grid[path[w]] - before) * (grid[path[w]] - before);
                before = grid[path[w]];
            }
            cost += coupling * before * before;
            best = std::min(best, cost);
            size_t w = 0;
            while (w < W && ++path[w] == G)
                path[w++] = 0;
            if (w == W)
                return best;
        }
    }

    double profile_cost(const std::vector<std::vector<double>> &E, const std::vector<double> &grid, const std::vector<int> &path, double coupling)
    {
        double cost = 0.0, before = 0.0;
        for (size_t w = 0; w < E.size(); w++)
        {
            cost += E[w][path[w]] + coupling * (grid[path[w]] - before) * (grid[path[w]] - before);
            before = grid[path[w]];
        }
        return cost + coupling * before * before;
    }

    std::vector<double> grid_of(int k)
    {
        std::vector<double> g;
        for (int i = -k; i <= k; i++)
            g.push_back(i * 0.1);
        return g;
    }
} // namespace

TEST_CASE("loop fine time: the profile is the brute-force minimum; no support gives zero; the ends are pinned; ties as the rule orders")
{
    std::mt19937_64 rng(7);
    std::uniform_real_distribution<double> u(0.0, 0.05);
    const std::vector<double> grid = grid_of(2);
    for (int trial = 0; trial < 20; trial++)
    {
        std::vector<std::vector<double>> E(5, std::vector<double>(grid.size()));
        for (auto &row : E)
            for (double &e : row)
                e = u(rng);
        std::fill(E[trial % 5].begin(), E[trial % 5].end(), 0.0); // an unsupported window
        const std::vector<int> path = fine_time_profile(E, grid, rule());
        CHECK(profile_cost(E, grid, path, 0.10) == doctest::Approx(brute_profile_cost(E, grid, 0.10)).epsilon(1e-12));
    }

    const std::vector<double> wide = grid_of(5);
    for (const int k : fine_time_profile(std::vector<std::vector<double>>(7, std::vector<double>(wide.size(), 0.0)), wide, rule()))
        CHECK(wide[k] == 0.0);

    // one window wants +1.0 m strongly: a ramp up and down around it, the ends pinned
    const std::vector<double> g20 = grid_of(20);
    std::vector<std::vector<double>> E(9, std::vector<double>(g20.size(), 0.0));
    for (size_t k = 0; k < g20.size(); k++)
        E[4][k] = std::abs(g20[k] - 1.0) > 1e-9 ? 0.25 : 0.0;
    const std::vector<int> path = fine_time_profile(E, g20, rule());
    std::vector<double> d;
    for (const int k : path)
        d.push_back(g20[k]);
    CHECK(d[4] == doctest::Approx(1.0));
    for (int w = 0; w < 4; w++)
        CHECK(d[w + 1] > d[w]);
    for (int w = 4; w < 8; w++)
        CHECK(d[w + 1] < d[w]);
    CHECK(std::abs(d[0]) < std::abs(d[1]));

    // -0.2 and +0.2 cost the same: the negative; then +0.1 costs exactly as much as +-0.2: the smaller |d|
    const std::vector<double> g3 = grid_of(3);
    std::vector<std::vector<double>> T(1, std::vector<double>(g3.size(), 1.0));
    T[0][1] = T[0][5] = 0.0;
    CHECK(g3[fine_time_profile(T, g3, rule())[0]] == doctest::Approx(-0.2));
    T[0][4] = 0.10 * 2 * (0.2 * 0.2 - 0.1 * 0.1);
    CHECK(g3[fine_time_profile(T, g3, rule())[0]] == doctest::Approx(0.1));
    CHECK(fine_time_own_offset(T[0], g3, rule()) == doctest::Approx(-0.2)); // alone, without the coupling
}

TEST_CASE("loop fine time: the profile on the nodes is numpy.interp through the pins and the windows' middles")
{
    const std::vector<double> middles{1.0, 2.0, 3.0}, offsets{0.5, -1.0, 0.25};
    CHECK(fine_time_offset_at(0.0, 0.5, 3.5, middles, offsets) == 0.0);  // before the first pin
    CHECK(fine_time_offset_at(0.5, 0.5, 3.5, middles, offsets) == 0.0);  // at it
    CHECK(fine_time_offset_at(0.75, 0.5, 3.5, middles, offsets) == 0.25);
    CHECK(fine_time_offset_at(1.0, 0.5, 3.5, middles, offsets) == 0.5);
    CHECK(fine_time_offset_at(1.5, 0.5, 3.5, middles, offsets) == -0.25);
    CHECK(fine_time_offset_at(3.25, 0.5, 3.5, middles, offsets) == 0.125);
    CHECK(fine_time_offset_at(3.5, 0.5, 3.5, middles, offsets) == 0.0);  // at the last pin
    CHECK(fine_time_offset_at(4.0, 0.5, 3.5, middles, offsets) == 0.0);
}

TEST_CASE("loop fine time: the edge classes")
{
    CHECK(fine_time_edge_class(0.3, 0.0, 5.0, 0.2, rule()) == FineTimeEdgeClass::unconfirmed);
    CHECK(fine_time_edge_class(0.9, 0.0, 0.15, 0.2, rule()) == FineTimeEdgeClass::agrees);
    CHECK(fine_time_edge_class(0.9, 1.2, -0.4, 0.2, rule()) == FineTimeEdgeClass::agrees); // beyond the limit, no larger than before
    CHECK(fine_time_edge_class(0.9, 0.0, 0.5, 0.2, rule()) == FineTimeEdgeClass::contradicted);
    CHECK(std::string(fine_time_edge_class_name(FineTimeEdgeClass::contradicted)) == "contradicted");
}

TEST_CASE("loop fine time: the apply rule removes contradicted candidates round by round; an edge between two removes both")
{
    // run 0 is contradicted by edge a while it is in the set; run 1 by edge b only once run 0 has left; run 2 never; run 3
    // was no candidate (a contradicted reading of it changes nothing)
    std::vector<bool> set{true, true, true, false};
    int rounds = 0;
    const auto removed = fine_time_apply_rule(set, [&](const std::vector<bool> &s) {
        rounds++;
        std::vector<std::pair<int, std::string>> out{{3, "d"}};
        if (s[0])
            out.emplace_back(0, "a");
        if (!s[0])
            out.emplace_back(1, "b");
        return out;
    });
    CHECK(rounds == 3);
    CHECK(set == std::vector<bool>{false, false, true, false});
    CHECK(removed == std::vector<std::string>{"a", "b", "", ""});

    std::vector<bool> both{true, true};
    const auto r2 = fine_time_apply_rule(both, [](const std::vector<bool> &s) {
        std::vector<std::pair<int, std::string>> out;
        if (s[0] && s[1])
            out = {{0, "0 -> 9"}, {1, "0 -> 9"}};
        return out;
    });
    CHECK(both == std::vector<bool>{false, false});
    CHECK(r2 == std::vector<std::string>{"0 -> 9", "0 -> 9"});
}

TEST_CASE("loop fine time: supports reach a segment's windows beyond the run, cut at the ends; overlapping supports are marked")
{
    const LoopFineTimeRule r = rule();
    const auto apart = fine_time_supports({{10, 20}, {40, 50}}, 100, r);
    CHECK(apart[0].first_window == 4);
    CHECK(apart[0].last_window == 26);
    CHECK(!apart[0].overlap);
    CHECK(!apart[1].overlap);
    const auto close = fine_time_supports({{10, 20}, {30, 35}, {80, 98}}, 100, r);
    CHECK(close[0].overlap);
    CHECK(close[1].overlap);
    CHECK(!close[2].overlap);
    CHECK(close[2].last_window == 99);
    CHECK(fine_time_supports({{2, 5}}, 100, r)[0].first_window == 0);
}

TEST_CASE("loop fine time: the score curve of a patch in front of a wall is lowest where the offset puts it onto the wall")
{
    // reference: patches on the plane x = 5 (normals along x); source: patches on x = 4.7, i.e. 0.3 m short of it
    std::vector<V3> wall, source;
    for (int a = 0; a < 20; a++)
        for (int b = 0; b < 20; b++)
        {
            wall.emplace_back(5.0, 0.1 * a, 0.1 * b);
            if (a % 2 == 0 && b % 2 == 0)
                source.emplace_back(4.7, 0.1 * a + 0.05, 0.1 * b);
        }
    std::vector<V3> normals(wall.size(), X), source_normals(source.size(), -X);
    const FineTimeIndex index(wall, 0.5);
    const std::vector<double> grid = grid_of(6);
    const FineTimeCurve c = fine_time_score_curve(source, source_normals, grid, X, index, normals, rule());
    REQUIRE(c.score.size() == grid.size());
    const size_t best = static_cast<size_t>(std::min_element(c.score.begin(), c.score.end()) - c.score.begin());
    CHECK(grid[best] == doctest::Approx(0.3));
    CHECK(c.score[best] == doctest::Approx(0.05 * 0.05)); // the 0.05 m across to the nearest wall point
    CHECK(c.matched[best] == static_cast<int>(source.size()));
    CHECK(c.matched[0] == 0);                  // -0.6: 0.9 m away, nothing within 0.5 m
    CHECK(c.score[0] == doctest::Approx(0.25)); // every patch counts 0.5 m
    // a normal at 60 degrees to the wall's does not match
    std::vector<V3> tilted(source.size(), V3(0.5, std::sqrt(0.75), 0.0));
    CHECK(fine_time_score_curve(source, tilted, grid, X, index, normals, rule()).matched[best] == 0);
}

// ---- the synthetic corridor of tests/test_loop_fine_time.py ----------------------------------------------------------
namespace
{
    // A corridor along x (walls at y = +-1) with door jambs across it, walked out (0..20 m in 20 s) and back (40..60 s), 100
    // windows of 4 nodes, each window's points (within 4 m along x) seen from its node 1. The windows of `weak` are weak along x
    // (information ratio 0.01, placed move 0.2 m against 0.4 m elsewhere). odometry_error / corrected_error [m] along x
    // are added to the odometry's poses / to the poses after the loop closure (tests/test_loop_fine_time.py::corridor adds
    // `error` to the latter).
    struct Corridor
    {
        OdometryConfig config;
        std::vector<Window> odometry, corrected;
        std::vector<V3> truth; // per node, all windows in order
        MemoryPointStore store;
        std::vector<LoopClosureSegment> segments;
        std::vector<LoopOdometryRun> runs;
        std::vector<LoopSegmentPoints> points;
        int nodes_per_window = 4;
    };

    double bump(double t)
    {
        return 1.2 * std::max(0.0, 1.0 - std::abs(t - 8.2) / 3.0);
    }

    void build_corridor(Corridor &c, const std::vector<double> &jambs, const std::function<double(double)> &odometry_error,
                        const std::function<double(double)> &corrected_error, const std::vector<std::pair<int, int>> &weak = {{13, 27}})
    {
        c.config = OdometryConfig{};
        c.config.use_loop_closure = true;
        c.config.loop_closure_segment_windows = 6;
        c.config.min_distance_lidar = 0.1;
        c.config.max_distance_lidar = 40.0;
        c.config.holdout_block_s = 0.0;
        c.config.loop_closure_odometry_from_readings = true;
        c.config.loop_closure_fine_time_placement = true;
        c.config.use_exact_ndt_bucket_statistics = false;

        const std::vector<double> g = numpy_arange(0.0, 2.4, 0.06), xs = numpy_arange(-2.0, 22.0, 0.06);
        REQUIRE(g.size() == 40);
        REQUIRE(xs.size() == 400);
        std::vector<V3> scene;
        for (const double s : {-1.0, 1.0})
            for (const double z : g)
                for (const double x : xs)
                    scene.emplace_back(x, s, z);
        for (const double xj : jambs)
            for (const double y0 : {-1.0, 0.4})
            {
                const std::vector<double> yy = numpy_arange(y0, y0 + 0.6001, 0.06);
                REQUIRE(yy.size() == 11);
                for (const double z : g)
                    for (const double y : yy)
                        scene.emplace_back(xj, y, z);
            }
        const int W = 100, per = c.nodes_per_window;
        c.odometry.assign(W, Window{});
        c.corrected.assign(W, Window{});
        c.truth.clear();
        for (int w = 0; w < W; w++)
        {
            for (int m = 0; m < per; m++)
            {
                const double t = (w < 50 ? w * 0.4 : 40.0 + (w - 50) * 0.4) + m * 0.1;
                const V3 truth(t <= 20.0 ? t : 20.0 - (t - 40.0), 0.0, 1.2);
                c.truth.push_back(truth);
                Eigen::Affine3d p0 = Eigen::Affine3d::Identity(), p1 = Eigen::Affine3d::Identity();
                p0.translation() = truth;
                p1.translation() = truth;
                if (odometry_error)
                    p0.translation().x() += odometry_error(t);
                if (corrected_error)
                    p1.translation().x() += corrected_error(t);
                c.odometry[w].poses.push_back(p0);
                c.corrected[w].poses.push_back(p1);
                c.odometry[w].timestamps.emplace_back(t, t);
            }
            c.odometry[w].motion_model_poses = c.odometry[w].poses;
            bool in_weak = false;
            for (const auto &[a, b] : weak)
                in_weak = in_weak || (w >= a && w <= b);
            c.odometry[w].readings = WindowReadings{in_weak ? 0.01 : 0.5, X, false, in_weak ? V3(0.2, 0.0, 0.0) : V3(0.4, 0.0, 0.0), 0.4};
            c.corrected[w].timestamps = c.odometry[w].timestamps;
            c.corrected[w].motion_model_poses = c.corrected[w].poses;
            c.corrected[w].readings = c.odometry[w].readings;
            // the points seen from node 1 (its true pose), within 4 m along x
            const V3 &at = c.truth[w * per + 1];
            std::vector<Point3Di> points;
            for (const V3 &s : scene)
                if (std::abs(s.x() - at.x()) <= 4.0)
                {
                    Point3Di p{};
                    p.point = V3(s.x() - at.x(), s.y() - at.y(), s.z() - at.z());
                    p.timestamp = c.odometry[w].timestamps[1].first;
                    p.index_pose = 1;
                    points.push_back(p);
                }
            c.store.save(PointSet::WindowIntermediate, w, points);
        }
        c.segments = loop_closure_segments(c.odometry, c.config);
        std::vector<std::optional<WindowReadings>> readings;
        for (const Window &w : c.odometry)
            readings.push_back(w.readings);
        c.runs = loop_odometry_step_information(readings, c.segments, loop_odometry_rule(c.config)).runs;
        c.points.clear();
        for (const LoopClosureSegment &s : c.segments)
            c.points.push_back(loop_segment_points(c.odometry, s, c.config, c.store, true));
    }

    std::vector<std::vector<Eigen::Affine3d>> poses_of(const std::vector<Window> &windows)
    {
        std::vector<std::vector<Eigen::Affine3d>> p;
        for (const Window &w : windows)
            p.push_back(w.poses);
        return p;
    }

    std::vector<Eigen::Affine3d> segment_poses(const Corridor &c)
    {
        std::vector<Eigen::Affine3d> p;
        for (const LoopClosureSegment &s : c.segments)
            p.push_back(s.pose);
        return p;
    }

    // the step on the corridor's corrected poses with the curves from its points
    LoopFineTimeReport step(Corridor &c, std::vector<Window> &windows, const std::vector<LoopClosureEdge> &kept = {},
                            const std::vector<const LoopClosureTry *> &tries = {})
    {
        PointFineTimeScores scores(c.points, c.segments, c.store, c.config);
        QuietLogger logger;
        LoopFineTimeReport report;
        std::string why;
        const bool ok = loop_fine_time_step(windows, poses_of(c.odometry), c.segments, c.runs, kept, tries, segment_poses(c), c.config, scores, logger,
                                            report, why);
        CHECK_MESSAGE(ok, why);
        return report;
    }

    bool same_bits(const std::vector<Window> &a, const std::vector<Window> &b)
    {
        if (a.size() != b.size())
            return false;
        for (size_t w = 0; w < a.size(); w++)
        {
            if (a[w].poses.size() != b[w].poses.size())
                return false;
            for (size_t n = 0; n < a[w].poses.size(); n++)
                if (a[w].poses[n].matrix() != b[w].poses[n].matrix())
                    return false;
        }
        return true;
    }

    // the profile's grid steps from the middle, per window of the support
    std::vector<int> steps_of(const LoopFineTimeRunReport &r)
    {
        std::vector<int> s;
        const int middle = static_cast<int>(r.curves.grid.size()) / 2;
        for (const int k : r.offset_index)
            s.push_back(k - middle);
        return s;
    }
} // namespace

// The expected offsets (grid steps of 0.1 m from 0, per window of the support 7..33) are those of tools/loop_fine_time.py on
// the same scene (run_rule(corridor(...), [])), printed by a script that runs tests/test_loop_fine_time.py::corridor.
TEST_CASE("loop fine time corridor: an error inside the blind stretch is taken out where jambs are seen, nothing moves outside")
{
    Corridor c;
    build_corridor(c, {3.0, 6.5, 9.0, 13.0, 17.0}, nullptr, bump);
    std::vector<Window> windows = c.corrected;
    const auto start = std::chrono::steady_clock::now();
    const LoopFineTimeReport report = step(c, windows);
    MESSAGE("the step on the corridor: ", std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count(), " s");
    REQUIRE(report.runs.size() == 1);
    const LoopFineTimeRunReport &r = report.runs[0];
    CHECK(r.run.first_window == 13);
    CHECK(r.run.last_window == 27);
    CHECK(r.run.support_first == 7);
    CHECK(r.run.support_last == 33);
    CHECK(r.status == "applied");
    CHECK(r.curves.grid.size() == 93);
    CHECK(r.curves.reference_patches == 1186); // the prototype's count
    CHECK(std::all_of(r.supported.begin(), r.supported.end(), [](char s) { return s != 0; }));
    CHECK(steps_of(r) == std::vector<int>{0, 0, 0, 0, 0, 0, 0, -2, -4, -5, -7, -8, -10, -11, -11, -9, -8, -6, -4, -3, -1, 0, 0, 0, 0, 0, 0});
    // the prototype's held-out means: 0.1024737731481481 -> 0.0038317068923227854
    MESSAGE("held-out mean before ", r.held_out_before, " after ", r.held_out_after, " (prototype 0.1024737731481481 -> 0.0038317068923227854)");
    CHECK(r.held_out_before == 0.1024737731481481);
    CHECK(r.held_out_after == 0.0038317068923227854);
    CHECK(r.held_out_after < r.held_out_before);

    const int per = c.nodes_per_window;
    double before = 0.0, after = 0.0;
    for (int w = 13; w <= 27; w++)
        for (int n = 0; n < per; n++)
        {
            before = std::max(before, std::abs(c.corrected[w].poses[n].translation().x() - c.truth[w * per + n].x()));
            after = std::max(after, std::abs(windows[w].poses[n].translation().x() - c.truth[w * per + n].x()));
        }
    MESSAGE("largest error along the corridor in the run: ", before, " m before, ", after, " m after");
    CHECK(before > 1.0);
    CHECK(after <= 0.1 + 1e-9); // to one grid step
    for (int w = 0; w < 100; w++)
        for (int n = 0; n < per; n++)
        {
            const Eigen::Affine3d &a = windows[w].poses[n], &b = c.corrected[w].poses[n];
            if (w < 7 || w > 33)
                CHECK(a.matrix() == b.matrix());
            CHECK(a.linear() == b.linear());
            CHECK(a.translation().y() == b.translation().y());
            CHECK(a.translation().z() == b.translation().z());
        }
    for (int w = 7; w <= 33; w++)
        CHECK(windows[w].motion_model_poses.back().matrix() == windows[w].poses.back().matrix());
}

TEST_CASE("loop fine time corridor: a right trajectory stays, bit for bit")
{
    Corridor c;
    build_corridor(c, {3.0, 6.5, 9.0, 13.0, 17.0}, nullptr, nullptr);
    std::vector<Window> windows = c.corrected;
    const LoopFineTimeReport report = step(c, windows);
    REQUIRE(report.runs.size() == 1);
    CHECK(report.runs[0].status == "unchanged");
    CHECK(steps_of(report.runs[0]) == std::vector<int>(27, 0));
    MESSAGE("held-out mean ", report.runs[0].held_out_before, " (prototype 0.0033834374999999993)");
    CHECK(report.runs[0].held_out_before == 0.0033834374999999993);
    CHECK(same_bits(windows, c.corrected));
}

TEST_CASE("loop fine time corridor: nothing across the passage: not timed, bit for bit unchanged")
{
    Corridor c;
    build_corridor(c, {}, nullptr, bump);
    std::vector<Window> windows = c.corrected;
    const LoopFineTimeReport report = step(c, windows);
    REQUIRE(report.runs.size() == 1);
    const LoopFineTimeRunReport &r = report.runs[0];
    CHECK(r.status == "not timed");
    CHECK(r.cause == "no surface across the passage seen by another visit");
    CHECK(r.curves.reference_patches == 0);
    CHECK(std::none_of(r.supported.begin(), r.supported.end(), [](char s) { return s != 0; }));
    CHECK(same_bits(windows, c.corrected));
}

TEST_CASE("loop fine time corridor: equal jambs every 2 m do not move a right trajectory by more than one grid step")
{
    std::vector<double> jambs;
    for (double x = 1.0; x < 20.0; x += 2.0)
        jambs.push_back(x);
    Corridor c;
    build_corridor(c, jambs, nullptr, nullptr);
    std::vector<Window> windows = c.corrected;
    const LoopFineTimeReport report = step(c, windows);
    REQUIRE(report.runs.size() == 1);
    const LoopFineTimeRunReport &r = report.runs[0];
    CHECK(r.curves.bound_m > 2.0); // the search reaches beyond one spacing
    CHECK(steps_of(r) == std::vector<int>(27, 0)); // the prototype's profile
    CHECK(r.curves.reference_patches == 2374);
    double moved = 0.0;
    for (size_t w = 0; w < windows.size(); w++)
        for (size_t n = 0; n < windows[w].poses.size(); n++)
            moved = std::max(moved, (windows[w].poses[n].translation() - c.corrected[w].poses[n].translation()).cwiseAbs().maxCoeff());
    CHECK(moved <= 0.1 + 1e-9);
}

// ---- loop edges read at their time support, and the apply rule (on the corridor with the bump) ----
namespace
{
    // a kept edge i -> j with the fixed sigmas, its registration `correction`, and per-window information along x only in the
    // source windows listed (the others none)
    struct ConstructedEdge
    {
        LoopClosureEdge edge;
        LoopClosureTry registration;
    };

    ConstructedEdge constructed_edge(const Corridor &c, int i, int j, const Eigen::Affine3d &correction, const std::vector<int> &informed_windows)
    {
        ConstructedEdge e;
        e.edge.i = i;
        e.edge.j = j;
        e.edge.correction = correction;
        e.edge.information = false;
        e.registration.i = i;
        e.registration.j = j;
        e.registration.registered = true;
        e.registration.correction = correction;
        for (int w = c.segments[j].first_window; w <= c.segments[j].last_window; w++)
        {
            Eigen::Matrix3d m = Eigen::Matrix3d::Identity() * 40.0;
            m(0, 0) = std::find(informed_windows.begin(), informed_windows.end(), w) != informed_windows.end() ? 100.0 : 0.0;
            e.registration.window_information.push_back(m);
        }
        return e;
    }

    Eigen::Affine3d shift_x(double x)
    {
        Eigen::Affine3d a = Eigen::Affine3d::Identity();
        a.translation().x() = x;
        return a;
    }
} // namespace

TEST_CASE("loop fine time edges: an edge whose information lies late in its segment agrees; one the informative scans contradict stops the run")
{
    Corridor c;
    build_corridor(c, {3.0, 6.5, 9.0, 13.0, 17.0}, nullptr, bump);
    // segment 4 = windows 24-29: the profile moves its first pose (window 24) by -0.4 m, its information lies in windows 28
    // and 29, which neither the bump nor the profile moves; the target 12 (windows 72-77) is on the way back
    const ConstructedEdge late = constructed_edge(c, 12, 4, Eigen::Affine3d::Identity(), {28, 29});
    {
        std::vector<Window> windows = c.corrected;
        const LoopFineTimeReport report = step(c, windows, {late.edge}, {&late.registration});
        REQUIRE(report.runs.size() == 1);
        CHECK(report.runs[0].status == "applied");
        REQUIRE(report.edges.size() == 1);
        const LoopFineTimeEdgeRow &row = report.edges[0];
        MESSAGE("late edge: first pose ", row.first_pose_m, " m, time support ", row.rho_before_m, " -> ", row.rho_after_m, " m, limit ", row.limit_m,
                ", informed ", row.informed_share);
        CHECK(!row.target);
        CHECK(row.first_pose_m == doctest::Approx(bump(24 * 0.4)));
        CHECK(std::abs(row.rho_before_m) < 1e-12);
        CHECK(std::abs(row.rho_after_m) < 1e-12);
        CHECK(row.informed_share == doctest::Approx(1.0));
        CHECK(row.limit_m == doctest::Approx(0.15)); // 3 sigma of 0.05 m
        CHECK(row.cls == FineTimeEdgeClass::agrees);
        CHECK(!same_bits(windows, c.corrected));
    }
    // segment 3 = windows 18-23, all informative and moved by the profile; its registration says the bump was right (it moves
    // the source by the bump's mean there), so the profile takes the source away from it
    double mean_bump = 0.0;
    for (int w = 18; w <= 23; w++)
        mean_bump += bump(w * 0.4 + 0.2) / 6.0;
    const ConstructedEdge against = constructed_edge(c, 12, 3, shift_x(mean_bump), {18, 19, 20, 21, 22, 23});
    std::vector<Window> windows = c.corrected;
    const LoopFineTimeReport report = step(c, windows, {late.edge, against.edge}, {&late.registration, &against.registration});
    REQUIRE(report.runs.size() == 1);
    CHECK(report.runs[0].status == "not applied");
    CHECK(report.runs[0].cause == "edge 12 -> 3 contradicted");
    REQUIRE(report.edges.size() == 2);
    const LoopFineTimeEdgeRow &row = report.edges[1];
    MESSAGE("contradicting edge: time support ", row.rho_before_m, " -> ", row.rho_after_m, " m, limit ", row.limit_m);
    // the table is the last round's (the prototype's): the run's profile has left the set, so the edge reads the same
    // before and after there; the round that removed the run is marked
    CHECK(row.stopped_run);
    CHECK(row.rho_after_m == row.rho_before_m);
    CHECK(!report.edges[0].stopped_run);
    CHECK(same_bits(windows, c.corrected)); // the run stays as the pose graph left it
}

TEST_CASE("loop fine time edges: a later visit registered onto a moved part of the run stops the run")
{
    Corridor c;
    build_corridor(c, {3.0, 6.5, 9.0, 13.0, 17.0}, nullptr, bump);
    // target segment 3 (its first node, window 18, lies in the support and moves with the profile), source 12 on the way back
    // (outside every support); the registration agrees with the target where the loop closure left it (K_i C = I)
    const double k_i = bump(18 * 0.4);
    const ConstructedEdge onto = constructed_edge(c, 3, 12, shift_x(-k_i), {72, 73, 74, 75, 76, 77});
    std::vector<Window> windows = c.corrected;
    const LoopFineTimeReport report = step(c, windows, {onto.edge}, {&onto.registration});
    REQUIRE(report.runs.size() == 1);
    REQUIRE(report.edges.size() == 1);
    const LoopFineTimeEdgeRow &row = report.edges[0];
    MESSAGE("edge onto the moved part: time support ", row.rho_before_m, " -> ", row.rho_after_m, " m, limit ", row.limit_m, ", informed ",
            row.informed_share);
    CHECK(row.target);
    CHECK(std::abs(row.rho_before_m) < 1e-12);
    CHECK(row.informed_share == doctest::Approx(1.0)); // no source window is blind
    CHECK(row.stopped_run);
    CHECK(report.runs[0].status == "not applied");
    CHECK(report.runs[0].cause == "edge 3 -> 12 contradicted");
    CHECK(same_bits(windows, c.corrected));
}

// ---- through close_loops and the loop closure replay ----
namespace
{
    // every pair asked for registered and accepted, with the correction the truth gives (segment j's first node moved to
    // where segment i's odometry error puts the truth there: O_i T_i^-1 T_j O_j^-1), a well-determined information and
    // per-window blocks
    class CorridorRegistrations : public LoopClosureRegistrations
    {
    public:
        explicit CorridorRegistrations(const Corridor &c) : c_(c), segments_(c.segments) {}
        const LoopClosureTry *registration(int i, int j, bool cross_floor) override
        {
            const auto key = std::make_tuple(i, j, cross_floor);
            if (const auto it = index_.find(key); it != index_.end())
                return &tries_[it->second];
            LoopClosureTry t;
            t.i = i;
            t.j = j;
            t.cross_floor = cross_floor;
            t.registered = true;
            t.overlap_before = 0.6;
            t.overlap_after = 0.9;
            t.information = Eigen::Matrix<double, 6, 6>::Identity() * 1e4;
            t.centre = segments_[j].center;
            const auto truth_at = [&](int k)
            {
                Eigen::Affine3d T = Eigen::Affine3d::Identity();
                T.translation() = c_.truth[segments_[k].first_window * c_.nodes_per_window];
                return T;
            };
            const auto odometry_at = [&](int k) { return c_.odometry[segments_[k].first_window].poses[0]; };
            t.correction = odometry_at(i) * truth_at(i).inverse() * truth_at(j) * odometry_at(j).inverse();
            t.shift_m = (t.correction * t.centre - t.centre).norm();
            for (int w = segments_[j].first_window; w <= segments_[j].last_window; w++)
            {
                Eigen::Matrix3d m = Eigen::Matrix3d::Identity() * 40.0;
                m(0, 0) = 10.0 + w - segments_[j].first_window;
                t.window_information.push_back(m);
            }
            index_[key] = tries_.size();
            tries_.push_back(t);
            return &tries_.back();
        }
        std::vector<CapturedLoopTry> captured(const OdometryConfig &config) const
        {
            std::vector<CapturedLoopTry> out;
            for (const LoopClosureTry &t : tries_)
                out.push_back({t, loop_try_accepted(t, config), true});
            return out;
        }

    private:
        const Corridor &c_;
        const std::vector<LoopClosureSegment> &segments_;
        std::deque<LoopClosureTry> tries_;
        std::map<std::tuple<int, int, bool>, size_t> index_;
    };

    bool contains(const std::vector<std::string> &lines, const std::string &text)
    {
        return std::any_of(lines.begin(), lines.end(), [&](const std::string &l) { return l.find(text) != std::string::npos; });
    }
} // namespace

TEST_CASE("loop fine time replay: a capture made with the step on replays bit for bit; another graph or other eligible runs are refused")
{
    // the odometry carries the bump; a second, short weak run (windows 70-72, sigma about 0.6 m) is not eligible at 1 m
    Corridor c;
    build_corridor(c, {3.0, 6.5, 9.0, 13.0, 17.0}, bump, nullptr, {{13, 27}, {70, 72}});
    REQUIRE(c.runs.size() == 2);
    MESSAGE("runs' sigmas ", c.runs[0].sigma_m, " and ", c.runs[1].sigma_m, " m");
    REQUIRE(c.runs[0].sigma_m >= 1.0);
    REQUIRE(c.runs[1].sigma_m < 1.0);
    REQUIRE(c.runs[1].sigma_m > 0.5);

    std::vector<Window> windows = c.odometry;
    CorridorRegistrations registrations(c);
    PointFineTimeScores scores(c.points, c.segments, c.store, c.config);
    QuietLogger logger;
    LoopClosureReport report;
    const int loops = close_loops(windows, c.segments, registrations, c.config, logger, &report, &scores);
    REQUIRE(report.outcome == "applied");
    REQUIRE(loops > 0);
    REQUIRE(report.fine_time.ran);
    REQUIRE(report.fine_time.runs.size() == 1);
    REQUIRE(report.fine_time.runs[0].scored);
    REQUIRE(scores.captured().has_value());
    MESSAGE("close_loops: ", loops, " loops, fine time: ", report.fine_time.runs[0].status, " (", report.fine_time.runs[0].cause, "), largest offset ",
            report.fine_time.runs[0].largest_offset_m, " m, ", report.fine_time.edges.size(), " edges read, ", report.fine_time.seconds, " s");
    CHECK(contains(logger.infos, "loop closure fine time: run windows 13-27"));
    REQUIRE(report.fine_time.runs[0].status == "applied"); // the replay below must reproduce a move, not only the graph

    LoopClosureCapture capture;
    capture.windows = c.odometry;
    capture.segments = c.segments;
    capture.tries = registrations.captured(c.config);
    for (const Window &w : windows)
        capture.result_poses.push_back(w.poses);
    capture.loops = loops;
    capture.fine_time = scores.captured();

    // the capture's own settings, from memory and through the file
    const LoopClosureReplayResult own = replay_loop_closure(capture, c.config, logger);
    CHECK(own.missing.empty());
    CHECK(own.report.outcome == "applied");
    CHECK(own.identical_nodes == own.nodes);
    const auto file = std::filesystem::temp_directory_path() / "lidar_odometry_tests_loop_fine_time_capture.bin";
    std::string why;
    REQUIRE(write_loop_closure_capture(file, capture, c.config, why));
    LoopClosureCapture back;
    std::map<std::string, std::string> settings;
    REQUIRE(read_loop_closure_capture(file, back, settings, why));
    std::filesystem::remove(file);
    REQUIRE(back.fine_time.has_value());
    CHECK(back.fine_time->curves.size() == capture.fine_time->curves.size());
    CHECK(loop_closure_replay_settings(settings, c.config).changed.empty());
    const LoopClosureReplayResult from_file = replay_loop_closure(back, c.config, logger);
    CHECK(from_file.identical_nodes == from_file.nodes);

    // the replay's JSON and per-window CSV
    const auto json = std::filesystem::temp_directory_path() / "lidar_odometry_tests_loop_fine_time.json";
    const auto csv = std::filesystem::temp_directory_path() / "lidar_odometry_tests_loop_fine_time.csv";
    REQUIRE(write_loop_closure_replay_json(json, own, LoopClosureSettingsDifference{}, c.config, why));
    REQUIRE(write_loop_fine_time_csv(csv, own.report.fine_time, why));
    std::string json_text, csv_text;
    {
        std::ifstream in(json);
        json_text.assign(std::istreambuf_iterator<char>(in), {});
        std::ifstream in2(csv);
        csv_text.assign(std::istreambuf_iterator<char>(in2), {});
    }
    std::filesystem::remove(json);
    std::filesystem::remove(csv);
    CHECK(json_text.find("\"fine_time\"") != std::string::npos);
    CHECK(json_text.find("\"rho_after_m\"") != std::string::npos);
    CHECK(csv_text.rfind("run,window,in_run,start_s,mid_s,offset_m,supported,train_patches,held_patches,matched_most,train_before_m,", 0) == 0);
    CHECK(std::count(csv_text.begin(), csv_text.end(), '\n') == 1 + 27);

    // another pose graph result (a kept edge's try rejected, so the graph is solved without it): refused, nothing applied
    QuietLogger graph_logger;
    const LoopClosureEdge &first_kept = report.kept.front();
    const LoopClosureReplayResult other_graph = replay_loop_closure(capture, c.config, graph_logger, {{first_kept.i, first_kept.j}});
    MESSAGE("without the try ", first_kept.i, "-", first_kept.j, ": ", other_graph.report.outcome, ", kept ", other_graph.report.kept.size(),
            ", largest segment correction ", other_graph.report.largest_segment_correction_m, " (run ", report.largest_segment_correction_m, ")");
    CHECK(other_graph.report.outcome == "fine_time_curves_unavailable");
    CHECK(other_graph.loops == 0);
    CHECK(same_bits(other_graph.windows, capture.windows));
    CHECK(contains(graph_logger.errors, "another pose graph result"));
    CHECK(contains(graph_logger.errors, "--from-snapshot"));

    // another min_sigma that makes the second run eligible: refused, nothing applied
    OdometryConfig sigma = c.config;
    sigma.loop_closure_fine_time_min_sigma_m = 0.5;
    QuietLogger sigma_logger;
    const LoopClosureReplayResult other_runs = replay_loop_closure(capture, sigma, sigma_logger);
    CHECK(other_runs.report.outcome == "fine_time_curves_unavailable");
    CHECK(same_bits(other_runs.windows, capture.windows));
    CHECK(contains(sigma_logger.errors, "other eligible runs"));

    // no eligible run at all: no curves asked for, the result of the graph alone (= the step off)
    OdometryConfig none = c.config;
    none.loop_closure_fine_time_min_sigma_m = 100.0;
    OdometryConfig off = c.config;
    off.loop_closure_fine_time_placement = false;
    const LoopClosureReplayResult without = replay_loop_closure(capture, none, logger);
    const LoopClosureReplayResult switched_off = replay_loop_closure(capture, off, logger);
    CHECK(without.report.outcome == "applied");
    CHECK(switched_off.report.outcome == "applied");
    CHECK(!switched_off.report.fine_time.ran);
    CHECK(same_bits(without.windows, switched_off.windows));
    CHECK(!same_bits(own.windows, switched_off.windows)); // the step moved nodes, and the replay reproduces that move

    // captured curves that don't fit what the run's sigma gives: refused as unavailable, never applied
    const auto refused_with = [&](const std::function<void(LoopFineTimeCurves &)> &damage, const std::string &cause)
    {
        LoopClosureCapture bad = capture;
        damage(bad.fine_time->curves[0]);
        QuietLogger bad_logger;
        const LoopClosureReplayResult r = replay_loop_closure(bad, c.config, bad_logger);
        CHECK_MESSAGE(r.report.outcome == "fine_time_curves_unavailable", cause);
        CHECK(same_bits(r.windows, capture.windows));
        CHECK_MESSAGE(contains(bad_logger.errors, cause), cause);
    };
    refused_with([](LoopFineTimeCurves &x) { x.grid[1] += 1e-15; }, "another offset grid");
    refused_with([](LoopFineTimeCurves &x) { x.bound_m = std::nextafter(x.bound_m, 100.0); }, "another offset grid");
    refused_with([](LoopFineTimeCurves &x) { x.grid.pop_back(); }, "another offset grid");
    refused_with([](LoopFineTimeCurves &x) { x.windows[3].training.pop_back(); }, "of another length than the grid");
    refused_with([](LoopFineTimeCurves &x) { x.windows[3].training_matched.clear(); }, "of another length than the grid");
    refused_with([](LoopFineTimeCurves &x) { x.windows[5].held_out.push_back(0.0); }, "of another length than the grid");
    refused_with([](LoopFineTimeCurves &x) { x.windows.pop_back(); }, "don't cover its support");

    // a capture without curves (an older format, or made with the step off) and the step on: refused
    LoopClosureCapture older = capture;
    older.fine_time.reset();
    QuietLogger older_logger;
    CHECK(replay_loop_closure(older, c.config, older_logger).report.outcome == "fine_time_curves_unavailable");
    CHECK(contains(older_logger.errors, "holds no fine-time score curves"));

    // the step without the readings rule: refused before anything is registered
    OdometryConfig no_readings = c.config;
    no_readings.loop_closure_odometry_from_readings = false;
    const LoopClosureReplayResult refused = replay_loop_closure(capture, no_readings, logger);
    CHECK(refused.report.outcome == "fine_time_without_readings");
    CHECK(refused.loops == 0);
    CHECK(same_bits(refused.windows, capture.windows));
}

TEST_CASE("loop fine time capture file: format 4 keeps the curves bit for bit; format 3 loads without them")
{
    Corridor c;
    build_corridor(c, {3.0, 6.5, 9.0, 13.0, 17.0}, nullptr, bump);
    LoopClosureCapture capture;
    capture.windows = c.odometry;
    capture.segments = c.segments;
    for (const Window &w : c.corrected)
        capture.result_poses.push_back(w.poses);
    LoopFineTimeCapture f;
    f.segment_poses = segment_poses(c);
    LoopFineTimeRun run;
    run.run = 1;
    run.first_window = 13;
    run.last_window = 27;
    run.support_first = 7;
    run.support_last = 33;
    run.overlap = true;
    run.sigma_m = 1.0 / 3.0;
    run.pin_begin_s = 2.8;
    run.pin_end_s = 13.6;
    run.axis_odometry = V3(1.0, 2.0, 3.0).normalized();
    run.axis = V3(-1.0, 2.0, 0.5).normalized();
    f.eligible = {run};
    LoopFineTimeCurves curves;
    curves.support_first = 7;
    curves.support_last = 8;
    curves.bound_m = 4.7;
    curves.grid = {-0.1, 0.0, 0.1};
    curves.reference_patches = 17;
    LoopFineTimeWindowCurves a, b;
    a.training_patches = 9;
    a.held_out_patches = 3;
    a.training = {1.0 / 7.0, 0.25, 1e-300};
    a.training_matched = {1, 9, 0};
    b.training_patches = 2;
    b.held_out = {0.5, 0.125, 1.0 / 9.0};
    b.held_out_matched = {4, 4, 4};
    curves.windows = {a, b};
    f.curves = {curves};
    capture.fine_time = f;
    CapturedLoopTry tried; // one try with a block per window of its 6-window source segment
    tried.registration.i = 0;
    tried.registration.j = 2;
    tried.registration.registered = true;
    tried.registration.window_information.assign(6, Eigen::Matrix3d::Identity());
    capture.tries = {tried};

    const auto file = std::filesystem::temp_directory_path() / "lidar_odometry_tests_loop_fine_time_format.bin";
    std::string why;
    REQUIRE(write_loop_closure_capture(file, capture, c.config, why));
    LoopClosureCapture back;
    std::map<std::string, std::string> settings;
    REQUIRE(read_loop_closure_capture(file, back, settings, why));
    REQUIRE(back.fine_time.has_value());
    const LoopFineTimeCapture &g = *back.fine_time;
    REQUIRE(g.segment_poses.size() == f.segment_poses.size());
    for (size_t k = 0; k < g.segment_poses.size(); k++)
        CHECK(g.segment_poses[k].matrix() == f.segment_poses[k].matrix());
    REQUIRE(g.eligible.size() == 1);
    CHECK(g.eligible[0].run == 1);
    CHECK(g.eligible[0].support_last == 33);
    CHECK(g.eligible[0].overlap);
    CHECK(g.eligible[0].sigma_m == run.sigma_m);
    CHECK(g.eligible[0].pin_end_s == run.pin_end_s);
    CHECK(g.eligible[0].axis == run.axis);
    CHECK(g.eligible[0].axis_odometry == run.axis_odometry);
    REQUIRE(g.curves.size() == 1);
    CHECK(g.curves[0].bound_m == 4.7);
    CHECK(g.curves[0].grid == curves.grid);
    CHECK(g.curves[0].reference_patches == 17);
    REQUIRE(g.curves[0].windows.size() == 2);
    CHECK(g.curves[0].windows[0].training == a.training);
    CHECK(g.curves[0].windows[0].training_matched == a.training_matched);
    CHECK(g.curves[0].windows[0].held_out.empty());
    CHECK(g.curves[0].windows[1].held_out == b.held_out);
    CHECK(g.curves[0].windows[1].held_out_matched == b.held_out_matched);
    CHECK(g.curves[0].windows[1].training_patches == 2);

    // format 3: format 4 without the block at the end (here the whole block)
    std::string bytes;
    {
        std::ifstream in(file, std::ios::binary);
        bytes.assign(std::istreambuf_iterator<char>(in), {});
    }
    LoopClosureCapture without = capture;
    without.fine_time.reset();
    REQUIRE(write_loop_closure_capture(file, without, c.config, why));
    std::string short_bytes;
    {
        std::ifstream in(file, std::ios::binary);
        short_bytes.assign(std::istreambuf_iterator<char>(in), {});
    }
    REQUIRE(short_bytes.size() + 1 < bytes.size());
    short_bytes.pop_back(); // the flag byte "no curves"
    const std::uint32_t three = 3;
    std::memcpy(&short_bytes[8], &three, sizeof(three));
    {
        std::ofstream out(file, std::ios::binary | std::ios::trunc);
        out.write(short_bytes.data(), static_cast<std::streamsize>(short_bytes.size()));
    }
    REQUIRE(read_loop_closure_capture(file, back, settings, why));
    CHECK(!back.fine_time.has_value());
    CHECK(back.windows.size() == capture.windows.size());

    // damaged counts are a damaged file (refused, nothing thrown): offsets from the fine-time flag at the end of the format 3 part
    const size_t flag = short_bytes.size();
    const size_t S = capture.segments.size();
    const size_t grid_count = flag + 1 + (8 + S * 128) + (8 + 93) + 8 + 4 + 4 + 8;
    const size_t windows_count = grid_count + 8 + 3 * 8 + 4;
    const size_t try_blocks = flag - 6 * 72 - 8;
    const auto damaged = [&](size_t at, std::uint64_t count, const char *what)
    {
        std::string d = bytes;
        std::memcpy(&d[at], &count, sizeof(count));
        {
            std::ofstream out(file, std::ios::binary | std::ios::trunc);
            out.write(d.data(), static_cast<std::streamsize>(d.size()));
        }
        LoopClosureCapture r;
        bool read = true;
        CHECK_NOTHROW(read = read_loop_closure_capture(file, r, settings, why));
        CHECK_MESSAGE(!read, what);
        MESSAGE(what, ": ", why);
    };
    {
        std::uint64_t stored = 0;
        std::memcpy(&stored, &bytes[windows_count], sizeof(stored));
        REQUIRE(stored == 2); // the offsets are right
        std::memcpy(&stored, &bytes[grid_count], sizeof(stored));
        REQUIRE(stored == 3);
        std::memcpy(&stored, &bytes[try_blocks], sizeof(stored));
        REQUIRE(stored == 6);
    }
    damaged(windows_count, 50000000ULL, "a curve's window count of 5e7");
    damaged(windows_count, capture.windows.size() + 1, "more curve windows than the capture has");
    damaged(grid_count, 1000000000ULL, "a grid of 1e9 offsets");
    damaged(grid_count, 200, "a grid longer than the rule makes");
    damaged(try_blocks, 7, "7 blocks for a 6-window segment");
    damaged(try_blocks, 50000000ULL, "5e7 blocks of a try");
    std::filesystem::remove(file);
}

TEST_CASE("loop fine time replay: a pose graph result with another number of segment poses is another one, also with none")
{
    LoopFineTimeCapture captured;
    captured.segment_poses = {Eigen::Affine3d::Identity(), Eigen::Affine3d::Identity()};
    CapturedFineTimeScores scores(&captured);
    const std::vector<Window> windows;
    const std::vector<std::vector<Eigen::Affine3d>> odometry;
    const std::vector<LoopFineTimeRun> eligible; // the same (none) in both
    for (const size_t n : {size_t(0), size_t(1), size_t(3)})
    {
        const std::vector<Eigen::Affine3d> poses(n, Eigen::Affine3d::Identity());
        std::vector<LoopFineTimeCurves> out;
        std::string why;
        CHECK_MESSAGE(!scores.curves({windows, odometry, poses, eligible}, out, why), n, " poses");
        CHECK(why.find("another pose graph result") != std::string::npos);
    }
    const std::vector<Eigen::Affine3d> same(2, Eigen::Affine3d::Identity());
    std::vector<LoopFineTimeCurves> out;
    std::string why;
    CHECK(scores.curves({windows, odometry, same, eligible}, out, why));
}

TEST_CASE("loop fine time edges: a source window is blind in any eligible run's support, and every window of a run without scores is")
{
    // run A (13-27, support 7-33) is scored; runs B (40-46, support 34-52) and C (55-60, support 49-66) overlap, so neither
    // has scores and every window of their supports is blind. The edge's source segment 5 (windows 30-35) spans A's support
    // (30-33) and B's (34, 35); its information along x lies in windows 34 and 35 only.
    Corridor c;
    build_corridor(c, {3.0, 6.5, 9.0, 13.0, 17.0}, nullptr, bump, {{13, 27}, {40, 46}, {55, 60}});
    REQUIRE(c.runs.size() == 3);
    for (const LoopOdometryRun &r : c.runs)
        REQUIRE(r.sigma_m >= 1.0);
    const ConstructedEdge edge = constructed_edge(c, 12, 5, Eigen::Affine3d::Identity(), {34, 35});
    std::vector<Window> windows = c.corrected;
    const LoopFineTimeReport report = step(c, windows, {edge.edge}, {&edge.registration});
    REQUIRE(report.runs.size() == 3);
    CHECK(report.runs[0].scored);
    CHECK(report.runs[1].cause == "supports overlap");
    CHECK(report.runs[2].cause == "supports overlap");
    const LoopFineTimeEdgeRow *along_a = nullptr;
    for (const LoopFineTimeEdgeRow &row : report.edges)
        if (row.run == 0)
            along_a = &row;
    REQUIRE(along_a);
    MESSAGE("edge 12 -> 5 along run A: informed ", along_a->informed_share, ", ", std::string(fine_time_edge_class_name(along_a->cls)));
    CHECK(along_a->informed_share == 0.0); // windows 34 and 35 are blind (run B has no scores), the others carry nothing
    CHECK(along_a->cls == FineTimeEdgeClass::unconfirmed);
}
