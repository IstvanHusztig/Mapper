#include <doctest/doctest.h>

#include "synthetic_recording.h"

#include <algorithm>
#include <chrono>

// the loop recording the loop-closure and consistency tests run on: its points lie on the scene, map to their own node
// by timestamp as the consistency pass maps them, and the second lap passes every place of the first again

using namespace synthetic;

TEST_CASE("loop recording: points on the scene, timestamps map to their node, windows filled, laps revisit")
{
    const auto start = std::chrono::steady_clock::now();
    const Recording r = loop_recording(2);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

    size_t total = 0, fewest = SIZE_MAX, most = 0;
    for (const auto &w : r.points)
    {
        total += w.size();
        fewest = std::min(fewest, w.size());
        most = std::max(most, w.size());
    }
    MESSAGE("loop recording: ", r.windows.size(), " windows, ", all_poses(r.windows).size(), " nodes, ", total, " points (",
            fewest, "..", most, " per window), lap ", r.lap_time_s, " s, built in ", ms, " ms");
    CHECK(fewest >= 2000);
    CHECK(most <= 6000);

    // every point lies on a face within 1 cm when placed with its true pose
    double farthest = 0.0;
    for (size_t w = 0; w < r.points.size(); w++)
        for (const Point3Di &p : r.points[w])
            farthest = std::max(farthest, distance_to_scene(r.scene, r.truth[w][p.index_pose] * p.point));
    CHECK(farthest <= 0.01);

    // the consistency pass assigns a point to the last node whose timestamp is below the point's
    std::vector<double> node_times;
    for (const Window &w : r.windows)
        for (const auto &t : w.timestamps)
            node_times.push_back(t.first);
    int mismatches = 0;
    for (size_t w = 0; w < r.points.size(); w++)
        for (const Point3Di &p : r.points[w])
        {
            const auto lower = std::lower_bound(node_times.begin(), node_times.end(), p.timestamp);
            const long node = static_cast<long>(std::distance(node_times.begin(), lower)) - 1;
            if (node != static_cast<long>(w * r.windows[w].poses.size() + p.index_pose))
                mismatches++;
        }
    CHECK(mismatches == 0);

    // the path is continuous: 0.1 m between nodes at 2 m/s, and at most 1 rad/s * 50 ms of turn in the corners
    const auto truth = all_poses(r.truth);
    double longest_step = 0.0, largest_turn = 0.0;
    for (size_t k = 1; k < truth.size(); k++)
    {
        longest_step = std::max(longest_step, (truth[k].translation() - truth[k - 1].translation()).norm());
        largest_turn = std::max(largest_turn, Eigen::AngleAxisd(truth[k].linear() * truth[k - 1].linear().transpose()).angle());
    }
    CHECK(longest_step <= 0.1 + 1e-9);
    CHECK(largest_turn <= 0.05 + 1e-9);

    // every node of the first lap has a node of the second lap within 0.1 m, more than 10 s later
    const size_t lap_nodes = static_cast<size_t>(r.lap_time_s / r.node_dt);
    double worst = 0.0;
    for (size_t i = 0; i < lap_nodes; i++)
    {
        double nearest = 1e9;
        for (size_t j = lap_nodes; j < truth.size(); j++)
            nearest = std::min(nearest, (truth[j].translation() - truth[i].translation()).norm());
        if (i + lap_nodes < truth.size())
            worst = std::max(worst, nearest);
    }
    CHECK(r.lap_time_s > 10.0);
    CHECK(worst <= 0.1);
}
