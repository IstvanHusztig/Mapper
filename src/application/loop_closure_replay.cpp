#include "loop_closure_replay.h"
#include "angle_units.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <tuple>

namespace
{
    // the registrations of a capture, looked up by pair; a pair it doesn't hold is noted as missing
    class CapturedRegistrations : public LoopClosureRegistrations
    {
    public:
        explicit CapturedRegistrations(const LoopClosureCapture &capture) : capture_(capture)
        {
            for (size_t k = 0; k < capture.tries.size(); k++)
            {
                const LoopClosureTry &t = capture.tries[k].registration;
                index_.emplace(std::make_tuple(t.i, t.j, t.cross_floor), k);
            }
        }

        const LoopClosureTry *registration(int i, int j, bool cross_floor) override
        {
            const auto key = std::make_tuple(i, j, cross_floor);
            if (const auto it = index_.find(key); it != index_.end())
                return &capture_.tries[it->second].registration;
            const std::string name = std::string(cross_floor ? "cross-floor " : "") + "segments " + std::to_string(i) + " -> " + std::to_string(j);
            if (std::find(missing.begin(), missing.end(), name) == missing.end())
                missing.push_back(name);
            return nullptr;
        }

        std::vector<std::string> missing;

    private:
        const LoopClosureCapture &capture_;
        std::map<std::tuple<int, int, bool>, size_t> index_;
    };

    bool same_segment(const LoopClosureSegment &a, const LoopClosureSegment &b)
    {
        return a.first_window == b.first_window && a.last_window == b.last_window && a.pose.matrix() == b.pose.matrix() && a.center == b.center &&
               a.path_begin == b.path_begin && a.path_end == b.path_end && a.t_begin == b.t_begin && a.t_end == b.t_end;
    }
} // namespace

LoopClosureReplayResult replay_loop_closure(const LoopClosureCapture &capture, const OdometryConfig &params, Logger &logger)
{
    LoopClosureReplayResult result;
    result.windows = capture.windows;
    const std::vector<LoopClosureSegment> segments = loop_closure_segments(result.windows, params);
    result.segments_match = segments.size() == capture.segments.size();
    for (size_t k = 0; result.segments_match && k < segments.size(); k++)
        result.segments_match = same_segment(segments[k], capture.segments[k]);
    if (!result.segments_match)
        return result;

    CapturedRegistrations registrations(capture);
    result.loops = close_loops(result.windows, segments, registrations, params, logger, &result.report);
    result.missing = registrations.missing;

    for (size_t w = 0; w < result.windows.size() && w < capture.result_poses.size(); w++)
    {
        const auto &poses = result.windows[w].poses;
        const auto &expected = capture.result_poses[w];
        for (size_t n = 0; n < poses.size() && n < expected.size(); n++)
        {
            if (poses[n].matrix() == expected[n].matrix())
                result.identical_nodes++;
            result.largest_position_difference_m =
                std::max(result.largest_position_difference_m, (poses[n].translation() - expected[n].translation()).norm());
            const double angle = Eigen::AngleAxisd(expected[n].linear().transpose() * poses[n].linear()).angle();
            result.largest_rotation_difference_deg = std::max(result.largest_rotation_difference_deg, angle * RAD_TO_DEG);
        }
    }
    // a node the capture's result lacks (or has in excess) counts as different
    size_t expected_nodes = 0;
    for (const auto &poses : capture.result_poses)
        expected_nodes += poses.size();
    size_t replay_nodes = 0;
    for (const auto &w : result.windows)
        replay_nodes += w.poses.size();
    result.nodes = std::max(expected_nodes, replay_nodes);
    return result;
}

std::vector<TrajectoryChunk> trajectory_chunks(const std::vector<Window> &windows, const OdometryConfig &params)
{
    // the grouping of save_result (lidar_odometry.cpp): a chunk is closed after its 51st window
    std::vector<TrajectoryChunk> chunks;
    std::vector<Eigen::Affine3d> poses;
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Eigen::Vector3d> angles;
    const auto close = [&]()
    {
        TrajectoryChunk c;
        c.first_pose = poses.front();
        for (size_t k = 0; k < poses.size(); k++)
            c.rows.push_back({timestamps[k], poses[0].inverse() * poses[k], angles[k]});
        chunks.push_back(std::move(c));
        poses.clear();
        timestamps.clear();
        angles.clear();
    };
    int counter = 0;
    for (const Window &w : windows)
    {
        poses.insert(poses.end(), w.poses.begin(), w.poses.end());
        timestamps.insert(timestamps.end(), w.timestamps.begin(), w.timestamps.end());
        angles.insert(angles.end(), w.imu_angles.begin(), w.imu_angles.end());
        counter++;
        if (counter > 50)
        {
            close();
            counter = 0;
        }
    }
    if (counter > params.min_counter_concatenated_trajectory_nodes && !poses.empty())
        close();
    return chunks;
}
