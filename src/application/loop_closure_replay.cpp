#include "loop_closure_replay.h"
#include "angle_units.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <tuple>

namespace
{
    // the registrations of a capture, looked up by pair; a pair it doesn't hold is noted as missing
    class CapturedRegistrations : public LoopClosureRegistrations
    {
    public:
        CapturedRegistrations(const LoopClosureCapture &capture, const std::vector<std::pair<int, int>> &reject)
            : capture_(capture), reject_(reject.begin(), reject.end())
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

        bool rejected(int i, int j, bool) override
        {
            if (reject_.count({i, j}) == 0)
                return false;
            asked.insert({i, j});
            return true;
        }

        std::vector<std::string> missing;
        std::set<std::pair<int, int>> asked; // the tries named for rejection that close_loops asked for

    private:
        const LoopClosureCapture &capture_;
        std::set<std::pair<int, int>> reject_;
        std::map<std::tuple<int, int, bool>, size_t> index_;
    };

    bool same_segment(const LoopClosureSegment &a, const LoopClosureSegment &b)
    {
        return a.first_window == b.first_window && a.last_window == b.last_window && a.pose.matrix() == b.pose.matrix() && a.center == b.center &&
               a.path_begin == b.path_begin && a.path_end == b.path_end && a.t_begin == b.t_begin && a.t_end == b.t_end;
    }
} // namespace

bool parse_loop_try_names(const std::string &text, std::vector<std::pair<int, int>> &tries, std::string &why)
{
    tries.clear();
    std::stringstream list(text);
    std::string item;
    while (std::getline(list, item, ','))
    {
        int i = 0, j = 0;
        char dash = 0;
        std::istringstream in(item);
        if (!(in >> i >> dash >> j) || dash != '-' || i < 0 || j < 0 || !(in >> std::ws).eof())
        {
            why = "'" + item + "' is not a try i-j";
            return false;
        }
        tries.emplace_back(i, j);
    }
    if (tries.empty())
    {
        why = "no try named";
        return false;
    }
    return true;
}

std::string loop_try_name(const std::pair<int, int> &t)
{
    return std::to_string(t.first) + "-" + std::to_string(t.second);
}

LoopClosureReplayResult replay_loop_closure(const LoopClosureCapture &capture, const OdometryConfig &params, Logger &logger,
                                            const std::vector<std::pair<int, int>> &reject)
{
    LoopClosureReplayResult result;
    result.windows = capture.windows;
    const std::vector<LoopClosureSegment> segments = loop_closure_segments(result.windows, params);
    result.segments_match = segments.size() == capture.segments.size();
    for (size_t k = 0; result.segments_match && k < segments.size(); k++)
        result.segments_match = same_segment(segments[k], capture.segments[k]);
    if (!result.segments_match)
        return result;

    for (const auto &t : reject)
    {
        const bool held = std::any_of(capture.tries.begin(), capture.tries.end(),
                                      [&](const CapturedLoopTry &c) { return c.registration.i == t.first && c.registration.j == t.second; });
        if (!held && std::find(result.rejected_not_in_capture.begin(), result.rejected_not_in_capture.end(), loop_try_name(t)) ==
                         result.rejected_not_in_capture.end())
            result.rejected_not_in_capture.push_back(loop_try_name(t));
    }
    if (!result.rejected_not_in_capture.empty())
        return result;

    CapturedRegistrations registrations(capture, reject);
    CapturedFineTimeScores fine_time(capture.fine_time ? &*capture.fine_time : nullptr);
    result.loops = close_loops(result.windows, segments, registrations, params, logger, &result.report, &fine_time);
    result.missing = registrations.missing;
    for (const auto &t : std::set<std::pair<int, int>>(reject.begin(), reject.end()))
        (registrations.asked.count(t) ? result.rejected : result.rejected_not_asked).push_back(loop_try_name(t));

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
