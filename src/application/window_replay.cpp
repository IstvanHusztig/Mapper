#include "window_replay.h"
#include "angle_units.h"
#include "memory_point_store.h"
#include "window_loop.h"

#include <algorithm>

namespace
{
    // a replay writes no preview and has no progress to show
    class NoPreview : public PreviewSink
    {
    public:
        void calibration_validation(const std::vector<Point3Di> &, std::size_t, Logger &) override {}
        void window_cloud(std::size_t, const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &,
                          const std::vector<double> &) override {}
        std::string location() const override { return ""; }
    };

    class NoProgress : public ProgressReporter
    {
    public:
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };
}

WindowReplayResult replay_window_capture(const WindowCapture &capture, const OdometryConfig &params, bool same_settings, Logger &logger,
                                         WindowTermSink *terms)
{
    // windows 0 .. min(last + 1, n - 1): the run's first window (the loop's settings and the diagnostics' time origin read
    // it), the captured ones, empty windows between (never read)
    std::vector<Window> windows(capture.windows_begin + capture.windows.size());
    windows[0] = capture.first_window;
    std::copy(capture.windows.begin(), capture.windows.end(), windows.begin() + capture.windows_begin);

    MemoryPointStore points;
    for (std::size_t i = capture.first; i <= capture.last; i++)
        points.save(PointSet::WindowIntermediate, i, capture.points[i - capture.first]);

    OdometryState state = capture.state;
    WindowLoop loop;
    start_window_loop(loop, capture.first_window, params, logger);
    loop.carry = capture.carry;

    std::vector<int> forced;
    for (const WindowOutcome &o : capture.outcomes)
        forced.push_back(o.last_iteration);
    WindowReplayResult result;
    WindowRange range;
    range.outcomes = &result.outcomes;
    range.terms = terms;
    if (same_settings)
        range.forced_last_iteration = &forced;

    NoPreview preview;
    NoProgress progress;
    double ts_failure = 0.0;
    result.cancelled = !process_windows(loop, windows, capture.first, capture.last, params, state, ts_failure, false, points, preview,
                                        logger, progress, range);

    for (std::size_t k = 0; k < result.outcomes.size() && k < capture.outcomes.size(); k++)
    {
        const WindowOutcome &replayed = result.outcomes[k], &captured = capture.outcomes[k];
        result.iteration_differences.push_back(replayed.last_iteration - captured.last_iteration);
        bool same = replayed.poses.size() == captured.poses.size();
        for (std::size_t n = 0; n < std::min(replayed.poses.size(), captured.poses.size()); n++)
        {
            const Eigen::Affine3d &a = replayed.poses[n], &b = captured.poses[n];
            same = same && a.matrix() == b.matrix();
            result.largest_position_difference_m = std::max(result.largest_position_difference_m, (a.translation() - b.translation()).norm());
            const double angle = Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle() * RAD_TO_DEG;
            result.largest_rotation_difference_deg = std::max(result.largest_rotation_difference_deg, angle);
        }
        if (same)
            result.identical++;
        else if (result.first_different_window < 0)
            result.first_different_window = static_cast<int>(capture.first + k);
    }
    return result;
}
