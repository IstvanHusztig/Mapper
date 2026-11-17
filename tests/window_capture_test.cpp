#include <doctest/doctest.h>

#include "memory_point_store.h"
#include "odometry_snapshot_file.h"
#include "run_odometry.h"
#include "synthetic_recording.h"
#include "window_capture_file.h"
#include "window_replay.h"
#include "window_term_csv.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>

// Window capture and replay (stage 1 of docs/code-split-and-region-agents-plan.md) on one lap of the loop recording (see
// synthetic_recording.h): the capture doesn't change the run, the file gives back the capture, and a replay with the same
// settings gives the run's windows bit for bit.

using namespace synthetic;
namespace fs = std::filesystem;

namespace
{
    class NoPreview : public PreviewSink
    {
    public:
        void calibration_validation(const std::vector<Point3Di> &, std::size_t, Logger &) override {}
        void window_cloud(std::size_t, const std::vector<Eigen::Vector3d> &, const std::vector<unsigned short> &,
                          const std::vector<double> &) override {}
        std::string location() const override { return "none"; }
    };

    class NoProgress : public ProgressReporter
    {
    public:
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    };

    class KeepCapture : public WindowCaptureSink
    {
    public:
        bool write(const WindowCapture &capture, std::string &) override
        {
            kept = capture;
            written = true;
            return true;
        }
        WindowCapture kept;
        bool written = false;
    };

    OdometryConfig capture_config()
    {
        OdometryConfig c;
        c.nr_iter = 5;                           // enough to move every window; the test is about reproducing, not accuracy
        c.real_time_threshold_seconds = 1000.0;  // the time limit must not end a window here
        c.in_out_params_indoor.resolution_X = c.in_out_params_indoor.resolution_Y = c.in_out_params_indoor.resolution_Z = 0.3;
        return c;
    }

    // the window loop over the recording; the map is seeded from window 0's points, placed with its true poses
    std::vector<Window> run(const Recording &r, const OdometryConfig &c, WindowCaptureSink *capture, WindowTermSink *terms = nullptr)
    {
        MemoryPointStore store;
        for (size_t w = 0; w < r.points.size(); w++)
            store.save(PointSet::WindowIntermediate, w, r.points[w]);
        OdometryState state;
        state.m_g = r.truth[0][0];
        for (const Point3Di &p : r.points[0])
        {
            Point3Di q = p;
            q.point = r.truth[0][0].inverse() * (r.truth[0][p.index_pose] * p.point);
            state.initial_points.push_back(q);
        }
        std::vector<Window> windows = r.windows;
        NoPreview preview;
        NoProgress progress;
        NullLogger logger;
        double ts_failure = 0.0;
        CHECK(optimize_windows(windows, c, state, ts_failure, false, store, preview, logger, progress, capture, terms));
        return windows;
    }

    class KeepTerms : public WindowTermSink
    {
    public:
        void write(const WindowTermRecord &record) override { records.push_back(record); }
        std::vector<WindowTermRecord> records;
    };

    bool same_poses(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b)
    {
        if (a.size() != b.size())
            return false;
        for (size_t i = 0; i < a.size(); i++)
            if (a[i].matrix() != b[i].matrix())
                return false;
        return true;
    }
}

TEST_CASE("window capture: the capture leaves the run unchanged, and a replay with the same settings repeats it bit for bit")
{
    const Recording r = loop_recording(1);
    OdometryConfig c = capture_config();
    const std::vector<Window> plain = run(r, c, nullptr);

    c.capture_windows_from_s = 5.0;
    c.capture_windows_to_s = 12.0;
    KeepCapture sink;
    const std::vector<Window> captured_run = run(r, c, &sink);
    REQUIRE(sink.written);
    const WindowCapture &capture = sink.kept;
    MESSAGE("windows ", capture.first + 1, " to ", capture.last + 1, " of ", capture.n_windows, " captured");
    CHECK(capture.first == 5);
    CHECK(capture.last == 12);
    REQUIRE(capture.outcomes.size() == capture.last - capture.first + 1);

    // the run with the capture is the run without it, and the capture holds its results
    for (size_t i = 0; i < plain.size(); i++)
        CHECK(same_poses(plain[i].poses, captured_run[i].poses));
    for (size_t k = 0; k < capture.outcomes.size(); k++)
        CHECK(same_poses(capture.outcomes[k].poses, captured_run[capture.first + k].poses));

    // through the file
    const fs::path file = fs::temp_directory_path() / "lidar_odometry_tests_window_capture.bin";
    std::string why;
    REQUIRE(write_window_capture(file, capture, c, why));
    WindowCapture read;
    std::uint64_t hash = 0;
    REQUIRE(read_window_capture(file, read, hash, why));
    fs::remove(file);
    CHECK(hash == odometry_settings_hash(c));

    // the replay with the same settings: every window as in the run
    NullLogger logger;
    const WindowReplayResult same = replay_window_capture(read, c, true, logger);
    CHECK_FALSE(same.cancelled);
    CHECK(same.identical == capture.outcomes.size());
    CHECK(same.largest_position_difference_m == 0.0);

    // with other settings it runs too, and shows the difference (here fewer iterations)
    OdometryConfig fewer = c;
    fewer.nr_iter = 1;
    const WindowReplayResult other = replay_window_capture(read, fewer, false, logger);
    CHECK_FALSE(other.cancelled);
    CHECK(other.outcomes.size() == capture.outcomes.size());
    CHECK(other.identical < capture.outcomes.size());
    MESSAGE("replay with 1 iteration instead of 5: largest difference ", other.largest_position_difference_m * 1000.0, " mm");
}

TEST_CASE("window capture: the range is the windows that start within it")
{
    std::vector<Window> windows(5);
    for (size_t i = 0; i < windows.size(); i++)
        windows[i].timestamps = {{100.0 + 2.0 * i, 0.0}};
    std::size_t first = 0, last = 0;
    CHECK(capture_range(windows, 1.0, 5.0, first, last));
    CHECK(first == 1); // starts at 2 s
    CHECK(last == 2);  // starts at 4 s
    CHECK(capture_range(windows, 0.0, 100.0, first, last));
    CHECK((first == 0 && last == 4));
    CHECK_FALSE(capture_range(windows, 4.5, 5.5, first, last)); // no window starts there
    CHECK_FALSE(capture_range(windows, -1.0, -1.0, first, last)); // off
    CHECK_FALSE(capture_range(windows, 3.0, 2.0, first, last));   // off
}

// The per-term window table (log_window_terms, stage 3): it only observes, so the run is the same bits with it on, and the
// setting is not one of the odometry settings a capture or snapshot is checked against.
TEST_CASE("window term table: the run is the same with log_window_terms on, every window gets its tables, and the files hold them")
{
    const Recording r = loop_recording(1);
    OdometryConfig c = capture_config();
    const std::vector<Window> plain = run(r, c, nullptr);

    OdometryConfig with_terms = c;
    with_terms.log_window_terms = true;
    CHECK(odometry_settings_hash(with_terms) == odometry_settings_hash(c));
    KeepTerms sink;
    const std::vector<Window> logged = run(r, with_terms, nullptr, &sink);
    for (size_t i = 0; i < plain.size(); i++)
        CHECK(same_poses(plain[i].poses, logged[i].poses));

    REQUIRE(sink.records.size() == plain.size());
    size_t rows = 0;
    for (const WindowTermRecord &record : sink.records)
    {
        CAPTURE(record.window);
        CHECK(record.guess.failure.empty());
        CHECK(record.result.failure.empty());
        CHECK_FALSE(record.result.rows.empty());
        CHECK(record.result.rows.back().name == "damping");
        rows += record.guess.rows.size() + record.result.rows.size();
    }

    // the CSV files: a header and a row per window, state and term; a row per window
    // into a folder that doesn't exist yet, as a CLI run's output folder during the odometry
    const fs::path root = fs::temp_directory_path() / "lidar_odometry_tests_window_terms";
    fs::remove_all(root);
    const fs::path folder = root / "result";
    {
        FolderWindowTermSink files(folder, "t_");
        for (const WindowTermRecord &record : sink.records)
            files.write(record);
        CHECK(files.ok());
    }
    const auto lines = [](const fs::path &file)
    {
        std::ifstream in(file);
        std::vector<std::string> out;
        for (std::string line; std::getline(in, line);)
            out.push_back(line);
        return out;
    };
    const auto columns = [](const std::string &line) { return std::count(line.begin(), line.end(), ',') + 1; };
    const std::vector<std::string> terms = lines(folder / "t_window_terms.csv"), features = lines(folder / "t_window_features.csv");
    fs::remove_all(root);
    REQUIRE(terms.size() == 1 + rows);
    REQUIRE(features.size() == 1 + sink.records.size());
    CHECK(terms[0].rfind("window,time_s,state,term,in_total,cost,residuals,weight_shift_along", 0) == 0);
    CHECK(columns(terms[0]) == 7 + 2 * 12);
    CHECK(columns(terms[1]) == columns(terms[0]));
    CHECK(columns(features[1]) == columns(features[0]));
    CHECK(features[0].find("change_turn_vertical") != std::string::npos);
}
