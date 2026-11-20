#include <doctest/doctest.h>

#include "angle_units.h"
#include "memory_point_store.h"
#include "odometry_snapshot_file.h"
#include "run_odometry.h"
#include "synthetic_recording.h"
#include "window_capture_file.h"
#include "window_replay.h"
#include "window_term_csv.h"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <limits>
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

// Loop guard of stage 4 (docs/code-split-and-region-agents-plan.md): the window loop's own logic (damping, convergence,
// lm_step_check with its rejections and final cost-only call), which neither the replays and baselines (step check off)
// nor the characterization guard (single iterations) cover. Runs the synthetic loop recording in four variants and
// writes every window's outcome and state as exact hexadecimal doubles. Not a regression test of values (the bits may
// differ between compilers and configurations): it is recorded before the stage-4 refactor and compared after every
// commit. Run: lidar_odometry_tests -tc="window loop guard*" --no-skip; the file is LO_LOOP_GUARD_OUT, or
// window_loop_guard.txt in the temp folder.
TEST_CASE("window loop guard: every window's outcome over the loop's option branches, as exact bits" * doctest::skip())
{
    const Recording r = loop_recording(1);
    struct Variant
    {
        const char *name;
        bool step_check;
        double lm_factor_initial;
        double convergence_delta_threshold;
    };
    const Variant variants[] = {{"plain", false, 5.0, 1e-12},
                                {"step check", true, 5.0, 1e-12},
                                {"step check, weak damping, early stop", true, 1e-4, 1e-3},
                                {"early stop", false, 1e-4, 1e-3}};
    const char *env = std::getenv("LO_LOOP_GUARD_OUT");
    const fs::path file = env ? fs::path(env) : fs::temp_directory_path() / "window_loop_guard.txt";
    std::ofstream out(file);
    out << std::hexfloat;
    for (const Variant &v : variants)
    {
        OdometryConfig c = capture_config();
        c.nr_iter = 12; // past iteration 10, where the damping grows by schedule
        c.lm_step_check = v.step_check;
        c.lm_factor_initial = v.lm_factor_initial;
        c.convergence_delta_threshold = v.convergence_delta_threshold;
        c.capture_windows_from_s = 0.0; // every window's outcome, through the capture
        c.capture_windows_to_s = 1e9;
        KeepCapture sink;
        const std::vector<Window> windows = run(r, c, &sink);
        REQUIRE(sink.written);
        int rejected = 0, early = 0;
        out << "variant " << v.name << "\n";
        for (size_t k = 0; k < sink.kept.outcomes.size(); k++)
        {
            const WindowOutcome &o = sink.kept.outcomes[k];
            const Window &w = windows[sink.kept.first + k];
            rejected += o.rejected_steps;
            early += o.last_iteration < c.nr_iter - 1;
            out << "window " << sink.kept.first + k << " last_iteration " << o.last_iteration << " rejected " << o.rejected_steps << "\n";
            for (const Eigen::Affine3d &p : o.poses)
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 4; j++)
                        out << p(i, j) << (i == 2 && j == 3 ? "\n" : " ");
            out << "velocity " << w.velocity.start.transpose() << " " << w.velocity.end.transpose() << " end_cov "
                << w.velocity.end_covariance.reshaped().transpose() << "\n";
            out << "gyro_bias " << w.gyro_bias.mean.transpose() << " cov " << w.gyro_bias.covariance.reshaped().transpose() << "\n";
        }
        MESSAGE(std::string(v.name), ": ", sink.kept.outcomes.size(), " windows, ", rejected, " steps rejected, ", early, " ended before the iteration limit");
    }
    MESSAGE("loop guard written to ", file.string());
}

// The window verdict (stage 4) only observes: the run is the same bits with log_window_verdict on, and the setting is not
// one of the odometry settings a capture or snapshot is checked against.
TEST_CASE("window verdict: the run is the same with log_window_verdict on")
{
    const Recording r = loop_recording(1);
    OdometryConfig c = capture_config();
    const std::vector<Window> plain = run(r, c, nullptr);
    OdometryConfig with_lines = c;
    with_lines.log_window_verdict = true;
    CHECK(odometry_settings_hash(with_lines) == odometry_settings_hash(c));
    const std::vector<Window> logged = run(r, with_lines, nullptr);
    for (size_t i = 0; i < plain.size(); i++)
        CHECK(same_poses(plain[i].poses, logged[i].poses));
}

// change honest-inertial-carry: the three carry settings are in the hash only away from their defaults, so captures and
// snapshots written before they existed still count as made with the same settings
TEST_CASE("settings hash: the honest-carry settings change it only away from their defaults")
{
    const OdometryConfig c = capture_config();
    const std::uint64_t base = odometry_settings_hash(c);
    OdometryConfig edges = c, joint = c, window = c;
    edges.imu_edges_follow_state = true;
    joint.imu_carry_joint_covariance = true;
    window.imu_accel_window_sigma_mps2 = 0.05;
    CHECK(odometry_settings_hash(edges) != base);
    CHECK(odometry_settings_hash(joint) != base);
    CHECK(odometry_settings_hash(window) != base);
    OdometryConfig back = window;
    back.imu_accel_window_sigma_mps2 = 0.0;
    CHECK(odometry_settings_hash(back) == base);
}

// format 2 added the carry's velocity-offset cross block; a format-1 file (written before it) still reads, with a zero
// block, and still replays bit for bit with its own settings
TEST_CASE("window capture: a file of the previous format reads and replays")
{
    const Recording r = loop_recording(1);
    OdometryConfig c = capture_config();
    c.capture_windows_from_s = 5.0;
    c.capture_windows_to_s = 12.0;
    KeepCapture sink;
    run(r, c, &sink);
    REQUIRE(sink.written);

    const fs::path file = fs::temp_directory_path() / "lidar_odometry_tests_window_capture_v1.bin";
    std::string why;
    REQUIRE(write_window_capture(file, sink.kept, c, why, 1));
    WindowCapture read;
    std::uint64_t hash = 0;
    REQUIRE(read_window_capture(file, read, hash, why));
    fs::remove(file);
    CHECK(hash == odometry_settings_hash(c));
    CHECK(read.carry.velocity_offset_cov == Eigen::Matrix3d::Zero());

    NullLogger logger;
    const WindowReplayResult same = replay_window_capture(read, c, true, logger);
    CHECK_FALSE(same.cancelled);
    CHECK(same.identical == sink.kept.outcomes.size());
}

namespace
{
    // a record with made-up values in every field the features file had before the handed-on state (consistency-harness
    // 3.1), some readings absent
    WindowTermRecord made_up_record()
    {
        WindowTermRecord r;
        r.window = 7;
        r.time_s = 12.5;
        r.duration_s = 0.375;
        r.iterations = 11;
        r.delta = 2.5e-7;
        r.lm_factor = 0.125;
        r.at_rest = false;
        r.turn_window = true;
        r.acc_term = true;
        r.guess_move = Eigen::Vector3d(0.25, -0.5, 0.0625);
        r.result_move = Eigen::Vector3d(0.3, -0.45, 0.07);
        r.velocity_start = Eigen::Vector3d(0.7, -1.1, 0.02);
        r.velocity_end = Eigen::Vector3d(0.8, -1.2, 0.03);
        r.gyro_bias_deg_s = Eigen::Vector3d(0.01, -0.02, 0.15);
        r.acc_offset_mps2 = Eigen::Vector3d(0.004, 0.005, -0.006);
        for (int m = 0; m < kTermMotions; m++)
            for (int d = 0; d < kTermDirections; d++)
                r.change[m][d] = 0.5 + m * kTermDirections + d;
        r.change[3][2] = std::numeric_limits<double>::quiet_NaN();
        r.result.directions = {Eigen::Vector3d(0.6, 0.8, 0.0), Eigen::Vector3d(-0.8, 0.6, 0.0), Eigen::Vector3d::UnitZ()};
        r.result.translation_information = Eigen::Vector3d(4.0, 2.0, 1.0).asDiagonal();
        r.result.ndt_matched = 1234;
        r.result.ndt_unmatched = 56;
        r.result.turn_defined = false;
        r.guess.failure = "made up";
        WindowEvidence &e = r.verdict.evidence;
        e.heading_bend_deg = 0.75;
        e.roll_bend_deg = -0.125;
        e.speed_mps = 1.5;
        e.velocity_chi2 = 3.25;
        e.stop = LmStop::converged;
        e.rejected_steps = 2;
        e.steps_not_applied = 1;
        e.big_jump_nodes = 3;
        e.lidar_share_along = 0.875;
        r.verdict.bent = Check::clear;
        r.verdict.fast = Check::trigger;
        WindowNoiseSigmas &n = r.noise;
        n.translation_m = Eigen::Vector3d::Constant(1e-4);
        n.rotation_min_deg = Eigen::Vector3d(0.01, 0.02, 0.03);
        n.rotation_median_deg = Eigen::Vector3d(0.011, 0.021, 0.031);
        n.rotation_max_deg = Eigen::Vector3d(0.012, 0.022, 0.032);
        n.accelerometer_min_mps = 0.004;
        n.accelerometer_max_mps = 0.006;
        n.velocity_prior_mps = Eigen::Vector3d(0.0046, 0.005, 0.0052);
        n.bias_prior_deg_s = Eigen::Vector3d(0.3, 0.31, 0.32);
        n.offset_prior_mps2 = Eigen::Vector3d(0.07, 0.071, 0.072);
        n.ndt_information_scale = 0.1;
        n.ndt_huber_cutoff = 3.0;
        return r;
    }

    std::vector<std::string> file_lines(const fs::path &file)
    {
        std::ifstream in(file);
        std::vector<std::string> out;
        for (std::string line; std::getline(in, line);)
            out.push_back(line);
        return out;
    }

    // one record through the CSV sink: the features file's lines
    std::vector<std::string> features_lines_of(const WindowTermRecord &record)
    {
        const fs::path folder = fs::temp_directory_path() / "lidar_odometry_tests_window_features";
        fs::remove_all(folder);
        {
            FolderWindowTermSink files(folder);
            files.write(record);
            CHECK(files.ok());
        }
        std::vector<std::string> lines = file_lines(folder / "window_features.csv");
        fs::remove_all(folder);
        return lines;
    }

    std::vector<std::string> csv_fields(const std::string &line)
    {
        std::vector<std::string> out(1);
        for (const char ch : line)
            if (ch == ',')
                out.emplace_back();
            else
                out.back() += ch;
        return out;
    }

    // the columns of consistency-harness D9: prepared 30, handoff 1, handed on 36, features 4
    constexpr size_t kNewColumns = 71;

    Eigen::Affine3d pose_at(const Eigen::Vector3d &position, double heading_deg)
    {
        Eigen::Affine3d p = Eigen::Affine3d::Identity();
        p.linear() = Eigen::AngleAxisd(heading_deg * DEG_TO_RAD, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        p.translation() = position;
        return p;
    }

    // made_up_record() with every value of consistency-harness D9 present
    WindowTermRecord fully_handed_on(WindowTermRecord r)
    {
        const auto diag = [](double a, double b, double c) { return Eigen::Matrix3d(Eigen::Vector3d(a, b, c).asDiagonal()); };
        const double d = DEG_TO_RAD;
        WindowPreparedState &p = r.prepared;
        p.first_pose = pose_at(Eigen::Vector3d(1.0, 2.0, 3.0), 30.0);
        p.velocity_prior_mean = Eigen::Vector3d(0.7, -1.0, 0.02);
        p.velocity_prior_cov = diag(0.02 * 0.02, 0.03 * 0.03, 0.04 * 0.04);
        p.offset_prior_mean = Eigen::Vector3d::Zero();
        p.offset_prior_cov = diag(0.04 * 0.04, 0.05 * 0.05, 0.06 * 0.06);
        p.velocity_offset_prior_cov = diag(-1e-4, -2e-4, -3e-4);
        p.gravity = Eigen::Vector3d(0.01, -0.02, 1.01);
        p.bias_prior_mean_rad_s = Eigen::Vector3d(0.0, 0.0, 0.1 * d);
        p.bias_prior_cov = diag(0.2 * d * 0.2 * d, 0.3 * d * 0.3 * d, 0.4 * d * 0.4 * d);
        r.handoff = WindowHandOff::done;
        WindowHandedOnState &h = r.handed_on;
        h.velocity_end = Eigen::Vector3d(0.8, -1.2, 0.03);
        h.velocity_end_cov = diag(0.005 * 0.005, 0.006 * 0.006, 0.007 * 0.007);
        h.gravity = Eigen::Vector3d(0.01, -0.02, 1.0105);
        h.to_mps2_before_fold = 9.80665 / 1.01;
        h.offset_cov = diag(0.03 * 0.03, 0.035 * 0.035, 0.04 * 0.04);
        h.velocity_offset_cov = diag(-4e-5, -5e-5, -6e-5);
        h.bias_mean_rad_s = Eigen::Vector3d(0.01, -0.02, 0.15) * d;
        h.bias_cov = diag(0.03 * d * 0.03 * d, 0.04 * d * 0.04 * d, 0.05 * d * 0.05 * d);
        h.last_node_solved = WindowNode{pose_at(Eigen::Vector3d(1.3, 2.1, 3.0), 31.0), 12.875};
        h.last_node_handed_on = WindowNode{pose_at(Eigen::Vector3d(1.31, 2.1, 3.0), 31.5), 12.875};
        r.rotation_rate_rad_s = 4.0 * d;
        r.start_acceleration_mps2 = Eigen::Vector3d(0.1, -0.25, 0.05);
        return r;
    }
}

TEST_CASE("window term csv: the features file's old columns keep their names, order and values; the new ones follow")
{
    // the header and row written for made_up_record() by the writer before consistency-harness 3.1 (commit 5d45957)
    const std::string old_header = "window,time_s,duration_s,iterations,delta,lm_factor,at_rest,turn_window,acc_term,along_x,along_y,gue"
                                   "ss_move_x,guess_move_y,guess_move_z,result_move_x,result_move_y,result_move_z,velocity_start_x,veloc"
                                   "ity_start_y,velocity_start_z,velocity_end_x,velocity_end_y,velocity_end_z,gyro_bias_x_deg_s,gyro_bia"
                                   "s_y_deg_s,gyro_bias_z_deg_s,acc_offset_x_mps2,acc_offset_y_mps2,acc_offset_z_mps2,information_ratio,"
                                   "weak_axis_x,weak_axis_y,weak_axis_z,ndt_matched,ndt_unmatched,turn_defined,table_failed,change_shift"
                                   "_along,change_shift_sideways,change_shift_vertical,change_stretch_along,change_stretch_sideways,chan"
                                   "ge_stretch_vertical,change_curve_along,change_curve_sideways,change_curve_vertical,change_turn_along"
                                   ",change_turn_sideways,change_turn_vertical,verdict_bent,verdict_bent_severely,verdict_fast,verdict_b"
                                   "ias_high,heading_bend_deg,roll_bend_deg,pitch_bend_deg,speed_mps,max_bias_deg_s,velocity_chi2,stop,r"
                                   "ejected_steps,steps_not_applied,big_jump_nodes,remaining_step_m,lidar_share_along,noise_translation_"
                                   "x_m,noise_translation_y_m,noise_translation_z_m,noise_rotation_min_om_deg,noise_rotation_min_fi_deg,"
                                   "noise_rotation_min_ka_deg,noise_rotation_median_om_deg,noise_rotation_median_fi_deg,noise_rotation_m"
                                   "edian_ka_deg,noise_rotation_max_om_deg,noise_rotation_max_fi_deg,noise_rotation_max_ka_deg,noise_vel"
                                   "ocity_continuity_min_mps,noise_velocity_continuity_max_mps,noise_accelerometer_min_mps,noise_acceler"
                                   "ometer_max_mps,noise_velocity_prior_1_mps,noise_velocity_prior_2_mps,noise_velocity_prior_3_mps,nois"
                                   "e_bias_prior_1_deg_s,noise_bias_prior_2_deg_s,noise_bias_prior_3_deg_s,noise_offset_prior_1_mps2,noi"
                                   "se_offset_prior_2_mps2,noise_offset_prior_3_mps2,noise_zero_velocity_mps,noise_ndt_information_scale"
                                   ",noise_ndt_huber_cutoff";
    const std::string old_row = "7,12.5,0.375,11,2.5e-07,0.125,0,1,1,0.6,0.8,0.25,-0.5,0.0625,0.3,-0.45,0.07,0.7,-1.1,0.02,0.8,-1.2,0"
                                ".03,0.01,-0.02,0.15,0.004,0.005,-0.006,0.25,0,0,1,1234,56,0,1,0.5,1.5,2.5,3.5,4.5,5.5,6.5,7.5,8.5,9."
                                "5,10.5,,CLEAR,NOT MEASURED,TRIGGER,NOT MEASURED,0.75,-0.125,,1.5,,3.25,converged,2,1,3,,0.875,0.0001"
                                ",0.0001,0.0001,0.01,0.02,0.03,0.011,0.021,0.031,0.012,0.022,0.032,,,0.004,0.006,0.0046,0.005,0.0052,"
                                "0.3,0.31,0.32,0.07,0.071,0.072,,0.1,3";
    const std::vector<std::string> old_names = csv_fields(old_header), old_values = csv_fields(old_row);
    REQUIRE(old_names.size() == 93);

    for (const bool filled : {false, true})
    {
        CAPTURE(filled);
        const WindowTermRecord record = filled ? fully_handed_on(made_up_record()) : made_up_record();
        const std::vector<std::string> lines = features_lines_of(record);
        REQUIRE(lines.size() == 2);
        const std::vector<std::string> names = csv_fields(lines[0]), values = csv_fields(lines[1]);
        REQUIRE(names.size() == old_names.size() + kNewColumns);
        REQUIRE(values.size() == names.size());
        for (size_t k = 0; k < old_names.size(); k++)
        {
            CAPTURE(k);
            CHECK(names[k] == old_names[k]);
            CHECK(values[k] == old_values[k]);
        }
        CHECK(names[old_names.size()] == "prepared_first_x_m");
        CHECK(names.back() == "start_acceleration_z_mps2");
        // absent: every new field empty
        if (!filled)
            for (size_t k = old_names.size(); k < names.size(); k++)
            {
                CAPTURE(names[k]);
                CHECK(values[k].empty());
            }
    }
}

TEST_CASE("window term csv: the prepared and handed-on state, every field written, absent ones empty")
{
    const WindowTermRecord record = fully_handed_on(made_up_record());
    const std::vector<std::string> lines = features_lines_of(record);
    REQUIRE(lines.size() == 2);
    const std::vector<std::string> names = csv_fields(lines[0]), values = csv_fields(lines[1]);
    REQUIRE(values.size() == names.size());
    const auto at = [&](const std::string &name)
    {
        const auto it = std::find(names.begin(), names.end(), name);
        REQUIRE_MESSAGE(it != names.end(), name);
        return values[it - names.begin()];
    };
    const auto near = [&](const std::string &name, double expected)
    {
        CAPTURE(name);
        const std::string v = at(name);
        REQUIRE_FALSE(v.empty());
        CHECK(std::stod(v) == doctest::Approx(expected).epsilon(1e-8));
    };
    // every new column of the filled record is written
    for (size_t k = 93; k < names.size(); k++)
    {
        CAPTURE(names[k]);
        CHECK_FALSE(values[k].empty());
    }
    near("prepared_first_x_m", 1.0);
    near("prepared_first_y_m", 2.0);
    near("prepared_first_z_m", 3.0);
    near("prepared_first_ka_deg", 30.0);
    near("prepared_first_om_deg", 0.0);
    near("prepared_velocity_prior_y_mps", -1.0);
    near("prepared_velocity_prior_sigma_x_mps", 0.02);
    near("prepared_velocity_prior_sigma_z_mps", 0.04);
    near("prepared_offset_prior_z_mps2", 0.0);
    near("prepared_offset_prior_sigma_y_mps2", 0.05);
    near("prepared_velocity_offset_cov_xx", -1e-4);
    near("prepared_velocity_offset_cov_zz", -3e-4);
    near("prepared_gravity_z_g", 1.01);
    near("prepared_bias_prior_z_deg_s", 0.1);
    near("prepared_bias_prior_sigma_x_deg_s", 0.2);
    CHECK(at("handoff") == "done");
    near("handed_velocity_end_x_mps", 0.8);
    near("handed_velocity_end_sigma_y_mps", 0.006);
    near("handed_gravity_z_g", 1.0105);
    near("handed_to_mps2_before_fold", 9.80665 / 1.01);
    near("handed_offset_sigma_x_mps2", 0.03);
    near("handed_velocity_offset_cov_yy", -5e-5);
    near("handed_bias_z_deg_s", 0.15);
    near("handed_bias_sigma_z_deg_s", 0.05);
    near("handed_last_solved_time_s", 12.875);
    near("handed_last_solved_x_m", 1.3);
    near("handed_last_solved_ka_deg", 31.0);
    near("handed_last_time_s", 12.875);
    near("handed_last_x_m", 1.31);
    near("handed_last_ka_deg", 31.5);
    near("rotation_rate_mean_deg_s", 4.0);
    near("start_acceleration_y_mps2", -0.25);

    // a cancelled hand-off: the handed-on columns are empty, the rest stays
    WindowTermRecord cancelled = record;
    cancelled.handoff = WindowHandOff::cancelled;
    const std::vector<std::string> c = csv_fields(features_lines_of(cancelled)[1]);
    REQUIRE(c.size() == names.size());
    for (size_t k = 93; k < names.size(); k++)
    {
        CAPTURE(names[k]);
        if (names[k] == "handoff")
            CHECK(c[k] == "cancelled");
        else if (names[k].rfind("handed_", 0) == 0)
            CHECK(c[k].empty());
        else
            CHECK(c[k] == values[k]);
    }

    // single values absent: their columns empty, their neighbours written
    WindowTermRecord partial = record;
    partial.prepared.velocity_offset_prior_cov.reset();
    partial.handed_on.to_mps2_before_fold.reset();
    partial.handed_on.last_node_handed_on.reset();
    partial.rotation_rate_rad_s.reset();
    const std::vector<std::string> p = csv_fields(features_lines_of(partial)[1]);
    REQUIRE(p.size() == names.size());
    for (size_t k = 93; k < names.size(); k++)
    {
        CAPTURE(names[k]);
        const bool gone = names[k].rfind("prepared_velocity_offset_cov_", 0) == 0 || names[k] == "handed_to_mps2_before_fold" ||
                          names[k] == "handed_last_time_s" || names[k].rfind("handed_last_", 0) == 0 && names[k].rfind("handed_last_solved", 0) != 0 ||
                          names[k] == "rotation_rate_mean_deg_s";
        CHECK(p[k].empty() == gone);
    }
}
