#include <doctest/doctest.h>

#include "synthetic_loop.h"
#include "truth_statistics.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

// The loop's class tables (openspec change consistency-harness, task 6.2; design D8 "Pairing" and "Reported", D5's
// normalised errors, D10's readings). On demand only: nothing here runs in ctest except the small check of the table
// statistics below (pure arithmetic, milliseconds).
//
// R1, R2 and R3 (synthetic_loop.h: loop_sensor, loop_route) are run through the real front end and optimize_windows in
// the MATCHED world (LoopWorld::matched: imu_noise_from_config of the run's own settings) over independent seeds
// (SensorProfile::seed draws the points and every IMU error), under three settings:
//   off      loop_candidate_config(sensor, false): the stage 8 candidate set, both layer switches off
//   on       loop_candidate_config(sensor, true):  the same, imu_edges_follow_state on (until 2026-11-25 also
//            imu_carry_joint_covariance, layer 2, since removed; the joint move is always NOT MEASURED now)
//   shipped  loop_shipped_config(sensor): settings/livox-vertical.toml as loaded, only real_time_threshold_seconds 1000 and
//            log_window_terms true set, and the front end fields of front_end_config copied in (the values the copy
//            changes are written to the output folder's settings.txt; for the loop's sensors none of them differ)
// off and on share one recording per route and seed (their worlds are the same: the switches change no claimed noise), so
// they are paired. The shipped file claims other noise (imu_acceleration_offset_rw_mps2_sqrt_s 0.0001 against the
// candidate's 0.01), so its world, matched to its own claims, is another recording with the same seed: a second,
// separately reported measurement (D8), never paired with off or on.
//
// The one command (Release, from the worktree's root, Git Bash; the output folder is created; existing files in it are
// overwritten; with no environment set it runs 20 seeds of R1, R2, R3 under off and on and of R1, R3 under shipped into
// run-<today>; R2 is never run under shipped, see kShippedR2Why):
//   LOOP_TABLES_SEEDS=20 out/build/x64-Release/bin/lidar_odometry_tests.exe -tc="synthetic loop: class tables" --no-skip -s
//       > /c/Users/User/Downloads/dLidar/experiments/consistency/loop/loop_tables.log 2>&1
// (cmd: set LOOP_TABLES_SEEDS=20 then the same exe line.) One line per run goes to the log ("seed <s> R<r> <setting>: ...").
// Environment (all optional):
//   LOOP_TABLES_SEEDS       number of seeds (default 20)
//   LOOP_TABLES_FIRST_SEED  first seed (default 1); the seeds are first .. first + count - 1
//   LOOP_TABLES_OUT         output folder (default C:\Users\User\Downloads\dLidar\experiments\consistency\loop\run-<YYYY-MM-DD>)
//   LOOP_TABLES_ROUTES      routes, e.g. "3" or "1,3" (default "1,2,3")
//   LOOP_TABLES_SETTINGS    settings, e.g. "off,on" (default "off,on,shipped")
// Outputs (rewritten after every seed, so an interrupted run leaves what it measured):
//   windows.csv   one row per window of every run (empty cell = NOT MEASURED; the *_why columns say why); for R2's blind
//                 windows the blind preconditions' truth-placed values (blind_*_truth_precondition) beside the run's own
//                 readings of the same quantities (blind_*_run_RESULT, results)
//   runs.csv      one row per run: validity, failed preconditions, errors, windows, time; for R2 the exit's along-axis
//                 share, landmark cells mapped (share) and accepted landmark matches, each truth-placed (the precondition's
//                 value, *_truth_precondition) and the run's own reading (*_run_RESULT, a result)
// No precondition depends on the run's own outcome (user's decision of 2026-11-21; synthetic_loop.h).
//   tables.md     the class tables, the exclusions per setting and the paired comparison off vs on
//   settings.txt  the seeds, routes, settings and the shipped set's overrides
// Time measured 2026-11-21 (Release): see the task's hand-back; about 0.55 s per window (R1 79, R2 124, R3 35 windows).

using namespace synthetic;

namespace
{
#ifdef NDEBUG
    constexpr bool loop_cases_skipped = false;
#else
    constexpr bool loop_cases_skipped = true;
#endif

    // ---- statistics of a column (descriptive) ----

    // numpy's default percentile (linear interpolation between order statistics); p in [0, 1]
    double percentile(std::vector<double> v, double p)
    {
        if (v.empty())
            return std::nan("");
        std::sort(v.begin(), v.end());
        const double h = (v.size() - 1) * p;
        const std::size_t lo = static_cast<std::size_t>(std::floor(h));
        const std::size_t hi = std::min(lo + 1, v.size() - 1);
        return v[lo] + (h - lo) * (v[hi] - v[lo]);
    }

    double mean(const std::vector<double> &v)
    {
        if (v.empty())
            return std::nan("");
        double s = 0.0;
        for (double x : v)
            s += x;
        return s / v.size();
    }

    double rms(const std::vector<double> &v)
    {
        if (v.empty())
            return std::nan("");
        double s = 0.0;
        for (double x : v)
            s += x * x;
        return std::sqrt(s / v.size());
    }

    // The 1 % two-sided interval of the mean of n normalised errors per degree of freedom, if they were n independent
    // chi-square draws with `dof` degrees of freedom each: chi2_interval(n dof, 0.01) / (n dof). Windows of one walk are not
    // independent, so this is a reference, not a test.
    std::pair<double, double> mean_per_dof_interval(int n, int dof)
    {
        if (n <= 0)
            return {std::nan(""), std::nan("")};
        const double k = double(n) * dof;
        const auto [lo, hi] = truth_stats::chi2_interval(k, 0.01);
        return {lo / k, hi / k};
    }

    std::string num(double v, int digits)
    {
        if (!std::isfinite(v))
            return "-";
        std::ostringstream s;
        s << std::fixed << std::setprecision(digits) << v;
        return s.str();
    }

    std::string cell(const std::optional<double> &v)
    {
        if (!v)
            return "";
        std::ostringstream s;
        s << std::setprecision(9) << *v;
        return s.str();
    }

    std::string cell(double v)
    {
        std::ostringstream s;
        s << std::setprecision(9) << v;
        return s.str();
    }

    // ---- the settings ----

    enum class Setting
    {
        off,
        on,
        shipped
    };
    const char *setting_name(Setting s) { return s == Setting::off ? "off" : s == Setting::on ? "on" : "shipped"; }

    // R2 is not run under `shipped` (the user's decision of 2026-11-21): its blind-corridor preconditions fail by design
    // there, because the shipped file uses the previous NDT cell statistics (use_exact_ndt_bucket_statistics false), so
    // every shipped R2 run would be excluded.
    const char *kShippedR2Why = "R2 is not run under `shipped` (user's decision of 2026-11-21): its blind-corridor preconditions fail by design there, "
                                "because the shipped file uses the previous NDT cell statistics (use_exact_ndt_bucket_statistics false), so every "
                                "shipped R2 run would be excluded.";
    bool route_runs_under(int route, Setting s) { return !(route == 2 && s == Setting::shipped); }

    OdometryConfig setting_config(Setting s, const SensorProfile &sensor, std::vector<std::string> *changed = nullptr)
    {
        if (s == Setting::shipped)
            return loop_shipped_config(sensor, changed);
        return loop_candidate_config(sensor, s == Setting::on);
    }

    // ---- what the tables need of one window and one run ----

    struct WindowRow
    {
        int route = 0;
        Setting setting = Setting::off;
        std::uint64_t seed = 0;
        int window = 0;
        WindowClass cls = WindowClass::seen;
        ObservabilityClass obs = ObservabilityClass::not_moving;
        bool valid_run = false;
        double position_error_m = 0.0;
        double attitude_error_deg = 0.0;
        std::optional<double> velocity_error_mps;
        std::optional<double> velocity_nees; // 3 dof
        std::optional<double> move_velocity, move_joint, move_bias;
        std::optional<double> velocity_chi2, heading_bend_deg;
        Check bent = Check::not_measured;
    };

    struct RunRow
    {
        int route = 0;
        Setting setting = Setting::off;
        std::uint64_t seed = 0;
        bool valid = false;
        bool completed = false;
        std::vector<std::string> failed;                 // text, as LoopRun::failed
        std::map<std::string, int> failed_names;         // precondition name -> windows where it fails
        int windows_cut = 0, windows_run = 0;
        double end_error_m = 0.0, largest_error_m = 0.0;
        std::optional<double> largest_velocity_nees;
        double loop_seconds = 0.0;  // optimize_windows
        double run_seconds = 0.0;   // run_loop (cut, truth, observability, loop, preconditions)
        double generate_seconds = 0.0;
        int time_limit_windows = 0, map_rebuilds = -1, moving_judged_at_rest = 0, unavailable_total = 0;
        // R2 only: the exit's along-axis share, truth-placed (the precondition's value) and the run's own reading (a RESULT,
        // never a precondition; user's decision of 2026-11-21)
        std::optional<double> exit_share_truth, exit_share_run;
        // R2 only: the exit's landmark readings, truth-placed (the preconditions' values) and the run's own (RESULTS)
        std::optional<double> landmark_mapped_share_truth, landmark_mapped_share_run, landmark_matches_truth, landmark_matches_run;
    };

    std::string finite_cell(double v) { return std::isfinite(v) ? cell(v) : std::string(); }

    std::string velocity_nees_why(const LoopWindow &w)
    {
        if (w.errors.velocity_end_normalised)
            return "measured";
        if (!w.record)
            return "no record";
        if (w.record->handoff != WindowHandOff::done)
            return "hand-off not done";
        if (!w.record->handed_on.velocity_end)
            return "no end velocity";
        if (!w.record->handed_on.velocity_end_cov)
            return "no end-velocity covariance";
        return "factorisation failed";
    }

    const char *optional_check(Check c) { return check_name(c); }

    void write_window_header(std::ostream &o)
    {
        o << "route,setting,seed,window,time_s,truth_class,observability_class,along_share,valid_run,at_rest,"
             "err_pos_x_m,err_pos_y_m,err_pos_z_m,err_pos_m,err_att_deg,"
             "err_vel_x_mps,err_vel_y_mps,err_vel_z_mps,vel_nees_3dof,vel_nees_why,"
             "heading_bend_deg,roll_bend_deg,pitch_bend_deg,speed_mps,max_bias_deg_s,velocity_chi2,information_ratio,"
             "stop,iterations,remaining_step_m,lidar_share_along,"
             "move_velocity,move_velocity_why,move_joint,move_joint_why,move_bias,move_bias_why,"
             "rotation_rate_rad_s,start_acceleration_mps2,bent,fast,bias_high";
        // R2's blind windows: the preconditions' truth-placed values and the run's own readings (RESULTS); empty elsewhere
        for (const char *q : {"along_share", "projected_along_across", "side_share", "up_share", "side_normals", "floor_normals", "travel_angle_deg"})
            o << ",blind_" << q << "_truth_precondition,blind_" << q << "_run_RESULT";
        o << "\n";
    }

    void write_blind_cells(std::ostream &csv, const LoopWindow &w)
    {
        const bool blind = w.cls == WindowClass::blind;
        const BlindReadings &t = w.blind_truth, &r = w.blind_run;
        const double BlindReadings::*q[] = {&BlindReadings::along_share, &BlindReadings::projected_ratio, &BlindReadings::side_share, &BlindReadings::up_share,
                                            &BlindReadings::side_normals, &BlindReadings::floor_normals, &BlindReadings::travel_angle_deg};
        for (const auto m : q)
            csv << "," << (blind && t.measured ? finite_cell(t.*m) : std::string()) << "," << (blind && r.measured ? finite_cell(r.*m) : std::string());
    }

    // one run's windows: the CSV rows and the table rows
    void collect_windows(const LoopRun &run, int route, Setting setting, std::uint64_t seed, std::ostream &csv, std::vector<WindowRow> &rows)
    {
        for (int i = 0; i < run.windows_run && i < static_cast<int>(run.windows.size()); i++)
        {
            const LoopWindow &w = run.windows[i];
            WindowRow r;
            r.route = route;
            r.setting = setting;
            r.seed = seed;
            r.window = w.window;
            r.cls = w.cls;
            r.obs = w.observability.cls;
            r.valid_run = run.valid();
            r.position_error_m = w.errors.last_position.norm();
            r.attitude_error_deg = w.errors.last_rotation_rad * 180.0 / 3.14159265358979323846;
            if (w.errors.velocity_end)
                r.velocity_error_mps = w.errors.velocity_end->norm();
            r.velocity_nees = w.errors.velocity_end_normalised;
            const WindowEvidence *ev = w.record ? &w.record->verdict.evidence : nullptr;
            if (ev)
            {
                r.move_velocity = ev->moves.start_velocity.value;
                r.move_joint = ev->moves.joint.value;
                r.move_bias = ev->moves.gyro_bias.value;
                r.velocity_chi2 = ev->velocity_chi2;
                if (ev->heading_bend_deg)
                    r.heading_bend_deg = std::abs(*ev->heading_bend_deg);
                r.bent = w.record->verdict.bent;
            }
            rows.push_back(r);

            const Eigen::Vector3d &p = w.errors.last_position;
            csv << "R" << route << "," << setting_name(setting) << "," << seed << "," << w.window << "," << (w.record ? cell(w.record->time_s) : "") << ","
                << window_class_name(w.cls) << "," << observability_class_name(w.observability.cls) << "," << cell(w.observability.along_share) << ","
                << int(r.valid_run) << "," << (w.record ? std::to_string(int(w.record->at_rest)) : "") << "," << cell(p.x()) << "," << cell(p.y()) << ","
                << cell(p.z()) << "," << cell(p.norm()) << "," << cell(r.attitude_error_deg) << ",";
            if (w.errors.velocity_end)
                csv << cell(w.errors.velocity_end->x()) << "," << cell(w.errors.velocity_end->y()) << "," << cell(w.errors.velocity_end->z()) << ",";
            else
                csv << ",,,";
            csv << cell(r.velocity_nees) << "," << velocity_nees_why(w) << ",";
            if (ev)
            {
                csv << cell(ev->heading_bend_deg) << "," << cell(ev->roll_bend_deg) << "," << cell(ev->pitch_bend_deg) << "," << cell(ev->speed_mps) << ","
                    << cell(ev->max_bias_deg_s) << "," << cell(ev->velocity_chi2) << "," << cell(ev->information_ratio) << "," << lm_stop_name(ev->stop) << ","
                    << ev->iterations << "," << cell(ev->remaining_step_m) << "," << cell(ev->lidar_share_along) << ",";
                for (const NormalisedMove *m : {&ev->moves.start_velocity, &ev->moves.joint, &ev->moves.gyro_bias})
                    csv << cell(m->value) << "," << move_not_measured_name(m->reason) << ",";
                csv << cell(w.record->rotation_rate_rad_s) << ","
                    << (w.record->start_acceleration_mps2 ? cell(w.record->start_acceleration_mps2->norm()) : std::string()) << ","
                    << optional_check(w.record->verdict.bent) << "," << optional_check(w.record->verdict.fast) << ","
                    << optional_check(w.record->verdict.bias_high);
            }
            else
                csv << std::string(11, ',') << ",no record,,no record,,no record,,," << check_name(Check::not_measured) << ","
                    << check_name(Check::not_measured) << "," << check_name(Check::not_measured);
            write_blind_cells(csv, w);
            csv << "\n";
        }
    }

    RunRow run_row(const LoopRun &run, int route, Setting setting, std::uint64_t seed)
    {
        RunRow r;
        r.route = route;
        r.setting = setting;
        r.seed = seed;
        r.valid = run.valid();
        r.completed = run.completed;
        r.failed = run.failed();
        for (const Precondition &p : run.preconditions)
            if (!p.holds)
                r.failed_names[p.name]++;
        if (!run.completed)
            r.failed_names["the loop completed"]++;
        r.windows_cut = run.windows_cut;
        r.windows_run = run.windows_run;
        r.end_error_m = run.end_error_m();
        r.largest_error_m = run.largest_error_m();
        r.largest_velocity_nees = run.largest_velocity_end_normalised();
        r.loop_seconds = run.seconds;
        r.time_limit_windows = run.time_limit_windows;
        r.map_rebuilds = run.map_rebuilds;
        r.moving_judged_at_rest = run.moving_judged_at_rest;
        r.unavailable_total = run.unavailable.total();
        r.exit_share_truth = run.r2.exit_share_truth;
        r.exit_share_run = run.r2.exit_share_run;
        const R2Design &d = run.r2;
        if (d.present && d.exit_in_run && !d.landmark_cells.empty())
        {
            const double cells = double(d.landmark_cells.size());
            if (d.exit_truth_counted)
            {
                r.landmark_mapped_share_truth = d.landmark_cells_mapped / cells;
                r.landmark_matches_truth = double(d.exit_landmark_matches);
            }
            if (d.landmark_cells_mapped_run)
                r.landmark_mapped_share_run = *d.landmark_cells_mapped_run / cells;
            if (d.exit_landmark_matches_run)
                r.landmark_matches_run = double(*d.exit_landmark_matches_run);
        }
        return r;
    }

    void write_runs_csv(const std::filesystem::path &file, const std::vector<RunRow> &runs)
    {
        std::ofstream o(file);
        o << "route,setting,seed,valid,completed,failed_count,failed_preconditions,windows_cut,windows_run,end_error_m,largest_error_m,"
             "largest_vel_nees_3dof,loop_seconds,run_seconds,generate_seconds,time_limit_windows,map_rebuilds,moving_judged_at_rest,unavailable_total,"
             "exit_share_truth_precondition,exit_share_run_RESULT,landmark_mapped_share_truth_precondition,landmark_mapped_share_run_RESULT,"
             "landmark_matches_truth_precondition,landmark_matches_run_RESULT\n";
        for (const RunRow &r : runs)
        {
            std::string failed;
            for (const std::string &f : r.failed)
                failed += (failed.empty() ? "" : "; ") + f;
            std::replace(failed.begin(), failed.end(), '"', '\'');
            o << "R" << r.route << "," << setting_name(r.setting) << "," << r.seed << "," << int(r.valid) << "," << int(r.completed) << "," << r.failed.size()
              << ",\"" << failed << "\"," << r.windows_cut << "," << r.windows_run << "," << cell(r.end_error_m) << "," << cell(r.largest_error_m) << ","
              << cell(r.largest_velocity_nees) << "," << cell(r.loop_seconds) << "," << cell(r.run_seconds) << "," << cell(r.generate_seconds) << ","
              << r.time_limit_windows << "," << r.map_rebuilds << "," << r.moving_judged_at_rest << "," << r.unavailable_total << ","
              << cell(r.exit_share_truth) << "," << cell(r.exit_share_run) << "," << cell(r.landmark_mapped_share_truth) << "," << cell(r.landmark_mapped_share_run)
              << "," << cell(r.landmark_matches_truth) << "," << cell(r.landmark_matches_run) << "\n";
        }
    }

    // ---- the tables ----

    struct Group
    {
        std::set<std::uint64_t> seeds;
        int windows = 0;
        std::vector<double> nees_per_dof; // normalised end-velocity error / 3
        int nees_unavailable = 0;
        std::vector<double> pos, att, vel;
        std::vector<double> move_v, move_j, move_b, chi2, bend;
        int move_v_na = 0, move_j_na = 0, move_b_na = 0, chi2_na = 0, bend_na = 0;
        int bent = 0, bent_measured = 0;
        void add(const WindowRow &r)
        {
            seeds.insert(r.seed);
            windows++;
            if (r.velocity_nees)
                nees_per_dof.push_back(*r.velocity_nees / 3.0);
            else
                nees_unavailable++;
            pos.push_back(r.position_error_m);
            att.push_back(r.attitude_error_deg);
            if (r.velocity_error_mps)
                vel.push_back(*r.velocity_error_mps);
            auto put = [](const std::optional<double> &v, std::vector<double> &to, int &na)
            {
                if (v)
                    to.push_back(*v);
                else
                    na++;
            };
            put(r.move_velocity, move_v, move_v_na);
            put(r.move_joint, move_j, move_j_na);
            put(r.move_bias, move_b, move_b_na);
            put(r.velocity_chi2, chi2, chi2_na);
            put(r.heading_bend_deg, bend, bend_na);
            if (r.bent != Check::not_measured)
            {
                bent_measured++;
                bent += r.bent == Check::trigger;
            }
        }
    };

    const char *kErrorHeader = "| class | seeds | windows | NEES n | NOT MEASURED | mean / dof | 1 % interval of the mean (if independent) | median / dof | p95 / dof | "
                               "RMS position [m] | RMS attitude [deg] | RMS velocity [m/s] |\n"
                               "|---|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|---:|\n";

    void error_line(std::ostream &o, const std::string &name, const Group &g)
    {
        const auto [lo, hi] = mean_per_dof_interval(static_cast<int>(g.nees_per_dof.size()), 3);
        o << "| " << name << " | " << g.seeds.size() << " | " << g.windows << " | " << g.nees_per_dof.size() << " | " << g.nees_unavailable << " | "
          << num(mean(g.nees_per_dof), 3) << " | [" << num(lo, 3) << ", " << num(hi, 3) << "] | " << num(percentile(g.nees_per_dof, 0.5), 3) << " | "
          << num(percentile(g.nees_per_dof, 0.95), 3) << " | " << num(rms(g.pos), 3) << " | " << num(rms(g.att), 3) << " | " << num(rms(g.vel), 3) << " |\n";
    }

    const char *kReadingHeader = "| class | windows | move v: n / NM / median / p95 | move joint: n / NM / median / p95 | move bias: n / NM / median / p95 | "
                                 "velocity chi2: n / NM / median | abs heading bend [deg]: n / NM / median / p95 | bent > 3 deg / measured |\n"
                                 "|---|---:|---|---|---|---|---|---|\n";

    std::string reading_cell(const std::vector<double> &v, int na, bool p95 = true)
    {
        std::string s = std::to_string(v.size()) + " / " + std::to_string(na) + " / " + num(percentile(v, 0.5), 3);
        if (p95)
            s += " / " + num(percentile(v, 0.95), 3);
        return s;
    }

    void reading_line(std::ostream &o, const std::string &name, const Group &g)
    {
        o << "| " << name << " | " << g.windows << " | " << reading_cell(g.move_v, g.move_v_na) << " | " << reading_cell(g.move_j, g.move_j_na) << " | "
          << reading_cell(g.move_b, g.move_b_na) << " | " << reading_cell(g.chi2, g.chi2_na, false) << " | " << reading_cell(g.bend, g.bend_na) << " | " << g.bent
          << " / " << g.bent_measured << " |\n";
    }

    struct Plan
    {
        std::vector<std::uint64_t> seeds;
        std::vector<int> routes;
        std::vector<Setting> settings;
        std::filesystem::path out;
    };

    bool has_setting(const Plan &p, Setting s) { return std::find(p.settings.begin(), p.settings.end(), s) != p.settings.end(); }

    void write_tables(const std::filesystem::path &file, const Plan &plan, const std::vector<RunRow> &runs, const std::vector<WindowRow> &rows,
                      const std::vector<std::string> &shipped_overrides)
    {
        std::ofstream o(file);
        o << "# The loop on synthetic recordings: class tables (consistency-harness task 6.2)\n\n"
          << "Measurements, not a verdict. Matched world (each setting's recording drawn from that setting's own noise claims). "
          << "Routes R1 (loop, seen), R2 (out and back through a blind corridor), R3 (rest, walk, rest). Settings: `off` / `on` = the stage 8 "
          << "candidate set with both layer switches off / on (one recording per route and seed, paired); `shipped` = settings/livox-vertical.toml "
          << "(its own world, another recording with the same seed; a separate measurement, never paired with off or on).\n\n"
          << kShippedR2Why << "\n\n"
          << "No precondition depends on the run's own outcome (user's decision of 2026-11-21: an outcome-based precondition biases "
          << "the paired comparison). R2's preconditions are computed from the truth: at the exit (along-axis share, landmark cells in "
          << "the map before the exit, accepted landmark matches) from the exit window's points placed with its true poses against the "
          << "map built from the truth-placed points of the windows before it; in each blind window (along-axis, sideways and vertical "
          << "shares, matched side and floor normals, projected NDT weight along / across the true axis, travel direction) from its "
          << "truth-placed points against the truth-built map. The run's own readings of the same quantities are RESULTS "
          << "(runs.csv `*_run_RESULT`, windows.csv `blind_*_run_RESULT`), never preconditions.\n\n"
          << "**Independence:** the windows of one walk are NOT independent draws (D5, D6). The number of independent draws behind a row is its "
          << "`seeds` column. The 1 % interval is the chi-square interval of the mean of `NEES n` normalised errors per degree of freedom IF they "
          << "were independent 3-dof draws (truth_statistics::chi2_interval(3n, 0.01) / 3n); it is a reference scale, not a test. "
          << "NEES = the handed-on end velocity's error against its handed-on covariance (3 dof, D5), shown per dof (expected 1 under an honest "
          << "claim). Errors are at each window's last node, against the truth aligned at window 0's first node. Only valid runs (every "
          << "precondition holds) enter the tables; the excluded runs are listed per setting.\n\n";
        o << "Seeds planned: " << plan.seeds.size() << " (" << (plan.seeds.empty() ? 0 : plan.seeds.front()) << " to "
          << (plan.seeds.empty() ? 0 : plan.seeds.back()) << "); runs written: " << runs.size() << ".\n\n";
        o << "Shipped set overrides (file -> test):\n";
        for (const std::string &s : shipped_overrides)
            o << "- " << s << "\n";
        o << "\n";

        for (int route : plan.routes)
        {
            o << "## R" << route << "\n\n### Runs and exclusions per setting\n\n";
            std::map<int, std::set<std::uint64_t>> valid_seeds;
            std::vector<Setting> route_settings;
            for (Setting s : plan.settings)
                if (route_runs_under(route, s))
                    route_settings.push_back(s);
                else
                    o << "- `" << setting_name(s) << "`: not run. " << kShippedR2Why << "\n";
            for (Setting s : route_settings)
            {
                int n = 0, valid = 0;
                std::ostringstream excl;
                for (const RunRow &r : runs)
                    if (r.route == route && r.setting == s)
                    {
                        n++;
                        if (r.valid)
                        {
                            valid++;
                            valid_seeds[int(s)].insert(r.seed);
                        }
                        else
                        {
                            excl << "  - seed " << r.seed << ": " << r.failed.size() << " failed:";
                            for (const auto &[name, count] : r.failed_names)
                                excl << " " << name << " (" << count << ")" << ";";
                            excl << "\n";
                        }
                    }
                o << "- `" << setting_name(s) << "`: " << n << " runs, " << valid << " valid, " << n - valid << " excluded\n" << excl.str();
            }
            o << "\n";
            for (Setting s : route_settings)
            {
                std::vector<Group> by_class(kWindowClasses), by_obs(kObservabilityClasses);
                Group all;
                for (const WindowRow &r : rows)
                    if (r.route == route && r.setting == s && r.valid_run)
                    {
                        by_class[int(r.cls)].add(r);
                        by_obs[int(r.obs)].add(r);
                        all.add(r);
                    }
                o << "### R" << route << " `" << setting_name(s) << "`: " << valid_seeds[int(s)].size() << " valid seeds (independent draws)\n\n";
                o << "Errors by truth class:\n\n" << kErrorHeader;
                for (int c = 0; c < kWindowClasses; c++)
                    if (by_class[c].windows > 0)
                        error_line(o, window_class_name(WindowClass(c)), by_class[c]);
                error_line(o, "all", all);
                o << "\nErrors by observability class (along travel, by the truth):\n\n" << kErrorHeader;
                for (int c = 0; c < kObservabilityClasses; c++)
                    if (by_obs[c].windows > 0)
                        error_line(o, observability_class_name(ObservabilityClass(c)), by_obs[c]);
                o << "\nReadings (D10) by truth class (NM = NOT MEASURED; the reasons per window are in windows.csv):\n\n" << kReadingHeader;
                for (int c = 0; c < kWindowClasses; c++)
                    if (by_class[c].windows > 0)
                        reading_line(o, window_class_name(WindowClass(c)), by_class[c]);
                reading_line(o, "all", all);
                o << "\n";
            }

            if (has_setting(plan, Setting::off) && has_setting(plan, Setting::on))
            {
                std::vector<std::uint64_t> paired;
                for (std::uint64_t s : valid_seeds[int(Setting::off)])
                    if (valid_seeds[int(Setting::on)].count(s))
                        paired.push_back(s);
                o << "### R" << route << " paired comparison `off` vs `on`\n\n"
                  << "Seeds valid under both: " << paired.size() << " (excluded: " << valid_seeds[int(Setting::off)].size() - paired.size()
                  << " valid only under off, " << valid_seeds[int(Setting::on)].size() - paired.size() << " valid only under on; see above). "
                  << "Per seed, the mean NEES / dof over that seed's windows of the class; the difference on - off per seed, its mean and range. "
                  << "A measurement; no ranking.\n\n"
                  << "| class | paired seeds with the class | NEES n off / on | mean / dof off | mean / dof on | on - off per seed: mean [min, max] | "
                     "seeds with on lower | RMS position off / on [m] |\n|---|---:|---|---:|---:|---|---:|---|\n";
                for (int c = 0; c <= kWindowClasses; c++) // kWindowClasses: all
                {
                    Group goff, gon;
                    std::vector<double> diffs;
                    int on_lower = 0;
                    for (std::uint64_t seed : paired)
                    {
                        Group soff, son;
                        for (const WindowRow &r : rows)
                            if (r.route == route && r.seed == seed && (c == kWindowClasses || int(r.cls) == c))
                            {
                                if (r.setting == Setting::off)
                                    soff.add(r), goff.add(r);
                                else if (r.setting == Setting::on)
                                    son.add(r), gon.add(r);
                            }
                        if (!soff.nees_per_dof.empty() && !son.nees_per_dof.empty())
                        {
                            const double d = mean(son.nees_per_dof) - mean(soff.nees_per_dof);
                            diffs.push_back(d);
                            on_lower += d < 0.0;
                        }
                    }
                    if (goff.windows == 0 && gon.windows == 0)
                        continue;
                    const double dmin = diffs.empty() ? std::nan("") : *std::min_element(diffs.begin(), diffs.end());
                    const double dmax = diffs.empty() ? std::nan("") : *std::max_element(diffs.begin(), diffs.end());
                    o << "| " << (c == kWindowClasses ? "all" : window_class_name(WindowClass(c))) << " | " << diffs.size() << " | "
                      << goff.nees_per_dof.size() << " / " << gon.nees_per_dof.size() << " | " << num(mean(goff.nees_per_dof), 3) << " | "
                      << num(mean(gon.nees_per_dof), 3) << " | " << num(mean(diffs), 3) << " [" << num(dmin, 3) << ", " << num(dmax, 3) << "] | " << on_lower
                      << " of " << diffs.size() << " | " << num(rms(goff.pos), 3) << " / " << num(rms(gon.pos), 3) << " |\n";
                }
                o << "\n";
            }
        }
    }

    // ---- the environment ----

    std::string env(const char *name, const std::string &fallback)
    {
        const char *v = std::getenv(name);
        return v && *v ? std::string(v) : fallback;
    }

    std::vector<std::string> split(const std::string &s)
    {
        std::vector<std::string> out;
        std::string cur;
        for (char ch : s + ",")
            if (ch == ',' || ch == ' ' || ch == ';')
            {
                if (!cur.empty())
                    out.push_back(cur);
                cur.clear();
            }
            else
                cur += ch;
        return out;
    }

    std::string today()
    {
        const std::time_t t = std::time(nullptr);
        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        char buf[16];
        std::strftime(buf, sizeof buf, "%Y-%m-%d", &tm);
        return buf;
    }

    Plan plan_from_environment()
    {
        Plan p;
        const int count = std::stoi(env("LOOP_TABLES_SEEDS", "20"));
        const std::uint64_t first = std::stoull(env("LOOP_TABLES_FIRST_SEED", "1"));
        for (int i = 0; i < count; i++)
            p.seeds.push_back(first + i);
        for (const std::string &r : split(env("LOOP_TABLES_ROUTES", "1,2,3")))
        {
            const int route = std::stoi(r[0] == 'R' || r[0] == 'r' ? r.substr(1) : r);
            if (route < 1 || route > 3)
                throw std::invalid_argument("LOOP_TABLES_ROUTES: route " + r);
            p.routes.push_back(route);
        }
        for (const std::string &s : split(env("LOOP_TABLES_SETTINGS", "off,on,shipped")))
        {
            if (s == "off")
                p.settings.push_back(Setting::off);
            else if (s == "on")
                p.settings.push_back(Setting::on);
            else if (s == "shipped")
                p.settings.push_back(Setting::shipped);
            else
                throw std::invalid_argument("LOOP_TABLES_SETTINGS: " + s);
        }
        p.out = env("LOOP_TABLES_OUT", "C:\\Users\\User\\Downloads\\dLidar\\experiments\\consistency\\loop\\run-" + today());
        return p;
    }
}

// The table statistics on known numbers (no run; part of every ctest).
TEST_CASE("synthetic loop tables: percentile, RMS and the interval of a mean per degree of freedom")
{
    CHECK(percentile({1.0, 2.0, 3.0, 4.0}, 0.5) == doctest::Approx(2.5));
    CHECK(percentile({1.0, 2.0, 3.0, 4.0, 5.0}, 0.95) == doctest::Approx(4.8)); // numpy: 4.8
    CHECK(std::isnan(percentile({}, 0.5)));
    CHECK(rms({3.0, 4.0}) == doctest::Approx(std::sqrt(12.5)));
    // one 3-dof draw: the interval is chi2(3)'s 0.5 % and 99.5 % quantiles over 3
    const auto [lo1, hi1] = mean_per_dof_interval(1, 3);
    CHECK(lo1 == doctest::Approx(0.07172 / 3.0).epsilon(1e-3));
    CHECK(hi1 == doctest::Approx(12.838 / 3.0).epsilon(1e-3));
    // it narrows around 1 with the count
    const auto [lo, hi] = mean_per_dof_interval(200, 3);
    CHECK(lo > 0.8);
    CHECK(hi < 1.25);
    CHECK(lo < 1.0);
    CHECK(hi > 1.0);
}

// On demand (task 6.2): the class tables. The command and the environment are in the header of this file.
TEST_CASE("synthetic loop: class tables" * doctest::skip())
{
    REQUIRE_FALSE(loop_cases_skipped); // Release only
    const Plan plan = plan_from_environment();
    std::filesystem::create_directories(plan.out);
    MESSAGE("class tables: ", plan.seeds.size(), " seeds, ", plan.routes.size(), " routes, ", plan.settings.size(), " settings -> ", plan.out.string());

    std::vector<std::string> shipped_overrides;
    loop_shipped_config(loop_sensor(2), &shipped_overrides); // the same fields for every route's sensor (front_end_config reads no range)
    {
        std::vector<std::string> r1;
        loop_shipped_config(loop_sensor(1), &r1);
        CHECK(r1 == shipped_overrides);
    }
    {
        std::ofstream s(plan.out / "settings.txt");
        s << "consistency-harness task 6.2, class tables\nseeds:";
        for (std::uint64_t seed : plan.seeds)
            s << " " << seed;
        s << "\nroutes:";
        for (int r : plan.routes)
            s << " R" << r;
        s << "\nsettings:";
        for (Setting x : plan.settings)
            s << " " << setting_name(x);
        s << "\n" << kShippedR2Why;
        s << "\nworld: matched (imu_noise_from_config of each setting's own config)\nshipped set overrides (file -> test):\n";
        for (const std::string &o : shipped_overrides)
            s << "  " << o << "\n";
    }

    std::ofstream windows_csv(plan.out / "windows.csv");
    write_window_header(windows_csv);
    std::vector<WindowRow> rows;
    std::vector<RunRow> runs;
    const auto start = std::chrono::steady_clock::now();

    for (std::uint64_t seed : plan.seeds)
    {
        for (int route : plan.routes)
        {
            SensorProfile sensor = loop_sensor(route);
            sensor.seed = seed;
            const Route r = loop_route(route);
            // off and on share a world (the switches claim no other noise): one recording for both
            std::optional<InertialRecording> candidate_rec;
            double candidate_generate_s = 0.0;
            for (Setting setting : plan.settings)
            {
                if (!route_runs_under(route, setting))
                    continue; // kShippedR2Why
                const OdometryConfig config = setting_config(setting, sensor);
                const InertialRecording *rec = nullptr;
                std::optional<InertialRecording> own;
                double generate_s = 0.0;
                if (setting == Setting::shipped)
                {
                    const auto t0 = std::chrono::steady_clock::now();
                    own = generate_loop_recording(r, sensor, config, LoopWorld::matched);
                    generate_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                    rec = &*own;
                }
                else
                {
                    if (!candidate_rec)
                    {
                        const auto t0 = std::chrono::steady_clock::now();
                        candidate_rec = generate_loop_recording(r, sensor, loop_candidate_config(sensor, false), LoopWorld::matched);
                        candidate_generate_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
                        generate_s = candidate_generate_s;
                    }
                    rec = &*candidate_rec;
                }
                const auto t1 = std::chrono::steady_clock::now();
                const LoopRun run = run_loop(*rec, config, LoopWorld::matched);
                RunRow row = run_row(run, route, setting, seed);
                row.run_seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
                row.generate_seconds = generate_s;
                collect_windows(run, route, setting, seed, windows_csv, rows);
                windows_csv.flush();
                MESSAGE("seed ", seed, " R", route, " ", std::string(setting_name(setting)), ": ", std::string(run.valid() ? "valid" : "NOT valid"), ", ", run.windows_run,
                        " windows, loop ", num(run.seconds, 1), " s, run ", num(row.run_seconds, 1), " s, generate ", num(generate_s, 1), " s, end error ",
                        num(row.end_error_m, 3), " m, failed ", row.failed.size(), ", time limit ", run.time_limit_windows,
                        route == 2 ? "; " + r2_readings_text(run) : std::string());
                CHECK(run.completed);
                runs.push_back(std::move(row));
            }
        }
        write_runs_csv(plan.out / "runs.csv", runs);
        write_tables(plan.out / "tables.md", plan, runs, rows, shipped_overrides);
    }
    const double total_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    MESSAGE("class tables done: ", runs.size(), " runs, ", rows.size(), " windows, ", num(total_s, 1), " s -> ", plan.out.string());
    std::size_t planned = 0;
    for (int route : plan.routes)
        for (Setting s : plan.settings)
            planned += route_runs_under(route, s) ? plan.seeds.size() : 0;
    CHECK(runs.size() == planned);
}
