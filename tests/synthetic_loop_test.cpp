#include <doctest/doctest.h>

#include "synthetic_loop.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <iomanip>
#include <sstream>
#include <string>

// The loop on synthetic recordings (openspec change consistency-harness, design D8, task 6.1): the runner of
// synthetic_loop.h through the real front end and optimize_windows, with the stage 8 candidate settings.
//
// In every Release ctest: the smoke case. It completes, its preconditions hold, its declared deterministic fields repeat
// on a second run bit for bit, its unavailable readings are counted and reported; its time is reported (the budget is one
// minute in Release; a run that reached the solver's wall-clock limit fails the preconditions and its time does not count).
// The loop-level cases are skipped in Debug (unoptimised, many times slower).
//
// The whole routes are task 6.2's (synthetic_loop_tables_test.cpp: the class tables and their one command; minutes each, about
// 0.55 s per window in Release: R1 79, R2 124, R3 35 windows); the case
// below only lists what each gives, on demand:
//   lidar_odometry_tests -tc="synthetic loop: whole routes*" --no-skip -s
// Measured while building the runner (2026-11-21, switches off, 1 cm jitter), with R1 and R2 then unwalked: R2's window 12
// (constant 1.5 m/s, no gait) passed the rest test (A2 / F10) and the estimate stalled in the corridor (27 m behind at the
// exit). Measured again with all routes walked (2026-11-21, same settings; readings, not checked): no moving window is
// taken for rest in any route. R1 79 windows, 43.1 s, all its preconditions hold, last node 14.5 m off (solved 18.5 m from
// the start, truth 4.0 m). R2 124 windows, 70.1 s, 162 of 209 preconditions hold: the blind along share and projected
// along/sideways weight fail in most blind windows, the exit has 1 accepted landmark match (>= 50) and along share 0.016
// (>= 0.04); last node 16.2 m off. R3 35 windows, 19.2 s, its solved last node 3.8 m from the start against the true 9.6 m.
// In all three the solved position falls behind the truth from about window 10 on and later runs backwards.
// Those runs drew the world from ImuNoise{}'s defaults, not from the run's settings (5 to 10 times the settings' walks;
// experiments/consistency/loop/analysis-claude.md). Task 6.1a: every loop recording is drawn from a declared LoopWorld (the
// main set: matched to the run's settings, a precondition per value), R1 and R3 run in loop_scene_along_structure(), R2 with
// 40 points per node (loop_sensor); the controls (noise-free world, filter gain 0) and the labelled mismatch case are on
// demand below ("synthetic loop controls*"). Their measurements are in the task's hand-back, not asserted here.

using namespace synthetic;

namespace
{
#ifdef NDEBUG
    constexpr bool loop_cases_skipped = false;
#else
    constexpr bool loop_cases_skipped = true; // Debug: the loop-level cases are skipped
#endif

    std::string f(double v, int digits)
    {
        std::ostringstream s;
        s << std::fixed << std::setprecision(digits) << v;
        return s.str();
    }

    std::string preconditions_text(const LoopRun &run, bool failed_only)
    {
        std::ostringstream s;
        for (const Precondition &p : run.preconditions)
            if (!failed_only || !p.holds)
                s << "\n  " << (p.holds ? "holds " : "FAILS ") << p.name << (p.window ? " w" + std::to_string(p.window) : std::string()) << ": "
                  << std::setprecision(6) << p.value << (p.at_least ? " >= " : " <= ") << p.tolerance;
        for (const std::string &n : run.not_evaluated)
            s << "\n  not evaluated: " << n;
        return s.str();
    }

    // the loop's recordings (task 6.1a): the sensor and route of synthetic_loop.h (loop_sensor, loop_route); the world
    // declared with the run and drawn from its settings (LoopWorld)
    InertialRecording loop_recording(int route, LoopWorld world, std::uint64_t seed = 1, bool along_structure = true)
    {
        SensorProfile s = loop_sensor(route);
        s.seed = seed;
        return generate_loop_recording(synthetic::loop_route(route, along_structure), s, loop_candidate_config(s), world);
    }

    const InertialRecording &r1_recording()
    {
        static const InertialRecording rec = loop_recording(1, LoopWorld::matched);
        return rec;
    }

    const InertialRecording &r2_recording()
    {
        static const InertialRecording rec = loop_recording(2, LoopWorld::matched);
        return rec;
    }

    const InertialRecording &r3_recording()
    {
        static const InertialRecording rec = loop_recording(3, LoopWorld::matched);
        return rec;
    }

    // one probe line: the end and largest error, the preconditions, the classes, the largest normalised end-velocity error
    std::string probe_line(const LoopRun &run)
    {
        int holds = 0;
        for (const Precondition &p : run.preconditions)
            holds += p.holds;
        std::ostringstream s;
        s << "PROBE " << run.route << " | " << loop_world_name(run.world) << " | gain " << run.ahrs_gain << " | end " << f(run.end_error_m(), 3)
          << " m | largest " << f(run.largest_error_m(), 3) << " m | preconditions " << holds << "/" << run.preconditions.size()
          << (run.valid() ? " (valid)" : " (NOT valid)") << " | truth";
        for (int c = 0; c < kWindowClasses; c++)
            s << " " << window_class_name(WindowClass(c)) << " " << run.class_windows[c];
        s << " | observability";
        for (int o = 0; o < kObservabilityClasses; o++)
            s << " " << observability_class_name(ObservabilityClass(o)) << " " << run.observability_windows[o];
        const std::optional<double> nv = run.largest_velocity_end_normalised();
        s << " | largest normalised end-velocity error " << (nv ? f(*nv, 1) : std::string("NOT MEASURED")) << " | " << f(run.seconds, 1)
          << " s, time limit " << run.time_limit_windows;
        return s.str();
    }

    // a run's per-window observability: share along travel, the smallest eigenvalue ratio, the class, beside the truth class
    std::string observability_table(const LoopRun &run, int every)
    {
        std::ostringstream t;
        for (const LoopWindow &w : run.windows)
            if (w.window % every == 0 || w.window == run.windows_run)
                t << "\n  w" << w.window << " " << window_class_name(w.cls) << " share along " << f(w.observability.along_share, 3) << " smallest ratio "
                  << f(w.observability.smallest_ratio, 3) << " -> " << observability_class_name(w.observability.cls);
        return t.str();
    }

    // R2's blind windows: the projected NDT shift weight along / sideways by the table's travel direction (the definition
    // before 2026-11-21) and by the true corridor axis (since), the travel direction's and the removed direction's angle
    std::string blind_axis_table(const LoopRun &run)
    {
        std::ostringstream t;
        for (const LoopWindow &w : run.windows)
            if (w.cls == WindowClass::blind)
            {
                const BlindAxisWeight &b = w.blind_axis;
                t << "\n  BLIND w" << w.window << " travel " << std::setprecision(4) << b.travel_ratio << " true axis ";
                if (b.measured)
                    t << b.true_axis_ratio;
                else
                    t << "NOT MEASURED";
                t << " | travel angle " << f(b.travel_angle_deg, 3) << " deg, removed direction " << f(b.removed_angle_deg, 3) << " deg, axis z "
                  << std::setprecision(3) << b.axis_vertical << ", condition " << b.condition;
            }
        return t.str();
    }
}

// The true-axis blindness reading (ProjectedShiftWeight): from three shift weights of a projected quadratic form with one
// removed direction, the weight along any other direction, against the form itself. Pure algebra, no run.
TEST_CASE("synthetic loop: projected shift weight along any direction from the table's three")
{
    // translation information with a weak direction v (ratio 0.001 < 0.02: removed), the others kept (ratios 0.5, 1)
    const Eigen::Vector3d v = Eigen::Vector3d(1.0, 0.03, 0.002).normalized();
    const Eigen::Vector3d e1 = Eigen::Vector3d::UnitZ().cross(v).normalized();
    const Eigen::Vector3d e2 = v.cross(e1);
    WindowTermTable t;
    t.translation_information = 0.001 * v * v.transpose() + 0.5 * e1 * e1.transpose() + 1.0 * e2 * e2.transpose();
    Eigen::Matrix3d W; // any positive definite raw translation block sum
    W << 900.0, 40.0, -15.0, 40.0, 700.0, 25.0, -15.0, 25.0, 1200.0;
    const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - v * v.transpose();
    const Eigen::Matrix3d Wp = P * W * P;
    // the table's directions: travel 1.3 deg off the x axis, sideways across it, vertical
    const double a = 1.3 * 3.14159265358979323846 / 180.0;
    t.directions = {Eigen::Vector3d(std::cos(a), std::sin(a), 0.0), Eigen::Vector3d(-std::sin(a), std::cos(a), 0.0), Eigen::Vector3d::UnitZ()};
    WindowTermRow indoor, outdoor;
    indoor.name = "ndt_indoor";
    outdoor.name = "ndt_outdoor";
    for (int d = 0; d < kTermDirections; d++)
    {
        const double wd = t.directions[d].dot(Wp * t.directions[d]);
        indoor.weight[0][d] = 0.7 * wd; // the two grids' rows add up
        outdoor.weight[0][d] = 0.3 * wd;
    }
    t.rows = {indoor, outdoor};
    const ProjectedShiftWeight q = projected_shift_weight(t, 0.02, 0.04);
    REQUIRE_MESSAGE(q.measured, q.why);
    CHECK(std::abs(std::abs(q.removed.dot(v)) - 1.0) < 1e-12);
    const std::array<Eigen::Vector3d, 4> probes{Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d(0.3, -0.8, 0.5).normalized(), v};
    for (const Eigen::Vector3d &u : probes)
        CHECK(std::abs(q.along(u) - u.dot(Wp * u)) <= 1e-9 * Wp.norm());
    // nothing removed (the weak ratio above the full threshold): not measured
    t.translation_information = 0.1 * v * v.transpose() + 0.5 * e1 * e1.transpose() + 1.0 * e2 * e2.transpose();
    CHECK_FALSE(projected_shift_weight(t, 0.02, 0.04).measured);
}

// ---------------------------------------------------------------------------------------------------------------------
// The smoke case: R3 (rest, walk, rest) to its 14th window, twice, and R2 to its 10th window once (the R2 design, made from
// the whole recording, is exercised: the turn, the nominated exit and the landmark cells; the return leg and the exit lie
// beyond the run and are listed as not evaluated).
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("synthetic loop smoke: a short run completes, its preconditions hold, its deterministic fields repeat" * doctest::skip(loop_cases_skipped))
{
    const auto start = std::chrono::steady_clock::now();
    const InertialRecording &r3 = r3_recording();
    const OdometryConfig c3 = loop_candidate_config(r3.sensor);
    constexpr int r3_windows = 14;
    const LoopRun a = run_loop(r3, c3, LoopWorld::matched, r3_windows);
    const LoopRun b = run_loop(r3, c3, LoopWorld::matched, r3_windows);
    const InertialRecording &r2 = r2_recording();
    constexpr int r2_windows = 10;
    const LoopRun c = run_loop(r2, loop_candidate_config(r2.sensor), LoopWorld::matched, r2_windows);
    const double total_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    for (const LoopRun *run : {&a, &b, &c})
    {
        MESSAGE(run->summary(), preconditions_text(*run, false));
        MESSAGE("unavailable (NOT MEASURED) in ", run->route, ": ", run->unavailable.text());
        CHECK(run->completed);
        CHECK(run->valid());
        for (const std::string &why : run->failed())
            MESSAGE("failed precondition: ", why);
        CHECK(run->time_limit_windows == 0);
        CHECK(run->map_rebuilds == 0);
        CHECK(run->unavailable.records_missing == 0);
        CHECK(run->unavailable.handoff_not_done == 0);
        // the unavailable readings are counted: switches off, so the joint move is absent in every window
        CHECK(run->unavailable.move_joint == run->windows_run);
        CHECK(run->unavailable.total() > 0);
        // every window has its observability class (by the truth)
        int classified = 0;
        for (int o = 0; o < kObservabilityClasses; o++)
            classified += run->observability_windows[o];
        CHECK(classified == run->windows_run);
    }
    REQUIRE(a.windows_run == r3_windows);
    CHECK(a.windows_cut == r3.n_windows);
    CHECK(a.class_windows[int(WindowClass::rest)] >= 2);
    CHECK(a.class_windows[int(WindowClass::seen)] >= 1);

    std::string difference;
    const bool repeat = same_deterministic_fields(a, b, difference);
    CHECK_MESSAGE(repeat, "first difference: ", difference);

    REQUIRE(c.windows_run == r2_windows);
    CHECK(c.r2.present);
    CHECK(c.r2.turn_window > r2_windows);
    CHECK(c.r2.exit_window > c.r2.turn_window);
    CHECK_FALSE(c.r2.exit_in_run);
    CHECK(c.r2.landmark_cells.size() >= 20);
    CHECK(c.not_evaluated.size() == 1);

    MESSAGE("synthetic loop smoke: ", f(total_s, 1), " s in all (loop ", f(a.seconds, 1), " + ", f(b.seconds, 1), " + ", f(c.seconds, 1), " s; ",
            a.windows_run + b.windows_run + c.windows_run, " windows); repeat ", std::string(repeat ? "bit for bit" : "DIFFERS"), "; R2 design: turn window ",
            c.r2.turn_window, ", exit window ", c.r2.exit_window, " (", c.r2.exit_end_wall_hits, " end-wall hits), ", c.r2.landmark_cells.size(),
            " landmark cells");
    if (total_s > 60.0)
        WARN_MESSAGE(false, "the smoke case took more than one minute: ", f(total_s, 1), " s");
}

// ---------------------------------------------------------------------------------------------------------------------
// On demand (task 6.2 measures; this lists): each route whole, switches off, its summary, its failed preconditions, the R2
// design and the per-window positions against the truth every 5 windows.
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("synthetic loop: whole routes R1, R2, R3, preconditions listed" * doctest::skip())
{
    for (const InertialRecording *rec : {&r1_recording(), &r2_recording(), &r3_recording()})
    {
        const LoopRun run = run_loop(*rec, loop_candidate_config(rec->sensor), LoopWorld::matched);
        MESSAGE(run.summary(), preconditions_text(run, false));
        MESSAGE("unavailable: ", run.unavailable.text());
        std::ostringstream rest;
        for (const LoopWindow &w : run.windows)
            if (w.record && w.record->at_rest && w.cls != WindowClass::rest)
                rest << " w" << w.window << " (" << window_class_name(w.cls) << ")";
        MESSAGE("moving windows judged at rest:", rest.str().empty() ? std::string(" none") : rest.str());
        if (!run.windows.empty())
        {
            const LoopWindow &first = run.windows.front(), &last = run.windows.back();
            const Eigen::Vector3d start = first.truth.poses.front().translation();
            MESSAGE("end (last node of w", last.window, "): error ", f(last.errors.last_position.norm(), 3), " m (", f(last.errors.last_position.x(), 3), " ",
                    f(last.errors.last_position.y(), 3), " ", f(last.errors.last_position.z(), 3), "); from the start solved ",
                    f((last.poses.back().translation() - start).norm(), 3), " m, truth ", f((last.truth.poses.back().translation() - start).norm(), 3), " m");
        }
        if (run.r2.present)
            MESSAGE("R2: turn ", run.r2.turn_window, ", exit ", run.r2.exit_window, " (end-wall hits ", run.r2.exit_end_wall_hits, "), landmark cells ",
                    run.r2.landmark_cells.size(), ", in the map before the exit ", run.r2.landmark_cells_mapped, ", exit points ", run.r2.exit_points,
                    ", in landmark keys ", run.r2.exit_points_in_landmark_keys, ", accepted anywhere ", run.r2.exit_points_accepted, ", accepted landmark matches ",
                    run.r2.exit_landmark_matches);
        std::ostringstream t;
        for (const LoopWindow &w : run.windows)
            if (w.window % 5 == 0 || w.window == run.windows_run)
            {
                const Eigen::Vector3d e = w.poses.back().translation(), g = w.truth.poses.back().translation();
                t << "\n  w" << w.window << " " << window_class_name(w.cls) << " solved " << f(e.x(), 3) << " " << f(e.y(), 3) << " " << f(e.z(), 3)
                  << " truth " << f(g.x(), 3) << " " << f(g.y(), 3) << " " << f(g.z(), 3) << " at rest " << (w.record ? int(w.record->at_rest) : -1);
            }
        MESSAGE(t.str());
        CHECK(run.completed);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// On demand (task 6.1a (3)): the controls beside the production setting, per route: the matched world with the shipped
// filter gain 0.05 (production), the noise-free world with gain 0.05, the noise-free world with gain 0. One PROBE line per
// run, then the failed preconditions and the observability every 5 windows.
//   lidar_odometry_tests -tc="synthetic loop controls*" --no-skip -s
// ---------------------------------------------------------------------------------------------------------------------
namespace
{
    void control_runs(int route)
    {
        struct Setting
        {
            LoopWorld world;
            double gain;
        };
        for (const Setting &set : {Setting{LoopWorld::matched, 0.05}, Setting{LoopWorld::noise_free, 0.05}, Setting{LoopWorld::noise_free, 0.0}})
        {
            const InertialRecording rec = loop_recording(route, set.world);
            OdometryConfig config = loop_candidate_config(rec.sensor);
            config.ahrs_gain = set.gain;
            const LoopRun run = run_loop(rec, config, set.world);
            MESSAGE(probe_line(run), "\n", run.summary(), preconditions_text(run, false), observability_table(run, 5));
            if (run.r2.present)
                MESSAGE("R2 blind windows (projected NDT shift weight along / sideways):", blind_axis_table(run));
            if (run.r2.present)
                MESSAGE("R2: turn ", run.r2.turn_window, ", exit ", run.r2.exit_window, " (end-wall hits ", run.r2.exit_end_wall_hits, "), landmark cells ",
                        run.r2.landmark_cells.size(), ", in the map before the exit ", run.r2.landmark_cells_mapped, ", accepted landmark matches ",
                        run.r2.exit_landmark_matches);
            CHECK(run.completed);
        }
    }
}

TEST_CASE("synthetic loop controls: R1" * doctest::skip()) { control_runs(1); }
TEST_CASE("synthetic loop controls: R2" * doctest::skip()) { control_runs(2); }
TEST_CASE("synthetic loop controls: R3" * doctest::skip()) { control_runs(3); }

// the matched world of R3 over independent seeds (the sensor's seed draws the points and every IMU error)
TEST_CASE("synthetic loop controls: R3 matched, seeds 1 to 3" * doctest::skip())
{
    for (std::uint64_t seed = 1; seed <= 3; seed++)
    {
        const InertialRecording rec = loop_recording(3, LoopWorld::matched, seed);
        const LoopRun run = run_loop(rec, loop_candidate_config(rec.sensor), LoopWorld::matched);
        MESSAGE("seed ", seed, ": ", probe_line(run), preconditions_text(run, true));
        CHECK(run.completed);
    }
}

// the labelled model-mismatch case: the code's default world (ImuNoise{}), the world of task 6.1's first runs; its
// "world matches settings" preconditions fail by design (the run is never valid)
TEST_CASE("synthetic loop controls: the default world (model mismatch), R1 R2 R3" * doctest::skip())
{
    for (int route : {1, 2, 3})
    {
        const InertialRecording rec = loop_recording(route, LoopWorld::default_mismatch);
        const LoopRun run = run_loop(rec, loop_candidate_config(rec.sensor), LoopWorld::default_mismatch);
        MESSAGE(probe_line(run), preconditions_text(run, true));
        CHECK_FALSE(run.valid());
    }
}

// The observability classes without a run (seconds): R1 and R3 in loop_scene (before task 6.1a) and with the along-travel
// structure, R2 in its own scene, in the matched world.
TEST_CASE("synthetic loop controls: observability by scene, no run" * doctest::skip())
{
    for (int route : {1, 3})
        for (const bool along : {false, true})
        {
            const InertialRecording rec = loop_recording(route, LoopWorld::matched, 1, along);
            const LoopRun run = observe_loop(rec, loop_candidate_config(rec.sensor), LoopWorld::matched);
            MESSAGE(run.route, std::string(along ? " with along-travel structure: " : " in loop_scene: "), run.observability_text(), observability_table(run, 3));
        }
    const InertialRecording rec = loop_recording(2, LoopWorld::matched);
    const LoopRun run = observe_loop(rec, loop_candidate_config(rec.sensor), LoopWorld::matched);
    MESSAGE("R2: ", run.observability_text(), observability_table(run, 3));
}
