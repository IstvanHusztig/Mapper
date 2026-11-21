#include <doctest/doctest.h>

#include "synthetic_loop.h"

#include <algorithm>
#include <chrono>
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
// The whole routes are task 6.2's (minutes each, about 0.55 s per window in Release: R1 79, R2 124, R3 35 windows); the case
// below only lists what each gives, on demand:
//   lidar_odometry_tests -tc="synthetic loop: whole routes*" --no-skip -s
// Measured while building the runner (2026-11-21, switches off, 1 cm jitter): R2 completes (124 windows, 68.6 s, no time
// limit, no rebuild) but its blind and exit preconditions fail, because window 12 (constant 1.5 m/s, no gait) passes the
// rest test (A2 / F10), the end velocity drops to 0.22 m/s and the estimate stalls in the corridor (27 m behind at the
// exit); R3 (walking) completes and its solved last node ends 5.8 m short of the true 9.6 m walk (a reading, not checked).

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

    // the loop's sensor: the routes' profile with 1 cm jitter along the face normal. With the routes' 3 mm (below K9's
    // 3.162 mm) the exact cells of axis-aligned planes are skipped and the LiDAR sees only edges and corners: measured 0
    // accepted matches at R2's exit. loop_chains_test uses 1 cm for the same reason.
    SensorProfile loop_sensor()
    {
        SensorProfile s;
        s.jitter_sigma_m = 0.01;
        return s;
    }

    const InertialRecording &r1_recording()
    {
        static const InertialRecording rec = generate_recording(route_r1(), loop_sensor());
        return rec;
    }

    const InertialRecording &r2_recording()
    {
        static const InertialRecording rec = []
        {
            SensorProfile s = loop_sensor();
            s.max_range_m = 6.0;
            return generate_recording(route_r2(), s);
        }();
        return rec;
    }

    const InertialRecording &r3_recording()
    {
        static const InertialRecording rec = generate_recording(route_r3(), loop_sensor());
        return rec;
    }
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
    const LoopRun a = run_loop(r3, c3, r3_windows);
    const LoopRun b = run_loop(r3, c3, r3_windows);
    const InertialRecording &r2 = r2_recording();
    constexpr int r2_windows = 10;
    const LoopRun c = run_loop(r2, loop_candidate_config(r2.sensor), r2_windows);
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
        const LoopRun run = run_loop(*rec, loop_candidate_config(rec->sensor));
        MESSAGE(run.summary(), preconditions_text(run, true));
        MESSAGE("unavailable: ", run.unavailable.text());
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
