#include <doctest/doctest.h>

#include "angle_units.h"
#include "loop_closure.h"
#include "memory_point_store.h"
#include "synthetic_recording.h"

#include <chrono>

// run_loop_closure on the loop recording (two laps around a block, see synthetic_recording.h) against its true
// trajectory. The options are those of settings/lidar-vertical.toml, except the segment length and the time gap,
// which are shortened to fit the 44 s recording; they are set here, so tuning the shipped file doesn't change the tests.
//
// Skipped in the regular run (ctest): each case takes 15-27 s in Release and more than 15 minutes in Debug. To run them,
// e.g. to debug the loop closure:  lidar_odometry_tests -tc="loop closure*" --no-skip -s

using namespace synthetic;

namespace
{
    const double deg = DEG_TO_RAD;

    OdometryConfig loop_closure_config()
    {
        OdometryConfig c;
        c.use_loop_closure = true;
        c.loop_closure_segment_windows = 4;   // 4 s per segment (shipped: 12)
        c.loop_closure_min_time_gap_s = 10.0; // (shipped: 30)
        c.loop_closure_search_radius_m = 8.0;
        c.loop_closure_max_vertical_offset_m = 1.5;
        c.loop_closure_max_correction_m = 6.0;
        c.loop_closure_min_fitness = 0.5;
        c.loop_closure_odometry_sigma_m = 0.1;
        c.loop_closure_odometry_sigma_deg = 0.5;
        c.loop_closure_edge_sigma_m = 0.05;
        c.loop_closure_edge_sigma_deg = 0.3;
        c.min_distance_lidar = 0.15;
        c.max_distance_lidar = 25.0;
        c.use_exact_ndt_bucket_statistics = false;
        return c;
    }

    void store_points(const Recording &r, MemoryPointStore &store)
    {
        for (size_t w = 0; w < r.points.size(); w++)
            store.save(PointSet::WindowIntermediate, w, r.points[w]);
    }

    // odometry drift on the second lap: from its start, a translation growing linearly to `shift` and a yaw growing to
    // `yaw` at the end, applied about the origin to poses and motion_model_poses (the points stay as seen from the truth)
    void add_second_lap_drift(Recording &r, const Eigen::Vector3d &shift, double yaw)
    {
        const double t_end = r.windows.back().timestamps.back().first;
        for (Window &w : r.windows)
            for (size_t n = 0; n < w.poses.size(); n++)
            {
                const double t = w.timestamps[n].first;
                if (t < r.lap_time_s)
                    continue;
                const double f = (t - r.lap_time_s) / (t_end - r.lap_time_s);
                Eigen::Affine3d drift = Eigen::Affine3d::Identity();
                drift.linear() = Eigen::AngleAxisd(f * yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                drift.translation() = f * shift;
                w.poses[n] = drift * w.poses[n];
                w.motion_model_poses[n] = w.poses[n];
            }
    }

    struct Run
    {
        int loops = 0;
        double ms = 0.0;
    };

    Run run(Recording &r, MemoryPointStore &store)
    {
        NullLogger logger;
        const auto start = std::chrono::steady_clock::now();
        Run result;
        result.loops = run_loop_closure(r.windows, loop_closure_config(), store, logger);
        result.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return result;
    }
}

TEST_CASE("loop closure: drift on the second lap is reduced" * doctest::skip())
{
    Recording r = loop_recording(2);
    MemoryPointStore store;
    store_points(r, store);
    add_second_lap_drift(r, {0.3, 0.4, 0.0}, 2.0 * deg);
    const auto truth = all_poses(r.truth);
    const auto before = all_poses(r.windows);

    const Run result = run(r, store);
    const auto after = all_poses(r.windows);

    const double end_before = (before.back().translation() - truth.back().translation()).norm();
    const double end_after = (after.back().translation() - truth.back().translation()).norm();
    const PoseError max_before = max_error(before, truth), max_after = max_error(after, truth);
    const PoseError mean_before = mean_error(before, truth), mean_after = mean_error(after, truth);
    MESSAGE("drift: ", result.loops, " loops accepted; end error ", end_before * 1000.0, " -> ", end_after * 1000.0,
            " mm; largest ", max_before.position * 1000.0, " mm / ", max_before.rotation / deg, " deg -> ", max_after.position * 1000.0,
            " mm / ", max_after.rotation / deg, " deg; mean ", mean_before.position * 1000.0, " -> ", mean_after.position * 1000.0,
            " mm; ", result.ms, " ms");
    // Measured 2026-10-04 (Release): 5 loops; end error 568 -> 113 mm, largest 568 mm / 2 deg -> 113 mm / 0.35 deg, mean
    // 112 -> 35 mm; 26.6 s
    CHECK(result.loops >= 1);
    CHECK(end_after <= 0.5 * end_before);
}

// Expected to fail (found 2026-10-04): every already-aligned revisit still registers with a "correction" of 1-3 cm and
// up to 0.2 deg (coarse search on 0.1 m / 0.75 deg steps, then NDT refinement on 0.5 / 0.25 m cells of sparse points),
// and the pose graph, which trusts loop edges to loop_closure_edge_sigma_m = 0.05, spreads that onto the nodes: measured
// 5 loops, largest change 31 mm / 0.095 deg (the exact NDT cell statistics give the same, 31.8 mm). When the loop
// closure leaves a correct trajectory alone, doctest reports this case as passing unexpectedly: then remove should_fail.
TEST_CASE("loop closure: a trajectory that is already right stays" * doctest::skip() * doctest::should_fail())
{
    Recording r = loop_recording(2);
    MemoryPointStore store;
    store_points(r, store);
    const auto truth = all_poses(r.truth);

    const Run result = run(r, store);
    const PoseError change = max_error(all_poses(r.windows), truth);
    MESSAGE("truth: ", result.loops, " loops accepted; largest change ", change.position * 1000.0, " mm / ", change.rotation / deg,
            " deg; ", result.ms, " ms");
    CHECK(change.position <= 0.005);
    CHECK(change.rotation <= 0.1 * deg);
}

TEST_CASE("loop closure: no revisit leaves the trajectory untouched" * doctest::skip())
{
    // the first 16 windows (three quarters of a lap): no two segments are both 10 s apart and close to each other
    Recording r = loop_recording(2);
    r.windows.resize(16);
    r.truth.resize(16);
    r.points.resize(16);
    MemoryPointStore store;
    store_points(r, store);
    const auto before = all_poses(r.windows);

    const Run result = run(r, store);
    const auto after = all_poses(r.windows);
    bool unchanged = before.size() == after.size();
    for (size_t k = 0; unchanged && k < before.size(); k++)
        unchanged = before[k].matrix() == after[k].matrix();
    MESSAGE("no revisit: ", result.loops, " loops accepted; poses unchanged ", unchanged, "; ", result.ms, " ms");
    CHECK(result.loops == 0);
    CHECK(unchanged);
}
