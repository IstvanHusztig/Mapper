#include <doctest/doctest.h>

#include "angle_units.h"
#include "memory_point_store.h"
#include "ndt_map.h"
#include "run_odometry.h"
#include "synthetic_recording.h"
#include "synthetic_routes.h"
#include "toml_config_repository.h"
#include "truth_statistics.h"
#include "window_loop.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

// F-e of the consistency harness (openspec change consistency-harness, task 5.5; design D5, D7 rows K1, K2, K11 and the
// F2 row): short chains through the REAL loop steps (process_windows) from a known carried state and a map of true points.
// Each chain is MEASURED as a paired sensitivity (with and without one change, on the same data) and reported beside the
// audit's hypothesis. Nothing is marked doctest::should_fail (task 5.8, after the orchestrator's review); the CHECKs are the
// fixture's validity (blindness, the injected error retained after preparation, matches to the designated structure, no map
// rebuild), never the measured outcome.
//
// The fixture (hand-built analytic windows, as K3 of task 5.4, not the generator's front-end cut: the attitude is exact,
// so a turned boundary is the only attitude error, and the truth is a closed form):
//  - level motion along world x, identity attitude, a perfect gyro (imu_angles zero, no gyro noise, no gyro bias);
//  - 500 Hz, 180-node windows (the Livox timing); window w >= 1 covers motion time [(w - 1) T, w T); window 0 is a rest
//    window that only gives the loop its gravity (start_window_loop);
//  - the windows are in the front end's form (identity poses, zero translations: the loop predicts them), except window 1,
//    the predecessor history, which holds the true poses as if solved;
//  - accelerometer sample k = the exact mean specific force over [t_k, t_k + 2 ms) (synthetic_inertial.h's convention),
//    plus, where stated, white noise of the model's density (ImuNoise: 0.01056 m/s2/sqrt(Hz), 0.236 m/s2 per sample);
//  - points: 20 rays per node from the true pose, uniform on the sphere, cast on a box scene (synthetic::cast_ray) up to
//    the range, each hit moved by N(0, 1 cm) along its face normal (exact cell statistics need more than 3.2 mm, K9);
//  - the grids seeded with update_rgd from the scene's faces sampled at uniform random positions, one per 25 cm2, with the same jitter (a regular
//    lattice put rows of map points on cell boundaries; the map is exact at
//    entry; every finished window's solved points enter it, as in a run);
//  - the carry set consistently before window 2: velocity valid, its mean the true velocity at window 1's last node, its
//    covariance (20 mm/s)^2, the end time, the gyro bias zero with (0.05 deg/s)^2, the offset covariance start_carry's;
//  - an injected error is put into the SOLVED predecessor between two process_windows calls (the predictor rebuilds the
//    next first pose from it), and read back from the next window's record.prepared.first_pose before any result is used.
//
// Settings: settings/livox-vertical.toml (shipped) with the stage 8 candidate's differences (design D8; research.md "Limits
// of the evidence"): use_exact_ndt_bucket_statistics true, degenerate_projection_ratio 0.02 / _full 0.04,
// imu_acceleration_offset_rw_mps2_sqrt_s 0.01 (ahrs_gain plays no part: no AHRS here). And for the test: log_window_terms
// true (the records), real_time_threshold_seconds 1000 (no time limit: the result must not depend on the machine).

using synthetic::NullLogger;

namespace
{
    constexpr double dt = 0.002;
    constexpr double g0 = 9.80665;
    constexpr int T = 180;
    constexpr double t_origin = 100.0;

    // level motion along x: x(tau) = x0 + v0 tau + a tau^2 / 2 + (A / w^2)(1 - cos w tau)
    struct Motion1D
    {
        double x0 = 0.0, v0 = 0.0, accel = 0.0, surge = 0.0, omega = 1.0;
        double x(double tau) const { return x0 + v0 * tau + 0.5 * accel * tau * tau + surge / (omega * omega) * (1.0 - std::cos(omega * tau)); }
        double v(double tau) const { return v0 + accel * tau + surge / omega * std::sin(omega * tau); }
    };

    double node_tau(int w, int k) { return ((w - 1) * T + k) * dt; }
    double node_time(int w, int k) { return t_origin + (w * T + k) * dt; }

    struct ChainSpec
    {
        synthetic::Scene scene; // already shifted off the cell boundaries
        Motion1D motion;
        double y = 0.0, z = 0.0; // the path's lateral position
        double range = 6.0;
        int rays_per_node = 20;
        double jitter_m = 0.01;
        double accel_noise_mps2 = 0.0; // per sample
        int n_windows = 0;             // including window 0
        std::uint64_t seed = 1;
        double carry_sigma_mps = 0.02;
        bool draw_carry_error = false; // the carried velocity's error drawn from its claimed covariance
    };

    Eigen::Vector3d true_position(const ChainSpec &s, double tau) { return Eigen::Vector3d(s.motion.x(tau), s.y, s.z); }

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

    class KeepTerms : public WindowTermSink
    {
    public:
        void write(const WindowTermRecord &record) override { records.push_back(record); }
        std::vector<WindowTermRecord> records;
    };

    OdometryConfig candidate_config()
    {
        OdometryConfig c;
        RunPaths paths;
        GuiPreferences gui;
        TomlConfigRepository repo;
        const bool loaded = repo.load(std::string(LO_REPO_DIR) + "/settings/livox-vertical.toml", c, paths, gui);
        REQUIRE(loaded);
        c.use_exact_ndt_bucket_statistics = true;
        c.degenerate_projection_ratio = 0.02;
        c.degenerate_projection_ratio_full = 0.04;
        c.imu_acceleration_offset_rw_mps2_sqrt_s = 0.01;
        c.log_window_terms = true;
        c.real_time_threshold_seconds = 1000.0;
        return c;
    }

    // the scene's faces sampled at one point per 25 cm2, jittered along the normal, moved by `shift`
    std::vector<Point3Di> map_points(const ChainSpec &s, truth_stats::Rng &rng, const Eigen::Vector3d &shift)
    {
        std::vector<Point3Di> points;
        for (const synthetic::Face &f : s.scene.faces)
        {
            const int a = (f.axis + 1) % 3, b = (f.axis + 2) % 3;
            const int nu = std::max(1, int(std::ceil((f.hi[a] - f.lo[a]) / 0.05)));
            const int nv = std::max(1, int(std::ceil((f.hi[b] - f.lo[b]) / 0.05)));
            // uniform random in-plane positions (one per 5 cm x 5 cm on average): a regular lattice would put whole rows of
            // map points exactly on cell boundaries, and any shift of the map would then move them across (a step, not a
            // sensitivity: measured 28.5 mm/s for a 1 um shift before this)
            for (int i = 0; i < nu; i++)
                for (int j = 0; j < nv; j++)
                {
                    Point3Di p{};
                    const double u = rng.uniform(), v = rng.uniform();
                    p.point = synthetic::face_point(f, u, v, s.jitter_m * rng.normal()) + shift;
                    p.index_point = int(points.size());
                    points.push_back(p);
                }
        }
        return points;
    }

    void seed_grids(OdometryState &state, const ChainSpec &s, const OdometryConfig &c, const Eigen::Vector3d &shift, std::uint64_t seed)
    {
        truth_stats::Rng rng(seed);
        state.buckets_indoor.clear();
        state.buckets_outdoor.clear();
        std::vector<Point3Di> points = map_points(s, rng, shift);
        update_rgd(c.in_out_params_indoor, state.buckets_indoor, points, s.scene.centre, c.use_exact_ndt_bucket_statistics);
        if (c.ablation_study_use_hierarchical_rgd)
            update_rgd(c.in_out_params_outdoor, state.buckets_outdoor, points, s.scene.centre, c.use_exact_ndt_bucket_statistics);
    }

    struct ChainWorld
    {
        std::vector<Window> windows;
        std::vector<std::vector<Eigen::Affine3d>> truth;
        MemoryPointStore store;
        OdometryState state;
    };

    void build_world(ChainWorld &world, const ChainSpec &s, const OdometryConfig &c)
    {
        truth_stats::Rng rng(s.seed);
        world.windows.assign(s.n_windows, Window{});
        world.truth.assign(s.n_windows, {});
        for (int w = 0; w < s.n_windows; w++)
        {
            Window &win = world.windows[w];
            int point_index = 0;
            std::vector<Point3Di> points;
            for (int k = 0; k < T; k++)
            {
                const double tau = w == 0 ? 0.0 : node_tau(w, k);
                Eigen::Affine3d truth = Eigen::Affine3d::Identity();
                truth.translation() = true_position(s, tau);
                world.truth[w].push_back(truth);
                win.timestamps.push_back({node_time(w, k), node_time(w, k)});
                win.imu_angles.push_back(Eigen::Vector3d::Zero());
                if (w == 0)
                {
                    win.poses.push_back(truth);
                    win.imu_acc.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
                    continue;
                }
                win.poses.push_back(Eigen::Affine3d::Identity());
                const double a_mean = (s.motion.v(tau + dt) - s.motion.v(tau)) / dt;
                const double n = s.accel_noise_mps2;
                win.imu_acc.push_back(Eigen::Vector3d((a_mean + n * rng.normal()) / g0, n * rng.normal() / g0, 1.0 + n * rng.normal() / g0));
                for (int r = 0; r < s.rays_per_node; r++)
                {
                    Eigen::Vector3d dir(rng.normal(), rng.normal(), rng.normal());
                    dir.normalize();
                    double distance = 0.0;
                    int face = -1;
                    const Eigen::Vector3d origin = truth.translation();
                    if (!synthetic::cast_ray(s.scene, origin, dir, s.range, distance, face))
                        continue;
                    Eigen::Vector3d hit = origin + distance * dir;
                    hit[s.scene.faces[face].axis] += s.jitter_m * rng.normal();
                    Point3Di p{};
                    p.point = hit - origin; // identity attitude: the body frame is the world frame moved to the node
                    p.timestamp = node_time(w, k);
                    p.index_pose = k;
                    p.index_point = point_index++;
                    points.push_back(p);
                }
            }
            win.motion_model_poses = win.poses;
            if (w >= 1)
                world.store.save(PointSet::WindowIntermediate, w, points);
        }
        // the predecessor history: window 1 as solved, at the truth
        world.windows[1].poses = world.truth[1];
        world.windows[1].motion_model_poses = world.truth[1];
        seed_grids(world.state, s, c, Eigen::Vector3d::Zero(), s.seed + 1000);
    }

    // a change made between two process_windows calls, to the solved window `after` (its successor's predecessor)
    struct Injection
    {
        int after = 1;            // the window modified; the chain is processed 2..after, then after + 1..last
        double along_m = 0.0;     // its poses moved along travel (x, its own body x)
        double pitch_deg = 0.0;   // its attitudes turned about its body y
        std::optional<Eigen::Vector3d> map_shift; // the grids re-seeded from the true points moved by this (K1)
    };

    struct ChainRun
    {
        bool ok = false;
        std::vector<WindowTermRecord> records; // index = window - 2
        std::vector<int> end_matches;          // per window: points on the designated face matched to a usable cell
        int map_rebuilds = 0;
        double seconds = 0.0;
    };

    ChainRun run_chain(const ChainSpec &s, const OdometryConfig &c, const Injection &inj, int designated_face = -1)
    {
        ChainRun run;
        ChainWorld world;
        build_world(world, s, c);
        NoPreview preview;
        NoProgress progress;
        NullLogger logger;
        double ts_failure = 0.0;
        WindowLoop loop;
        start_window_loop(loop, world.windows[0], c, logger);
        truth_stats::Rng rng(s.seed + 2000);
        loop.carry.velocity_valid = true;
        loop.carry.velocity_mean = Eigen::Vector3d(s.motion.v(node_tau(1, T - 1)), 0.0, 0.0);
        if (s.draw_carry_error)
            loop.carry.velocity_mean += s.carry_sigma_mps * Eigen::Vector3d(rng.normal(), rng.normal(), rng.normal());
        loop.carry.velocity_cov = Eigen::Matrix3d::Identity() * (s.carry_sigma_mps * s.carry_sigma_mps);
        loop.carry.velocity_time_s = node_time(1, T - 1);
        const double sb = 0.05 * DEG_TO_RAD;
        loop.carry.bias.mean.setZero();
        loop.carry.bias.covariance = Eigen::Matrix3d::Identity() * (sb * sb);

        KeepTerms sink;
        WindowRange range;
        range.terms = &sink;
        const size_t last = world.windows.size() - 1;
        const auto start = std::chrono::steady_clock::now();
        run.ok = true;
        if (inj.after >= 2)
            run.ok = process_windows(loop, world.windows, 2, inj.after, c, world.state, ts_failure, false, world.store, preview, logger,
                                     progress, range);
        for (Eigen::Affine3d &p : world.windows[inj.after].poses)
        {
            p.translation() += p.linear() * Eigen::Vector3d(inj.along_m, 0.0, 0.0);
            p.linear() = p.linear() * Eigen::AngleAxisd(inj.pitch_deg * DEG_TO_RAD, Eigen::Vector3d::UnitY()).toRotationMatrix();
        }
        if (inj.map_shift)
            seed_grids(world.state, s, c, *inj.map_shift, s.seed + 1000);
        if (run.ok)
            run.ok = process_windows(loop, world.windows, inj.after + 1, last, c, world.state, ts_failure, false, world.store, preview,
                                     logger, progress, range);
        run.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        run.records = sink.records;
        run.map_rebuilds = loop.carry.map_rebuilds;
        run.ok = run.ok && run.records.size() == last - 1;

        // the matches to the designated face, computed from the window's points placed with its final poses and the fine grid
        const Eigen::Vector3d b(c.in_out_params_indoor.resolution_X, c.in_out_params_indoor.resolution_Y, c.in_out_params_indoor.resolution_Z);
        for (size_t w = 2; w <= last; w++)
        {
            int matches = 0;
            if (designated_face >= 0)
            {
                const synthetic::Face &f = s.scene.faces[designated_face];
                std::vector<Point3Di> points;
                world.store.load(PointSet::WindowIntermediate, w, points);
                for (const Point3Di &p : points)
                {
                    const Eigen::Vector3d g = world.windows[w].poses[p.index_pose] * p.point;
                    if (std::abs(g[f.axis] - f.value) > 0.05)
                        continue;
                    const auto it = world.state.buckets_indoor.find(ndt_cell_index(g, b));
                    Eigen::Matrix3d scratch;
                    if (it != world.state.buckets_indoor.end() && ndt_cell_inverse(it->second, scratch) != nullptr)
                        matches++;
                }
            }
            run.end_matches.push_back(matches);
        }
        return run;
    }

    // what a record says along travel (x)
    struct Reading
    {
        double first_x_error = 0.0;      // prepared first pose minus the truth [m]
        double first_pitch_deg = 0.0;    // prepared first attitude's turn about y [deg]
        double prior_mean_error = 0.0;   // start velocity prior mean minus the true start velocity [m/s]
        double prior_sigma = 0.0;
        double start_error = 0.0;        // solved start velocity minus the truth [m/s]
        double start_move = 0.0;         // solved start velocity minus its prior mean [m/s]
        double end_error = 0.0;          // handed-on end velocity minus the truth at the last node [m/s]
        double end_sigma = 0.0;          // its claimed sigma along x
        Eigen::Vector3d bias_deg_s = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias_sigma_deg_s = Eigen::Vector3d::Zero();
        double ndt_along = 0.0, ndt_sideways = 0.0;  // projected NDT weight of a shift along travel / sideways (result)
        double raw_along_share = 0.0;    // u^T I u / tr I of the unweighted translation information (indoor grid)
        bool at_rest = false;
        int iterations = 0;
        double delta = 0.0; // the last iteration's update
    };

    Reading read(const WindowTermRecord &r, const ChainSpec &s)
    {
        Reading m;
        const int w = r.window - 1;
        if (r.prepared.first_pose)
        {
            m.first_x_error = r.prepared.first_pose->translation().x() - s.motion.x(node_tau(w, 0));
            const Eigen::AngleAxisd aa(r.prepared.first_pose->linear());
            m.first_pitch_deg = aa.angle() * aa.axis().y() * RAD_TO_DEG;
        }
        if (r.prepared.velocity_prior_mean)
        {
            m.prior_mean_error = r.prepared.velocity_prior_mean->x() - s.motion.v(node_tau(w, 0));
            m.prior_sigma = std::sqrt((*r.prepared.velocity_prior_cov)(0, 0));
            m.start_move = r.velocity_start.x() - r.prepared.velocity_prior_mean->x();
        }
        m.start_error = r.velocity_start.x() - s.motion.v(node_tau(w, 0));
        if (r.handoff == WindowHandOff::done && r.handed_on.velocity_end)
        {
            m.end_error = r.handed_on.velocity_end->x() - s.motion.v(node_tau(w, T - 1));
            m.end_sigma = std::sqrt((*r.handed_on.velocity_end_cov)(0, 0));
        }
        if (r.handoff == WindowHandOff::done && r.handed_on.bias_mean_rad_s)
        {
            m.bias_deg_s = *r.handed_on.bias_mean_rad_s * RAD_TO_DEG;
            m.bias_sigma_deg_s = r.handed_on.bias_cov->diagonal().cwiseMax(0.0).cwiseSqrt() * RAD_TO_DEG;
        }
        for (const WindowTermRow &row : r.result.rows)
            if (row.name == "ndt_indoor" || row.name == "ndt_outdoor")
            {
                m.ndt_along += row.weight[0][0];
                m.ndt_sideways += row.weight[0][1];
            }
        const Eigen::Matrix3d &I = r.result.translation_information;
        if (I.trace() > 0.0)
            m.raw_along_share = r.result.directions[0].dot(I * r.result.directions[0]) / I.trace();
        m.at_rest = r.at_rest;
        m.iterations = r.iterations;
        m.delta = r.delta;
        return m;
    }

    std::string f(double v, int precision = 2)
    {
        std::ostringstream o;
        o << std::fixed << std::setprecision(precision) << v;
        return o.str();
    }

    // ---- the corridor of the blind stretch and of K2 ----
    // a closed corridor 2 m wide, 2.5 m high, from a back wall at x = -3 to an end wall at x = 14.5 (the designated
    // structure of the exit, face index 1), range 6 m. Path at y = 0, z = 1.2, 1.5 m/s with a 1 m/s2 surge at 1.5 Hz (so
    // that no window is taken as at rest, unit case A2). Windows 2..6 see the back wall, 7..15 see neither end (blind by
    // position), from 16 the end wall comes into range.
    constexpr int corridor_end_face = 1;
    constexpr double corridor_back = -3.0, corridor_end = 14.5;

    ChainSpec corridor_spec(std::uint64_t seed, double accel_noise)
    {
        ChainSpec s;
        s.scene = synthetic::shifted(synthetic::box_scene({corridor_back, -1.0, 0.0}, {corridor_end, 1.0, 2.5}, false), synthetic::grid_offset);
        s.motion.x0 = synthetic::grid_offset.x();
        s.motion.v0 = 1.5;
        s.motion.surge = 1.0;
        s.motion.omega = 2.0 * 3.14159265358979323846 * 1.5;
        s.y = synthetic::grid_offset.y();
        s.z = 1.2 + synthetic::grid_offset.z();
        s.range = 6.0;
        s.n_windows = 22;
        s.seed = seed;
        s.accel_noise_mps2 = accel_noise;
        return s;
    }

    // blind by position: every node of the window farther than the range (plus 5 jitter sigmas) from both end walls
    bool blind_by_position(const ChainSpec &s, int w)
    {
        const double back = s.scene.faces[0].value, end = s.scene.faces[corridor_end_face].value, margin = s.range + 5.0 * s.jitter_m;
        for (int k = 0; k < T; k++)
        {
            const double x = s.motion.x(node_tau(w, k));
            if (x - back <= margin || end - x <= margin)
                return false;
        }
        return true;
    }

    // ---- the seen room of K1, K11 and F2 ----
    // a closed room 12 m x 6 m x 3 m, range 10 m: both end walls, the side walls, floor and ceiling in view everywhere
    ChainSpec room_spec(const Motion1D &motion, int n_windows)
    {
        ChainSpec s;
        s.scene = synthetic::shifted(synthetic::box_scene({-3.0, -3.0, 0.0}, {9.0, 3.0, 3.0}, false), synthetic::grid_offset);
        s.motion = motion;
        s.motion.x0 = synthetic::grid_offset.x();
        s.y = synthetic::grid_offset.y() + 0.4; // off the centre line, so that the side walls are not symmetric
        s.z = 1.2 + synthetic::grid_offset.z();
        s.range = 10.0;
        s.n_windows = n_windows;
        return s;
    }

    Motion1D surge_motion(double v0)
    {
        Motion1D m;
        m.v0 = v0;
        m.surge = 1.0;
        m.omega = 2.0 * 3.14159265358979323846 * 1.5;
        return m;
    }

    constexpr double model_accel_noise = 0.01056 / 0.044721359549995794; // ImuNoise's density / sqrt(2 ms) [m/s2]
}

// The chains run every window to the full 100 iterations (the shipped rule), about 1 s per window: the cases below are
// doctest::skip(). Run them with
//   lidar_odometry_tests -tc="loop chain:*" --no-skip -s
// (about 4 min without the seeds case, which adds about 11 min). In every ctest: the smoke case "loop chain smoke".

// ---------------------------------------------------------------------------------------------------------------------
// The smoke case (in every ctest): the corridor to the second blind window, with and without a 0.10 m boundary error:
// the injected error is in the first blind window's prepared first pose, the blind windows are blind, no map rebuild.
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("loop chain smoke: the corridor's fixture is valid (blindness, injected error retained, no rebuild)")
{
    ChainSpec s = corridor_spec(1, model_accel_noise);
    s.n_windows = 9;
    const OdometryConfig c = candidate_config();
    Injection control;
    control.after = 6;
    Injection injected = control;
    injected.along_m = 0.10;
    const ChainRun a = run_chain(s, c, control), b = run_chain(s, c, injected);
    REQUIRE(a.ok);
    REQUIRE(b.ok);
    CHECK(a.map_rebuilds == 0);
    CHECK(b.map_rebuilds == 0);
    CHECK(blind_by_position(s, 7));
    CHECK(blind_by_position(s, 8));
    CHECK_FALSE(blind_by_position(s, 6));
    const double retained = read(b.records[7 - 2], s).first_x_error - read(a.records[7 - 2], s).first_x_error;
    CHECK(retained == doctest::Approx(0.10).epsilon(1e-6).scale(1.0));
    for (const ChainRun *r : {&a, &b})
        for (int w : {7, 8})
        {
            const Reading m = read(r->records[w - 2], s);
            CHECK(m.ndt_along / m.ndt_sideways < 1e-3);
            CHECK(m.ndt_sideways > 0.0);
            CHECK(m.raw_along_share < 0.02); // unweighted share, measured 0.006-0.007 (corner cells); the criterion is the projected weight
            CHECK_FALSE(m.at_rest);
        }
    MESSAGE("loop chain smoke: retained ", f(retained * 1e3, 3), " mm of 100 mm; ", f(a.seconds + b.seconds, 1), " s");
}

// ---------------------------------------------------------------------------------------------------------------------
// 1. The blind stretch, and 2. K2 (F1): its exit with a boundary position error injected in the predecessor's frame.
// One chain per setting (switches off / on) and injected error (0, 0.10, 0.24 m along travel), the same data (seed 1,
// model accelerometer noise): the error is put into window 6 (solved, the last one that sees the back wall), so it is
// carried through the blind windows 7..15 and meets the end wall's cells from window 16 on.
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("loop chain: the blind stretch and its exit (K2), switches off and on, boundary error 0 / 0.10 / 0.24 m" * doctest::skip())
{
    const ChainSpec s = corridor_spec(1, model_accel_noise);
    const int inject_after = 6;
    std::vector<int> blind;
    for (int w = 2; w < s.n_windows; w++)
        if (blind_by_position(s, w))
            blind.push_back(w);
    REQUIRE(!blind.empty());
    CHECK(blind.front() == inject_after + 1);
    const int last_blind = blind.back();
    // the designated exit window: the first whose first node is within range - 1 m of the end wall
    int exit_window = -1;
    for (int w = last_blind + 1; w < s.n_windows && exit_window < 0; w++)
        if (s.scene.faces[corridor_end_face].value - s.motion.x(node_tau(w, 0)) <= s.range - 1.0)
            exit_window = w;
    REQUIRE(exit_window > 0);
    MESSAGE("corridor: blind windows ", blind.front(), "..", last_blind, " (by position), designated exit window ", exit_window,
            ", last window ", s.n_windows - 1, "; window duration T = ", f((T - 1) * dt, 3), " s");

    double total_seconds = 0.0;
    for (const bool switches : {false, true})
    {
        OdometryConfig c = candidate_config();
        c.imu_edges_follow_state = switches;
        std::vector<ChainRun> runs;
        const std::vector<double> errors{0.0, 0.10, 0.24};
        for (double e : errors)
        {
            Injection inj;
            inj.after = inject_after;
            inj.along_m = e;
            runs.push_back(run_chain(s, c, inj, corridor_end_face));
            total_seconds += runs.back().seconds;
            REQUIRE(runs.back().ok);
            CHECK(runs.back().map_rebuilds == 0); // validity: no rebuild
        }
        const std::string tag = switches ? "switches ON (imu_edges_follow_state)" : "switches OFF";

        // validity: blindness (projected NDT weight along travel near zero, sideways kept) in every blind window, every run
        double worst_along_ratio = 0.0, least_sideways = 1e300, worst_raw_share = 0.0;
        for (const ChainRun &r : runs)
            for (int w : blind)
            {
                const Reading m = read(r.records[w - 2], s);
                worst_along_ratio = std::max(worst_along_ratio, m.ndt_along / m.ndt_sideways);
                least_sideways = std::min(least_sideways, m.ndt_sideways);
                worst_raw_share = std::max(worst_raw_share, m.raw_along_share);
                CHECK_FALSE(m.at_rest);
            }
        MESSAGE(tag, ": blindness over the blind windows of all three runs: projected NDT shift weight along / sideways at most ",
                worst_along_ratio, ", sideways at least ", least_sideways, "; unweighted information share along travel at most ",
                worst_raw_share);
        CHECK(worst_along_ratio < 1e-3);
        CHECK(least_sideways > 0.0);
        CHECK(worst_raw_share < 0.02); // set after measuring 0.0072 (the corner cells' normals); D5's criterion is the projected weight above

        // 1. the blind stretch (control run, no injected error): per window the handed-on velocity error along travel
        {
            std::ostringstream t;
            t << tag << ", the blind stretch (no injected error, seed 1): window | blind | v_end error [mm/s] | claimed sigma [mm/s] "
                        "| ratio | start move from prior [mm/s] | NDT along/sideways\n";
            for (int w = 2; w < s.n_windows; w++)
            {
                const Reading m = read(runs[0].records[w - 2], s);
                t << "  " << w << " | " << (blind_by_position(s, w) ? "blind" : "seen ") << " | " << f(m.end_error * 1e3) << " | "
                  << f(m.end_sigma * 1e3) << " | " << f(m.end_error / m.end_sigma) << " | " << f(m.start_move * 1e3) << " | "
                  << std::setprecision(3) << std::scientific << m.ndt_along / m.ndt_sideways << std::fixed << "\n";
            }
            MESSAGE(t.str());
        }

        // 2. K2: validity first: the injected error is in the first blind window's prepared first pose and is still there at
        // the exit (paired against the control's own error), and the exit window matches the end wall's mapped cells
        // validity of each run for K2 (D8: a run that fails a precondition is excluded and reported, not asserted): the
        // injected error retained within 5 % at the first window after the blind stretch, and at least 20 of the exit
        // window's points matched to usable cells of the end wall (the designated structure)
        std::vector<bool> valid(runs.size(), true);
        std::ostringstream validity;
        for (size_t j = 0; j < runs.size(); j++)
        {
            const double e = errors[j];
            const double at_first = read(runs[j].records[inject_after + 1 - 2], s).first_x_error - read(runs[0].records[inject_after + 1 - 2], s).first_x_error;
            const double at_exit_entry = read(runs[j].records[last_blind + 1 - 2], s).first_x_error - read(runs[0].records[last_blind + 1 - 2], s).first_x_error;
            CHECK(at_first == doctest::Approx(e).epsilon(1e-6).scale(1.0)); // the injection itself
            const bool retained = std::abs(at_exit_entry - e) <= 0.05 * e;
            const bool matched = runs[j].end_matches[exit_window - 2] >= 20;
            valid[j] = retained && matched;
            validity << "e = " << e << ": in window " << inject_after + 1 << " " << f(at_first * 1e3, 3) << " mm, in window " << last_blind + 1
                     << " " << f(at_exit_entry * 1e3, 3) << " mm" << (retained ? "" : " (NOT retained within 5 %)") << ", exit matches "
                     << runs[j].end_matches[exit_window - 2] << (matched ? "" : " (< 20)") << " -> " << (valid[j] ? "VALID" : "EXCLUDED") << "; ";
        }
        MESSAGE(tag, ", K2 validity per run: ", validity.str());

        std::ostringstream t;
        t << tag << ", K2 after the blind stretch: window | end-wall matches (e = 0 / 0.10 / 0.24) | NDT along/sideways (e = 0) | "
                    "per e: prepared first pose error [mm], v_start error, v_end error, claimed sigma [mm/s], ratio, and the difference to e = 0 [mm/s]\n";
        for (int w = last_blind; w < s.n_windows; w++)
        {
            const Reading c0 = read(runs[0].records[w - 2], s);
            t << "  " << w << (w == exit_window ? "*" : " ") << " | " << runs[0].end_matches[w - 2] << " / " << runs[1].end_matches[w - 2]
              << " / " << runs[2].end_matches[w - 2] << " | " << std::scientific << std::setprecision(2) << c0.ndt_along / c0.ndt_sideways
              << std::fixed;
            for (size_t j = 0; j < runs.size(); j++)
            {
                const Reading m = read(runs[j].records[w - 2], s);
                t << " | e=" << errors[j] << (valid[j] ? "" : " (excl.)") << ": x0 " << f(m.first_x_error * 1e3, 1) << " mm, " << f(m.start_error * 1e3, 1) << ", " << f(m.end_error * 1e3, 1) << ", "
                  << f(m.end_sigma * 1e3, 2) << ", " << f(m.end_error / m.end_sigma, 1);
                if (j > 0)
                    t << " (d " << f((m.start_error - c0.start_error) * 1e3, 1) << " / " << f((m.end_error - c0.end_error) * 1e3, 1) << ")";
            }
            t << "\n";
        }
        t << "  (* the designated exit window; hypothesis: a jump near e / T = " << f(0.10 / ((T - 1) * dt) * 1e3, 0) << " / "
          << f(0.24 / ((T - 1) * dt) * 1e3, 0) << " mm/s, then ringing)";
        MESSAGE(t.str());
    }
    MESSAGE("blind stretch / K2: 6 chains of 20 windows in ", f(total_seconds, 1), " s");
}

// ---------------------------------------------------------------------------------------------------------------------
// 1b. The blind stretch over several seeds (accelerometer noise, point jitter and the carried velocity's error redrawn):
// the summed normalised error of the last blind window's handed-on velocity along travel, against chi-square(M).
// Long: run with lidar_odometry_tests -tc="loop chain: the blind stretch over seeds*" --no-skip -s
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("loop chain: the blind stretch over seeds, the last blind window's normalised velocity error" * doctest::skip())
{
    const int M = 20;
    const truth_stats::PowerPlan plan = truth_stats::runs_for_power(1, 1.5, 0.01, 2);
    MESSAGE("design: one scalar per run (r = 1), two primary comparisons (off, on), 1 %, 90 % power for x1.5 and /1.5: M = ", plan.runs,
            " runs; this smoke check uses M = ", M);
    for (const bool switches : {false, true})
    {
        OdometryConfig c = candidate_config();
        c.imu_edges_follow_state = switches;
        double sum = 0.0, seconds = 0.0;
        int used = 0, last_blind = 0;
        std::ostringstream each;
        for (int seed = 1; seed <= M; seed++)
        {
            ChainSpec s = corridor_spec(100 + seed, model_accel_noise);
            s.n_windows = 17; // through the blind stretch and one window after it
            s.draw_carry_error = true;
            last_blind = 0;
            for (int w = 2; w < s.n_windows; w++)
                if (blind_by_position(s, w))
                    last_blind = w;
            const ChainRun r = run_chain(s, c, Injection{});
            seconds += r.seconds;
            if (!r.ok)
                continue;
            const Reading m = read(r.records[last_blind - 2], s);
            sum += (m.end_error / m.end_sigma) * (m.end_error / m.end_sigma);
            used++;
            each << f(m.end_error / m.end_sigma) << " ";
        }
        const auto interval = truth_stats::chi2_interval(used, 0.01 / 2);
        MESSAGE(std::string(switches ? "switches ON" : "switches OFF"), ": window ", last_blind, ", M = ", used, " valid runs: sum of normalised errors ", f(sum),
                " (", f(sum / used), " per run); the 1 % interval (2 comparisons) [", f(interval.first), ", ", f(interval.second),
                "]; ratios: ", each.str(), "; ", f(seconds, 1), " s");
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// 3. K1 (F1): a window in a seen room solved against the map displaced along travel by 1 mm and 10 mm, paired with the
// zero-shift control (the grids re-seeded from the same true points, unmoved). Noise-free accelerometer: the inertial data
// and the relative motions are exactly consistent. Three windows (2..4) first settle the carry on the exact map; window 5
// is the one measured (and 6 after it). Sensitivity only: the displaced map breaks the exact-map premise.
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("loop chain: K1, the velocity response to a map displaced along travel" * doctest::skip())
{
    const ChainSpec s = room_spec(surge_motion(1.0), 7);
    const double Tw = (T - 1) * dt;
    std::ostringstream t;
    double seconds = 0.0;
    t << "K1, seen room, window 5 (T = " << f(Tw, 3) << " s): setting | shift | v_start, v_end response [mm/s per mm] | "
         "x T [per T] | claimed sigma: prior / handed-on end [mm/s] | iterations | window 6 v_start response\n";
    for (const bool switches : {false, true})
    {
        OdometryConfig c = candidate_config();
        c.imu_edges_follow_state = switches;
        Injection base;
        base.after = 4;
        base.map_shift = Eigen::Vector3d::Zero();
        const ChainRun control = run_chain(s, c, base);
        REQUIRE(control.ok);
        CHECK(control.map_rebuilds == 0);
        seconds += control.seconds;
        const Reading c5 = read(control.records[5 - 2], s), c6 = read(control.records[6 - 2], s);
        // validity: the window is seen along travel, not at rest
        CHECK(c5.ndt_along > 0.1 * c5.ndt_sideways);
        CHECK_FALSE(c5.at_rest);
        t << "  " << (switches ? "switches on " : "switches off") << " | 0 (control) | v_start err " << f(c5.start_error * 1e3, 3) << ", v_end err "
          << f(c5.end_error * 1e3, 3) << " mm/s | | " << f(c5.prior_sigma * 1e3) << " / " << f(c5.end_sigma * 1e3) << " | " << c5.iterations << "\n";
        for (double d_mm : {0.001, 0.1, 1.0, 3.0, 10.0})
        {
            Injection inj = base;
            inj.map_shift = Eigen::Vector3d(d_mm * 1e-3, 0.0, 0.0);
            const ChainRun r = run_chain(s, c, inj);
            REQUIRE(r.ok);
            seconds += r.seconds;
            const Reading m5 = read(r.records[5 - 2], s), m6 = read(r.records[6 - 2], s);
            CHECK(m5.first_x_error == c5.first_x_error); // the shift is in the map only: the prepared window is the control's
            const double ds = (m5.start_error - c5.start_error) / d_mm * 1e3, de = (m5.end_error - c5.end_error) / d_mm * 1e3;
            t << "  " << (switches ? "switches on " : "switches off") << " | " << d_mm << " mm | " << f(ds, 3) << ", " << f(de, 3) << " | "
              << f(ds * Tw, 2) << ", " << f(de * Tw, 2) << " | " << f(m5.prior_sigma * 1e3) << " / "
              << f(m5.end_sigma * 1e3) << " | " << m5.iterations << " (last delta " << std::scientific << std::setprecision(2) << m5.delta
              << std::fixed << ") | " << f((m6.start_error - c6.start_error) / d_mm * 1e3, 3) << " | total dv_start, dv_end "
              << f((m5.start_error - c5.start_error) * 1e3, 3) << ", " << f((m5.end_error - c5.end_error) * 1e3, 3) << " mm/s\n";
        }
    }
    t << "  (a map displaced by +d along travel; the response x T is (dv / d) T, so the hypothesis [acc] 1.5-4 per T and [lidar] "
         "1-1.5 per T reads as 1.5-4 or 1-1.5 in that column; the sign: + means the velocity rises)";
    MESSAGE(t.str());
    MESSAGE("K1: 6 chains of 5 windows in ", f(seconds, 1), " s");
}

// ---------------------------------------------------------------------------------------------------------------------
// 4. K11 (F5): a perfect gyro; the boundary attitude turned by 0.05 deg about the body y axis (pitch) in the predecessor's
// frame (window 4, solved), verified in window 5's prepared first pose; the gyro-bias estimate's response over windows
// 5..10 against its claimed sigma, paired with the unturned control. Seen room, noise-free accelerometer.
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("loop chain: K11, the gyro bias after a boundary attitude turned by 0.05 deg" * doctest::skip())
{
    const ChainSpec s = room_spec(surge_motion(1.0), 11);
    const OdometryConfig c = candidate_config();
    Injection base;
    base.after = 4;
    Injection turned = base;
    turned.pitch_deg = 0.05;
    const ChainRun control = run_chain(s, c, base);
    const ChainRun run = run_chain(s, c, turned);
    REQUIRE(control.ok);
    REQUIRE(run.ok);
    CHECK(control.map_rebuilds == 0);
    CHECK(run.map_rebuilds == 0);
    const double retained = read(run.records[5 - 2], s).first_pitch_deg - read(control.records[5 - 2], s).first_pitch_deg;
    MESSAGE("K11: the turn in window 5's prepared first pose ", f(retained, 5), " deg (injected 0.05 deg about body y)");
    CHECK(retained == doctest::Approx(0.05).epsilon(1e-3));

    std::ostringstream t;
    t << "K11, perfect gyro (true bias 0): window | bias y turned, control [deg/s] | claimed sigma y [deg/s] | (turned - control) / sigma | "
         "|response| / sigma, x and z | first pitch error turned [deg]\n";
    for (int w = 2; w < s.n_windows; w++)
    {
        const Reading a = read(run.records[w - 2], s), b = read(control.records[w - 2], s);
        const Eigen::Vector3d d = a.bias_deg_s - b.bias_deg_s;
        t << "  " << w << " | " << f(a.bias_deg_s.y(), 5) << ", " << f(b.bias_deg_s.y(), 5) << " | " << f(a.bias_sigma_deg_s.y(), 5) << " | "
          << f(d.y() / a.bias_sigma_deg_s.y()) << " | " << f(std::abs(d.x()) / a.bias_sigma_deg_s.x()) << ", "
          << f(std::abs(d.z()) / a.bias_sigma_deg_s.z()) << " | " << f(a.first_pitch_deg, 5) << "\n";
    }
    t << "  (hypothesis: leaves zero by 3-5 claimed sigmas; " << f(control.seconds + run.seconds, 1) << " s)";
    MESSAGE(t.str());
}

// ---------------------------------------------------------------------------------------------------------------------
// 5. F2: constant acceleration 2 m/s2 (from 0.3 m/s), handoff_tail_nodes 15 (the candidate's) and 0, the same data:
// per window the start velocity's move from its prior, the prior mean's own error and the handed-on error. Tail 0 also
// changes the map (the tail's points enter it at the solved poses, not at the re-propagated ones). Seen room, noise-free
// accelerometer.
// ---------------------------------------------------------------------------------------------------------------------
TEST_CASE("loop chain: F2, the start velocity's move under constant acceleration, tail 15 and tail 0" * doctest::skip())
{
    Motion1D motion;
    motion.v0 = 0.3;
    motion.accel = 2.0;
    const ChainSpec s = room_spec(motion, 7);
    std::ostringstream t;
    t << "F2, 2 m/s2: window | true v_start [m/s] | per tail (15 / 0): start move from prior, prior mean error, v_start error, "
         "v_end error [mm/s], first pose error [mm]\n";
    std::vector<ChainRun> runs;
    for (int tail : {15, 0})
    {
        OdometryConfig c = candidate_config();
        c.handoff_tail_nodes = tail;
        runs.push_back(run_chain(s, c, Injection{}));
        REQUIRE(runs.back().ok);
        CHECK(runs.back().map_rebuilds == 0);
    }
    for (int w = 2; w < s.n_windows; w++)
    {
        t << "  " << w << " | " << f(s.motion.v(node_tau(w, 0)), 3);
        for (const ChainRun &r : runs)
        {
            const Reading m = read(r.records[w - 2], s);
            CHECK_FALSE(m.at_rest);
            t << " | " << f(m.start_move * 1e3, 2) << ", " << f(m.prior_mean_error * 1e3, 2) << ", " << f(m.start_error * 1e3, 2) << ", "
              << f(m.end_error * 1e3, 2) << ", " << f(m.first_x_error * 1e3, 3);
        }
        t << "\n";
    }
    t << "  (hypothesis: +17-19 mm/s start move, a fitted hypothesis; " << f(runs[0].seconds + runs[1].seconds, 1) << " s)";
    MESSAGE(t.str());
}
