#include <doctest/doctest.h>

#include "angle_units.h"
#include "scan_to_map.h"
#include "window_steps.h"
#include <transformations.h>
#include "synthetic_recording.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <random>
#include <string>

// optimize_lidar_odometry on synthetic scenes with a known true trajectory: a closed box room (and an open corridor),
// both NDT maps built with the app's update_rgd (exact cell statistics), and a window of nodes 10 ms apart moving through it. The options are
// the values of settings/l2-vertical.toml, set here so that tuning the shipped file doesn't change these tests.

namespace
{
    using namespace synthetic;

    const double deg = DEG_TO_RAD;

    // the room with its origin at offset in the world (grid_offset keeps its faces off the cell boundaries)
    Scene room(const Eigen::Vector3d &offset = grid_offset) { return shifted(box_scene({-4.0, -2.5, 0.0}, {4.0, 2.5, 3.0}, false), offset); }
    Scene corridor() { return shifted(box_scene({-15.0, -2.5, 0.0}, {15.0, 2.5, 3.0}, true), grid_offset); }

    struct Maps
    {
        NDTBucketMapType indoor;
        NDTBucketMapType outdoor;
    };

    // both reference maps, as seed_map builds them: update_rgd with the viewport at the scene centre and the exact cell
    // statistics (use_exact_ndt_bucket_statistics)
    Maps build_maps(const Scene &s, const ScanToMapOptions &options)
    {
        std::vector<Point3Di> points = sample_scene(s);
        Maps maps;
        update_rgd(options.grid_indoor, maps.indoor, points, s.centre, true);
        update_rgd(options.grid_outdoor, maps.outdoor, points, s.centre, true);
        return maps;
    }

    constexpr int n_nodes = 50;
    constexpr double node_dt = 0.01;
    constexpr int points_per_node = 60;

    // the true trajectory: from (-1, 0, 1.5) along x at 1 m/s, yawing at 20 deg/s (in the scene, whose origin is at offset)
    std::vector<Eigen::Affine3d> true_poses(const Eigen::Vector3d &offset = grid_offset)
    {
        std::vector<Eigen::Affine3d> poses;
        for (int k = 0; k < n_nodes; k++)
        {
            const double t = k * node_dt;
            poses.push_back(pose(Eigen::Vector3d(-1.0 + 1.0 * t, 0.0, 1.5) + offset, 20.0 * deg * t));
        }
        return poses;
    }

    // what each node sees: points_per_node points spread over the faces by area (not the map's points, own seed),
    // with a +-5 mm jitter, given in the node's frame
    std::vector<Point3Di> observe(const Scene &s, const std::vector<Eigen::Affine3d> &truth)
    {
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        std::uniform_real_distribution<double> jitter(-0.005, 0.005);
        std::vector<double> areas;
        for (const Face &f : s.faces)
            areas.push_back(face_area(f));
        std::discrete_distribution<int> pick_face(areas.begin(), areas.end());

        std::vector<Point3Di> points;
        for (int k = 0; k < static_cast<int>(truth.size()); k++)
            for (int j = 0; j < points_per_node; j++)
            {
                const Face &f = s.faces[pick_face(rng)];
                const double u = unit(rng), v = unit(rng);
                Point3Di p{};
                p.point = truth[k].inverse() * face_point(f, u, v, jitter(rng));
                p.timestamp = k * node_dt;
                p.index_pose = k;
                p.index_point = static_cast<int>(points.size());
                points.push_back(p);
            }
        return points;
    }

    // a window whose guess (poses and motion_model_poses) is the given trajectory
    Window make_window(const std::vector<Eigen::Affine3d> &guess)
    {
        Window w;
        w.poses = guess;
        w.motion_model_poses = guess;
        for (int k = 0; k < static_cast<int>(guess.size()); k++)
            w.timestamps.push_back({k * node_dt, k * node_dt});
        return w;
    }

    // the settings of settings/l2-vertical.toml that the optimizer reads
    OdometryConfig test_config()
    {
        OdometryConfig c;
        c.in_out_params_indoor.resolution_X = c.in_out_params_indoor.resolution_Y = c.in_out_params_indoor.resolution_Z = 0.15;
        c.in_out_params_outdoor.resolution_X = c.in_out_params_outdoor.resolution_Y = c.in_out_params_outdoor.resolution_Z = 0.4;
        c.min_distance_lidar = 0.15;
        c.max_distance_lidar = 25.0;
        c.min_distance_lidar_outdoor_grid = 1.0;
        c.ablation_study_use_planarity = false;
        c.ablation_study_use_norm = false;
        c.ablation_study_use_hierarchical_rgd = true;
        c.ablation_study_use_view_point_and_normal_vectors = true;
        c.motion_model_correction = TaitBryanPose{};
        c.lidar_odometry_motion_model_x_1_sigma_m = c.lidar_odometry_motion_model_y_1_sigma_m = c.lidar_odometry_motion_model_z_1_sigma_m = 1e-4;
        c.lidar_odometry_motion_model_om_1_sigma_deg = 0.17724539;
        c.lidar_odometry_motion_model_fi_1_sigma_deg = 0.17724539;
        c.lidar_odometry_motion_model_ka_1_sigma_deg = 0.26586808;
        c.lidar_odometry_motion_model_fix_origin_x_1_sigma_m = c.lidar_odometry_motion_model_fix_origin_y_1_sigma_m =
            c.lidar_odometry_motion_model_fix_origin_z_1_sigma_m = 1e-6;
        c.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg = c.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg =
            c.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg = 1e-6;
        c.lidar_odometry_motion_model_acc_1_sigma_mps2 = 1.0;
        c.use_time_scaled_rotation_sigma = true;
        c.gyro_arw_x_deg_sqrt_s = 0.028;
        c.gyro_arw_y_deg_sqrt_s = 0.042;
        c.gyro_arw_z_deg_sqrt_s = 0.057;
        c.rotation_sigma_inflation_k = 7.0898154;
        c.rotation_sigma_floor_deg = 0.0035449077;
        c.use_gyro_bias_estimation = false; // the bias test switches it on
        c.gyro_bias_max_deg_s = 20.0;
        c.lm_factor_initial = 5.0;
        return c;
    }

    struct Options
    {
        ScanToMapOptions scan_to_map;
        MotionModelOptions motion_model;
        MotionModelRotationOptions rotation;
        MotionModelTranslationImuOptions translation_imu; // off
        VelocityPrior velocity_prior;                     // off
    };

    Options options_from(const OdometryConfig &c)
    {
        Options o;
        o.scan_to_map = get_scan_to_map_options(c);
        o.motion_model = get_motion_model_options(c);
        o.rotation = get_motion_model_rotation_options(c);
        const double bias_sigma0 = 10.0 * deg; // gyro_bias_initial_sigma_deg_s
        o.rotation.bias_prior_cov = Eigen::Matrix3d::Identity() * bias_sigma0 * bias_sigma0;
        return o;
    }

    // what one Levenberg-Marquardt iteration leaves behind
    struct IterationResult
    {
        int iteration = 0;
        double delta = 0.0;
    };

    // the iteration loop of run_lm_iterations (app/run_odometry.cpp): lm_factor starts at lm_factor_initial and grows as
    // next_lm_factor says; after_iteration (optional) sees the window after each iteration; returns the last delta
    double run_iterations(std::vector<Point3Di> &points, Window &window, Maps &maps, const Options &o, double lm_factor_initial,
                          int iterations, Logger &logger, Eigen::Matrix3d *translation_information = nullptr,
                          const std::function<void(const IterationResult &, const Window &)> &after_iteration = {},
                          ScanToMapWorkspace *workspace = nullptr)
    {
        double lm_factor = lm_factor_initial;
        double delta = 0.0;
        Eigen::Vector3d acc_offset = Eigen::Vector3d::Zero();
        Eigen::Matrix3d acc_offset_cov = Eigen::Matrix3d::Identity();
        for (int iter = 0; iter < iterations; iter++)
        {
            delta = 100000.0;
            const bool applied = optimize_lidar_odometry(points, window, maps.indoor, maps.outdoor, o.scan_to_map, o.motion_model,
                                                         o.rotation, o.translation_imu, o.velocity_prior, lm_factor, delta, acc_offset,
                                                         acc_offset_cov, logger, translation_information, nullptr, workspace);
            if (after_iteration)
                after_iteration({iter, delta}, window);
            lm_factor = next_lm_factor(lm_factor, iter, applied);
        }
        return delta;
    }

    // signed heading (yaw) of a's last node relative to b's last node [rad]
    double end_heading_error(const std::vector<Eigen::Affine3d> &a, const std::vector<Eigen::Affine3d> &b)
    {
        const Eigen::Matrix3d r = a.back().linear() * b.back().linear().transpose();
        return std::atan2(r(1, 0), r(0, 0));
    }
}

TEST_CASE("scan-to-map fixtures: scene points on the faces, maps filled, points per node in range")
{
    const OdometryConfig config = test_config();
    const Options o = options_from(config);
    const Scene s = room();

    double farthest = 0.0; // largest distance of a scene point from its nearest face
    for (const Point3Di &p : sample_scene(s))
    {
        double nearest = 1e9;
        for (const Face &f : s.faces)
            nearest = std::min(nearest, std::abs(p.point[f.axis] - f.value));
        farthest = std::max(farthest, nearest);
    }
    CHECK(farthest <= 0.01);

    const Maps maps = build_maps(s, o.scan_to_map);
    CHECK(maps.indoor.size() > 1000);
    CHECK(maps.outdoor.size() > 100);

    const std::vector<Eigen::Affine3d> truth = true_poses();
    const std::vector<Point3Di> points = observe(s, truth);
    std::vector<int> in_range(n_nodes, 0);
    for (const Point3Di &p : points)
        if (p.point.norm() >= config.min_distance_lidar && p.point.norm() <= config.max_distance_lidar)
            in_range[p.index_pose]++;
    CHECK(*std::min_element(in_range.begin(), in_range.end()) >= 40);
    CHECK(*std::max_element(in_range.begin(), in_range.end()) <= 80);
}

namespace
{
    // the drifting guess of a gyro bias: true translations, yaw off by bias * t (node 0 is exact, as the fix origin holds it)
    std::vector<Eigen::Affine3d> drifting_guess(const std::vector<Eigen::Affine3d> &truth, double bias_rad_s)
    {
        std::vector<Eigen::Affine3d> guess = truth;
        for (int k = 0; k < static_cast<int>(guess.size()); k++)
            guess[k].linear() = Eigen::AngleAxisd(bias_rad_s * k * node_dt, Eigen::Vector3d::UnitZ()).toRotationMatrix() * truth[k].linear();
        return guess;
    }
}

TEST_CASE("scan-to-map: a perfect guess stays at the truth")
{
    const Options o = options_from(test_config());
    const Scene s = room();
    Maps maps = build_maps(s, o.scan_to_map);
    const std::vector<Eigen::Affine3d> truth = true_poses();
    std::vector<Point3Di> points = observe(s, truth);
    Window w = make_window(truth);
    NullLogger logger;

    // Measured 2026-10-04 with exact cell statistics: 0.37 mm, 0.046 deg, delta 9e-8 (with the previous statistics, whose
    // cell mean is the cell's first point: 1.5 mm, 0.046 deg, delta 1.5e-4). The rotation error scales with the point noise
    // (0.022 deg at +-1 mm). The limits record this with margin. Measured 2026-10-14 (scene at grid_offset, cell index with
    // floor, C10): 0.34 mm, 0.038 deg, delta 8e-8.
    const double delta = run_iterations(points, w, maps, o, 5.0, 30, logger);
    const PoseError e = max_error(w.poses, truth);
    MESSAGE("perfect guess after 30 iterations: position ", e.position * 1000.0, " mm, rotation ", e.rotation / deg, " deg, delta ", delta);
    CHECK(e.position < 0.001);
    CHECK(e.rotation < 0.1 * deg);
    CHECK(delta < 1e-6);
}

TEST_CASE("scan-to-map: where a plane lies in its cell")
{
    // The room placed so that some of its faces lie exactly on cell boundaries. A face on a boundary splits its points
    // (+-5 mm) between two cells, each holding half a slab whose mean lies off the plane, which costs NDT accuracy. Real
    // surfaces lie anywhere in a cell, so the other tests keep every face off the boundaries (grid_offset); this one
    // records the cost per case. Measured 2026-10-14 (largest position error of a perfect guess after 30 iterations):
    // 0.34 mm with no face on a boundary, 0.61 mm with floor and ceiling on 0.15 m boundaries, 1.52 mm with the x walls on
    // 0.4 m boundaries (the coarse grid costs the most, finding C18), 2.26 mm with both (the scene at the world origin, as
    // these tests had it before 2026-10-14).
    struct Case
    {
        std::string name;
        Eigen::Vector3d offset;
        double limit_m;
    };
    const double g = grid_offset.x();
    const Case cases[] = {
        {"at grid_offset (no face on a boundary)", grid_offset, 0.001},
        {"floor and ceiling on 0.15 m boundaries", {g, g, 0.0}, 0.001},
        {"x walls on 0.4 m boundaries", {0.0, g, g}, 0.002},
        {"at the world origin (both)", Eigen::Vector3d::Zero(), 0.003},
    };
    const Options o = options_from(test_config());
    for (const Case &c : cases)
    {
        const Scene s = room(c.offset);
        Maps maps = build_maps(s, o.scan_to_map);
        const std::vector<Eigen::Affine3d> truth = true_poses(c.offset);
        std::vector<Point3Di> points = observe(s, truth);
        Window w = make_window(truth);
        NullLogger logger;
        run_iterations(points, w, maps, o, 5.0, 30, logger);
        const PoseError e = max_error(w.poses, truth);
        MESSAGE("perfect guess, room ", c.name, ": position ", e.position * 1000.0, " mm, rotation ", e.rotation / deg, " deg");
        CHECK(e.position < c.limit_m);
        CHECK(e.rotation < 0.1 * deg);
    }
}

TEST_CASE("scan-to-map: a node 1 m or more from its guess is a big jump, counted before anything is applied")
{
    // the step decision of optimize_lidar_odometry: stepped_poses adds the pose part of a step and counts the nodes that
    // would end 1 m or more from their guess; with one such node the optimizer applies nothing (before, it held that
    // node and applied the step to the others and to the extra unknowns)
    const std::vector<Eigen::Affine3d> truth = true_poses();
    Window w = make_window(truth);
    NullLogger logger;
    std::vector<double> step(w.poses.size() * 6 + 3, 0.0); // 3 extra unknowns after the poses, ignored here
    step[0] = 0.2;                                        // node 0: 0.2 m in x
    const SteppedPoses small = stepped_poses(w, step, 1.0, logger);
    CHECK(small.big_jump_nodes == 0);
    REQUIRE(small.poses.size() == w.poses.size());
    CHECK(std::abs(small.poses[0].px - (pose_tait_bryan_from_affine_matrix(w.poses[0]).px + 0.2)) < 1e-12);
    step[6 * (w.poses.size() - 1) + 1] = 1.2; // last node: 1.2 m in y
    const SteppedPoses big = stepped_poses(w, step, 1.0, logger);
    CHECK(big.big_jump_nodes == 1);
}

TEST_CASE("scan-to-map: the damping grows every 10 iterations and at once after a step that was not applied")
{
    // a rejected step leaves the window unchanged; with the same damping the next iteration would get the same step
    CHECK(next_lm_factor(5.0, 0, true) == 5.0);
    CHECK(next_lm_factor(5.0, 3, true) == 5.0);
    CHECK(next_lm_factor(5.0, 10, true) == 50.0);
    CHECK(next_lm_factor(5.0, 3, false) == 50.0);
    CHECK(next_lm_factor(5.0, 10, false) == 500.0);
}

TEST_CASE("scan-to-map: a yaw drift is corrected")
{
    const Options o = options_from(test_config());
    const Scene s = room();
    Maps maps = build_maps(s, o.scan_to_map);
    const std::vector<Eigen::Affine3d> truth = true_poses();
    std::vector<Point3Di> points = observe(s, truth);
    const double bias = 0.5 * deg;
    const std::vector<Eigen::Affine3d> guess = drifting_guess(truth, bias);
    Window w = make_window(guess);
    NullLogger logger;

    run_iterations(points, w, maps, o, 5.0, 30, logger);
    const PoseError before = max_error(guess, truth);
    const PoseError after = max_error(w.poses, truth);
    const double heading_before = end_heading_error(guess, truth);
    const double heading_after = end_heading_error(w.poses, truth);
    MESSAGE("yaw drift: end heading error ", heading_before / deg, " -> ", heading_after / deg, " deg; largest position error ",
            before.position * 1000.0, " -> ", after.position * 1000.0, " mm; largest rotation error ", before.rotation / deg, " -> ",
            after.rotation / deg, " deg");
    // Measured 2026-10-04: 0.245 -> 0.014 deg at the end; the positions (exact in the guess) pick up 0.69 mm (1.8 mm with
    // the previous cell statistics). Measured 2026-10-14 (scene at grid_offset, C10): 0.245 -> 0.024 deg, 0.57 mm
    CHECK(std::abs(heading_after) <= 0.1 * std::abs(heading_before));
    CHECK(after.position < 0.0015);
}

TEST_CASE("scan-to-map: the gyro bias is estimated")
{
    OdometryConfig c = test_config();
    c.use_gyro_bias_estimation = true;
    const Options o = options_from(c);
    const Scene s = room();
    Maps maps = build_maps(s, o.scan_to_map);
    const std::vector<Eigen::Affine3d> truth = true_poses();
    std::vector<Point3Di> points = observe(s, truth);
    const double bias = 0.5 * deg;
    const std::vector<Eigen::Affine3d> guess = drifting_guess(truth, bias);
    Window w = make_window(guess);
    NullLogger logger;

    run_iterations(points, w, maps, o, 5.0, 30, logger);
    const Eigen::Vector3d b = w.gyro_bias.mean / deg;
    const PoseError after = max_error(w.poses, truth);
    MESSAGE("estimated bias ", b.x(), " ", b.y(), " ", b.z(), " deg/s (injected 0 0 0.5); end heading error ",
            end_heading_error(w.poses, truth) / deg, " deg; largest position error ", after.position * 1000.0, " mm");
    // Measured 2026-10-04: 0.038 -0.042 0.473 deg/s (0.045 -0.031 0.469 with the previous cell statistics); 2026-10-14
    // (scene at grid_offset, C10): 0.016 -0.022 0.452 deg/s
    CHECK(std::abs(b.z() - 0.5) < 0.075); // within 15 % (the former Approx(0.5).epsilon(0.1) allowed 0.15 through its scale of 1)
    CHECK(std::abs(b.x()) < 0.05);
    CHECK(std::abs(b.y()) < 0.05);
}

TEST_CASE("scan-to-map: a gyro bias carried from window to window is removed once")
{
    // The window loop's bias chain (optimize_windows): apply_gyro_bias_prior removes the carried bias b0 from the guess and
    // from motion_model_poses, the optimizer estimates the window's bias b, and b (with its covariance) is carried on. Each
    // window's guess is the true trajectory with the gyro's yaw drift bias * t from its first node. With b0 = 0 (a single
    // window) the measurement is right either way; from the second window on it must subtract only b - b0, since b0 is
    // already out of the increments. Subtracting the full b made the chain settle near half the bias (finding C6).
    OdometryConfig c = test_config();
    c.use_gyro_bias_estimation = true;
    c.gyro_bias_rw_deg_s_sqrt_s = 0.005; // settings/l2-vertical.toml
    Options o = options_from(c);
    const Scene s = room();
    Maps maps = build_maps(s, o.scan_to_map);
    NullLogger logger;

    const double bias = 2.0 * deg;
    constexpr int n_windows = 6;
    GyroBiasState carried;
    carried.covariance = o.rotation.bias_prior_cov;
    std::vector<double> estimates;
    for (int j = 0; j < n_windows; j++)
    {
        // window j continues the trajectory of true_poses: from x = -1 + 0.5 j m at 1 m/s, yawing at 20 deg/s
        std::vector<Eigen::Affine3d> truth;
        for (int k = 0; k < n_nodes; k++)
        {
            const double t = (j * n_nodes + k) * node_dt;
            truth.push_back(pose(Eigen::Vector3d(-1.0 + 1.0 * t, 0.0, 1.5) + grid_offset, 20.0 * deg * t));
        }
        std::vector<Point3Di> points = observe(s, truth);
        Window w = make_window(drifting_guess(truth, bias));
        const GyroBiasState prior = apply_gyro_bias_prior(w, carried, c.gyro_bias_rw_deg_s_sqrt_s * deg);
        o.rotation.bias_prior_mean = prior.mean;
        o.rotation.bias_prior_cov = prior.covariance;
        run_iterations(points, w, maps, o, 5.0, 30, logger);
        carried = w.gyro_bias;
        estimates.push_back(w.gyro_bias.mean.z() / deg);
    }
    const Eigen::Vector3d b = carried.mean / deg;
    MESSAGE("chained bias estimates (z, deg/s, injected 2.0): ", estimates[0], " ", estimates[1], " ", estimates[2], " ",
            estimates[3], " ", estimates[4], " ", estimates[5], "; last window x ", b.x(), " y ", b.y());
    CHECK(std::abs(b.z() - 2.0) < 0.1); // within 5 %
    CHECK(std::abs(b.x()) < 0.05);
    CHECK(std::abs(b.y()) < 0.05);
}

// How many Levenberg-Marquardt iterations the optimizer needs (design decision 8): the two drift scenarios run the 100
// iterations of a real run (nr_iter), and the test reports when the poses stop changing (N_final) and when the update first
// falls below the fast-mode convergence_delta_threshold of 1e-2 (N_fast), with the error against the truth at both.
// Only N_final < 100 is asserted; the numbers are a measurement (see docs/PERFORMANCE.md).
TEST_CASE("scan-to-map: iterations needed")
{
    for (const bool estimate_bias : {false, true})
    {
        OdometryConfig c = test_config();
        c.use_gyro_bias_estimation = estimate_bias;
        const Options o = options_from(c);
        const Scene s = room();
        Maps maps = build_maps(s, o.scan_to_map);
        const std::vector<Eigen::Affine3d> truth = true_poses();
        std::vector<Point3Di> points = observe(s, truth);
        Window w = make_window(drifting_guess(truth, 0.5 * deg));
        NullLogger logger;

        constexpr int iterations = 100;
        std::vector<std::vector<Eigen::Affine3d>> poses_after;
        std::vector<double> deltas;
        const auto start = std::chrono::steady_clock::now();
        run_iterations(points, w, maps, o, 5.0, iterations, logger, nullptr,
                       [&](const IterationResult &r, const Window &window)
                       {
                           poses_after.push_back(window.poses);
                           deltas.push_back(r.delta);
                       });
        const double ms_per_iteration = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() / iterations;

        // N_final: the number of iterations after which the poses stay within 0.1 mm and 0.001 deg of the result after 100
        const std::vector<Eigen::Affine3d> &final_poses = poses_after.back();
        int n_final = iterations;
        for (int i = iterations - 1; i >= 0; i--)
        {
            const PoseError e = max_error(poses_after[i], final_poses);
            if (e.position > 1e-4 || e.rotation > 0.001 * deg)
                break;
            n_final = i + 1;
        }
        // N_fast: the number of iterations run when delta first falls below 1e-2 (run_lm_iterations stops there)
        int n_fast = 0;
        for (int i = 0; i < iterations; i++)
            if (deltas[i] < 1e-2)
            {
                n_fast = i + 1;
                break;
            }

        const PoseError at_final = max_error(final_poses, truth);
        const PoseError at_fast = n_fast > 0 ? max_error(poses_after[n_fast - 1], truth) : PoseError{};
        MESSAGE(std::string(estimate_bias ? "with" : "without"), " bias estimation: N_final ", n_final, ", N_fast ", n_fast,
                "; error vs truth at N_fast ", at_fast.position * 1000.0, " mm / ", at_fast.rotation / deg, " deg, after ", iterations,
                " ", at_final.position * 1000.0, " mm / ", at_final.rotation / deg, " deg; ", ms_per_iteration, " ms per iteration");
        CHECK(n_final < iterations);
    }
}

namespace
{
    // the translation information the optimizer reports for a window on the true trajectory (one iteration): the sum of
    // n * n^T over the matched surface normals, which optimize_windows uses to detect degenerate windows
    Eigen::Matrix3d translation_information(const Scene &s, const Options &o)
    {
        Maps maps = build_maps(s, o.scan_to_map);
        const std::vector<Eigen::Affine3d> truth = true_poses();
        std::vector<Point3Di> points = observe(s, truth);
        Window w = make_window(truth);
        NullLogger logger;
        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        run_iterations(points, w, maps, o, 5.0, 1, logger, &info);
        return info;
    }
}

TEST_CASE("scan-to-map: the translation information is strong in every direction in a closed room")
{
    const Eigen::Vector3d eigenvalues =
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(translation_information(room(), options_from(test_config()))).eigenvalues();
    MESSAGE("room: information eigenvalues ", eigenvalues.x(), " ", eigenvalues.y(), " ", eigenvalues.z());
    // Measured 2026-10-04: 477 807 1226 (677 740 1419 with the previous cell statistics)
    CHECK(eigenvalues.minCoeff() > 0.1 * eigenvalues.maxCoeff());
}

// The previous cell statistics fail this (found 2026-10-04): they fix a cell's normal once, from its first 3 points and the
// initial 3 cm isotropic covariance, so on this corridor 60 % of the indoor cells got a normal more than 30 deg off the
// faces, and the information was 925 along x against 155 across. With the exact statistics and no normal for cells whose
// points lie on one line, it is 1.7 along against 987 across (measured 2026-10-04).
TEST_CASE("scan-to-map: the translation information shows a degenerate corridor")
{
    const Eigen::Matrix3d info = translation_information(corridor(), options_from(test_config()));
    const double along = info(0, 0);
    const double across = std::min(info(1, 1), info(2, 2));
    MESSAGE("corridor: information along x ", along, ", across (smaller of y and z) ", across);
    CHECK(along <= 0.01 * across);
}

namespace
{
    // one window optimization with the multithreaded or the sequential node loop: the room or corridor, the drifting guess of a 0.5 deg/s gyro bias
    // with bias estimation on, the points in scene order (grouped by node) or shuffled (as decimate leaves them, ordered by
    // cell, not by node)
    struct MultithreadRun
    {
        std::vector<Eigen::Affine3d> poses;
        Eigen::Vector3d bias;
        Eigen::Matrix3d translation_information;
        double delta = 0.0;
    };

    MultithreadRun run_window(const Scene &s, bool multithread, bool shuffled, int iterations, ScanToMapWorkspace *workspace = nullptr,
                              bool cached_inverse = true)
    {
        OdometryConfig c = test_config();
        c.use_gyro_bias_estimation = true;
        Options o = options_from(c);
        o.scan_to_map.multithread = multithread;
        Maps maps = build_maps(s, o.scan_to_map);
        if (!cached_inverse) // the loops then invert every cell on the spot, as before the cache
        {
            for (auto &cell : maps.indoor)
                cell.second.inverse_computed = false;
            for (auto &cell : maps.outdoor)
                cell.second.inverse_computed = false;
        }
        const std::vector<Eigen::Affine3d> truth = true_poses();
        std::vector<Point3Di> points = observe(s, truth);
        if (shuffled)
            std::shuffle(points.begin(), points.end(), std::mt19937(11));
        Window w = make_window(drifting_guess(truth, 0.5 * deg));
        NullLogger logger;
        MultithreadRun r;
        r.translation_information = Eigen::Matrix3d::Zero();
        r.delta = run_iterations(points, w, maps, o, 5.0, iterations, logger, &r.translation_information, {}, workspace);
        r.poses = w.poses;
        r.bias = w.gyro_bias.mean;
        return r;
    }

    // bit for bit: every element of every pose, the bias, the translation information and the last delta
    bool identical(const MultithreadRun &a, const MultithreadRun &b)
    {
        if (a.poses.size() != b.poses.size() || a.delta != b.delta || a.bias != b.bias || a.translation_information != b.translation_information)
            return false;
        for (size_t k = 0; k < a.poses.size(); k++)
            if (a.poses[k].matrix() != b.poses[k].matrix())
                return false;
        return true;
    }
}

// multithreading must not change results (change deterministic-multithread): each node's point terms are summed in point
// order whether the nodes run in parallel or one after the other
TEST_CASE("scan-to-map: multithreaded equals single-threaded, bit for bit")
{
    for (const bool use_corridor : {false, true})
        for (const bool shuffled : {false, true})
            for (const int iterations : {1, 30})
            {
                CAPTURE(use_corridor);
                CAPTURE(shuffled);
                CAPTURE(iterations);
                const Scene s = use_corridor ? corridor() : room();
                const MultithreadRun single = run_window(s, false, shuffled, iterations);
                const MultithreadRun multi = run_window(s, true, shuffled, iterations);
                CHECK(identical(single, multi));
            }
}

TEST_CASE("scan-to-map: two multithreaded runs are identical, bit for bit")
{
    for (const bool use_corridor : {false, true})
    {
        CAPTURE(use_corridor);
        const Scene s = use_corridor ? corridor() : room();
        const MultithreadRun first = run_window(s, true, true, 30);
        const MultithreadRun second = run_window(s, true, true, 30);
        CHECK(identical(first, second));
    }
}

// Keeping the Cholesky analyses between iterations (change fixed-pattern-precise-fp) must not change a result, and must
// save analyses: the pattern of a window's normal matrix is the same from one iteration to the next
TEST_CASE("scan-to-map: reusing the Cholesky analyses changes nothing, bit for bit")
{
    for (const bool use_corridor : {false, true})
    {
        CAPTURE(use_corridor);
        const Scene s = use_corridor ? corridor() : room();
        const MultithreadRun without = run_window(s, true, false, 30);
        ScanToMapWorkspace workspace;
        const MultithreadRun with = run_window(s, true, false, 30, &workspace);
        CHECK(identical(without, with));
        CHECK(workspace.damped.calls() == 30);
        CHECK(workspace.damped.analyses() <= 3);
        CHECK(workspace.covariance.calls() == 30);
        CHECK(workspace.covariance.analyses() <= 3);
    }
}

// the inverse covariance stored in the cells (change ndt-lookup-cache) is the one the loops would compute on the spot
TEST_CASE("scan-to-map: stored cell inverses equal inverting on the spot, bit for bit")
{
    for (const bool use_corridor : {false, true})
        for (const int iterations : {1, 30})
        {
            CAPTURE(use_corridor);
            CAPTURE(iterations);
            const Scene s = use_corridor ? corridor() : room();
            const MultithreadRun cached = run_window(s, true, false, iterations, nullptr, true);
            const MultithreadRun on_the_spot = run_window(s, true, false, iterations, nullptr, false);
            CHECK(identical(cached, on_the_spot));
        }
}

namespace
{
    // one window of the corridor or room with the degeneracy-aware NDT terms: the guess (also the motion model) is the truth
    // moved by a shift growing linearly from 0 at the first node (held by the fix-origin prior) to `shift` at the last; 30 iterations
    std::vector<Eigen::Affine3d> run_projected(const Scene &s, const Eigen::Vector3d &shift, double ratio, double ratio_full, bool multithread)
    {
        OdometryConfig c = test_config();
        c.use_gyro_bias_estimation = false;
        Options o = options_from(c);
        o.scan_to_map.multithread = multithread;
        o.scan_to_map.degenerate_projection_ratio = ratio;
        o.scan_to_map.degenerate_projection_ratio_full = ratio_full;
        Maps maps = build_maps(s, o.scan_to_map);
        const std::vector<Eigen::Affine3d> truth = true_poses();
        std::vector<Point3Di> points = observe(s, truth);
        std::vector<Eigen::Affine3d> guess = truth;
        for (size_t k = 0; k < guess.size(); k++)
            guess[k].translation() += shift * (double(k) / double(guess.size() - 1));
        Window w = make_window(guess);
        NullLogger logger;
        run_iterations(points, w, maps, o, 5.0, 30, logger);
        return w.poses;
    }
}

// change odometry-stall-fix: along a direction the geometry doesn't constrain, the NDT terms carry no translation information,
// so the window stays where the motion model puts it, while the constrained directions are still corrected
TEST_CASE("scan-to-map: degeneracy-aware NDT terms leave the blind direction to the motion model")
{
    const Eigen::Vector3d shift(0.05, 0.03, 0.0); // at the last node 5 cm along the corridor (x), 3 cm across (y)
    const std::vector<Eigen::Affine3d> truth = true_poses();
    const std::vector<Eigen::Affine3d> projected = run_projected(corridor(), shift, 0.1, 0.2, true);
    const std::vector<Eigen::Affine3d> plain = run_projected(corridor(), shift, 0.0, 0.0, true);
    // largest change along the axis (against the guess) and largest error across it (against the truth)
    const auto measure = [&](const std::vector<Eigen::Affine3d> &poses, double &along, double &across)
    {
        along = across = 0.0;
        for (size_t k = 0; k < truth.size(); k++)
        {
            const Eigen::Vector3d d = poses[k].translation() - truth[k].translation();
            along = std::max(along, std::abs(d.x() - shift.x() * (double(k) / double(truth.size() - 1))));
            across = std::max(across, std::abs(d.y()));
        }
    };
    double along = 0.0, across = 0.0, plain_along = 0.0, plain_across = 0.0;
    measure(projected, along, across);
    measure(plain, plain_along, plain_across);
    MESSAGE("corridor: change along the axis ", along * 1000.0, " mm with projection, ", plain_along * 1000.0, " mm without; error across ",
            across * 1000.0, " mm with, ", plain_across * 1000.0, " mm without (30 mm before)");
    CHECK(along < 0.002);                   // the shift along the axis is kept (only the motion model acts there)
    CHECK(across < 0.25 * 0.03);            // across the axis the LiDAR still corrects most of the 30 mm (the rest is the
    CHECK(across <= plain_across + 0.001);  // motion model's pull towards the guess, the same as without the projection)

    // multithreaded equals single-threaded
    const std::vector<Eigen::Affine3d> single = run_projected(corridor(), shift, 0.1, 0.2, false);
    for (size_t k = 0; k < truth.size(); k++)
        CHECK(projected[k].matrix() == single[k].matrix());
}

TEST_CASE("scan-to-map: degeneracy-aware NDT terms change nothing in a closed room")
{
    const Eigen::Vector3d shift(0.02, -0.02, 0.01);
    const std::vector<Eigen::Affine3d> off = run_projected(room(), shift, 0.0, 0.0, true);
    const std::vector<Eigen::Affine3d> on = run_projected(room(), shift, 0.1, 0.2, true); // the room's ratio is about 0.48
    for (size_t k = 0; k < off.size(); k++)
        CHECK(off[k].matrix() == on[k].matrix());
}

// change imu-always-on: the accelerometer in the window with the node rotations, noise values as in l2-vertical.toml, and the
// start and end velocity as unknowns
namespace
{
    constexpr double g_mps2 = 9.80665;

    struct AlwaysOnCase
    {
        std::vector<Eigen::Affine3d> truth;
        std::vector<Eigen::Affine3d> guess;
        std::vector<Eigen::Vector3d> world_acceleration; // true acceleration of each node [m/s^2]
        std::vector<double> times;                       // node timestamps (node_dt apart unless changed)
        Eigen::Vector3d velocity_prior = Eigen::Vector3d::Zero();
        double velocity_prior_sigma = 0.02;
        bool at_rest = false;
        bool imu = true;            // false: the term off (the velocity-continuity prior instead)
        bool points = true;         // false: no LiDAR points (only the motion model and the accelerometer)
        double translation_sigma_m = 1e-3;
        double offset_sigma = 0.01;      // acceleration offset prior: a carried bias, not a fresh one
        double rotation_sigma_deg = 0.0; // > 0: constant rotation sigmas instead of the gyro's (time-scaled ARW)
        bool project_degenerate = true;         // the shipped degeneracy-aware NDT terms (0.1 / 0.2)
        bool normal_only_covariance = false;    // carried_covariance_normal_information_only
    };

    // runs 30 iterations; the accelerometer reads the specific force R^T (a + g) in g
    Window run_always_on(const Scene &s, const AlwaysOnCase &k)
    {
        OdometryConfig c = test_config();
        c.lidar_odometry_motion_model_x_1_sigma_m = c.lidar_odometry_motion_model_y_1_sigma_m = c.lidar_odometry_motion_model_z_1_sigma_m =
            k.translation_sigma_m;
        if (k.rotation_sigma_deg > 0.0)
        {
            c.use_time_scaled_rotation_sigma = false;
            c.lidar_odometry_motion_model_om_1_sigma_deg = c.lidar_odometry_motion_model_fi_1_sigma_deg =
                c.lidar_odometry_motion_model_ka_1_sigma_deg = k.rotation_sigma_deg;
        }
        Options o = options_from(c);
        // the shipped degeneracy-aware NDT terms: without them, planar cells keep an in-plane pull along the corridor
        o.scan_to_map.degenerate_projection_ratio = k.project_degenerate ? 0.1 : 0.0;
        o.scan_to_map.degenerate_projection_ratio_full = k.project_degenerate ? 0.2 : 0.0;
        o.scan_to_map.carried_covariance_normal_information_only = k.normal_only_covariance;
        Maps maps = build_maps(s, o.scan_to_map);
        std::vector<Point3Di> points = k.points ? observe(s, k.truth) : std::vector<Point3Di>{};
        Window w = make_window(k.guess);
        if (!k.times.empty())
            for (size_t i = 0; i < w.timestamps.size(); i++)
            {
                w.timestamps[i] = {k.times[i], k.times[i]};
                for (Point3Di &p : points)
                    if (p.index_pose == static_cast<int>(i))
                        p.timestamp = k.times[i];
            }
        std::vector<Eigen::Vector3d> acc;
        for (size_t i = 0; i < k.truth.size(); i++)
            acc.push_back(k.truth[i].linear().transpose() * (k.world_acceleration[i] + Eigen::Vector3d(0.0, 0.0, g_mps2)) / g_mps2);
        if (k.imu)
        {
            o.translation_imu.enabled = true;
            o.translation_imu.acc = &acc;
            o.translation_imu.gravity = Eigen::Vector3d(0.0, 0.0, 1.0);
            o.translation_imu.to_mps2 = g_mps2;
            o.translation_imu.span_s = 0.008; // one term per node: anchors 10 ms apart, well inside the 2 * span_s gap limit
            o.translation_imu.offset_prior_cov = Eigen::Matrix3d::Identity() * k.offset_sigma * k.offset_sigma;
            o.translation_imu.noise_density_mps2_sqrt_hz = 0.0013;
            o.translation_imu.floor_sigma_mps2 = 0.1;
            o.translation_imu.velocity_prior_mean = k.velocity_prior;
            o.translation_imu.velocity_prior_cov = Eigen::Matrix3d::Identity() * k.velocity_prior_sigma * k.velocity_prior_sigma;
            o.translation_imu.at_rest = k.at_rest;
            o.translation_imu.zero_velocity_sigma_mps = 0.01;
            w.velocity.start = w.velocity.end = k.velocity_prior;
        }
        else
            o.velocity_prior = VelocityPrior{true, k.velocity_prior};
        NullLogger logger;
        run_iterations(points, w, maps, o, 5.0, 30, logger);
        return w;
    }

    // along x at speed v0 with acceleration a (both along x), no rotation, from (-1, 0, 1.5)
    std::vector<Eigen::Affine3d> straight(double v0, double a)
    {
        std::vector<Eigen::Affine3d> poses;
        for (int i = 0; i < n_nodes; i++)
        {
            const double t = i * node_dt;
            poses.push_back(pose(Eigen::Vector3d(-1.0 + v0 * t + 0.5 * a * t * t, 0.0, 1.5) + grid_offset, 0.0));
        }
        return poses;
    }

    // walking along x: speed v0 plus a surge of amplitude a_mps2 at frequency f_hz (acceleration a sin(2 pi f t))
    constexpr double pi = 3.14159265358979323846;
    constexpr double walk_v0 = 1.0, walk_a = 2.0, walk_f = 2.0;
    double walk_x(double t)
    {
        const double w = 2.0 * pi * walk_f;
        return -1.0 + walk_v0 * t + walk_a / w * (t - std::sin(w * t) / w);
    }
}

TEST_CASE("scan-to-map, imu always on: a blind corridor follows the accelerometer along its axis")
{
    // the truth surges like a walk (2 m/s^2 at 2 Hz) along the corridor; the guess keeps the start speed (8 cm short at the end).
    // A constant acceleration over the whole window would instead look like a bias, which the offset e takes in part
    AlwaysOnCase k;
    for (int i = 0; i < n_nodes; i++)
    {
        const double t = i * node_dt;
        k.truth.push_back(pose(Eigen::Vector3d(walk_x(t), 0.0, 1.5) + grid_offset, 0.0));
        k.world_acceleration.push_back(Eigen::Vector3d(walk_a * std::sin(2.0 * pi * walk_f * t), 0.0, 0.0));
    }
    k.guess = straight(walk_v0, 0.0);
    k.velocity_prior = Eigen::Vector3d(walk_v0, 0.0, 0.0);
    // The translation motion model ties each step to the guess's step: a velocity prior on every node (sigma_p / dt) around the
    // guess, which together hold the window's mean velocity at the guess's. With this constant-velocity guess that is the start
    // velocity, while the surge raises the true mean by a / w = 0.16 m/s; at 1e-3 m the motion model and the start prior split it,
    // and the window follows the surge's shape 0.106 m/s too slow (measured 2026-10-09: 54 of 80 mm left). The pipeline therefore
    // predicts the guess from the accelerometer (integrate_window_translations); here the motion model is loose, to test the term
    k.translation_sigma_m = 0.03;
    const Window on = run_always_on(corridor(), k);
    k.imu = false;
    const Window off = run_always_on(corridor(), k);
    const auto end_error = [&](const Window &w) { return (w.poses.back().translation() - k.truth.back().translation()).x(); };
    const double guess_error = (k.guess.back().translation() - k.truth.back().translation()).x();
    const double t_end = (n_nodes - 1) * node_dt, w = 2.0 * pi * walk_f;
    const double v_end = walk_v0 + walk_a / w * (1.0 - std::cos(w * t_end));
    MESSAGE("blind corridor, end error along the axis: guess ", guess_error * 1000.0, " mm, accelerometer off ", end_error(off) * 1000.0,
            " mm, always on ", end_error(on) * 1000.0, " mm; end velocity ", on.velocity.end.x(), " m/s (true ", v_end, ")");
    CHECK(std::abs(end_error(on)) < 0.2 * std::abs(guess_error));
    CHECK(std::abs(end_error(off)) > 0.5 * std::abs(guess_error)); // without the accelerometer nothing sees the axis
    CHECK(on.velocity.end.x() == doctest::Approx(v_end).epsilon(0.02));
}

TEST_CASE("scan-to-map, imu always on: a blind window at rest stays put")
{
    // at rest in the corridor, guessed at rest, but the previous window's end velocity says 0.5 m/s along the axis
    AlwaysOnCase k;
    k.truth = straight(0.0, 0.0);
    k.guess = k.truth;
    k.world_acceleration.assign(n_nodes, Eigen::Vector3d::Zero());
    k.velocity_prior = Eigen::Vector3d(0.5, 0.0, 0.0);
    k.velocity_prior_sigma = 0.1;
    k.at_rest = true;
    const Window rest = run_always_on(corridor(), k);
    k.at_rest = false;
    const Window moving = run_always_on(corridor(), k);
    const PoseError e = max_error(rest.poses, k.truth), e_moving = max_error(moving.poses, k.truth);
    MESSAGE("blind window at rest: largest position error ", e.position * 1000.0, " mm with the zero-velocity measurement, ",
            e_moving.position * 1000.0, " mm without; velocities ", rest.velocity.start.x(), " / ", rest.velocity.end.x(), " m/s");
    CHECK(e.position < 0.003);
    CHECK(std::abs(rest.velocity.end.x()) < 0.05);
}

TEST_CASE("scan-to-map, imu always on: a stationary window in the room doesn't drift")
{
    AlwaysOnCase k;
    k.truth = straight(0.0, 0.0);
    k.guess = k.truth;
    k.world_acceleration.assign(n_nodes, Eigen::Vector3d::Zero());
    const Window w = run_always_on(room(), k);
    const PoseError e = max_error(w.poses, k.truth);
    MESSAGE("stationary room: largest position error ", e.position * 1000.0, " mm, rotation ", e.rotation / deg, " deg");
    CHECK(e.position < 0.001);
    CHECK(w.velocity.end.norm() < 0.01);
}

TEST_CASE("scan-to-map, imu always on: a tilt drift is corrected in the attitude, not the positions")
{
    // no LiDAR points: only the motion model and the accelerometer. The guess's roll drifts from 0 to 1 deg over the window
    // (node 0 is held by the fix-origin prior); the accelerometer still reads gravity straight down in the true frame. Within one
    // window the gyro (time-scaled ARW) is far stronger than the accelerometer's tilt information, so the rotation sigma is set
    // loose here: the test checks that the term's rotation Jacobian pulls the right way, by the right amount
    AlwaysOnCase k;
    k.truth = straight(0.0, 0.0);
    k.guess = k.truth;
    for (int i = 0; i < n_nodes; i++)
        k.guess[i].linear() = Eigen::AngleAxisd(1.0 * deg * i / (n_nodes - 1), Eigen::Vector3d::UnitX()).toRotationMatrix();
    k.world_acceleration.assign(n_nodes, Eigen::Vector3d::Zero());
    k.points = false;
    k.translation_sigma_m = 1e-4;
    k.rotation_sigma_deg = 1.7724539;
    const Window w = run_always_on(room(), k);
    const PoseError before = max_error(k.guess, k.truth), after = max_error(w.poses, k.truth);
    MESSAGE("tilt drift: largest rotation error ", before.rotation / deg, " -> ", after.rotation / deg, " deg, largest position error ",
            after.position * 1000.0, " mm");
    CHECK(after.rotation < 0.5 * before.rotation);
    CHECK(after.position < 0.005);
}

TEST_CASE("scan-to-map, imu always on: a gap in the IMU samples drops the stretched terms")
{
    // a 60 ms gap between nodes 24 and 25 (the terms across it span more than 2 * span_s); the window still converges
    AlwaysOnCase k;
    k.truth = straight(1.0, 0.0);
    for (int i = 0; i < n_nodes; i++)
        k.times.push_back(i * node_dt + (i >= 25 ? 0.05 : 0.0));
    for (int i = 0; i < n_nodes; i++)
        k.truth[i].translation().x() = grid_offset.x() - 1.0 + 1.0 * k.times[i];
    k.guess = k.truth;
    for (int i = 0; i < n_nodes; i++)
        k.guess[i].translation() += Eigen::Vector3d(0.01, -0.01, 0.005) * (double(i) / (n_nodes - 1));
    k.world_acceleration.assign(n_nodes, Eigen::Vector3d::Zero());
    k.velocity_prior = Eigen::Vector3d(1.0, 0.0, 0.0);
    const Window w = run_always_on(room(), k);
    const PoseError e = max_error(w.poses, k.truth);
    MESSAGE("IMU gap: largest position error ", e.position * 1000.0, " mm; end velocity ", w.velocity.end.x(), " m/s");
    CHECK(e.position < 0.002);
    CHECK(std::isfinite(w.velocity.end.x()));
}

// change honest-degeneracy-measure (a): with the previous cell statistics a cell's normal_vector is fixed from its first 3
// points, and the corridor's information then reads strong along its axis. The normals from all of a cell's points show the
// blind direction, without changing the cells the points are matched to.
TEST_CASE("scan-to-map: with the previous cell statistics, the normals from all points show the corridor's blind direction")
{
    const auto information = [](bool all_point_normals)
    {
        Options o = options_from(test_config());
        o.scan_to_map.degenerate_projection_all_point_normals = all_point_normals;
        const Scene s = corridor();
        std::vector<Point3Di> map_points = sample_scene(s);
        Maps maps;
        update_rgd(o.scan_to_map.grid_indoor, maps.indoor, map_points, s.centre, false);
        update_rgd(o.scan_to_map.grid_outdoor, maps.outdoor, map_points, s.centre, false);
        const std::vector<Eigen::Affine3d> truth = true_poses();
        std::vector<Point3Di> points = observe(s, truth);
        Window w = make_window(truth);
        NullLogger logger;
        Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
        run_iterations(points, w, maps, o, 5.0, 1, logger, &info);
        return info;
    };
    const Eigen::Matrix3d first_three = information(false), all_points = information(true);
    const auto share = [](const Eigen::Matrix3d &info) { return info(0, 0) / std::max(info(1, 1), info(2, 2)); };
    MESSAGE("corridor, previous statistics: share along the axis ", share(first_three), " with the first-3-point normals, ", share(all_points),
            " with the normals from all points");
    CHECK(share(first_three) > 0.2);  // reads as if the axis were constrained (above the projection thresholds)
    CHECK(share(all_points) < 0.02);  // the axis is blind
}

// change honest-degeneracy-measure (d): the covariance handed to the next window counts the NDT terms only by their
// normal-direction information. In the corridor without the projection, the points' pull along the walls makes the end
// velocity along the axis look well known; with the setting it is about as uncertain as the accelerometer leaves it, while
// across the axis, where the walls measure, it stays as it was. The poses are the same either way.
TEST_CASE("scan-to-map, imu always on: the carried covariance takes no confidence from the pull along a wall")
{
    const auto run = [](bool normal_only)
    {
        AlwaysOnCase k;
        k.truth = straight(1.0, 0.0);
        k.guess = k.truth;
        k.world_acceleration.assign(n_nodes, Eigen::Vector3d::Zero());
        k.velocity_prior = Eigen::Vector3d(1.0, 0.0, 0.0);
        k.velocity_prior_sigma = 0.5;
        k.project_degenerate = false;
        k.normal_only_covariance = normal_only;
        return run_always_on(corridor(), k);
    };
    const Window plain = run(false), honest = run(true);
    const auto sigma = [](const Window &w, int axis) { return std::sqrt(w.velocity.end_covariance(axis, axis)); };
    MESSAGE("corridor, end velocity sigma along the axis ", sigma(plain, 0), " -> ", sigma(honest, 0), " m/s, across ", sigma(plain, 1), " -> ",
            sigma(honest, 1), " m/s");
    CHECK(sigma(honest, 0) > 2.0 * sigma(plain, 0)); // measured 5.9 -> 13.6 mm/s; across 3.9 -> 3.9 mm/s
    CHECK(sigma(honest, 1) < 2.0 * sigma(plain, 1));
    for (size_t i = 0; i < plain.poses.size(); i++)
        CHECK(plain.poses[i].matrix() == honest.poses[i].matrix());
}
