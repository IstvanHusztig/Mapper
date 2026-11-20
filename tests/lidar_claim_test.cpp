#include <doctest/doctest.h>

#include "angle_units.h"
#include "ndt_map.h"
#include "ndt_term.h"
#include "odometry_config.h"
#include "optimizer_options.h"
#include "synthetic_recording.h"
#include "truth_statistics.h"
#include "window_noise_model.h"
#include <transformations.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// K8 of the consistency harness (openspec change consistency-harness, task 5.6; design D7 row K8, research.md F4): the
// LiDAR (NDT) term's claimed precision against its actual scatter, by Monte Carlo on synthetic scenes with a known truth.
// MEASURED and reported beside the audit's hypotheses; nothing is marked doctest::should_fail (task 5.8). The CHECKs are
// the fixture's validity, never the honest outcome.
//
// The fixture:
//  - scenes: the room (8 x 5 x 3 m, closed) and the corridor (30 x 5 x 3 m, open at both ends) of scan_to_map_test, and a
//    single flat wall along travel (y = 2.5 m, 30 m long, 3 m high); all at synthetic::grid_offset (every face at least
//    2.5 cm from a cell boundary of either grid);
//  - one window: 50 nodes 10 ms apart, from (-1, 0, 1.5) along x at 1 m/s yawing at 20 deg/s (scan_to_map_test's);
//  - noise model: every scan or map point at a uniform random position on the faces (by area: no ray casting, no
//    range-dependent density, no occlusion), moved along its face normal by N(0, sigma), sigma = 1 cm, independent;
//  - scan: 444 points per m2 of face (about 10 per 15 cm cell face), assigned to the nodes in turn, in each node's true
//    frame, redrawn per trial;
//  - the exact map ("true map"): each cell's mean and covariance computed analytically as the moments of the noise model
//    in that cell (uniform on each face's part inside the cell, N(0, sigma^2) along its normal; several faces mixed by
//    area), the normal as update_rgd picks it; fixed over trials. This is the audit's premise "an exact map"; the
//    Gaussian's tails beyond the cell (2.5 sigma away at least) are ignored;
//  - the "one earlier window" map: an independent draw at the scan's density and noise, redrawn per trial, at the true
//    positions (no pose error of the earlier window: the audit's 1.4 does not contain that either), through update_rgd
//    with exact cell statistics (use_exact_ndt_bucket_statistics; its 3 cm isotropic prior worth one point included);
//  - both grids 0.15 m and 0.4 m; the NDT term as the shipped Livox file sets it: Huber 3, view-point test on, no
//    planarity / range weight, ranges 0.15 m (fine), 1.0 m (coarse) to 40 m. The information scale is 1 here: the term's
//    per-point information before the shipped 0.1. In a pure-NDT solve the scale cancels from the estimate and multiplies
//    every claimed sigma by 1 / sqrt(0.1) = 3.162 (every ratio below is divided by 3.162 under the shipped scale);
//  - the estimate: Gauss-Newton on the term's own node blocks (sum_ndt_grid_blocks, public) from the truth, 5 steps
//    (a) a 1-D rigid shift of all nodes along one world axis with every other motion held: claimed sigma
//    1 / sqrt(m^T H m); (b) a rigid 6-DoF motion of the whole window (rotation about the nodes' centroid; NDT only, no
//    motion model): the translation's marginal sigma sqrt((J^T H J)^-1)_aa, labelled separately. The claim is taken from
//    the last linearization. The last step is reported in claimed sigmas (convergence);
//  - ratio R = sqrt(mean(e^2 / claim^2)) over trials (1 if the claim is honest), with its 95 % chi-square interval.
//
// Hypotheses (audit_lidar_solver.md section 3, research.md F4): one-earlier-window map, independent 1 cm noise on scan and
// map, R = 1.4 with the fine grid alone (sqrt 2: the map's cell means as noisy as the scan) and about 1.8 with both grids
// (sqrt 2 x sqrt 1.6; on the real run both grids added +57 to +80 % information); 1.0 if honest. The flat wall: the scan
// shifted 5 mm along the wall, the in-plane information positive and the pull about 0 (under 0.5 mm); on the run the
// in-plane part saw 0.41 of an offset.

namespace
{
    using namespace synthetic;

    constexpr double sigma = 0.01;         // normal noise of every point [m]
    constexpr double scan_density = 444.0; // scan (and one-window map) points per m2 of face
    constexpr int n_nodes = 50;
    constexpr double node_dt = 0.01;
    constexpr int gn_steps = 5;

    // Where the scenes lie in the grids. synthetic::grid_offset keeps every face 2.5 cm (2.5 sigma) from a cell boundary:
    // the points beyond it fall into the next cell, which the exact map doesn't have (and a one-window map has only as a
    // few-point cell), so the matched points of a face are a one-sided truncated Gaussian, whose mean is off by
    // sigma phi(2.5) / Phi(2.5) = 0.18 mm. clear_offset keeps every face of these scenes at least 5 cm (5 sigma) from every
    // boundary of both grids (found by a search over offsets), so the truncation is below 1e-6 sigma; the claim is
    // measured there, and the truncation separately on the wall at grid_offset.
    const Eigen::Vector3d clear_offset(0.30, 0.15, 0.075);

    Scene room() { return shifted(box_scene({-4.0, -2.5, 0.0}, {4.0, 2.5, 3.0}, false), clear_offset); }
    Scene corridor() { return shifted(box_scene({-15.0, -2.5, 0.0}, {15.0, 2.5, 3.0}, true), clear_offset); }
    // one wall along travel at y = 2.5, the viewport on the travel side
    Scene wall(const Eigen::Vector3d &offset)
    {
        Scene s;
        s.faces.push_back({1, 2.5, Eigen::Vector3d(-15.0, 2.5, 0.0), Eigen::Vector3d(15.0, 2.5, 3.0)});
        s.centre = Eigen::Vector3d(0.0, 0.0, 1.5);
        return shifted(s, offset);
    }

    std::vector<Eigen::Affine3d> true_poses(const Eigen::Vector3d &offset)
    {
        std::vector<Eigen::Affine3d> poses;
        for (int k = 0; k < n_nodes; k++)
        {
            const double t = k * node_dt;
            poses.push_back(pose(Eigen::Vector3d(-1.0 + 1.0 * t, 0.0, 1.5) + offset, 20.0 * DEG_TO_RAD * t));
        }
        return poses;
    }

    Window make_window(const std::vector<Eigen::Affine3d> &poses)
    {
        Window w;
        w.poses = poses;
        w.motion_model_poses = poses;
        for (int k = 0; k < n_nodes; k++)
            w.timestamps.push_back({k * node_dt, k * node_dt});
        return w;
    }

    double scene_area(const Scene &s)
    {
        double a = 0.0;
        for (const Face &f : s.faces)
            a += face_area(f);
        return a;
    }

    std::vector<Eigen::Vector3d> draw_points(const Scene &s, double density, truth_stats::Rng &rng)
    {
        const double total = scene_area(s);
        const int n = static_cast<int>(std::lround(total * density));
        std::vector<Eigen::Vector3d> out;
        out.reserve(n);
        for (int i = 0; i < n; i++)
        {
            double r = rng.uniform() * total;
            const Face *f = &s.faces.back();
            for (const Face &face : s.faces)
            {
                r -= face_area(face);
                if (r < 0.0)
                {
                    f = &face;
                    break;
                }
            }
            const double u = rng.uniform(), v = rng.uniform();
            out.push_back(face_point(*f, u, v, sigma * rng.normal()));
        }
        return out;
    }

    // the exact cells of the noise model: per cell the area-weighted moments of the faces' parts inside it
    NDTBucketMapType exact_map(const Scene &s, double cell, const Eigen::Vector3d &viewport)
    {
        struct Moments
        {
            double w = 0.0;
            Eigen::Vector3d m1 = Eigen::Vector3d::Zero();
            Eigen::Matrix3d m2 = Eigen::Matrix3d::Zero();
        };
        std::map<uint64_t, Moments> acc;
        const Eigen::Vector3d b(cell, cell, cell);
        for (const Face &f : s.faces)
        {
            const int n = f.axis, a1 = (n + 1) % 3, a2 = (n + 2) % 3;
            for (int i1 = static_cast<int>(std::floor(f.lo[a1] / cell)); i1 * cell < f.hi[a1]; i1++)
                for (int i2 = static_cast<int>(std::floor(f.lo[a2] / cell)); i2 * cell < f.hi[a2]; i2++)
                {
                    const double lo1 = std::max(f.lo[a1], i1 * cell), hi1 = std::min(f.hi[a1], (i1 + 1) * cell);
                    const double lo2 = std::max(f.lo[a2], i2 * cell), hi2 = std::min(f.hi[a2], (i2 + 1) * cell);
                    const double w1 = hi1 - lo1, w2 = hi2 - lo2;
                    if (w1 <= 0.0 || w2 <= 0.0)
                        continue;
                    Eigen::Vector3d c;
                    c[n] = f.value;
                    c[a1] = 0.5 * (lo1 + hi1);
                    c[a2] = 0.5 * (lo2 + hi2);
                    Eigen::Matrix3d d = Eigen::Matrix3d::Zero();
                    d(n, n) = sigma * sigma;
                    d(a1, a1) = w1 * w1 / 12.0;
                    d(a2, a2) = w2 * w2 / 12.0;
                    Moments &m = acc[ndt_cell_index(c, b)];
                    const double area = w1 * w2;
                    m.w += area;
                    m.m1 += area * c;
                    m.m2 += area * (c * c.transpose() + d);
                }
        }
        NDTBucketMapType map;
        for (const auto &kv : acc)
        {
            NdtBucket bucket{};
            bucket.mean = kv.second.m1 / kv.second.w;
            bucket.cov = kv.second.m2 / kv.second.w - bucket.mean * bucket.mean.transpose();
            bucket.number_of_points = static_cast<unsigned long long>(std::lround(kv.second.w * 1e4)); // area in cm2, for the report
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(bucket.cov);
            bucket.normal_vector = Eigen::Vector3d::Zero();
            if (e.eigenvalues()(1) >= 3.0 * e.eigenvalues()(0)) // update_rgd's ambiguous_normal_ratio
            {
                Eigen::Vector3d nv = e.eigenvectors().col(0).normalized();
                if (nv.dot(viewport - bucket.mean) < 0.0)
                    nv *= -1.0;
                bucket.normal_vector = nv;
            }
            refresh_cell_inverse(bucket);
            map.emplace(kv.first, bucket);
        }
        return map;
    }

    struct Maps
    {
        NDTBucketMapType fine, coarse;
    };

    Maps window_maps(const std::vector<Eigen::Vector3d> &world, const Scene &s, const ScanToMapOptions &o)
    {
        std::vector<Point3Di> points(world.size());
        for (size_t i = 0; i < world.size(); i++)
        {
            points[i] = Point3Di{};
            points[i].point = world[i];
        }
        Maps m;
        update_rgd(o.grid_indoor, m.fine, points, s.centre, true);
        update_rgd(o.grid_outdoor, m.coarse, points, s.centre, true);
        return m;
    }

    // world points to a scan: node k = i mod n_nodes, in the node's true frame
    std::vector<Point3Di> to_scan(const std::vector<Eigen::Vector3d> &world, const std::vector<Eigen::Affine3d> &truth)
    {
        std::vector<Point3Di> scan(world.size());
        std::vector<Eigen::Affine3d> inverse(truth.size());
        for (size_t k = 0; k < truth.size(); k++)
            inverse[k] = truth[k].inverse();
        for (size_t i = 0; i < world.size(); i++)
        {
            const int k = static_cast<int>(i % n_nodes);
            Point3Di p{};
            p.point = inverse[k] * world[i];
            p.timestamp = k * node_dt;
            p.index_pose = k;
            p.index_point = static_cast<int>(i);
            scan[i] = p;
        }
        return scan;
    }

    ScanToMapOptions claim_options()
    {
        OdometryConfig c;
        c.in_out_params_indoor.resolution_X = c.in_out_params_indoor.resolution_Y = c.in_out_params_indoor.resolution_Z = 0.15;
        c.in_out_params_outdoor.resolution_X = c.in_out_params_outdoor.resolution_Y = c.in_out_params_outdoor.resolution_Z = 0.4;
        c.min_distance_lidar = 0.15;
        c.max_distance_lidar = 40.0;
        c.min_distance_lidar_outdoor_grid = 1.0;
        c.ablation_study_use_planarity = false;
        c.ablation_study_use_norm = false;
        c.ablation_study_use_hierarchical_rgd = true;
        c.ablation_study_use_view_point_and_normal_vectors = true;
        c.degenerate_projection_ratio = 0.0;
        c.degenerate_projection_ratio_full = 0.0;
        c.ndt_huber_cutoff = 3.0;
        c.ndt_information_scale = 1.0;
        return get_scan_to_map_options(c);
    }

    using Block = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;
    using Mat6 = Eigen::Matrix<double, 6, 6>;

    enum GridSet
    {
        fine_only = 0,
        coarse_only = 1,
        both_grids = 2
    };
    const std::string grid_set_name[3] = {"fine", "coarse", "both"};

    // the node blocks and right-hand sides of the grids asked for, at the window's poses
    struct Linearization
    {
        NdtSums fine, coarse;
        int matched_fine = 0, matched_coarse = 0;

        Block block(GridSet g, int k) const
        {
            if (g == fine_only)
                return fine.blocks[k];
            if (g == coarse_only)
                return coarse.blocks[k];
            return Block(fine.blocks[k] + coarse.blocks[k]);
        }
        Vec6 rhs(GridSet g, int k) const
        {
            if (g == fine_only)
                return fine.rhs[k];
            if (g == coarse_only)
                return coarse.rhs[k];
            return fine.rhs[k] + coarse.rhs[k];
        }
    };

    struct Fixture
    {
        ScanToMapOptions o = claim_options();
        NdtNoise noise = ndt_noise(o);
        std::vector<Eigen::Affine3d> truth;
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        std::vector<Mat6> js;
        explicit Fixture(const Eigen::Vector3d &offset);
    };

    Linearization linearize(const Fixture &fx, const Maps &maps, const std::vector<Eigen::Affine3d> &poses, const std::vector<Point3Di> &scan, GridSet g)
    {
        const Window w = make_window(poses);
        Linearization l;
        if (g != coarse_only)
        {
            l.fine = sum_ndt_grid_blocks(indoor_ndt_grid(maps.fine, fx.o, fx.noise), w, scan, fx.o, true);
            for (int k = 0; k < n_nodes; k++)
                l.matched_fine += l.fine.node_matched[k];
        }
        if (g != fine_only)
        {
            l.coarse = sum_ndt_grid_blocks(outdoor_ndt_grid(maps.coarse, fx.o, fx.noise), w, scan, fx.o, true);
            for (int k = 0; k < n_nodes; k++)
                l.matched_coarse += l.coarse.node_matched[k];
        }
        return l;
    }

    // 1-D rigid shift along world axis a: the step and the claimed sigma
    struct Scalar
    {
        double step = 0.0, claim = 0.0, information = 0.0;
    };
    Scalar shift_1d(const Linearization &l, GridSet g, int a)
    {
        double h = 0.0, r = 0.0;
        for (int k = 0; k < n_nodes; k++)
        {
            h += l.block(g, k)(a, a);
            r += l.rhs(g, k)(a);
        }
        return {r / h, 1.0 / std::sqrt(h), h};
    }

    // a rigid motion xi = (t, theta) of the whole window: rotation theta (rotation vector) about c, then the shift t
    Eigen::Affine3d rigid_motion(const Vec6 &xi, const Eigen::Vector3d &c)
    {
        const Eigen::Vector3d th = xi.tail<3>();
        const double angle = th.norm();
        const Eigen::Matrix3d r = angle > 0.0 ? Eigen::AngleAxisd(angle, th / angle).toRotationMatrix() : Eigen::Matrix3d::Identity();
        Eigen::Affine3d m = Eigen::Affine3d::Identity();
        m.linear() = r;
        m.translation() = c - r * c + xi.head<3>();
        return m;
    }

    Vec6 tait_bryan_vector(const Eigen::Affine3d &m)
    {
        const TaitBryanPose p = pose_tait_bryan_from_affine_matrix(m);
        Vec6 v;
        v << p.px, p.py, p.pz, p.om, p.fi, p.ka;
        return v;
    }

    // d(node k's Tait-Bryan parameters) / d(xi) at xi = 0, central differences
    std::vector<Mat6> rigid_jacobians(const std::vector<Eigen::Affine3d> &poses, const Eigen::Vector3d &c)
    {
        std::vector<Mat6> js(poses.size());
        const double h = 1e-6;
        for (size_t k = 0; k < poses.size(); k++)
            for (int j = 0; j < 6; j++)
            {
                Vec6 d = Vec6::Zero();
                d(j) = h;
                js[k].col(j) = (tait_bryan_vector(rigid_motion(d, c) * poses[k]) - tait_bryan_vector(rigid_motion(-d, c) * poses[k])) / (2.0 * h);
            }
        return js;
    }

    Fixture::Fixture(const Eigen::Vector3d &offset) : truth(true_poses(offset))
    {
        for (const auto &p : truth)
            c += p.translation();
        c /= static_cast<double>(truth.size());
        js = rigid_jacobians(truth, c);
    }

    struct Rigid
    {
        Vec6 step = Vec6::Zero();
        Eigen::Vector3d marginal = Eigen::Vector3d::Zero(); // translation marginal sigmas
    };
    Rigid rigid_6dof(const Linearization &l, GridSet g, const std::vector<Mat6> &js)
    {
        Mat6 h = Mat6::Zero();
        Vec6 r = Vec6::Zero();
        for (int k = 0; k < n_nodes; k++)
        {
            const Mat6 b = l.block(g, k);
            h += js[k].transpose() * b * js[k];
            r += js[k].transpose() * l.rhs(g, k);
        }
        const Eigen::LDLT<Mat6> ldlt(h);
        Rigid out;
        out.step = ldlt.solve(r);
        const Mat6 cov = ldlt.solve(Mat6::Identity());
        for (int a = 0; a < 3; a++)
            out.marginal(a) = std::sqrt(cov(a, a));
        return out;
    }

    // errors and claims of one quantity over trials
    struct Series
    {
        std::vector<double> e, c;
        double last_step = 0.0; // largest |last Gauss-Newton step| / claim
        void add(double error, double claim)
        {
            e.push_back(error);
            c.push_back(claim);
        }
        int n() const { return static_cast<int>(e.size()); }
        double sum_z2() const
        {
            double s = 0.0;
            for (int i = 0; i < n(); i++)
                s += (e[i] / c[i]) * (e[i] / c[i]);
            return s;
        }
        double ratio() const { return std::sqrt(sum_z2() / n()); }
        // the same about the mean of e / claim (the scatter without a common bias)
        double centred_ratio() const
        {
            double m = 0.0;
            for (int i = 0; i < n(); i++)
                m += e[i] / c[i];
            m /= n();
            double s = 0.0;
            for (int i = 0; i < n(); i++)
                s += (e[i] / c[i] - m) * (e[i] / c[i] - m);
            return n() > 1 ? std::sqrt(s / (n() - 1)) : 0.0;
        }
        double rms_error() const
        {
            double s = 0.0;
            for (double x : e)
                s += x * x;
            return std::sqrt(s / n());
        }
        double mean_error() const
        {
            double s = 0.0;
            for (double x : e)
                s += x;
            return s / n();
        }
        double rms_claim() const
        {
            double s = 0.0;
            for (double x : c)
                s += x * x;
            return std::sqrt(s / n());
        }
        std::string text() const
        {
            const auto iv = truth_stats::chi2_interval(n(), 0.05);
            std::ostringstream o;
            o << std::fixed << std::setprecision(3) << "R " << ratio() << " [" << std::sqrt(sum_z2() / iv.second) << ", "
              << std::sqrt(sum_z2() / iv.first) << "], about the mean " << centred_ratio() << " (rms " << 1000.0 * rms_error() << " mm, mean " << 1000.0 * mean_error() << " mm, claim "
              << 1000.0 * rms_claim() << " mm, last step " << std::setprecision(2) << last_step << " claims)";
            return o.str();
        }
    };

    enum MapKind
    {
        true_map = 0,
        window_map = 1
    };
    const std::string map_name[2] = {"exact map", "one-window map"};

    // the results of one scene over trials, per map kind, grid set and axis
    struct SceneSeries
    {
        Series one_d[2][3][3]; // [map][grid set][axis]
        Series rigid[2][3][3]; // translation marginal, rotation free
        double information[2][3][3] = {}; // summed 1-D information at the truth (duplication factor)
        double matched_fine = 0.0, matched_coarse = 0.0, scan_points = 0.0;
        int trials = 0;
    };

    void solve_1d(const Fixture &fx, const Maps &maps, const std::vector<Point3Di> &scan, const Linearization &at_truth, GridSet g, int a, Series &out)
    {
        Scalar sc = shift_1d(at_truth, g, a);
        double shift = sc.step;
        for (int it = 1; it < gn_steps; it++)
        {
            std::vector<Eigen::Affine3d> moved = fx.truth;
            for (auto &p : moved)
                p.translation()(a) += shift;
            sc = shift_1d(linearize(fx, maps, moved, scan, g), g, a);
            shift += sc.step;
        }
        out.add(shift, sc.claim);
        out.last_step = std::max(out.last_step, std::abs(sc.step) / sc.claim);
    }

    void solve_6dof(const Fixture &fx, const Maps &maps, const std::vector<Point3Di> &scan, const Linearization &at_truth, GridSet g, Series out[3])
    {
        Rigid r = rigid_6dof(at_truth, g, fx.js);
        Vec6 xi = r.step;
        for (int it = 1; it < gn_steps; it++)
        {
            // the step composes: the next motion is the step's motion after the current one (xi + step to first order;
            // with steps of micrometres and microradians the composition error is negligible)
            std::vector<Eigen::Affine3d> moved = fx.truth;
            const Eigen::Affine3d m = rigid_motion(xi, fx.c);
            for (auto &p : moved)
                p = m * p;
            r = rigid_6dof(linearize(fx, maps, moved, scan, g), g, rigid_jacobians(moved, fx.c));
            xi += r.step;
        }
        for (int a = 0; a < 3; a++)
        {
            out[a].add(xi(a), r.marginal(a));
            out[a].last_step = std::max(out[a].last_step, std::abs(r.step(a)) / r.marginal(a));
        }
    }

    void run_trial(const Fixture &fx, const Scene &s, const Maps &exact, std::uint64_t seed, SceneSeries &out)
    {
        truth_stats::Rng scan_rng(seed * 2 + 1), map_rng(seed * 2 + 2);
        const std::vector<Point3Di> scan = to_scan(draw_points(s, scan_density, scan_rng), fx.truth);
        const Maps window = window_maps(draw_points(s, scan_density, map_rng), s, fx.o);
        for (int m = 0; m < 2; m++)
        {
            const Maps &maps = m == true_map ? exact : window;
            const Linearization l = linearize(fx, maps, fx.truth, scan, both_grids);
            if (m == true_map)
            {
                out.matched_fine += l.matched_fine;
                out.matched_coarse += l.matched_coarse;
                out.scan_points += static_cast<double>(scan.size());
            }
            for (int g = 0; g < 3; g++)
            {
                for (int a = 0; a < 3; a++)
                {
                    out.information[m][g][a] += shift_1d(l, static_cast<GridSet>(g), a).information;
                    solve_1d(fx, maps, scan, l, static_cast<GridSet>(g), a, out.one_d[m][g][a]);
                }
                solve_6dof(fx, maps, scan, l, static_cast<GridSet>(g), out.rigid[m][g]);
            }
        }
        out.trials++;
    }

    std::string axis_name(int a) { return std::string(1, "xyz"[a]); }

    struct CellStats
    {
        double median_points = 0.0;
        double median_normal_sigma_mm = 0.0;
        int cells = 0;
    };
    CellStats cell_stats(const NDTBucketMapType &m)
    {
        std::vector<double> n, s;
        for (const auto &kv : m)
        {
            const NdtBucket &b = kv.second;
            if (b.number_of_points < 3)
                continue;
            n.push_back(static_cast<double>(b.number_of_points));
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> e(b.cov);
            s.push_back(1000.0 * std::sqrt(std::max(0.0, e.eigenvalues()(0))));
        }
        CellStats c;
        c.cells = static_cast<int>(n.size());
        if (n.empty())
            return c;
        std::nth_element(n.begin(), n.begin() + n.size() / 2, n.end());
        std::nth_element(s.begin(), s.begin() + s.size() / 2, s.end());
        c.median_points = n[n.size() / 2];
        c.median_normal_sigma_mm = s[s.size() / 2];
        return c;
    }

    void report_scene(const std::string &name, const SceneSeries &r, const CellStats &exact_cells, const CellStats &window_cells)
    {
        MESSAGE("K8 ", name, ": ", r.trials, " trials; scan points per trial ", r.scan_points / r.trials, ", matched fine ",
                100.0 * r.matched_fine / r.scan_points, " %, coarse ", 100.0 * r.matched_coarse / r.scan_points, " % (exact map); exact map: ",
                exact_cells.cells, " fine cells, normal sigma ", exact_cells.median_normal_sigma_mm, " mm; one-window map (one draw): ",
                window_cells.cells, " fine cells with 3+ points, median ", window_cells.median_points, " points, normal sigma ",
                window_cells.median_normal_sigma_mm, " mm");
        for (int m = 0; m < 2; m++)
            for (int g = 0; g < 3; g++)
                for (int a = 0; a < 3; a++)
                    MESSAGE("K8 ", name, " ", map_name[m], ", ", grid_set_name[g], ", ", axis_name(a), ": 1-D ", r.one_d[m][g][a].text(),
                            " | 6-DoF marginal ", r.rigid[m][g][a].text());
        for (int a = 0; a < 3; a++)
        {
            const auto &d = r.one_d;
            MESSAGE("K8 ", name, " ", axis_name(a), ": duplication (1-D information both / fine at the truth, exact map) ",
                    r.information[true_map][both_grids][a] / r.information[true_map][fine_only][a],
                    ", one-window map ", r.information[window_map][both_grids][a] / r.information[window_map][fine_only][a],
                    " (hypothesis 1.57-1.80, C18 / F4); scatter both / fine: exact ", d[true_map][both_grids][a].rms_error() / d[true_map][fine_only][a].rms_error(),
                    ", one-window ", d[window_map][both_grids][a].rms_error() / d[window_map][fine_only][a].rms_error(),
                    "; map-noise factor R(one-window) / R(exact): fine ", d[window_map][fine_only][a].ratio() / d[true_map][fine_only][a].ratio(),
                    ", both ", d[window_map][both_grids][a].ratio() / d[true_map][both_grids][a].ratio(),
                    " (hypothesis 1.41); R one-window: fine ", d[window_map][fine_only][a].ratio(), " (hypothesis 1.4), both ",
                    d[window_map][both_grids][a].ratio(), " (hypothesis about 1.8)");
        }
    }

    // the flat wall: claimed hold and actual pull of an in-plane offset (1-D shifts)
    struct WallSeries
    {
        Series at_truth[2][3][3];               // [map][grid set][axis], solved from the truth
        double info[2][3][3] = {};              // at the truth
        double one_step_response[2][3][2] = {}; // [map][grid][x, z]: - first step / offset, summed over trials
        double left_after[2][3][2] = {};        // offset left after wall_steps steps from 5 mm [m], summed
        int trials = 0;
    };
    constexpr int wall_steps = 5;
    constexpr double wall_offset = 0.005;

    // offsets: the in-plane offset runs too (else only the solves from the truth)
    void run_wall_trial(const Fixture &fx, const Scene &s, const Maps &exact, std::uint64_t seed, bool offsets, WallSeries &out)
    {
        truth_stats::Rng scan_rng(seed * 2 + 1), map_rng(seed * 2 + 2);
        const std::vector<Point3Di> scan = to_scan(draw_points(s, scan_density, scan_rng), fx.truth);
        const Maps window = window_maps(draw_points(s, scan_density, map_rng), s, fx.o);
        for (int m = 0; m < 2; m++)
        {
            const Maps &maps = m == true_map ? exact : window;
            const Linearization l = linearize(fx, maps, fx.truth, scan, both_grids);
            for (int g = 0; g < 3; g++)
                for (int a = 0; a < 3; a++)
                {
                    out.info[m][g][a] += shift_1d(l, static_cast<GridSet>(g), a).information;
                    solve_1d(fx, maps, scan, l, static_cast<GridSet>(g), a, out.at_truth[m][g][a]);
                }
            for (int i = 0; offsets && i < 2; i++)
            {
                const int a = i == 0 ? 0 : 2; // along the wall: x (travel) and z (vertical)
                for (int g = 0; g < 3; g++)
                {
                    double shift = wall_offset;
                    for (int it = 0; it < wall_steps; it++)
                    {
                        std::vector<Eigen::Affine3d> moved = fx.truth;
                        for (auto &p : moved)
                            p.translation()(a) += shift;
                        const Scalar sc = shift_1d(linearize(fx, maps, moved, scan, static_cast<GridSet>(g)), static_cast<GridSet>(g), a);
                        if (it == 0)
                            out.one_step_response[m][g][i] += -sc.step / wall_offset;
                        shift += sc.step;
                    }
                    out.left_after[m][g][i] += shift;
                }
            }
        }
        out.trials++;
    }

    void report_wall(const WallSeries &r, const CellStats &exact_cells, const CellStats &window_cells)
    {
        MESSAGE("K8 wall: ", r.trials, " trials; exact map ", exact_cells.cells, " fine cells, normal sigma ", exact_cells.median_normal_sigma_mm,
                " mm; one-window map median ", window_cells.median_points, " points, normal sigma ", window_cells.median_normal_sigma_mm, " mm");
        for (int m = 0; m < 2; m++)
            for (int g = 0; g < 3; g++)
            {
                const std::string label = "K8 wall " + map_name[m] + ", " + grid_set_name[g];
                MESSAGE(label, ": solved from the truth, x (along travel, in-plane) ", r.at_truth[m][g][0].text(), "; y (normal) ",
                        r.at_truth[m][g][1].text(), "; z (vertical, in-plane) ", r.at_truth[m][g][2].text());
                MESSAGE(label, ": 5 mm offset along x: claimed hold ", 1000.0 / std::sqrt(r.info[m][g][0] / r.trials), " mm, first-step response ",
                        r.one_step_response[m][g][0] / r.trials, " of the offset, ", 1000.0 * r.left_after[m][g][0] / r.trials, " mm left after ",
                        wall_steps, " steps; along z: claimed hold ", 1000.0 / std::sqrt(r.info[m][g][2] / r.trials), " mm, first-step response ",
                        r.one_step_response[m][g][1] / r.trials, ", ", 1000.0 * r.left_after[m][g][1] / r.trials,
                        " mm left (hypotheses: in-plane information positive, pull about 0, under 0.5 mm of 5 (response < 0.1); on the run 0.41)");
            }
    }

    void report_truncation(const WallSeries &r)
    {
        for (int m = 0; m < 2; m++)
            for (int g = 0; g < 3; g++)
                MESSAGE("K8 wall at grid_offset (face 2.5 cm from a cell boundary), ", map_name[m], ", ", grid_set_name[g], ": y (normal) ",
                        r.at_truth[m][g][1].text(), " (one-sided truncation at 2.5 sigma predicts a mean of 0.18 mm in size; at clear_offset see above)");
    }

    struct ClaimRun
    {
        SceneSeries room, corridor;
        WallSeries wall, wall_truncated;
        CellStats cells_exact[3], cells_window[3];
        double seconds = 0.0;
    };

    ClaimRun run_claim(int room_trials, int corridor_trials, int wall_trials)
    {
        const auto t0 = std::chrono::steady_clock::now();
        const Fixture fx(clear_offset);
        ClaimRun run;
        const Scene scenes[3] = {room(), corridor(), wall(clear_offset)};
        Maps exact[3];
        for (int i = 0; i < 3; i++)
        {
            exact[i].fine = exact_map(scenes[i], fx.o.grid_indoor.resolution_X, scenes[i].centre);
            exact[i].coarse = exact_map(scenes[i], fx.o.grid_outdoor.resolution_X, scenes[i].centre);
            run.cells_exact[i] = cell_stats(exact[i].fine);
            truth_stats::Rng rng(9100 + i);
            run.cells_window[i] = cell_stats(window_maps(draw_points(scenes[i], scan_density, rng), scenes[i], fx.o).fine);
        }
        for (int t = 0; t < room_trials; t++)
            run_trial(fx, scenes[0], exact[0], 1000 + t, run.room);
        for (int t = 0; t < corridor_trials; t++)
            run_trial(fx, scenes[1], exact[1], 200000 + t, run.corridor);
        for (int t = 0; t < wall_trials; t++)
            run_wall_trial(fx, scenes[2], exact[2], 400000 + t, true, run.wall);
        // the same wall 2.5 cm from a boundary (synthetic::grid_offset): the truncation of the normal noise
        const Fixture fx_grid(grid_offset);
        const Scene wall_grid = wall(grid_offset);
        Maps exact_grid;
        exact_grid.fine = exact_map(wall_grid, fx.o.grid_indoor.resolution_X, wall_grid.centre);
        exact_grid.coarse = exact_map(wall_grid, fx.o.grid_outdoor.resolution_X, wall_grid.centre);
        for (int t = 0; t < wall_trials; t++)
            run_wall_trial(fx_grid, wall_grid, exact_grid, 400000 + t, false, run.wall_truncated);
        run.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return run;
    }

    void report(const ClaimRun &run)
    {
        MESSAGE("K8 noise model: scan and map points at uniform random positions on the faces, N(0, ", 1000.0 * sigma,
                " mm) along the normal, independent; ", scan_density, " points/m2; Huber 3, information scale 1 (the shipped 0.1 divides every R by 3.162); ",
                gn_steps, " Gauss-Newton steps; ", run.seconds, " s");
        if (run.room.trials > 0)
            report_scene("room", run.room, run.cells_exact[0], run.cells_window[0]);
        if (run.corridor.trials > 0)
            report_scene("corridor", run.corridor, run.cells_exact[1], run.cells_window[1]);
        if (run.wall.trials > 0)
        {
            report_wall(run.wall, run.cells_exact[2], run.cells_window[2]);
            report_truncation(run.wall_truncated);
        }
    }

    void check_validity(const ClaimRun &run)
    {
        // the scans are matched in both grids, the exact map's planar cells are as thin as the noise, the one-window map's
        // cells are above the cell test's limit (K9), the wall sees its normal far better than its plane, the solves settled
        CHECK(run.room.matched_fine / run.room.scan_points > 0.95);
        CHECK(run.room.matched_coarse / run.room.scan_points > 0.95);
        CHECK(run.cells_exact[0].median_normal_sigma_mm == doctest::Approx(10.0).epsilon(0.01));
        CHECK(run.cells_window[0].median_normal_sigma_mm > 3.2);
        CHECK(run.wall.info[true_map][fine_only][1] > 10.0 * run.wall.info[true_map][fine_only][0]);
        for (int m = 0; m < 2; m++)
            for (int g = 0; g < 3; g++)
                for (int a = 0; a < 3; a++)
                {
                    CHECK(std::isfinite(run.room.one_d[m][g][a].ratio()));
                    CHECK(run.room.one_d[m][g][a].last_step < 0.5);
                    CHECK(run.room.rigid[m][g][a].last_step < 0.5);
                }
    }
}

TEST_CASE("lidar claim smoke: K8 fixture valid, the NDT term's claim against its scatter on a few trials")
{
    const ClaimRun run = run_claim(4, 0, 1);
    report(run);
    check_validity(run);
}

TEST_CASE("lidar claim: K8 the NDT term's claimed precision against its scatter, 500 room trials" * doctest::skip())
{
    const ClaimRun run = run_claim(500, 150, 60);
    report(run);
    check_validity(run);
}
