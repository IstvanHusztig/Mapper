#include "loop_fine_time.h"
#include "loop_closure.h"
#include "angle_units.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <map>
#include <sstream>
#include <tuple>
#include <unordered_set>

// ---- the pure parts ---------------------------------------------------------------------------------------------------

double fine_time_pairwise_sum(const double *a, std::size_t n)
{
    if (n < 8)
    {
        double res = 0.0;
        for (std::size_t i = 0; i < n; i++)
            res += a[i];
        return res;
    }
    if (n <= 128)
    {
        double r[8];
        for (int j = 0; j < 8; j++)
            r[j] = a[j];
        std::size_t i = 8;
        for (; i < n - (n % 8); i += 8)
            for (int j = 0; j < 8; j++)
                r[j] += a[i + j];
        double res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; i++)
            res += a[i];
        return res;
    }
    std::size_t n2 = n / 2;
    n2 -= n2 % 8;
    return fine_time_pairwise_sum(a, n2) + fine_time_pairwise_sum(a + n2, n - n2);
}

namespace
{
    struct VoxelKey
    {
        std::int64_t x, y, z;
        bool operator==(const VoxelKey &o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct VoxelKeyHash
    {
        std::size_t operator()(const VoxelKey &k) const
        {
            std::uint64_t h = 1469598103934665603ULL;
            for (const std::int64_t v : {k.x, k.y, k.z})
                h = (h ^ static_cast<std::uint64_t>(v)) * 1099511628211ULL;
            return static_cast<std::size_t>(h);
        }
    };

    VoxelKey voxel_of(const Eigen::Vector3d &p, double v)
    {
        return {static_cast<std::int64_t>(std::floor(p.x() / v)), static_cast<std::int64_t>(std::floor(p.y() / v)),
                static_cast<std::int64_t>(std::floor(p.z() / v))};
    }

    // the dot product in the order numpy's einsum and matmul take it for three elements
    double dot3(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
    {
        return (a.x() * b.x() + a.y() * b.y()) + a.z() * b.z();
    }

    double mean_of(const std::vector<double> &v)
    {
        return fine_time_pairwise_sum(v.data(), v.size()) / static_cast<double>(v.size());
    }

    // the best of the candidates, tried in the order of preference, a later one taken only when lower by more than the
    // tolerance (rule 1); cost(r) gives the candidate's cost
    template <typename Cost>
    std::pair<double, int> best_of(const std::vector<int> &preference, double tolerance, Cost cost)
    {
        int arg = preference[0];
        double best = cost(arg);
        for (size_t p = 1; p < preference.size(); p++)
        {
            const int r = preference[p];
            const double c = cost(r);
            if (c < best - tolerance)
            {
                best = c;
                arg = r;
            }
        }
        return {best, arg};
    }

    int zero_index(const std::vector<double> &grid)
    {
        int zero = 0;
        for (int k = 1; k < static_cast<int>(grid.size()); k++)
            if (std::abs(grid[k]) < std::abs(grid[zero]))
                zero = k;
        return zero;
    }
} // namespace

std::vector<std::size_t> fine_time_thin(const std::vector<Eigen::Vector3d> &points, double v)
{
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    std::vector<std::size_t> kept;
    for (std::size_t k = 0; k < points.size(); k++)
        if (seen.insert(voxel_of(points[k], v)).second)
            kept.push_back(k);
    return kept;
}

std::size_t FineTimeIndex::CellHash::operator()(const Cell &c) const
{
    return VoxelKeyHash{}(VoxelKey{c.x, c.y, c.z});
}

FineTimeIndex::FineTimeIndex(const std::vector<Eigen::Vector3d> &points, double cell) : points_(points), cell_(cell)
{
    // a cell's list holds the indices of the points in it, in input order
    for (std::size_t k = 0; k < points_.size(); k++)
    {
        const VoxelKey c = voxel_of(points_[k], cell_);
        cells_[Cell{c.x, c.y, c.z}].push_back(k);
    }
}

void FineTimeIndex::nearest(const Eigen::Vector3d &q, int k, double reach, std::vector<std::pair<double, std::size_t>> &out) const
{
    out.clear();
    const double limit = reach * reach;
    const double r = reach * (1.0 + 1e-9);
    std::int64_t lo[3], hi[3];
    for (int a = 0; a < 3; a++)
    {
        lo[a] = static_cast<std::int64_t>(std::floor((q(a) - r) / cell_));
        hi[a] = static_cast<std::int64_t>(std::floor((q(a) + r) / cell_));
    }
    for (std::int64_t x = lo[0]; x <= hi[0]; x++)
        for (std::int64_t y = lo[1]; y <= hi[1]; y++)
            for (std::int64_t z = lo[2]; z <= hi[2]; z++)
            {
                const auto it = cells_.find(Cell{x, y, z});
                if (it == cells_.end())
                    continue;
                for (const std::size_t i : it->second)
                {
                    const Eigen::Vector3d &p = points_[i];
                    const double dx = p.x() - q.x(), dy = p.y() - q.y(), dz = p.z() - q.z();
                    const double s = ((0.0 + dx * dx) + dy * dy) + dz * dz;
                    if (s < limit)
                        out.emplace_back(s, i);
                }
            }
    const std::size_t keep = std::min(out.size(), static_cast<std::size_t>(std::max(0, k)));
    std::partial_sort(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(keep), out.end());
    out.resize(keep);
}

FineTimePatches fine_time_patches(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &axis, double v, const LoopFineTimeRule &rule)
{
    FineTimePatches out;
    const std::vector<std::size_t> keep = fine_time_thin(points, v);
    if (keep.size() < 16)
        return out;
    std::vector<Eigen::Vector3d> y;
    y.reserve(keep.size());
    for (const std::size_t k : keep)
        y.push_back(points[k]);
    const FineTimeIndex index(y, rule.neighbour_reach_m);
    // the neighbours nearer than the reach (squared, with a little room: the reach itself is compared as a distance below)
    const double search = rule.neighbour_reach_m * (1.0 + 1e-12);
    std::vector<std::pair<double, std::size_t>> nb;
    const int K = rule.neighbours;
    for (std::size_t i = 0; i < y.size(); i++)
    {
        index.nearest(y[i], K, search, nb);
        if (static_cast<int>(nb.size()) < K || !(std::sqrt(nb[K - 1].first) < rule.neighbour_reach_m))
            continue;
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto &[s, j] : nb)
            mean += y[j];
        mean /= static_cast<double>(K);
        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        for (int a = 0; a < 3; a++)
            for (int b = 0; b < 3; b++)
            {
                double sum = 0.0;
                for (const auto &[s, j] : nb)
                    sum += (y[j](a) - mean(a)) * (y[j](b) - mean(b));
                scatter(a, b) = sum;
            }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(scatter);
        const Eigen::Vector3d l = eig.eigenvalues(); // ascending
        const Eigen::Vector3d normal = eig.eigenvectors().col(0);
        if (l(0) < rule.flat * l(1) && l(1) > rule.wide * l(2) && std::abs(dot3(normal, axis)) >= rule.across)
        {
            out.point.push_back(keep[i]);
            out.normal.push_back(normal);
        }
    }
    return out;
}

bool fine_time_held_out(const Eigen::Vector3d &q, const LoopFineTimeRule &rule)
{
    const std::uint64_t cx = static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(q.x() / rule.holdout_cell_m)));
    const std::uint64_t cy = static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(q.y() / rule.holdout_cell_m)));
    const std::uint64_t cz = static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(q.z() / rule.holdout_cell_m)));
    const std::int64_t h = static_cast<std::int64_t>(73856093ULL * cx + 19349663ULL * cy + 83492791ULL * cz); // wraps as numpy's int64
    std::int64_t m = h % rule.holdout_one_in;
    if (m < 0)
        m += rule.holdout_one_in;
    return m == 0;
}

std::vector<std::size_t> fine_time_take(std::size_t n, std::size_t limit)
{
    std::vector<std::size_t> out;
    if (n <= limit)
    {
        for (std::size_t i = 0; i < n; i++)
            out.push_back(i);
        return out;
    }
    for (std::uint64_t i = 0; i < limit; i++)
        out.push_back(static_cast<std::size_t>(i * static_cast<std::uint64_t>(n - 1) / static_cast<std::uint64_t>(limit - 1)));
    return out;
}

std::vector<double> fine_time_offset_grid(double sigma_m, const LoopFineTimeRule &rule, double &bound_m)
{
    bound_m = std::min(rule.bound_sigmas * sigma_m, rule.bound_max_m);
    const int k = static_cast<int>(std::floor(bound_m * std::round(1.0 / rule.grid_m)));
    std::vector<double> grid;
    for (int i = -k; i <= k; i++)
        grid.push_back(static_cast<double>(i) * rule.grid_m);
    return grid;
}

FineTimeCurve fine_time_score_curve(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n, const std::vector<double> &grid,
                                    const Eigen::Vector3d &axis, const FineTimeIndex &reference, const std::vector<Eigen::Vector3d> &reference_normals,
                                    const LoopFineTimeRule &rule)
{
    FineTimeCurve out;
    std::vector<double> squared(q.size());
    std::vector<std::pair<double, std::size_t>> nb;
    for (const double g : grid)
    {
        const Eigen::Vector3d shift(g * axis.x(), g * axis.y(), g * axis.z());
        int matched = 0;
        for (std::size_t p = 0; p < q.size(); p++)
        {
            const Eigen::Vector3d y(q[p].x() + shift.x(), q[p].y() + shift.y(), q[p].z() + shift.z());
            reference.nearest(y, rule.match_neighbours, rule.match_reach_m, nb);
            double d = rule.match_reach_m;
            bool ok = false;
            for (const auto &[s, i] : nb)
                if (std::abs(dot3(reference_normals[i], n[p])) >= rule.normal_agreement)
                {
                    ok = true;
                    d = std::min(d, std::sqrt(s));
                }
            matched += ok;
            squared[p] = d * d;
        }
        out.score.push_back(mean_of(squared));
        out.matched.push_back(matched);
    }
    return out;
}

std::vector<int> fine_time_preference(const std::vector<double> &grid)
{
    std::vector<int> order(grid.size());
    for (size_t k = 0; k < grid.size(); k++)
        order[k] = static_cast<int>(k);
    const auto size = [&](int k) { return std::abs(std::nearbyint(grid[k] * 1e6)); };
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return size(a) != size(b) ? size(a) < size(b) : (grid[a] > 0.0) < (grid[b] > 0.0); });
    return order;
}

std::vector<int> fine_time_profile(const std::vector<std::vector<double>> &E, const std::vector<double> &grid, const LoopFineTimeRule &rule)
{
    const int W = static_cast<int>(E.size());
    const int G = static_cast<int>(grid.size());
    if (W == 0 || G == 0)
        return {};
    const std::vector<int> pref = fine_time_preference(grid);
    const auto trans = [&](int a, int b)
    {
        const double d = grid[a] - grid[b];
        return rule.coupling * (d * d);
    };
    const int zero = zero_index(grid);
    std::vector<double> val(G), next(G);
    for (int b = 0; b < G; b++)
        val[b] = trans(zero, b) + E[0][b];
    std::vector<std::vector<int>> back(W);
    for (int w = 1; w < W; w++)
    {
        back[w].resize(G);
        for (int c = 0; c < G; c++)
        {
            const auto [best, arg] = best_of(pref, rule.cost_tolerance, [&](int r) { return val[r] + trans(r, c); });
            next[c] = best + E[w][c];
            back[w][c] = arg;
        }
        std::swap(val, next);
    }
    std::vector<int> path(W);
    path[W - 1] = best_of(pref, rule.cost_tolerance, [&](int r) { return val[r] + trans(r, zero); }).second;
    for (int w = W - 1; w >= 1; w--)
        path[w - 1] = back[w][path[w]];
    return path;
}

double fine_time_own_offset(const std::vector<double> &score, const std::vector<double> &grid, const LoopFineTimeRule &rule)
{
    return grid[best_of(fine_time_preference(grid), rule.cost_tolerance, [&](int r) { return score[r]; }).second];
}

double fine_time_offset_at(double t, double pin_begin, double pin_end, const std::vector<double> &middles, const std::vector<double> &offsets)
{
    // numpy.interp with left = right = 0: the interval j with x_j <= t < x_{j+1}, slope (y_{j+1} - y_j) / (x_{j+1} - x_j)
    std::vector<double> x{pin_begin}, y{0.0};
    x.insert(x.end(), middles.begin(), middles.end());
    y.insert(y.end(), offsets.begin(), offsets.end());
    x.push_back(pin_end);
    y.push_back(0.0);
    const size_t n = x.size();
    if (std::isnan(t))
        return t;
    if (t < x.front() || t > x.back())
        return 0.0;
    if (t == x.back())
        return y.back();
    const size_t j = static_cast<size_t>(std::upper_bound(x.begin(), x.end(), t) - x.begin()) - 1; // x_j <= t < x_{j+1}
    if (j + 1 >= n)
        return y.back();
    if (x[j] == t)
        return y[j];
    const double slope = (y[j + 1] - y[j]) / (x[j + 1] - x[j]);
    return slope * (t - x[j]) + y[j];
}

FineTimeEdgeClass fine_time_edge_class(double informed_share, double rho_before, double rho_after, double limit, const LoopFineTimeRule &rule)
{
    if (informed_share < rule.informed_share)
        return FineTimeEdgeClass::unconfirmed;
    if (std::abs(rho_after) <= limit || std::abs(rho_after) <= std::abs(rho_before))
        return FineTimeEdgeClass::agrees;
    return FineTimeEdgeClass::contradicted;
}

const char *fine_time_edge_class_name(FineTimeEdgeClass c)
{
    switch (c)
    {
    case FineTimeEdgeClass::unconfirmed:
        return "unconfirmed";
    case FineTimeEdgeClass::agrees:
        return "agrees";
    case FineTimeEdgeClass::contradicted:
        return "contradicted";
    default:
        return "";
    }
}

std::vector<std::string> fine_time_apply_rule(std::vector<bool> &in_set,
                                              const std::function<std::vector<std::pair<int, std::string>>(const std::vector<bool> &)> &contradicted)
{
    std::vector<std::string> removed_by(in_set.size());
    for (;;)
    {
        const std::vector<std::pair<int, std::string>> found = contradicted(in_set);
        std::vector<bool> out(in_set.size(), false);
        for (const auto &[run, edge] : found)
            if (run >= 0 && run < static_cast<int>(in_set.size()) && in_set[run])
            {
                out[run] = true;
                removed_by[run] = edge;
            }
        bool any = false;
        for (size_t r = 0; r < in_set.size(); r++)
            if (out[r])
            {
                in_set[r] = false;
                any = true;
            }
        if (!any)
            return removed_by;
    }
}

std::vector<FineTimeSupport> fine_time_supports(const std::vector<std::pair<int, int>> &runs, int windows, const LoopFineTimeRule &rule)
{
    std::vector<FineTimeSupport> out;
    for (const auto &[first, last] : runs)
        out.push_back({std::max(0, first - rule.flank_windows), std::min(windows - 1, last + rule.flank_windows), false});
    for (size_t a = 0; a < out.size(); a++)
        for (size_t b = 0; b < out.size(); b++)
            if (a != b && out[a].first_window <= out[b].last_window && out[b].first_window <= out[a].last_window)
                out[a].overlap = true;
    return out;
}

bool fine_time_curves_fit(const std::vector<LoopFineTimeRun> &eligible, const std::vector<LoopFineTimeCurves> &curves, const LoopFineTimeRule &rule,
                          std::string &why)
{
    const auto bits_equal = [](double a, double b) { return std::memcmp(&a, &b, sizeof(double)) == 0; };
    if (curves.size() != eligible.size())
    {
        why = "the score curves are for " + std::to_string(curves.size()) + " runs, not the " + std::to_string(eligible.size()) + " eligible ones";
        return false;
    }
    for (size_t e = 0; e < eligible.size(); e++)
    {
        const LoopFineTimeRun &run = eligible[e];
        const LoopFineTimeCurves &c = curves[e];
        if (run.overlap)
            continue; // its curves are not read
        const std::string name = "the score curves of the run of windows " + std::to_string(run.first_window) + "-" + std::to_string(run.last_window);
        if (c.support_first != run.support_first || c.support_last != run.support_last ||
            static_cast<int>(c.windows.size()) != run.support_last - run.support_first + 1)
        {
            why = name + " don't cover its support";
            return false;
        }
        double bound = 0.0;
        const std::vector<double> grid = fine_time_offset_grid(run.sigma_m, rule, bound);
        bool same_grid = bits_equal(bound, c.bound_m) && grid.size() == c.grid.size();
        for (size_t k = 0; same_grid && k < grid.size(); k++)
            same_grid = bits_equal(grid[k], c.grid[k]);
        if (!same_grid || std::find(c.grid.begin(), c.grid.end(), 0.0) == c.grid.end())
        {
            why = name + " have another offset grid than its sigma gives";
            return false;
        }
        const size_t G = c.grid.size();
        for (const LoopFineTimeWindowCurves &wc : c.windows)
        {
            const auto sized = [G](const auto &v) { return v.empty() || v.size() == G; };
            if (!sized(wc.training) || !sized(wc.training_matched) || !sized(wc.held_out) || !sized(wc.held_out_matched) ||
                wc.training.empty() != wc.training_matched.empty())
            {
                why = name + " have a window's curve of another length than the grid";
                return false;
            }
        }
    }
    return true;
}

// ---- the replay's source ------------------------------------------------------------------------------------------------

namespace
{
    bool same_run(const LoopFineTimeRun &a, const LoopFineTimeRun &b)
    {
        return a.run == b.run && a.first_window == b.first_window && a.last_window == b.last_window && a.support_first == b.support_first &&
               a.support_last == b.support_last && a.overlap == b.overlap && a.sigma_m == b.sigma_m && a.pin_begin_s == b.pin_begin_s &&
               a.pin_end_s == b.pin_end_s && a.axis_odometry == b.axis_odometry && a.axis == b.axis;
    }

    std::string runs_text(const std::vector<LoopFineTimeRun> &runs)
    {
        std::ostringstream s;
        if (runs.empty())
            s << "none";
        for (size_t k = 0; k < runs.size(); k++)
            s << (k ? ", " : "") << "windows " << runs[k].first_window << "-" << runs[k].last_window;
        return s.str();
    }
} // namespace

bool CapturedFineTimeScores::curves(const LoopFineTimeQuery &query, std::vector<LoopFineTimeCurves> &out, std::string &why)
{
    const std::string remedy = "; a --from-snapshot run with capture_loop_closure computes them from the points";
    if (!capture_)
    {
        why = "the capture holds no fine-time score curves (made with loop_closure_fine_time_placement off, without an eligible run, or in a "
              "format before 4)" +
              remedy;
        return false;
    }
    if (query.segment_poses.size() != capture_->segment_poses.size())
    {
        why = "the captured fine-time score curves belong to another pose graph result (" + std::to_string(query.segment_poses.size()) +
              " segment poses now, " + std::to_string(capture_->segment_poses.size()) + " in the capture)" + remedy;
        return false;
    }
    size_t differing = 0;
    for (size_t k = 0; k < query.segment_poses.size(); k++)
        differing += query.segment_poses[k].matrix() != capture_->segment_poses[k].matrix();
    if (differing > 0)
    {
        why = "the captured fine-time score curves belong to another pose graph result (" + std::to_string(differing) + " of " +
              std::to_string(query.segment_poses.size()) + " segment poses differ from the capturing run's)" + remedy;
        return false;
    }
    bool same = query.eligible.size() == capture_->eligible.size();
    for (size_t k = 0; same && k < query.eligible.size(); k++)
        same = same_run(query.eligible[k], capture_->eligible[k]);
    if (!same)
    {
        why = "the captured fine-time score curves belong to other eligible runs (captured: " + runs_text(capture_->eligible) +
              "; now: " + runs_text(query.eligible) + ")" + remedy;
        return false;
    }
    out = capture_->curves;
    return true;
}

// ---- the step (declared in loop_closure.h) ------------------------------------------------------------------------------

LoopFineTimeRule loop_fine_time_rule(const OdometryConfig &params)
{
    LoopFineTimeRule r;
    r.min_sigma_m = params.loop_closure_fine_time_min_sigma_m;
    r.flank_windows = std::max(1, params.loop_closure_segment_windows);
    return r;
}

std::optional<std::pair<std::string, std::string>> loop_fine_time_refusal(const OdometryConfig &params)
{
    if (params.loop_closure_fine_time_placement && !params.loop_closure_odometry_from_readings)
        return std::make_pair(std::string("fine_time_without_readings"),
                              std::string("loop_closure_fine_time_placement needs loop_closure_odometry_from_readings (it places the weak runs "
                                          "of the readings rule)"));
    return std::nullopt;
}

namespace
{
    // rule 1: a window's start (its first node's time), end (the next window's start; the last window: its last node's
    // time), middle time, and middle node (its first node at or after the middle time; its last node if none)
    struct WindowTimes
    {
        std::vector<double> start, end, middle;
        std::vector<size_t> middle_node;
    };

    size_t first_at_or_after(const Window &w, double t)
    {
        const auto it = std::lower_bound(w.timestamps.begin(), w.timestamps.end(), t,
                                         [](const std::pair<double, double> &s, double v) { return s.first < v; });
        return std::min(static_cast<size_t>(it - w.timestamps.begin()), w.timestamps.size() - 1);
    }

    WindowTimes window_times(const std::vector<Window> &windows)
    {
        WindowTimes t;
        const size_t n = windows.size();
        for (size_t w = 0; w < n; w++)
            t.start.push_back(windows[w].timestamps.front().first);
        for (size_t w = 0; w < n; w++)
        {
            t.end.push_back(w + 1 < n ? t.start[w + 1] : windows[w].timestamps.back().first);
            t.middle.push_back((t.start[w] + t.end[w]) / 2.0);
            t.middle_node.push_back(first_at_or_after(windows[w], t.middle[w]));
        }
        return t;
    }

    // rule 2: the axis turned by the rotation of the correction (pose after times the inverse of the pose before) at the first
    // node at or after the run's middle time (the last node if none)
    Eigen::Vector3d turned_axis(const std::vector<Window> &windows, const std::vector<std::vector<Eigen::Affine3d>> &odometry_poses,
                                const WindowTimes &times, int first_window, int last_window, const Eigen::Vector3d &axis)
    {
        const double t = (times.start[first_window] + times.end[last_window]) / 2.0;
        int w = static_cast<int>(windows.size()) - 1;
        size_t n = windows[w].timestamps.size() - 1;
        for (int k = 0; k < static_cast<int>(windows.size()); k++)
            if (windows[k].timestamps.back().first >= t)
            {
                w = k;
                n = first_at_or_after(windows[k], t);
                break;
            }
        return (windows[w].poses[n] * odometry_poses[w][n].inverse()).linear() * axis;
    }

    std::string edge_name(const LoopClosureEdge &e)
    {
        return std::to_string(e.i) + (e.cross_floor ? " => " : " -> ") + std::to_string(e.j);
    }
} // namespace

bool loop_fine_time_curves_from_points(const LoopFineTimeQuery &query, const std::vector<LoopSegmentPoints> &segment_points,
                                       const std::vector<LoopClosureSegment> &segments, PointStore &point_store, const OdometryConfig &params,
                                       std::vector<LoopFineTimeCurves> &out, std::string &why)
{
    out.clear();
    const LoopFineTimeRule rule = loop_fine_time_rule(params);
    const std::vector<Window> &windows = query.windows;
    const int W = static_cast<int>(windows.size());
    if (segment_points.size() != segments.size())
    {
        why = "the segments' points don't match the segments";
        return false;
    }
    for (const LoopSegmentPoints &s : segment_points)
        if (s.local.size() != s.points.size())
        {
            why = "the segments' points were kept without their nodes' frames";
            return false;
        }
    const WindowTimes times = window_times(windows);
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<Eigen::Vector3d> local;
    std::vector<int> node;
    for (const LoopFineTimeRun &run : query.eligible)
    {
        LoopFineTimeCurves c;
        c.support_first = run.support_first;
        c.support_last = run.support_last;
        if (run.overlap)
        {
            out.push_back(std::move(c));
            continue;
        }
        c.grid = fine_time_offset_grid(run.sigma_m, rule, c.bound_m);
        const Eigen::Vector3d &a = run.axis;
        // rule 5: the support's windows, placed (rule 1), in file order; their box
        std::vector<std::vector<Eigen::Vector3d>> source;
        Eigen::Vector3d lo(inf, inf, inf), hi(-inf, -inf, -inf);
        for (int w = run.support_first; w <= run.support_last; w++)
        {
            loop_window_points(windows[w], w, params, point_store, local, node);
            std::vector<Eigen::Vector3d> x;
            x.reserve(local.size());
            for (size_t k = 0; k < local.size(); k++)
            {
                x.push_back(windows[w].poses[node[k]] * local[k]);
                lo = lo.cwiseMin(x.back());
                hi = hi.cwiseMax(x.back());
            }
            source.push_back(std::move(x));
        }
        // rule 4: the qualifying windows, the reference points in the box, their patches
        std::vector<char> qualifying(W);
        for (int w = 0; w < W; w++)
            qualifying[w] = times.end[w] <= run.pin_begin_s - rule.separation_s || times.start[w] >= run.pin_end_s + rule.separation_s;
        for (const LoopFineTimeRun &e : query.eligible)
            for (int w = e.support_first; w <= e.support_last; w++)
                qualifying[w] = 0;
        const double margin = c.bound_m + rule.box_margin_m;
        const Eigen::Vector3d box_lo(lo.x() - margin, lo.y() - margin, lo.z() - margin);
        const Eigen::Vector3d box_hi(hi.x() + margin, hi.y() + margin, hi.z() + margin);
        std::vector<Eigen::Vector3d> reference;
        for (size_t k = 0; k < segments.size(); k++)
        {
            bool any = false;
            for (int w = segments[k].first_window; w <= segments[k].last_window && !any; w++)
                any = qualifying[w] != 0;
            if (!any)
                continue;
            const LoopSegmentPoints &s = segment_points[k];
            for (size_t p = 0; p < s.points.size(); p++)
            {
                const int w = s.window[p];
                if (!qualifying[w])
                    continue;
                const Eigen::Vector3d x = windows[w].poses[s.node[p]] * s.local[p];
                if (x.x() >= box_lo.x() && x.y() >= box_lo.y() && x.z() >= box_lo.z() && x.x() <= box_hi.x() && x.y() <= box_hi.y() &&
                    x.z() <= box_hi.z())
                    reference.push_back(x);
            }
        }
        const FineTimePatches rp = fine_time_patches(reference, a, rule.reference_voxel_m, rule);
        std::vector<Eigen::Vector3d> X;
        for (const size_t k : rp.point)
            X.push_back(reference[k]);
        c.reference_patches = static_cast<int>(X.size());
        const FineTimeIndex index(X, rule.match_reach_m);
        // rule 5: each window's patches, split by the hash, at most max_patches of each kind, their curves
        for (const std::vector<Eigen::Vector3d> &x : source)
        {
            const FineTimePatches sp = fine_time_patches(x, a, rule.source_voxel_m, rule);
            std::vector<size_t> kinds[2]; // training, held out
            for (size_t k = 0; k < sp.point.size(); k++)
                kinds[fine_time_held_out(x[sp.point[k]], rule) ? 1 : 0].push_back(k);
            LoopFineTimeWindowCurves wc;
            for (int kind = 0; kind < 2; kind++)
            {
                std::vector<Eigen::Vector3d> q, n;
                for (const size_t t : fine_time_take(kinds[kind].size(), static_cast<size_t>(rule.max_patches)))
                {
                    q.push_back(x[sp.point[kinds[kind][t]]]);
                    n.push_back(sp.normal[kinds[kind][t]]);
                }
                (kind == 0 ? wc.training_patches : wc.held_out_patches) = static_cast<int>(q.size());
                if (static_cast<int>(q.size()) < rule.min_patches || X.empty())
                    continue;
                FineTimeCurve curve = fine_time_score_curve(q, n, c.grid, a, index, rp.normal, rule);
                (kind == 0 ? wc.training : wc.held_out) = std::move(curve.score);
                (kind == 0 ? wc.training_matched : wc.held_out_matched) = std::move(curve.matched);
            }
            c.windows.push_back(std::move(wc));
        }
        out.push_back(std::move(c));
    }
    return true;
}

bool PointFineTimeScores::curves(const LoopFineTimeQuery &query, std::vector<LoopFineTimeCurves> &out, std::string &why)
{
    if (!loop_fine_time_curves_from_points(query, segment_points_, segments_, point_store_, params_, out, why))
        return false;
    captured_ = LoopFineTimeCapture{query.segment_poses, query.eligible, out};
    return true;
}

namespace
{
    // one kept edge touching one weak run (rule 7)
    struct Touch
    {
        size_t edge = 0;
        int run = 0;       // into the readings rule's runs
        int eligible = -1; // into the eligible runs, -1 for a run that is not eligible
        bool target = false;
        Eigen::Vector3d axis = Eigen::Vector3d::UnitX(); // a
    };

    void fill_run_statistics(LoopFineTimeRunReport &rr, const WindowTimes &times)
    {
        const LoopFineTimeRun &run = rr.run;
        double gap = 0.0;
        for (int w = run.support_first; w <= run.support_last; w++)
        {
            const size_t j = static_cast<size_t>(w - run.support_first);
            const double off = rr.offsets_m[j];
            rr.largest_offset_m = std::max(rr.largest_offset_m, std::abs(off));
            if (!rr.supported[j] && off != 0.0)
            {
                rr.unsupported_moved++;
                rr.unsupported_largest_m = std::max(rr.unsupported_largest_m, std::abs(off));
            }
            if (w < run.first_window || w > run.last_window)
                continue;
            rr.run_windows++;
            rr.supported_in_run += rr.supported[j] != 0;
            gap = rr.supported[j] ? 0.0 : gap + (times.end[w] - times.start[w]);
            rr.longest_unsupported_s = std::max(rr.longest_unsupported_s, gap);
        }
    }
} // namespace

bool loop_fine_time_step(std::vector<Window> &windows, const std::vector<std::vector<Eigen::Affine3d>> &odometry_poses,
                         const std::vector<LoopClosureSegment> &segments, const std::vector<LoopOdometryRun> &runs,
                         const std::vector<LoopClosureEdge> &kept, const std::vector<const LoopClosureTry *> &kept_tries,
                         const std::vector<Eigen::Affine3d> &segment_poses, const OdometryConfig &params, LoopFineTimeScores &scores, Logger &logger,
                         LoopFineTimeReport &report, std::string &why)
{
    const auto clock_start = std::chrono::steady_clock::now();
    report = LoopFineTimeReport{};
    const LoopFineTimeRule rule = loop_fine_time_rule(params);
    const int W = static_cast<int>(windows.size());
    if (W == 0 || odometry_poses.size() != windows.size() || kept_tries.size() != kept.size())
    {
        why = "the windows or edges given to the fine-time step don't fit";
        return false;
    }
    for (int w = 0; w < W; w++)
        if (windows[w].poses.empty() || windows[w].timestamps.size() != windows[w].poses.size() || odometry_poses[w].size() != windows[w].poses.size())
        {
            why = "window " + std::to_string(w) + " has no nodes or no time for each";
            return false;
        }
    const WindowTimes times = window_times(windows);
    const double t0 = times.start[0];
    report.time_origin_s = t0;

    // rule 2: the eligible runs, their supports and axes
    std::vector<int> eligible_of(runs.size(), -1);
    std::vector<std::pair<int, int>> bounds;
    std::vector<int> eligible_runs;
    for (size_t r = 0; r < runs.size(); r++)
        if (runs[r].sigma_m >= rule.min_sigma_m)
        {
            eligible_of[r] = static_cast<int>(eligible_runs.size());
            eligible_runs.push_back(static_cast<int>(r));
            bounds.emplace_back(runs[r].first_window, runs[r].last_window);
        }
    const std::vector<FineTimeSupport> supports = fine_time_supports(bounds, W, rule);
    std::vector<LoopFineTimeRun> eligible;
    for (size_t e = 0; e < eligible_runs.size(); e++)
    {
        const LoopOdometryRun &r = runs[eligible_runs[e]];
        LoopFineTimeRun f;
        f.run = eligible_runs[e];
        f.first_window = r.first_window;
        f.last_window = r.last_window;
        f.support_first = supports[e].first_window;
        f.support_last = supports[e].last_window;
        f.overlap = supports[e].overlap;
        f.sigma_m = r.sigma_m;
        f.pin_begin_s = times.start[f.support_first];
        f.pin_end_s = times.end[f.support_last];
        f.axis_odometry = r.axis;
        f.axis = turned_axis(windows, odometry_poses, times, r.first_window, r.last_window, r.axis);
        eligible.push_back(f);
    }
    report.ran = true;
    log_info(logger) << "loop closure fine time: " << runs.size() << " weak run(s), " << eligible.size() << " eligible (sigma >= " << rule.min_sigma_m
                     << " m)";
    if (eligible.empty())
    {
        report.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - clock_start).count();
        return true;
    }

    // the curves (rules 4 and 5)
    std::vector<LoopFineTimeCurves> curves;
    const LoopFineTimeQuery query{windows, odometry_poses, segment_poses, eligible};
    if (!scores.curves(query, curves, why))
        return false;
    if (!fine_time_curves_fit(eligible, curves, rule, why))
        return false;

    // rules 5 and 6 and conditions 1 to 3 of rule 8
    std::vector<bool> in_set(eligible.size(), false);
    for (size_t e = 0; e < eligible.size(); e++)
    {
        LoopFineTimeRunReport rr;
        rr.run = eligible[e];
        rr.largest_axis_angle_deg = runs[eligible[e].run].largest_axis_angle_deg;
        rr.status = "not timed";
        for (int w = rr.run.support_first; w <= rr.run.support_last; w++)
        {
            rr.start_s.push_back(times.start[w]);
            rr.middle_s.push_back(times.middle[w]);
        }
        if (rr.run.overlap)
        {
            rr.cause = "supports overlap";
            report.runs.push_back(std::move(rr));
            continue;
        }
        rr.scored = true;
        rr.curves = curves[e];
        const LoopFineTimeCurves &c = rr.curves;
        const size_t n = c.windows.size();
        std::vector<std::vector<double>> E(n, std::vector<double>(c.grid.size(), 0.0));
        bool own_supported = false;
        for (size_t j = 0; j < n; j++)
        {
            const LoopFineTimeWindowCurves &wc = c.windows[j];
            const bool supported =
                !wc.training.empty() && *std::max_element(wc.training_matched.begin(), wc.training_matched.end()) >= rule.min_matched;
            rr.supported.push_back(supported);
            if (supported)
                E[j] = wc.training;
            const int w = rr.run.support_first + static_cast<int>(j);
            own_supported = own_supported || (supported && w >= rr.run.first_window && w <= rr.run.last_window);
        }
        rr.offset_index = fine_time_profile(E, c.grid, rule);
        for (const int k : rr.offset_index)
            rr.offsets_m.push_back(c.grid[k]);
        const int zero = zero_index(c.grid);
        std::vector<double> before, after;
        for (size_t j = 0; j < n; j++)
            if (!c.windows[j].held_out.empty())
            {
                before.push_back(c.windows[j].held_out[zero]);
                after.push_back(c.windows[j].held_out[rr.offset_index[j]]);
            }
        rr.held_out_windows = static_cast<int>(before.size());
        if (!before.empty())
        {
            rr.held_out_before = mean_of(before);
            rr.held_out_after = mean_of(after);
        }
        fill_run_statistics(rr, times);
        const bool moved = std::any_of(rr.offsets_m.begin(), rr.offsets_m.end(), [](double d) { return d != 0.0; });
        if (!own_supported)
            rr.cause = "no surface across the passage seen by another visit";
        else if (!moved)
        {
            rr.status = "unchanged";
            rr.cause = "the profile is zero";
        }
        else if (before.empty())
            rr.cause = "no held-out score";
        else if (!(rr.held_out_after < rr.held_out_before))
            rr.cause = "held-out not better";
        else
        {
            rr.status = "candidate";
            in_set[e] = true;
        }
        report.runs.push_back(std::move(rr));
    }

    // rule 7: every kept edge touching a run, along that run's axis
    std::vector<Touch> touches;
    for (size_t k = 0; k < kept.size(); k++)
    {
        const LoopClosureSegment &source = segments[kept[k].j];
        const int target_window = segments[kept[k].i].first_window;
        for (size_t r = 0; r < runs.size(); r++)
        {
            const int e = eligible_of[r];
            const int lo = e >= 0 ? eligible[e].support_first : runs[r].first_window;
            const int hi = e >= 0 ? eligible[e].support_last : runs[r].last_window;
            const bool on_source = source.first_window <= hi && lo <= source.last_window;
            const bool on_target = e >= 0 && eligible[e].support_first <= target_window && target_window <= eligible[e].support_last;
            if (!on_source && !on_target)
                continue;
            Touch t;
            t.edge = k;
            t.run = static_cast<int>(r);
            t.eligible = e;
            t.target = !on_source;
            t.axis = e >= 0 ? eligible[e].axis : turned_axis(windows, odometry_poses, times, runs[r].first_window, runs[r].last_window, runs[r].axis);
            touches.push_back(t);
        }
    }
    const auto offset_of = [&](const LoopFineTimeRunReport &rr, double t)
    { return fine_time_offset_at(t, rr.run.pin_begin_s, rr.run.pin_end_s, rr.middle_s, rr.offsets_m); };
    // the nodes' positions with the profiles of the runs in the set
    const auto positions_after = [&](const std::vector<bool> &set)
    {
        std::vector<std::vector<Eigen::Vector3d>> p(W);
        for (int w = 0; w < W; w++)
            for (const Eigen::Affine3d &pose : windows[w].poses)
                p[w].push_back(pose.translation());
        for (size_t e = 0; e < eligible.size(); e++)
        {
            if (!set[e])
                continue;
            const LoopFineTimeRunReport &rr = report.runs[e];
            const Eigen::Vector3d &a = rr.run.axis;
            for (int w = rr.run.support_first; w <= rr.run.support_last; w++)
                for (size_t n = 0; n < p[w].size(); n++)
                {
                    const double d = offset_of(rr, windows[w].timestamps[n].first);
                    p[w][n] = Eigen::Vector3d(p[w][n].x() + d * a.x(), p[w][n].y() + d * a.y(), p[w][n].z() + d * a.z());
                }
        }
        return p;
    };
    // rule 7: a window is blind when it lies in an eligible run's support (any of them) and is unsupported there; a run
    // without scores (its support overlaps another's) has no supported window
    std::vector<char> blind_window(W, 0);
    for (const LoopFineTimeRunReport &rr : report.runs)
        for (int w = rr.run.support_first; w <= rr.run.support_last; w++)
            blind_window[w] = 1;
    for (const LoopFineTimeRunReport &rr : report.runs)
        if (rr.scored)
            for (int w = rr.run.support_first; w <= rr.run.support_last; w++)
                if (rr.supported[w - rr.run.support_first])
                    blind_window[w] = 0;
    std::vector<char> stopped(touches.size(), 0);
    const auto read_edges = [&](const std::vector<bool> &set)
    {
        const std::vector<std::vector<Eigen::Vector3d>> after = positions_after(set);
        std::vector<std::pair<int, std::string>> contradicted;
        report.edges.clear();
        for (size_t ti = 0; ti < touches.size(); ti++)
        {
            const Touch &t = touches[ti];
            const LoopClosureEdge &e = kept[t.edge];
            const LoopClosureSegment &source = segments[e.j];
            const int count = source.last_window - source.first_window + 1;
            const LoopClosureTry *tried = kept_tries[t.edge];
            const std::vector<Eigen::Matrix3d> none;
            const std::vector<Eigen::Matrix3d> &blocks =
                tried && static_cast<int>(tried->window_information.size()) == count ? tried->window_information : none;
            const int wi = segments[e.i].first_window;
            const Eigen::Affine3d K0 = windows[wi].poses[0] * odometry_poses[wi][0].inverse();
            Eigen::Affine3d moved = windows[wi].poses[0];
            moved.translation() = after[wi][0];
            const Eigen::Affine3d K1 = moved * odometry_poses[wi][0].inverse();
            std::vector<Eigen::Vector3d> middle_odometry, middle_before, middle_after;
            for (int w = source.first_window; w <= source.last_window; w++)
            {
                const size_t n = times.middle_node[w];
                middle_odometry.push_back(odometry_poses[w][n].translation());
                middle_before.push_back(windows[w].poses[n].translation());
                middle_after.push_back(after[w][n]);
            }
            const Eigen::Vector3d &a0 = runs[t.run].axis;
            const Eigen::Vector3d first_odometry = odometry_poses[source.first_window][0].translation();
            const Eigen::Vector3d first_before = windows[source.first_window].poses[0].translation();
            const LoopEdgeTimeSupport b =
                loop_edge_time_support(blocks, a0, t.axis, K0, e.correction, first_odometry, first_before, middle_odometry, middle_before);
            const LoopEdgeTimeSupport s =
                loop_edge_time_support(blocks, a0, t.axis, K1, e.correction, first_odometry, first_before, middle_odometry, middle_after);
            LoopFineTimeEdgeRow row;
            row.i = e.i;
            row.j = e.j;
            row.cross_floor = e.cross_floor;
            row.run = t.run;
            row.eligible = t.eligible >= 0;
            row.target = t.target;
            row.first_window = source.first_window;
            row.last_window = source.last_window;
            row.first_pose_m = b.first_pose_residual_m;
            row.rho_before_m = b.residual_m;
            row.rho_after_m = s.residual_m;
            row.shares = b.shares;
            // the sigma along the axis with the edge's other components held: (u' W u)^(-1/2), u = R_i' a0
            const Eigen::Matrix3d Wt =
                e.information ? Eigen::Matrix3d(e.omega.topLeftCorner<3, 3>())
                              : Eigen::Matrix3d(Eigen::Matrix3d::Identity() / (params.loop_closure_edge_sigma_m * params.loop_closure_edge_sigma_m));
            const Eigen::Vector3d u = segments[e.i].pose.linear().transpose() * a0;
            double h = 0.0;
            for (int c = 0; c < 3; c++)
                h += ((u(0) * Wt(0, c) + u(1) * Wt(1, c)) + u(2) * Wt(2, c)) * u(c);
            row.sigma_a_m = h > 0.0 ? std::pow(h, -0.5) : std::numeric_limits<double>::infinity();
            row.limit_m = row.sigma_a_m * params.loop_closure_max_residual_m / params.loop_closure_edge_sigma_m;
            if (t.eligible >= 0 && report.runs[t.eligible].scored)
            {
                if (!b.along_axis)
                    row.cls = FineTimeEdgeClass::unconfirmed;
                else
                {
                    std::vector<double> informed;
                    for (int w = source.first_window; w <= source.last_window; w++)
                    {
                        if (!blind_window[w])
                            informed.push_back(b.shares[w - source.first_window]);
                    }
                    row.informed_share = fine_time_pairwise_sum(informed.data(), informed.size());
                    row.cls = fine_time_edge_class(row.informed_share, row.rho_before_m, row.rho_after_m, row.limit_m, rule);
                }
                if (row.cls == FineTimeEdgeClass::contradicted && set[t.eligible])
                {
                    contradicted.emplace_back(t.eligible, edge_name(e));
                    stopped[ti] = 1;
                }
            }
            row.stopped_run = stopped[ti] != 0;
            report.edges.push_back(row);
        }
        return contradicted;
    };

    // rule 8: candidates leave the set while an edge contradicts them; the rest are applied
    const std::vector<std::string> removed_by = fine_time_apply_rule(in_set, read_edges);
    for (size_t e = 0; e < eligible.size(); e++)
    {
        LoopFineTimeRunReport &rr = report.runs[e];
        if (rr.status != "candidate")
            continue;
        if (in_set[e])
        {
            rr.status = "applied";
            rr.cause.clear();
        }
        else
        {
            rr.status = "not applied";
            rr.cause = "edge " + removed_by[e] + " contradicted";
        }
    }
    const std::vector<std::vector<Eigen::Vector3d>> placed = positions_after(in_set);
    for (size_t e = 0; e < eligible.size(); e++)
    {
        if (!in_set[e])
            continue;
        const LoopFineTimeRun &run = report.runs[e].run;
        for (int w = run.support_first; w <= run.support_last; w++)
        {
            for (size_t n = 0; n < windows[w].poses.size(); n++)
                windows[w].poses[n].translation() = placed[w][n];
            windows[w].motion_model_poses = windows[w].poses;
        }
    }

    // the log (design D8)
    for (const LoopFineTimeRunReport &rr : report.runs)
    {
        const LoopFineTimeRun &r = rr.run;
        auto line = log_info(logger);
        line << "loop closure fine time: run windows " << r.first_window << "-" << r.last_window << " (t " << std::fixed << std::setprecision(2)
             << times.start[r.first_window] - t0 << "-" << times.end[r.last_window] - t0 << " s), support " << r.support_first << "-" << r.support_last
             << ", axis " << std::setprecision(3) << r.axis.x() << " " << r.axis.y() << " " << r.axis.z() << ", largest axis angle "
             << std::setprecision(1) << rr.largest_axis_angle_deg << " deg, sigma " << std::setprecision(2) << r.sigma_m << " m: " << rr.status;
        if (!rr.cause.empty())
            line << " (" << rr.cause << ")";
        if (rr.scored)
        {
            line << "; " << rr.curves.reference_patches << " reference patches, supported " << rr.supported_in_run << " of " << rr.run_windows
                 << " run windows, longest unsupported " << rr.longest_unsupported_s << " s, largest offset " << std::setprecision(3)
                 << rr.largest_offset_m << " m, " << rr.unsupported_moved << " unsupported moved (largest " << rr.unsupported_largest_m
                 << " m), held-out ";
            if (rr.held_out_windows > 0)
                line << std::setprecision(4) << std::sqrt(rr.held_out_before) << " -> " << std::sqrt(rr.held_out_after) << " m (RMS over "
                     << rr.held_out_windows << " windows)";
            else
                line << "none";
        }
    }
    for (const LoopFineTimeEdgeRow &row : report.edges)
    {
        const LoopOdometryRun &r = runs[row.run];
        auto line = log_info(logger);
        line << "loop closure fine time edge " << row.i << (row.cross_floor ? " => " : " -> ") << row.j << " (" << (row.target ? "target" : "source")
             << ", run " << r.first_window << "-" << r.last_window << "): first pose " << std::fixed << std::setprecision(3) << row.first_pose_m
             << " m, time support ";
        if (std::isfinite(row.rho_before_m))
            line << row.rho_before_m << " -> " << row.rho_after_m << " m";
        else
            line << "none (no information along the axis)";
        line << ", limit " << row.limit_m << " m, informed ";
        if (std::isfinite(row.informed_share))
            line << std::setprecision(2) << row.informed_share;
        else
            line << "-";
        line << ", " << (row.cls == FineTimeEdgeClass::none ? "no class (run without scores)" : fine_time_edge_class_name(row.cls));
        if (row.stopped_run)
            line << " (it contradicted the run's profile and stopped it; read here without that profile)";
    }
    report.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - clock_start).count();
    return true;
}
