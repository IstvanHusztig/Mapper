#pragma once

// Fine-time placement of a blind stretch (change loop-closure-fine-time-guard, the rule frozen in
// experiments/consistency/livox4_lever/fine_time/RULE.md, ported from the prototype tools/loop_fine_time.py; "rule n" below is
// section n of RULE.md). The loop closure's last step: the windows of a weak run of the readings rule (and one segment's
// windows on either side, the support) are compared with the surfaces across the run's axis that windows at least 12 s
// away saw, over a grid of offsets along the axis; a smooth profile of offsets, zero at both ends, is fitted by dynamic
// programming and applied to the nodes' positions only when the held-out scans agree and no loop edge contradicts it.
//
// This header holds the pure parts (each tested on its own in tests/loop_fine_time_test.cpp), the port through which
// close_loops gets the score curves (the run computes them from the points, a replay takes them from the capture), and
// the report. The step itself (loop_fine_time_step) and the curves from the points are declared in loop_closure.h.
//
// Every sum that the prototype makes with numpy over an array (a mean of squared distances, a held-out mean, an informed
// share) is made here in numpy's order (fine_time_pairwise_sum), so that the scores agree with the prototype's to the bit
// where the inputs do; what still differs is named where it happens (the scatter matrices and their eigenvectors, the
// order among neighbours at equal distance).

#include "window.h"

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// The constants of RULE.md (design D10). min_sigma_m and flank_windows come from the settings (loop_fine_time_rule in
// loop_closure.h: loop_closure_fine_time_min_sigma_m, loop_closure_segment_windows); the others are frozen.
struct LoopFineTimeRule
{
    double min_sigma_m = 1.0;        // rule 2: eligible runs
    int flank_windows = 6;           // rule 2: windows before and after a run in its support
    double separation_s = 12.0;      // rule 4: a reference window ends this long before the support or starts this long after
    int neighbours = 14;             // rule 3: a patch's neighbourhood, itself included
    double neighbour_reach_m = 0.65; // rule 3: the 14th neighbour nearer than this
    double flat = 0.12;              // rule 3: l0 < flat l1
    double wide = 0.08;              // rule 3: l1 > wide l2
    double across = 0.5;             // rule 3: |normal . axis| >= across
    double reference_voxel_m = 0.10; // rule 4
    double source_voxel_m = 0.07;    // rule 5
    double box_margin_m = 1.5;       // rule 4: the reference box beyond the source points' box and the bound
    double bound_sigmas = 1.5;       // rule 5: bound = min(bound_sigmas sigma, bound_max_m)
    double bound_max_m = 8.0;
    double grid_m = 0.1;             // rule 5: the offsets' step
    double holdout_cell_m = 1.0;     // rule 5: the held-out hash's cells
    int holdout_one_in = 5;          // rule 5: h == 0 of h = hash mod holdout_one_in is held out
    int max_patches = 256;           // rule 5: of each kind
    int min_patches = 8;             // rule 5: a kind with fewer has no score
    int match_neighbours = 4;        // rule 5: the nearest reference patches looked at
    double match_reach_m = 0.5;      // rule 5: ... within this distance; an unmatched patch counts this distance
    double normal_agreement = 0.80;  // rule 5: |m . n| >= normal_agreement
    int min_matched = 8;             // rule 5: matched training patches at one offset at least for a supported window
    double coupling = 0.10;          // rule 6
    double cost_tolerance = 1e-12;   // rule 1: the dynamic programming's comparison
    double informed_share = 0.5;     // rule 7: below it an edge is unconfirmed
};

// ---- the pure parts ---------------------------------------------------------------------------------------------------

// numpy's order of a sum over a contiguous array (pairwise summation: below 8 elements one after the other from 0, up to
// 128 in eight running sums, above that the two halves split at a multiple of 8 and added)
double fine_time_pairwise_sum(const double *values, std::size_t n);

// rule 1: the indices of the points kept when thinned to voxels of side v (key floor(coordinate / v) per axis): the first
// point of each voxel in input order, the kept ones in input order
std::vector<std::size_t> fine_time_thin(const std::vector<Eigen::Vector3d> &points, double v);

// Exact nearest neighbours on a grid of cubic cells. nearest: of the points nearer than `reach` (Euclidean distance as
// sqrt(((0 + dx^2) + dy^2) + dz^2), the form scipy's cKDTree computes), the k nearest by (distance, index): ties at equal
// distance go to the lower index (scipy's order among equal distances is not specified). Returns (squared distance, index)
// pairs in that order.
class FineTimeIndex
{
public:
    FineTimeIndex(const std::vector<Eigen::Vector3d> &points, double cell);
    void nearest(const Eigen::Vector3d &q, int k, double reach, std::vector<std::pair<double, std::size_t>> &out) const;
    std::size_t size() const { return points_.size(); }

private:
    // a cell by its exact coordinates (no two cells share a list, so a query reads each list once without a visited set)
    struct Cell
    {
        std::int64_t x, y, z;
        bool operator==(const Cell &o) const { return x == o.x && y == o.y && z == o.z; }
    };
    struct CellHash
    {
        std::size_t operator()(const Cell &c) const;
    };
    std::vector<Eigen::Vector3d> points_;
    double cell_;
    std::unordered_map<Cell, std::vector<std::size_t>, CellHash> cells_;
};

// rule 3: the patches of a point set (thinned to v first): the indices into `points` of the patches' points and their unit
// normals, in input order. Fewer than 16 kept points: none. The scatter matrix is summed over the 14 neighbours in their
// order and decomposed by Eigen's SelfAdjointEigenSolver; numpy's matmul and eigh may round differently, which matters
// only for a point whose planarity or angle lies at a limit to the last bit.
struct FineTimePatches
{
    std::vector<std::size_t> point;
    std::vector<Eigen::Vector3d> normal;
};
FineTimePatches fine_time_patches(const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &axis, double v, const LoopFineTimeRule &rule);

// rule 5: held out by the spatial hash of the position (1 m cells, (73856093 cx + 19349663 cy + 83492791 cz) mod 5 == 0,
// 64-bit integers, the non-negative remainder)
bool fine_time_held_out(const Eigen::Vector3d &q, const LoopFineTimeRule &rule);

// rule 5: the indices taken of n patches when at most `limit` are used: all, or floor(i (n - 1) / (limit - 1)), i < limit
std::vector<std::size_t> fine_time_take(std::size_t n, std::size_t limit);

// rule 5: the offsets s_k = k grid_m, k = -floor(bound / grid_m) .. floor(bound / grid_m), bound = min(1.5 sigma, 8 m)
std::vector<double> fine_time_offset_grid(double sigma_m, const LoopFineTimeRule &rule, double &bound_m);

// rule 5 for one kind of patches of one window: per offset the mean squared distance to the reference (an unmatched patch
// counts match_reach_m) and how many patches were matched
struct FineTimeCurve
{
    std::vector<double> score;
    std::vector<int> matched;
};
FineTimeCurve fine_time_score_curve(const std::vector<Eigen::Vector3d> &q, const std::vector<Eigen::Vector3d> &n, const std::vector<double> &grid,
                                    const Eigen::Vector3d &axis, const FineTimeIndex &reference, const std::vector<Eigen::Vector3d> &reference_normals,
                                    const LoopFineTimeRule &rule);

// rule 6: the grid indices in the order of preference among equal costs: smaller |d| first, of two with the same |d| the
// negative first
std::vector<int> fine_time_preference(const std::vector<double> &grid);

// rule 6: the profile, the grid index of every window. E: per window its score per offset, all zeros for an unsupported
// window. Exact dynamic programming from the first window to the last with both ends pinned to 0.
std::vector<int> fine_time_profile(const std::vector<std::vector<double>> &E, const std::vector<double> &grid, const LoopFineTimeRule &rule);

// the offset one window's curve prefers on its own (a reading, not part of the rule)
double fine_time_own_offset(const std::vector<double> &score, const std::vector<double> &grid, const LoopFineTimeRule &rule);

// rule 6 on a node: linear interpolation in time through (pin_begin, 0), (each window's middle time, its offset),
// (pin_end, 0), numpy.interp's arithmetic; 0 outside the pins
double fine_time_offset_at(double t, double pin_begin, double pin_end, const std::vector<double> &middles, const std::vector<double> &offsets);

// rule 7: the class of an edge that touches an eligible run with scores
enum class FineTimeEdgeClass
{
    none, // the run has no scores (not eligible, or its support overlaps another's)
    unconfirmed,
    agrees,
    contradicted
};
FineTimeEdgeClass fine_time_edge_class(double informed_share, double rho_before, double rho_after, double limit, const LoopFineTimeRule &rule);
const char *fine_time_edge_class_name(FineTimeEdgeClass c);

// rule 8's iteration: starting with every candidate in the set, `contradicted(set)` reads the edges with the profiles of
// the runs in the set and returns, in its table's order, (run, edge name) for every contradicted reading of a run in the
// set; those runs leave the set, until none is contradicted. Returns per run the name of the edge that removed it (the
// last such reading of its round), empty for a run that stayed or never was a candidate.
std::vector<std::string> fine_time_apply_rule(std::vector<bool> &in_set,
                                              const std::function<std::vector<std::pair<int, std::string>>(const std::vector<bool> &)> &contradicted);

// rule 2: the support of a run (its windows and flank_windows on either side, cut at the ends) and whether the supports of
// two eligible runs share a window. runs: (first window, last window) of each eligible run.
struct FineTimeSupport
{
    int first_window = 0;
    int last_window = 0;
    bool overlap = false;
};
std::vector<FineTimeSupport> fine_time_supports(const std::vector<std::pair<int, int>> &runs, int windows, const LoopFineTimeRule &rule);

// ---- the port: score curves of the eligible runs' supports ------------------------------------------------------------

// an eligible run (rule 2)
struct LoopFineTimeRun
{
    int run = 0; // its index among the readings rule's runs (LoopClosureReport::odometry_runs)
    int first_window = 0;
    int last_window = 0;
    int support_first = 0;
    int support_last = 0;
    bool overlap = false;    // its support shares a window with another eligible run's: not treated
    double sigma_m = 0.0;
    double pin_begin_s = 0.0; // the support's first window's start and last window's end
    double pin_end_s = 0.0;
    Eigen::Vector3d axis_odometry = Eigen::Vector3d::UnitX(); // a0, the readings rule's axis (odometry frame)
    Eigen::Vector3d axis = Eigen::Vector3d::UnitX();          // a = R a0, turned by the correction at the run's middle
};

// one window's curves (rule 5); an empty score: that kind has no score (fewer than min_patches patches, or no reference)
struct LoopFineTimeWindowCurves
{
    int training_patches = 0; // after the limit of max_patches
    int held_out_patches = 0;
    std::vector<double> training;
    std::vector<int> training_matched;
    std::vector<double> held_out;
    std::vector<int> held_out_matched;
};

// the curves of one eligible run's support; for a run whose support overlaps another's, windows is empty
struct LoopFineTimeCurves
{
    int support_first = 0;
    int support_last = 0;
    double bound_m = 0.0;
    std::vector<double> grid;
    int reference_patches = 0;
    std::vector<LoopFineTimeWindowCurves> windows; // support_first .. support_last
};

// what close_loops asks with: the windows after the node correction, their poses before it, the pose graph's segment poses
// (what the curves belong to) and the eligible runs
struct LoopFineTimeQuery
{
    const std::vector<Window> &windows;                               // poses after the node correction
    const std::vector<std::vector<Eigen::Affine3d>> &odometry_poses; // each window's poses before it
    const std::vector<Eigen::Affine3d> &segment_poses;
    const std::vector<LoopFineTimeRun> &eligible;
};

class LoopFineTimeScores
{
public:
    virtual ~LoopFineTimeScores() = default;
    // one LoopFineTimeCurves per eligible run, in order; false (and `why` says why) if this source can't give them (a
    // capture without curves, or with those of another pose graph result or another set of eligible runs)
    virtual bool curves(const LoopFineTimeQuery &query, std::vector<LoopFineTimeCurves> &out, std::string &why) = 0;
};

// whether the curves can be read for the eligible runs (whatever their source): one per run; for each run not overlapping
// another, its support's windows, the offset grid and bound its sigma gives (bit for bit, with an exact 0), and every
// window's curves empty or of the grid's length. false (and `why` names what doesn't fit) otherwise.
bool fine_time_curves_fit(const std::vector<LoopFineTimeRun> &eligible, const std::vector<LoopFineTimeCurves> &curves, const LoopFineTimeRule &rule,
                          std::string &why);

// what a capture keeps of the step (capture format 4): the curves with the pose graph result and the eligible runs they
// were computed for
struct LoopFineTimeCapture
{
    std::vector<Eigen::Affine3d> segment_poses;
    std::vector<LoopFineTimeRun> eligible;
    std::vector<LoopFineTimeCurves> curves;
};

// the replay's source: the captured curves, only for the captured pose graph result (bit for bit) and the same eligible runs
class CapturedFineTimeScores : public LoopFineTimeScores
{
public:
    explicit CapturedFineTimeScores(const LoopFineTimeCapture *capture) : capture_(capture) {}
    bool curves(const LoopFineTimeQuery &query, std::vector<LoopFineTimeCurves> &out, std::string &why) override;

private:
    const LoopFineTimeCapture *capture_;
};

// ---- the report (design D8) -------------------------------------------------------------------------------------------

struct LoopFineTimeRunReport
{
    LoopFineTimeRun run;
    double largest_axis_angle_deg = 0.0; // between a weak window's weak axis and the run's axis
    std::string status;                  // applied, unchanged, not timed, not applied
    std::string cause;                   // why not applied / not timed / unchanged ("" when applied)
    bool scored = false;                 // the curves were computed (not for overlapping supports)
    LoopFineTimeCurves curves;
    std::vector<int> offset_index;       // per window of the support, into curves.grid
    std::vector<double> offsets_m;
    std::vector<char> supported;
    std::vector<double> start_s, middle_s; // per window of the support (seconds as the windows' node times)
    double held_out_before = std::numeric_limits<double>::quiet_NaN(); // mean held-out score [m^2] at 0 / at the profile
    double held_out_after = std::numeric_limits<double>::quiet_NaN();
    int held_out_windows = 0;
    int supported_in_run = 0;
    int run_windows = 0;
    double longest_unsupported_s = 0.0;
    double largest_offset_m = 0.0;
    int unsupported_moved = 0;
    double unsupported_largest_m = 0.0;
};

struct LoopFineTimeEdgeRow
{
    int i = 0;
    int j = 0;
    bool cross_floor = false;
    int run = 0;          // index into LoopClosureReport::odometry_runs
    bool eligible = false;
    bool target = false;  // touches the run only with its target's first node
    int first_window = 0; // segment j's windows (the shares' order)
    int last_window = 0;
    double first_pose_m = 0.0;
    double rho_before_m = std::numeric_limits<double>::quiet_NaN();
    double rho_after_m = std::numeric_limits<double>::quiet_NaN();
    double limit_m = 0.0;
    double sigma_a_m = 0.0;
    double informed_share = std::numeric_limits<double>::quiet_NaN();
    FineTimeEdgeClass cls = FineTimeEdgeClass::none;
    // contradicted a run in the set in some round of rule 8 and so removed it; the table is the last round's (as the
    // prototype's), where that run's profile is no longer in the set, so its class there can read "agrees"
    bool stopped_run = false;
    std::vector<double> shares; // empty: no information along the axis
};

struct LoopFineTimeReport
{
    bool ran = false; // the step ran (the setting on and the loop closure applied)
    std::vector<LoopFineTimeRunReport> runs; // the eligible runs, in order
    std::vector<LoopFineTimeEdgeRow> edges;  // rule 7's table, as of the last round of rule 8
    double seconds = 0.0;                    // the step's time (curves included)
    double time_origin_s = 0.0;              // the first window's first node (the CSV's times count from it)
};
