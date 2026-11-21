#pragma once

// The loop on synthetic recordings (openspec change consistency-harness, design D8, task 6.1): a route's recording cut
// by the real front end (cut_into_windows) and run through optimize_windows with a keeping WindowTermSink and
// log_window_terms on. Per window: the truth (aligned once, at the first retained node, which is conditioned on: D5), the
// record, its class by the truth, its errors; per run: the preconditions of D8, each computed with a tolerance declared
// here BEFORE any run (LoopTolerances), and listed when it fails. A run whose preconditions do not all hold is marked
// (LoopRun::valid() false, failed() lists them) and must not be used silently. Test helpers only.
//
// R2's outbound landmark cells are designated by key before the run, from the generator's hits (the truth), not from any
// result: the indoor grid's cells of the latent points that the outbound leg hit in room A, minus every cell the return leg
// hit before the nominated exit window (what it could have mapped itself). The return matches are computed by the test from
// the exit window's points, placed with its solved poses, against the map as it was before the exit window (taken through
// the window capture port, which records the loop's state before a range: run_odometry.cpp), because the loop's
// translation matrix and match counts carry no identity of the visit.

#include "synthetic_routes.h"
#include "window_term_sink.h"

#include <Eigen/Dense>

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace synthetic
{
// The stage 8 candidate set (design D8): settings/livox-vertical.toml with use_exact_ndt_bucket_statistics true,
// degenerate_projection_ratio 0.02 / _full 0.04, imu_acceleration_offset_rw_mps2_sqrt_s 0.01; the front end's fields of
// front_end_config(sensor) (the cut must match the generator's prediction); log_window_terms true; and, for the test,
// real_time_threshold_seconds 1000 (the result must not depend on the machine; a window that still reaches the limit
// invalidates the run). `switches` sets imu_edges_follow_state and imu_carry_joint_covariance.
OdometryConfig loop_candidate_config(const SensorProfile &sensor, bool switches = false);

// ---- one sensor model (task 6.1a (1)) ----

// the accelerometer span at which the generator's white density gives the term's shipped weight (design D4: a term's
// variance is (density^2 / tau + floor^2) tau^2; white samples of density D = sqrt(density^2 + floor^2 tau0) match it at
// tau0 only, as ImuNoise documents)
constexpr double kAccelDensitySpan_s = 0.011;

// The world's IMU errors as the run's settings claim them: gyro white noise gyro_arw_{x,y,z}_deg_sqrt_s, gyro bias walk
// gyro_bias_rw_deg_s_sqrt_s, acceleration offset walk imu_acceleration_offset_rw_mps2_sqrt_s, accelerometer density the
// generator's effective one sqrt(imu_accel_noise_density^2 + imu_accel_floor_sigma^2 kAccelDensitySpan_s); initial bias
// and offset zero.
ImuNoise imu_noise_from_config(const OdometryConfig &config);

// The world a loop recording is drawn from, declared with the run (its values are a precondition of the run):
//  - matched: imu_noise_from_config of the run's settings (the main set);
//  - noise_free: ImuNoise::none() (the control: the run's settings still claim their noise; the precondition is that the
//    world is noise-free);
//  - default_mismatch: ImuNoise{}'s defaults (the code's built-in values, the world of task 6.1's first runs), a labelled
//    model-mismatch case: its precondition "world matches settings" fails by design, so the run is never valid.
enum class LoopWorld
{
    matched,
    noise_free,
    default_mismatch
};
const char *loop_world_name(LoopWorld w);
ImuNoise loop_world_noise(LoopWorld w, const OdometryConfig &config);
// generate_recording with loop_world_noise(world, config)
InertialRecording generate_loop_recording(const Route &route, const SensorProfile &sensor, const OdometryConfig &config, LoopWorld world);

// The loop's sensor for route 1, 2 or 3 (task 6.1a), seed 1:
//  - R1 and R3: the routes' profile with 1 cm jitter along the face normal. With the routes' 3 mm (below K9's 3.162 mm) the
//    exact cells of axis-aligned planes are skipped and the LiDAR sees only edges and corners: measured 0 accepted matches
//    at R2's exit. loop_chains_test uses 1 cm for the same reason.
//  - R2: the same, with 6 m range (the corridor's middle has no face within range) and 40 points per node while moving.
//    With the routes' 10 the blind stretch is not blind along travel by the truth (measured 2026-11-21 with observe_loop:
//    freshly mapped cells hold few points, their normals tilt with the 1 cm jitter, and the along-travel share of the
//    truth-placed information is 0.021 to 0.032 outbound, partly kept by the projection; 14 of 28 blind windows removed);
//    with 20 points 25 of 28, with 40 all 28 (share 0.012 to 0.016 outbound, 0.003 to 0.006 on the return).
SensorProfile loop_sensor(int route);
// The loop's route 1, 2 or 3: R1 and R3 with along_structure (loop_scene_along_structure, task 6.1a), R2 as designed.
Route loop_route(int route, bool along_structure = true);

// The shipped Livox settings for a loop run (task 6.2's second set, design D8): settings/livox-vertical.toml as loaded,
// with only real_time_threshold_seconds 1000 and log_window_terms true set, and the front end's fields of
// front_end_config(sensor) copied in (the cut must be the generator's). `changed` receives one line per field whose value
// the copy actually changed (with the file's and the test's value); the two test fields are always listed.
OdometryConfig loop_shipped_config(const SensorProfile &sensor, std::vector<std::string> *changed = nullptr);

// a window's class, by the truth (D8 "per class of window"): the first that applies, in this order
enum class WindowClass
{
    rest,  // the body does not move over the window's nodes
    turn,  // the true heading (without the gait's twist) changes faster than LoopTolerances::turn_rate_rad_s over the window
    exit,  // R2: a window nominated by the generator's hits as the first to see a room's end walls after a blind stretch
    blind, // R2: every node's true position inside [blind_x_lo, blind_x_hi]
    seen   // everything else
};
constexpr int kWindowClasses = 5;
const char *window_class_name(WindowClass c);

// What the LiDAR can tell along travel (task 6.1a (2)), by the truth: the window's points placed with the TRUE poses,
// matched to an indoor map built from truth (the run's own seed, then every earlier window's points at their true poses,
// with the run's grid settings and update_rgd, as the loop builds its map), through the production term
// (sum_ndt_grid_blocks, window_translation_information). The share u^T I u / lambda_max(I) along the true travel
// direction u (first to last node) against the run's projection thresholds:
enum class ObservabilityClass
{
    removed,     // share < degenerate_projection_ratio: the projection removes the LiDAR's information along travel
    partly_kept, // share < degenerate_projection_ratio_full: scaled down
    kept_whole,  // share >= degenerate_projection_ratio_full
    not_moving   // the body travels less than LoopTolerances::observability_min_travel_m over the window: no direction
};
constexpr int kObservabilityClasses = 4;
const char *observability_class_name(ObservabilityClass c);

// The tolerances of the preconditions, declared before any run (D8: "predeclared"). They are the run's validity, never its
// outcome. Ratios of the indoor grid's translation information I (sum n n^T of the matched cells' normals, world frame,
// the record's result table) are taken against its largest eigenvalue, as the degeneracy projection does.
struct LoopTolerances
{
    // every run
    // the sensor's jitter along the face normal: above the thinnest normal sigma an exact cell of an axis-aligned plane can
    // have and stay usable (K9: 3.162 mm; below it the plane's cells are skipped, and the LiDAR sees only edges and corners)
    double min_jitter_sigma_m = 0.0035;
    int time_limit_windows = 0; // windows that stopped at the solver's wall-clock limit (the run's timing and repeat are void)
    int map_rebuilds = 0;
    double turn_rate_rad_s = 0.3;  // class turn: |heading change| / duration above this (the heading without the gait's twist)
    // window 0's first node, aligned to before the run, moved by the run (largest entry of the pose matrix): the origin fix is
    // a finite weight (K10), so the node moves by a little; the alignment is valid while this is small
    double origin_moved = 1e-5;
    // the world as declared (LoopWorld): each IMU error value against the declared one, |a - b| / max(|a|, |b|) (0 when
    // both are zero); float tolerance
    double world_noise_relative = 1e-6;
    // observability: no travel direction below this distance from the window's first to its last node (a reading's
    // definition, not a precondition)
    double observability_min_travel_m = 0.1;
    // R2, each blind window
    int min_blind_windows_return = 5;   // the return leg's blind stretch must be at least this long (the time in the regime)
    double blind_max_along_share = 0.02;     // u^T I u / lambda_max(I) along the true corridor axis: near zero
    // the NDT rows' projected weight of a shift along the TRUE corridor axis / horizontally across it (BlindAxisWeight;
    // measured along the table's travel direction until the user's decision of 2026-11-21; tolerance unchanged)
    double blind_max_projected_along = 1e-3;
    double blind_min_side_share = 0.1;       // the transverse information is kept: sideways and vertical shares
    double blind_min_up_share = 0.1;
    double blind_min_side_normals = 200.0;   // sum over matched cells of (n . u_side)^2: matched side normals
    double blind_min_floor_normals = 200.0;  // sum over matched cells of (n . u_up)^2: matched floor and ceiling normals
    double blind_max_travel_angle_deg = 10.0; // the table's travel direction against the true corridor axis
    // R2, the nominated exit (the return leg's first window with this many generator hits on room A's x-facing faces)
    int exit_min_end_wall_hits = 100;
    // full support along the axis: u^T I u / lambda_max(I) at least the configuration's degenerate_projection_ratio_full
    // (the projection keeps the axis whole); the value is read from the run's settings
    int min_landmark_cells = 20;         // designated outbound landmark cells
    double min_landmark_cells_mapped = 0.5; // share of them that the map before the exit window holds
    int exit_min_landmark_matches = 50;  // accepted matches of the exit window's points to the landmark cells
};

struct Precondition
{
    std::string name;
    int window = 0;      // 1-based, 0 for the whole run
    double value = 0.0;
    double tolerance = 0.0;
    bool at_least = true; // value >= tolerance (else value <= tolerance)
    bool holds = false;
};

// the truth of one window, in the estimator's world frame (aligned at window 0's first node)
struct WindowTruth
{
    int first_node = 0; // the recording's node index of the window's first node
    std::vector<Eigen::Affine3d> poses;  // every node
    Eigen::Vector3d velocity_first = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_last = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias_rad_s = Eigen::Vector3d::Zero();  // the generated body bias (sensor axes), before any subtraction
    Eigen::Vector3d accel_offset_mps2 = Eigen::Vector3d::Zero(); // the generated offset (body axes)
    double x_min = 0.0, x_max = 0.0; // the true positions along the route's x (truth frame) over the nodes
    double heading_change_rad = 0.0; // last node's route heading minus the first's (gait twist averaged out), truth frame
    double duration_s = 0.0;
};

// what the run made of a window, against the truth
struct WindowErrors
{
    Eigen::Vector3d last_position = Eigen::Vector3d::Zero();     // solved last node minus the truth [m]
    double last_rotation_rad = 0.0;                                // angle of the solved last node's attitude against the truth
    std::optional<Eigen::Vector3d> velocity_end;                   // handed-on end velocity minus the truth at the last node [m/s]
    std::optional<double> velocity_end_normalised;                 // e^T P^-1 e with the handed-on covariance (3 dof); D5
};

// the window's along-travel observability by the truth (ObservabilityClass)
struct WindowObservability
{
    ObservabilityClass cls = ObservabilityClass::not_moving;
    double along_share = 0.0;    // u^T I u / lambda_max(I) (0 without information)
    double smallest_ratio = 0.0; // lambda_min / lambda_max: what the projection itself acts on
    Eigen::Vector3d travel = Eigen::Vector3d::Zero(); // u, estimator world frame
    Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
};

// The projected NDT rows' (ndt_indoor + ndt_outdoor) weight of a shift along ANY world direction u, from a result table.
// The table gives it only along its own three directions (travel, sideways, vertical). A shift moves every node's
// translation by u and the NDT terms are node blocks projected as M B M (M = diag(P, I), ndt_degeneracy_projection of the
// table's translation_information, the matrix the table projected with), so the weight is the quadratic form
// w(u) = u^T (P W P) u, W the sum of the nodes' raw translation blocks. Where the projection removes one direction v
// (s = 0: P v = 0), P W P lives in the plane across v and has three unknowns S (2x2 in a basis E of that plane): the
// table's three weights fix them exactly, w(d) = (E^T d)^T S (E^T d). measured false (with `why`) where no direction or
// more than one is removed, or the three directions do not fix S (the system's condition above 1e12).
struct ProjectedShiftWeight
{
    bool measured = false;
    std::string why;
    Eigen::Vector3d removed = Eigen::Vector3d::Zero(); // v (world frame)
    Eigen::Matrix<double, 3, 2> E = Eigen::Matrix<double, 3, 2>::Zero();
    Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
    double condition = 0.0; // of the 3x3 system that fixes S
    double along(const Eigen::Vector3d &u) const;
};
ProjectedShiftWeight projected_shift_weight(const WindowTermTable &result, double projection_ratio, double projection_ratio_full);

// R2's blindness reading of a blind window (precondition "projected NDT weight along / sideways"): the projected shift
// weights along the table's travel direction (the definition before 2026-11-21) and along the true corridor axis (the
// definition since: the user's decision of 2026-11-21, made after seeing the values, because the walking gait wobbles the
// travel direction by 0.1 to 2.4 deg against the corridor, so the old value was about sin^2 of that angle)
struct BlindAxisWeight
{
    bool measured = false;
    double travel_ratio = 0.0;      // the table's: along travel / sideways
    double true_axis_ratio = 0.0;   // along the true axis / horizontally across it
    double travel_angle_deg = 0.0;  // the table's travel direction against the true axis
    double removed_angle_deg = 0.0; // the removed direction v against the true axis
    double axis_vertical = 0.0;     // the true axis's vertical component (estimator world frame)
    double condition = 0.0;
};

struct LoopWindow
{
    int window = 0; // 1-based, as WindowTermRecord::window
    WindowClass cls = WindowClass::seen;
    WindowObservability observability;
    WindowTruth truth;
    BlindAxisWeight blind_axis; // R2 blind windows only
    std::optional<WindowTermRecord> record;
    std::vector<Eigen::Affine3d> poses; // as solved
    WindowErrors errors;
};

// the readings a run did not have (NOT MEASURED), counted per reading
struct Unavailable
{
    int records_missing = 0;
    int handoff_not_done = 0;
    int heading_bend = 0, speed = 0, max_bias = 0, velocity_chi2 = 0, information_ratio = 0, remaining_step = 0, lidar_share = 0;
    int bent_check = 0, fast_check = 0, bias_check = 0;
    int move_velocity = 0, move_joint = 0, move_bias = 0; // normalised moves NOT MEASURED (any reason)
    int move_reasons[3][kMoveNotMeasuredReasons] = {};      // the same per reason (velocity, joint, bias; window_verdict.h)
    int rotation_rate = 0, start_acceleration = 0;
    int velocity_end_normalised = 0;
    int total() const;
    std::string text() const;
};

struct R2Design
{
    bool present = false;
    int turn_window = 0;   // 1-based: the first window whose first node heads back (truth)
    int exit_window = 0;   // 1-based: the nominated exit of the return leg (over the whole recording), 0 if none
    bool exit_in_run = false; // the run reaches it (a shortened run does not evaluate the return leg and the exit)
    int exit_end_wall_hits = 0;
    std::set<std::uint64_t> landmark_cells; // the indoor grid's keys in the estimator's world frame
    int landmark_cells_mapped = 0;          // of them, in the map before the exit window
    int exit_landmark_matches = 0;          // the exit window's accepted matches to them
    int exit_points = 0;                    // the exit window's points in range
    int exit_points_in_landmark_keys = 0;   // of them, placed into a landmark cell's key (whether the map accepts it or not)
    int exit_points_accepted = 0;           // of them, accepted by any cell of the map before the exit
};

struct LoopRun
{
    std::string route;
    LoopWorld world = LoopWorld::matched;
    double ahrs_gain = 0.0;
    bool estimated = true;     // false: observe_loop (truth, classes, observability and the R2 design only; no run)
    bool switches = false;
    bool completed = false;    // optimize_windows returned true
    int windows_cut = 0;       // what the front end made (before any truncation)
    int windows_run = 0;
    double seconds = 0.0;      // optimize_windows' wall time
    int time_limit_windows = 0;
    int map_rebuilds = -1;     // -1: the loop's line was not seen
    // a reading, not a precondition: windows in which the body moves (truth) that the loop's rest test took as at rest
    // (A2 / F10 of task 5.4: a constant-velocity window passes the rest test; all routes are walked, so it counts what the gait does not prevent)
    int moving_judged_at_rest = 0;
    int first_moving_judged_at_rest = 0; // 1-based, 0: none
    std::vector<LoopWindow> windows;
    std::vector<Precondition> preconditions;
    std::vector<std::string> not_evaluated; // preconditions a shortened run does not reach (listed, never counted as holding)
    R2Design r2;
    Unavailable unavailable;
    int class_windows[kWindowClasses] = {0, 0, 0, 0, 0};
    double class_seconds[kWindowClasses] = {0, 0, 0, 0, 0};
    int observability_windows[kObservabilityClasses] = {0, 0, 0, 0};
    int class_observability[kWindowClasses][kObservabilityClasses] = {}; // truth class x observability class
    double projection_ratio = 0.0, projection_ratio_full = 0.0;        // the run's thresholds

    // readings over the run's windows (descriptive)
    double end_error_m() const;      // the last window's last node, |solved - truth|
    double largest_error_m() const;  // the same, largest over the windows' last nodes
    std::optional<double> largest_velocity_end_normalised() const;
    std::string observability_text() const; // the counts, and per truth class

    bool valid() const; // completed and every precondition holds
    std::vector<std::string> failed() const; // the preconditions that do not hold, as text
    std::string summary() const;             // one line: windows, time, classes, preconditions, unavailable
};

// Cuts the recording with `config` (its front end fields must be front_end_config's), keeps the first `max_windows`
// windows (0: all) and runs optimize_windows on them with a keeping term sink (and, for R2 with an exit in the range, a
// capture sink over the exit window). Every precondition is computed with `tol`.
// `world` is the world the recording was declared to be drawn from (LoopWorld); its values are checked.
LoopRun run_loop(const InertialRecording &recording, const OdometryConfig &config, LoopWorld world, int max_windows = 0, const LoopTolerances &tol = {});
// the same up to the run: the cut, the truth, the classes, the observability, the R2 design and the run-independent
// preconditions (seconds; for scene design). LoopRun::estimated is false.
LoopRun observe_loop(const InertialRecording &recording, const OdometryConfig &config, LoopWorld world, int max_windows = 0, const LoopTolerances &tol = {});

// The declared deterministic fields (D8 "declared deterministic fields repeat"): per window the iterations, the last
// update, the damping, the solved poses, start and end velocity, gyro bias, acceleration offset, the stop reason, the
// handed-on velocity and bias, the three normalised moves; per run the window count and the preconditions' values.
// Compared bit for bit; `difference` names the first field that differs. A run that reached the time limit is not
// comparable (false, with the reason).
bool same_deterministic_fields(const LoopRun &a, const LoopRun &b, std::string &difference);
}
