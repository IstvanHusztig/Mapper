#pragma once

// The window verdict (stage 4 of docs/code-split-and-region-agents-plan.md): after a window's iterations, what the run
// knows about it as readings and flags, and at the end of a run the run's triggers. It follows the agreed evaluation
// contract (openspec/changes/sensor-neutral-yardstick/criteria-proposal.md, 2026-11-12): these are investigation
// triggers, not a statement about the map's accuracy, and a reading a window does not have is NOT MEASURED, never clear.
// It only observes: no window is processed differently because of it.

#include "lm_solver.h"
#include "run_reporting.h"

#include <Eigen/Dense>

#include <array>
#include <optional>
#include <string>
#include <vector>

enum class Check
{
    clear,
    trigger,
    not_measured
};
const char *check_name(Check c);

// the agreed limits (the contract and tools/run_health.py)
struct VerdictLimits
{
    double bend_deg = 3.0;          // a window is bent above it
    double severe_bend_deg = 10.0;  // bent severely above it
    double speed_mps = 2.5;         // fast above it
    double bias_deg_s = 0.5;        // bias high above it, per axis
    double bent_share = 0.01;       // the run's trigger: more than this share of the windows bent
    double chi2_median = 5.0;       // the run's trigger on the velocity chi2 of the weak windows
    double chi2_p95 = 20.0;
    double weak_ratio = 0.45;       // weak windows: information ratio below it (as log_window_diagnostics selects them)
};

// The normalised move of a carried state (openspec change consistency-harness, design D10): with the prior N(m, P) the
// window was prepared with, its result x and the covariance P_post the window hands on for it, d = x - m and
//     q = d^T P^-1 d / (n - tr(P^-1 P_post)),
// whose expected value is 1 in a linear-Gaussian update with a fixed observation model. Descriptive: P_post is conditional
// on the fixed first node, the NDT weights are re-weighted and the problem is linearised. No limit, flag or trigger.
enum class MoveNotMeasured
{
    measured,
    absent,          // the window has no such term, or no covariance was taken for its present state
    not_finite,      // an input is NaN or infinite
    factorisation,   // the prior covariance's Cholesky factorisation failed
    ill_conditioned, // a prior variance not positive, or the scaled prior's reciprocal condition below the limit
    non_physical,    // an eigenvalue of the whitened posterior outside [0, 1]: not a reduction of uncertainty
    small_reduction, // n - tr(P^-1 P_post) below the limit: the window adds almost nothing to the state
    last_step,       // the last accepted step moved a component by more than its share of the prior sigma
    clamped          // the gyro bias ended at its clamp
};
constexpr int kMoveNotMeasuredReasons = 9;
const char *move_not_measured_name(MoveNotMeasured reason);

struct NormalisedMove
{
    std::optional<double> value;
    MoveNotMeasured reason = MoveNotMeasured::absent; // measured exactly when value is set
    std::optional<double> reduction;                  // n - tr(P^-1 P_post), where it was computed
};

struct NormalisedMoveLimits
{
    double step_share = 0.01;      // the last step's largest share of the prior sigma, per component
    double eigen_tolerance = 1e-6; // around [0, 1] for the whitened posterior's eigenvalues
    double min_reduction = 0.5;
    double min_rcond = 1e-12; // of the prior scaled to unit variances
};

// the normalised move of one state. The prior is whitened through the Cholesky factor of its correlation form
// (S^-1 P S^-1 = L L^T, S the prior sigmas), so the units of the components don't matter; last_step: the change of the
// state by the step taken after P_post was computed (zero if none).
NormalisedMove normalised_move(const Eigen::VectorXd &prior_mean, const Eigen::MatrixXd &prior_cov, const Eigen::VectorXd &result,
                               const Eigen::MatrixXd &posterior_cov, const Eigen::VectorXd &last_step, const NormalisedMoveLimits &limits = {});

// what the readings need from a window's problem, taken out of it before it goes out of scope
// (WindowProblem::honesty_inputs, called in run_odometry.cpp's run_lm_iterations)
struct HonestyInputs
{
    // a covariance was taken during the iterations, and the state still holds it (a step taken back restores both)
    bool covariance_valid = false;
    bool velocity = false;    // the accelerometer term: start velocity and acceleration offset
    bool joint_prior = false; // imu_carry_joint_covariance: the prior on (start velocity, offset) is one joint Gaussian
    bool bias = false;        // gyro-bias estimation
    bool bias_clamped = false;

    Eigen::Vector3d velocity_prior_mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d velocity_prior_cov = Eigen::Matrix3d::Zero();
    Eigen::Vector3d offset_prior_mean = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> joint_prior_cov = Eigen::Matrix<double, 6, 6>::Zero(); // (start velocity, offset); joint mode
    Eigen::Vector3d start_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_offset = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> start_posterior = Eigen::Matrix<double, 6, 6>::Zero(); // (start velocity, offset)
    Eigen::Vector3d start_velocity_step = Eigen::Vector3d::Zero();                    // since the covariance was taken
    Eigen::Vector3d acc_offset_step = Eigen::Vector3d::Zero();

    Eigen::Vector3d bias_prior_mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d bias_prior_cov = Eigen::Matrix3d::Zero();
    Eigen::Vector3d bias_mean = Eigen::Vector3d::Zero();
    Eigen::Matrix3d bias_posterior = Eigen::Matrix3d::Zero();
    Eigen::Vector3d bias_step = Eigen::Vector3d::Zero();
};

// a window's normalised moves: the start velocity (with the accelerometer term), start velocity and offset jointly (with
// the joint prior), the gyro bias (with its estimation); NOT MEASURED otherwise, or for a reason of normalised_move
struct HonestyReadings
{
    NormalisedMove start_velocity;
    NormalisedMove joint;
    NormalisedMove gyro_bias;
};
HonestyReadings honesty_readings(const HonestyInputs &inputs, const NormalisedMoveLimits &limits = {});

// a normalised move as text: the value with 3 decimals, or "NOT MEASURED (<reason>)"
std::string move_text(const NormalisedMove &move);

// what the loop knows about a window; an empty optional is a reading the window does not have
struct WindowEvidence
{
    int window = 0;      // 1-based
    double time_s = 0.0; // the window's first node, from the run's first node
    std::optional<double> heading_bend_deg;  // vertical component of the rotation guess -> result at the last node
    std::optional<double> roll_bend_deg;     // the horizontal components (world x, y)
    std::optional<double> pitch_bend_deg;
    std::optional<double> speed_mps;         // norm of the end velocity (windows with the accelerometer term)
    std::optional<double> max_bias_deg_s;    // largest |gyro bias| over the axes (windows that estimate it)
    std::optional<double> velocity_chi2;     // start velocity against its prior, 3 residuals (windows with the prior)
    std::optional<double> information_ratio; // smallest / largest eigenvalue of the translation information
    LmStop stop = LmStop::iteration_limit;
    int iterations = 0;
    double lm_factor = 0.0;
    int rejected_steps = 0;
    int steps_not_applied = 0; // failed solves
    int big_jump_nodes = 0;
    std::optional<double> remaining_step_m;  // log_window_terms: the largest |pull| of all terms together (shift, stretch, curve)
    std::optional<double> lidar_share_along; // log_window_terms: the NDT grids' share of the weight on stretch along travel
    HonestyReadings moves;                   // the normalised moves (every run)
};

struct WindowVerdict
{
    WindowEvidence evidence;
    Check bent = Check::not_measured;
    Check bent_severely = Check::not_measured;
    Check fast = Check::not_measured;
    Check bias_high = Check::not_measured;
    bool any() const { return bent == Check::trigger || bent_severely == Check::trigger || fast == Check::trigger || bias_high == Check::trigger; }
};

WindowVerdict judge_window(const WindowEvidence &evidence, const VerdictLimits &limits = {});

// one flag over a run
struct FlagSummary
{
    const char *name = "";
    int measured = 0;
    int flagged = 0;
    int first_window = 0; // 0: none
    double first_time_s = 0.0;
    int worst_window = 0;
    double worst_time_s = 0.0;
    double worst_value = 0.0;
    Check run = Check::not_measured; // the run's trigger
};

// median, 95th percentile (linear interpolation, as numpy) and largest of a reading over the windows that have it
struct ReadingSummary
{
    const char *name = "";
    int measured = 0;
    double median = 0.0, p95 = 0.0, largest = 0.0;
};

// one normalised move over a run: its values over the windows that have it, and why the others don't
struct MoveSummary
{
    ReadingSummary reading;
    std::array<int, kMoveNotMeasuredReasons> not_measured{}; // windows per reason (index measured stays 0)
};

struct RunVerdict
{
    int windows = 0;
    bool complete = true; // false: a cancelled run
    std::vector<FlagSummary> flags; // bent, bent severely, fast, bias high
    ReadingSummary chi2_weak;       // velocity chi2 of the windows with an information ratio below weak_ratio
    ReadingSummary chi2_all;
    Check chi2_run = Check::not_measured; // the run's trigger on chi2_weak
    std::vector<ReadingSummary> readings;
    std::vector<MoveSummary> moves; // start velocity, start velocity and offset, gyro bias; listed also when none was measured
    int stopped[4] = {0, 0, 0, 0}; // per LmStop
    int rejected_steps = 0, steps_not_applied = 0, big_jump_nodes = 0;
};

RunVerdict summarize_run(const std::vector<WindowVerdict> &windows, bool complete, const VerdictLimits &limits = {});

double percentile_linear(std::vector<double> values, double p);

// the summary's log lines ("window verdict summary: ...")
void log_run_verdict(const RunVerdict &run, const VerdictLimits &limits, Logger &logger);

// the per-window line ("window verdict <n>: ..."), for a window with a flag
void log_window_verdict(const WindowVerdict &verdict, Logger &logger);
