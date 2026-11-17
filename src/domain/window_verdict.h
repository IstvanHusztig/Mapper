#pragma once

// The window verdict (stage 4 of docs/code-split-and-region-agents-plan.md): after a window's iterations, what the run
// knows about it as readings and flags, and at the end of a run the run's triggers. It follows the agreed evaluation
// contract (openspec/changes/sensor-neutral-yardstick/criteria-proposal.md, 2026-11-12): these are investigation
// triggers, not a statement about the map's accuracy, and a reading a window does not have is NOT MEASURED, never clear.
// It only observes: no window is processed differently because of it.

#include "lm_solver.h"
#include "run_reporting.h"

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

struct RunVerdict
{
    int windows = 0;
    bool complete = true; // false: a cancelled run
    std::vector<FlagSummary> flags; // bent, bent severely, fast, bias high
    ReadingSummary chi2_weak;       // velocity chi2 of the windows with an information ratio below weak_ratio
    ReadingSummary chi2_all;
    Check chi2_run = Check::not_measured; // the run's trigger on chi2_weak
    std::vector<ReadingSummary> readings;
    int stopped[4] = {0, 0, 0, 0}; // per LmStop
    int rejected_steps = 0, steps_not_applied = 0, big_jump_nodes = 0;
};

RunVerdict summarize_run(const std::vector<WindowVerdict> &windows, bool complete, const VerdictLimits &limits = {});

double percentile_linear(std::vector<double> values, double p);

// the summary's log lines ("window verdict summary: ...")
void log_run_verdict(const RunVerdict &run, const VerdictLimits &limits, Logger &logger);

// the per-window line ("window verdict <n>: ..."), for a window with a flag
void log_window_verdict(const WindowVerdict &verdict, Logger &logger);
