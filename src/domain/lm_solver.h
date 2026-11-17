#pragma once

// Levenberg-Marquardt over any problem that can linearize itself, take the covariance of its undamped unknowns, apply a
// step and keep a snapshot of its state (stage 4 of docs/code-split-and-region-agents-plan.md). The solver knows no term,
// pose or map; the window's side is WindowProblem (window_problem.h).

#include "normal_equations.h"
#include "phase_timing.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <functional>
#include <vector>

// the normal equations of a problem at its present state
struct LmLinearization
{
    Eigen::SparseMatrix<double> AtPA; // undamped
    Eigen::SparseMatrix<double> AtPB; // right-hand side, one column
    int damped = 0;                   // the leading unknowns get the damping; the others are the "extra" unknowns
    double cost = 0.0;                // when the cost was asked for
};

class LmProblem
{
public:
    virtual ~LmProblem() = default;
    // the normal equations at the present state; with_cost: also its cost
    virtual LmLinearization linearize(bool with_cost) = 0;
    // the columns of the inverse of the undamped matrix for the extra unknowns (n x n_extra), when that solve succeeded
    virtual void take_covariance(const Eigen::MatrixXd &extra_columns) = 0;
    // applies a step (h_x, one entry per unknown; solved false: the solve failed), sets delta (the size of the update)
    // and returns whether the step was applied
    virtual bool apply(const std::vector<double> &h_x, bool solved, double &delta) = 0;
    // the step check (lm_solve with step_check): keep the present state as a candidate; make the candidate the accepted
    // state; go back to the accepted state
    virtual void snapshot() = 0;
    virtual void accept_snapshot() = 0;
    virtual void restore_accepted() = 0;
};

// what the solver keeps from one call to the next: the symbolic analyses of its two factorizations (results are the same
// without them)
struct LmWorkspace
{
    PatternCachedCholesky &covariance;
    PatternCachedCholesky &damped;
};

// the solver's own phases (null: not timed)
struct LmTiming
{
    PhaseTime *covariance = nullptr;
    PhaseTime *solve = nullptr;
    PhaseTime *update = nullptr;
};

struct LmIteration
{
    bool applied = false;
    double cost = 0.0; // the cost at the state the iteration started from (with_cost)
};

// One iteration: linearize; the covariance of the extra unknowns from the undamped matrix; with snapshot, keep the state;
// without apply_step stop here (a cost-only call); else damp the leading unknowns by lm_factor, solve and apply.
LmIteration lm_iteration(LmProblem &problem, double lm_factor, bool apply_step, bool with_cost, bool snapshot, double &delta,
                         LmWorkspace workspace, const LmTiming &timing = {});

// how lm_solve's iterations ended
enum class LmStop
{
    converged,       // the update fell below convergence_delta_threshold
    iteration_limit, // the last iteration ran
    time_limit,      // out_of_time said so
    forced           // a replay: the iterations ended where the captured run's did
};
const char *lm_stop_name(LmStop stop);

struct LmOptions
{
    int last_iteration = 99;                   // index of the last iteration
    bool forced = false;                       // last_iteration is a replay's: no time limit
    double initial_factor = 5.0;
    double convergence_delta_threshold = 1e-12;
    bool step_check = false;                   // lm_step_check
    std::function<bool()> out_of_time;         // asked after each counted iteration; null: no time limit
    std::function<void(double delta)> after_iteration;           // after each accepted iteration, before the convergence test
    std::function<void(double factor, double delta)> factor_change; // when the damping factor is about to change
};

struct LmResult
{
    int last_iteration = 0; // index of the last iteration run (rejected ones count)
    double delta = 100000.0;
    double lm_factor = 0.0; // the damping after the iterations
    int rejected_steps = 0;
    LmStop stop = LmStop::iteration_limit;
};

// The iterations of a window (formerly run_lm_iterations): the damping grows tenfold after iterations 10, 20, ... and at
// once after a step that was not applied (next_lm_factor); they end when the update falls below the threshold, after the
// last iteration, or when out_of_time says so. With step_check each call also gives the cost of the state it starts from;
// when that is higher than the accepted state's, the step that led there is taken back and the damping raised tenfold
// (that iteration skips the convergence test, the scheduled factor update and the time check); after the loop a
// cost-only call checks the last step, so the problem ends at the lowest-cost state it reached.
LmResult lm_solve(LmProblem &problem, const LmOptions &options, LmWorkspace workspace, const LmTiming &timing = {});

// Levenberg-Marquardt damping for the next iteration: x10 after iterations 10, 20, ..., and x10 at once after a step
// that was not applied (failed solve or big jump); without that the next iteration builds the same system, gets the
// same step and rejects it again, until the 10th
double next_lm_factor(double lm_factor, int iter, bool step_applied);
