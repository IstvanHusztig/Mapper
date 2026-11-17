#include "lm_solver.h"

#include <vector>

double next_lm_factor(double lm_factor, int iter, bool step_applied)
{
    if (!step_applied)
        lm_factor *= 10.0;
    if (iter % 10 == 0 && iter > 0)
        lm_factor *= 10.0;
    return lm_factor;
}

const char *lm_stop_name(LmStop stop)
{
    switch (stop)
    {
    case LmStop::converged:
        return "converged";
    case LmStop::iteration_limit:
        return "iteration limit";
    case LmStop::time_limit:
        return "time limit";
    case LmStop::forced:
        return "forced";
    }
    return "?";
}

namespace
{
    // marginal covariance of the extra unknowns: their columns of the inverse of the undamped matrix, handed to the problem
    // when the factorization and the solve succeed
    void covariance_of_extra_unknowns(LmProblem &problem, const LmLinearization &lin, PatternCachedCholesky &cholesky)
    {
        const int n = static_cast<int>(lin.AtPA.rows());
        const int n_extra = n - lin.damped;
        if (n_extra <= 0)
            return;
        cholesky.compute(lin.AtPA);
        const PatternCachedCholesky::Solver &solver = cholesky.solver();
        if (solver.info() != Eigen::Success)
            return;
        Eigen::MatrixXd unit = Eigen::MatrixXd::Zero(n, n_extra);
        for (int r = 0; r < n_extra; r++)
            unit(lin.damped + r, r) = 1.0;
        const Eigen::MatrixXd cols = solver.solve(unit);
        if (solver.info() != Eigen::Success)
            return;
        problem.take_covariance(cols);
    }

    // the damped solve: lm_factor on the leading (damped) unknowns only; the step, one entry per unknown
    std::vector<double> damped_step(LmLinearization &lin, double lm_factor, PatternCachedCholesky &cholesky, bool &solved)
    {
        const int n = static_cast<int>(lin.AtPA.rows());
        Eigen::SparseMatrix<double> AtPA_I(n, n);
        if (n > lin.damped)
        {
            // LM damping on the pose parameters only: lm_factor grows to ~1e10, which is negligible against the pose
            // curvature (~1e8+) but would freeze the bias (curvature ~1e3); the bias is kept well-posed by its prior
            std::vector<Eigen::Triplet<double>> damping;
            damping.reserve(lin.damped);
            for (int k = 0; k < lin.damped; k++)
                damping.emplace_back(k, k, 1.0);
            AtPA_I.setFromTriplets(damping.begin(), damping.end());
        }
        else
            AtPA_I.setIdentity();
        AtPA_I *= lm_factor;
        lin.AtPA += (AtPA_I);

        cholesky.compute(lin.AtPA);
        Eigen::SparseMatrix<double> x = cholesky.solver().solve(lin.AtPB);
        solved = cholesky.solver().info() == Eigen::Success;
        return solution_by_row(x, n);
    }
}

LmIteration lm_iteration(LmProblem &problem, double lm_factor, bool apply_step, bool with_cost, bool snapshot, double &delta,
                         LmWorkspace workspace, const LmTiming &timing)
{
    LmIteration it;
    LmLinearization lin = problem.linearize(with_cost);
    it.cost = lin.cost;
    {
        PhaseTimer covariance_timer(timing.covariance);
        covariance_of_extra_unknowns(problem, lin, workspace.covariance);
    }
    if (snapshot)
        problem.snapshot();
    if (!apply_step)
        return it;

    PhaseTimer solve_timer(timing.solve);
    bool solved = false;
    const std::vector<double> h_x = damped_step(lin, lm_factor, workspace.damped, solved);
    solve_timer.stop();
    PhaseTimer update_timer(timing.update);
    it.applied = problem.apply(h_x, solved, delta);
    return it;
}

LmResult lm_solve(LmProblem &problem, const LmOptions &options, LmWorkspace workspace, const LmTiming &timing)
{
    LmResult result;
    result.stop = options.forced ? LmStop::forced : LmStop::iteration_limit;
    double delta = 100000.0;
    double lm_factor = options.initial_factor;
    int iter_end = 0;
    // step check: the cost of the accepted state (the state with the lowest cost so far)
    double accepted_cost = 0.0;
    bool have_accepted = false;
    int rejected_steps = 0;

    for (int iter = 0; iter <= options.last_iteration; iter++)
    {
        iter_end = iter;
        delta = 100000.0;
        bool applied = false;
        if (options.step_check)
        {
            const LmIteration it = lm_iteration(problem, lm_factor, true, true, true, delta, workspace, timing);
            applied = it.applied;
            if (have_accepted && it.cost > accepted_cost)
            {
                // the previous step raised the cost: back to the state before it, with more damping
                problem.restore_accepted();
                lm_factor *= 10.0;
                rejected_steps++;
                continue; // the rejected step says nothing about convergence
            }
            // the state this call started from is the best so far
            problem.accept_snapshot();
            accepted_cost = it.cost;
            have_accepted = true;
        }
        else
            applied = lm_iteration(problem, lm_factor, true, false, false, delta, workspace, timing).applied;
        if (options.after_iteration)
            options.after_iteration(delta);
        if (delta < options.convergence_delta_threshold)
        {
            result.stop = LmStop::converged;
            break;
        }

        const double next_factor = next_lm_factor(lm_factor, iter, applied);
        if (options.factor_change && next_factor != lm_factor)
            options.factor_change(lm_factor, delta);
        lm_factor = next_factor;

        if (!options.forced && options.out_of_time && options.out_of_time())
        {
            result.stop = LmStop::time_limit;
            break;
        }
    }

    if (options.step_check)
    {
        // the last step: keep it only if it lowered the cost
        const double last_delta = delta;
        const LmIteration it = lm_iteration(problem, lm_factor, false, true, false, delta, workspace, timing);
        delta = last_delta;
        if (have_accepted && it.cost > accepted_cost)
        {
            problem.restore_accepted();
            rejected_steps++;
        }
    }
    result.rejected_steps = rejected_steps;
    result.last_iteration = iter_end;
    result.delta = delta;
    result.lm_factor = lm_factor;
    return result;
}
