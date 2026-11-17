#include <doctest/doctest.h>

#include "lm_solver.h"

#include <cmath>
#include <string>
#include <vector>

// The Levenberg-Marquardt solver (lm_solver.h, stage 4 of docs/code-split-and-region-agents-plan.md) on a problem without
// a window, map or term: cost (x - t)^T A (x - t) in two unknowns, the first damped, the second an "extra" unknown.

namespace
{
    class Quadratic : public LmProblem
    {
    public:
        Eigen::Matrix2d A;
        Eigen::Vector2d target{1.0, -2.0};
        Eigen::Vector2d x{0.0, 0.0};
        Eigen::MatrixXd covariance;                 // what take_covariance received
        int apply_calls = 0;
        int refuse_call = -1;                        // this apply call is refused (step not applied)
        int bad_call = -1;                           // this apply call moves away from the target instead
        int snapshots = 0;
        Eigen::Vector2d candidate{0.0, 0.0}, accepted{0.0, 0.0};

        Quadratic() { A << 4.0, 1.0, 1.0, 3.0; }

        double cost_at(const Eigen::Vector2d &v) const { return (v - target).dot(A * (v - target)); }

        LmLinearization linearize(bool with_cost) override
        {
            LmLinearization lin;
            lin.AtPA = A.sparseView();
            const Eigen::Vector2d g = A * (target - x);
            lin.AtPB = Eigen::MatrixXd(g).sparseView();
            lin.damped = 1;
            lin.cost = with_cost ? cost_at(x) : 0.0;
            return lin;
        }
        void take_covariance(const Eigen::MatrixXd &extra_columns) override { covariance = extra_columns; }
        bool apply(const std::vector<double> &h_x, bool solved, double &delta) override
        {
            const int call = apply_calls++;
            delta = 1000000.0;
            if (!solved || call == refuse_call)
                return false;
            Eigen::Vector2d h(h_x[0], h_x[1]);
            if (call == bad_call)
                h = Eigen::Vector2d(5.0, 5.0);
            x += h;
            delta = std::abs(h.x()) + std::abs(h.y());
            return true;
        }
        void snapshot() override
        {
            snapshots++;
            candidate = x;
        }
        void accept_snapshot() override { accepted = candidate; }
        void restore_accepted() override { x = accepted; }
    };

    struct Caches
    {
        PatternCachedCholesky covariance, damped;
        LmWorkspace workspace() { return LmWorkspace{covariance, damped}; }
    };

    LmOptions no_stop(int last_iteration)
    {
        LmOptions o;
        o.last_iteration = last_iteration;
        o.initial_factor = 5.0;
        o.convergence_delta_threshold = -1.0; // the update is never below it
        return o;
    }
}

TEST_CASE("lm solver: one iteration takes the damped step, and hands over the covariance from the undamped matrix")
{
    Quadratic q;
    Caches c;
    double delta = 0.0;
    const LmIteration it = lm_iteration(q, 1e-12, true, true, false, delta, c.workspace());
    CHECK(it.applied);
    CHECK(it.cost == doctest::Approx(q.cost_at(Eigen::Vector2d::Zero())));
    CHECK(q.x.x() == doctest::Approx(1.0));
    CHECK(q.x.y() == doctest::Approx(-2.0));
    CHECK(delta == doctest::Approx(3.0));
    // the extra unknown's column of A^-1 (undamped)
    const Eigen::Matrix2d inverse = q.A.inverse();
    REQUIRE(q.covariance.rows() == 2);
    REQUIRE(q.covariance.cols() == 1);
    CHECK(q.covariance(0, 0) == doctest::Approx(inverse(0, 1)));
    CHECK(q.covariance(1, 0) == doctest::Approx(inverse(1, 1)));
    CHECK(q.snapshots == 0);

    SUBCASE("damping on the leading unknown only")
    {
        Quadratic d;
        double delta_d = 0.0;
        lm_iteration(d, 1e6, true, false, false, delta_d, c.workspace());
        // x0 hardly moves; x1 moves to its minimum with x0 held: A11 x1 = A11 t1 + A10 (t0 - x0)
        CHECK(std::abs(d.x.x()) < 1e-5);
        CHECK(d.x.y() == doctest::Approx(-2.0 + 1.0 / 3.0).epsilon(1e-5));
    }
    SUBCASE("without a step: the covariance, nothing else")
    {
        Quadratic n;
        double delta_n = 42.0;
        const LmIteration none = lm_iteration(n, 5.0, false, true, true, delta_n, c.workspace());
        CHECK_FALSE(none.applied);
        CHECK(n.x == Eigen::Vector2d::Zero());
        CHECK(delta_n == 42.0);
        CHECK(n.apply_calls == 0);
        CHECK(n.covariance.rows() == 2);
        CHECK(n.snapshots == 1);
    }
}

TEST_CASE("lm solver: the damping grows tenfold after iterations 10, 20, ... and at once after a refused step")
{
    Caches c;
    SUBCASE("by schedule")
    {
        Quadratic q;
        std::vector<double> changed_from;
        LmOptions o = no_stop(20);
        o.factor_change = [&](double factor, double) { changed_from.push_back(factor); };
        const LmResult r = lm_solve(q, o, c.workspace());
        REQUIRE(changed_from.size() == 2); // after iteration 10 and after iteration 20
        CHECK(changed_from[0] == 5.0);
        CHECK(changed_from[1] == 50.0);
        CHECK(r.lm_factor == 500.0);
        CHECK(r.last_iteration == 20);
        CHECK(r.stop == LmStop::iteration_limit);
    }
    SUBCASE("a refused step")
    {
        Quadratic q;
        q.refuse_call = 2;
        std::vector<double> changed_from;
        std::vector<Eigen::Vector2d> after;
        LmOptions o = no_stop(4);
        o.factor_change = [&](double factor, double) { changed_from.push_back(factor); };
        o.after_iteration = [&](double) { after.push_back(q.x); };
        lm_solve(q, o, c.workspace());
        REQUIRE(changed_from.size() == 1);
        CHECK(changed_from[0] == 5.0); // after iteration 2: 5 -> 50
        REQUIRE(after.size() == 5);
        CHECK(after[2] == after[1]); // the refused step left the state as it was
    }
}

TEST_CASE("lm solver: the iterations end at convergence, the iteration limit, a forced last iteration or the time limit")
{
    Caches c;
    SUBCASE("converged")
    {
        Quadratic q;
        LmOptions o = no_stop(99);
        o.initial_factor = 1e-9;
        o.convergence_delta_threshold = 1e-6;
        const LmResult r = lm_solve(q, o, c.workspace());
        CHECK(r.stop == LmStop::converged);
        CHECK(r.last_iteration < 5);
        CHECK(std::string(lm_stop_name(r.stop)) == "converged");
    }
    SUBCASE("iteration limit")
    {
        Quadratic q;
        const LmResult r = lm_solve(q, no_stop(3), c.workspace());
        CHECK(r.stop == LmStop::iteration_limit);
        CHECK(r.last_iteration == 3);
        CHECK(q.apply_calls == 4);
    }
    SUBCASE("forced: no time limit")
    {
        Quadratic q;
        LmOptions o = no_stop(3);
        o.forced = true;
        int asked = 0;
        o.out_of_time = [&]() { asked++; return true; };
        const LmResult r = lm_solve(q, o, c.workspace());
        CHECK(r.stop == LmStop::forced);
        CHECK(r.last_iteration == 3);
        CHECK(asked == 0);
    }
    SUBCASE("time limit")
    {
        Quadratic q;
        LmOptions o = no_stop(10);
        int asked = 0;
        o.out_of_time = [&]() { return ++asked == 2; };
        const LmResult r = lm_solve(q, o, c.workspace());
        CHECK(r.stop == LmStop::time_limit);
        CHECK(r.last_iteration == 1);
    }
}

TEST_CASE("lm solver: with the step check a step that raised the cost is taken back; without it nothing is saved")
{
    Caches c;
    SUBCASE("step check")
    {
        Quadratic q;
        q.bad_call = 1; // iteration 1 moves away from the target
        std::vector<double> changed_from;
        LmOptions o = no_stop(4);
        o.initial_factor = 1e-9;
        o.step_check = true;
        o.factor_change = [&](double factor, double) { changed_from.push_back(factor); };
        const LmResult r = lm_solve(q, o, c.workspace());
        CHECK(r.rejected_steps == 1);
        // iteration 2 found the higher cost and went back to the state iteration 1 started from, with ten times the damping
        CHECK(r.lm_factor == doctest::Approx(1e-8));
        CHECK(q.cost_at(q.x) < 1e-12);
        CHECK(changed_from.empty()); // the rejection's increase is not a scheduled change
    }
    SUBCASE("the last step is checked too")
    {
        Quadratic q;
        q.bad_call = 2; // the last iteration's step is bad
        LmOptions o = no_stop(2);
        o.initial_factor = 1e-9;
        o.step_check = true;
        const LmResult r = lm_solve(q, o, c.workspace());
        CHECK(r.rejected_steps == 1);
        CHECK(q.cost_at(q.x) < 1e-12);
    }
    SUBCASE("no step check: no snapshots")
    {
        Quadratic q;
        lm_solve(q, no_stop(4), c.workspace());
        CHECK(q.snapshots == 0);
    }
}
