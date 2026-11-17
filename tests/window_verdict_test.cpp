#include <doctest/doctest.h>

#include "window_verdict.h"

#include <cmath>
#include <vector>

// The window verdict (window_verdict.h, stage 4) on made-up evidence: the flags at their limits, missing readings, and the
// run summary's counts, coverage and triggers.

namespace
{
    WindowEvidence evidence(int window, double bend_deg)
    {
        WindowEvidence e;
        e.window = window;
        e.time_s = 0.5 * window;
        e.heading_bend_deg = bend_deg;
        e.speed_mps = 1.0;
        e.max_bias_deg_s = 0.1;
        return e;
    }
}

TEST_CASE("window verdict: a flag only above its limit, none at it")
{
    CHECK(judge_window(evidence(1, 3.0)).bent == Check::clear);
    CHECK(judge_window(evidence(1, 3.01)).bent == Check::trigger);
    CHECK(judge_window(evidence(1, -3.01)).bent == Check::trigger); // the size counts, not the sign
    CHECK(judge_window(evidence(1, 10.0)).bent_severely == Check::clear);
    CHECK(judge_window(evidence(1, 10.5)).bent_severely == Check::trigger);
    WindowEvidence e = evidence(1, 0.0);
    e.speed_mps = 2.5;
    CHECK(judge_window(e).fast == Check::clear);
    e.speed_mps = 2.51;
    CHECK(judge_window(e).fast == Check::trigger);
    e.max_bias_deg_s = 0.51;
    CHECK(judge_window(e).bias_high == Check::trigger);
    CHECK(judge_window(e).any());
    CHECK_FALSE(judge_window(evidence(1, 1.0)).any());
}

TEST_CASE("window verdict: a missing reading is NOT MEASURED, and the summary counts it as not measured")
{
    WindowEvidence e = evidence(1, 0.5);
    e.speed_mps.reset();
    e.max_bias_deg_s.reset();
    const WindowVerdict v = judge_window(e);
    CHECK(v.fast == Check::not_measured);
    CHECK(v.bias_high == Check::not_measured);
    CHECK(v.bent == Check::clear);

    std::vector<WindowVerdict> windows{v, judge_window(evidence(2, 0.5))};
    const RunVerdict r = summarize_run(windows, true);
    REQUIRE(r.flags.size() == 4);
    CHECK(r.flags[2].measured == 1); // fast: one of two windows has a speed
    CHECK(r.flags[2].run == Check::clear);
    std::vector<WindowVerdict> none{v};
    CHECK(summarize_run(none, true).flags[2].run == Check::not_measured);
}

TEST_CASE("window verdict: the run summary's counts, shares, first and worst window, coverage and triggers")
{
    // 200 windows: two bent (exactly 1 %: clear), then a third (1.5 %: trigger); one bent severely
    std::vector<WindowVerdict> windows;
    for (int k = 1; k <= 200; k++)
        windows.push_back(judge_window(evidence(k, k == 50 ? 4.0 : (k == 120 ? -6.0 : 0.2))));
    RunVerdict r = summarize_run(windows, true);
    CHECK(r.windows == 200);
    CHECK(r.flags[0].flagged == 2);
    CHECK(r.flags[0].run == Check::clear);
    CHECK(r.flags[0].first_window == 50);
    CHECK(r.flags[0].worst_window == 120);
    CHECK(r.flags[0].worst_value == 6.0);
    CHECK(r.flags[1].run == Check::clear);
    windows[150] = judge_window(evidence(151, 12.0));
    r = summarize_run(windows, false);
    CHECK(r.flags[0].flagged == 3);
    CHECK(r.flags[0].run == Check::trigger);
    CHECK(r.flags[1].flagged == 1);
    CHECK(r.flags[1].run == Check::trigger);
    CHECK_FALSE(r.complete);
}

TEST_CASE("window verdict: the velocity chi2 over the weak windows and over all; percentiles as numpy")
{
    CHECK(percentile_linear({1.0, 2.0, 3.0, 4.0}, 50.0) == 2.5);
    CHECK(percentile_linear({1.0, 2.0, 3.0, 4.0, 5.0}, 95.0) == doctest::Approx(4.8));
    std::vector<WindowVerdict> windows;
    for (int k = 1; k <= 10; k++)
    {
        WindowEvidence e = evidence(k, 0.0);
        e.information_ratio = k <= 4 ? 0.2 : 0.8; // windows 1-4 are weak
        e.velocity_chi2 = k <= 4 ? 8.0 : 1.0;
        windows.push_back(judge_window(e));
    }
    const RunVerdict r = summarize_run(windows, true);
    CHECK(r.chi2_weak.measured == 4);
    CHECK(r.chi2_weak.median == 8.0);
    CHECK(r.chi2_run == Check::trigger); // median above 5
    CHECK(r.chi2_all.measured == 10);
    CHECK(r.chi2_all.median == 1.0);
    WindowEvidence without = evidence(11, 0.0);
    std::vector<WindowVerdict> unmeasured{judge_window(without)};
    CHECK(summarize_run(unmeasured, true).chi2_run == Check::not_measured);
}
