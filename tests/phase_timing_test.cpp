#include <doctest/doctest.h>

#include "phase_timing.h"

#include <thread>

TEST_CASE("phase timer: a null target does nothing")
{
    PhaseTimer timer(nullptr);
    timer.stop();
    timer.stop();
}

TEST_CASE("phase timer: adds one call and the elapsed time")
{
    PhaseTime t;
    {
        PhaseTimer timer(&t);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    CHECK(t.calls == 1);
    CHECK(t.seconds >= 0.001);
}

TEST_CASE("phase timer: stop() counts once, timers on the same target add up")
{
    PhaseTime t;
    {
        PhaseTimer timer(&t);
        timer.stop();
        const double after_stop = t.seconds;
        CHECK(t.calls == 1);
        CHECK(after_stop >= 0.0);
        timer.stop();
        CHECK(t.calls == 1);
        CHECK(t.seconds == after_stop);
    }
    CHECK(t.calls == 1);
    {
        PhaseTimer timer(&t);
    }
    CHECK(t.calls == 2);
    CHECK(t.seconds >= 0.0);
}
