#include <doctest/doctest.h>

#include "holdout.h"

TEST_CASE("hold-out: odd blocks are left out, even blocks kept, 0 keeps everything")
{
    CHECK_FALSE(is_held_out(0.004, 0.01)); // block 0
    CHECK(is_held_out(0.015, 0.01));       // block 1
    CHECK_FALSE(is_held_out(0.025, 0.01)); // block 2
    CHECK(is_held_out(-0.005, 0.01));      // block -1 (times before the recording's zero)
    CHECK_FALSE(is_held_out(0.015, 0.0));

    std::vector<Point3Di> points(6);
    for (int i = 0; i < 6; i++)
        points[i].timestamp = 0.005 + 0.01 * i; // blocks 0 .. 5
    std::vector<Point3Di> kept = points;
    remove_held_out(kept, 0.01);
    REQUIRE(kept.size() == 3);
    for (int i = 0; i < 3; i++)
        CHECK(kept[i].timestamp == doctest::Approx(0.005 + 0.02 * i));
    std::vector<Point3Di> all = points;
    remove_held_out(all, 0.0);
    CHECK(all.size() == 6);
}
