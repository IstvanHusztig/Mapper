#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "structures.h"

// Hold-out evaluation mode ([evaluation] holdout_block_s): the points whose timestamps fall into odd blocks of block_s
// seconds are left out of every map and point term (odometry, loop closure, consistency) but still saved, so map
// sharpness can be measured on points no stage fitted (tools/holdout_sharpness.py). block_s = 0: off.
inline bool is_held_out(double timestamp_s, double block_s)
{
    return block_s > 0.0 && static_cast<long long>(std::floor(timestamp_s / block_s)) % 2 != 0;
}

inline void remove_held_out(std::vector<Point3Di> &points, double block_s)
{
    if (block_s > 0.0)
        points.erase(std::remove_if(points.begin(), points.end(), [block_s](const Point3Di &p) { return is_held_out(p.timestamp, block_s); }),
                     points.end());
}
