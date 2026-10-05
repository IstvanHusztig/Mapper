#pragma once

// Angle conversion in double precision for the odometry optimizers (formerly defined in the optimizer file).
// The GUI viewer (src/presentation/gui/viewer.h) defines float constants with the same names: never include both in one
// translation unit.

#include <cmath>

const double DEG_TO_RAD = M_PI / 180.0f;
const double RAD_TO_DEG = 180.0f / M_PI;
