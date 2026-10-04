#pragma once

// Builds the services of a run (infrastructure): the Fusion AHRS and the oneTBB executor.

#include "service_ports.h"

RunServices make_run_services();
