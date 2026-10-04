#include "run_services.h"

#include "fusion_orientation_estimator.h"

RunServices make_run_services()
{
    RunServices services;
    services.orientation = std::make_unique<FusionOrientationEstimator>();
    return services;
}
