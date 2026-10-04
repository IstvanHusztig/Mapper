#include "run_services.h"

#include "fusion_orientation_estimator.h"
#include "tbb_executor.h"

RunServices make_run_services()
{
    RunServices services;
    services.orientation = std::make_unique<FusionOrientationEstimator>();
    services.parallel = std::make_unique<TbbExecutor>();
    return services;
}
