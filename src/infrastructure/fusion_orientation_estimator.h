#pragma once

// OrientationEstimator adapter for the Fusion AHRS (infrastructure).

#include "service_ports.h"

#include <Fusion.h>

class FusionOrientationEstimator : public OrientationEstimator
{
public:
    FusionOrientationEstimator();

    void reset(AxesConvention convention, float gain) override;
    void update(const Eigen::Vector3f &gyro_deg_s, const Eigen::Vector3f &acc_g, float dt_s) override;
    Eigen::Quaternionf orientation() const override;

private:
    FusionAhrs ahrs_{};
    // Public settings retain rejection angles in degrees; Fusion's internal
    // settings store transformed thresholds and cannot be passed back in.
    FusionAhrsSettings settings_{FusionConventionNwu, 0.5f, 90.0f, 90.0f, 0};
};
