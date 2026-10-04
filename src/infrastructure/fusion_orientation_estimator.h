#pragma once

// OrientationEstimator adapter for the Fusion AHRS (infrastructure).

#include "service_ports.h"

#include <Fusion.h>

class FusionOrientationEstimator : public OrientationEstimator
{
public:
    FusionOrientationEstimator();

    void reset(AxesConvention convention, float gain) override;
    void set_gain(float gain) override;
    void update(const Eigen::Vector3f &gyro_deg_s, const Eigen::Vector3f &acc_g, float dt_s) override;
    Eigen::Quaternionf orientation() const override;

private:
    FusionAhrs ahrs_;
};
