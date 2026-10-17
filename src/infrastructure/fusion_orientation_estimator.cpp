#include "fusion_orientation_estimator.h"

FusionOrientationEstimator::FusionOrientationEstimator()
{
    FusionAhrsInitialise(&ahrs_);
}

void FusionOrientationEstimator::reset(AxesConvention convention, float gain)
{
    FusionAhrsInitialise(&ahrs_);
    switch (convention)
    {
    case AxesConvention::Nwu:
        ahrs_.settings.convention = FusionConventionNwu;
        break;
    case AxesConvention::Enu:
        ahrs_.settings.convention = FusionConventionEnu;
        break;
    case AxesConvention::Ned:
        ahrs_.settings.convention = FusionConventionNed;
        break;
    }
    ahrs_.settings.gain = gain;
}

void FusionOrientationEstimator::set_gain(float gain)
{
    ahrs_.settings.gain = gain;
}

void FusionOrientationEstimator::update(const Eigen::Vector3f &gyro_deg_s, const Eigen::Vector3f &acc_g, float dt_s)
{
    const FusionVector gyroscope = {gyro_deg_s.x(), gyro_deg_s.y(), gyro_deg_s.z()};
    const FusionVector accelerometer = {acc_g.x(), acc_g.y(), acc_g.z()};
    // FusionAhrsUpdateNoMagnetometer without its heading reset during the start-up ramp: a turn made then stays in the
    // orientation (the windows start inside the ramp)
    const FusionVector no_magnetometer = {0.0f, 0.0f, 0.0f}; // FUSION_VECTOR_ZERO is a C compound literal
    FusionAhrsUpdate(&ahrs_, gyroscope, accelerometer, no_magnetometer, dt_s);
}

Eigen::Quaternionf FusionOrientationEstimator::orientation() const
{
    const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs_);
    return Eigen::Quaternionf(q.element.w, q.element.x, q.element.y, q.element.z);
}
