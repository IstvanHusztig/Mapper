#pragma once

// Service ports of a processing run (application layer): third-party algorithms the use cases call, behind interfaces
// so that the application doesn't depend on them.
//   OrientationEstimator - the AHRS filter that turns gyroscope + accelerometer samples into an orientation

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>

// the world axes the orientation is expressed in (north-west-up, east-north-up, north-east-down)
enum class AxesConvention
{
    Nwu,
    Enu,
    Ned
};

class OrientationEstimator
{
public:
    virtual ~OrientationEstimator() = default;
    // starts a fresh filter
    virtual void reset(AxesConvention convention, float gain) = 0;
    // the accelerometer's weight from the next update on (0: gyroscope only)
    virtual void set_gain(float gain) = 0;
    // one sample: angular velocity [deg/s], acceleration [g], time since the previous sample [s]
    virtual void update(const Eigen::Vector3f &gyro_deg_s, const Eigen::Vector3f &acc_g, float dt_s) = 0;
    virtual Eigen::Quaternionf orientation() const = 0;
};

// the services one run uses; built by the composition root (make_run_services)
struct RunServices
{
    std::unique_ptr<OrientationEstimator> orientation;

    bool complete() const { return orientation != nullptr; }
};
