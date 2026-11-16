#pragma once

// Service ports of a processing run (application layer): third-party algorithms the use cases call, behind interfaces
// so that the application doesn't depend on them.
//   OrientationEstimator - the AHRS filter that turns gyroscope + accelerometer samples into an orientation
//   ParallelExecutor     - runs independent pieces of work, possibly in parallel

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstddef>
#include <functional>
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
    // one sample: angular velocity [deg/s], acceleration [g], time since the previous sample [s]
    virtual void update(const Eigen::Vector3f &gyro_deg_s, const Eigen::Vector3f &acc_g, float dt_s) = 0;
    virtual Eigen::Quaternionf orientation() const = 0;
};

class ParallelExecutor
{
public:
    virtual ~ParallelExecutor() = default;
    // runs body(i) for every i in [0, n), possibly in parallel and in any order; returns when all are done
    virtual void for_each_index(std::size_t n, const std::function<void(std::size_t)> &body) = 0;
};

// the services one run uses; built by the composition root (make_run_services)
struct RunServices
{
    std::unique_ptr<OrientationEstimator> orientation;
    std::unique_ptr<ParallelExecutor> parallel;

    bool complete() const { return orientation && parallel; }
};
