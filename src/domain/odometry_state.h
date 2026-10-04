#ifndef _ODOMETRY_STATE_H_
#define _ODOMETRY_STATE_H_

#include <Eigen/Dense>
#include <vector>
#include <structures.h>
#include "ndt_map.h"
#include "odometry_config.h"

// what a run produces and consumes (the settings are in OdometryConfig)
struct OdometryState
{
    Eigen::Affine3d m_g = Eigen::Affine3d::Identity(); // pose of the first trajectory node
    std::vector<Point3Di> initial_points;
    double consecutive_distance = 0.0;
    std::vector<Point3Di> reference_points;
    NDTBucketMapType reference_buckets;
    double total_length_of_calculated_trajectory = 0.0;
    NDTBucketMapType buckets_indoor;
    NDTBucketMapType buckets_outdoor;
    RunPaths paths;
};

#endif
