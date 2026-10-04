#ifndef _NDT_MAP_H_
#define _NDT_MAP_H_

// the regular-grid NDT maps the odometry registers against: cell index, decimation and the map update

#include <Eigen/Dense>
#include <cstdint>
#include <unordered_map>
#include <vector>
#include <ndt_types.h>
#include <structures.h>

using NDTBucketMapType = std::unordered_map<uint64_t, NdtBucket>;
using NDTBucketMapType2 = std::unordered_map<uint64_t, NdtBucket2>;

unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z);
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b);

// this function reduces number of points by preserving only first point for each bucket {bucket_x, bucket_y, bucket_z}
std::vector<Point3Di> decimate(const std::vector<Point3Di> &points, double bucket_x, double bucket_y, double bucket_z);

// adds points to the cells of a regular-grid NDT map (mean, covariance, normal oriented towards viewport);
// exact = OdometryConfig::use_exact_ndt_bucket_statistics: true uses the statistics of all of a cell's points,
// false the previous ones
void update_rgd(const NdtGridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global, Eigen::Vector3d viewport, bool exact);

void update_rgd_spherical_coordinates(const NdtGridParameters &rgd_params, NDTBucketMapType &buckets,
                                      std::vector<Point3Di> &points_global, std::vector<Eigen::Vector3d> &points_global_spherical);

#endif
