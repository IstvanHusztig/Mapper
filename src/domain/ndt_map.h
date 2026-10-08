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

// The inverse covariance of a cell as the point loops use it, and whether the cell may be used at all (an entry of the
// inverse beyond +-100000 means a nearly flat cell and the cell is skipped). update_rgd stores both in the cell once, so
// the loops don't invert the same 3x3 matrix for every point in every iteration; a cell that wasn't through update_rgd
// is inverted on the spot, with the same expression, so the values are the same either way.
inline bool ndt_inverse_usable(const Eigen::Matrix3d &infm)
{
    const double threshold = 100000.0;
    return !((infm.array() > threshold).any() || (infm.array() < -threshold).any());
}

inline void refresh_cell_inverse(NdtBucket &bucket)
{
    bucket.cov_inverse = bucket.cov.inverse();
    bucket.inverse_usable = ndt_inverse_usable(bucket.cov_inverse);
    bucket.inverse_computed = true;
}

// nullptr if the cell must be skipped; otherwise the inverse (the cell's own or `scratch`)
inline const Eigen::Matrix3d *ndt_cell_inverse(const NdtBucket &bucket, Eigen::Matrix3d &scratch)
{
    if (bucket.inverse_computed)
        return bucket.inverse_usable ? &bucket.cov_inverse : nullptr;
    scratch = bucket.cov.inverse();
    return ndt_inverse_usable(scratch) ? &scratch : nullptr;
}

unsigned long long int ndt_cell_key(const int16_t x, const int16_t y, const int16_t z);
unsigned long long int ndt_cell_index(const Eigen::Vector3d p, const Eigen::Vector3d b);

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
