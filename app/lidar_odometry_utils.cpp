#include "lidar_odometry_utils.h"
#include <filesystem>
#include <algorithm>

namespace fs = std::filesystem;

// this function provides unique index
unsigned long long int get_index(const int16_t x, const int16_t y, const int16_t z)
{
    return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
           ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
           ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
}

// this function provides unique index for input point p and 3D space decomposition into buckets b
unsigned long long int get_rgd_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    int16_t x = static_cast<int16_t>(p.x() / b.x());
    int16_t y = static_cast<int16_t>(p.y() / b.y());
    int16_t z = static_cast<int16_t>(p.z() / b.z());
    return get_index(x, y, z);
}

std::vector<Point3Di> decimate(const std::vector<Point3Di>& points, double bucket_x, double bucket_y, double bucket_z)
{
    Eigen::Vector3d b(bucket_x, bucket_y, bucket_z);
    std::vector<Point3Di> out;

    std::vector<PointCloud::PointBucketIndexPair> ip;
    ip.resize(points.size());
    out.reserve(points.size());

    for (int i = 0; i < points.size(); i++)
    {
        ip[i].index_of_point = i;
        ip[i].index_of_bucket = get_rgd_index(points[i].point, b);
    }
    std::sort(ip.begin(), ip.end(), [](const PointCloud::PointBucketIndexPair& a, const PointCloud::PointBucketIndexPair& b)
        { return a.index_of_bucket < b.index_of_bucket; });

    if (ip.size() != 0)
	    out.emplace_back(points[ip[0].index_of_point]);

    for (int i = 1; i < ip.size(); i++)
        if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket)
            out.emplace_back(points[ip[i].index_of_point]);

    return out;
}

// the previous cell statistics, kept unchanged behind use_exact_ndt_bucket_statistics = false: the mean is the cell's
// first point, the normal is fixed from the first 3 points, and later points update the covariance only if they pass
// the view-point test against that normal
static void update_rgd_legacy(const NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                              std::vector<Point3Di> &points_global, Eigen::Vector3d viewport)
{
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global[i].point, b);

        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it != buckets.end())
        {
            auto &this_bucket = bucket_it->second;
            this_bucket.number_of_points++;
            const auto &curr_mean = points_global[i].point;
            const auto &mean = this_bucket.mean;

            auto mean_diff = mean - curr_mean;
            Eigen::Matrix3d cov_update;
            cov_update.row(0) = mean_diff.x() * mean_diff;
            cov_update.row(1) = mean_diff.y() * mean_diff;
            cov_update.row(2) = mean_diff.z() * mean_diff;

            if (this_bucket.number_of_points == 2)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
            }

            if (this_bucket.number_of_points == 3)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                // calculate normal vector
                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(this_bucket.cov, Eigen::ComputeEigenvectors);
                Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();

                Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
                nv.normalize();

                // flip towards viewport
                if (nv.dot(viewport - this_bucket.mean) < 0.0)
                {
                    nv *= -1.0;
                }
                this_bucket.normal_vector = nv;
            }

            if (this_bucket.number_of_points > 3)
            {
                Eigen::Vector3d &nv = this_bucket.normal_vector;

                if (nv.dot(viewport - this_bucket.mean) >= 0.0)
                {
                    this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                      cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
                }
            }
        }
        else
        {
            NDT::Bucket bucket_to_add;
            bucket_to_add.mean = points_global[i].point;
            bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03; //ToDo move to params
            bucket_to_add.number_of_points = 1;
            bucket_to_add.index_begin = 0;
            bucket_to_add.index_end = 0;
            // no normal until the 3rd point; must not be left uninitialized (it is read by the viewpoint check)
            bucket_to_add.normal_vector = Eigen::Vector3d::Zero();
            buckets.emplace(index_of_bucket, bucket_to_add);
        }
    }
}

void update_rgd(const NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global, Eigen::Vector3d viewport, bool exact)
{
    update_rgd_legacy(rgd_params, buckets, points_global, viewport);
}

void update_rgd_spherical_coordinates(const NDT::GridParameters &rgd_params, NDTBucketMapType &buckets,
                                      std::vector<Point3Di> &points_global, std::vector<Eigen::Vector3d> &points_global_spherical)
{
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);

    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = get_rgd_index(points_global_spherical[i], b);

        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it != buckets.end())
        {
            auto &this_bucket = bucket_it->second;
            this_bucket.number_of_points++;
            const auto &curr_mean = points_global[i].point;
            const auto &mean = this_bucket.mean;

            auto mean_diff = mean - curr_mean;
            Eigen::Matrix3d cov_update;
            cov_update.row(0) = mean_diff.x() * mean_diff;
            cov_update.row(1) = mean_diff.y() * mean_diff;
            cov_update.row(2) = mean_diff.z() * mean_diff;

            if (this_bucket.number_of_points == 2)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
            }

            if (this_bucket.number_of_points >= 3)
            {
                this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                                  cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
            }
        }
        else
        {
            NDT::Bucket bucket_to_add;
            bucket_to_add.mean = points_global[i].point;
            bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03;
            bucket_to_add.number_of_points = 1;
            bucket_to_add.index_begin = 0;
            bucket_to_add.index_end = 0;
            // no normal until the 3rd point; must not be left uninitialized (it is read by the viewpoint check)
            bucket_to_add.normal_vector = Eigen::Vector3d::Zero();
            buckets.emplace(index_of_bucket, bucket_to_add);
        }
    }
}
