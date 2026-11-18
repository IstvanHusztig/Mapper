#include "ndt_map.h"
#include <algorithm>
#include <cmath>
#include <utility>

// this function provides unique index
unsigned long long int ndt_cell_key(const int16_t x, const int16_t y, const int16_t z)
{
    return ((static_cast<unsigned long long int>(x) << 32) & (0x0000FFFF00000000ull)) |
           ((static_cast<unsigned long long int>(y) << 16) & (0x00000000FFFF0000ull)) |
           ((static_cast<unsigned long long int>(z) << 0) & (0x000000000000FFFFull));
}

// this function provides unique index for input point p and 3D space decomposition into buckets b
unsigned long long int ndt_cell_index(const Eigen::Vector3d p, const Eigen::Vector3d b)
{
    // floor, not truncation towards zero: truncation gave the cells through 0 (on each axis) double width
    int16_t x = static_cast<int16_t>(std::floor(p.x() / b.x()));
    int16_t y = static_cast<int16_t>(std::floor(p.y() / b.y()));
    int16_t z = static_cast<int16_t>(std::floor(p.z() / b.z()));
    return ndt_cell_key(x, y, z);
}

// a point's index and its cell, as PointCloud::PointBucketIndexPair (same layout), without including point_cloud.h
struct PointBucketIndex
{
    int index_of_point = 0;
    long long unsigned int index_of_bucket = 0;
};

std::vector<Point3Di> decimate(const std::vector<Point3Di>& points, double bucket_x, double bucket_y, double bucket_z)
{
    Eigen::Vector3d b(bucket_x, bucket_y, bucket_z);
    std::vector<Point3Di> out;

    std::vector<PointBucketIndex> ip;
    ip.resize(points.size());
    out.reserve(points.size());

    for (int i = 0; i < points.size(); i++)
    {
        ip[i].index_of_point = i;
        ip[i].index_of_bucket = ndt_cell_index(points[i].point, b);
    }
    std::sort(ip.begin(), ip.end(), [](const PointBucketIndex& a, const PointBucketIndex& b)
        { return a.index_of_bucket < b.index_of_bucket; });

    if (ip.size() != 0)
	    out.emplace_back(points[ip[0].index_of_point]);

    for (int i = 1; i < ip.size(); i++)
        if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket)
            out.emplace_back(points[ip[i].index_of_point]);

    return out;
}

namespace
{
    constexpr double ambiguous_normal_ratio = 3.0;  // no normal if the two smallest spreads are closer than this factor

    // the cells an update_rgd call added or changed get their inverse covariance, each once, in key order
    void refresh_cell_inverses(NDTBucketMapType &buckets, std::vector<uint64_t> cells)
    {
        std::sort(cells.begin(), cells.end());
        cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
        for (const auto index : cells)
            refresh_cell_inverse(buckets.at(index));
    }

    // the previous cell statistics (use_exact_ndt_bucket_statistics = false), split into its steps; every expression is
    // the one of the single function they came from, in the same order, so not one bit changes

    // a new cell: its first point is the mean (for good), the covariance starts isotropic, no normal until the 3rd point
    NdtBucket new_legacy_cell(const Eigen::Vector3d &point)
    {
        NdtBucket bucket_to_add;
        bucket_to_add.mean = point;
        bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03; //ToDo move to params
        bucket_to_add.number_of_points = 1;
        bucket_to_add.index_begin = 0;
        bucket_to_add.index_end = 0;
        // must not be left uninitialized (it is read by the viewpoint check)
        bucket_to_add.normal_vector = Eigen::Vector3d::Zero();
        return bucket_to_add;
    }

    // the running covariance about the cell's fixed mean, with number_of_points already counting the new point;
    // cov_update is (mean - point)(mean - point)^T
    void add_to_legacy_cov(NdtBucket &this_bucket, const Eigen::Matrix3d &cov_update)
    {
        this_bucket.cov = this_bucket.cov * (this_bucket.number_of_points - 1) / this_bucket.number_of_points +
                          cov_update * (this_bucket.number_of_points - 1) / (this_bucket.number_of_points * this_bucket.number_of_points);
    }

    // the normal fixed at the 3rd point: the cross product of the eigenvectors of the two largest spreads, turned
    // towards the viewport
    Eigen::Vector3d legacy_cell_normal(const Eigen::Matrix3d &cov, const Eigen::Vector3d &mean, const Eigen::Vector3d &viewport)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(cov, Eigen::ComputeEigenvectors);
        Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();

        Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
        nv.normalize();

        if (nv.dot(viewport - mean) < 0.0)
        {
            nv *= -1.0;
        }
        return nv;
    }

    // one more point in an existing cell: counted always (C33); the 2nd and 3rd update the covariance, the 3rd fixes the
    // normal, later ones update the covariance only if the cell's normal faces this call's viewport
    void add_point_to_legacy_cell(NdtBucket &this_bucket, const Eigen::Vector3d &curr_mean, const Eigen::Vector3d &viewport)
    {
        this_bucket.number_of_points++;
        const auto &mean = this_bucket.mean;

        auto mean_diff = mean - curr_mean;
        Eigen::Matrix3d cov_update;
        cov_update.row(0) = mean_diff.x() * mean_diff;
        cov_update.row(1) = mean_diff.y() * mean_diff;
        cov_update.row(2) = mean_diff.z() * mean_diff;

        if (this_bucket.number_of_points == 2)
        {
            add_to_legacy_cov(this_bucket, cov_update);
        }

        if (this_bucket.number_of_points == 3)
        {
            add_to_legacy_cov(this_bucket, cov_update);
            this_bucket.normal_vector = legacy_cell_normal(this_bucket.cov, this_bucket.mean, viewport);
        }

        if (this_bucket.number_of_points > 3)
        {
            Eigen::Vector3d &nv = this_bucket.normal_vector;

            if (nv.dot(viewport - this_bucket.mean) >= 0.0)
            {
                add_to_legacy_cov(this_bucket, cov_update);
            }
        }
    }
}

// the previous cell statistics, kept unchanged behind use_exact_ndt_bucket_statistics = false: the mean is the cell's
// first point, the normal is fixed from the first 3 points, and later points update the covariance only if they pass
// the view-point test against that normal
static void update_rgd_legacy(const NdtGridParameters &rgd_params, NDTBucketMapType &buckets,
                              std::vector<Point3Di> &points_global, Eigen::Vector3d viewport)
{
    Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);
    std::vector<uint64_t> changed; // the cells whose covariance may have changed, refreshed once at the end

    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = ndt_cell_index(points_global[i].point, b);
        changed.push_back(index_of_bucket);

        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it != buckets.end())
            add_point_to_legacy_cell(bucket_it->second, points_global[i].point, viewport);
        else
            buckets.emplace(index_of_bucket, new_legacy_cell(points_global[i].point));
    }

    refresh_cell_inverses(buckets, std::move(changed));
}

void update_rgd(const NdtGridParameters &rgd_params, NDTBucketMapType &buckets,
                std::vector<Point3Di> &points_global, Eigen::Vector3d viewport, bool exact)
{
    if (!exact)
    {
        update_rgd_legacy(rgd_params, buckets, points_global, viewport);
        return;
    }

    // exact statistics: the running mean (Welford) and the covariance about it, with the initial isotropic covariance
    // as a prior worth one point; the normals of the cells this call changed are recomputed once, after all points
    const Eigen::Vector3d b(rgd_params.resolution_X, rgd_params.resolution_Y, rgd_params.resolution_Z);
    std::vector<uint64_t> touched;
    std::vector<uint64_t> inserted;

    for (const auto &p : points_global)
    {
        const auto index_of_bucket = ndt_cell_index(p.point, b);
        auto bucket_it = buckets.find(index_of_bucket);

        if (bucket_it == buckets.end())
        {
            NdtBucket bucket_to_add;
            bucket_to_add.mean = p.point;
            bucket_to_add.cov = Eigen::Matrix3d::Identity() * 0.03 * 0.03;
            bucket_to_add.number_of_points = 1;
            bucket_to_add.index_begin = 0;
            bucket_to_add.index_end = 0;
            bucket_to_add.normal_vector = Eigen::Vector3d::Zero(); // no normal below 3 points
            buckets.emplace(index_of_bucket, bucket_to_add);
            inserted.push_back(index_of_bucket);
            continue;
        }

        auto &bucket = bucket_it->second;
        // the far side of a thin wall: a normal from an earlier call that faces away from this call's viewport
        // (a zero normal passes)
        if (bucket.normal_vector.dot(viewport - bucket.mean) < 0.0)
            continue;

        const double n = static_cast<double>(bucket.number_of_points);
        const Eigen::Vector3d d = p.point - bucket.mean;
        bucket.mean += d / (n + 1.0);
        // (x - mean_old)(x - mean_new)^T, written symmetric: mean_new = mean_old + d / (n + 1)
        bucket.cov = (n * bucket.cov + (n / (n + 1.0)) * d * d.transpose()) / (n + 1.0);
        bucket.number_of_points++;
        touched.push_back(index_of_bucket);
    }

    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    for (const auto index : touched)
    {
        auto &bucket = buckets.at(index);
        if (bucket.number_of_points < 3)
            continue;
        // the direction of the smallest spread (eigenvalues ascending), flipped towards the viewport; none if the two
        // smallest spreads are too close to tell apart, as for points on one line or one cross-section of the cell
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(bucket.cov, Eigen::ComputeEigenvectors);
        if (eigen_solver.eigenvalues()(1) < ambiguous_normal_ratio * eigen_solver.eigenvalues()(0))
        {
            bucket.normal_vector = Eigen::Vector3d::Zero();
            continue;
        }
        Eigen::Vector3d nv = eigen_solver.eigenvectors().col(0).normalized();
        if (nv.dot(viewport - bucket.mean) < 0.0)
            nv *= -1.0;
        bucket.normal_vector = nv;
    }

    // the cells this call added or changed get their inverse covariance (a cell only changes its cov through the loop above)
    touched.insert(touched.end(), inserted.begin(), inserted.end());
    refresh_cell_inverses(buckets, std::move(touched));
}
