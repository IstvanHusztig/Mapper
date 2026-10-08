#ifndef _NDT_TYPES_H_
#define _NDT_TYPES_H_

// The value types of the NDT grids, without the rest of ndt.h (which pulls in point_cloud.h, GL and session.h).
// Class NDT keeps the old names as aliases (NDT::GridParameters, NDT::Bucket, NDT::BucketCoef, NDT::Bucket2).

#include <Eigen/Eigen>
#include <unordered_map>
#include <vector>

struct NdtGridParameters
{
	double bounding_box_min_X;
	double bounding_box_min_Y;
	double bounding_box_min_Z;
	double bounding_box_max_X;
	double bounding_box_max_Y;
	double bounding_box_max_Z;
	double bounding_box_extension;
	int number_of_buckets_X;
	int number_of_buckets_Y;
	int number_of_buckets_Z;
	long long unsigned int number_of_buckets;
	double resolution_X;
	double resolution_Y;
	double resolution_Z;
};

struct NdtBucket
{
	long long unsigned int index_begin;
	long long unsigned int index_end;
	long long unsigned int number_of_points;
	Eigen::Vector3d mean;
	Eigen::Matrix3d cov;
	Eigen::Vector3d normal_vector;
	// the inverse of cov and whether the cell may be used (see ndt_cell_inverse in ndt_map.h); filled by update_rgd,
	// valid only while cov is unchanged
	Eigen::Matrix3d cov_inverse = Eigen::Matrix3d::Zero();
	bool inverse_computed = false;
	bool inverse_usable = false;
};

struct NdtBucketCoef
{
	Eigen::Vector3d mean;
	Eigen::Matrix3d cov;
	Eigen::Vector3d normal_vector;
	std::vector<long long unsigned int> point_indexes;
	bool valid = false;
};

struct NdtBucket2
{
	int classification = 0; // 1 - ceiling, 2 - floor
	long long unsigned int index_begin_inclusive;
	long long unsigned int index_end_exclusive;
	// long long unsigned int number_of_points;
	std::unordered_map<long long unsigned int, NdtBucketCoef> buckets;
};

#endif
