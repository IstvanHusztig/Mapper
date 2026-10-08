#pragma once

// Normal-equation helpers of the least-squares optimizers.

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

//! Structure holds block of Hessian from observations to be sumed
struct Blocks
{
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPAndtBlocksToSum;
    Eigen::Matrix<double, 6, 1> AtPBndtBlocksToSum;
    int c = 0;
};

// adds each block to AtPAndt at (c, c) and subtracts its right-hand side from AtPBndt at c
void SumBlocks(const std::vector<Blocks> &blocks, Eigen::MatrixXd &AtPAndt, Eigen::MatrixXd &AtPBndt);

// sparse copy of the band of a normal matrix (see normal_equations.cpp)
Eigen::SparseMatrix<double> pose_band_to_sparse(const Eigen::MatrixXd &m, int n_pose_params, int max_offset = 17);

// The normal matrix of a window, stored by band instead of densely. Unknowns: n_pose_params pose parameters, then n_extra
// extra ones (gyro bias, acceleration offset). The pose-pose part couples parameters up to `band` apart; the rows and
// columns of the extra unknowns are stored completely, both triangles separately (callers add to both). Each element
// receives exactly the additions it is given, in the order it gets them, so a sum is the same bits as in a dense matrix
// that gets the same additions. A pose-pose write outside the band is dropped, as the dense matrix's band conversion
// never read it, and counted in dropped_writes() (a nonzero count means the band was chosen too narrow).
class BandedNormalMatrix
{
public:
    BandedNormalMatrix(int n_pose_params, int n_extra, int band);

    int size() const { return n_pose_ + n_extra_; }
    int band() const { return band_; }
    long long dropped_writes() const { return dropped_; }

    void add(int row, int col, double value);
    // block(i, j) is added at (c + i, c + j); the 6x6 block must lie within the band (band >= 5)
    void add_diagonal_block6(int c, const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &block);
    double operator()(int row, int col) const; // 0.0 for a pose-pose element outside the band

    // the nonzero elements as a sparse matrix; equal to pose_band_to_sparse of the dense matrix with the same additions
    Eigen::SparseMatrix<double> to_sparse() const;

private:
    int n_pose_;
    int n_extra_;
    int band_;
    int width_;                    // 2 * band_ + 1
    std::vector<double> pose_pose_; // row r, column c at r * width_ + (c - r + band_)
    std::vector<double> pose_extra_; // pose row r, extra column j at r * n_extra_ + j
    std::vector<double> extra_pose_; // extra row j, pose column c at j * n_pose_ + c
    std::vector<double> extra_extra_; // extra row j, extra column k at j * n_extra_ + k
    long long dropped_ = 0;
};
