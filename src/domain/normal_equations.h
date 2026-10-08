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

// The normal matrix of a window, stored by band instead of densely. Unknowns: n_pose_params pose parameters, then n_extra
// extra ones (gyro bias, acceleration offset). The pose-pose part couples parameters up to `band` apart; the rows and
// columns of the extra unknowns are stored completely, both triangles separately (callers add to both). Each element
// receives exactly the additions it is given, in the order it gets them, so a sum is the same bits as in a dense matrix
// that gets the same additions. A pose-pose write outside the band is dropped, as the dense matrix's band conversion
// never read it, and counted in dropped_writes() (a nonzero count means the band was chosen too narrow).
// The sparse pattern is the set of elements that were written at all (also where the sum is exactly zero), so it
// depends on which terms were added and not on whether a value happens to cancel; that keeps it the same from one
// iteration to the next (see PatternCachedCholesky).
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

    // the written elements as a sparse matrix (explicit zeros included)
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
    // which elements were ever written: the sparse pattern is this structure, whatever the values are
    std::vector<unsigned char> w_pose_pose_, w_pose_extra_, w_extra_pose_, w_extra_extra_;
};

// A sparse Cholesky factorization that keeps the symbolic analysis (fill-reducing ordering and elimination tree) from one
// call to the next while the sparsity pattern of the matrix stays the same. The analysis depends only on the pattern, so
// the result is the same bits as analysing every time; it is repeated whenever the pattern differs from the last one.
class PatternCachedCholesky
{
public:
    using Solver = Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>>;

    void compute(const Eigen::SparseMatrix<double> &a);
    const Solver &solver() const { return solver_; }

    long long calls() const { return calls_; }
    long long analyses() const { return analyses_; }

private:
    Solver solver_;
    bool analyzed_ = false;
    int rows_ = 0;
    int cols_ = 0;
    std::vector<int> outer_; // the pattern of the last analysed matrix
    std::vector<int> inner_;
    long long calls_ = 0;
    long long analyses_ = 0;
};
