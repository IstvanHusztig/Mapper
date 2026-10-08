#include "normal_equations.h"

#include <algorithm>

//! Sumation of blocks
void SumBlocks(const std::vector<Blocks> &blocks, Eigen::MatrixXd &AtPAndt, Eigen::MatrixXd &AtPBndt)
{
    for (auto &block : blocks)
    {
        const int c = block.c;
        AtPAndt.block<6, 6>(c, c) += block.AtPAndtBlocksToSum;
        AtPBndt.block<6, 1>(c, 0) -= block.AtPBndtBlocksToSum;
    }
};

BandedNormalMatrix::BandedNormalMatrix(int n_pose_params, int n_extra, int band)
    : n_pose_(n_pose_params), n_extra_(n_extra), band_(std::max(0, std::min(band, n_pose_params - 1))), width_(2 * band_ + 1)
{
    pose_pose_.assign(static_cast<size_t>(n_pose_) * width_, 0.0);
    pose_extra_.assign(static_cast<size_t>(n_pose_) * n_extra_, 0.0);
    extra_pose_.assign(static_cast<size_t>(n_extra_) * n_pose_, 0.0);
    extra_extra_.assign(static_cast<size_t>(n_extra_) * n_extra_, 0.0);
    w_pose_pose_.assign(pose_pose_.size(), 0);
    w_pose_extra_.assign(pose_extra_.size(), 0);
    w_extra_pose_.assign(extra_pose_.size(), 0);
    w_extra_extra_.assign(extra_extra_.size(), 0);
}

void BandedNormalMatrix::add(int row, int col, double value)
{
    size_t index;
    std::vector<double> *values;
    std::vector<unsigned char> *written;
    if (row < n_pose_ && col < n_pose_)
    {
        const int offset = col - row;
        if (offset > band_ || offset < -band_)
        {
            dropped_++;
            return;
        }
        index = static_cast<size_t>(row) * width_ + offset + band_;
        values = &pose_pose_;
        written = &w_pose_pose_;
    }
    else if (row < n_pose_)
    {
        index = static_cast<size_t>(row) * n_extra_ + (col - n_pose_);
        values = &pose_extra_;
        written = &w_pose_extra_;
    }
    else if (col < n_pose_)
    {
        index = static_cast<size_t>(row - n_pose_) * n_pose_ + col;
        values = &extra_pose_;
        written = &w_extra_pose_;
    }
    else
    {
        index = static_cast<size_t>(row - n_pose_) * n_extra_ + (col - n_pose_);
        values = &extra_extra_;
        written = &w_extra_extra_;
    }
    (*values)[index] += value;
    (*written)[index] = 1;
}

void BandedNormalMatrix::add_diagonal_block6(int c, const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &block)
{
    for (int i = 0; i < 6; i++)
    {
        double *row = &pose_pose_[static_cast<size_t>(c + i) * width_ + band_ - i];
        unsigned char *wrow = &w_pose_pose_[static_cast<size_t>(c + i) * width_ + band_ - i];
        for (int j = 0; j < 6; j++)
        {
            row[j] += block(i, j);
            wrow[j] = 1;
        }
    }
}

double BandedNormalMatrix::operator()(int row, int col) const
{
    if (row < n_pose_)
    {
        if (col < n_pose_)
        {
            const int offset = col - row;
            return (offset > band_ || offset < -band_) ? 0.0 : pose_pose_[static_cast<size_t>(row) * width_ + offset + band_];
        }
        return pose_extra_[static_cast<size_t>(row) * n_extra_ + (col - n_pose_)];
    }
    if (col < n_pose_)
        return extra_pose_[static_cast<size_t>(row - n_pose_) * n_pose_ + col];
    return extra_extra_[static_cast<size_t>(row - n_pose_) * n_extra_ + (col - n_pose_)];
}

Eigen::SparseMatrix<double> BandedNormalMatrix::to_sparse() const
{
    const int n = size();
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(n_pose_) * width_ + static_cast<size_t>(n_extra_) * n * 2);
    for (int col = 0; col < n; col++)
    {
        if (col >= n_pose_)
        {
            for (int row = 0; row < n; row++)
            {
                const double v = (*this)(row, col);
                const bool w = row < n_pose_ ? w_pose_extra_[static_cast<size_t>(row) * n_extra_ + (col - n_pose_)] : w_extra_extra_[static_cast<size_t>(row - n_pose_) * n_extra_ + (col - n_pose_)];
                if (w)
                    triplets.emplace_back(row, col, v);
            }
            continue;
        }
        const int row_end = std::min(n_pose_, col + band_ + 1);
        for (int row = std::max(0, col - band_); row < row_end; row++)
        {
            const double v = pose_pose_[static_cast<size_t>(row) * width_ + (col - row) + band_];
            if (w_pose_pose_[static_cast<size_t>(row) * width_ + (col - row) + band_])
                triplets.emplace_back(row, col, v);
        }
        for (int row = n_pose_; row < n; row++)
        {
            const double v = extra_pose_[static_cast<size_t>(row - n_pose_) * n_pose_ + col];
            if (w_extra_pose_[static_cast<size_t>(row - n_pose_) * n_pose_ + col])
                triplets.emplace_back(row, col, v);
        }
    }
    Eigen::SparseMatrix<double> s(n, n);
    s.setFromTriplets(triplets.begin(), triplets.end());
    return s;
}

void PatternCachedCholesky::compute(const Eigen::SparseMatrix<double> &a)
{
    calls_++;
    bool same_pattern = analyzed_ && a.isCompressed() && a.rows() == rows_ && a.cols() == cols_ &&
                        outer_.size() == static_cast<size_t>(a.cols()) + 1 && inner_.size() == static_cast<size_t>(a.nonZeros());
    if (same_pattern)
        same_pattern = std::equal(outer_.begin(), outer_.end(), a.outerIndexPtr()) &&
                       std::equal(inner_.begin(), inner_.end(), a.innerIndexPtr());
    if (!same_pattern)
    {
        solver_.analyzePattern(a);
        analyses_++;
        analyzed_ = a.isCompressed();
        if (analyzed_)
        {
            rows_ = static_cast<int>(a.rows());
            cols_ = static_cast<int>(a.cols());
            outer_.assign(a.outerIndexPtr(), a.outerIndexPtr() + a.cols() + 1);
            inner_.assign(a.innerIndexPtr(), a.innerIndexPtr() + a.nonZeros());
        }
    }
    solver_.factorize(a);
}
