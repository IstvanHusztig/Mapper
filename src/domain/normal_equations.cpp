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
}

void BandedNormalMatrix::add(int row, int col, double value)
{
    if (row < n_pose_)
    {
        if (col < n_pose_)
        {
            const int offset = col - row;
            if (offset > band_ || offset < -band_)
            {
                dropped_++;
                return;
            }
            pose_pose_[static_cast<size_t>(row) * width_ + offset + band_] += value;
        }
        else
            pose_extra_[static_cast<size_t>(row) * n_extra_ + (col - n_pose_)] += value;
    }
    else if (col < n_pose_)
        extra_pose_[static_cast<size_t>(row - n_pose_) * n_pose_ + col] += value;
    else
        extra_extra_[static_cast<size_t>(row - n_pose_) * n_extra_ + (col - n_pose_)] += value;
}

void BandedNormalMatrix::add_diagonal_block6(int c, const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> &block)
{
    for (int i = 0; i < 6; i++)
    {
        double *row = &pose_pose_[static_cast<size_t>(c + i) * width_ + band_ - i];
        for (int j = 0; j < 6; j++)
            row[j] += block(i, j);
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
                if (v != 0.0)
                    triplets.emplace_back(row, col, v);
            }
            continue;
        }
        const int row_end = std::min(n_pose_, col + band_ + 1);
        for (int row = std::max(0, col - band_); row < row_end; row++)
        {
            const double v = pose_pose_[static_cast<size_t>(row) * width_ + (col - row) + band_];
            if (v != 0.0)
                triplets.emplace_back(row, col, v);
        }
        for (int row = n_pose_; row < n; row++)
        {
            const double v = extra_pose_[static_cast<size_t>(row - n_pose_) * n_pose_ + col];
            if (v != 0.0)
                triplets.emplace_back(row, col, v);
        }
    }
    Eigen::SparseMatrix<double> s(n, n);
    s.setFromTriplets(triplets.begin(), triplets.end());
    return s;
}
