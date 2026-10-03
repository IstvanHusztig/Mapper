#include "normal_equations.h"

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

//! Sparse copy of a normal matrix whose pose part couples only parameters up to max_offset apart
//! (default: nodes up to two apart); columns and rows from n_pose_params on (extra unknowns such as the gyro bias)
//! are copied completely. Much cheaper than sparseView(), which scans every element of the dense matrix.
Eigen::SparseMatrix<double> pose_band_to_sparse(const Eigen::MatrixXd &m, int n_pose_params, int max_offset)
{
    const int n = static_cast<int>(m.rows());
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(n_pose_params) * (2 * max_offset + 1) + static_cast<size_t>(n - n_pose_params) * n * 2);
    for (int col = 0; col < n; col++)
    {
        const bool extra_col = col >= n_pose_params;
        const int row_begin = extra_col ? 0 : std::max(0, col - max_offset);
        const int row_end = extra_col ? n : std::min(n_pose_params, col + max_offset + 1);
        for (int row = row_begin; row < row_end; row++)
            if (m(row, col) != 0.0)
                triplets.emplace_back(row, col, m(row, col));
        if (!extra_col)
            for (int row = n_pose_params; row < n; row++)
                if (m(row, col) != 0.0)
                    triplets.emplace_back(row, col, m(row, col));
    }
    Eigen::SparseMatrix<double> s(n, n);
    s.setFromTriplets(triplets.begin(), triplets.end());
    return s;
}
