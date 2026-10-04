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
