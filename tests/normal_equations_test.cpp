#include <doctest/doctest.h>

#include "normal_equations.h"

TEST_CASE("pose_band_to_sparse keeps the pose band and the extra rows and columns")
{
    const int n_pose = 12;
    const int n = n_pose + 3; // 3 extra unknowns, e.g. the gyro bias
    const int max_offset = 2;
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(n, n);
    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++)
            if (r >= n_pose || c >= n_pose || std::abs(r - c) <= max_offset)
                m(r, c) = r * 100.0 + c + 1.0;

    const Eigen::SparseMatrix<double> s = pose_band_to_sparse(m, n_pose, max_offset);
    CHECK(Eigen::MatrixXd(s) == m);
    CHECK(s.nonZeros() == Eigen::SparseMatrix<double>(m.sparseView()).nonZeros());

    // pose entries outside the band are dropped
    Eigen::MatrixXd outside = m;
    outside(0, 10) = 5.0;
    outside(10, 0) = 5.0;
    CHECK(Eigen::MatrixXd(pose_band_to_sparse(outside, n_pose, max_offset)) == m);
}

TEST_CASE("SumBlocks adds each block at its offset and subtracts its right-hand side")
{
    std::vector<Blocks> blocks(2);
    blocks[0].AtPAndtBlocksToSum = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Identity() * 2.0;
    blocks[0].AtPBndtBlocksToSum = Eigen::Matrix<double, 6, 1>::Constant(1.0);
    blocks[0].c = 0;
    blocks[1].AtPAndtBlocksToSum = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Identity() * 3.0;
    blocks[1].AtPBndtBlocksToSum = Eigen::Matrix<double, 6, 1>::Constant(4.0);
    blocks[1].c = 6;

    Eigen::MatrixXd AtPA = Eigen::MatrixXd::Identity(12, 12);
    Eigen::MatrixXd AtPB = Eigen::MatrixXd::Zero(12, 1);
    SumBlocks(blocks, AtPA, AtPB);

    for (int i = 0; i < 6; i++)
    {
        CHECK(AtPA(i, i) == 3.0);
        CHECK(AtPA(6 + i, 6 + i) == 4.0);
        CHECK(AtPB(i, 0) == -1.0);
        CHECK(AtPB(6 + i, 0) == -4.0);
    }
    CHECK(AtPA(0, 6) == 0.0);
    CHECK(AtPA(6, 0) == 0.0);
}
