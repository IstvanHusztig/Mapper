#include <doctest/doctest.h>

#include "normal_equations.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace
{
    // The dense reference of BandedNormalMatrix::to_sparse (the band-only copy the optimizer used before BandedNormalMatrix).
    // Pose part: only parameters up to max_offset apart; columns and rows from n_pose_params on (the extra unknowns,
    // such as the gyro bias) are copied completely.
    Eigen::SparseMatrix<double> pose_band_to_sparse(const Eigen::MatrixXd &m, int n_pose_params, int max_offset = 17)
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
}

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

namespace
{
    // the same additions to a dense matrix and to the banded one, in the same order
    struct DenseAndBand
    {
        Eigen::MatrixXd dense;
        BandedNormalMatrix band;
        int n_pose;

        DenseAndBand(int n_pose_params, int n_extra, int half_width)
            : dense(Eigen::MatrixXd::Zero(n_pose_params + n_extra, n_pose_params + n_extra)), band(n_pose_params, n_extra, half_width), n_pose(n_pose_params)
        {
        }

        void add(int r, int c, double v)
        {
            dense(r, c) += v;
            band.add(r, c, v);
        }
    };

    // the banded matrix as a sparse matrix is the dense one's band conversion: same pattern, same bits
    void check_same_as_dense(const DenseAndBand &x, int half_width)
    {
        const Eigen::SparseMatrix<double> reference = pose_band_to_sparse(x.dense, x.n_pose, half_width);
        const Eigen::SparseMatrix<double> s = x.band.to_sparse();
        REQUIRE(s.isCompressed());
        REQUIRE(reference.isCompressed());
        REQUIRE(s.nonZeros() == reference.nonZeros());
        bool same_pattern = true, same_values = true;
        for (int k = 0; k <= s.outerSize(); k++)
            same_pattern = same_pattern && s.outerIndexPtr()[k] == reference.outerIndexPtr()[k];
        for (int k = 0; k < s.nonZeros(); k++)
        {
            same_pattern = same_pattern && s.innerIndexPtr()[k] == reference.innerIndexPtr()[k];
            same_values = same_values && s.valuePtr()[k] == reference.valuePtr()[k];
        }
        CHECK(same_pattern);
        CHECK(same_values);
    }
}

TEST_CASE("BandedNormalMatrix: the sums are those of the dense matrix, bit for bit")
{
    for (const int n_extra : {0, 3, 6})
        for (const int half_width : {17, 113})
        {
            CAPTURE(n_extra);
            CAPTURE(half_width);
            const int n_nodes = 40;
            const int n_pose = n_nodes * 6;
            DenseAndBand x(n_pose, n_extra, half_width);
            std::mt19937 rng(5);
            std::uniform_real_distribution<double> value(-1e3, 1e3);
            std::uniform_int_distribution<int> any_node(0, n_nodes - 1);
            std::uniform_int_distribution<int> any_extra(0, std::max(0, n_extra - 1));

            for (int round = 0; round < 3; round++)
            {
                // diagonal 6x6 blocks, several additions per block (the NDT point terms)
                for (int i = 0; i < 400; i++)
                {
                    const int k = any_node(rng);
                    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> block;
                    for (int r = 0; r < 6; r++)
                        for (int c = 0; c < 6; c++)
                            block(r, c) = value(rng);
                    for (int r = 0; r < 6; r++)
                        for (int c = 0; c < 6; c++)
                            x.dense(k * 6 + r, k * 6 + c) += block(r, c);
                    x.band.add_diagonal_block6(k * 6, block);
                }
                // edges between neighbouring nodes, all four 6x6 blocks (the motion model)
                for (int k = 1; k < n_nodes; k++)
                    for (int r = 0; r < 6; r++)
                        for (int c = 0; c < 6; c++)
                        {
                            x.add((k - 1) * 6 + r, (k - 1) * 6 + c, value(rng));
                            x.add((k - 1) * 6 + r, k * 6 + c, value(rng));
                            x.add(k * 6 + r, (k - 1) * 6 + c, value(rng));
                            x.add(k * 6 + r, k * 6 + c, value(rng));
                        }
                // couplings with the extra unknowns, both triangles, and their own blocks
                for (int j = 0; j < n_extra; j++)
                {
                    for (int i = 0; i < 100; i++)
                    {
                        const int row = any_node(rng) * 6 + (i % 6);
                        const double v = value(rng);
                        x.add(row, n_pose + j, v);
                        x.add(n_pose + j, row, v);
                    }
                    for (int k = 0; k < n_extra; k++)
                        x.add(n_pose + j, n_pose + k, value(rng));
                }
                // terms between nodes up to the band apart (the accelerometer term), within the band
                for (int i = 0; i < 300; i++)
                {
                    const int a = any_node(rng) * 6 + (i % 3);
                    const int b = std::clamp(a + (i % 2 ? 1 : -1) * (i % (half_width + 1)), 0, n_pose - 1);
                    x.add(a, b, value(rng));
                }
            }
            CHECK(x.band.dropped_writes() == 0);
            check_same_as_dense(x, half_width);

            // reads give the dense values
            bool reads_equal = true;
            for (int r = 0; r < x.band.size(); r += 7)
                for (int c = 0; c < x.band.size(); c += 5)
                    if (std::abs(r - c) <= half_width || r >= n_pose || c >= n_pose)
                        reads_equal = reads_equal && x.band(r, c) == x.dense(r, c);
            CHECK(reads_equal);
        }
}

// Pristine elements, so the result is an exact zero however the compiler orders the two additions (with /fp:fast it may
// fold an adjacent +v, -v pair on a non-zero element in one matrix and not in the other).
TEST_CASE("BandedNormalMatrix: an element that cancels to exactly zero is not stored")
{
    DenseAndBand x(60, 3, 17);
    x.add(7, 9, 123.456);
    x.add(7, 9, -123.456);
    x.add(60, 4, 5.0);
    x.add(60, 4, -5.0);
    x.add(8, 62, 0.25);
    x.add(8, 62, -0.25);
    x.add(20, 21, 1.5); // one that stays
    CHECK(x.band.to_sparse().nonZeros() == 1);
    check_same_as_dense(x, 17);
}

TEST_CASE("BandedNormalMatrix: a pose-pose write outside the band is dropped and counted, like the dense band conversion")
{
    const int half_width = 17;
    DenseAndBand x(60, 3, half_width);
    x.add(0, 10, 1.0);
    x.add(10, 0, 1.0);
    x.add(30, 30 + half_width, 2.0);       // the last element inside the band
    x.add(30 + half_width, 30, 2.0);
    x.add(0, 40, 5.0);                     // outside
    x.add(40, 0, 5.0);                     // outside
    x.add(0, 60, 7.0);                     // a pose row with an extra column is never outside
    x.add(60, 59, 7.0);
    CHECK(x.band.dropped_writes() == 2);
    check_same_as_dense(x, half_width);
    CHECK(x.band(0, 40) == 0.0);
}

TEST_CASE("BandedNormalMatrix: a band wider than the matrix covers all of it")
{
    DenseAndBand x(12, 3, 100);
    CHECK(x.band.band() == 11);
    for (int r = 0; r < 15; r++)
        for (int c = 0; c < 15; c++)
            x.add(r, c, r * 100.0 + c + 1.0);
    CHECK(x.band.dropped_writes() == 0);
    check_same_as_dense(x, 100);
}
