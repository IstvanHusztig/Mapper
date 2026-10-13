#include <doctest/doctest.h>

#include "normal_equations.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace
{
    // The dense reference of BandedNormalMatrix::to_sparse (the band-only copy the optimizer used before BandedNormalMatrix),
    // taking the elements that were written (written != 0) instead of the nonzero ones.
    // Pose part: only parameters up to max_offset apart; columns and rows from n_pose_params on (the extra unknowns,
    // such as the gyro bias) are copied completely.
    Eigen::SparseMatrix<double> pose_band_to_sparse(const Eigen::MatrixXd &m, const Eigen::MatrixXi &written, int n_pose_params,
                                                    int max_offset = 17)
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
                if (written(row, col) != 0)
                    triplets.emplace_back(row, col, m(row, col));
            if (!extra_col)
                for (int row = n_pose_params; row < n; row++)
                    if (written(row, col) != 0)
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

    const Eigen::MatrixXi nonzero = (m.array() != 0.0).cast<int>();
    const Eigen::SparseMatrix<double> s = pose_band_to_sparse(m, nonzero, n_pose, max_offset);
    CHECK(Eigen::MatrixXd(s) == m);
    CHECK(s.nonZeros() == Eigen::SparseMatrix<double>(m.sparseView()).nonZeros());

    // pose entries outside the band are dropped
    Eigen::MatrixXd outside = m;
    outside(0, 10) = 5.0;
    outside(10, 0) = 5.0;
    Eigen::MatrixXi outside_written = nonzero;
    outside_written(0, 10) = outside_written(10, 0) = 1;
    CHECK(Eigen::MatrixXd(pose_band_to_sparse(outside, outside_written, n_pose, max_offset)) == m);
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
        Eigen::MatrixXi written;
        BandedNormalMatrix band;
        int n_pose;

        DenseAndBand(int n_pose_params, int n_extra, int half_width)
            : dense(Eigen::MatrixXd::Zero(n_pose_params + n_extra, n_pose_params + n_extra)),
              written(Eigen::MatrixXi::Zero(n_pose_params + n_extra, n_pose_params + n_extra)), band(n_pose_params, n_extra, half_width), n_pose(n_pose_params)
        {
        }

        void add(int r, int c, double v)
        {
            dense(r, c) += v;
            written(r, c) = 1;
            band.add(r, c, v);
        }
    };

    // the banded matrix as a sparse matrix is the dense one's band conversion: same pattern, same bits
    void check_same_as_dense(const DenseAndBand &x, int half_width)
    {
        const Eigen::SparseMatrix<double> reference = pose_band_to_sparse(x.dense, x.written, x.n_pose, half_width);
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
                        {
                            x.dense(k * 6 + r, k * 6 + c) += block(r, c);
                            x.written(k * 6 + r, k * 6 + c) = 1;
                        }
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

// A sum that cancels to exactly zero stays in the pattern as an explicit zero: the pattern depends on what was written, not
// on the values, so it cannot change from one iteration to the next because an entry happens to be exactly zero
TEST_CASE("BandedNormalMatrix: an element that cancels to exactly zero stays in the pattern")
{
    DenseAndBand x(60, 3, 17);
    x.add(7, 9, 123.456);
    x.add(7, 9, -123.456);
    x.add(60, 4, 5.0);
    x.add(60, 4, -5.0);
    x.add(8, 62, 0.25);
    x.add(8, 62, -0.25);
    x.add(20, 21, 1.5); // one that does not cancel
    const Eigen::SparseMatrix<double> sparse = x.band.to_sparse();
    CHECK(sparse.nonZeros() == 4);
    CHECK(sparse.coeff(7, 9) == 0.0);
    check_same_as_dense(x, 17);

    // an element that was never written is not in the pattern, even next to written ones
    CHECK(sparse.coeff(7, 10) == 0.0);
    bool pattern_has_unwritten = false;
    for (int k = 0; k < sparse.outerSize(); k++)
        for (Eigen::SparseMatrix<double>::InnerIterator it(sparse, k); it; ++it)
            pattern_has_unwritten = pattern_has_unwritten || x.written(it.row(), it.col()) == 0;
    CHECK(!pattern_has_unwritten);
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

TEST_CASE("BandedNormalMatrix: a diagonal block puts all 36 elements in the pattern, zeros included")
{
    BandedNormalMatrix m(30, 0, 17);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> block = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();
    block(0, 0) = 1.0;
    m.add_diagonal_block6(12, block);
    CHECK(m.to_sparse().nonZeros() == 36);
}

namespace
{
    // a symmetric positive definite banded matrix (both triangles stored), values from the seed
    Eigen::SparseMatrix<double> spd_band(int n, int band, unsigned seed, bool extra_entry = false)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> value(-1.0, 1.0);
        std::vector<Eigen::Triplet<double>> t;
        for (int i = 0; i < n; i++)
        {
            t.emplace_back(i, i, 4.0 * band + 1.0 + value(rng));
            for (int d = 1; d <= band && i + d < n; d++)
            {
                const double v = value(rng);
                t.emplace_back(i, i + d, v);
                t.emplace_back(i + d, i, v);
            }
        }
        if (extra_entry)
        {
            t.emplace_back(0, n - 1, 0.5);
            t.emplace_back(n - 1, 0, 0.5);
        }
        Eigen::SparseMatrix<double> a(n, n);
        a.setFromTriplets(t.begin(), t.end());
        return a;
    }

    Eigen::VectorXd solve_with(PatternCachedCholesky &chol, const Eigen::SparseMatrix<double> &a, const Eigen::VectorXd &b)
    {
        chol.compute(a);
        REQUIRE(chol.solver().info() == Eigen::Success);
        return chol.solver().solve(b);
    }
}

TEST_CASE("PatternCachedCholesky: the analysis is kept for the same pattern and redone for another, results unchanged")
{
    const int n = 300;
    const Eigen::VectorXd b = Eigen::VectorXd::LinSpaced(n, -1.0, 1.0);
    PatternCachedCholesky cached;

    // five matrices with one pattern and different values: one analysis; each result equals that of a fresh object
    for (unsigned seed = 1; seed <= 5; seed++)
    {
        const Eigen::SparseMatrix<double> a = spd_band(n, 7, seed);
        const Eigen::VectorXd x = solve_with(cached, a, b);
        PatternCachedCholesky fresh;
        const Eigen::VectorXd y = solve_with(fresh, a, b);
        CHECK(x == y);
        CHECK((a * x - b).norm() < 1e-9 * b.norm());
    }
    CHECK(cached.calls() == 5);
    CHECK(cached.analyses() == 1);

    // a different pattern (one more entry): analysed again, then kept again
    for (unsigned seed = 6; seed <= 8; seed++)
    {
        const Eigen::SparseMatrix<double> a = spd_band(n, 7, seed, true);
        const Eigen::VectorXd x = solve_with(cached, a, b);
        PatternCachedCholesky fresh;
        CHECK(x == solve_with(fresh, a, b));
    }
    CHECK(cached.calls() == 8);
    CHECK(cached.analyses() == 2);

    // a different size: analysed again
    const Eigen::VectorXd b2 = Eigen::VectorXd::Ones(n + 10);
    const Eigen::SparseMatrix<double> a2 = spd_band(n + 10, 7, 9);
    PatternCachedCholesky fresh2;
    CHECK(solve_with(cached, a2, b2) == solve_with(fresh2, a2, b2));
    CHECK(cached.analyses() == 3);
}

TEST_CASE("solution_by_row keeps an entry the sparse result does not store in its place")
{
    // a one-column solution with rows 0, 1 and 3 stored and row 2 an exact zero, as a sparse solve returns it
    Eigen::SparseMatrix<double> x(4, 1);
    x.insert(0, 0) = 1.5;
    x.insert(1, 0) = -2.0;
    x.insert(3, 0) = 4.0;
    x.makeCompressed();
    const std::vector<double> v = solution_by_row(x, 4);
    REQUIRE(v.size() == 4);
    CHECK(v[0] == 1.5);
    CHECK(v[1] == -2.0);
    CHECK(v[2] == 0.0);
    CHECK(v[3] == 4.0);
}
