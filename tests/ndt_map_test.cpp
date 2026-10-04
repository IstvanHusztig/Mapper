#include <doctest/doctest.h>

#include "ndt_map.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <numbers>
#include <random>

// update_rgd on synthetic planes: the exact cell statistics (mean, covariance and normal of all of a cell's points) and
// the previous ones, whose quirks are pinned here so that the switch-off path stays as it was.

namespace
{
    NdtGridParameters grid(double resolution)
    {
        NdtGridParameters g{};
        g.resolution_X = g.resolution_Y = g.resolution_Z = resolution;
        return g;
    }

    Point3Di make_point(const Eigen::Vector3d &p)
    {
        Point3Di q{};
        q.point = p;
        return q;
    }

    double angle_deg(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
    {
        return std::acos(std::clamp(std::abs(a.normalized().dot(b.normalized())), 0.0, 1.0)) * 180.0 / std::numbers::pi;
    }

    // a tilted plane through (1, 1, 1), sampled on a 2 cm grid over 2 m x 2 m, line by line, with +-2 mm noise along
    // its normal (fixed seed); all coordinates stay positive, so every cell has the nominal size
    struct SampledPlane
    {
        Eigen::Vector3d normal;
        Eigen::Vector3d viewport;
        std::vector<Point3Di> points;
    };

    SampledPlane tilted_plane()
    {
        SampledPlane plane;
        plane.normal = Eigen::Vector3d(0.2, -0.3, 1.0).normalized();
        const Eigen::Vector3d u = plane.normal.cross(Eigen::Vector3d::UnitX()).normalized();
        const Eigen::Vector3d v = plane.normal.cross(u);
        const Eigen::Vector3d origin(2.0, 2.0, 2.0);
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> noise(-0.002, 0.002);
        for (int i = 0; i <= 100; i++)
            for (int j = 0; j <= 100; j++)
                plane.points.push_back(make_point(origin + (i * 0.02 - 1.0) * u + (j * 0.02 - 1.0) * v + noise(rng) * plane.normal));
        plane.viewport = origin + 3.0 * plane.normal;
        return plane;
    }

    // each cell's points, computed directly: their count, centroid, and whether they span an area (the second-largest
    // spread is above (1 cm)^2); points on one line, as in a sliver of the plane cut by a cell corner, define no plane
    struct CellPoints
    {
        int count = 0;
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        bool spans_area = false;
    };

    std::map<uint64_t, CellPoints> cells_of(const std::vector<Point3Di> &points, double resolution)
    {
        std::map<uint64_t, std::vector<Eigen::Vector3d>> grouped;
        const Eigen::Vector3d b(resolution, resolution, resolution);
        for (const auto &p : points)
            grouped[get_rgd_index(p.point, b)].push_back(p.point);
        std::map<uint64_t, CellPoints> cells;
        for (const auto &[index, ps] : grouped)
        {
            CellPoints &c = cells[index];
            c.count = static_cast<int>(ps.size());
            for (const auto &p : ps)
                c.centroid += p;
            c.centroid /= c.count;
            Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
            for (const auto &p : ps)
                scatter += (p - c.centroid) * (p - c.centroid).transpose();
            c.spans_area = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(scatter / c.count).eigenvalues()(1) > 1e-4;
        }
        return cells;
    }
}

TEST_CASE("update_rgd exact: a plane inserted line by line gives the centroid and the plane normal")
{
    SampledPlane plane = tilted_plane();
    const double resolution = 0.5;
    NDTBucketMapType buckets;
    update_rgd(grid(resolution), buckets, plane.points, plane.viewport, true);

    const auto expected = cells_of(plane.points, resolution);
    REQUIRE(buckets.size() == expected.size());
    int checked = 0;
    for (const auto &[index, e] : expected)
    {
        const NdtBucket &bucket = buckets.at(index);
        CHECK(bucket.number_of_points == e.count);
        if (e.count < 10)
            continue;
        CHECK((bucket.mean - e.centroid).norm() < 1e-3);
        if (!e.spans_area)
        {
            CHECK(bucket.normal_vector == Eigen::Vector3d::Zero()); // a sliver: points on one line define no plane
            continue;
        }
        checked++;
        CHECK(angle_deg(bucket.normal_vector, plane.normal) < 5.0);
        CHECK(bucket.normal_vector.dot(plane.viewport - bucket.mean) > 0.0); // oriented towards the viewport
    }
    CHECK(checked > 10);
}

TEST_CASE("update_rgd exact: the insertion order doesn't change the cells")
{
    SampledPlane plane = tilted_plane();
    NDTBucketMapType in_lines, shuffled;
    update_rgd(grid(0.5), in_lines, plane.points, plane.viewport, true);
    std::shuffle(plane.points.begin(), plane.points.end(), std::mt19937(3));
    update_rgd(grid(0.5), shuffled, plane.points, plane.viewport, true);

    REQUIRE(in_lines.size() == shuffled.size());
    for (const auto &[index, a] : in_lines)
    {
        const NdtBucket &b = shuffled.at(index);
        CHECK(a.number_of_points == b.number_of_points);
        CHECK((a.mean - b.mean).cwiseAbs().maxCoeff() < 1e-9);
        CHECK((a.cov - b.cov).cwiseAbs().maxCoeff() < 1e-9);
        CHECK((a.normal_vector - b.normal_vector).cwiseAbs().maxCoeff() < 1e-9);
    }
}

// one 0.5 m cell of the plane z = 0.7, scanned along x first: its first 3 points lie on a line
namespace
{
    std::vector<Point3Di> one_cell_in_lines()
    {
        std::mt19937 rng(11);
        std::uniform_real_distribution<double> noise(-0.002, 0.002);
        std::vector<Point3Di> points;
        for (double y : {0.75, 0.55, 0.65, 0.85, 0.95})
            for (int i = 0; i < 9; i++)
                points.push_back(make_point(Eigen::Vector3d(0.55 + 0.05 * i, y, 0.7 + noise(rng))));
        return points;
    }
}

TEST_CASE("update_rgd previous statistics: first point as mean, normal fixed from the first 3 points")
{
    auto points = one_cell_in_lines();
    const Eigen::Vector3d viewport(0.75, 0.75, 3.0);

    NDTBucketMapType all;
    update_rgd(grid(0.5), all, points, viewport, false);
    REQUIRE(all.size() == 1);
    const NdtBucket &cell = all.begin()->second;
    CHECK(cell.number_of_points == points.size());
    CHECK(cell.mean == points[0].point);

    std::vector<Point3Di> first_three(points.begin(), points.begin() + 3);
    NDTBucketMapType three;
    update_rgd(grid(0.5), three, first_three, viewport, false);
    CHECK(cell.normal_vector == three.begin()->second.normal_vector);
    // the first 3 points are collinear, so that normal is far off the plane
    CHECK(angle_deg(cell.normal_vector, Eigen::Vector3d::UnitZ()) > 30.0);

    // the exact statistics find the plane from the same points
    NDTBucketMapType exact;
    update_rgd(grid(0.5), exact, points, viewport, true);
    CHECK(angle_deg(exact.begin()->second.normal_vector, Eigen::Vector3d::UnitZ()) < 5.0);
}

TEST_CASE("update_rgd exact: cells with fewer than 3 points have no normal")
{
    std::vector<Point3Di> points{make_point({0.6, 0.6, 0.6}), make_point({0.7, 0.6, 0.6})};
    NDTBucketMapType buckets;
    update_rgd(grid(0.5), buckets, points, Eigen::Vector3d(0, 0, 3), true);
    REQUIRE(buckets.size() == 1);
    CHECK(buckets.begin()->second.number_of_points == 2);
    CHECK(buckets.begin()->second.normal_vector == Eigen::Vector3d::Zero());
    CHECK((buckets.begin()->second.mean - Eigen::Vector3d(0.65, 0.6, 0.6)).norm() < 1e-12);
}

TEST_CASE("update_rgd exact: points on one line give no normal")
{
    // 12 points along x with +-2 mm noise in z: the spread across the line is the same in y and z, so no normal is defined
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> noise(-0.002, 0.002);
    std::vector<Point3Di> points;
    for (int i = 0; i < 12; i++)
        points.push_back(make_point(Eigen::Vector3d(0.52 + 0.04 * i, 0.75, 0.7 + noise(rng))));
    NDTBucketMapType buckets;
    update_rgd(grid(0.5), buckets, points, Eigen::Vector3d(0.75, 0.75, 3.0), true);
    REQUIRE(buckets.size() == 1);
    CHECK(buckets.begin()->second.number_of_points == 12);
    CHECK(buckets.begin()->second.normal_vector == Eigen::Vector3d::Zero());

    // the same points with a second line next to them span a plane, which gets its normal
    for (int i = 0; i < 12; i++)
        points.push_back(make_point(Eigen::Vector3d(0.52 + 0.04 * i, 0.85, 0.7 + noise(rng))));
    NDTBucketMapType plane;
    update_rgd(grid(0.5), plane, points, Eigen::Vector3d(0.75, 0.75, 3.0), true);
    CHECK(angle_deg(plane.begin()->second.normal_vector, Eigen::Vector3d::UnitZ()) < 5.0);
}

TEST_CASE("update_rgd exact: the far side of a thin wall doesn't enter the near side's cell")
{
    // a wall 2 cm thick inside one cell (x index 2): side A at x = 1.10 seen from x = -2, side B at x = 1.12 from x = +4
    auto side = [](double x)
    {
        std::vector<Point3Di> points;
        for (int i = 0; i < 9; i++)
            for (int j = 0; j < 9; j++)
                points.push_back(make_point(Eigen::Vector3d(x, 0.55 + 0.05 * i, 0.55 + 0.05 * j)));
        return points;
    };
    auto a = side(1.10), b = side(1.12);

    NDTBucketMapType buckets;
    update_rgd(grid(0.5), buckets, a, Eigen::Vector3d(-2.0, 0.75, 0.75), true);
    REQUIRE(buckets.size() == 1);
    const NdtBucket before = buckets.begin()->second;
    CHECK(angle_deg(before.normal_vector, Eigen::Vector3d::UnitX()) < 1e-3);
    CHECK(before.normal_vector.x() < 0.0);

    update_rgd(grid(0.5), buckets, b, Eigen::Vector3d(4.0, 0.75, 0.75), true);
    REQUIRE(buckets.size() == 1);
    const NdtBucket &after = buckets.begin()->second;
    CHECK(after.number_of_points == before.number_of_points);
    CHECK(after.mean == before.mean);
    CHECK(after.cov == before.cov);
    CHECK(after.normal_vector == before.normal_vector);
}
