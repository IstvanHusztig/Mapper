#include <doctest/doctest.h>

#include "degeneracy.h"

#include <cmath>
#include <numbers>
#include <random>

// detect_degeneracy on hand-made point-to-plane observations: a closed room, a corridor open along x, a round tunnel
// along x, the corridor with noisy normals, and the room moved far from the origin (change hessian-degeneracy-detection)

namespace
{
    constexpr double point_sigma = 0.02;
    constexpr double snr = 10.0;

    // the normal's rotation covariance of a typical filled 0.15 m planar cell (20 points, 4 cm spread in the plane)
    Eigen::Matrix3d typical_normal_cov(const Eigen::Vector3d &n)
    {
        Eigen::Vector3d a = std::abs(n.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        Eigen::Matrix3d vectors;
        vectors.col(0) = n;
        vectors.col(1) = n.cross(a).normalized();
        vectors.col(2) = n.cross(vectors.col(1));
        return normal_rotation_covariance(Eigen::Vector3d(1e-6, 0.04 * 0.04, 0.04 * 0.04), vectors, 20.0, point_sigma);
    }

    void add(std::vector<PlaneObservation> &obs, const Eigen::Vector3d &p, const Eigen::Vector3d &n)
    {
        PlaneObservation o;
        o.point = p;
        o.normal = n.normalized();
        o.normal_cov = typical_normal_cov(o.normal);
        obs.push_back(o);
    }

    // walls y = +-2.5 and floor / ceiling z = 0 / 3 over x in [-xl, xl]; with end walls at x = +-xl when closed
    std::vector<PlaneObservation> box(double xl, bool closed, int per_face)
    {
        std::mt19937 rng(5);
        std::uniform_real_distribution<double> u(0.0, 1.0);
        std::vector<PlaneObservation> obs;
        for (int i = 0; i < per_face; i++)
        {
            const double x = -xl + 2.0 * xl * u(rng), y = -2.5 + 5.0 * u(rng), z = 3.0 * u(rng);
            add(obs, {x, -2.5, z}, {0, 1, 0});
            add(obs, {x, 2.5, z}, {0, -1, 0});
            add(obs, {x, y, 0.0}, {0, 0, 1});
            add(obs, {x, y, 3.0}, {0, 0, -1});
            if (closed)
            {
                add(obs, {-xl, y, z}, {1, 0, 0});
                add(obs, {xl, y, z}, {-1, 0, 0});
            }
        }
        return obs;
    }

    // the p of the eigen-direction closest to axis d
    double p_along(const Eigen::Matrix3d &directions, const Eigen::Vector3d &p, const Eigen::Vector3d &d)
    {
        int best = 0;
        for (int k = 1; k < 3; k++)
            if (std::abs(directions.col(k).dot(d)) > std::abs(directions.col(best).dot(d)))
                best = k;
        return p(best);
    }
}

TEST_CASE("degeneracy: a closed room constrains every direction")
{
    const auto obs = box(4.0, true, 500);
    const DegeneracyVerdict v = detect_degeneracy(obs, Eigen::Vector3d(0, 0, 1.5), point_sigma, snr);
    REQUIRE(v.valid);
    MESSAGE("room: translation p ", v.translation_p.transpose(), " info ", v.translation_information.transpose(), "; rotation p ", v.rotation_p.transpose(), " info ", v.rotation_information.transpose());
    for (int k = 0; k < 3; k++)
    {
        CHECK(v.translation_p(k) == 1.0);
        CHECK(v.rotation_p(k) == 1.0);
    }
}

TEST_CASE("degeneracy: a corridor open along x leaves the translation along x unconstrained")
{
    const auto obs = box(15.0, false, 500);
    const DegeneracyVerdict v = detect_degeneracy(obs, Eigen::Vector3d(0, 0, 1.5), point_sigma, snr);
    REQUIRE(v.valid);
    CHECK(p_along(v.translation_directions, v.translation_p, Eigen::Vector3d::UnitX()) < 0.01);
    CHECK(p_along(v.translation_directions, v.translation_p, Eigen::Vector3d::UnitY()) == 1.0);
    CHECK(p_along(v.translation_directions, v.translation_p, Eigen::Vector3d::UnitZ()) == 1.0);
    for (int k = 0; k < 3; k++)
        CHECK(v.rotation_p(k) == 1.0); // walls and floor fix every rotation
}

TEST_CASE("degeneracy: a round tunnel leaves the translation along it and the rotation about it unconstrained")
{
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    std::vector<PlaneObservation> obs;
    for (int i = 0; i < 3000; i++)
    {
        const double x = -10.0 + 20.0 * u(rng), a = 2.0 * std::numbers::pi * u(rng);
        const Eigen::Vector3d radial(0.0, std::cos(a), std::sin(a));
        add(obs, Eigen::Vector3d(x, 0, 0) + 2.0 * radial, -radial);
    }
    const DegeneracyVerdict v = detect_degeneracy(obs, Eigen::Vector3d::Zero(), point_sigma, snr);
    REQUIRE(v.valid);
    CHECK(p_along(v.translation_directions, v.translation_p, Eigen::Vector3d::UnitX()) < 0.01);
    CHECK(p_along(v.translation_directions, v.translation_p, Eigen::Vector3d::UnitY()) == 1.0);
    CHECK(p_along(v.rotation_directions, v.rotation_p, Eigen::Vector3d::UnitX()) < 0.01);
    CHECK(p_along(v.rotation_directions, v.rotation_p, Eigen::Vector3d::UnitZ()) == 1.0);
}

TEST_CASE("degeneracy: noisy normals don't make the corridor's axis look constrained")
{
    auto obs = box(15.0, false, 500);
    std::mt19937 rng(13);
    std::normal_distribution<double> tilt(0.0, 2.0 * std::numbers::pi / 180.0); // 2 deg
    std::normal_distribution<double> e(0.0, point_sigma);
    for (auto &o : obs)
    {
        const Eigen::Vector3d axis(tilt(rng), tilt(rng), tilt(rng));
        o.normal = (Eigen::AngleAxisd(axis.norm(), axis.normalized()) * o.normal).normalized();
        o.point += Eigen::Vector3d(e(rng), e(rng), e(rng));
    }
    const DegeneracyVerdict v = detect_degeneracy(obs, Eigen::Vector3d(0, 0, 1.5), point_sigma, snr);
    REQUIRE(v.valid);
    const double px = p_along(v.translation_directions, v.translation_p, Eigen::Vector3d::UnitX());
    MESSAGE("corridor with 2 deg normal noise: p along the axis ", px, ", information ", v.translation_information.transpose());
    CHECK(px < 0.1);
}

TEST_CASE("degeneracy: the verdict doesn't depend on where the world origin is")
{
    auto obs = box(15.0, false, 300);
    const DegeneracyVerdict near = detect_degeneracy(obs, Eigen::Vector3d(0, 0, 1.5), point_sigma, snr);
    const Eigen::Vector3d shift(1000.0, -500.0, 20.0);
    for (auto &o : obs)
        o.point += shift;
    const DegeneracyVerdict far = detect_degeneracy(obs, Eigen::Vector3d(0, 0, 1.5) + shift, point_sigma, snr);
    for (int k = 0; k < 3; k++)
    {
        CHECK(far.translation_p(k) == doctest::Approx(near.translation_p(k)).epsilon(1e-6));
        CHECK(far.rotation_p(k) == doctest::Approx(near.rotation_p(k)).epsilon(1e-6));
    }
}
