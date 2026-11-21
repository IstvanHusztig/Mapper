#include <doctest/doctest.h>

#include "angle_units.h"
#include "ndt_map.h"
#include "ndt_term.h"
#include "optimizer_options.h"
#include "odometry_config.h"
#include "window_equations.h"
#include "window_noise_model.h"
#include "window_solver.h"

#include <cmath>
#include <iomanip>
#include <sstream>

// Unit-level cases of the consistency harness (openspec change consistency-harness, task 5.4; design D7): K7 (C16, the
// fractional projection's own step), K9 (the cell test's rotation dependence) and K10 (C37, the origin fix's rotation
// weight). Each MEASURES and reports its number beside the audit's hypothesis and names its oracle. The measuring cases
// are not marked: the honest outcome is stated in a MESSAGE, their CHECKs pin the MEASURED characterisation. Task 5.8 adds
// one case each for K7 and K9, marked doctest::should_fail, that asserts the oracle (K7: an argued bound; K9: rotation
// invariance). K10 (C37, harmless) is not marked.

// K7 (C16): the degeneracy projection scales a weak translation direction by s in (0, 1) (M = diag(P, I), P = s w w^T + the
// rest): the node's block becomes M B M and its right-hand side M g, so along w the information is s^2 k and the force
// s g, and the block's own step is s g / (s^2 k) = g / (s k). Reached through add_projected_ndt_blocks (public, ndt_term.h)
// on a hand-made NdtSums of one node with the window's translation information deciding s; the step is read from the
// equations' diagonal and right-hand side. Oracle: the step of the unprojected block is g / k; a projection that only
// removes information must not move the node further, so at most g / k.
TEST_CASE("unit case K7: the fractional projection's own step along the weak axis")
{
    Window w;
    w.poses.push_back(Eigen::Affine3d::Identity());
    w.timestamps.push_back({0.0, 0.0});
    MotionModelRotationOptions rotation;
    rotation.estimate_bias = false;
    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});

    const double k = 50.0;    // the information along the weak axis z
    const double g = 0.2;     // the force along it (a unit-less number; the step is g / k)
    NdtNoise noise;
    noise.projection_ratio = 0.1;
    noise.projection_ratio_full = 0.2;

    std::ostringstream table;
    double step_at_half = 0.0;
    for (double s : {1.0, 0.75, 0.5, 0.25})
    {
        NdtSums sums;
        Eigen::Matrix<double, 6, 6, Eigen::RowMajor> block = Eigen::Matrix<double, 6, 6>::Zero();
        block.diagonal() << 100.0 * k, 100.0 * k, k, 10.0 * k, 10.0 * k, 10.0 * k;
        Eigen::Matrix<double, 6, 1> force = Eigen::Matrix<double, 6, 1>::Zero();
        force(2) = g;
        sums.blocks.push_back(block);
        sums.rhs.push_back(force);
        // the window's translation information: z at the ratio that gives the scale s on the ramp 0.1 .. 0.2
        const double ratio = noise.projection_ratio + s * (noise.projection_ratio_full - noise.projection_ratio);
        sums.normal_information.push_back(Eigen::Vector3d(1.0, 1.0, ratio).asDiagonal());

        WindowEquations e(u);
        add_projected_ndt_blocks(e, sums, noise);
        const double step = e.rhs(2, 0) / e.matrix(2, 2);
        const double unprojected = g / k;
        table << "s " << s << ": step " << step << " = " << step / unprojected << " g/k;  ";
        // the strong directions pass unchanged
        CHECK(e.matrix(0, 0) == doctest::Approx(100.0 * k).epsilon(1e-12));
        CHECK(e.matrix(3, 3) == doctest::Approx(10.0 * k).epsilon(1e-12));
        // the measured characterisation: the information is s^2 k, the force s g, the step g / (s k)
        CHECK(e.matrix(2, 2) == doctest::Approx(s * s * k).epsilon(1e-9));
        CHECK(e.rhs(2, 0) == doctest::Approx(s * g).epsilon(1e-9));
        CHECK(step == doctest::Approx(unprojected / s).epsilon(1e-9));
        if (s == 0.5)
            step_at_half = step;
    }
    MESSAGE("K7 (block B = k on the weak axis, force g, step in units of g/k): ", table.str());
    MESSAGE("K7 s = 0.5: the block's own step is ", step_at_half / (g / k), " g/k (hypothesis 2 g/k; oracle: at most 1 g/k)");
    CHECK(step_at_half / (g / k) == doctest::Approx(2.0).epsilon(1e-9));
}

// K7 / C16 marker: asserts the oracle. The oracle is an ARGUED BOUND (a projection that removes information must not
// lengthen the step), not a verified outcome. Cause: add_ndt_blocks (src/domain/ndt_term.cpp:331) forms M B M and M g, so
// along a fractionally kept axis the information is s^2 k and the force s g and the block's own step g / (s k): 2 g/k at
// s = 0.5. When this starts to pass, doctest reports it: remove should_fail.
TEST_CASE("unit case K7 marker: the fractional projection's own step at s = 0.5 is at most g / k" * doctest::should_fail())
{
    Window w;
    w.poses.push_back(Eigen::Affine3d::Identity());
    w.timestamps.push_back({0.0, 0.0});
    MotionModelRotationOptions rotation;
    rotation.estimate_bias = false;
    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
    const double k = 50.0, g = 0.2, s = 0.5;
    NdtNoise noise;
    noise.projection_ratio = 0.1;
    noise.projection_ratio_full = 0.2;
    NdtSums sums;
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> block = Eigen::Matrix<double, 6, 6>::Zero();
    block.diagonal() << 100.0 * k, 100.0 * k, k, 10.0 * k, 10.0 * k, 10.0 * k;
    Eigen::Matrix<double, 6, 1> force = Eigen::Matrix<double, 6, 1>::Zero();
    force(2) = g;
    sums.blocks.push_back(block);
    sums.rhs.push_back(force);
    sums.normal_information.push_back(
        Eigen::Vector3d(1.0, 1.0, noise.projection_ratio + s * (noise.projection_ratio_full - noise.projection_ratio)).asDiagonal());
    WindowEquations e(u);
    add_projected_ndt_blocks(e, sums, noise);
    const double step_in_g_over_k = (e.rhs(2, 0) / e.matrix(2, 2)) / (g / k);
    CAPTURE(step_in_g_over_k);
    CHECK(step_in_g_over_k <= 1.0 + 1e-9);
}

// K9 (the cell test, ndt_map.h ndt_inverse_usable): a cell is skipped when an entry of its inverse covariance exceeds
// 1e5 in absolute value. The entries are not rotation invariant: a vertical plane with its normal along an axis has the
// whole 1 / sigma_n^2 in one diagonal entry; turned 45 deg in XY it is shared between two (and the off-diagonal),
// half each. Reached through refresh_cell_inverse (public, ndt_map.h) on an NdtBucket with a stated covariance. Oracle:
// rotation invariance, the same plane is usable in both orientations or in neither.
namespace
{
    // the covariance of a vertical plane cell: normal in the XY plane at angle phi from x, tangent variances sigma_t1^2
    // (horizontal, in the plane) and sigma_t2^2 (vertical)
    NdtBucket plane_cell(double sigma_n, double phi_deg, double sigma_t1 = 0.05, double sigma_t2 = 0.08)
    {
        NdtBucket b;
        const Eigen::Matrix3d r = Eigen::AngleAxisd(phi_deg * DEG_TO_RAD, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        b.cov = r * Eigen::Vector3d(sigma_n * sigma_n, sigma_t1 * sigma_t1, sigma_t2 * sigma_t2).asDiagonal() * r.transpose();
        refresh_cell_inverse(b);
        return b;
    }

    // the thinnest normal sigma at which the cell is usable, by bisection on [0.5 mm, 10 mm]
    double thinnest_usable_mm(double phi_deg)
    {
        double lo = 0.0005, hi = 0.01; // lo unusable, hi usable
        for (int i = 0; i < 60; i++)
        {
            const double mid = 0.5 * (lo + hi);
            (plane_cell(mid, phi_deg).inverse_usable ? hi : lo) = mid;
        }
        return 0.5 * (lo + hi) * 1000.0;
    }
}

TEST_CASE("unit case K9: the cell test on the same plane, axis-aligned and turned 45 degrees")
{
    const double sigma_n = 0.003; // a 3 mm thick plane
    const NdtBucket aligned = plane_cell(sigma_n, 0.0);
    const NdtBucket turned = plane_cell(sigma_n, 45.0);

    // validity: the same plane, the same inverse eigenvalues in both (the cell itself is rotation invariant)
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ea(aligned.cov_inverse), et(turned.cov_inverse);
    CHECK(ea.eigenvalues().maxCoeff() == doctest::Approx(1.0 / (sigma_n * sigma_n)).epsilon(1e-9));
    CHECK(et.eigenvalues().maxCoeff() == doctest::Approx(ea.eigenvalues().maxCoeff()).epsilon(1e-9));
    CHECK(aligned.inverse_computed);
    CHECK(turned.inverse_computed);

    MESSAGE("K9 3 mm plane (tangent sigmas 5 cm horizontal, 8 cm vertical): largest inverse entry ", aligned.cov_inverse.cwiseAbs().maxCoeff(),
            " axis-aligned, ", turned.cov_inverse.cwiseAbs().maxCoeff(), " turned 45 deg; usable: aligned ", aligned.inverse_usable,
            ", turned ", turned.inverse_usable, " (hypothesis: only when turned; oracle: both or neither)");
    std::ostringstream thresholds;
    for (double phi : {0.0, 15.0, 30.0, 45.0})
        thresholds << phi << " deg: " << std::fixed << std::setprecision(3) << thinnest_usable_mm(phi) << " mm;  ";
    MESSAGE("K9 thinnest normal sigma that is usable, by the angle of the normal: ", thresholds.str(), "(hypothesis 3.2 mm axis-aligned, 2.2 mm at 45 deg)");

    // the measured characterisation
    CHECK_FALSE(aligned.inverse_usable);
    CHECK(turned.inverse_usable);
    CHECK(thinnest_usable_mm(0.0) == doctest::Approx(3.16).epsilon(0.01));
    CHECK(thinnest_usable_mm(45.0) == doctest::Approx(2.24).epsilon(0.01));
}

// K10 (C37): the origin fix's rotation weight. The noise model takes the six sigmas as they are set; the rotation rows
// of the equations are in radians but the settings in degrees (HDMapping's "Fix deg to rad"). Reached through
// build_window_noise_model (public) with the shipped defaults (1e-6 deg), and through fix_origin on the equations.
// Oracle: 1 / sigma^2 with sigma in radians.
TEST_CASE("unit case K10: the origin fix's rotation weight against 1 / sigma^2 in radians")
{
    const OdometryConfig config; // the built-in defaults: every origin sigma 1e-6 (m, deg)
    Window w;
    for (int k = 0; k < 5; k++)
    {
        w.poses.push_back(Eigen::Affine3d::Identity());
        w.timestamps.push_back({k * 0.002, k * 0.002});
    }
    w.motion_model_poses = w.poses;
    MotionModelRotationOptions rotation;
    rotation.estimate_bias = false;
    const MotionModelOptions mm = get_motion_model_options(config);
    const WindowNoiseModel noise = build_window_noise_model(w, ScanToMapOptions{}, mm, rotation, {}, VelocityPrior{});

    const double sigma_deg = config.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg;
    const double oracle = 1.0 / ((sigma_deg * DEG_TO_RAD) * (sigma_deg * DEG_TO_RAD));
    MESSAGE("K10 origin weights: position ", noise.origin[0], " (1 / (1e-6 m)^2), rotation ", noise.origin[3], " ", noise.origin[4], " ",
            noise.origin[5], "; oracle 1 / (", sigma_deg, " deg in rad)^2 = ", oracle, " (hypothesis: 1e12 where 3.3e15 was meant)");

    const WindowUnknowns u = layout_window_unknowns(w, rotation, {});
    WindowEquations e(u);
    fix_origin(e, noise);
    MESSAGE("K10 the equations' diagonal at the first node: ", e.matrix(0, 0), " ", e.matrix(1, 1), " ", e.matrix(2, 2), " | ", e.matrix(3, 3), " ",
            e.matrix(4, 4), " ", e.matrix(5, 5));

    // the position rows are in metres and right; the measured characterisation of the rotation rows
    CHECK(noise.origin[0] == doctest::Approx(1e12).epsilon(1e-9));
    CHECK(noise.origin[3] == doctest::Approx(1e12).epsilon(1e-9));
    CHECK(noise.origin[4] == doctest::Approx(1e12).epsilon(1e-9));
    CHECK(noise.origin[5] == doctest::Approx(1e12).epsilon(1e-9));
    CHECK(e.matrix(3, 3) == doctest::Approx(noise.origin[3]).epsilon(1e-9));
    CHECK(oracle == doctest::Approx(3.283e15).epsilon(1e-3));
    CHECK(oracle / noise.origin[3] == doctest::Approx(3283.0).epsilon(1e-3));
}

// K9 / F9 marker: asserts the oracle, rotation invariance: the same 3 mm plane is usable axis-aligned and turned 45 deg, or
// in neither. Cause: ndt_inverse_usable (src/domain/ndt_map.h:22) tests each ENTRY of the inverse covariance against 1e5,
// not an eigenvalue; the entries depend on the orientation (3 mm: 1.1e5 in one entry axis-aligned, 5.6e4 in each of two
// turned). When this starts to pass, doctest reports it: remove should_fail.
TEST_CASE("unit case K9 marker: the cell test gives the same usability for the plane axis-aligned and turned 45 degrees" * doctest::should_fail())
{
    const NdtBucket aligned = plane_cell(0.003, 0.0);
    const NdtBucket turned = plane_cell(0.003, 45.0);
    CAPTURE(aligned.cov_inverse.cwiseAbs().maxCoeff());
    CAPTURE(turned.cov_inverse.cwiseAbs().maxCoeff());
    CHECK(aligned.inverse_usable == turned.inverse_usable);
}
