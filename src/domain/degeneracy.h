#pragma once

// Noise-aware degeneracy test of a window's rigid motion (change hessian-degeneracy-detection; Hatleskog & Alexis,
// "Probabilistic Degeneracy Detection for Point-to-Plane Error Minimization", RA-L 2025, adapted to NDT terms).
//
// Each matched point on a planar cell is one point-to-plane observation along the cell's normal. Together they constrain
// the rigid motion (translation, rotation about `centre`) of the window: H = sum v v^T, v = [n ; (p - centre) x n].
// Noise in the points and the cells' normals adds spurious information: E[H_est] = H + Sigma. Translation and rotation are
// separated by Schur complements, and each eigen-direction gets the probability p that its information exceeds
// snr_factor times what noise alone gives. The derivation is in openspec/changes/archive/*hessian-degeneracy-detection/design.md.

#include <Eigen/Dense>
#include <vector>

struct PlaneObservation
{
    Eigen::Vector3d point = Eigen::Vector3d::Zero();      // world frame
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();    // unit normal of the cell
    Eigen::Matrix3d normal_cov = Eigen::Matrix3d::Zero(); // covariance of the normal's small rotation [rad^2]
};

// covariance of the small rotation of a cell's normal (paper, Eq. 34): spreads ascending, columns of vectors their directions,
// count the cell's points, point_sigma_m the LiDAR's point noise
Eigen::Matrix3d normal_rotation_covariance(const Eigen::Vector3d &spreads, const Eigen::Matrix3d &vectors, double count, double point_sigma_m);

// a cell counts as a plane when its middle spread is at least three times its smallest and it has at least 3 points
bool is_planar_cell(const Eigen::Vector3d &spreads, double count);

struct DegeneracyVerdict
{
    bool valid = false;                                                  // false: too few observations, p all 0
    Eigen::Matrix3d translation_directions = Eigen::Matrix3d::Identity(); // columns: eigenvectors of the translation information
    Eigen::Vector3d translation_p = Eigen::Vector3d::Zero();             // probability that each is constrained
    Eigen::Vector3d translation_information = Eigen::Vector3d::Zero();   // its eigenvalues (before the noise correction)
    Eigen::Matrix3d rotation_directions = Eigen::Matrix3d::Identity();    // the same for rotation about the centre
    Eigen::Vector3d rotation_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_information = Eigen::Vector3d::Zero();
};

// observations in a fixed order (the result is then the same bits on every run)
DegeneracyVerdict detect_degeneracy(const std::vector<PlaneObservation> &observations, const Eigen::Vector3d &centre, double point_sigma_m,
                                    double snr_factor);
