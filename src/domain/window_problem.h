#pragma once

// A window's scan-to-map optimization as a problem for the Levenberg-Marquardt solver (lm_solver.h): it linearizes with
// the term functions (ndt_term, motion_model_terms, accelerometer_term, the origin fix), takes the covariance of the
// extra unknowns (gyro bias, acceleration offset, end velocity), applies a step to the window (window_solver.h) and keeps
// snapshots of the window's state for the step check.

#include "lm_solver.h"
#include "ndt_map.h"
#include "optimizer_options.h"
#include "phase_timing.h"
#include "run_reporting.h"
#include "scan_to_map.h"
#include "window.h"
#include "window_equations.h"
#include "window_noise_model.h"
#include <structures.h>

#include <Eigen/Dense>
#include <vector>

class WindowProblem : public LmProblem
{
public:
    // noise: the window's noise model (window_noise_model.h), built before the first iteration; it holds the layout of the
    // unknowns. translation_information: when not null, the window's translation information from the matched normals is
    // written to it at every linearization; timing: when not null, the problem's phases are added to it
    WindowProblem(std::vector<Point3Di> &points, Window &window, NDTBucketMapType &buckets_indoor, NDTBucketMapType &buckets_outdoor,
                  const ScanToMapOptions &scan_to_map, const MotionModelOptions &motion_model, const MotionModelRotationOptions &rotation_options,
                  const MotionModelTranslationImuOptions &translation_imu_options, const WindowNoiseModel &noise,
                  Eigen::Vector3d &acc_offset, Eigen::Matrix3d &acc_offset_cov, Logger &logger, Eigen::Matrix3d *translation_information,
                  ScanToMapTiming *timing, ScanToMapWorkspace &workspace);

    LmLinearization linearize(bool with_cost) override;
    void take_covariance(const Eigen::MatrixXd &extra_columns) override;
    bool apply(const std::vector<double> &h_x, bool solved, double &delta) override;
    void snapshot() override;
    void accept_snapshot() override;
    void restore_accepted() override;

    // the cost of the present state as last linearized with the cost (or restored with it)
    const ScanToMapCost &cost() const { return cost_; }

private:
    // what an iteration changes, and the step check takes back
    struct State
    {
        std::vector<Eigen::Affine3d> poses;
        VelocityState velocity;
        GyroBiasState gyro_bias;
        Eigen::Vector3d acc_offset;
        Eigen::Matrix3d acc_offset_cov;
        Eigen::Matrix3d translation_information;
        ScanToMapCost cost;
    };
    State state() const;
    void set_state(const State &s);

    std::vector<Point3Di> &points_;
    Window &window_;
    NDTBucketMapType &buckets_indoor_;
    NDTBucketMapType &buckets_outdoor_;
    const ScanToMapOptions &scan_to_map_;
    const MotionModelOptions &motion_model_;
    const MotionModelRotationOptions &rotation_options_;
    const MotionModelTranslationImuOptions &translation_imu_options_;
    const WindowNoiseModel &noise_;
    Eigen::Vector3d &acc_offset_;
    Eigen::Matrix3d &acc_offset_cov_;
    Logger &logger_;
    Eigen::Matrix3d *translation_information_;
    ScanToMapTiming *timing_;
    ScanToMapWorkspace &workspace_;
    const WindowUnknowns &unknowns_; // noise_.unknowns
    ScanToMapCost cost_;
    State candidate_, accepted_;
};
