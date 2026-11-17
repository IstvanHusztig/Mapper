#include "window_term_csv.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace
{
    // an empty field for a value that doesn't exist (NaN: no pull where a term has no weight)
    std::string number(double v)
    {
        if (!std::isfinite(v))
            return "";
        std::ostringstream o;
        o << std::setprecision(9) << v;
        return o.str();
    }

    std::string vector3(const Eigen::Vector3d &v) { return number(v.x()) + "," + number(v.y()) + "," + number(v.z()); }

    std::string terms_header()
    {
        std::string h = "window,time_s,state,term,in_total,cost,residuals";
        for (const char *quantity : {"weight", "pull"})
            for (int m = 0; m < kTermMotions; m++)
                for (int d = 0; d < kTermDirections; d++)
                    h += std::string(",") + quantity + "_" + term_motion_name(m) + "_" + term_direction_name(d);
        return h;
    }

    std::string features_header()
    {
        std::string h = "window,time_s,duration_s,iterations,delta,lm_factor,at_rest,turn_window,acc_term,along_x,along_y,"
                        "guess_move_x,guess_move_y,guess_move_z,result_move_x,result_move_y,result_move_z,"
                        "velocity_start_x,velocity_start_y,velocity_start_z,velocity_end_x,velocity_end_y,velocity_end_z,"
                        "gyro_bias_x_deg_s,gyro_bias_y_deg_s,gyro_bias_z_deg_s,acc_offset_x_mps2,acc_offset_y_mps2,acc_offset_z_mps2,"
                        "information_ratio,weak_axis_x,weak_axis_y,weak_axis_z,ndt_matched,ndt_unmatched,turn_defined,table_failed";
        for (int m = 0; m < kTermMotions; m++)
            for (int d = 0; d < kTermDirections; d++)
                h += std::string(",change_") + term_motion_name(m) + "_" + term_direction_name(d);
        h += ",verdict_bent,verdict_bent_severely,verdict_fast,verdict_bias_high,heading_bend_deg,roll_bend_deg,pitch_bend_deg,speed_mps,"
             "max_bias_deg_s,velocity_chi2,stop,rejected_steps,steps_not_applied,big_jump_nodes,remaining_step_m,lidar_share_along";
        return h;
    }

    void write_rows(std::ofstream &out, const WindowTermRecord &r, const char *state, const WindowTermTable &t)
    {
        for (const WindowTermRow &row : t.rows)
        {
            out << r.window << "," << number(r.time_s) << "," << state << "," << row.name << "," << (row.in_total ? 1 : 0) << ","
                << number(row.cost) << "," << row.residuals;
            for (const auto *values : {&row.weight, &row.pull})
                for (int m = 0; m < kTermMotions; m++)
                    for (int d = 0; d < kTermDirections; d++)
                        out << "," << number((*values)[m][d]);
            out << "\n";
        }
    }
}

void FolderWindowTermSink::write(const WindowTermRecord &r)
{
    if (failed_)
        return;
    if (!terms_.is_open())
    {
        // a CLI run's output folder is only created when the results are saved, after the odometry
        std::error_code ec;
        std::filesystem::create_directories(folder_, ec);
        terms_.open(terms_file());
        features_.open(features_file());
        if (!terms_ || !features_)
        {
            failed_ = true;
            return;
        }
        terms_ << terms_header() << "\n";
        features_ << features_header() << "\n";
    }
    write_rows(terms_, r, "guess", r.guess);
    write_rows(terms_, r, "result", r.result);

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(r.result.translation_information);
    const double largest = eig.eigenvalues().maxCoeff();
    const double ratio = largest > 0.0 ? eig.eigenvalues().minCoeff() / largest : 0.0;
    const Eigen::Vector3d &along = r.result.directions[0];
    features_ << r.window << "," << number(r.time_s) << "," << number(r.duration_s) << "," << r.iterations << "," << number(r.delta) << ","
              << number(r.lm_factor) << "," << (r.at_rest ? 1 : 0) << "," << (r.turn_window ? 1 : 0) << "," << (r.acc_term ? 1 : 0) << ","
              << number(along.x()) << "," << number(along.y()) << "," << vector3(r.guess_move) << "," << vector3(r.result_move) << ","
              << vector3(r.velocity_start) << "," << vector3(r.velocity_end) << "," << vector3(r.gyro_bias_deg_s) << ","
              << vector3(r.acc_offset_mps2) << "," << number(ratio) << "," << vector3(eig.eigenvectors().col(0)) << "," << r.result.ndt_matched
              << "," << r.result.ndt_unmatched << "," << (r.result.turn_defined ? 1 : 0) << ","
              << (r.guess.failure.empty() && r.result.failure.empty() ? 0 : 1);
    for (int m = 0; m < kTermMotions; m++)
        for (int d = 0; d < kTermDirections; d++)
            features_ << "," << number(r.change[m][d]);
    // the window verdict: its flags (CLEAR / TRIGGER / NOT MEASURED) and readings (empty where NOT MEASURED)
    const WindowVerdict &v = r.verdict;
    const WindowEvidence &e = v.evidence;
    const auto opt = [](const std::optional<double> &x) { return x ? number(*x) : std::string(); };
    features_ << "," << check_name(v.bent) << "," << check_name(v.bent_severely) << "," << check_name(v.fast) << "," << check_name(v.bias_high)
              << "," << opt(e.heading_bend_deg) << "," << opt(e.roll_bend_deg) << "," << opt(e.pitch_bend_deg) << "," << opt(e.speed_mps) << ","
              << opt(e.max_bias_deg_s) << "," << opt(e.velocity_chi2) << "," << lm_stop_name(e.stop) << "," << e.rejected_steps << ","
              << e.steps_not_applied << "," << e.big_jump_nodes << "," << opt(e.remaining_step_m) << "," << opt(e.lidar_share_along);
    features_ << "\n";
    terms_.flush();
    features_.flush();
    failed_ = !terms_ || !features_;
}
