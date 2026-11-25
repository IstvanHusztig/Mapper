#include "window_term_csv.h"

#include <angle_units.h>
#include <transformations.h>

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
        // the window's noise model as sigmas (empty where the window has no such weight)
        h += ",noise_translation_x_m,noise_translation_y_m,noise_translation_z_m";
        for (const char *stat : {"min", "median", "max"})
            for (const char *axis : {"om", "fi", "ka"})
                h += std::string(",noise_rotation_") + stat + "_" + axis + "_deg";
        h += ",noise_velocity_continuity_min_mps,noise_velocity_continuity_max_mps,noise_accelerometer_min_mps,noise_accelerometer_max_mps,"
             "noise_velocity_prior_1_mps,noise_velocity_prior_2_mps,noise_velocity_prior_3_mps,noise_bias_prior_1_deg_s,noise_bias_prior_2_deg_s,"
             "noise_bias_prior_3_deg_s,noise_offset_prior_1_mps2,noise_offset_prior_2_mps2,noise_offset_prior_3_mps2,noise_zero_velocity_mps,"
             "noise_ndt_information_scale,noise_ndt_huber_cutoff";
        // consistency-harness D9: the state the window was prepared with, what it handed on, and two features (empty where
        // absent). Poses as position and Tait-Bryan angles om, fi, ka; covariances as sigmas (square roots of the
        // diagonal), the velocity-offset cross block as its diagonal
        const auto xyz = [&h](const std::string &name, const std::string &unit)
        {
            for (const char *axis : {"x", "y", "z"})
                h += "," + name + "_" + axis + "_" + unit;
        };
        const auto pose = [&h, &xyz](const std::string &name)
        {
            xyz(name, "m");
            for (const char *angle : {"om", "fi", "ka"})
                h += "," + name + "_" + angle + "_deg";
        };
        pose("prepared_first");
        xyz("prepared_velocity_prior", "mps");
        xyz("prepared_velocity_prior_sigma", "mps");
        xyz("prepared_offset_prior", "mps2");
        xyz("prepared_offset_prior_sigma", "mps2");
        h += ",prepared_velocity_offset_cov_xx,prepared_velocity_offset_cov_yy,prepared_velocity_offset_cov_zz";
        xyz("prepared_gravity", "g");
        xyz("prepared_bias_prior", "deg_s");
        xyz("prepared_bias_prior_sigma", "deg_s");
        h += ",handoff";
        xyz("handed_velocity_end", "mps");
        xyz("handed_velocity_end_sigma", "mps");
        xyz("handed_gravity", "g");
        h += ",handed_to_mps2_before_fold";
        xyz("handed_offset_sigma", "mps2");
        h += ",handed_velocity_offset_cov_xx,handed_velocity_offset_cov_yy,handed_velocity_offset_cov_zz";
        xyz("handed_bias", "deg_s");
        xyz("handed_bias_sigma", "deg_s");
        h += ",handed_last_solved_time_s";
        pose("handed_last_solved");
        h += ",handed_last_time_s";
        pose("handed_last");
        h += ",rotation_rate_mean_deg_s";
        xyz("start_acceleration", "mps2");
        // the normalised moves of the verdict (consistency-harness D10): value (empty when NOT MEASURED), the reason, and
        // the trace reduction n - tr(P^-1 P_post) where it was computed
        for (const char *move : {"move_start_velocity", "move_joint", "move_gyro_bias"})
            h += std::string(",") + move + "," + move + "_reason," + move + "_reduction";
        return h;
    }

    // the columns of an absent value: empty fields
    std::string empty_fields(int n) { return std::string(n, ','); }

    std::string vector3_or_empty(const std::optional<Eigen::Vector3d> &v, double scale = 1.0)
    {
        return v ? "," + vector3(*v * scale) : empty_fields(3);
    }

    std::string sigmas_or_empty(const std::optional<Eigen::Matrix3d> &cov, double scale = 1.0)
    {
        return cov ? "," + vector3(cov->diagonal().cwiseMax(0.0).cwiseSqrt() * scale) : empty_fields(3);
    }

    std::string diagonal_or_empty(const std::optional<Eigen::Matrix3d> &m) { return m ? "," + vector3(m->diagonal()) : empty_fields(3); }

    std::string pose_or_empty(const std::optional<Eigen::Affine3d> &pose)
    {
        if (!pose)
            return empty_fields(6);
        const TaitBryanPose p = pose_tait_bryan_from_affine_matrix(*pose);
        return "," + vector3(Eigen::Vector3d(p.px, p.py, p.pz)) + "," + vector3(Eigen::Vector3d(p.om, p.fi, p.ka) * RAD_TO_DEG);
    }

    std::string node_or_empty(const std::optional<WindowNode> &node)
    {
        return node ? "," + number(node->time_s) + pose_or_empty(node->pose) : empty_fields(7);
    }

    const char *handoff_name(WindowHandOff h)
    {
        switch (h)
        {
        case WindowHandOff::done:
            return "done";
        case WindowHandOff::cancelled:
            return "cancelled";
        default:
            return "";
        }
    }

    // the columns of consistency-harness D9, in the header's order
    std::string handed_on_fields(const WindowTermRecord &r)
    {
        const WindowPreparedState &p = r.prepared;
        std::string s = pose_or_empty(p.first_pose) + vector3_or_empty(p.velocity_prior_mean) + sigmas_or_empty(p.velocity_prior_cov) +
                        vector3_or_empty(p.offset_prior_mean) + sigmas_or_empty(p.offset_prior_cov) +
                        diagonal_or_empty(p.velocity_offset_prior_cov) + vector3_or_empty(p.gravity) +
                        vector3_or_empty(p.bias_prior_mean_rad_s, RAD_TO_DEG) + sigmas_or_empty(p.bias_prior_cov, RAD_TO_DEG);
        s += std::string(",") + handoff_name(r.handoff);
        const WindowHandedOnState none{};
        const WindowHandedOnState &h = r.handoff == WindowHandOff::done ? r.handed_on : none;
        s += vector3_or_empty(h.velocity_end) + sigmas_or_empty(h.velocity_end_cov) + vector3_or_empty(h.gravity) + "," +
             (h.to_mps2_before_fold ? number(*h.to_mps2_before_fold) : std::string()) + sigmas_or_empty(h.offset_cov) +
             diagonal_or_empty(h.velocity_offset_cov) + vector3_or_empty(h.bias_mean_rad_s, RAD_TO_DEG) +
             sigmas_or_empty(h.bias_cov, RAD_TO_DEG) + node_or_empty(h.last_node_solved) + node_or_empty(h.last_node_handed_on);
        s += "," + (r.rotation_rate_rad_s ? number(*r.rotation_rate_rad_s * RAD_TO_DEG) : std::string()) +
             vector3_or_empty(r.start_acceleration_mps2);
        return s;
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
              << number(r.lm_factor) << "," << (r.at_rest ? 1 : 0) << ",0," << (r.acc_term ? 1 : 0) << ","
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
    const WindowNoiseSigmas &n = r.noise;
    const auto sigma = [](double x) { return std::isfinite(x) ? number(x) : std::string(); };
    for (const Eigen::Vector3d *v : {&n.translation_m, &n.rotation_min_deg, &n.rotation_median_deg, &n.rotation_max_deg})
        features_ << "," << sigma(v->x()) << "," << sigma(v->y()) << "," << sigma(v->z());
    features_ << "," << sigma(n.velocity_continuity_min_mps) << "," << sigma(n.velocity_continuity_max_mps) << "," << sigma(n.accelerometer_min_mps)
              << "," << sigma(n.accelerometer_max_mps);
    for (const Eigen::Vector3d *v : {&n.velocity_prior_mps, &n.bias_prior_deg_s, &n.offset_prior_mps2})
        features_ << "," << sigma(v->x()) << "," << sigma(v->y()) << "," << sigma(v->z());
    features_ << "," << sigma(n.zero_velocity_mps) << "," << sigma(n.ndt_information_scale) << "," << sigma(n.ndt_huber_cutoff);
    features_ << handed_on_fields(r);
    for (const NormalisedMove *m : {&e.moves.start_velocity, &e.moves.joint, &e.moves.gyro_bias})
        features_ << "," << opt(m->value) << "," << move_not_measured_name(m->reason) << "," << opt(m->reduction);
    features_ << "\n";
    terms_.flush();
    features_.flush();
    failed_ = !terms_ || !features_;
}
