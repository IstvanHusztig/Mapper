#include "lidar_odometry.h"
#include "consistency.h"
#include "tbb/tbb.h"
#include <mutex>

namespace fs = std::filesystem;

const float RAD_TO_DEG = 180.0f / M_PI;

bool load_data(RecordingSource& recording,
    PreviewSink& preview,
    const OdometryConfig &params,
    std::vector<std::vector<Point3Di>>& pointsPerFile,
    Imu& imu_data,
    bool debugMsg,
    Logger& logger)
{
    log_info(logger) << "Loading data..\n--------------\n";

    // --- Load IMU data
    log_info(logger) << "loading IMU data from 'imu****.csv' ..\n";

    std::size_t minSize = recording.chunk_count();
    // Each thread loads into its own local vector
    std::vector<Imu> imuChunks(minSize);

    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, minSize),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i < range.end(); ++i)
            {
                imuChunks[i] = recording.read_imu(i, logger);
            }
        });

    // Merge all chunks sequentially at the end
    for (auto& chunk : imuChunks)
        imu_data.insert(imu_data.end(), std::make_move_iterator(chunk.begin()), std::make_move_iterator(chunk.end()));

    // Sort once after merging
    std::sort(imu_data.begin(), imu_data.end(),
        [](const auto& a, const auto& b) {
            return std::get<0>(a).first < std::get<0>(b).first;
        });

    log_info(logger) << "Loading point clouds (" << minSize << " files)..\n";
    log_info(logger) << "total   - filter = kept\n-----------------------\n";

    pointsPerFile.resize(minSize);

    PointFilter filter;
    filter.omit_zero_timestamp = true;
    filter.xy_inner = params.filter_threshold_xy_inner;
    filter.xy_outer = params.filter_threshold_xy_outer;
    filter.lever_arm = get_lidar_imu_lever_arm(params);

    // --- Parallel load of LAZ files
    tbb::parallel_for(size_t(0), minSize, [&](size_t i)
        {
            auto data = recording.read_points(i, filter, logger);

            std::sort(data.begin(), data.end(), [](const Point3Di& a, const Point3Di& b)
                { return a.timestamp < b.timestamp; });

            // Optional calibration validation (first file only)
            if ((i == 0) && params.save_calibration_validation)
                preview.calibration_validation(data, (size_t)params.calibration_validation_points, logger);

            // Store results
            pointsPerFile[i] = std::move(data);
        });

    // --- Summary
    size_t totalPoints = 0;
    for (const auto& pp : pointsPerFile)
        totalPoints += pp.size();

    log_info(logger) << "TOTAL: " << totalPoints << "\n..loading finished.\n\n";
    return true;
}

void calculate_trajectory(
    ImuTrajectory& trajectory, Imu& imu_data, bool fusionConventionNwu, bool fusionConventionEnu, bool fusionConventionNed, double ahrs_gain, bool debugMsg, bool use_remove_imu_bias_from_first_stationary_scan, Logger& logger)
{
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    if (fusionConventionNwu)
        ahrs.settings.convention = FusionConventionNwu;
    else if (fusionConventionEnu)
        ahrs.settings.convention = FusionConventionEnu;
    else if (fusionConventionNed)
        ahrs.settings.convention = FusionConventionNed;
    ahrs.settings.gain = ahrs_gain;
    int counter = 1;

    bool first = true;
    double last_ts = 0.0;

    log_info(logger) << "start calculating trajectory..";

    double bias_gyr_x = 0.0;
    double bias_gyr_y = 0.0;
    double bias_gyr_z = 0.0;

    if (use_remove_imu_bias_from_first_stationary_scan)
    {
        if (imu_data.size() > 1000)
        {
            std::vector<double> gyr_x;
            std::vector<double> gyr_y;
            std::vector<double> gyr_z;

            for (int i = 0; i < 1000; i++)
            {
                const auto& [timestamp_pair, gyr, acc] = imu_data[i];

                gyr_x.push_back(gyr.axis.x * RAD_TO_DEG);
                gyr_y.push_back(gyr.axis.y * RAD_TO_DEG);
                gyr_z.push_back(gyr.axis.z * RAD_TO_DEG);
            }

            double mean_x = 0, mean_y = 0, mean_z = 0;
            for (int i = 0; i < 1000; i++)
            {
                mean_x += gyr_x[i];
                mean_y += gyr_y[i];
                mean_z += gyr_z[i];
            }
            mean_x /= 1000.0;
            mean_y /= 1000.0;
            mean_z /= 1000.0;

            double var_x = 0, var_y = 0, var_z = 0;
            for (int i = 0; i < 1000; i++)
            {
                var_x += (gyr_x[i] - mean_x) * (gyr_x[i] - mean_x);
                var_y += (gyr_y[i] - mean_y) * (gyr_y[i] - mean_y);
                var_z += (gyr_z[i] - mean_z) * (gyr_z[i] - mean_z);
            }
            double total_variance = (var_x + var_y + var_z) / 1000.0;

            if (total_variance > 5.0)
            {
                log_info(logger) << "WARNING: Excessive movement detected (" << total_variance << "). Bias set to 0.";
                bias_gyr_x = 0.0;
                bias_gyr_y = 0.0;
                bias_gyr_z = 0.0;
            }
            else
            {
                std::nth_element(gyr_x.begin(), gyr_x.begin() + 500, gyr_x.end());
                std::nth_element(gyr_y.begin(), gyr_y.begin() + 500, gyr_y.end());
                std::nth_element(gyr_z.begin(), gyr_z.begin() + 500, gyr_z.end());
                bias_gyr_x = gyr_x[500];
                bias_gyr_y = gyr_y[500];
                bias_gyr_z = gyr_z[500];
            }
        }

        log_info(logger) << "------------------";
        log_info(logger) << "GYRO BIAS";
        log_info(logger) << "bias_gyr_x [deg/s]: " << bias_gyr_x;
        log_info(logger) << "bias_gyr_y [deg/s]: " << bias_gyr_y;
        log_info(logger) << "bias_gyr_z [deg/s]: " << bias_gyr_z;
        log_info(logger) << "------------------";
    }

    for (const auto& [timestamp_pair, gyr, acc] : imu_data)
    {
        const FusionVector gyroscope = { static_cast<float>(gyr.axis.x * RAD_TO_DEG) - bias_gyr_x, static_cast<float>(gyr.axis.y * RAD_TO_DEG) - bias_gyr_y, static_cast<float>(gyr.axis.z * RAD_TO_DEG) - bias_gyr_z };
        const FusionVector accelerometer = { acc.axis.x , acc.axis.y, acc.axis.z };
     
        // DYNAMIC AHRS GAIN: Check accelerometer magnitude against 1.0G
        double acc_mag = std::sqrt(acc.axis.x * acc.axis.x + acc.axis.y * acc.axis.y + acc.axis.z * acc.axis.z);
        if (acc_mag < 0.9 || acc_mag > 1.1)
        {
            ahrs.settings.gain = 0.0f; // Drop gain to prevent linear acceleration pitch/roll drift
        }
        else
        {
            ahrs.settings.gain = ahrs_gain; // Restore user gain when stable
        }

        if (first)
        {
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1.0 / 200.0);
            first = false;
        }
        else
        {
            double curr_ts = timestamp_pair.first;
            double ts_diff = curr_ts - last_ts;

            // Safety catch for catastrophic cancellation jitter
            if (ts_diff <= 0.0)
                ts_diff = 0.002;

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, ts_diff);
        }

        last_ts = timestamp_pair.first;

        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

        Eigen::Quaterniond d{ quat.element.w, quat.element.x, quat.element.y, quat.element.z };
        Eigen::Affine3d t{ Eigen::Matrix4d::Identity() };
        t.rotate(d);

        trajectory[timestamp_pair.first] = std::pair(t.matrix(), timestamp_pair.second);

        if (debugMsg)
        {
            const FusionEuler euler = FusionQuaternionToEuler(quat);
            counter++;
            if (counter % 100 == 0)
            {
                log_debug(logger) << std::fixed << std::setprecision(1) << "Roll " << euler.angle.roll << ", Pitch " << euler.angle.pitch << ", Yaw " << euler.angle.yaw
                                  << " [" << counter++ << " of " << (int)imu_data.size() << "]";
            }
        }
    }
}

bool compute_step_1(
    std::vector<std::vector<Point3Di>>& pointsPerFile, const OdometryConfig &params, OdometryState& state, ImuTrajectory& trajectory, std::vector<Window>& windows, PointStore& point_store, Logger& logger, ProgressReporter& progress, const Imu* imu_data)
{
    int number_of_initial_points = 0;
    double timestamp_begin = 0.0;

    // Step 1: Pre-reserve to avoid vector reallocations
    state.initial_points.reserve(params.threshold_initial_points);

    for (const auto& pp : pointsPerFile)
    {
        for (const auto& p : pp)
        {
            number_of_initial_points++;
            state.initial_points.push_back(p);
            if (number_of_initial_points > params.threshold_initial_points)
            {
                timestamp_begin = p.timestamp;
                break;
            }
        }
        if (number_of_initial_points > params.threshold_initial_points)
            break;
    }

    std::vector<std::pair<double, double>> timestamps;
    std::vector<Eigen::Affine3d> poses;
    timestamps.reserve(trajectory.size());
    poses.reserve(trajectory.size());

    for (const auto& [time, pose_pair] : trajectory)
    {
        if (time < timestamp_begin)
            continue;

        timestamps.emplace_back(time, pose_pair.second);
        poses.emplace_back(pose_pair.first);
    }

    timestamps.shrink_to_fit();
    poses.shrink_to_fit();

    log_info(logger) << "number of poses: " << poses.size();

    if (poses.empty())
    {
        log_error(logger) << "Loading poses went wrong! Could not load poses!";
        return false;
    }

    // raw accelerometer of each trajectory node (the trajectory has one node per IMU sample, keyed by its timestamp)
    std::vector<Eigen::Vector3d> accelerations;
    if (imu_data && !imu_data->empty())
    {
        accelerations.reserve(timestamps.size());
        size_t j = 0;
        for (const auto& ts : timestamps)
        {
            while (j + 1 < imu_data->size() && std::get<0>((*imu_data)[j]).first < ts.first)
                j++;
            const FusionVector& a = std::get<2>((*imu_data)[j]);
            accelerations.emplace_back(a.axis.x, a.axis.y, a.axis.z);
        }
    }

    int threshold = params.threshold_nr_poses;

    const int n_iter = poses.size() / threshold;
    windows.clear();
    windows.reserve(n_iter);

    for (size_t i = 0; i < n_iter; i++)
    {
        if (i % 50 == 0)
            progress.progress(i + 1, n_iter);

        Window& wd = windows.emplace_back();  // construct in-place
        wd.poses.reserve(threshold);
        wd.motion_model_poses.reserve(threshold);
        wd.timestamps.reserve(threshold);
        wd.imu_angles.reserve(threshold);

        const size_t start_idx = i * threshold;
        const size_t end_idx = (i + 1) * threshold;

        for (size_t idx = start_idx; idx < end_idx; idx++)
        {
            wd.poses.emplace_back(poses[idx]);
            wd.motion_model_poses.emplace_back(poses[idx]);
            wd.timestamps.emplace_back(timestamps[idx]);

            const TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(poses[idx]);
            wd.imu_angles.emplace_back(tb.om, tb.fi, tb.ka);

            if (!accelerations.empty())
                wd.imu_acc.emplace_back(accelerations[idx]);
        }

        std::vector<Point3Di> points;

        for (size_t index = 0; index < pointsPerFile.size(); index++)
        {
            auto lower = std::lower_bound(
                pointsPerFile[index].begin(), pointsPerFile[index].end(),
                wd.timestamps[0].first,
                [](const Point3Di& point, double timestamp)
                {
                    return point.timestamp < timestamp;
                });
            auto upper = std::lower_bound(
                pointsPerFile[index].begin(), pointsPerFile[index].end(),
                wd.timestamps[wd.timestamps.size() - 1].first,
                [](const Point3Di& point, double timestamp)
                {
                    return point.timestamp < timestamp;
                });
            points.reserve(points.size() + std::distance(lower, upper));
            if (lower != upper)
                points.insert(points.end(), std::make_move_iterator(lower), std::make_move_iterator(upper));
        }

        for (size_t k = 0; k < points.size(); k++)
        {
            Point3Di& p = points[k];
            auto lower = std::lower_bound(wd.timestamps.begin(), wd.timestamps.end(), p.timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                { return lhs.first < rhs; });

            int index_pose = std::distance(wd.timestamps.begin(), lower) - 1;

            if (index_pose >= 0 && index_pose < wd.timestamps.size() &&
                std::abs(p.timestamp - wd.timestamps[index_pose].first) <= 0.01)
                p.index_pose = index_pose;
            else
                p.index_pose = -1;
        }

        auto it = std::remove_if(points.begin(), points.end(),
            [](const Point3Di& p) { return p.index_pose == -1; });
        points.erase(it, points.end());

        if (points.size() <= 1000)
            windows.pop_back();
        else
        {
            std::vector<Point3Di> intermediate_points;

            if (params.decimation > 0.0 && points.size() > 1000)
                intermediate_points = decimate(points, params.decimation, params.decimation, params.decimation);

            const std::size_t window = windows.size() - 1;

            if (!point_store.save(PointSet::WindowOriginal, window, points))
                log_info(logger) << "problem with point store: original points of window " << window;

            // ensure we always have intermediate points
            if (!point_store.save(PointSet::WindowIntermediate, window, intermediate_points.size() > 0 ? intermediate_points : points))
                log_info(logger) << "problem with point store: intermediate points of window " << window;
        }

        progress.checkpoint();
    }
    progress.progress(n_iter, n_iter);

    state.m_g = windows[0].poses[0];
    return true;
}

void run_consistency(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger)
{
    log_info(logger) << "Point cloud consistency and trajectory smoothness START";
    for (int i = 0; i < params.num_constistency_iter; i++)
    {
        log_info(logger) << "Iteration " << i + 1 << " of " << params.num_constistency_iter;
        for (int ii = 0; ii < windows.size(); ii++)
        {
            windows[ii].motion_model_poses = windows[ii].poses;
        }
        if (!params.use_mutliple_gaussian)
        {
            Consistency(windows, params, point_store, logger);
        }
        else
        {
            Consistency2(windows, params, point_store, logger);
        }
    }
    log_info(logger) << "Point cloud consistency and trajectory smoothness FINISHED";
}

void save_result(std::vector<Window> &windows, const OdometryConfig &params, const OdometryState &state, const GuiPreferences &gui, double elapsed_time_s, ResultSink &sink, PointStore &point_store, Logger &logger)
{
    sink.begin();
    // concatenate data
    std::vector<Window> chunks;
    Window wd;
    int counter = 0;
    int pose_offset = 0;
    std::vector<int> point_sizes_per_chunk;

    std::vector<Point3Di> original_points_to_save;
    int original_points_to_save_counter = 0;
    for (int i = 0; i < windows.size(); i++)
    {
        std::vector<Point3Di> original_points;
        if (!point_store.load(PointSet::WindowOriginal, i, original_points))
            log_info(logger) << "problem with point store: original points of window " << i;

        if (i % 1000 == 0)
        {
            log_info(logger) << "processing window " << i + 1 << "/" << (int)windows.size() << " ";
        }
        auto tmp_data = original_points;
        point_sizes_per_chunk.push_back(tmp_data.size());
        for (auto &t : tmp_data)
        {
            t.index_pose += pose_offset;
        }

        wd.poses.insert(std::end(wd.poses),
                                          std::begin(windows[i].poses), std::end(windows[i].poses));

        wd.timestamps.insert(std::end(wd.timestamps),
                                                     std::begin(windows[i].timestamps), std::end(windows[i].timestamps));

        original_points_to_save.insert(std::end(original_points_to_save),
                                       std::begin(tmp_data), std::end(tmp_data));

        wd.imu_angles.insert(std::end(wd.imu_angles), std::begin(windows[i].imu_angles), std::end(windows[i].imu_angles));
        pose_offset += windows[i].poses.size();

        counter++;
        if (counter > 50)
        {
            log_info(logger) << "session chunk " << original_points_to_save_counter;

            if (!point_store.save(PointSet::SessionChunk, original_points_to_save_counter, original_points_to_save))
                log_info(logger) << "problem with point store: session chunk " << original_points_to_save_counter;

            original_points_to_save_counter++;

            chunks.push_back(wd);
            wd.poses.clear();
            wd.timestamps.clear();
            original_points_to_save.clear();
            wd.imu_angles.clear();

            counter = 0;
            pose_offset = 0;
        }
    }

    if (counter > params.min_counter_concatenated_trajectory_nodes)
    {
        log_info(logger) << "session chunk " << original_points_to_save_counter;

        if (!point_store.save(PointSet::SessionChunk, original_points_to_save_counter, original_points_to_save))
            log_info(logger) << "problem with point store: session chunk " << original_points_to_save_counter;

        original_points_to_save_counter++;

        chunks.push_back(wd);
    }

    sink.point_sizes(point_sizes_per_chunk);

    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::vector<int>> index_poses;
    if (params.export_merged_laz)
        sink.open_merged_cloud();

    for (int i = 0; i < chunks.size(); i++)
    {
        std::vector<int> index_poses_i;

        std::vector<Eigen::Vector3d> global_pointcloud;
        std::vector<unsigned short> intensity;
        std::vector<double> timestamps;

        std::vector<Point3Di> original_points_to_save;
        if (!point_store.load(PointSet::SessionChunk, i, original_points_to_save))
            log_info(logger) << "problem with point store: session chunk " << i;

        points_to_vector(
            original_points_to_save, chunks[i].poses,
            params.threshould_output_filter, &index_poses_i, global_pointcloud, intensity, timestamps, true, params.save_index_pose);
        sink.chunk_cloud(i, global_pointcloud, intensity, timestamps);

        if (params.export_merged_laz)
        {
            // chunk points are stored relative to the chunk's first pose: back to world coordinates
            const Eigen::Affine3d &chunk_pose = chunks[i].poses[0];
            std::vector<Eigen::Vector3d> world(global_pointcloud.size());
            for (size_t k = 0; k < global_pointcloud.size(); k++)
                world[k] = chunk_pose * global_pointcloud[k];
            sink.merged_cloud(world, intensity, timestamps);
        }
        
        if(params.save_index_pose){
            index_poses.push_back(index_poses_i);
        }
        
        m_poses.push_back(chunks[i].poses[0]);

        // trajectory, relative to the chunk's first node
        std::vector<TrajectoryRow> rows;
        rows.reserve(chunks[i].poses.size());
        for (int j = 0; j < chunks[i].poses.size(); j++)
        {
            auto pose = chunks[i].poses[0].inverse() * chunks[i].poses[j];
            rows.push_back({chunks[i].timestamps[j], pose, chunks[i].imu_angles[j]});
        }
        if (!sink.chunk_trajectory(i, rows))
            return;
    }
    if (params.export_merged_laz)
        sink.close_merged_cloud();

    sink.chunk_poses(m_poses);

    if (params.save_index_pose)
        sink.index_poses(index_poses);

    SessionSummary summary;
    summary.trajectory_length_m = state.total_length_of_calculated_trajectory;
    summary.elapsed_seconds = elapsed_time_s;
    summary.decimation = params.decimation;
    summary.threshold_nr_poses = params.threshold_nr_poses;
    summary.index_poses = params.save_index_pose;
    summary.chunk_count = chunks.size();
    sink.session(summary);

    // parameters (loadable TOML) and the processing results report
    sink.parameters(params, state.paths, gui);
    sink.processing_results(params, state, windows, elapsed_time_s);
}

void save_trajectory_to_ascii(std::vector<Window> &windows, std::string output_file_name)
{
    ofstream file;
    file.open(output_file_name);
    for (const auto &wd : windows)
    {
        for (const auto &it : wd.poses)
        {
            file << it(0, 3) << " " << it(1, 3) << " " << it(2, 3) << std::endl;
        }
    }
    file.close();
}

