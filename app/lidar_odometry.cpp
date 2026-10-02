#include "lidar_odometry.h"
#include "mandeye_recording_source.h" // load_point_cloud for the GUI's reference clouds
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
    Trajectory& trajectory, Imu& imu_data, bool fusionConventionNwu, bool fusionConventionEnu, bool fusionConventionNed, double ahrs_gain, bool debugMsg, bool use_remove_imu_bias_from_first_stationary_scan, Logger& logger)
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
    std::vector<std::vector<Point3Di>>& pointsPerFile, const OdometryConfig &params, OdometryState& state, Trajectory& trajectory, std::vector<WorkerData>& worker_data, PointStore& point_store, Logger& logger, ProgressReporter& progress, const Imu* imu_data)
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
    worker_data.clear();
    worker_data.reserve(n_iter);

    for (size_t i = 0; i < n_iter; i++)
    {
        if (i % 50 == 0)
            progress.progress(i + 1, n_iter);

        WorkerData& wd = worker_data.emplace_back();  // construct in-place
        wd.intermediate_trajectory.reserve(threshold);
        wd.intermediate_trajectory_motion_model.reserve(threshold);
        wd.intermediate_trajectory_timestamps.reserve(threshold);
        wd.imu_om_fi_ka.reserve(threshold);

        const size_t start_idx = i * threshold;
        const size_t end_idx = (i + 1) * threshold;

        for (size_t idx = start_idx; idx < end_idx; idx++)
        {
            wd.intermediate_trajectory.emplace_back(poses[idx]);
            wd.intermediate_trajectory_motion_model.emplace_back(poses[idx]);
            wd.intermediate_trajectory_timestamps.emplace_back(timestamps[idx]);

            const TaitBryanPose tb = pose_tait_bryan_from_affine_matrix(poses[idx]);
            wd.imu_om_fi_ka.emplace_back(tb.om, tb.fi, tb.ka);

            if (!accelerations.empty())
                wd.imu_acc.emplace_back(accelerations[idx]);
        }

        std::vector<Point3Di> points;

        for (size_t index = 0; index < pointsPerFile.size(); index++)
        {
            auto lower = std::lower_bound(
                pointsPerFile[index].begin(), pointsPerFile[index].end(),
                wd.intermediate_trajectory_timestamps[0].first,
                [](const Point3Di& point, double timestamp)
                {
                    return point.timestamp < timestamp;
                });
            auto upper = std::lower_bound(
                pointsPerFile[index].begin(), pointsPerFile[index].end(),
                wd.intermediate_trajectory_timestamps[wd.intermediate_trajectory_timestamps.size() - 1].first,
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
            auto lower = std::lower_bound(wd.intermediate_trajectory_timestamps.begin(), wd.intermediate_trajectory_timestamps.end(), p.timestamp,
                [](std::pair<double, double> lhs, double rhs) -> bool
                { return lhs.first < rhs; });

            int index_pose = std::distance(wd.intermediate_trajectory_timestamps.begin(), lower) - 1;

            if (index_pose >= 0 && index_pose < wd.intermediate_trajectory_timestamps.size() &&
                std::abs(p.timestamp - wd.intermediate_trajectory_timestamps[index_pose].first) <= 0.01)
                p.index_pose = index_pose;
            else
                p.index_pose = -1;
        }

        auto it = std::remove_if(points.begin(), points.end(),
            [](const Point3Di& p) { return p.index_pose == -1; });
        points.erase(it, points.end());

        if (points.size() <= 1000)
            worker_data.pop_back();
        else
        {
            std::vector<Point3Di> intermediate_points;

            if (params.decimation > 0.0 && points.size() > 1000)
                intermediate_points = decimate(points, params.decimation, params.decimation, params.decimation);

            const std::size_t window = worker_data.size() - 1;

            if (!point_store.save(PointSet::WindowOriginal, window, points))
                log_info(logger) << "problem with point store: original points of window " << window;

            // ensure we always have intermediate points
            if (!point_store.save(PointSet::WindowIntermediate, window, intermediate_points.size() > 0 ? intermediate_points : points))
                log_info(logger) << "problem with point store: intermediate points of window " << window;
        }

        progress.checkpoint();
    }
    progress.progress(n_iter, n_iter);

    state.m_g = worker_data[0].intermediate_trajectory[0];
    return true;
}

void run_consistency(std::vector<WorkerData> &worker_data, const OdometryConfig &params, PointStore &point_store, Logger &logger)
{
    log_info(logger) << "Point cloud consistency and trajectory smoothness START";
    for (int i = 0; i < params.num_constistency_iter; i++)
    {
        log_info(logger) << "Iteration " << i + 1 << " of " << params.num_constistency_iter;
        for (int ii = 0; ii < worker_data.size(); ii++)
        {
            worker_data[ii].intermediate_trajectory_motion_model = worker_data[ii].intermediate_trajectory;
        }
        if (!params.use_mutliple_gaussian)
        {
            Consistency(worker_data, params, point_store, logger);
        }
        else
        {
            Consistency2(worker_data, params, point_store, logger);
        }
    }
    log_info(logger) << "Point cloud consistency and trajectory smoothness FINISHED";
}

namespace
{
    // one LAZ with all chunks in world coordinates, written chunk by chunk (memory stays flat on long recordings);
    // the header's point count and bounding box are filled in from the written points when it is closed
    class MergedLazWriter
    {
    public:
        explicit MergedLazWriter(Logger &logger) : logger_(&logger) {}

        bool open(const std::string &filename)
        {
            if (laszip_create(&writer_))
                return false;
            laszip_header *header;
            if (laszip_get_header_pointer(writer_, &header))
                return false;
            header->file_source_ID = 4711;
            header->global_encoding = (1 << 0);
            header->version_major = 1;
            header->version_minor = 2;
            header->point_data_format = 1;
            header->point_data_record_length = 28;
            header->x_scale_factor = header->y_scale_factor = header->z_scale_factor = 0.0001;
            header->x_offset = header->y_offset = header->z_offset = 0.0;
            if (laszip_open_writer(writer_, filename.c_str(), 1) || laszip_get_point_pointer(writer_, &point_))
                return false;
            open_ = true;
            return true;
        }

        void write(const std::vector<Eigen::Vector3d> &points, const std::vector<unsigned short> &intensity, const std::vector<double> &timestamps)
        {
            if (!open_)
                return;
            for (size_t i = 0; i < points.size(); i++)
            {
                laszip_F64 coordinates[3] = {points[i].x(), points[i].y(), points[i].z()};
                point_->intensity = intensity[i];
                point_->gps_time = timestamps[i] * 1e9;
                if (laszip_set_coordinates(writer_, coordinates) || laszip_write_point(writer_) || laszip_update_inventory(writer_))
                {
                    log_error(*logger_) << "merged LAZ: write error, file incomplete";
                    close();
                    return;
                }
                count_++;
            }
        }

        void close()
        {
            if (!open_)
                return;
            laszip_close_writer(writer_);
            laszip_destroy(writer_);
            open_ = false;
        }

        long long count() const { return count_; }

    private:
        Logger *logger_;
        laszip_POINTER writer_ = nullptr;
        laszip_point *point_ = nullptr;
        bool open_ = false;
        long long count_ = 0;
    };
} // namespace

void save_result(std::vector<WorkerData> &worker_data, const OdometryConfig &params, const OdometryState &state, const GuiPreferences &gui, fs::path outwd, double elapsed_time_s, ResultSink &sink, PointStore &point_store, Logger &logger)
{
    sink.begin();
    // concatenate data
    std::vector<WorkerData> worker_data_concatenated;
    WorkerData wd;
    int counter = 0;
    int pose_offset = 0;
    std::vector<int> point_sizes_per_chunk;

    std::vector<Point3Di> original_points_to_save;
    int original_points_to_save_counter = 0;
    for (int i = 0; i < worker_data.size(); i++)
    {
        std::vector<Point3Di> original_points;
        if (!point_store.load(PointSet::WindowOriginal, i, original_points))
            log_info(logger) << "problem with point store: original points of window " << i;

        if (i % 1000 == 0)
        {
            log_info(logger) << "processing worker_data " << i + 1 << "/" << (int)worker_data.size() << " ";
        }
        auto tmp_data = original_points;
        point_sizes_per_chunk.push_back(tmp_data.size());
        for (auto &t : tmp_data)
        {
            t.index_pose += pose_offset;
        }

        wd.intermediate_trajectory.insert(std::end(wd.intermediate_trajectory),
                                          std::begin(worker_data[i].intermediate_trajectory), std::end(worker_data[i].intermediate_trajectory));

        wd.intermediate_trajectory_timestamps.insert(std::end(wd.intermediate_trajectory_timestamps),
                                                     std::begin(worker_data[i].intermediate_trajectory_timestamps), std::end(worker_data[i].intermediate_trajectory_timestamps));

        original_points_to_save.insert(std::end(original_points_to_save),
                                       std::begin(tmp_data), std::end(tmp_data));

        wd.imu_om_fi_ka.insert(std::end(wd.imu_om_fi_ka), std::begin(worker_data[i].imu_om_fi_ka), std::end(worker_data[i].imu_om_fi_ka));
        pose_offset += worker_data[i].intermediate_trajectory.size();

        counter++;
        if (counter > 50)
        {
            log_info(logger) << "session chunk " << original_points_to_save_counter;

            if (!point_store.save(PointSet::SessionChunk, original_points_to_save_counter, original_points_to_save))
                log_info(logger) << "problem with point store: session chunk " << original_points_to_save_counter;

            original_points_to_save_counter++;

            worker_data_concatenated.push_back(wd);
            wd.intermediate_trajectory.clear();
            wd.intermediate_trajectory_timestamps.clear();
            original_points_to_save.clear();
            wd.imu_om_fi_ka.clear();

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

        worker_data_concatenated.push_back(wd);
    }

    sink.point_sizes(point_sizes_per_chunk);

    std::vector<Eigen::Affine3d> m_poses;
    std::vector<std::string> file_names;
    std::vector<std::vector<int>> index_poses;
    MergedLazWriter merged(logger);
    if (params.export_merged_laz && !merged.open((outwd / "scan_lio_merged.laz").string()))
        log_error(logger) << "can not open " << (outwd / "scan_lio_merged.laz") << " for writing";

    for (int i = 0; i < worker_data_concatenated.size(); i++)
    {
        log_info(logger) << "------------------------";
        fs::path path(outwd);
        std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
        path /= filename;
        log_info(logger) << "saving to: " << path;
        std::vector<int> index_poses_i;

        std::vector<Eigen::Vector3d> global_pointcloud;
        std::vector<unsigned short> intensity;
        std::vector<double> timestamps;

        std::vector<Point3Di> original_points_to_save;
        if (!point_store.load(PointSet::SessionChunk, i, original_points_to_save))
            log_info(logger) << "problem with point store: session chunk " << i;

        points_to_vector(
            original_points_to_save, worker_data_concatenated[i].intermediate_trajectory,
            params.threshould_output_filter, &index_poses_i, global_pointcloud, intensity, timestamps, true, params.save_index_pose);
        exportLaz(path.string(), global_pointcloud, intensity, timestamps);

        if (params.export_merged_laz)
        {
            // chunk points are stored relative to the chunk's first pose: back to world coordinates
            const Eigen::Affine3d &chunk_pose = worker_data_concatenated[i].intermediate_trajectory[0];
            std::vector<Eigen::Vector3d> world(global_pointcloud.size());
            for (size_t k = 0; k < global_pointcloud.size(); k++)
                world[k] = chunk_pose * global_pointcloud[k];
            merged.write(world, intensity, timestamps);
        }
        
        if(params.save_index_pose){
            index_poses.push_back(index_poses_i);
        }
        
        m_poses.push_back(worker_data_concatenated[i].intermediate_trajectory[0]);
        file_names.push_back(filename);

        // save trajectory
        std::string trajectory_filename = ("trajectory_lio_" + std::to_string(i) + ".csv");
        fs::path pathtrj(outwd);
        pathtrj /= trajectory_filename;
        log_info(logger) << "saving to: " << pathtrj;

        std::ofstream outfile;
        outfile.open(pathtrj);
        if (!outfile.good())
        {
            log_info(logger) << "can not save file: " << pathtrj;
            return;
        }

        outfile << "timestamp_nanoseconds pose00 pose01 pose02 pose03 pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 timestampUnix_nanoseconds om_rad fi_rad ka_rad" << std::endl;
        for (int j = 0; j < worker_data_concatenated[i].intermediate_trajectory.size(); j++)
        {
            auto pose = worker_data_concatenated[i].intermediate_trajectory[0].inverse() * worker_data_concatenated[i].intermediate_trajectory[j];

            outfile
                << std::setprecision(20) << worker_data_concatenated[i].intermediate_trajectory_timestamps[j].first * 1e9 << " " << std::setprecision(10)
                << pose(0, 0) << " "
                << pose(0, 1) << " "
                << pose(0, 2) << " "
                << pose(0, 3) << " "
                << pose(1, 0) << " "
                << pose(1, 1) << " "
                << pose(1, 2) << " "
                << pose(1, 3) << " "
                << pose(2, 0) << " "
                << pose(2, 1) << " "
                << pose(2, 2) << " "
                << pose(2, 3) << " "
                << std::setprecision(20) << worker_data_concatenated[i].intermediate_trajectory_timestamps[j].second * 1e9 << " "
                << worker_data_concatenated[i].imu_om_fi_ka[j].x() << " "
                << worker_data_concatenated[i].imu_om_fi_ka[j].y() << " "
                << worker_data_concatenated[i].imu_om_fi_ka[j].z() << " "
                << std::endl;
        }
        outfile.close();
    }
    if (params.export_merged_laz)
    {
        merged.close();
        log_info(logger) << "saved " << merged.count() << " points to " << (outwd / "scan_lio_merged.laz");
    }

    sink.chunk_poses(m_poses);

    if (params.save_index_pose)
        sink.index_poses(index_poses);

    SessionSummary summary;
    summary.trajectory_length_m = state.total_length_of_calculated_trajectory;
    summary.elapsed_seconds = elapsed_time_s;
    summary.decimation = params.decimation;
    summary.threshold_nr_poses = params.threshold_nr_poses;
    summary.index_poses = params.save_index_pose;
    summary.chunk_count = worker_data_concatenated.size();
    sink.session(summary);

    // parameters (loadable TOML) and the processing results report
    sink.parameters(params, state.paths, gui);
    sink.processing_results(params, state, worker_data, elapsed_time_s);
}

void filter_reference_buckets(const OdometryConfig &params, OdometryState &state)
{
    NDTBucketMapType reference_buckets_out;
    for (const auto &b : state.reference_buckets)
    {
        if (b.second.number_of_points > 10)
        {
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(b.second.cov, Eigen::ComputeEigenvectors);

            auto eigen_values = eigen_solver.eigenvalues();
            double sum_ev = eigen_values.x() + eigen_values.y() + eigen_values.z();
            double ev1 = eigen_values.x();
            double ev2 = eigen_values.y();
            double ev3 = eigen_values.z();

            double planarity = 1 - ((3 * ev1 / sum_ev) * (3 * ev2 / sum_ev) * (3 * ev3 / sum_ev));
            if (planarity > 0.7)
            {
                reference_buckets_out[b.first] = b.second;
            }
        }
    }
    state.reference_buckets = reference_buckets_out;
}

void save_trajectory_to_ascii(std::vector<WorkerData> &worker_data, std::string output_file_name)
{
    ofstream file;
    file.open(output_file_name);
    for (const auto &wd : worker_data)
    {
        for (const auto &it : wd.intermediate_trajectory)
        {
            file << it(0, 3) << " " << it(1, 3) << " " << it(2, 3) << std::endl;
        }
    }
    file.close();
}

void load_reference_point_clouds(std::vector<std::string> input_file_names, const OdometryConfig &params, OdometryState &state, Logger &logger)
{
    state.reference_buckets.clear();
    state.reference_points.clear();

    for (size_t i = 0; i < input_file_names.size(); i++)
    {
        log_info(logger) << "loading reference point cloud from: " << input_file_names[i];
        auto pp = load_point_cloud(input_file_names[i].c_str(), false, 0, 10000, get_lidar_imu_lever_arm(params), logger);
        log_info(logger) << "loaded " << pp.size() << " reference points";
        state.reference_points.insert(std::end(state.reference_points), std::begin(pp), std::end(pp));
    }

    update_rgd(params.in_out_params_indoor, state.reference_buckets, state.reference_points);
    state.buckets_indoor = state.reference_buckets;
}
