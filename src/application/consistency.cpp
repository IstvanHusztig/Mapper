#include "consistency.h"
#include "holdout.h"
#include "ndt_map.h"
#include "normal_equations.h"
#include <execution>
#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/smoothness_tait_bryan_wc_jacobian.h>

#include "angle_units.h"
#include <mutex>
#include <numeric>
#include <semaphore>

namespace
{
    // the NDT point terms of one chunk of points, summed per node in point order: the diagonal 6x6 block and the right-hand
    // side of every node from first_node on (a chunk spans consecutive nodes)
    struct ChunkTerms
    {
        int first_node = 0;
        int n_nodes = 0;
        std::vector<double> blocks; // n_nodes * 36, row-major 6x6 per node
        std::vector<double> rhs;    // n_nodes * 6
        ConsistencyTiming timing;   // this chunk's own timers (thread seconds), merged by the caller
    };

    // at most this many chunks are worked on at the same time (a chunk holds its global points and its NDT map)
    constexpr int chunks_in_flight = 8;
    constexpr size_t smoothness_batch = 4096;
}

void Consistency(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger, ConsistencyTiming *timing,
                 ConsistencyData *shared)
{
    // log_stage_timing: the timers get a target only when the caller passed one
    const auto phase = [timing](PhaseTime ConsistencyTiming::*p) { return timing ? &(timing->*p) : nullptr; };
    PhaseTimer pass_timer(phase(&ConsistencyTiming::pass));

    ConsistencyData local_data; // without the caller's data every call loads its own points
    ConsistencyData &data = shared ? *shared : local_data;

    std::vector<Eigen::Affine3d> trajectory;
    std::vector<std::pair<double, double>> timestamps;

    std::vector<std::pair<int, int>> indexes;

    log_info(logger) << "preparing data START";
    for (int i = 0; i < windows.size(); i++)
    {
        for (int j = 0; j < windows[i].poses.size(); j++)
        {
            trajectory.push_back(windows[i].poses[j]);
            timestamps.push_back(windows[i].timestamps[j]);
            indexes.emplace_back(i, j);
        }
    }
    log_info(logger) << "preparing data FINISHED";

    // The points, their nodes and the chunks do not change between the passes (the node of a point follows from its
    // timestamp), so they are prepared once per run. A chunk is a block of data.chunk_points points of the loaded order;
    // points whose time lies outside the trajectory are dropped from their chunk.
    PhaseTimer load_timer(phase(&ConsistencyTiming::load_points));
    if (!data.loaded)
    {
        std::vector<Point3Di> all_points_local;
        for (int i = 0; i < windows.size(); i++)
        {
            std::vector<Point3Di> intermediate_points;
            if (!point_store.load(PointSet::WindowIntermediate, i, intermediate_points))
                log_info(logger) << "problem with point store: intermediate points of window " << i;
            remove_held_out(intermediate_points, params.holdout_block_s);

            all_points_local.insert(all_points_local.end(), intermediate_points.begin(), intermediate_points.end());
        }

        // the node of every point, in parallel (independent per point); -1 marks a point outside the trajectory
        std::for_each(std::execution::par, all_points_local.begin(), all_points_local.end(),
                      [&](Point3Di &p)
                      {
                          auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), p.timestamp,
                                                        [](std::pair<double, double> lhs, double rhs) -> bool
                                                        { return lhs.first < rhs; });
                          const int index_pose = static_cast<int>(std::distance(timestamps.begin(), lower)) - 1;
                          p.index_pose = (index_pose >= 0 && index_pose < static_cast<int>(trajectory.size())) ? index_pose : -1;
                      });

        data.points.clear();
        data.points.reserve(all_points_local.size());
        data.chunk_begin.assign(1, 0);
        for (size_t index = 0; index < all_points_local.size(); index++)
        {
            if (index > 0 && index % data.chunk_points == 0)
                data.chunk_begin.push_back(data.points.size());
            if (all_points_local[index].index_pose >= 0)
                data.points.push_back(all_points_local[index]);
        }
        data.chunk_begin.push_back(data.points.size());
        data.loaded = true;
    }
    load_timer.stop();
    PhaseTimer smoothness_timer(phase(&ConsistencyTiming::smoothness));

    Eigen::Vector3d b(params.in_out_params_indoor.resolution_X, params.in_out_params_indoor.resolution_Y, params.in_out_params_indoor.resolution_Z);

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < trajectory.size(); i++)
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));

    double angle = 0.1 * DEG_TO_RAD;
    double wangle = 1.0 / (angle * angle);

    // The normal equations are accumulated directly: every term has the form A^T P A and A^T P B with A = -J. A point's
    // term touches only its own node's 6x6 block, the smoothness term the three nodes it spans, so the matrix is banded
    // (smoothness reaches 17 parameters from the diagonal) and needs no triplets and no sparse products.
    const int n_nodes = static_cast<int>(trajectory.size());
    const int n_params = n_nodes * 6;
    BandedNormalMatrix normal(n_params, 0, 17);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n_params);

    // smoothness: the terms of a batch are evaluated in parallel and added in node order, so the sums are those of a
    // sequential loop
    const double smoothness_weights[6] = {1000000, 100000000, 100000000, wangle, wangle, wangle};
    struct SmoothnessTerm
    {
        Eigen::Matrix<double, 18, 18> JtPJ;
        Eigen::Matrix<double, 18, 1> JtPd;
    };
    const size_t n_smoothness = poses.size() >= 2 ? poses.size() - 2 : 0; // nodes 1 .. size - 2
    std::vector<SmoothnessTerm> smoothness_terms(std::min(smoothness_batch, n_smoothness));
    std::vector<size_t> smoothness_offsets(smoothness_terms.size());
    std::iota(smoothness_offsets.begin(), smoothness_offsets.end(), size_t{0});
    for (size_t batch_begin = 0; batch_begin < n_smoothness; batch_begin += smoothness_batch)
    {
        const size_t batch_size = std::min(smoothness_batch, n_smoothness - batch_begin);
        std::for_each(std::execution::par, smoothness_offsets.begin(), smoothness_offsets.begin() + batch_size,
                      [&](size_t t)
                      {
                          const size_t i = 1 + batch_begin + t;
                          Eigen::Matrix<double, 6, 1> delta;
                          smoothness_obs_eq_tait_bryan_wc(delta,
                                                          poses[i - 1].px, poses[i - 1].py, poses[i - 1].pz, poses[i - 1].om, poses[i - 1].fi, poses[i - 1].ka,
                                                          poses[i].px, poses[i].py, poses[i].pz, poses[i].om, poses[i].fi, poses[i].ka,
                                                          poses[i + 1].px, poses[i + 1].py, poses[i + 1].pz, poses[i + 1].om, poses[i + 1].fi, poses[i + 1].ka);

                          Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
                          smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                                   poses[i - 1].px, poses[i - 1].py, poses[i - 1].pz, poses[i - 1].om, poses[i - 1].fi, poses[i - 1].ka,
                                                                   poses[i].px, poses[i].py, poses[i].pz, poses[i].om, poses[i].fi, poses[i].ka,
                                                                   poses[i + 1].px, poses[i + 1].py, poses[i + 1].pz, poses[i + 1].om, poses[i + 1].fi, poses[i + 1].ka);

                          // J^T P J and J^T P delta (the sign of A = -J drops out of A^T P A and flips A^T P B)
                          Eigen::Matrix<double, 6, 18, Eigen::RowMajor> weighted_jacobian = jacobian;
                          Eigen::Matrix<double, 6, 1> weighted_delta = delta;
                          for (int row = 0; row < 6; row++)
                          {
                              weighted_jacobian.row(row) *= smoothness_weights[row];
                              weighted_delta(row) *= smoothness_weights[row];
                          }
                          smoothness_terms[t].JtPJ = jacobian.transpose() * weighted_jacobian;
                          smoothness_terms[t].JtPd = jacobian.transpose() * weighted_delta;
                      });

        for (size_t t = 0; t < batch_size; t++)
        {
            const int first = static_cast<int>(batch_begin + t) * 6; // node i - 1 = batch_begin + t
            for (int row = 0; row < 18; row++)
            {
                for (int col = 0; col < 18; col++)
                    normal.add(first + row, first + col, smoothness_terms[t].JtPJ(row, col));
                rhs(first + row) -= smoothness_terms[t].JtPd(row);
            }
        }
    }

    // the first node is held where it is
    for (int k = 0; k < 6; k++)
        normal.add(k, k, 1000000);

    smoothness_timer.stop();

    // ndt: the chunks are independent (each has its own map), so they are computed in parallel, a bounded number at a time,
    // and their sums are merged in chunk order afterwards: the result does not depend on the thread timing
    log_info(logger) << "NDT observations START";
    const size_t n_chunks = data.chunk_begin.size() - 1;
    std::vector<ChunkTerms> chunk_terms(n_chunks);
    std::vector<size_t> chunk_ids(n_chunks);
    std::iota(chunk_ids.begin(), chunk_ids.end(), size_t{0});
    std::counting_semaphore<chunks_in_flight> gate(chunks_in_flight);

    const auto compute_chunk = [&](size_t c)
    {
        ChunkTerms &out = chunk_terms[c];
        const size_t begin = data.chunk_begin[c], end = data.chunk_begin[c + 1];
        if (end - begin <= 10000)
            return;

        PhaseTimer prepare_timer(&out.timing.chunk_prepare);
        std::vector<Point3Di> points_global(end - begin);
        int first_node = data.points[begin].index_pose, last_node = first_node;
        for (size_t i = begin; i < end; i++)
        {
            Point3Di pl = data.points[i];
            first_node = std::min(first_node, pl.index_pose);
            last_node = std::max(last_node, pl.index_pose);
            pl.point = trajectory[pl.index_pose] * pl.point;
            points_global[i - begin] = pl;
        }
        out.first_node = first_node;
        out.n_nodes = last_node - first_node + 1;
        out.blocks.assign(static_cast<size_t>(out.n_nodes) * 36, 0.0);
        out.rhs.assign(static_cast<size_t>(out.n_nodes) * 6, 0.0);
        prepare_timer.stop();

        NDTBucketMapType buckets;
        PhaseTimer rgd_timer(&out.timing.update_rgd);
        update_rgd(params.in_out_params_indoor, buckets, points_global, trajectory[points_global[0].index_pose].translation(), params.use_exact_ndt_bucket_statistics);
        rgd_timer.stop();

        PhaseTimer loop_timer(&out.timing.point_loop);
        int pose_node = -1;
        TaitBryanPose pose_s{};
        for (size_t index = begin; index < end; index++)
        {
            const Point3Di &intermediate_points_i = data.points[index];
            if (intermediate_points_i.point.norm() < 1.0)
                continue;

            Eigen::Vector3d point_global = trajectory[intermediate_points_i.index_pose] * intermediate_points_i.point;
            auto index_of_bucket = ndt_cell_index(point_global, b);

            auto bucket_it = buckets.find(index_of_bucket);
            // no bucket found
            if (bucket_it == buckets.end())
                continue;

            auto &this_bucket = bucket_it->second;

            Eigen::Matrix3d inverse_scratch;
            const Eigen::Matrix3d *inverse = ndt_cell_inverse(this_bucket, inverse_scratch);
            if (inverse == nullptr)
                continue;
            const Eigen::Matrix3d &infm = *inverse;

            const Eigen::Vector3d &p_s = intermediate_points_i.point;
            // the points come in runs of one node: convert its pose once per run
            if (intermediate_points_i.index_pose != pose_node)
            {
                pose_node = intermediate_points_i.index_pose;
                pose_s = pose_tait_bryan_from_affine_matrix(trajectory[pose_node]);
            }
            double delta_x;
            double delta_y;
            double delta_z;

            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;

            point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
                                                          pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                          p_s.x(), p_s.y(), p_s.z(), this_bucket.mean.x(), this_bucket.mean.y(), this_bucket.mean.z());

            point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                   pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                   p_s.x(), p_s.y(), p_s.z());

            // A = -J, P = infm, B = delta: A^T P A = J^T P J, A^T P B = -J^T P delta
            const Eigen::Matrix<double, 6, 3> JtP = jacobian.transpose() * infm;
            const Eigen::Matrix<double, 6, 6, Eigen::RowMajor> block = JtP * jacobian;
            const Eigen::Matrix<double, 6, 1> JtPd = JtP * Eigen::Vector3d(delta_x, delta_y, delta_z);
            const size_t node = static_cast<size_t>(intermediate_points_i.index_pose - out.first_node);
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(&out.blocks[node * 36]) += block;
            Eigen::Map<Eigen::Matrix<double, 6, 1>>(&out.rhs[node * 6]) -= JtPd;
        }
        loop_timer.stop();
    };

    PhaseTimer chunk_stage_timer(phase(&ConsistencyTiming::chunk_stage));
    std::for_each(std::execution::par, chunk_ids.begin(), chunk_ids.end(),
                  [&](size_t c)
                  {
                      gate.acquire();
                      compute_chunk(c);
                      gate.release();
                  });
    chunk_stage_timer.stop();

    // merge in chunk order
    for (size_t c = 0; c < n_chunks; c++)
    {
        const ChunkTerms &out = chunk_terms[c];
        for (int k = 0; k < out.n_nodes; k++)
        {
            const int row = (out.first_node + k) * 6;
            normal.add_diagonal_block6(row, Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(&out.blocks[static_cast<size_t>(k) * 36]));
            rhs.segment<6>(row) += Eigen::Map<const Eigen::Matrix<double, 6, 1>>(&out.rhs[static_cast<size_t>(k) * 6]);
        }
        if (timing)
        {
            timing->chunk_prepare.seconds += out.timing.chunk_prepare.seconds;
            timing->update_rgd.seconds += out.timing.update_rgd.seconds;
            timing->point_loop.seconds += out.timing.point_loop.seconds;
        }
    }
    log_info(logger) << "NDT observations FINISHED";

    PhaseTimer assemble_timer(phase(&ConsistencyTiming::assemble));
    Eigen::SparseMatrix<double> AtPA = normal.to_sparse();
    Eigen::SparseMatrix<double> AtPB = rhs.sparseView();
    assemble_timer.stop();

    PhaseTimer solve_timer(phase(&ConsistencyTiming::solve));
    data.solver.compute(AtPA); // the pattern is the same in every pass, so the symbolic analysis is kept
    Eigen::SparseMatrix<double> x = data.solver.solver().solve(AtPB);
    const bool solved = data.solver.solver().info() == Eigen::Success;
    const std::vector<double> h_x = solution_by_row(x, static_cast<int>(6 * trajectory.size()));
    solve_timer.stop();

    PhaseTimer apply_timer(phase(&ConsistencyTiming::apply));
    if (solved)
    {
        int counter = 0;

        for (size_t i = 0; i < trajectory.size(); i++)
        {
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
    }

    for (int i = 0; i < indexes.size(); i++)
        windows[indexes[i].first].poses[indexes[i].second] = trajectory[i];
    apply_timer.stop();
}
