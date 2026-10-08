#include "consistency.h"
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
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            h_x.push_back(it.value());
    solve_timer.stop();

    PhaseTimer apply_timer(phase(&ConsistencyTiming::apply));
    if (h_x.size() == 6 * trajectory.size())
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

void Consistency2(std::vector<Window> &windows, const OdometryConfig &params, PointStore &point_store, Logger &logger)
{
    std::vector<Eigen::Affine3d> trajectory;
    std::vector<std::pair<double, double>> timestamps;
    std::vector<Point3Di> all_points_local;

    std::vector<std::pair<int, int>> indexes;
    bool multithread = true;

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

    for (int i = 0; i < windows.size(); i++)
    {
        std::vector<Point3Di> intermediate_points;
        if (!point_store.load(PointSet::WindowIntermediate, i, intermediate_points))
            log_info(logger) << "problem with point store: intermediate points of window " << i;

        for (int j = 0; j < intermediate_points.size(); j++)
            all_points_local.push_back(intermediate_points[j]);
    }

    std::vector<Eigen::Triplet<double>> tripletListA;
    std::vector<Eigen::Triplet<double>> tripletListP;
    std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::Vector3d b(params.in_out_params_indoor.resolution_X, params.in_out_params_indoor.resolution_Y, params.in_out_params_indoor.resolution_Z);

    std::vector<std::mutex> my_mutex(1);

    std::vector<TaitBryanPose> poses;

    for (size_t i = 0; i < trajectory.size(); i++)
        poses.push_back(pose_tait_bryan_from_affine_matrix(trajectory[i]));

    double angle = 0.01 * DEG_TO_RAD;
    double wangle = 1.0 / (angle * angle);

    // smoothness
    for (size_t i = 1; i < poses.size() - 1; i++)
    {
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                                        poses[i - 1].px,
                                        poses[i - 1].py,
                                        poses[i - 1].pz,
                                        poses[i - 1].om,
                                        poses[i - 1].fi,
                                        poses[i - 1].ka,
                                        poses[i].px,
                                        poses[i].py,
                                        poses[i].pz,
                                        poses[i].om,
                                        poses[i].fi,
                                        poses[i].ka,
                                        poses[i + 1].px,
                                        poses[i + 1].py,
                                        poses[i + 1].pz,
                                        poses[i + 1].om,
                                        poses[i + 1].fi,
                                        poses[i + 1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                                                 poses[i - 1].px,
                                                 poses[i - 1].py,
                                                 poses[i - 1].pz,
                                                 poses[i - 1].om,
                                                 poses[i - 1].fi,
                                                 poses[i - 1].ka,
                                                 poses[i].px,
                                                 poses[i].py,
                                                 poses[i].pz,
                                                 poses[i].om,
                                                 poses[i].fi,
                                                 poses[i].ka,
                                                 poses[i + 1].px,
                                                 poses[i + 1].py,
                                                 poses[i + 1].pz,
                                                 poses[i + 1].om,
                                                 poses[i + 1].fi,
                                                 poses[i + 1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i - 1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i + 1) * 6;

        for (size_t row = 0; row < 6; row++)
        {
            tripletListA.emplace_back(ir + row, ic_1, -jacobian(row, 0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row, 1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row, 2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row, 3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row, 4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row, 5));

            tripletListA.emplace_back(ir + row, ic_2, -jacobian(row, 6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row, 7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row, 8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row, 9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row, 10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row, 11));

            tripletListA.emplace_back(ir + row, ic_3, -jacobian(row, 12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row, 13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row, 14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row, 15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row, 16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row, 17));
        }
        tripletListB.emplace_back(ir, 0, delta(0, 0));
        tripletListB.emplace_back(ir + 1, 0, delta(1, 0));
        tripletListB.emplace_back(ir + 2, 0, delta(2, 0));
        tripletListB.emplace_back(ir + 3, 0, delta(3, 0));
        tripletListB.emplace_back(ir + 4, 0, delta(4, 0));
        tripletListB.emplace_back(ir + 5, 0, delta(5, 0));

        tripletListP.emplace_back(ir, ir, 100000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 100000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 100000000);
        tripletListP.emplace_back(ir + 3, ir + 3, wangle);
        tripletListP.emplace_back(ir + 4, ir + 4, wangle);
        tripletListP.emplace_back(ir + 5, ir + 5, wangle);
    }

    int ir = tripletListB.size();
    tripletListA.emplace_back(ir, 0, 1);
    tripletListA.emplace_back(ir + 1, 1, 1);
    tripletListA.emplace_back(ir + 2, 2, 1);
    tripletListA.emplace_back(ir + 3, 3, 1);
    tripletListA.emplace_back(ir + 4, 4, 1);
    tripletListA.emplace_back(ir + 5, 5, 1);

    tripletListP.emplace_back(ir, ir, 1000000);
    tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
    tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
    tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
    tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
    tripletListP.emplace_back(ir + 5, ir + 5, 1000000);

    tripletListB.emplace_back(ir, 0, 0);
    tripletListB.emplace_back(ir + 1, 0, 0);
    tripletListB.emplace_back(ir + 2, 0, 0);
    tripletListB.emplace_back(ir + 3, 0, 0);
    tripletListB.emplace_back(ir + 4, 0, 0);
    tripletListB.emplace_back(ir + 5, 0, 0);

    Eigen::SparseMatrix<double> matA(tripletListB.size(), trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(trajectory.size() * 6, trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(trajectory.size() * 6, 1);

    {
        Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
        AtPA = (AtP)*matA;
        AtPB = (AtP)*matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    log_info(logger) << "reindex data START";

    std::vector<Point3Di> points_local;
    std::vector<Point3Di> points_global;

    points_local.reserve(all_points_local.size());
    points_global.reserve(all_points_local.size());

    for (int index = 0; index < all_points_local.size(); index++)
    {
        if (index < all_points_local.size())
        {
            auto lower = std::lower_bound(timestamps.begin(), timestamps.end(), all_points_local[index].timestamp,
                                          [](std::pair<double, double> lhs, double rhs) -> bool
                                          { return lhs.first < rhs; });
            int index_pose = std::distance(timestamps.begin(), lower) - 1;
            if (index_pose >= 0 && index_pose < trajectory.size())
            {
                auto pl = all_points_local[index];
                pl.index_pose = index_pose;

                points_local.emplace_back(pl);
                pl.point = trajectory[pl.index_pose] * pl.point;
                points_global.emplace_back(pl);
            }
        }
    }

    int counter_index = 0;
    int counter = 0;
    for (int i = 0; i < points_global.size(); i++)
    {
        points_global[i].index_point = counter_index;

        if (counter > 1000000)
        {
            counter = 0;
            counter_index++;
        }
        counter++;
    }

    log_info(logger) << "reindex data FINISHED";

    log_info(logger) << "build regular grid decomposition START";

    std::vector<std::pair<unsigned long long int, unsigned int>> index_pairs;
    for (int i = 0; i < points_global.size(); i++)
    {
        auto index_of_bucket = ndt_cell_index(points_global[i].point, b);
        index_pairs.emplace_back(index_of_bucket, i);
    }
    std::sort(index_pairs.begin(), index_pairs.end(),
              [](const std::pair<unsigned long long int, unsigned int> &a, const std::pair<unsigned long long int, unsigned int> &b)
              { return a.first < b.first; });

    NDTBucketMapType2 buckets;

    for (unsigned int i = 0; i < index_pairs.size(); i++)
    {
        unsigned long long int index_of_bucket = index_pairs[i].first;
        if (buckets.contains(index_of_bucket))
            buckets[index_of_bucket].index_end_exclusive = i;
        else
        {
            buckets[index_of_bucket].index_begin_inclusive = i;
            buckets[index_of_bucket].index_end_exclusive = i;
        }
    }

    for (auto &b : buckets)
    {
        for (int i = b.second.index_begin_inclusive; i < b.second.index_end_exclusive; i++)
        {
            long long unsigned int index_point_segment = points_global[index_pairs[i].second].index_point;

            if (b.second.buckets.contains(index_point_segment))
                b.second.buckets[index_point_segment].point_indexes.push_back(index_pairs[i].second);
            else
            {
                NdtBucketCoef bc;
                bc.point_indexes.push_back(index_pairs[i].second);
                b.second.buckets[index_point_segment] = bc;
            }
        }

        for (auto &bb : b.second.buckets)
        {
            if (bb.second.point_indexes.size() >= 5)
            {
                bb.second.valid = true;

                bb.second.mean = Eigen::Vector3d(0, 0, 0);
                for (const auto &p : bb.second.point_indexes)
                    bb.second.mean += points_global[p].point;

                bb.second.mean /= bb.second.point_indexes.size();

                bb.second.cov.setZero();
                for (const auto &p : bb.second.point_indexes)
                {
                    bb.second.cov(0, 0) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(0, 1) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(0, 2) += (bb.second.mean.x() - points_global[p].point.x()) * (bb.second.mean.z() - points_global[p].point.z());
                    bb.second.cov(1, 0) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(1, 1) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(1, 2) += (bb.second.mean.y() - points_global[p].point.y()) * (bb.second.mean.z() - points_global[p].point.z());
                    bb.second.cov(2, 0) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.x() - points_global[p].point.x());
                    bb.second.cov(2, 1) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.y() - points_global[p].point.y());
                    bb.second.cov(2, 2) += (bb.second.mean.z() - points_global[p].point.z()) * (bb.second.mean.z() - points_global[p].point.z());
                }
                bb.second.cov /= bb.second.point_indexes.size();

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(bb.second.cov, Eigen::ComputeEigenvectors);

                Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();
                Eigen::Vector3d nv = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
                nv.normalize();

                //  flip towards viewport
                const auto &m = trajectory[points_global[bb.second.point_indexes[0]].index_pose];

                if (nv.dot(m.translation() - bb.second.mean) < 0.0)
                    nv *= -1.0;

                bb.second.normal_vector = nv;

                bb.second.point_indexes.clear();
            }
            else
                bb.second.valid = false;
        }
    }
    log_info(logger) << "build regular grid decomposition FINISHED";

    log_info(logger) << "ndt observations start START";

    counter = 0;

    for (int i = 0; i < points_global.size(); i++)
    {
        if (i % 10000 == 0)
            log_info(logger) << "computing " << i << " of " << points_global.size();

        if (points_local[i].point.norm() < params.consistency_min_distance)
            continue;

        auto index_of_bucket = ndt_cell_index(points_global[i].point, b);
        auto bucket_it = buckets.find(index_of_bucket);
        // no bucket found
        if (bucket_it == buckets.end())
            continue;

        for (auto &bb : bucket_it->second.buckets)
        {
            if (bb.second.valid)
            {
                auto &this_bucket = bb.second;

                const Eigen::Matrix3d &infm = this_bucket.cov.inverse();
                const double threshold = 100000.0;

                if ((infm.array() > threshold).any())
                    continue;

                if ((infm.array() < -threshold).any())
                    continue;

                // check nv
                Eigen::Vector3d &nv = this_bucket.normal_vector;
                Eigen::Vector3d viewport = trajectory[points_global[i].index_pose].translation();
                if (nv.dot(viewport - this_bucket.mean) < 0)
                    continue;

                const Eigen::Affine3d &m_pose = trajectory[points_global[i].index_pose];
                const Eigen::Vector3d &p_s = points_local[i].point;
                const TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
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

                int c = points_global[i].index_pose * 6;

                std::mutex &m = my_mutex[0];
                std::unique_lock lck(m);
                int ir = tripletListB.size();

                for (int row = 0; row < 3; row++)
                    for (int col = 0; col < 6; col++)
                        if (jacobian(row, col) != 0.0)
                            tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));

                tripletListB.emplace_back(ir, 0, delta_x);
                tripletListB.emplace_back(ir + 1, 0, delta_y);
                tripletListB.emplace_back(ir + 2, 0, delta_z);

                tripletListP.emplace_back(ir, ir, infm(0, 0));
                tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
                tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
                tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
                tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
                tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
                tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
                tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
                tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));

                counter++;
                if (counter > 10000000)
                {
                    counter = 0;
                    Eigen::SparseMatrix<double> matAndt(tripletListB.size(), trajectory.size() * 6);
                    Eigen::SparseMatrix<double> matPndt(tripletListB.size(), tripletListB.size());
                    Eigen::SparseMatrix<double> matBndt(tripletListB.size(), 1);

                    matAndt.setFromTriplets(tripletListA.begin(), tripletListA.end());
                    matPndt.setFromTriplets(tripletListP.begin(), tripletListP.end());
                    matBndt.setFromTriplets(tripletListB.begin(), tripletListB.end());

                    Eigen::SparseMatrix<double> AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
                    Eigen::SparseMatrix<double> AtPBndt(trajectory.size() * 6, 1);

                    {
                        Eigen::SparseMatrix<double> AtPndt = matAndt.transpose() * matPndt;
                        AtPAndt = (AtPndt)*matAndt;
                        AtPBndt = (AtPndt)*matBndt;
                    }

                    tripletListA.clear();
                    tripletListP.clear();
                    tripletListB.clear();

                    AtPA += AtPAndt;
                    AtPB += AtPBndt;
                }
            }
        }
    }

    if (counter > 0)
    {
        Eigen::SparseMatrix<double> matAndt(tripletListB.size(), trajectory.size() * 6);
        Eigen::SparseMatrix<double> matPndt(tripletListB.size(), tripletListB.size());
        Eigen::SparseMatrix<double> matBndt(tripletListB.size(), 1);

        matAndt.setFromTriplets(tripletListA.begin(), tripletListA.end());
        matPndt.setFromTriplets(tripletListP.begin(), tripletListP.end());
        matBndt.setFromTriplets(tripletListB.begin(), tripletListB.end());

        Eigen::SparseMatrix<double> AtPAndt(trajectory.size() * 6, trajectory.size() * 6);
        Eigen::SparseMatrix<double> AtPBndt(trajectory.size() * 6, 1);

        {
            Eigen::SparseMatrix<double> AtPndt = matAndt.transpose() * matPndt;
            AtPAndt = (AtPndt)*matAndt;
            AtPBndt = (AtPndt)*matBndt;
        }

        tripletListA.clear();
        tripletListP.clear();
        tripletListB.clear();
        AtPA += AtPAndt;
        AtPB += AtPBndt;
    }

    log_info(logger) << "ndt observations start FINISHED";

    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);
    std::vector<double> h_x;
    for (int k = 0; k < x.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(x, k); it; ++it)
            h_x.push_back(it.value());

    if (h_x.size() == 6 * trajectory.size())
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
}
