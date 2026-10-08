#pragma once

#include <chrono>

// Wall-clock totals for the diagnostic setting log_stage_timing. The timers only read the clock and add to these
// totals, which never enter a computation, so results are the same with timing on or off.

// time spent in one phase, summed over its calls
struct PhaseTime
{
    double seconds = 0.0;
    long long calls = 0;
};

// the phases of one optimize_lidar_odometry call, summed over all calls of a run
struct ScanToMapTiming
{
    PhaseTime setup;            // the dense normal matrix: allocate and zero
    PhaseTime ndt_indoor;       // the NDT point loop over the indoor grid
    PhaseTime ndt_outdoor;      // the NDT point loop over the outdoor grid (use_hierarchical_rgd)
    PhaseTime motion_terms;     // motion model, gyro bias, velocity prior and acceleration terms
    PhaseTime assembly;         // fixed-origin triplets and pose_band_to_sparse
    PhaseTime covariance_solve; // marginal covariance of the gyro bias and acceleration offset
    PhaseTime solve;            // the LM damping and the Cholesky solve
    PhaseTime update;           // applying the solution to the poses, the bias and the offset
};

// the phases of the consistency pass (Consistency), summed over all its passes
struct ConsistencyTiming
{
    PhaseTime pass;            // one whole pass
    PhaseTime load_points;     // reading every window's points and finding their nodes (first pass only)
    PhaseTime smoothness;      // the smoothness and fix-origin terms
    PhaseTime chunk_stage;     // all chunks, in parallel (wall-clock)
    PhaseTime chunk_prepare;   // per chunk: the global points (thread seconds, summed over the chunks)
    PhaseTime update_rgd;      // per chunk: the NDT map (thread seconds)
    PhaseTime point_loop;      // per chunk: the point terms (thread seconds)
    PhaseTime assemble;        // the sparse normal equations from the banded matrix
    PhaseTime solve;           // the Cholesky solve
    PhaseTime apply;           // applying the solution to the trajectory
};

// adds the time from construction to stop() (or destruction) to *target as one call; does nothing for a null target
class PhaseTimer
{
public:
    explicit PhaseTimer(PhaseTime *target) : target_(target)
    {
        if (target_)
            start_ = std::chrono::steady_clock::now();
    }
    ~PhaseTimer() { stop(); }
    PhaseTimer(const PhaseTimer &) = delete;
    PhaseTimer &operator=(const PhaseTimer &) = delete;

    void stop()
    {
        if (!target_)
            return;
        target_->seconds += std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
        target_->calls++;
        target_ = nullptr;
    }

private:
    PhaseTime *target_;
    std::chrono::steady_clock::time_point start_;
};
