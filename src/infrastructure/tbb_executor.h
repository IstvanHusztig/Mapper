#pragma once

// ParallelExecutor adapter for oneTBB (infrastructure).

#include "service_ports.h"

class TbbExecutor : public ParallelExecutor
{
public:
    void for_each_index(std::size_t n, const std::function<void(std::size_t)> &body) override;
};
