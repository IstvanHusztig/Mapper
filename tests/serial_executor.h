#pragma once

// A ParallelExecutor for tests: runs every index in order on the calling thread.

#include "service_ports.h"

class SerialExecutor : public ParallelExecutor
{
public:
    void for_each_index(std::size_t n, const std::function<void(std::size_t)> &body) override
    {
        for (std::size_t i = 0; i < n; i++)
            body(i);
    }
};
