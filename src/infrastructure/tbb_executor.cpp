#include "tbb_executor.h"

#include <tbb/tbb.h>

void TbbExecutor::for_each_index(std::size_t n, const std::function<void(std::size_t)> &body)
{
    tbb::parallel_for(std::size_t(0), n, [&](std::size_t i) { body(i); });
}
