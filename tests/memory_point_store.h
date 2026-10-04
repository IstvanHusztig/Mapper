#pragma once

// PointStore kept in memory, for tests that run the pipeline's algorithms on synthetic windows

#include "input_ports.h"

#include <map>
#include <utility>

class MemoryPointStore : public PointStore
{
public:
    bool save(PointSet set, std::size_t index, const std::vector<Point3Di> &points) override
    {
        sets_[{set, index}] = points;
        return true;
    }
    bool load(PointSet set, std::size_t index, std::vector<Point3Di> &points) override
    {
        const auto it = sets_.find({set, index});
        if (it == sets_.end())
            return false;
        points = it->second;
        return true;
    }
    void clear() override { sets_.clear(); }
    std::string location() const override { return "memory"; }

private:
    std::map<std::pair<PointSet, std::size_t>, std::vector<Point3Di>> sets_;
};
