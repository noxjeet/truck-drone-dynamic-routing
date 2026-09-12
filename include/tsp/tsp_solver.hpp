#pragma once
#include "tsp_types.hpp"

namespace tdr::tsp {

// Abstract interface for a TSP solver over a complete distance matrix.
// Node 0 is always the fixed start/end (depot) node.
class ITSPSolver {
public:
    virtual ~ITSPSolver() = default;
    virtual TSPResult solve(const DistanceMatrix& distances) const = 0;
    virtual std::string name() const = 0;
};

} // namespace tdr::tsp
