#pragma once
#include <vector>
#include <string>

namespace tdr::tsp {

using DistanceMatrix = std::vector<std::vector<double>>;

// Result of a single TSP solve. Node 0 is always the fixed start/end
// (depot) node, matching Paper 1's truck-only route convention.
struct TSPResult {
    std::vector<int> tour;   // visiting order; implicit return to tour.front() closes the cycle
    double cost = 0.0;       // total closed-tour length
    bool exact = false;      // true iff this is a certified global optimum
    bool succeeded = false;  // false if no result could be produced at all
    std::string method_name;
    double solve_time_ms = 0.0;
    std::string note;        // human-readable caveat (e.g. "heuristic substitute, not exact")
};

} // namespace tdr::tsp
