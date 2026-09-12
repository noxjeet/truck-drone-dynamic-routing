#pragma once
#include "tsp_solver.hpp"
#include <cstdint>

namespace tdr::tsp {

// Non-exact substitute TSP solver, used only when the instance is too
// large for HeldKarpSolver. See DECISIONS.md for the reproduction-fidelity
// implications of using this instead of CPLEX. Always reports exact = false.
//
// Construction: nearest-neighbor from multiple restarts. Improvement:
// 2-opt then Or-opt local search on each construction. Returns the best
// result found across restarts.
class HeuristicTSPSolver : public ITSPSolver {
public:
    explicit HeuristicTSPSolver(int num_restarts = 8, std::uint64_t seed = 42);

    TSPResult solve(const DistanceMatrix& distances) const override;
    std::string name() const override { return "NearestNeighbor2OptOrOpt"; }

private:
    int num_restarts_;
    std::uint64_t seed_;
};

} // namespace tdr::tsp
