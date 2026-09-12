#pragma once
#include "tsp_solver.hpp"
#include <cstdint>

namespace tdr::tsp {

// Centralizes the exact-vs-heuristic TSP solver decision (see
// DECISIONS.md) so it is made in exactly one place, and so the outcome
// (which solver actually ran, and whether the result is exact) is always
// recorded on the returned TSPResult rather than assumed by a caller.
//
//   n <= max_exact_n  -> HeldKarpSolver (exact)
//   n >  max_exact_n  -> HeuristicTSPSolver (documented substitute)
class TSPSolverFactory {
public:
    explicit TSPSolverFactory(int max_exact_n = 21,
                               int heuristic_restarts = 8,
                               std::uint64_t heuristic_seed = 42);

    TSPResult solve(const DistanceMatrix& distances) const;

private:
    int max_exact_n_;
    int heuristic_restarts_;
    std::uint64_t heuristic_seed_;
};

} // namespace tdr::tsp
