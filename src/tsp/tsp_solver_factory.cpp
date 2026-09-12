#include "tsp/tsp_solver_factory.hpp"
#include "tsp/held_karp_solver.hpp"
#include "tsp/heuristic_tsp_solver.hpp"

namespace tdr::tsp {

TSPSolverFactory::TSPSolverFactory(int max_exact_n, int heuristic_restarts, std::uint64_t heuristic_seed)
    : max_exact_n_(max_exact_n), heuristic_restarts_(heuristic_restarts), heuristic_seed_(heuristic_seed) {}

TSPResult TSPSolverFactory::solve(const DistanceMatrix& distances) const {
    const int n = static_cast<int>(distances.size());
    if (n <= max_exact_n_) {
        HeldKarpSolver exact(max_exact_n_);
        TSPResult r = exact.solve(distances);
        if (r.succeeded) return r;
        // Fall through to the heuristic only because the exact solver explicitly
        // refused (e.g. its own memory guard) -- never silently swallow a real error.
    }
    HeuristicTSPSolver heuristic(heuristic_restarts_, heuristic_seed_);
    return heuristic.solve(distances);
}

} // namespace tdr::tsp
