#pragma once
#include "tsp_solver.hpp"
#include <cstddef>

namespace tdr::tsp {

// Exact TSP solver via the Held-Karp dynamic program. See DECISIONS.md
// for why this exists (CPLEX availability on this machine is
// unconfirmed) and what it does and does not guarantee.
//
// Refuses to run (succeeded = false) rather than silently attempting an
// impractical solve, if n exceeds max_exact_n or the estimated memory
// required exceeds safety_memory_limit_bytes.
class HeldKarpSolver : public ITSPSolver {
public:
    static constexpr int kDefaultMaxExactN = 21;
    static constexpr std::size_t kDefaultSafetyMemoryLimitBytes =
        4ULL * 1024 * 1024 * 1024; // 4 GiB

    explicit HeldKarpSolver(int max_exact_n = kDefaultMaxExactN,
                             std::size_t safety_memory_limit_bytes = kDefaultSafetyMemoryLimitBytes);

    TSPResult solve(const DistanceMatrix& distances) const override;
    std::string name() const override { return "HeldKarpExact"; }

private:
    int max_exact_n_;
    std::size_t safety_memory_limit_bytes_;
};

} // namespace tdr::tsp
