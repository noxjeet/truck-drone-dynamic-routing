#include "tsp/held_karp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <string>

namespace tdr::tsp {

HeldKarpSolver::HeldKarpSolver(int max_exact_n, std::size_t safety_memory_limit_bytes)
    : max_exact_n_(max_exact_n), safety_memory_limit_bytes_(safety_memory_limit_bytes) {}

TSPResult HeldKarpSolver::solve(const DistanceMatrix& distances) const {
    TSPResult result;
    result.method_name = name();

    const int n = static_cast<int>(distances.size());

    if (n <= 0) {
        result.succeeded = false;
        result.note = "Empty distance matrix.";
        return result;
    }
    if (n == 1) {
        result.tour = {0};
        result.cost = 0.0;
        result.exact = true;
        result.succeeded = true;
        return result;
    }

    if (n > max_exact_n_) {
        result.succeeded = false;
        result.exact = false;
        result.note = "Instance size (" + std::to_string(n) +
            " nodes) exceeds configured max_exact_n (" + std::to_string(max_exact_n_) +
            "); refusing to run rather than silently producing an impractical solve. "
            "Use a heuristic solver for this size, or raise max_exact_n if enough "
            "memory/time is available.";
        return result;
    }

    const std::size_t full_mask_count = std::size_t(1) << n;
    const std::size_t estimated_bytes =
        full_mask_count * static_cast<std::size_t>(n) * (sizeof(double) + sizeof(int));
    if (estimated_bytes > safety_memory_limit_bytes_) {
        result.succeeded = false;
        result.exact = false;
        result.note = "Estimated memory for exact Held-Karp on n=" + std::to_string(n) +
            " (~" + std::to_string(estimated_bytes / (1024 * 1024)) +
            " MiB) exceeds the configured safety limit. Refusing to run.";
        return result;
    }

    const auto t0 = std::chrono::steady_clock::now();

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> dp(full_mask_count, std::vector<double>(n, INF));
    std::vector<std::vector<int>> parent(full_mask_count, std::vector<int>(n, -1));

    dp[1][0] = 0.0; // mask containing only node 0 (the depot), currently at node 0

    for (std::size_t mask = 1; mask < full_mask_count; ++mask) {
        if (!(mask & 1u)) continue; // the depot must always be in the visited set
        for (int j = 0; j < n; ++j) {
            if (!(mask & (std::size_t(1) << j))) continue;
            const double dj = dp[mask][j];
            if (dj == INF) continue;
            for (int k = 0; k < n; ++k) {
                if (mask & (std::size_t(1) << k)) continue; // already visited
                const std::size_t next_mask = mask | (std::size_t(1) << k);
                const double cand = dj + distances[j][k];
                if (cand < dp[next_mask][k]) {
                    dp[next_mask][k] = cand;
                    parent[next_mask][k] = j;
                }
            }
        }
    }

    const std::size_t full_mask = full_mask_count - 1;
    double best = INF;
    int best_j = -1;
    for (int j = 1; j < n; ++j) {
        if (dp[full_mask][j] == INF) continue;
        const double cand = dp[full_mask][j] + distances[j][0];
        if (cand < best) {
            best = cand;
            best_j = j;
        }
    }

    if (best_j == -1) {
        result.succeeded = false;
        result.note = "No feasible closed tour found (disconnected distance matrix?).";
        return result;
    }

    std::vector<int> tour;
    std::size_t mask = full_mask;
    int j = best_j;
    while (j != -1) {
        tour.push_back(j);
        const int pj = parent[mask][j];
        mask ^= (std::size_t(1) << j);
        j = pj;
    }
    std::reverse(tour.begin(), tour.end());

    const auto t1 = std::chrono::steady_clock::now();

    result.tour = std::move(tour);
    result.cost = best;
    result.exact = true;
    result.succeeded = true;
    result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

} // namespace tdr::tsp
