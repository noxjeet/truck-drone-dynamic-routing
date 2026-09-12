#include "tsp/heuristic_tsp_solver.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <random>

namespace tdr::tsp {

namespace {

double tour_cost(const DistanceMatrix& d, const std::vector<int>& tour) {
    double c = 0.0;
    const int n = static_cast<int>(tour.size());
    for (int i = 0; i < n; ++i) {
        c += d[tour[i]][tour[(i + 1) % n]];
    }
    return c;
}

std::vector<int> nearest_neighbor_tour(const DistanceMatrix& d, int start) {
    const int n = static_cast<int>(d.size());
    std::vector<bool> visited(n, false);
    std::vector<int> tour;
    tour.reserve(n);
    int current = start;
    visited[current] = true;
    tour.push_back(current);
    for (int step = 1; step < n; ++step) {
        int best = -1;
        double best_dist = std::numeric_limits<double>::infinity();
        for (int cand = 0; cand < n; ++cand) {
            if (visited[cand]) continue;
            if (d[current][cand] < best_dist) {
                best_dist = d[current][cand];
                best = cand;
            }
        }
        visited[best] = true;
        tour.push_back(best);
        current = best;
    }
    return tour;
}

void rotate_to_start_at_zero(std::vector<int>& tour) {
    auto it = std::find(tour.begin(), tour.end(), 0);
    std::rotate(tour.begin(), it, tour.end());
}

bool two_opt_pass(const DistanceMatrix& d, std::vector<int>& tour) {
    const int n = static_cast<int>(tour.size());
    bool improved_any = false;
    bool improved = true;
    while (improved) {
        improved = false;
        for (int i = 0; i < n - 1; ++i) {
            for (int k = i + 1; k < n; ++k) {
                const int a = tour[i];
                const int b = tour[(i + 1) % n];
                const int c = tour[k];
                const int e = tour[(k + 1) % n];
                if (a == c || b == e) continue;
                const double before = d[a][b] + d[c][e];
                const double after = d[a][c] + d[b][e];
                if (after + 1e-9 < before) {
                    std::reverse(tour.begin() + i + 1, tour.begin() + k + 1);
                    improved = true;
                    improved_any = true;
                }
            }
        }
    }
    return improved_any;
}

bool or_opt_pass(const DistanceMatrix& d, std::vector<int>& tour) {
    // Relocate segments of length 1..3 to a better position elsewhere in the tour.
    const int n = static_cast<int>(tour.size());
    bool improved_any = false;
    for (int seg_len = 1; seg_len <= 3 && seg_len < n - 1; ++seg_len) {
        bool improved = true;
        while (improved) {
            improved = false;
            for (int i = 0; i < n; ++i) {
                std::vector<int> seg;
                for (int s = 0; s < seg_len; ++s) seg.push_back(tour[(i + s) % n]);
                const int prev = tour[(i - 1 + n) % n];
                const int next = tour[(i + seg_len) % n];
                if (prev == seg.back() || next == seg.front()) continue;
                const double removed_cost =
                    d[prev][seg.front()] + d[seg.back()][next] - d[prev][next];
                bool moved = false;
                for (int j = 0; j < n && !moved; ++j) {
                    bool inside_segment = false;
                    for (int s = -1; s <= seg_len; ++s) {
                        if (((i + s + n) % n) == j) { inside_segment = true; break; }
                    }
                    if (inside_segment) continue;
                    const int u = tour[j];
                    const int v = tour[(j + 1) % n];
                    const double added_cost =
                        d[u][seg.front()] + d[seg.back()][v] - d[u][v];
                    if (added_cost + 1e-9 < removed_cost) {
                        std::vector<int> new_tour;
                        new_tour.reserve(n);
                        std::vector<bool> in_seg(n, false);
                        for (int c : seg) in_seg[c] = true;
                        for (int t = 0; t < n; ++t) {
                            if (in_seg[tour[t]]) continue;
                            new_tour.push_back(tour[t]);
                            if (tour[t] == u) {
                                for (int c : seg) new_tour.push_back(c);
                            }
                        }
                        tour = std::move(new_tour);
                        improved = true;
                        improved_any = true;
                        moved = true;
                    }
                }
                if (moved) break;
            }
        }
    }
    return improved_any;
}

} // namespace

HeuristicTSPSolver::HeuristicTSPSolver(int num_restarts, std::uint64_t seed)
    : num_restarts_(num_restarts), seed_(seed) {}

TSPResult HeuristicTSPSolver::solve(const DistanceMatrix& distances) const {
    TSPResult result;
    result.method_name = name();
    result.exact = false;
    result.note = "Heuristic substitute for an exact solve (nearest-neighbor "
        "construction + 2-opt + Or-opt local search, multi-start). Not a certified "
        "optimum -- see DECISIONS.md for the reproduction-fidelity implications.";

    const int n = static_cast<int>(distances.size());
    if (n <= 0) {
        result.succeeded = false;
        result.note = "Empty distance matrix.";
        return result;
    }
    if (n == 1) {
        result.tour = {0};
        result.cost = 0.0;
        result.succeeded = true;
        return result;
    }

    const auto t0 = std::chrono::steady_clock::now();

    std::mt19937_64 rng(seed_);
    std::uniform_int_distribution<int> pick_start(0, n - 1);

    std::vector<int> best_tour;
    double best_cost = std::numeric_limits<double>::infinity();

    const int restarts = std::max(1, num_restarts_);
    for (int r = 0; r < restarts; ++r) {
        const int start = (r == 0) ? 0 : pick_start(rng);
        std::vector<int> tour = nearest_neighbor_tour(distances, start);
        rotate_to_start_at_zero(tour);

        bool improved = true;
        while (improved) {
            const bool a = two_opt_pass(distances, tour);
            const bool b = or_opt_pass(distances, tour);
            improved = a || b;
        }
        rotate_to_start_at_zero(tour);

        const double cost = tour_cost(distances, tour);
        if (cost < best_cost) {
            best_cost = cost;
            best_tour = tour;
        }
    }

    const auto t1 = std::chrono::steady_clock::now();

    result.tour = std::move(best_tour);
    result.cost = best_cost;
    result.succeeded = true;
    result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

} // namespace tdr::tsp
