#include "tsp/held_karp_solver.hpp"
#include "tsp/heuristic_tsp_solver.hpp"
#include "tsp/tsp_solver_factory.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace tdr::tsp;

namespace {

int g_failures = 0;

void check(bool condition, const std::string& description) {
    if (condition) {
        std::cout << "[PASS] " << description << "\n";
    } else {
        std::cout << "[FAIL] " << description << "\n";
        ++g_failures;
    }
}

void check_near(double a, double b, double tol, const std::string& description) {
    check(std::fabs(a - b) <= tol,
          description + " (got " + std::to_string(a) + ", expected " + std::to_string(b) + ")");
}

DistanceMatrix random_symmetric_matrix(int n, std::mt19937_64& rng) {
    std::uniform_real_distribution<double> dist(1.0, 100.0);
    DistanceMatrix d(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            const double v = dist(rng);
            d[i][j] = v;
            d[j][i] = v;
        }
    }
    return d;
}

double brute_force_optimal(const DistanceMatrix& d) {
    const int n = static_cast<int>(d.size());
    std::vector<int> perm(n - 1);
    std::iota(perm.begin(), perm.end(), 1); // node 0 fixed as start/end
    double best = std::numeric_limits<double>::infinity();
    do {
        double cost = d[0][perm.front()];
        for (std::size_t i = 0; i + 1 < perm.size(); ++i) {
            cost += d[perm[i]][perm[i + 1]];
        }
        cost += d[perm.back()][0];
        best = std::min(best, cost);
    } while (std::next_permutation(perm.begin(), perm.end()));
    return best;
}

bool is_valid_permutation_tour(const std::vector<int>& tour, int n) {
    if (static_cast<int>(tour.size()) != n) return false;
    std::vector<bool> seen(n, false);
    for (int v : tour) {
        if (v < 0 || v >= n || seen[v]) return false;
        seen[v] = true;
    }
    return true;
}

double tour_cost(const DistanceMatrix& d, const std::vector<int>& tour) {
    double c = 0.0;
    const int n = static_cast<int>(tour.size());
    for (int i = 0; i < n; ++i) c += d[tour[i]][tour[(i + 1) % n]];
    return c;
}

} // namespace

int main() {
    std::mt19937_64 rng(1234);

    // 1. Held-Karp vs. independent brute-force enumeration on small instances.
    for (int n : {4, 5, 6, 7, 8}) {
        DistanceMatrix d = random_symmetric_matrix(n, rng);
        HeldKarpSolver solver;
        TSPResult r = solver.solve(d);
        check(r.succeeded && r.exact, "Held-Karp succeeds and is exact for n=" + std::to_string(n));
        const double bf = brute_force_optimal(d);
        check_near(r.cost, bf, 1e-6, "Held-Karp matches brute force for n=" + std::to_string(n));
        check(is_valid_permutation_tour(r.tour, n), "Held-Karp tour is a valid permutation for n=" + std::to_string(n));
        check_near(tour_cost(d, r.tour), r.cost, 1e-6, "Held-Karp reported cost matches recomputed tour cost for n=" + std::to_string(n));
    }

    // 2. Known geometry: unit square, optimal perimeter tour cost = 4.
    {
        std::vector<std::pair<double, double>> pts = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
        DistanceMatrix d(4, std::vector<double>(4, 0.0));
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                const double dx = pts[i].first - pts[j].first;
                const double dy = pts[i].second - pts[j].second;
                d[i][j] = std::sqrt(dx * dx + dy * dy);
            }
        }
        HeldKarpSolver solver;
        TSPResult r = solver.solve(d);
        check_near(r.cost, 4.0, 1e-9, "Unit-square optimal tour cost is exactly 4");
    }

    // 3. Threshold refusal (never silently attempts an oversized exact solve).
    {
        HeldKarpSolver solver(/*max_exact_n=*/5);
        DistanceMatrix d = random_symmetric_matrix(6, rng);
        TSPResult r = solver.solve(d);
        check(!r.succeeded && !r.exact, "Held-Karp refuses when n exceeds max_exact_n");
    }

    // 4. Memory-safety guard.
    {
        HeldKarpSolver solver(/*max_exact_n=*/20, /*safety_memory_limit_bytes=*/1024); // absurdly small
        DistanceMatrix d = random_symmetric_matrix(10, rng);
        TSPResult r = solver.solve(d);
        check(!r.succeeded, "Held-Karp refuses gracefully when the memory safety limit would be exceeded");
    }

    // 5. Heuristic solver produces a valid, correctly-costed, non-exact tour.
    {
        DistanceMatrix d = random_symmetric_matrix(30, rng);
        HeuristicTSPSolver heuristic(/*num_restarts=*/5, /*seed=*/7);
        TSPResult r = heuristic.solve(d);
        check(r.succeeded && !r.exact, "Heuristic solver succeeds and correctly reports exact=false");
        check(is_valid_permutation_tour(r.tour, 30), "Heuristic tour is a valid permutation for n=30");
        check_near(tour_cost(d, r.tour), r.cost, 1e-6, "Heuristic reported cost matches recomputed tour cost");
    }

    // 6. Local search should never leave the tour worse than a single nearest-neighbor pass.
    {
        DistanceMatrix d = random_symmetric_matrix(25, rng);
        HeuristicTSPSolver heuristic(/*num_restarts=*/1, /*seed=*/99);
        TSPResult r = heuristic.solve(d);

        std::vector<bool> visited(25, false);
        std::vector<int> nn_tour;
        int current = 0;
        visited[0] = true;
        nn_tour.push_back(0);
        for (int step = 1; step < 25; ++step) {
            int best = -1;
            double best_d = std::numeric_limits<double>::infinity();
            for (int c = 0; c < 25; ++c) {
                if (!visited[c] && d[current][c] < best_d) { best_d = d[current][c]; best = c; }
            }
            visited[best] = true;
            nn_tour.push_back(best);
            current = best;
        }
        const double nn_cost = tour_cost(d, nn_tour);
        check(r.cost <= nn_cost + 1e-6, "Heuristic (with local search) is never worse than pure nearest-neighbor");
    }

    // 7. Factory routes to the correct solver based on instance size.
    {
        TSPSolverFactory factory(/*max_exact_n=*/10);
        DistanceMatrix small = random_symmetric_matrix(8, rng);
        DistanceMatrix large = random_symmetric_matrix(15, rng);
        TSPResult r_small = factory.solve(small);
        TSPResult r_large = factory.solve(large);
        check(r_small.exact && r_small.method_name == "HeldKarpExact",
              "Factory picks exact Held-Karp for n <= max_exact_n");
        check(!r_large.exact && r_large.method_name == "NearestNeighbor2OptOrOpt",
              "Factory picks heuristic substitute for n > max_exact_n");
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
