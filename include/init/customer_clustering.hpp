#pragma once

// Phase F step (b): K-means customer clustering, as described in
// paper1-understanding.md Sec 5 / paper1-implementation-plan.md Sec 10-11.
// See DECISIONS.md ("Initial-solution construction (Phase F)") for the
// full [PAPER]/[INFERENCE]/[ASSUMPTION] breakdown.
//
// [PAPER] Paper 1 names K-means as the clustering algorithm used in
// initialization step (b), and gives an explicit cluster-count formula:
//   k = ceil(|C| / (|U| + 2))
// where |C| is the number of customers and |U| is the number of drones.
//
// [ASSUMPTION] Everything about *how* K-means is actually run -- initial
// centroid seeding, the convergence criterion, the iteration cap, and
// tie-breaking -- is not specified by the paper at all
// (paper1-implementation-plan.md Sec 10: "[ASSUMPTION] seeding/
// convergence details"). This file makes one simple, fully deterministic,
// no-randomness choice for each and documents it inline below, rather
// than inventing a more sophisticated variant (k-means++, multi-restart,
// etc.) that the paper gives no basis for.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <vector>

#include "model/node.hpp"

namespace tdr::init {

// [PAPER] k = ceil(|C| / (|U| + 2)). Clamped so a caller passing a
// negative drone count (which should never happen for a valid instance)
// cannot produce a divide-by-a-non-positive-number result; num_drones is
// otherwise used exactly as given.
inline int compute_cluster_count(std::size_t num_customers, int num_drones) {
    if (num_customers == 0) {
        return 0;
    }
    const int safe_num_drones = std::max(0, num_drones);
    const std::size_t divisor = static_cast<std::size_t>(safe_num_drones) + 2; // always >= 2
    return static_cast<int>((num_customers + divisor - 1) / divisor);
}

struct KMeansPolicy {
    // [ASSUMPTION] Neither value is specified by the paper. 100 iterations
    // is far more than these instance sizes (<= 50 customers) ever need
    // in practice; it exists only as a hard safety cap against an
    // unexpected non-terminating oscillation.
    int max_iterations = 100;
    // [ASSUMPTION] Convergence is declared once no customer changes
    // cluster assignment between iterations, OR every centroid moves less
    // than this epsilon in one iteration (whichever happens first).
    double convergence_epsilon = 1e-9;
};

struct KMeansResult {
    int cluster_count = 0; // the EFFECTIVE cluster count actually used (see run_kmeans)
    std::unordered_map<tdr::model::NodeId, int> assignment; // customer id -> cluster index in [0, cluster_count)
    std::vector<tdr::model::Point2D> centroids;              // size == cluster_count
    int iterations_run = 0;
    bool converged = false;
};

// Deterministic Lloyd's-algorithm K-means over customer coordinates.
//
// Seeding [ASSUMPTION]: customers are sorted by NodeId ascending, then k
// evenly index-spaced customers from that sorted list are taken as the
// initial centroids -- index floor(m * n / k) for m = 0..k-1. This is a
// pure function of the input (no RNG at all), and spreads the initial
// centroids across the customer set rather than clustering them at the
// low end of the id range (as picking "the first k customers" would).
//
// Iteration [ASSUMPTION]: standard Lloyd's algorithm -- assign each point
// to its nearest centroid by squared Euclidean distance, then recompute
// each centroid as the mean of its assigned points -- repeated until no
// point's assignment changes, or the largest centroid movement in one
// iteration falls below KMeansPolicy::convergence_epsilon, or
// max_iterations is reached (in which case converged stays false, but
// the last assignment computed is still returned rather than discarded).
//
// Tie-breaking [ASSUMPTION]: a customer exactly equidistant from two or
// more centroids is assigned to the LOWEST-indexed centroid.
//
// Empty clusters [ASSUMPTION]: if a centroid ends up with no points
// assigned, it is left in place (not reseeded) for the next iteration.
// An empty cluster is a legitimate outcome here -- Phase F's drone-trip
// construction step (initial_solution_builder.hpp) simply has nothing to
// do for an empty cluster and skips it.
//
// k is clamped to at most the number of customers (a cluster with no
// possible member is meaningless); this never actually triggers for the
// paper's own k = ceil(|C|/(|U|+2)) formula, since that can never exceed
// |C|, but is kept as a defensive clamp for direct callers of this
// function with an arbitrary k (e.g. tests).
inline KMeansResult run_kmeans(
    const std::unordered_map<tdr::model::NodeId, tdr::model::Point2D>& customer_coordinates,
    int k,
    const KMeansPolicy& policy = KMeansPolicy{}) {

    using tdr::model::NodeId;
    using tdr::model::Point2D;

    KMeansResult result;
    if (k <= 0 || customer_coordinates.empty()) {
        result.cluster_count = 0;
        return result;
    }

    std::vector<NodeId> ids;
    ids.reserve(customer_coordinates.size());
    for (const auto& [id, pt] : customer_coordinates) {
        (void)pt;
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());

    const std::size_t n = ids.size();
    const int effective_k = static_cast<int>(std::min<std::size_t>(static_cast<std::size_t>(k), n));
    result.cluster_count = effective_k;

    result.centroids.resize(static_cast<std::size_t>(effective_k));
    for (int m = 0; m < effective_k; ++m) {
        const std::size_t idx = (n * static_cast<std::size_t>(m)) / static_cast<std::size_t>(effective_k);
        result.centroids[static_cast<std::size_t>(m)] = customer_coordinates.at(ids[idx]);
    }

    std::unordered_map<NodeId, int> assignment;
    for (NodeId id : ids) assignment[id] = -1;

    for (int iter = 0; iter < policy.max_iterations; ++iter) {
        ++result.iterations_run;
        bool any_change = false;

        for (NodeId id : ids) {
            const Point2D& p = customer_coordinates.at(id);
            int best = 0;
            double best_dist_sq = std::numeric_limits<double>::infinity();
            for (int m = 0; m < effective_k; ++m) {
                const Point2D& c = result.centroids[static_cast<std::size_t>(m)];
                const double dx = p.x - c.x;
                const double dy = p.y - c.y;
                const double dist_sq = dx * dx + dy * dy;
                if (dist_sq < best_dist_sq) { // strict '<': ties keep the lower index already held in `best`
                    best_dist_sq = dist_sq;
                    best = m;
                }
            }
            if (assignment[id] != best) {
                assignment[id] = best;
                any_change = true;
            }
        }

        std::vector<Point2D> sums(static_cast<std::size_t>(effective_k), Point2D{0.0, 0.0});
        std::vector<int> counts(static_cast<std::size_t>(effective_k), 0);
        for (NodeId id : ids) {
            const int c = assignment[id];
            const Point2D& p = customer_coordinates.at(id);
            sums[static_cast<std::size_t>(c)].x += p.x;
            sums[static_cast<std::size_t>(c)].y += p.y;
            counts[static_cast<std::size_t>(c)] += 1;
        }

        double max_shift = 0.0;
        for (int m = 0; m < effective_k; ++m) {
            const int count = counts[static_cast<std::size_t>(m)];
            if (count == 0) continue; // empty cluster: leave centroid unchanged
            const Point2D new_centroid{sums[static_cast<std::size_t>(m)].x / count,
                                        sums[static_cast<std::size_t>(m)].y / count};
            Point2D& centroid = result.centroids[static_cast<std::size_t>(m)];
            const double dx = new_centroid.x - centroid.x;
            const double dy = new_centroid.y - centroid.y;
            max_shift = std::max(max_shift, std::sqrt(dx * dx + dy * dy));
            centroid = new_centroid;
        }

        if (!any_change || max_shift < policy.convergence_epsilon) {
            result.converged = true;
            result.assignment = assignment;
            return result;
        }
    }

    result.converged = false;
    result.assignment = assignment;
    return result;
}

} // namespace tdr::init
