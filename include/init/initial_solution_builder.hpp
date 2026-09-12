#pragma once

// Phase F: initial-solution construction (steps F1-F4). Orchestrates
// truck-only TSP (F1, truck_only_route.hpp), K-means clustering (F2,
// customer_clustering.hpp), FLP insertion (F3, reuses Phase D unchanged),
// and greedy drone-trip construction (F4). See DECISIONS.md
// ("Initial-solution construction (Phase F)") for the full
// [PAPER]/[INFERENCE]/[ASSUMPTION] breakdown -- that document is
// authoritative; comments here are a summary.
//
// RULE: this file never computes timing/endurance/objective itself. It
// only builds candidate routes/trips and asks
// tdr::timing::evaluate_solution whether each candidate is feasible.

#include <algorithm>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "flp/flexible_launch_point_generator.hpp"
#include "graph/spatial_network.hpp"
#include "init/customer_clustering.hpp"
#include "init/drone_fleet_capacity.hpp"
#include "init/truck_only_route.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/node.hpp"
#include "model/solution.hpp"
#include "model/truck_route.hpp"
#include "timing/timing_engine.hpp"
#include "tsp/tsp_solver_factory.hpp"

namespace tdr::init {

struct InitialSolutionPolicy {
    KMeansPolicy kmeans_policy;
    tdr::flp::FlpGenerationPolicy flp_policy;
};

struct InitialSolutionResult {
    tdr::model::Solution solution;
    tdr::model::NodeRegistry node_registry; // depot + customers + every candidate FLP (used or not)

    tdr::tsp::TSPResult tsp_result;         // F1 metadata (exact/heuristic, cost, etc.)
    tdr::model::TruckRoute truck_only_route; // F1: r0 itself (depot + every customer, no FLPs, no conversions)
    tdr::init::KMeansResult clustering;                      // F2 metadata
    std::vector<tdr::flp::GeneratedFlexibleLaunchPoint> candidate_flps; // F3: full frozen candidate pool, generated from truck_only_route

    tdr::timing::SolutionEvaluationResult evaluation; // Phase E's evaluation of `solution` (customer
                                                       // service assignment lives in evaluation.customer_records)
    bool feasible = false;
    double objective_value = 0.0;
    std::vector<std::string> diagnostics;
};

namespace detail {

// One round's candidate: converting customer `customer` via `trip`,
// scored by the resulting drone customer_arrival_time (lower is better).
struct DroneTripCandidate {
    tdr::model::NodeId customer = tdr::model::kInvalidNodeId;
    tdr::model::DroneTrip trip;
    double score = 0.0;
};

} // namespace detail

// F1+F2+F3+F4 orchestration. `network` must already have `depot_id` and
// every id in `customer_ids` registered (FLP nodes are added internally).
inline InitialSolutionResult build_initial_solution(
    tdr::model::NodeId depot_id,
    const std::vector<tdr::model::NodeId>& customer_ids,
    const std::unordered_map<tdr::model::NodeId, tdr::model::CustomerData>& customers,
    const tdr::graph::SpatialNetwork& network,
    const InitialSolutionPolicy& policy = InitialSolutionPolicy{},
    const tdr::tsp::TSPSolverFactory& solver_factory = tdr::tsp::TSPSolverFactory{}) {

    using tdr::model::DroneTrip;
    using tdr::model::NodeId;
    using tdr::model::NodeRegistry;
    using tdr::model::NodeType;
    using tdr::model::Point2D;
    using tdr::model::Solution;
    using tdr::model::TruckRoute;

    InitialSolutionResult out;

    // ---- F1: truck-only route -------------------------------------------
    TruckOnlyRouteResult truck_only = build_truck_only_route(depot_id, customer_ids, network, solver_factory);
    out.tsp_result = truck_only.tsp_result;
    if (!truck_only.tsp_result.succeeded) {
        out.diagnostics.push_back("F1: truck-only TSP solve failed: " + truck_only.tsp_result.note);
        return out;
    }
    const TruckRoute r0 = truck_only.route;
    out.truck_only_route = r0;

    // ---- F2: K-means clustering (independent of r0) ----------------------
    std::unordered_map<NodeId, Point2D> coords;
    coords.reserve(customer_ids.size());
    for (NodeId id : customer_ids) coords[id] = network.nodes().at(id).coordinates;
    const int k = compute_cluster_count(customer_ids.size(), network.drone_parameters().num_drones);
    out.clustering = run_kmeans(coords, k, policy.kmeans_policy);

    // ---- F3: frozen FLP candidate pool, generated once from r0 -----------
    NodeId next_flp_id = depot_id;
    for (NodeId id : customer_ids) next_flp_id = std::max(next_flp_id, id);
    next_flp_id += 1;

    tdr::flp::FlpGenerationResult flp_gen =
        tdr::flp::generate_flexible_launch_points(r0, network.nodes(), policy.flp_policy, next_flp_id);
    out.candidate_flps = flp_gen.points;

    NodeRegistry augmented_nodes;
    for (const auto& [id, node] : network.nodes().all()) {
        (void)id;
        augmented_nodes.add(node);
    }
    for (const auto& gen : flp_gen.points) augmented_nodes.add(gen.node);
    tdr::graph::SpatialNetwork augmented_network(augmented_nodes, network.truck_parameters(), network.drone_parameters());

    // Each customer's launch CANDIDATES = every FLP generated on the arc
    // that entered it in r0 (data.arc_to == customer id) -- ALL of them,
    // not just one. [Post-approval correctness patch: previously this
    // kept only the single closest same-arc FLP, an unintended narrowing
    // beyond "the FLP(s) attached to this customer" -- see DECISIONS.md,
    // "Post-approval correctness patch", Issue 2.] This is still scoped
    // to the arc directly entering the customer (the narrowest defensible
    // reading of Algorithm 2's "adjacent nodes" -- see F4 below for the
    // parallel reasoning on the recovery side); it does not search
    // earlier arcs or a geometric radius. [INFERENCE/ASSUMPTION -- see
    // DECISIONS.md for the remaining, explicitly acknowledged ambiguity.]
    std::unordered_map<NodeId, std::vector<const tdr::flp::GeneratedFlexibleLaunchPoint*>> launch_candidates_for;
    for (const auto& gen : flp_gen.points) {
        launch_candidates_for[gen.data.arc_to].push_back(&gen);
    }

    out.node_registry = augmented_nodes;

    // ---- F4: per-cluster greedy drone-trip construction -------------------
    TruckRoute working_route = r0;
    std::vector<DroneTrip> accepted_trips;
    std::unordered_set<NodeId> converted;       // customers already turned into drone trips
    std::unordered_set<NodeId> recovery_locked; // customers already used as someone's recovery target

    std::vector<std::vector<NodeId>> clusters_customers(static_cast<std::size_t>(std::max(0, out.clustering.cluster_count)));
    for (NodeId id : customer_ids) {
        auto it = out.clustering.assignment.find(id);
        if (it != out.clustering.assignment.end() && it->second >= 0 &&
            it->second < out.clustering.cluster_count) {
            clusters_customers[static_cast<std::size_t>(it->second)].push_back(id);
        }
    }
    for (auto& v : clusters_customers) std::sort(v.begin(), v.end());

    // [Post-approval correctness patch, Issue 1] `fleet_size` and
    // `slot_counter` are single running values shared across the WHOLE
    // construction (every cluster), not reset per cluster -- see
    // DECISIONS.md, "Post-approval correctness patch". There is no
    // per-cluster drone-count budget any more: whether a candidate trip
    // can be accepted is decided purely by whether the set of ALL trips
    // accepted so far (across every cluster processed up to this point)
    // plus this candidate would ever need more than `fleet_size` drones
    // simultaneously in the air, via `check_drone_fleet_capacity()` below.
    const int fleet_size = std::max(0, network.drone_parameters().num_drones);
    int slot_counter = 0;

    for (int c = 0; c < out.clustering.cluster_count; ++c) {
        while (true) {
            std::vector<detail::DroneTripCandidate> feasible_this_round;

            for (NodeId j : clusters_customers[static_cast<std::size_t>(c)]) {
                if (converted.count(j) || recovery_locked.count(j)) continue;

                auto launch_it = launch_candidates_for.find(j);
                if (launch_it == launch_candidates_for.end()) continue; // no candidate FLP for this customer

                auto pos_it = std::find(working_route.begin(), working_route.end(), j);
                if (pos_it == working_route.end()) continue; // defensive; should not happen given `converted`
                const std::size_t pos_j = static_cast<std::size_t>(pos_it - working_route.begin());

                NodeId k_candidate = tdr::model::kInvalidNodeId;
                for (std::size_t p = pos_j + 1; p < working_route.size(); ++p) {
                    const NodeType t = augmented_network.nodes().at(working_route[p]).type;
                    if (t == NodeType::Customer || t == NodeType::Depot) {
                        k_candidate = working_route[p];
                        break;
                    }
                }
                if (k_candidate == tdr::model::kInvalidNodeId) continue; // should not happen: route ends at depot

                // [Post-approval correctness patch, Issue 2] Try EVERY FLP
                // attached to this customer's entering arc (see the
                // [INFERENCE/ASSUMPTION] note above `launch_candidates_for`),
                // not just the single closest one -- each is a fully
                // independent candidate trip, evaluated on its own merits.
                for (const tdr::flp::GeneratedFlexibleLaunchPoint* launch : launch_it->second) {
                    DroneTrip trip;
                    trip.launch_node = launch->node.id;
                    trip.service_customer = j;
                    trip.recovery_node = k_candidate;

                    TruckRoute trial_route = working_route;
                    trial_route[pos_j] = launch->node.id;
                    Solution trial_solution;
                    trial_solution.truck_route = trial_route;
                    trial_solution.drone_trips = accepted_trips;
                    trial_solution.drone_trips.push_back(trip);

                    const tdr::timing::SolutionEvaluationResult trial_eval =
                        tdr::timing::evaluate_solution(trial_solution, augmented_network, customers);

                    if (trial_eval.drone_trip_timings.empty() || !trial_eval.drone_trip_timings.back().feasible) {
                        continue;
                    }
                    bool j_ok = false;
                    double arrival = 0.0;
                    for (const auto& rec : trial_eval.customer_records) {
                        if (rec.customer_id == j && rec.served_by_drone && rec.feasible) {
                            j_ok = true;
                            arrival = rec.service_arrival_time;
                            break;
                        }
                    }
                    if (!j_ok || !trial_eval.feasible) continue;

                    // Global, time-aware drone-fleet capacity check
                    // (Issue 1): reject this candidate if the trips
                    // accepted so far PLUS this one would ever require
                    // more than `fleet_size` drones simultaneously in
                    // flight. Uses only Phase E's own already-computed
                    // launch_time / recovery_complete_time values -- no
                    // timing logic is duplicated here.
                    const auto fleet_check = tdr::init::check_drone_fleet_capacity(
                        trial_eval.drone_trip_timings, fleet_size);
                    if (!fleet_check.feasible) continue;

                    feasible_this_round.push_back(detail::DroneTripCandidate{j, trip, arrival});
                }
            }

            if (feasible_this_round.empty()) break; // cluster exhausted this round

            std::sort(feasible_this_round.begin(), feasible_this_round.end(),
                      [](const detail::DroneTripCandidate& a, const detail::DroneTripCandidate& b) {
                          if (a.score != b.score) return a.score < b.score;
                          if (a.customer != b.customer) return a.customer < b.customer;
                          return a.trip.launch_node < b.trip.launch_node;
                      });
            const detail::DroneTripCandidate& chosen = feasible_this_round.front();

            auto pos_it = std::find(working_route.begin(), working_route.end(), chosen.customer);
            const std::size_t pos_j = static_cast<std::size_t>(pos_it - working_route.begin());
            working_route[pos_j] = chosen.trip.launch_node;

            DroneTrip committed = chosen.trip;
            committed.drone_slot = slot_counter++;
            accepted_trips.push_back(committed);
            converted.insert(chosen.customer);
            if (augmented_network.nodes().at(chosen.trip.recovery_node).type == NodeType::Customer) {
                recovery_locked.insert(chosen.trip.recovery_node);
            }
        }
    }

    out.solution.truck_route = working_route;
    out.solution.drone_trips = accepted_trips;
    out.evaluation = tdr::timing::evaluate_solution(out.solution, augmented_network, customers);
    out.feasible = out.evaluation.feasible;
    out.objective_value = out.evaluation.objective_value;
    if (!out.feasible) {
        for (const auto& reason : out.evaluation.violation_reasons) {
            out.diagnostics.push_back("F4: " + reason);
        }
    }

    return out;
}

} // namespace tdr::init
