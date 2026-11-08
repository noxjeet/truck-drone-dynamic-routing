#pragma once

// Phase F: initial-solution construction (steps F1-F4, paper Sec 5.2,
// Fig. 8, Algorithm 2). Orchestrates truck-only TSP (F1,
// truck_only_route.hpp), K-means clustering (F2, customer_clustering.hpp),
// flexible-launch-point generation (F3, reuses Phase D), and greedy
// drone-trip construction (F4). See DECISIONS.md ("Initial-solution
// construction (Phase F)" and "Paper-conformance corrections
// (29 Sep 2026)") for the full [PAPER]/[INFERENCE]/[ASSUMPTION] breakdown
// -- that document is authoritative; comments here are a summary.
//
// RULE: this file never computes timing/endurance/objective/feasibility
// itself. It only builds candidate routes/trips and asks
// tdr::timing::evaluate_solution whether each candidate is feasible.

#include <algorithm>
#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "flp/flexible_launch_point_generator.hpp"
#include "graph/spatial_network.hpp"
#include "init/customer_clustering.hpp"
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
    // F3: L_{a,b} for every arc (a, b) between the depot and the customers
    // (the paper's L = union of L_ij over i in C0, j in C_E), generated
    // once and fixed; only FLPs of arcs the final route traverses are used.
    std::vector<tdr::flp::GeneratedFlexibleLaunchPoint> candidate_flps;
    tdr::timing::FlpDataMap flp_data; // arc ownership of every FLP in candidate_flps (input to evaluate_solution)

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

// Assigns physical drone ids u in [0, num_drones) to the trips of
// `solution` so that no drone flies two trips whose route-position
// intervals [launch position, recovery position] overlap (the route-order
// form of constraint (28)). Trips are processed in route order of their
// launch position and each takes the lowest-numbered drone that is free
// at that launch position; for intervals processed by start this uses the
// minimum possible number of drones, so it fails only if more than
// num_drones trips are genuinely in flight at once.
// [IMPLEMENTATION ASSUMPTION] which free drone is chosen (lowest id) is
// not specified by the paper; see DECISIONS.md (Algorithm 2's per-drone
// loop is a professor-pending item).
// Trips whose launch/recovery cannot be located in route order are given
// drone 0; the evaluator rejects them for that reason anyway.
// Returns false iff more than num_drones drones would be needed.
inline bool assign_drone_ids_by_route_order(tdr::model::Solution& solution, int num_drones) {
    const auto& route = solution.truck_route;
    struct Placed { std::size_t trip; std::size_t lp; std::size_t rp; };
    std::vector<Placed> placed;
    for (std::size_t i = 0; i < solution.drone_trips.size(); ++i) {
        auto& t = solution.drone_trips[i];
        const auto lp = tdr::timing::detail::find_route_position(route, t.launch_node);
        const auto rp = lp ? tdr::timing::detail::find_route_position_from(route, t.recovery_node, *lp + 1)
                           : std::nullopt;
        if (!lp || !rp) { t.drone_id = 0; continue; }
        placed.push_back(Placed{i, *lp, *rp});
    }
    std::sort(placed.begin(), placed.end(), [](const Placed& a, const Placed& b) {
        if (a.lp != b.lp) return a.lp < b.lp;
        return a.trip < b.trip;
    });
    std::vector<std::optional<std::size_t>> last_recovery(static_cast<std::size_t>(std::max(0, num_drones)));
    for (const Placed& p : placed) {
        int chosen = -1;
        for (int u = 0; u < num_drones; ++u) {
            const auto& last = last_recovery[static_cast<std::size_t>(u)];
            if (!last || *last < p.lp) { chosen = u; break; }
        }
        if (chosen < 0) return false;
        solution.drone_trips[p.trip].drone_id = chosen;
        last_recovery[static_cast<std::size_t>(chosen)] = p.rp;
    }
    return true;
}

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

    // ---- F3: flexible launch points L_{a,b} for every arc -----------------
    // [PAPER] L_ij is defined for every i in C0, j in C_E (Table 2), and a
    // launch point of L_ij is usable only while the truck traverses (i,j)
    // (p9). Step (c) inserts launch points "on the current truck route"
    // (p19); because converting a customer to drone service changes the
    // route (it creates the arc (a, b) that bypasses the customer), the
    // candidate launch points of that conversion are those of the arc the
    // RESULTING route traverses. Generating L for every arc up front (a pure,
    // deterministic function of the arc and the policy) is the simplest way
    // to have them available; nothing here depends on it beyond lookup.
    std::vector<NodeId> truck_nodes;
    truck_nodes.push_back(depot_id);
    {
        std::vector<NodeId> sorted_customers = customer_ids;
        std::sort(sorted_customers.begin(), sorted_customers.end());
        truck_nodes.insert(truck_nodes.end(), sorted_customers.begin(), sorted_customers.end());
    }
    std::vector<tdr::flp::Arc> all_arcs;
    for (NodeId from : truck_nodes) {
        for (NodeId to : truck_nodes) {
            if (from != to) all_arcs.emplace_back(from, to);
        }
    }

    NodeId next_flp_id = depot_id;
    for (NodeId id : customer_ids) next_flp_id = std::max(next_flp_id, id);
    next_flp_id += 1;

    const tdr::flp::FlpGenerationResult flp_gen =
        tdr::flp::generate_flexible_launch_points_on_arcs(all_arcs, network.nodes(), policy.flp_policy, next_flp_id);
    if (!flp_gen.ok) {
        out.diagnostics.push_back("F3: " + flp_gen.error);
        return out;
    }
    out.candidate_flps = flp_gen.points;

    NodeRegistry augmented_nodes;
    for (const auto& [id, node] : network.nodes().all()) {
        (void)id;
        augmented_nodes.add(node);
    }
    std::map<tdr::flp::Arc, std::vector<NodeId>> flps_on_arc; // ordered by ratio (generation order)
    for (const auto& gen : flp_gen.points) {
        augmented_nodes.add(gen.node);
        out.flp_data[gen.node.id] = gen.data;
        flps_on_arc[{gen.data.arc_from, gen.data.arc_to}].push_back(gen.node.id);
    }
    tdr::graph::SpatialNetwork augmented_network(augmented_nodes, network.truck_parameters(), network.drone_parameters());
    out.node_registry = augmented_nodes;

    // ---- F4: per-cluster greedy drone-trip construction -------------------
    TruckRoute working_route = r0;
    Solution accepted;           // accepted.truck_route mirrors working_route
    accepted.truck_route = working_route;
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

    const int fleet_size = std::max(0, network.drone_parameters().num_drones);
    const auto is_flp = [&](NodeId id) { return augmented_network.nodes().at(id).type == NodeType::FlexibleLaunchPoint; };

    for (int c = 0; c < out.clustering.cluster_count; ++c) {
        while (true) {
            std::vector<detail::DroneTripCandidate> feasible_this_round;

            for (NodeId j : clusters_customers[static_cast<std::size_t>(c)]) {
                if (converted.count(j) || recovery_locked.count(j)) continue;

                auto pos_it = std::find(working_route.begin(), working_route.end(), j);
                if (pos_it == working_route.end()) continue; // defensive
                const std::size_t pos_j = static_cast<std::size_t>(pos_it - working_route.begin());

                // a / b: nearest truck (non-FLP) nodes before / after j.
                NodeId a = tdr::model::kInvalidNodeId;
                NodeId b = tdr::model::kInvalidNodeId;
                for (std::size_t q = pos_j; q-- > 0;) {
                    if (!is_flp(working_route[q])) { a = working_route[q]; break; }
                }
                for (std::size_t q = pos_j + 1; q < working_route.size(); ++q) {
                    if (!is_flp(working_route[q])) { b = working_route[q]; break; }
                }
                if (a == tdr::model::kInvalidNodeId || b == tdr::model::kInvalidNodeId) continue; // defensive
                if (a == b) continue; // (a, a) is not an arc (only when the route is depot-j-depot)

                // [INFERENCE/ASSUMPTION] Launch candidates: every launch point
                // of the arc (a, b) that the resulting route traverses after j
                // is removed. Recovery candidate: b, the next truck node after
                // j (unchanged narrowest reading; see DECISIONS.md).
                const auto arc_it = flps_on_arc.find({a, b});
                if (arc_it == flps_on_arc.end()) continue; // arc too short for any launch point
                const NodeId k_candidate = b;

                for (NodeId launch : arc_it->second) {
                    DroneTrip trip;
                    trip.launch_node = launch;
                    trip.service_customer = j;
                    trip.recovery_node = k_candidate;

                    Solution trial = accepted;
                    trial.truck_route[pos_j] = launch; // j leaves the truck route; the launch point takes its place
                    trial.drone_trips.push_back(trip);
                    if (!detail::assign_drone_ids_by_route_order(trial, fleet_size)) continue;

                    const tdr::timing::SolutionEvaluationResult trial_eval =
                        tdr::timing::evaluate_solution(trial, augmented_network, customers, out.flp_data);
                    if (!trial_eval.feasible) continue;
                    if (trial_eval.drone_trip_timings.empty() || !trial_eval.drone_trip_timings.back().feasible) continue;

                    double arrival = 0.0;
                    bool j_ok = false;
                    for (const auto& rec : trial_eval.customer_records) {
                        if (rec.customer_id == j && rec.served_by_drone && rec.feasible) {
                            j_ok = true;
                            arrival = rec.service_arrival_time;
                            break;
                        }
                    }
                    if (!j_ok) continue;

                    feasible_this_round.push_back(detail::DroneTripCandidate{j, trip, arrival});
                }
            }

            if (feasible_this_round.empty()) break; // cluster exhausted this round

            std::sort(feasible_this_round.begin(), feasible_this_round.end(),
                      [](const detail::DroneTripCandidate& x, const detail::DroneTripCandidate& y) {
                          if (x.score != y.score) return x.score < y.score;
                          if (x.customer != y.customer) return x.customer < y.customer;
                          return x.trip.launch_node < y.trip.launch_node;
                      });
            const detail::DroneTripCandidate& chosen = feasible_this_round.front();

            auto pos_it = std::find(working_route.begin(), working_route.end(), chosen.customer);
            const std::size_t pos_j = static_cast<std::size_t>(pos_it - working_route.begin());
            working_route[pos_j] = chosen.trip.launch_node;
            accepted.truck_route = working_route;
            accepted.drone_trips.push_back(chosen.trip);
            detail::assign_drone_ids_by_route_order(accepted, fleet_size); // succeeded for this set during the trial
            converted.insert(chosen.customer);
            if (augmented_network.nodes().at(chosen.trip.recovery_node).type == NodeType::Customer) {
                recovery_locked.insert(chosen.trip.recovery_node);
            }
        }
    }

    out.solution = accepted;
    out.evaluation = tdr::timing::evaluate_solution(out.solution, augmented_network, customers, out.flp_data);
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
