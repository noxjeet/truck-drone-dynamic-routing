#pragma once

// Centralized timing and feasibility engine for a static truck-and-drone
// Solution (Han, Chi & Sun 2026, Sec 4.2). This is the ONLY place in the
// repository that computes time, feasibility, or the objective value -- no
// later module (initial-solution construction, neighborhoods, VNS) is meant
// to recompute any of this itself. See DECISIONS.md ("Timing and
// feasibility engine (Phase E)" and "Paper-conformance corrections
// (29 Sep 2026)") for the complete [PAPER]/[INFERENCE]/[ASSUMPTION]
// breakdown. Paper page numbers below are PDF pages (journal page = 4071 +
// PDF page).
//
// TIMING MODEL IMPLEMENTED
//
//  Start: the truck departs the starting depot (route position 0) at time 0.
//  [PAPER, p15: "this paper assumes that operating time starts at 0".]
//
//  Truck arrival (route positions p = 1..n-1):
//    arrival[p] = departure[p-1] + d(route[p-1], route[p])
//  [PAPER, constraint (16). The printed (16) reads "l_j + d_ij - (1-x_ij)M
//  <= t_j"; the l_j is a typographical slip for l_i -- relating node j's own
//  departure to its own arrival would make (16) meaningless. Implemented as
//  the operationally coherent l_i.]
//
//  Flexible launch point i with n_i >= 1 drones launching there:
//    each launched drone u:  l^_{i,u} = t_i + SL
//      [PAPER, (21): t_i + SL <= l^_{i,u}, one SL per drone measured from
//      the truck's arrival; (19): l^_{i,u} <= l_i. Neither constraint orders
//      the drones against each other, so NO staggering is applied. The
//      earliest admissible value is used -- see "launch-delay slack" in
//      DECISIONS.md, an IMPLEMENTATION ASSUMPTION.]
//    truck:                  l_i = t_i + n_i * SL
//      [PAPER, (18): the term  sum_u sum_{s in P_i^+} y_s^u * SL  equals
//      n_i * SL. The SR term that (18) prints over P_i^+ is NOT applied
//      here -- see PAPER AMBIGUITY: SR below. Se_i^T is taken as 0 at a
//      launch point (the paper defines Se^T only for truck service nodes).]
//    A flexible launch point never hosts truck service or a recovery (node
//    roles, Phase B).
//
//  Customer / depot node p  (SR rule: DECISION D1, Option B, 8 Nov 2026):
//    r_p      = number of drones recovered at p
//    a_u      = drone u's own arrival at p  (l^_{j,u} + d^(j,p))
//    l_p = max( t_p + Se_p^T + r_p * SR ,  max_{u recovered at p} (a_u + SR) )
//    (Se^T = 0 for the depot; with r_p = 0 this is just t_p + Se_p^T.)
//    Mathematical content:
//      - recovery work is ADDITIVE to the truck's own service at p, and each
//        of the r_p recoveries costs SR of truck time (t_p + Se_p^T + r_p*SR);
//      - independently, constraint (20) bounds each drone from below:
//        l_p >= a_u + SR  (a drone that lands after the truck still needs SR);
//      - SR is NEVER charged at a launch point (l_i = t_i + n_i*SL above).
//
//    PAPER AMBIGUITY: SR -- RESOLVED BY DECISION D1 (user, 8 Nov 2026).
//    The paper prints (18) with its SR sum over P_i^+ (routes LAUNCHED at i),
//    while the explanatory text (p15) says (18) covers "drone recovery
//    tasks", and (20) separately imposes l_k >= l^_{j,u} + d^_{jk} + SR per
//    recovering drone. Two readings were considered:
//      (A) literal: n_i * SR charged at the launch point i, recovery at k
//          governed by (20) only;
//      (B) index slip (P_k^- intended): the rule above.
//    Option B is ADOPTED. It is an interpretation of an ambiguous printed
//    formula, not a value the paper states; see DECISIONS.md (Stage 2).
//    The pre-Stage-2 rule max(t_k,a_u)+SR (SR concurrent with truck service
//    and parallel across drones) matched neither reading and was replaced.
//
//  Drone trip s = <i, j, k> flown by drone u:
//    t^_{j,u} = l^_{i,u} + d^(i,j)             [PAPER, (17)]
//    l^_{j,u} = t^_{j,u} + Se_j^D              [PAPER, (22)]
//    drone arrival at k = l^_{j,u} + d^(j,k)   [INFERENCE, same additive logic]
//
//  Endurance [PAPER, (36) and (37), p15], both evaluated literally with the
//  schedule above:
//    (36): e >= (t_k - l^_{j,u}) + (l^_{j,u} - l^_{i,u}) * (1 + theta_j)
//    (37): e >= (l^_{j,u} - l^_{i,u}) * (1 + theta_j) + d^_{j,k}
//  where t_k is the TRUCK's arrival at the recovery node. Consequences
//  that follow directly from the equations (not added rules):
//    - the loaded portion (l^_{j,u} - l^_{i,u}) is outbound flight PLUS
//      Se_j^D, and theta_j multiplies both;
//    - the return leg is unscaled;
//    - if the truck reaches k after the drone, (36) counts the drone's
//      waiting until t_k (text p15: "the flight duration of a drone, while
//      awaiting the truck, must not exceed the permissible battery
//      capacity"; p30: "including waiting for the truck at the merging
//      point");
//    - SL and SR do not appear in (36)/(37) and do not count.
//  A trip is endurance-feasible iff both values are <= e (+ epsilon).
//
//  Physical drones [PAPER, u in U; (7), (11), (12), (28)]:
//    - every trip's drone_id must satisfy 0 <= u < |U|;
//    - (28): a drone launched at i and recovered at k cannot be relaunched
//      at a later launch point g before t_k. Because the truck visits route
//      positions in increasing time order, this is equivalent to: for each
//      drone, taking its trips in route order, the next launch position
//      lies strictly after the previous recovery position [INFERENCE,
//      assumes positive travel times; see DECISIONS.md];
//    - no more than |U| trips may be in flight at once (implied by the two
//      rules above; also checked directly for a clear diagnostic).
//
//  Flexible launch point rules:
//    - arc membership [PAPER, p9: "If the truck does not pass through arc
//      (i,j), it cannot access the launch point in set L_ij"; (13)-(15) and
//      their text p14]: every FLP of L_{a,b} must sit in the truck route
//      between truck nodes a and b (possibly next to other FLPs of the same
//      arc). The printed quantifiers of (13)-(15), read literally, would
//      forbid every arc into or out of any FLP; the text's meaning is
//      implemented. Order among FLPs of the same arc is not constrained by
//      the paper and is not checked;
//    - (6): a visited FLP must have at least one launch;
//    - (8)/(9): an FLP is entered and left at most once, i.e. it appears at
//      most once in the route.
//
//  Customer service and objective [PAPER, (1), (10)]: each customer is
//  served exactly once, by the truck or by exactly one feasible drone trip;
//  the objective is the sum over customers of the serving vehicle's arrival
//  time (t_j if truck-served, t^_{j,u} if drone-served). Violations make the
//  solution infeasible and are reported; the objective is still computed
//  best-effort.
//
//  INPUT VALIDATION (Stage 1, 8 Nov 2026). The evaluator is the only oracle
//  for search code, so it must never throw on a malformed Solution and must
//  never accept one. It reports a violation (and returns feasible == false)
//  for:
//    - a route position, or any CustomerData key, that is not a node of the
//      network (a route with an unknown id is returned immediately: its
//      timing cannot be computed);
//    - a CustomerData key that is not a Customer node, or whose node_id field
//      differs from its key;
//    - a Customer node of the network with no CustomerData entry;
//    - a route that does not start and end at the SAME depot node, or that
//      contains a depot node anywhere but its first and last position
//      (the paper's single depot-to-depot path, constraints (2)-(3));
//    - a customer appearing more than once in the truck route (constraint
//      (10): each customer is served exactly once; an FLP appearing twice is
//      already covered by (8)-(9)).
//  Customer coverage (truck xor exactly one drone trip) was already checked
//  per CustomerData entry; entries that do not refer to a network customer
//  are reported once and excluded from that coverage check and from truck
//  service-time lookup. No timing, objective, endurance, SR or launch rule is
//  changed by this validation; it only adds violations.

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/flexible_launch_point.hpp"
#include "model/node.hpp"
#include "model/solution.hpp"
#include "model/truck_route.hpp"

namespace tdr::timing {

// Small floating-point tolerance for endurance/time comparisons at the
// boundary.
inline constexpr double kTimeEpsilon = 1e-9;

// Implementation term: the per-FLP arc-ownership table (node id -> the
// L_{i,j} it belongs to), as produced by the FLP generator.
using FlpDataMap = std::unordered_map<tdr::model::NodeId, tdr::model::FlexibleLaunchPointData>;

// Timing for one drone trip, plus its own feasibility verdict.
struct DroneTripTiming {
    bool structurally_valid = false; // Phase B's has_distinct_endpoints() && has_valid_node_roles()
    bool on_route = false;           // launch and recovery nodes both found on the route, launch strictly before recovery
    bool has_customer_data = false;  // CustomerData exists for the served customer
    bool drone_id_valid = false;     // 0 <= drone_id < |U|

    double launch_time = 0.0;              // l^_{i,u} = t_i + SL
    double outbound_flight_time_raw = 0.0; // d^(i,j)
    double customer_arrival_time = 0.0;    // t^_{j,u} -- THE objective-feeding time if drone-served
    double customer_departure_time = 0.0;  // l^_{j,u} = t^_{j,u} + Se_j^D
    double return_flight_time_raw = 0.0;   // d^(j,k)
    double recovery_arrival_time = 0.0;    // l^_{j,u} + d^(j,k): the drone's own arrival at k
    double truck_arrival_at_recovery = 0.0; // t_k
    double recovery_complete_time = 0.0;   // max(t_k, drone arrival) + SR: earliest the drone is physically free again (NOT the truck-departure rule; always <= l_k, see header)

    double theta_used = 0.0;
    double endurance_value_36 = 0.0; // (t_k - l^_{j,u}) + (l^_{j,u} - l^_{i,u})(1 + theta_j)
    double endurance_value_37 = 0.0; // (l^_{j,u} - l^_{i,u})(1 + theta_j) + d^_{j,k}
    double generalized_flight_time = 0.0; // max(endurance_value_36, endurance_value_37): the binding one
    bool endurance_feasible = false;      // both values <= e (+ epsilon)

    bool feasible = false;
    std::string violation_reason; // empty iff feasible
};

// Truck arrival/departure at every route position (parallel to the route).
struct TruckTimingResult {
    std::vector<double> arrival_time;
    std::vector<double> departure_time;
};

// One customer's resolution.
struct CustomerServiceRecord {
    tdr::model::NodeId customer_id = tdr::model::kInvalidNodeId;
    bool served_by_truck = false;
    bool served_by_drone = false;
    double service_arrival_time = 0.0;
    bool feasible = true;
    std::string violation_reason;
};

// Waiting at a recovery rendezvous, for inspection (Assumption 7).
struct SynchronizationRecord {
    tdr::model::NodeId recovery_node = tdr::model::kInvalidNodeId;
    double truck_arrival = 0.0;
    double drone_arrival = 0.0;
    bool truck_waited = false;
    bool drone_waited = false;
    double wait_time = 0.0;
};

// The complete evaluation of one static Solution.
struct SolutionEvaluationResult {
    bool feasible = false;
    // Sum of every customer's service_arrival_time. Authoritative only when
    // feasible == true; computed best-effort (never penalized) otherwise.
    double objective_value = 0.0;

    TruckTimingResult truck_timing;
    std::vector<DroneTripTiming> drone_trip_timings; // parallel to solution.drone_trips
    std::vector<CustomerServiceRecord> customer_records;
    std::vector<SynchronizationRecord> synchronization_records;
    // Largest number of trips simultaneously in flight (route-position
    // intervals [launch position, recovery position]); must be <= |U|.
    int max_simultaneous_drones = 0;
    std::vector<std::string> violation_reasons;
};

namespace detail {

inline std::optional<std::size_t> find_route_position(const tdr::model::TruckRoute& route, tdr::model::NodeId id) {
    for (std::size_t i = 0; i < route.size(); ++i) {
        if (route[i] == id) return i;
    }
    return std::nullopt;
}

// Same as find_route_position, but only considers positions >= from_index.
// Used for the recovery node, which must lie strictly after the launch
// position; this also resolves the start/end depot sharing one NodeId.
inline std::optional<std::size_t> find_route_position_from(
    const tdr::model::TruckRoute& route, tdr::model::NodeId id, std::size_t from_index) {
    for (std::size_t i = from_index; i < route.size(); ++i) {
        if (route[i] == id) return i;
    }
    return std::nullopt;
}

struct TripIntermediate {
    double launch_time = 0.0;
    double outbound_flight_time_raw = 0.0;
    double customer_arrival_time = 0.0;
    double customer_departure_time = 0.0;
    double return_flight_time_raw = 0.0;
    double recovery_arrival_time = 0.0;
    double theta_used = 0.0;
    bool computed = false;
};

} // namespace detail

// Evaluates one complete static Solution. Pure function -- no mutation, no
// randomness, no optimization/search logic.
//
// `flp_data` must contain the arc ownership (FlexibleLaunchPointData) of
// every flexible launch point that appears in the truck route; an FLP
// without ownership data is reported as a violation, since its membership
// in some L_{i,j} cannot be verified.
inline SolutionEvaluationResult evaluate_solution(
    const tdr::model::Solution& solution,
    const tdr::graph::SpatialNetwork& network,
    const std::unordered_map<tdr::model::NodeId, tdr::model::CustomerData>& customers,
    const FlpDataMap& flp_data) {

    using tdr::model::DroneTrip;
    using tdr::model::NodeId;
    using tdr::model::NodeType;
    using tdr::model::TruckRoute;

    SolutionEvaluationResult result;
    const TruckRoute& route = solution.truck_route;
    std::vector<std::string>& reasons = result.violation_reasons;

    const auto& registry = network.nodes();
    const auto type_name = [](NodeType t) {
        switch (t) {
            case NodeType::Depot: return "depot";
            case NodeType::Customer: return "customer";
            case NodeType::FlexibleLaunchPoint: return "flexible launch point";
        }
        return "unknown";
    };

    // --- Instance data: CustomerData must match the network's customers ----
    // Keys are visited in ascending order so the reported violations are
    // deterministic.
    std::unordered_set<NodeId> invalid_customer_entries; // keys excluded from coverage/service lookup (already reported)
    // [P2] The CustomerData keys in ascending order. Besides fixing the order
    // of the violation reports below, this is the order in which customers are
    // resolved and summed into the objective, so the objective no longer
    // depends on the unordered_map's iteration order (which varies with its
    // insertion history and bucket count).
    std::vector<NodeId> keys;
    {
        keys.reserve(customers.size());
        for (const auto& kv : customers) keys.push_back(kv.first);
        std::sort(keys.begin(), keys.end());
        for (NodeId id : keys) {
            if (!registry.contains(id)) {
                reasons.push_back("CustomerData entry " + std::to_string(id) +
                                  " refers to a node id that is not in the network");
                invalid_customer_entries.insert(id);
            } else if (registry.at(id).type != NodeType::Customer) {
                reasons.push_back("CustomerData entry " + std::to_string(id) + " refers to a node that is a " +
                                  type_name(registry.at(id).type) + ", not a customer");
                invalid_customer_entries.insert(id);
            } else if (customers.at(id).node_id != id) {
                reasons.push_back("CustomerData stored under customer " + std::to_string(id) + " has node_id " +
                                  std::to_string(customers.at(id).node_id) + " (must equal its key)");
            }
        }
        // [P1] Ascending customer ids cached at SpatialNetwork construction
        // (identical to the former full-registry scan + sort, without the
        // O(|V|) pass over the FLP pool).
        for (NodeId id : network.customer_ids()) {
            if (customers.find(id) == customers.end()) {
                reasons.push_back("network customer " + std::to_string(id) +
                                  " has no CustomerData (service times and theta are unknown)");
            }
        }
    }

    // --- Route-level structural preconditions ---------------------------
    if (route.size() < 2) {
        reasons.push_back("truck route must contain at least a start and an end depot node");
        result.feasible = false;
        return result;
    }
    {
        // Every referenced id must exist; otherwise nothing below can be evaluated.
        bool all_known = true;
        for (std::size_t p = 0; p < route.size(); ++p) {
            if (!registry.contains(route[p])) {
                reasons.push_back("truck route position " + std::to_string(p) + " refers to node id " +
                                  std::to_string(route[p]) + ", which is not in the network");
                all_known = false;
            }
        }
        if (!all_known) {
            result.feasible = false;
            return result;
        }
    }

    const std::size_t n = route.size();
    const auto node_type_at = [&](std::size_t p) { return registry.at(route[p]).type; };

    if (node_type_at(0) != NodeType::Depot) {
        reasons.push_back("truck route does not start at a depot node");
    }
    if (node_type_at(n - 1) != NodeType::Depot) {
        reasons.push_back("truck route does not end at a depot node");
    }
    if (node_type_at(0) == NodeType::Depot && node_type_at(n - 1) == NodeType::Depot && route.front() != route.back()) {
        reasons.push_back("truck route starts at depot " + std::to_string(route.front()) +
                          " but ends at a different depot " + std::to_string(route.back()) +
                          " (a single depot identity is required, constraints (2)-(3))");
    }
    {
        for (std::size_t p = 1; p + 1 < n; ++p) {
            if (node_type_at(p) == NodeType::Depot) {
                reasons.push_back("depot node " + std::to_string(route[p]) + " appears at interior route position " +
                                  std::to_string(p) + " (the depot may appear only at the start and the end)");
            }
        }
        std::unordered_set<NodeId> seen_customers;
        for (std::size_t p = 0; p < n; ++p) {
            if (node_type_at(p) == NodeType::Customer && !seen_customers.insert(route[p]).second) {
                reasons.push_back("customer " + std::to_string(route[p]) + " appears more than once in the truck route "
                                  "(constraint (10): each customer is served exactly once)");
            }
        }
    }

    // --- Flexible launch point arc membership and (8)/(9) ----------------
    {
        std::unordered_map<NodeId, int> flp_occurrences;
        for (std::size_t p = 0; p < n; ++p) {
            if (node_type_at(p) != NodeType::FlexibleLaunchPoint) continue;
            const NodeId f = route[p];
            if (++flp_occurrences[f] == 2) {
                reasons.push_back("flexible launch point " + std::to_string(f) +
                                  " appears more than once in the truck route (constraints (8)-(9): each launch "
                                  "point is entered and left at most once)");
            }
            // Nearest truck (non-FLP) nodes before and after position p.
            NodeId a = tdr::model::kInvalidNodeId;
            NodeId b = tdr::model::kInvalidNodeId;
            for (std::size_t q = p; q-- > 0;) {
                if (node_type_at(q) != NodeType::FlexibleLaunchPoint) { a = route[q]; break; }
            }
            for (std::size_t q = p + 1; q < n; ++q) {
                if (node_type_at(q) != NodeType::FlexibleLaunchPoint) { b = route[q]; break; }
            }
            const auto it = flp_data.find(f);
            if (it == flp_data.end()) {
                reasons.push_back("flexible launch point " + std::to_string(f) +
                                  " has no arc-ownership data, so its membership in some L_(i,j) cannot be verified");
            } else if (it->second.arc_from != a || it->second.arc_to != b) {
                reasons.push_back("flexible launch point " + std::to_string(f) + " belongs to L_(" +
                                  std::to_string(it->second.arc_from) + "," + std::to_string(it->second.arc_to) +
                                  ") but the truck route passes it between " + std::to_string(a) + " and " +
                                  std::to_string(b) +
                                  " (p9: a launch point in L_ij is accessible only if the truck traverses arc (i,j); "
                                  "constraints (13)-(15))");
            }
        }
    }

    result.truck_timing.arrival_time.assign(n, 0.0);
    result.truck_timing.departure_time.assign(n, 0.0);
    std::vector<double>& arrival = result.truck_timing.arrival_time;
    std::vector<double>& departure = result.truck_timing.departure_time;

    const std::size_t trip_count = solution.drone_trips.size();
    std::vector<std::optional<std::size_t>> launch_pos(trip_count);
    std::vector<std::optional<std::size_t>> recovery_pos(trip_count);
    std::vector<bool> structurally_valid(trip_count, false);
    std::vector<bool> on_route(trip_count, false);
    std::vector<bool> has_customer_data(trip_count, false);
    std::vector<detail::TripIntermediate> intermediate(trip_count);

    for (std::size_t i = 0; i < trip_count; ++i) {
        const DroneTrip& trip = solution.drone_trips[i];
        structurally_valid[i] = tdr::model::has_distinct_endpoints(trip) &&
                                tdr::model::has_valid_node_roles(trip, network.nodes());
        launch_pos[i] = detail::find_route_position(route, trip.launch_node);
        recovery_pos[i] = launch_pos[i].has_value()
                              ? detail::find_route_position_from(route, trip.recovery_node, *launch_pos[i] + 1)
                              : std::nullopt;
        on_route[i] = launch_pos[i].has_value() && recovery_pos[i].has_value() && *launch_pos[i] < *recovery_pos[i];
        has_customer_data[i] = customers.find(trip.service_customer) != customers.end();
    }

    std::vector<std::vector<std::size_t>> trips_launching_at(n);
    std::vector<std::vector<std::size_t>> trips_recovering_at(n);
    for (std::size_t i = 0; i < trip_count; ++i) {
        if (on_route[i] && structurally_valid[i]) {
            trips_launching_at[*launch_pos[i]].push_back(i);
            trips_recovering_at[*recovery_pos[i]].push_back(i);
        }
    }

    // --- (6): a visited launch point must launch at least one drone -------
    for (std::size_t p = 0; p < n; ++p) {
        if (node_type_at(p) == NodeType::FlexibleLaunchPoint && trips_launching_at[p].empty()) {
            reasons.push_back("flexible launch point " + std::to_string(route[p]) + " at route position " +
                              std::to_string(p) +
                              " is visited but no drone is launched there (constraint (6))");
        }
    }

    const auto& drone = network.drone_parameters();

    // --- Single forward pass over route positions -----------------------
    for (std::size_t p = 0; p < n; ++p) {
        const NodeId v = route[p];

        if (p == 0) {
            arrival[p] = 0.0; // [PAPER, p15] operating time starts at 0
        } else {
            arrival[p] = departure[p - 1] + network.truck_time(route[p - 1], v);
        }

        const NodeType type = network.nodes().at(v).type;

        if (type == NodeType::FlexibleLaunchPoint) {
            const auto& launching = trips_launching_at[p];
            // (21): each drone may depart at t_i + SL (no staggering).
            for (std::size_t i : launching) {
                intermediate[i].launch_time = arrival[p] + drone.launch_prep_time_sl;
            }
            // (18): the truck leaves only after n_i * SL.
            departure[p] = arrival[p] + static_cast<double>(launching.size()) * drone.launch_prep_time_sl;
            continue;
        }

        // Customer or Depot: truck service (if any) and drone recovery (if any).
        // SR rule, DECISION D1 (Option B), see the header:
        //   l_p = max( t_p + Se_p^T + r_p * SR , max_u (a_u + SR) ).
        double service_complete = arrival[p];
        const auto customer_it = customers.find(v);
        if (customer_it != customers.end() && invalid_customer_entries.count(v) == 0) {
            service_complete = std::max(service_complete, arrival[p] + customer_it->second.truck_service_time);
        }
        std::size_t recoveries_at_p = 0;       // r_p
        double latest_drone_complete = arrival[p]; // max_u (a_u + SR); t_p is a harmless floor

        for (std::size_t i : trips_recovering_at[p]) {
            const DroneTrip& trip = solution.drone_trips[i];
            if (!has_customer_data[i]) continue; // flagged later
            const auto served_it = customers.find(trip.service_customer);
            const double theta = served_it->second.generalized_flight_time_theta;
            const double se2 = served_it->second.drone_service_time;

            detail::TripIntermediate& im = intermediate[i];
            im.outbound_flight_time_raw = network.drone_time(trip.launch_node, trip.service_customer);
            im.customer_arrival_time = im.launch_time + im.outbound_flight_time_raw;
            im.customer_departure_time = im.customer_arrival_time + se2;
            im.return_flight_time_raw = network.drone_time(trip.service_customer, trip.recovery_node);
            im.recovery_arrival_time = im.customer_departure_time + im.return_flight_time_raw;
            im.theta_used = theta;
            im.computed = true;

            ++recoveries_at_p;
            latest_drone_complete = std::max(latest_drone_complete, im.recovery_arrival_time + drone.recovery_time_sr);
        }

        const double truck_side_complete =
            service_complete + static_cast<double>(recoveries_at_p) * drone.recovery_time_sr; // t_p + Se_p^T + r_p*SR
        departure[p] = std::max(truck_side_complete, latest_drone_complete);
    }

    // --- Per-trip feasibility assembly -----------------------------------
    result.drone_trip_timings.resize(trip_count);
    for (std::size_t i = 0; i < trip_count; ++i) {
        const DroneTrip& trip = solution.drone_trips[i];
        DroneTripTiming& t = result.drone_trip_timings[i];
        t.structurally_valid = structurally_valid[i];
        t.on_route = on_route[i];
        t.has_customer_data = has_customer_data[i];
        t.drone_id_valid = trip.drone_id >= 0 && trip.drone_id < drone.num_drones;

        if (!t.structurally_valid) {
            t.violation_reason = "drone trip has an invalid launch/customer/recovery node id or role";
        } else if (!t.on_route) {
            t.violation_reason = "drone trip's launch and/or recovery node is not reachable in truck-route order";
        } else if (!t.has_customer_data) {
            t.violation_reason = "no CustomerData found for the trip's service customer";
        } else if (!intermediate[i].computed) {
            t.violation_reason = "drone trip timing could not be computed";
        } else {
            const detail::TripIntermediate& im = intermediate[i];
            t.launch_time = im.launch_time;
            t.outbound_flight_time_raw = im.outbound_flight_time_raw;
            t.customer_arrival_time = im.customer_arrival_time;
            t.customer_departure_time = im.customer_departure_time;
            t.return_flight_time_raw = im.return_flight_time_raw;
            t.recovery_arrival_time = im.recovery_arrival_time;
            t.theta_used = im.theta_used;
            t.truck_arrival_at_recovery = arrival[*recovery_pos[i]];
            t.recovery_complete_time = std::max(t.truck_arrival_at_recovery, t.recovery_arrival_time) + drone.recovery_time_sr;

            // (36) and (37), literally, with l^_{i,u} = launch_time and
            // l^_{j,u} = customer_departure_time.
            const double loaded = t.customer_departure_time - t.launch_time; // l^_{j,u} - l^_{i,u}
            t.endurance_value_36 = (t.truck_arrival_at_recovery - t.customer_departure_time) + loaded * (1.0 + t.theta_used);
            t.endurance_value_37 = loaded * (1.0 + t.theta_used) + t.return_flight_time_raw;
            t.generalized_flight_time = std::max(t.endurance_value_36, t.endurance_value_37);
            t.endurance_feasible = t.endurance_value_36 <= drone.endurance_seconds + kTimeEpsilon &&
                                   t.endurance_value_37 <= drone.endurance_seconds + kTimeEpsilon;

            if (!t.drone_id_valid) {
                t.violation_reason = "drone trip's drone id " + std::to_string(trip.drone_id) +
                                     " is not a drone of U = {0.." + std::to_string(drone.num_drones - 1) + "}";
            } else if (!t.endurance_feasible) {
                t.violation_reason = "drone trip violates battery endurance (constraints (36)/(37))";
            }
            t.feasible = t.drone_id_valid && t.endurance_feasible;
        }

        if (!t.feasible) {
            reasons.push_back("trip " + std::to_string(i) + ": " + t.violation_reason);
        }
    }

    // --- Physical drones: (28) per-drone sequencing and |U| --------------
    {
        // Trips that are placed on the route (valid roles, launch before recovery).
        std::vector<std::size_t> placed;
        for (std::size_t i = 0; i < trip_count; ++i) {
            if (structurally_valid[i] && on_route[i]) placed.push_back(i);
        }

        // (28): for each drone, trips in route order must not overlap.
        std::unordered_map<int, std::vector<std::size_t>> by_drone;
        for (std::size_t i : placed) {
            if (result.drone_trip_timings[i].drone_id_valid) by_drone[solution.drone_trips[i].drone_id].push_back(i);
        }
        std::vector<int> drone_ids;
        for (const auto& [u, trips] : by_drone) { (void)trips; drone_ids.push_back(u); }
        std::sort(drone_ids.begin(), drone_ids.end());
        for (int u : drone_ids) {
            auto& trips = by_drone[u];
            std::sort(trips.begin(), trips.end(), [&](std::size_t a, std::size_t b) {
                if (*launch_pos[a] != *launch_pos[b]) return *launch_pos[a] < *launch_pos[b];
                return a < b;
            });
            for (std::size_t m = 1; m < trips.size(); ++m) {
                const std::size_t prev = trips[m - 1];
                const std::size_t next = trips[m];
                if (*launch_pos[next] <= *recovery_pos[prev]) {
                    reasons.push_back("drone " + std::to_string(u) + " is relaunched (trip " + std::to_string(next) +
                                      ", route position " + std::to_string(*launch_pos[next]) +
                                      ") before its previous trip (trip " + std::to_string(prev) +
                                      ") is recovered at route position " + std::to_string(*recovery_pos[prev]) +
                                      " (constraint (28))");
                }
            }
        }

        // |U|: largest number of trips simultaneously in flight. A trip
        // occupies its drone over the route-position interval
        // [launch position, recovery position]; a launch position is never a
        // recovery position (FLP vs truck node), so there are no ties.
        int max_active = 0;
        for (std::size_t a : placed) {
            int active = 0;
            for (std::size_t b : placed) {
                if (*launch_pos[b] <= *launch_pos[a] && *launch_pos[a] <= *recovery_pos[b]) ++active;
            }
            max_active = std::max(max_active, active);
        }
        result.max_simultaneous_drones = max_active;
        if (max_active > drone.num_drones) {
            reasons.push_back(std::to_string(max_active) + " drones are simultaneously in flight but |U| = " +
                              std::to_string(drone.num_drones));
        }
    }

    // --- Customer service resolution --------------------------------------
    result.customer_records.reserve(customers.size());
    for (const NodeId customer_id : keys) { // [P2] ascending id order, deterministic
        if (invalid_customer_entries.count(customer_id) != 0) continue; // reported once above
        CustomerServiceRecord record;
        record.customer_id = customer_id;

        const std::optional<std::size_t> truck_pos = detail::find_route_position(route, customer_id);
        const bool truck_served = truck_pos.has_value();

        std::vector<std::size_t> serving_trips;
        for (std::size_t i = 0; i < trip_count; ++i) {
            if (solution.drone_trips[i].service_customer == customer_id) serving_trips.push_back(i);
        }

        if (truck_served && serving_trips.empty()) {
            record.served_by_truck = true;
            record.service_arrival_time = arrival[*truck_pos];
        } else if (!truck_served && serving_trips.size() == 1 && result.drone_trip_timings[serving_trips[0]].feasible) {
            record.served_by_drone = true;
            record.service_arrival_time = result.drone_trip_timings[serving_trips[0]].customer_arrival_time;
        } else {
            record.feasible = false;
            if (truck_served && !serving_trips.empty()) {
                record.violation_reason = "customer is served by both the truck and at least one drone trip";
            } else if (!truck_served && serving_trips.empty()) {
                record.violation_reason = "customer is not served by the truck or any drone trip";
            } else if (serving_trips.size() > 1) {
                record.violation_reason = "customer is served by more than one drone trip";
            } else {
                record.violation_reason = "customer's sole serving drone trip is itself infeasible";
                record.service_arrival_time = result.drone_trip_timings[serving_trips[0]].customer_arrival_time;
            }
        }

        if (!record.feasible) {
            reasons.push_back("customer " + std::to_string(customer_id) + ": " + record.violation_reason);
        }
        result.customer_records.push_back(record);
    }

    // --- Synchronization records (recovery rendezvous only) ---------------
    for (std::size_t i = 0; i < trip_count; ++i) {
        if (!result.drone_trip_timings[i].feasible) continue;
        const std::size_t pos = *recovery_pos[i];
        SynchronizationRecord sync;
        sync.recovery_node = solution.drone_trips[i].recovery_node;
        sync.truck_arrival = arrival[pos];
        sync.drone_arrival = result.drone_trip_timings[i].recovery_arrival_time;
        sync.truck_waited = sync.truck_arrival < sync.drone_arrival;
        sync.drone_waited = sync.drone_arrival < sync.truck_arrival;
        sync.wait_time = std::fabs(sync.truck_arrival - sync.drone_arrival);
        result.synchronization_records.push_back(sync);
    }

    // --- Objective and overall feasibility ---------------------------------
    // [P2] Summed in ascending customer-id order (customer_records order).
    double objective = 0.0;
    for (const auto& record : result.customer_records) objective += record.service_arrival_time;
    result.objective_value = objective;
    result.feasible = reasons.empty();
    return result;
}

} // namespace tdr::timing
