#pragma once

// Centralized timing and feasibility engine for a static truck-and-drone
// Solution (Paper 1's methodology; see paper1-implementation-plan.md
// Sec. 11-12 for the full chronological reconstruction this file
// implements, and DECISIONS.md ("Timing and feasibility engine (Phase
// E)") for the complete [PAPER]/[INFERENCE]/[ASSUMPTION] breakdown of
// every choice below). This is the ONLY place in the repository that
// computes time, feasibility, or the objective value -- no later module
// (initial-solution construction, neighborhoods, VNS) is meant to
// recompute any of this itself.
//
// EXACT TIMING MODEL IMPLEMENTED (written out before coding, per the
// review instructions for this phase):
//
//  Base case: the truck departs the starting depot (route position 0)
//  at time 0. [ASSUMPTION -- the paper never states an absolute start
//  time; 0 is the only sensible reference point, but it is still a
//  choice, not a stated value.]
//
//  Truck arrival propagation (route positions p = 1..n-1):
//    arrival[p] = departure[p-1] + truck_travel_time(route[p-1], route[p])
//  [PAPER, constraint 16, generalized from "customer j" to any node type
//  -- INFERENCE, see DECISIONS.md.]
//
//  Truck departure at position p depends on what happens there:
//   - FlexibleLaunchPoint node, one or more drones launching from it:
//       departure[p] = arrival[p] + SL
//     (one shared SL for however many drones launch there at once.
//     [ASSUMPTION: "parallel launch" reading, already flagged in the
//     Phase 2A plan Sec. 11].) A flex point can never also host truck
//     service or a recovery -- Phase B's node-role rules make that
//     structurally impossible (launch points are never Customer/Depot
//     typed), so no combination logic is needed here.
//   - Customer/Depot node:
//       service_complete = arrival[p] + Se1 (Se1 = 0 if this node has no
//         CustomerData, i.e. it is the depot) [PAPER, Table 2 +
//         constraint 18]
//       recovery_complete = max(arrival[p], drone_arrival) + SR, for
//         each drone recovering here, where drone_arrival is that
//         drone's own physical arrival at this node (computed below).
//         If more than one drone recovers at the same node on the same
//         visit, ALL of them share one SR window keyed off the LATEST
//         drone/truck arrival among them. [ASSUMPTION: extends the same
//         "parallel/simultaneous ops" reading already used for launch to
//         recovery, for consistency -- the paper does not discuss
//         multi-drone recovery at one node at all.]
//       departure[p] = max(service_complete, recovery_complete over all
//         recovering drones, or arrival[p] if none of the above apply)
//     [PAPER, constraint 18, only for the qualitative fact that departure
//     is gated on every op at that node finishing (service AND every
//     launch/recovery there). RECLASSIFIED FROM [PAPER] TO [INFERENCE]
//     after the Phase E audit (see DECISIONS.md, "Phase E audit"): the
//     specific claim that truck service (Se1) and a drone recovery (SR)
//     at the SAME node run CONCURRENTLY -- i.e. departure = max(Se1
//     completion, SR completion), not Se1 completion + SR -- is not a
//     verified literal equation (constraint 18's exact algebraic form is
//     an OCR-reconstructed, unverified transcription per
//     paper1-understanding.md Sec 2.4). It is the standard MILP-scheduling
//     reading of "cannot depart until X AND Y are both done" (two
//     independent >= lower-bound constraints on one departure variable,
//     which a minimization objective forces down to their max), and is
//     retained as the implemented behavior for that reason, but it is an
//     INFERENCE, not a directly-quotable paper rule. See test_timing.cpp
//     Block 9 for the hand-worked regression that specifically exercises
//     this concurrency (drone recovering at a truck-served customer with
//     nonzero Se1, with the drone arriving before/during/after the
//     service window).]
//
//  A single drone trip (i = launch_node, j = service_customer,
//  k = recovery_node):
//    launch_time            = arrival[pos(i)] + SL
//    outbound_flight_raw    = drone_travel_time(i, j)      [Phase C, raw]
//    customer_arrival       = launch_time + outbound_flight_raw
//                              [PAPER, constraint 17]
//    customer_departure     = customer_arrival + Se2(j)     [PAPER, Table 2
//                              + constraint 22]
//    return_flight_raw      = drone_travel_time(j, k)       [Phase C, raw]
//    recovery_arrival       = customer_departure + return_flight_raw
//                              [INFERENCE -- same additive logic as the
//                              outbound leg, not independently spelled
//                              out for the return leg in the source text]
//
//  REAL elapsed time NEVER uses the load-dependent generalized flight
//  time (eq. 35) -- customer_arrival above is always the RAW physical
//  arrival instant, because that is what "when did the customer's
//  waiting actually end" means. Eq. 35's generalized flight time is
//  used ONLY to check battery endurance, computed separately as:
//    generalized_flight_time = outbound_flight_raw * (1 + theta_j)
//                               + return_flight_raw
//  where theta_j = CustomerData::generalized_flight_time_theta for the
//  served customer j. [INFERENCE: theta is applied to the outbound
//  ("loaded") leg only, not the return ("empty") leg or the whole trip
//  -- see DECISIONS.md for why. This is one of the highest-impact
//  assumptions in this file; an alternative reading (theta over the
//  whole round trip) would change endurance feasibility at the margin.]
//  A trip is endurance-feasible iff generalized_flight_time <= e (the
//  drone's battery endurance). SL, SR, and Se2 (all ground time) never
//  count toward endurance. [INFERENCE, per the plan's own reasoning: a
//  drone that hasn't launched yet, or has already landed, or is sitting
//  at the customer delivering, is not consuming flight endurance.]
//  Likewise, any synchronization WAIT at recovery (one vehicle arriving
//  before the other) never counts toward endurance -- only the two
//  flight legs do; a landed-but-waiting drone is not still "flying".
//
//  Synchronization at recovery (Assumption 7, "first to arrive waits
//  for the other"): recovery_complete above already implements this via
//  max(truck_arrival, drone_arrival) + SR. [INFERENCE: formalizing the
//  prose "first arrives, waits" rule as max() -- never written as an
//  equation in the recoverable source text.]
//
//  A drone trip additionally requires, purely from route/temporal
//  structure (not from a numbered constraint):
//    - launch_node and recovery_node must each actually appear in the
//      truck route (constraints 5-6: only a traversed arc's launch
//      point is usable; recovery must be at a truck-visited node)
//    - the launch_node's route position must be strictly BEFORE the
//      recovery_node's route position, since the truck visits route
//      positions in increasing time order and never revisits an
//      earlier position -- a recovery scheduled at an already-passed
//      position is temporally impossible.
//    [INFERENCE: not a numbered constraint, but a direct, unavoidable
//    consequence of the route being a single ordered path traversed
//    once, forward, over increasing time.]
//
//  Customer waiting-time objective [PAPER, Sec 2.3/eq. 1 as corrected in
//  paper1-implementation-plan.md Sec. 12]: for EVERY customer, the
//  objective-feeding quantity is the arrival time of whichever vehicle
//  actually serves that customer -- arrival[pos] if truck-served,
//  customer_arrival_time (the drone's raw physical arrival at the
//  customer) if drone-served. The total objective is the SUM of this
//  quantity over all customers. It is explicitly NOT drone recovery
//  time, NOT total route duration, NOT total vehicle waiting, and NOT
//  makespan.
//
//  Each customer must be served exactly once, by exactly one vehicle
//  [PAPER, constraint 10]; violations (served by neither, by both, or
//  by more than one drone trip) make the solution infeasible and are
//  reported with a reason, but the (best-effort) objective value is
//  still computed for debugging.

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/node.hpp"
#include "model/solution.hpp"
#include "model/truck_route.hpp"

namespace tdr::timing {

// Small floating-point tolerance for endurance/time comparisons at the
// boundary (e.g. a trip whose generalized flight time equals e exactly
// should not be rejected due to floating-point rounding).
inline constexpr double kTimeEpsilon = 1e-9;

// Timing for one drone trip, plus its own feasibility verdict.
struct DroneTripTiming {
    bool structurally_valid = false; // Phase B's has_distinct_endpoints() && has_valid_node_roles()
    bool on_route = false;           // launch and recovery nodes both found on the route, launch strictly before recovery
    bool has_customer_data = false;  // CustomerData exists for the served customer

    double launch_time = 0.0;             // truck arrival at launch node + SL
    double outbound_flight_time_raw = 0.0; // drone_time(launch, customer) -- physical, no theta
    double customer_arrival_time = 0.0;    // launch_time + outbound_flight_time_raw -- THE objective-feeding time if drone-served
    double customer_departure_time = 0.0;  // customer_arrival_time + Se2
    double return_flight_time_raw = 0.0;   // drone_time(customer, recovery) -- physical, no theta
    double recovery_arrival_time = 0.0;    // customer_departure_time + return_flight_time_raw (drone's own physical landing instant)
    double recovery_complete_time = 0.0;   // max(truck_arrival_at_recovery, recovery_arrival_time) + SR

    double theta_used = 0.0;              // the customer's generalized_flight_time_theta, applied to the outbound leg only
    double generalized_flight_time = 0.0; // outbound_flight_time_raw * (1 + theta_used) + return_flight_time_raw
    bool endurance_feasible = false;      // generalized_flight_time <= drone endurance (+ epsilon)

    bool feasible = false;
    std::string violation_reason; // empty iff feasible
};

// The truck's arrival/departure time at every position in the route
// (parallel arrays, same length as the input TruckRoute). Indexed by
// route position, not by node id, so a route that happens to repeat a
// node id is still represented unambiguously.
struct TruckTimingResult {
    std::vector<double> arrival_time;
    std::vector<double> departure_time;
};

// One customer's resolution: which vehicle served it (if any, and if
// unambiguous), and the resulting objective-feeding service time.
struct CustomerServiceRecord {
    tdr::model::NodeId customer_id = tdr::model::kInvalidNodeId;
    bool served_by_truck = false;
    bool served_by_drone = false;
    double service_arrival_time = 0.0;
    bool feasible = true;
    std::string violation_reason;
};

// Records how much waiting happened at a recovery rendezvous, for
// inspection/debugging (Assumption 7: whoever arrives first waits).
struct SynchronizationRecord {
    tdr::model::NodeId recovery_node = tdr::model::kInvalidNodeId;
    double truck_arrival = 0.0;
    double drone_arrival = 0.0; // the drone's own physical arrival, pre-SR
    bool truck_waited = false;
    bool drone_waited = false;
    double wait_time = 0.0;
};

// The complete evaluation of one static Solution.
struct SolutionEvaluationResult {
    bool feasible = false;
    // Sum of every customer's service_arrival_time. Only authoritative
    // when feasible == true -- for an infeasible solution this is still
    // computed best-effort (never penalized/inflated), purely so a
    // caller inspecting a broken solution can see what its partial
    // objective would be, per the plan's "exclude, don't penalize"
    // convention (paper1-implementation-plan.md Sec. 12).
    double objective_value = 0.0;

    TruckTimingResult truck_timing;
    std::vector<DroneTripTiming> drone_trip_timings; // parallel to solution.drone_trips
    std::vector<CustomerServiceRecord> customer_records;
    std::vector<SynchronizationRecord> synchronization_records;
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
//
// [INFERENCE] Why this overload exists: the start and end depot are the
// same NodeId, and a round-trip truck route therefore contains that id
// twice (position 0 and position n-1). A drone trip's recovery_node is
// frequently the end depot. A plain first-occurrence search for that id
// would always find position 0 -- the START depot -- which is never a
// valid recovery position for a trip whose launch happens later in the
// route, and would incorrectly make on_route() false for every trip that
// recovers at the ending depot. Since recovery must occur at a route
// position strictly after the trip's launch position (route.md /
// timing_engine header comments: a truck traverses its route once,
// forward, in increasing time), searching for the recovery node starting
// from the launch position -- rather than from the beginning of the route
// -- is the physically correct search and also the fix for the
// repeated-depot-id ambiguity.
inline std::optional<std::size_t> find_route_position_from(
    const tdr::model::TruckRoute& route, tdr::model::NodeId id, std::size_t from_index) {
    for (std::size_t i = from_index; i < route.size(); ++i) {
        if (route[i] == id) return i;
    }
    return std::nullopt;
}

// Intermediate per-trip values computed during the single forward pass,
// kept only for trips whose contribution could actually be computed
// (structurally valid, on-route, with customer data available).
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

// Evaluates one complete static Solution: truck timing, every drone
// trip's timing and feasibility, per-customer service resolution,
// synchronization records, overall feasibility, and the customer
// waiting-time objective. Pure function -- no mutation, no randomness,
// no optimization/search logic of any kind.
inline SolutionEvaluationResult evaluate_solution(
    const tdr::model::Solution& solution,
    const tdr::graph::SpatialNetwork& network,
    const std::unordered_map<tdr::model::NodeId, tdr::model::CustomerData>& customers) {

    using tdr::model::DroneTrip;
    using tdr::model::NodeId;
    using tdr::model::NodeType;
    using tdr::model::TruckRoute;

    SolutionEvaluationResult result;
    const TruckRoute& route = solution.truck_route;
    std::vector<std::string>& reasons = result.violation_reasons;

    // --- Route-level structural preconditions ---------------------------
    if (route.size() < 2) {
        reasons.push_back("truck route must contain at least a start and an end depot node");
        result.feasible = false;
        return result; // nothing else can be computed without a route
    }
    if (network.nodes().at(route.front()).type != NodeType::Depot) {
        reasons.push_back("truck route does not start at a depot node");
    }
    if (network.nodes().at(route.back()).type != NodeType::Depot) {
        reasons.push_back("truck route does not end at a depot node");
    }

    const std::size_t n = route.size();
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
        // Recovery must occur strictly after launch in route order (see
        // find_route_position_from's comment for why this is a plain
        // forward search from just past the launch position, not a
        // whole-route first-occurrence search): this is also what makes
        // recovery at the ENDING depot resolve correctly rather than
        // matching the identical NodeId at the STARTING depot.
        recovery_pos[i] = launch_pos[i].has_value()
                               ? detail::find_route_position_from(route, trip.recovery_node, *launch_pos[i] + 1)
                               : std::nullopt;
        on_route[i] = launch_pos[i].has_value() && recovery_pos[i].has_value() && *launch_pos[i] < *recovery_pos[i];
        has_customer_data[i] = customers.find(trip.service_customer) != customers.end();
    }

    // Trips that launch from a given route position / recover at a given
    // route position, precomputed so the forward pass can look them up
    // by position without a linear scan each time.
    std::vector<std::vector<std::size_t>> trips_launching_at(n);
    std::vector<std::vector<std::size_t>> trips_recovering_at(n);
    for (std::size_t i = 0; i < trip_count; ++i) {
        if (on_route[i] && structurally_valid[i]) {
            trips_launching_at[*launch_pos[i]].push_back(i);
            trips_recovering_at[*recovery_pos[i]].push_back(i);
        }
    }

    const auto& truck = network.truck_parameters();
    const auto& drone = network.drone_parameters();

    // --- Single forward pass over route positions -----------------------
    for (std::size_t p = 0; p < n; ++p) {
        const NodeId v = route[p];

        if (p == 0) {
            arrival[p] = 0.0; // [ASSUMPTION] truck departs the start depot at time 0
        } else {
            arrival[p] = departure[p - 1] + network.truck_time(route[p - 1], v);
        }

        const NodeType type = network.nodes().at(v).type;

        if (type == NodeType::FlexibleLaunchPoint) {
            const auto& launching = trips_launching_at[p];
            if (!launching.empty()) {
                departure[p] = arrival[p] + drone.launch_prep_time_sl;
                for (std::size_t i : launching) {
                    intermediate[i].launch_time = departure[p];
                }
            } else {
                departure[p] = arrival[p];
            }
            continue;
        }

        // Customer or Depot: truck service (if any) and drone recovery (if any).
        double departure_candidate = arrival[p];
        const auto customer_it = customers.find(v);
        if (customer_it != customers.end()) {
            departure_candidate = std::max(departure_candidate, arrival[p] + customer_it->second.truck_service_time);
        }

        for (std::size_t i : trips_recovering_at[p]) {
            const DroneTrip& trip = solution.drone_trips[i];
            if (!has_customer_data[i]) continue; // can't compute this trip's timing without theta/Se2; flagged later
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

            const double recovery_complete = std::max(arrival[p], im.recovery_arrival_time) + drone.recovery_time_sr;
            departure_candidate = std::max(departure_candidate, recovery_complete);
        }

        departure[p] = departure_candidate;
    }

    // --- Per-trip feasibility assembly -----------------------------------
    result.drone_trip_timings.resize(trip_count);
    for (std::size_t i = 0; i < trip_count; ++i) {
        const DroneTrip& trip = solution.drone_trips[i];
        DroneTripTiming& t = result.drone_trip_timings[i];
        t.structurally_valid = structurally_valid[i];
        t.on_route = on_route[i];
        t.has_customer_data = has_customer_data[i];

        if (!t.structurally_valid) {
            t.violation_reason = "drone trip has an invalid launch/customer/recovery node id or role";
        } else if (!t.on_route) {
            t.violation_reason = "drone trip's launch and/or recovery node is not reachable in truck-route order";
        } else if (!t.has_customer_data) {
            t.violation_reason = "no CustomerData found for the trip's service customer";
        } else if (!intermediate[i].computed) {
            // Should not happen given the checks above, but guard anyway.
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
            t.generalized_flight_time = im.outbound_flight_time_raw * (1.0 + im.theta_used) + im.return_flight_time_raw;
            t.endurance_feasible = t.generalized_flight_time <= drone.endurance_seconds + kTimeEpsilon;
            t.recovery_complete_time = std::max(arrival[*recovery_pos[i]], t.recovery_arrival_time) + drone.recovery_time_sr;

            t.feasible = t.endurance_feasible;
            if (!t.feasible) {
                t.violation_reason = "drone trip's generalized flight time exceeds battery endurance";
            }
        }

        if (!t.feasible) {
            reasons.push_back("trip " + std::to_string(i) + ": " + t.violation_reason);
        }
    }

    // --- Customer service resolution --------------------------------------
    result.customer_records.reserve(customers.size());
    for (const auto& [customer_id, customer_data] : customers) {
        (void)customer_data;
        CustomerServiceRecord record;
        record.customer_id = customer_id;

        const std::optional<std::size_t> truck_pos = detail::find_route_position(route, customer_id);
        const bool truck_served = truck_pos.has_value();

        std::vector<std::size_t> serving_trips;
        for (std::size_t i = 0; i < trip_count; ++i) {
            if (solution.drone_trips[i].service_customer == customer_id) {
                serving_trips.push_back(i);
            }
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
    double objective = 0.0;
    for (const auto& record : result.customer_records) {
        objective += record.service_arrival_time;
    }
    result.objective_value = objective;
    result.feasible = reasons.empty();

    return result;
}

} // namespace tdr::timing
