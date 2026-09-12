#pragma once

#include "model/node.hpp"

namespace tdr::model {

inline constexpr int kUnassignedDroneSlot = -1;
inline constexpr int kUnknownRoutePosition = -1;

// A single drone flight: launch -> serve one customer -> recover.
// [PAPER] Sec 2.2 calls this a "single-flight route", a tuple
// s = (i, j, k) with i the launch node (must be in L), j the customer
// served (must be in C), and k the recovery node (must be in C_R,
// k != i, j). This struct is the concrete representation of that tuple.
//
// No timing (arrival/departure times, feasibility, endurance usage) is
// stored here. Per the implementation plan's architecture rule, the
// timing/feasibility engine (Phase E/F) is the only place that computes
// time or feasibility; a DroneTrip only ever records *which* nodes it
// connects, never *when*.
struct DroneTrip {
    // [ASSUMPTION] Nominal per-trip drone identity. The MILP formulation
    // itself does not require persistent drone identity -- only the
    // concurrency cap |U| matters there -- but the Random_Drone_Remove
    // shake neighborhood is phrased as acting on "a drone" (see
    // paper1-implementation-plan.md Sec 15), which implies some notion
    // of "this trip belongs to drone slot N". kUnassignedDroneSlot means
    // no slot has been assigned yet; this is a bookkeeping convenience,
    // not a paper-defined identifier.
    int drone_slot = kUnassignedDroneSlot;

    NodeId launch_node = kInvalidNodeId;       // i -- expected NodeType::FlexibleLaunchPoint
    NodeId service_customer = kInvalidNodeId;  // j -- expected NodeType::Customer
    NodeId recovery_node = kInvalidNodeId;     // k -- expected NodeType::Customer or NodeType::Depot (ending depot)

    // [ASSUMPTION: representation convenience only] Cached index of
    // launch_node / recovery_node within the owning Solution's truck
    // route, so later modules (neighborhoods, the timing engine) do not
    // need to re-search the route on every access. kUnknownRoutePosition
    // means "not populated" -- Phase B does not require these to be kept
    // in sync; whichever later module maintains a Solution is
    // responsible for that.
    int launch_position_in_route = kUnknownRoutePosition;
    int recovery_position_in_route = kUnknownRoutePosition;
};

// Structural well-formedness only: Paper 1 states k != i, j for every
// single-flight route (Sec 2.2). This checks identity distinctness and
// that all three nodes are actually set -- nothing about time,
// endurance, or feasibility (that is Phase E/F's responsibility).
inline bool has_distinct_endpoints(const DroneTrip& trip) {
    return trip.launch_node != kInvalidNodeId &&
           trip.service_customer != kInvalidNodeId &&
           trip.recovery_node != kInvalidNodeId &&
           trip.launch_node != trip.service_customer &&
           trip.launch_node != trip.recovery_node &&
           trip.service_customer != trip.recovery_node;
}

// Structural role check against an actual node registry: does
// launch_node really refer to a flexible launch point, does
// service_customer really refer to a customer, and does recovery_node
// refer to a customer or the depot? This is still purely about which
// *kind* of node each reference points to -- not about whether the
// resulting trip is reachable in time or within battery endurance.
inline bool has_valid_node_roles(const DroneTrip& trip, const NodeRegistry& nodes) {
    if (!nodes.contains(trip.launch_node) ||
        !nodes.contains(trip.service_customer) ||
        !nodes.contains(trip.recovery_node)) {
        return false;
    }
    if (nodes.at(trip.launch_node).type != NodeType::FlexibleLaunchPoint) {
        return false;
    }
    if (nodes.at(trip.service_customer).type != NodeType::Customer) {
        return false;
    }
    const NodeType recovery_type = nodes.at(trip.recovery_node).type;
    return recovery_type == NodeType::Customer || recovery_type == NodeType::Depot;
}

} // namespace tdr::model
