#pragma once

#include "model/node.hpp"

namespace tdr::model {

// Customer-specific data, keyed back to a Node of type
// NodeType::Customer via node_id. Kept separate from Node itself so that
// Node stays a uniform, minimal shape across all node types (see
// node.hpp).
//
// [PAPER] Table 2 defines these per-customer (subscript i / j), not as
// single global constants:
//   - Se1_i : service time when i is served by the truck (constraint 18)
//   - Se2_i : service time when i is served by a drone (constraint 22)
//   - theta_j : load-dependent generalized-flight-time coefficient used
//     in eq. 35 when j is drone-served
// so this struct keeps them per-customer to match that indexing, even
// though Paper 1's main experiments (Sec 6.1) happen to set a single
// constant service time and theta=1 for every customer. Whether Se1 and
// Se2 are truly always equal, or theta is truly always uniform, is a
// property of a particular experiment's *configuration* (Phase D), not
// of this data structure -- nothing here forces them to match.
struct CustomerData {
    NodeId node_id = kInvalidNodeId;

    double truck_service_time = 0.0;             // Se1_i [PAPER, Table 2 + constraint 18]
    double drone_service_time = 0.0;              // Se2_i [PAPER, Table 2 + constraint 22]
    double generalized_flight_time_theta = 0.0;   // theta_j [PAPER, Table 2, eq. 35]
};

} // namespace tdr::model
