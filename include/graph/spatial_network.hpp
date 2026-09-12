#pragma once

#include <vector>

#include "graph/distance.hpp"
#include "graph/travel_time.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"

namespace tdr::graph {

// A dense square matrix indexed [row][col]. Structurally identical to
// tdr::tsp::DistanceMatrix (both are plain vector<vector<double>>), so a
// matrix built here can be handed directly to the tsp/ module without
// any adapter -- deliberately kept as an independent alias rather than
// including tsp/tsp_types.hpp, since graph/ is lower-level
// infrastructure and should not depend on a solver module.
using DistanceMatrix = std::vector<std::vector<double>>;

// The static 2D spatial network: a fixed set of nodes (depot, customers,
// flexible launch points -- whatever the caller has registered) plus the
// truck's and drones' speed parameters, providing distance and
// travel-time queries between any two node ids.
//
// This is infrastructure only: it does not generate nodes (Phase D),
// does not know about flexible-launch-point placement rules (Phase D/E),
// and does not evaluate routes, feasibility, or endurance (Phase E/F).
// It answers exactly one question, in two units: "how far / how long
// between node A and node B", using the Euclidean distance model from
// Sec 2.1 and the constant-speed travel-time model from Assumption 2.
//
// [ASSUMPTION: engineering choice, not a paper detail] The NodeRegistry
// is stored by value (a copy), not by reference. A reference would save
// a copy but creates a dangling-reference risk if a SpatialNetwork ever
// outlives the registry it was built from -- for instance sizes here
// (at most a few thousand flexible launch points, per Table 4) a copy is
// cheap enough that the extra safety is worth it.
class SpatialNetwork {
public:
    SpatialNetwork(tdr::model::NodeRegistry nodes,
                   tdr::model::TruckParameters truck,
                   tdr::model::DroneParameters drone)
        : nodes_(std::move(nodes)), truck_(truck), drone_(drone) {}

    // [PAPER] Sec 2.1: straight-line Euclidean distance between two nodes.
    double distance(tdr::model::NodeId a, tdr::model::NodeId b) const {
        return euclidean_distance(nodes_.at(a), nodes_.at(b));
    }

    // [PAPER] Table 2: d_ij.
    double truck_time(tdr::model::NodeId a, tdr::model::NodeId b) const {
        return tdr::graph::truck_travel_time(distance(a, b), truck_);
    }

    // [PAPER] Table 2: d'_ij -- raw physical flight time. See
    // travel_time.hpp for why this is deliberately NOT eq. 35's
    // generalized flight time.
    double drone_time(tdr::model::NodeId a, tdr::model::NodeId b) const {
        return tdr::graph::drone_travel_time(distance(a, b), drone_);
    }

    const tdr::model::NodeRegistry& nodes() const { return nodes_; }
    const tdr::model::TruckParameters& truck_parameters() const { return truck_; }
    const tdr::model::DroneParameters& drone_parameters() const { return drone_; }

    // Small utility for later phases (e.g. Phase G's truck-only TSP
    // stage, which needs a dense matrix over just {depot, customers}):
    // build a dense matrix over an explicit, caller-chosen, ordered list
    // of node ids. Row/column i of the result corresponds to ids[i].
    // This does not decide *which* ids belong together -- that policy
    // (e.g. "depot plus all customers") belongs to whichever later phase
    // calls it.
    DistanceMatrix build_distance_matrix(const std::vector<tdr::model::NodeId>& ids) const {
        return build_matrix(ids, [this](tdr::model::NodeId a, tdr::model::NodeId b) { return distance(a, b); });
    }

    DistanceMatrix build_truck_time_matrix(const std::vector<tdr::model::NodeId>& ids) const {
        return build_matrix(ids, [this](tdr::model::NodeId a, tdr::model::NodeId b) { return truck_time(a, b); });
    }

    DistanceMatrix build_drone_time_matrix(const std::vector<tdr::model::NodeId>& ids) const {
        return build_matrix(ids, [this](tdr::model::NodeId a, tdr::model::NodeId b) { return drone_time(a, b); });
    }

private:
    template <typename Fn>
    DistanceMatrix build_matrix(const std::vector<tdr::model::NodeId>& ids, Fn value_for) const {
        const std::size_t n = ids.size();
        DistanceMatrix matrix(n, std::vector<double>(n, 0.0));
        for (std::size_t row = 0; row < n; ++row) {
            for (std::size_t col = 0; col < n; ++col) {
                matrix[row][col] = (row == col) ? 0.0 : value_for(ids[row], ids[col]);
            }
        }
        return matrix;
    }

    tdr::model::NodeRegistry nodes_;
    tdr::model::TruckParameters truck_;
    tdr::model::DroneParameters drone_;
};

} // namespace tdr::graph
