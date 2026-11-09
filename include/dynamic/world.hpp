#pragma once

// Dynamic extension (DM3): an immutable world snapshot (DM2 §3). A World owns everything the evaluator and SearchContext reference.
//   * NOT copyable and NOT movable: SearchContext holds references into it (move_types.hpp:38-43) and SpatialNetwork owns a copy of the
//     registry; a moved/copied World would leave contexts dangling. Worlds live behind shared_ptr<const World>.
//   * A "tentative world" is a NEW World built from this one (with_customer); the original is never modified.
//   * The FLP ownership map and arc index are immutable and shared between worlds that do not change them (with_customer shares them).

#include <chrono>
#include <memory>
#include <unordered_map>

#include "dynamic/request.hpp"
#include "flp/flp_arc_index.hpp"
#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"
#include "model/solution.hpp"
#include "search/move_types.hpp"
#include "timing/timing_engine.hpp"

namespace tdr::dynamic {

class World {
public:
    using NodeId = tdr::model::NodeId;
    using CustomerMap = std::unordered_map<tdr::model::NodeId, tdr::model::CustomerData>;

    static std::shared_ptr<const World> create(tdr::model::NodeRegistry nodes, CustomerMap customers, tdr::timing::FlpDataMap flp,
                                               tdr::model::TruckParameters truck, tdr::model::DroneParameters drone, int adjacency_window = 3) {
        return std::shared_ptr<const World>(new World(std::move(nodes), std::move(customers),
                                                      std::make_shared<const tdr::timing::FlpDataMap>(std::move(flp)), nullptr, truck, drone, adjacency_window));
    }

    World(const World&) = delete;
    World(World&&) = delete;
    World& operator=(const World&) = delete;
    World& operator=(World&&) = delete;

    const tdr::graph::SpatialNetwork& network() const { return network_; }
    const tdr::model::NodeRegistry& nodes() const { return network_.nodes(); }
    const CustomerMap& customers() const { return customers_; }
    const tdr::timing::FlpDataMap& flp_data() const { return *flp_; }
    const tdr::flp::FlpArcIndex& flp_index() const { return *idx_; }
    const tdr::model::TruckParameters& truck() const { return truck_; }
    const tdr::model::DroneParameters& drone() const { return drone_; }
    int adjacency_window() const { return adjacency_window_; }

    // View for the static operators / VNS. Valid only while this World is alive (references into it).
    tdr::search::SearchContext context() const {
        return tdr::search::SearchContext{network_, customers_, *flp_, *idx_, adjacency_window_};
    }

    // The unchanged evaluator against exactly this world.
    tdr::timing::SolutionEvaluationResult evaluate(const tdr::model::Solution& s) const {
        return tdr::timing::evaluate_solution(s, network_, customers_, *flp_);
    }

    // Tentative world with one more customer. nullptr if the id already exists (ids are content-addressed; a collision is a bug).
    std::shared_ptr<const World> with_customer(tdr::model::NodeId id, tdr::model::Point2D location, const tdr::model::CustomerData& data) const {
        if (network_.nodes().contains(id) || customers_.count(id)) return nullptr;
        tdr::model::NodeRegistry nodes = network_.nodes(); // one copy of the registry (SpatialNetwork stores it by value)
        nodes.add(tdr::model::Node{id, tdr::model::NodeType::Customer, location});
        CustomerMap customers = customers_;
        tdr::model::CustomerData cd = data;
        cd.node_id = id;
        customers.emplace(id, cd);
        return std::shared_ptr<const World>(new World(std::move(nodes), std::move(customers), flp_, idx_, truck_, drone_, adjacency_window_));
    }

    std::shared_ptr<const World> with_request(NodeId id, const Request& r) const {
        tdr::model::CustomerData cd;
        cd.truck_service_time = r.se1;
        cd.drone_service_time = r.se2;
        cd.generalized_flight_time_theta = r.theta;
        return with_customer(id, r.location, cd);
    }

    // Measurement helper: a world that rebuilds EVERYTHING (registry, customers, FLP map copy, arc index), i.e. the cost of a tentative
    // world without the sharing optimisation.
    std::shared_ptr<const World> deep_rebuild() const {
        return std::shared_ptr<const World>(new World(tdr::model::NodeRegistry(network_.nodes()), CustomerMap(customers_),
                                                      std::make_shared<const tdr::timing::FlpDataMap>(*flp_), nullptr, truck_, drone_, adjacency_window_));
    }

private:
    World(tdr::model::NodeRegistry nodes, CustomerMap customers, std::shared_ptr<const tdr::timing::FlpDataMap> flp,
          std::shared_ptr<const tdr::flp::FlpArcIndex> idx, tdr::model::TruckParameters truck, tdr::model::DroneParameters drone, int aw)
        : network_(std::move(nodes), truck, drone),
          customers_(std::move(customers)),
          flp_(std::move(flp)),
          idx_(idx ? std::move(idx) : std::make_shared<const tdr::flp::FlpArcIndex>(*flp_)),
          truck_(truck), drone_(drone), adjacency_window_(aw) {}

    tdr::graph::SpatialNetwork network_;
    CustomerMap customers_;
    std::shared_ptr<const tdr::timing::FlpDataMap> flp_;
    std::shared_ptr<const tdr::flp::FlpArcIndex> idx_;
    tdr::model::TruckParameters truck_;
    tdr::model::DroneParameters drone_;
    int adjacency_window_;
};

} // namespace tdr::dynamic
