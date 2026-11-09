#pragma once

// Deterministic static-instance generation (M1, 9 Nov 2026).
//
// [PAPER, Sec 6.1] The synthetic instances have 10 / 20 / 50 customers
// "evenly distributed in a 60 x 60 km distribution network", four datasets
// per size. The paper publishes no coordinates and no generator, so the
// distribution is taken to be i.i.d. uniform on the square
// ([ASSUMPTION], DECISIONS.md "M1"); exact numerical reproduction of the
// paper's instances is therefore impossible by construction.
//
// Layout of the generated instance: depot id 0, customers 1..n. FLP ids are
// assigned later by the FLP generator starting at n + 1 (unchanged from
// Phase F). The same (config, dataset_index, base_seed) always yields the
// identical instance on every platform (util/rng.hpp).

#include <string>
#include <unordered_map>
#include <vector>

#include "config/static_config.hpp"
#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"
#include "util/rng.hpp"

namespace tdr::instance {

struct StaticInstance {
    tdr::model::NodeRegistry nodes;                  // depot + customers (no FLPs)
    tdr::model::NodeId depot = 0;
    std::vector<tdr::model::NodeId> customers;       // 1..n, ascending
    std::unordered_map<tdr::model::NodeId, tdr::model::CustomerData> customer_data;
    tdr::model::TruckParameters truck;
    tdr::model::DroneParameters drone;
    std::uint64_t instance_seed = 0;                 // the derived seed actually used

    tdr::graph::SpatialNetwork make_network() const {
        return tdr::graph::SpatialNetwork(nodes, truck, drone);
    }
};

struct InstanceGenerationResult {
    StaticInstance instance;
    bool ok = true;
    std::vector<std::string> errors; // config validation problems when ok == false
};

inline tdr::model::Point2D depot_location(const tdr::config::InstanceConfig& c) {
    switch (c.depot_placement) {
        case tdr::config::DepotPlacement::Centre: return {c.area_side_m / 2.0, c.area_side_m / 2.0};
        case tdr::config::DepotPlacement::Corner: return {0.0, 0.0};
        case tdr::config::DepotPlacement::Explicit: return {c.depot_x_m, c.depot_y_m};
    }
    return {c.area_side_m / 2.0, c.area_side_m / 2.0};
}

inline InstanceGenerationResult generate_static_instance(const tdr::config::StaticConfig& config) {
    using namespace tdr::model;
    InstanceGenerationResult out;
    out.errors = tdr::config::validate(config);
    if (!out.errors.empty()) { out.ok = false; return out; }

    StaticInstance& inst = out.instance;
    inst.instance_seed = tdr::util::derive_seed(config.seed.base_seed, tdr::util::SeedStream::Instance,
                                                static_cast<std::uint64_t>(config.instance.dataset_index));
    tdr::util::Rng rng(inst.instance_seed);

    inst.depot = 0;
    inst.nodes.add(Node{0, NodeType::Depot, depot_location(config.instance)});

    const double side = config.instance.area_side_m;
    for (int i = 1; i <= config.instance.num_customers; ++i) {
        // Draw order is fixed: x then y, customers in ascending id.
        const double x = rng.uniform_real(0.0, side);
        const double y = rng.uniform_real(0.0, side);
        inst.nodes.add(Node{i, NodeType::Customer, Point2D{x, y}});
        inst.customers.push_back(i);
        CustomerData cd;
        cd.node_id = i;
        cd.truck_service_time = config.instance.truck_service_time_s;
        cd.drone_service_time = config.instance.drone_service_time_s;
        cd.generalized_flight_time_theta = config.instance.theta;
        inst.customer_data[i] = cd;
    }

    inst.truck.speed = config.vehicle.truck_speed_mps;
    inst.drone.speed = config.vehicle.drone_speed_mps;
    inst.drone.endurance_seconds = config.vehicle.endurance_s;
    inst.drone.launch_prep_time_sl = config.vehicle.launch_prep_time_s;
    inst.drone.recovery_time_sr = config.vehicle.recovery_time_s;
    inst.drone.num_drones = config.vehicle.num_drones;
    return out;
}

} // namespace tdr::instance
