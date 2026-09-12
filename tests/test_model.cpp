#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/flexible_launch_point.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"
#include "model/solution.hpp"
#include "model/truck_route.hpp"

#include <iostream>
#include <string>

using namespace tdr::model;

namespace {

int g_failures = 0;

void check(bool condition, const std::string& description) {
    if (condition) {
        std::cout << "[PASS] " << description << "\n";
    } else {
        std::cout << "[FAIL] " << description << "\n";
        ++g_failures;
    }
}

} // namespace

int main() {
    // 1 & 2. Nodes can represent depot/customer/flex-point, and ids,
    // types, and coordinates are stored and retrieved correctly via
    // NodeRegistry.
    {
        Node depot{/*id=*/0, NodeType::Depot, Point2D{0.0, 0.0}};
        Node customer{/*id=*/1, NodeType::Customer, Point2D{10.0, 20.0}};
        Node flex{/*id=*/2, NodeType::FlexibleLaunchPoint, Point2D{5.0, 5.0}};

        check(depot.type == NodeType::Depot, "Depot node has type Depot");
        check(customer.type == NodeType::Customer, "Customer node has type Customer");
        check(flex.type == NodeType::FlexibleLaunchPoint, "Flex-point node has type FlexibleLaunchPoint");

        NodeRegistry registry;
        check(registry.add(depot), "Registry accepts depot node");
        check(registry.add(customer), "Registry accepts customer node");
        check(registry.add(flex), "Registry accepts flex-point node");
        check(registry.size() == 3, "Registry size reflects three added nodes");

        check(registry.contains(1), "Registry reports customer id present");
        const Node& fetched = registry.at(1);
        check(fetched.id == 1, "Fetched node id matches");
        check(fetched.type == NodeType::Customer, "Fetched node type matches");
        check(fetched.coordinates.x == 10.0 && fetched.coordinates.y == 20.0,
              "Fetched node coordinates match");

        check(!registry.contains(99), "Registry correctly reports unknown id absent");

        // Duplicate id insertion must be rejected, not silently overwritten.
        Node duplicate{/*id=*/1, NodeType::FlexibleLaunchPoint, Point2D{-1.0, -1.0}};
        check(!registry.add(duplicate), "Registry rejects duplicate node id");
        check(registry.at(1).type == NodeType::Customer,
              "Original node at duplicate id is left unchanged after rejected insert");
    }

    // 3. Customer data can be represented correctly.
    {
        CustomerData c;
        c.node_id = 7;
        c.truck_service_time = 30.0;
        c.drone_service_time = 25.0;
        c.generalized_flight_time_theta = 1.0;

        check(c.node_id == 7, "CustomerData stores node_id");
        check(c.truck_service_time == 30.0, "CustomerData stores truck service time (Se1)");
        check(c.drone_service_time == 25.0, "CustomerData stores drone service time (Se2)");
        check(c.generalized_flight_time_theta == 1.0, "CustomerData stores theta");
    }

    // 4. A drone trip can represent launch -> customer -> recovery, and
    // its node roles can be validated against a registry.
    NodeRegistry registry;
    {
        registry.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        registry.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        registry.add(Node{2, NodeType::Customer, Point2D{20.0, 0.0}});
        registry.add(Node{3, NodeType::FlexibleLaunchPoint, Point2D{5.0, 0.0}});
        registry.add(Node{4, NodeType::FlexibleLaunchPoint, Point2D{15.0, 0.0}});

        DroneTrip trip;
        trip.launch_node = 3;
        trip.service_customer = 1;
        trip.recovery_node = 2;

        check(trip.launch_node == 3 && trip.service_customer == 1 && trip.recovery_node == 2,
              "DroneTrip stores launch/customer/recovery node ids");
        check(has_distinct_endpoints(trip), "Well-formed trip has three distinct endpoints");
        check(has_valid_node_roles(trip, registry),
              "Well-formed trip has correct node roles (launch=flex, customer=customer, recovery=customer)");

        DroneTrip recover_at_depot;
        recover_at_depot.launch_node = 4;
        recover_at_depot.service_customer = 2;
        recover_at_depot.recovery_node = 0; // ending depot
        check(has_valid_node_roles(recover_at_depot, registry),
              "Trip recovering at the depot has valid node roles");
    }

    // 5. Multiple drone trips can be represented.
    std::vector<DroneTrip> trips;
    {
        DroneTrip t1;
        t1.drone_slot = 0;
        t1.launch_node = 3;
        t1.service_customer = 1;
        t1.recovery_node = 2;

        DroneTrip t2;
        t2.drone_slot = 1;
        t2.launch_node = 4;
        t2.service_customer = 2;
        t2.recovery_node = 0;

        trips.push_back(t1);
        trips.push_back(t2);

        check(trips.size() == 2, "Two drone trips can be held together");
        check(trips[0].drone_slot == 0 && trips[1].drone_slot == 1,
              "Each drone trip keeps its own drone_slot");
    }

    // 6. A truck route can be represented as an ordered sequence of node ids.
    TruckRoute route;
    {
        route = {0, 3, 1, 4, 2, 0};
        check(route.size() == 6, "Truck route holds the expected number of stops");
        check(route.front() == 0 && route.back() == 0, "Truck route starts and ends at the depot id");
        check(route[2] == 1, "Truck route preserves stop order");
    }

    // 7. A complete static solution can contain a truck route and multiple drone trips.
    {
        Solution solution;
        solution.truck_route = route;
        solution.drone_trips = trips;

        check(solution.truck_route.size() == 6, "Solution carries the truck route");
        check(solution.drone_trips.size() == 2, "Solution carries all drone trips");
        check(solution.drone_trips[1].recovery_node == 0, "Solution's drone trip data is intact");
    }

    // 8. Parameters can be constructed with sensible values.
    {
        TruckParameters truck;
        truck.speed = 15.0;
        check(truck.speed == 15.0, "TruckParameters stores speed");

        DroneParameters drone;
        drone.speed = 12.0;
        drone.endurance_seconds = 1800.0;
        drone.launch_prep_time_sl = 60.0;
        drone.recovery_time_sr = 40.0;
        drone.num_drones = 2;

        check(drone.speed == 12.0, "DroneParameters stores speed");
        check(drone.endurance_seconds == 1800.0, "DroneParameters stores endurance");
        check(drone.launch_prep_time_sl == 60.0, "DroneParameters stores SL");
        check(drone.recovery_time_sr == 40.0, "DroneParameters stores SR");
        check(drone.num_drones == 2, "DroneParameters stores fleet size");
    }

    // 9. Invalid/ambiguous states the model explicitly rejects.
    {
        DroneTrip same_launch_and_customer;
        same_launch_and_customer.launch_node = 3;
        same_launch_and_customer.service_customer = 3;
        same_launch_and_customer.recovery_node = 2;
        check(!has_distinct_endpoints(same_launch_and_customer),
              "Trip with launch_node == service_customer is rejected as ill-formed");

        DroneTrip same_launch_and_recovery;
        same_launch_and_recovery.launch_node = 3;
        same_launch_and_recovery.service_customer = 1;
        same_launch_and_recovery.recovery_node = 3;
        check(!has_distinct_endpoints(same_launch_and_recovery),
              "Trip with launch_node == recovery_node is rejected as ill-formed");

        DroneTrip same_customer_and_recovery;
        same_customer_and_recovery.launch_node = 3;
        same_customer_and_recovery.service_customer = 1;
        same_customer_and_recovery.recovery_node = 1;
        check(!has_distinct_endpoints(same_customer_and_recovery),
              "Trip with service_customer == recovery_node is rejected as ill-formed");

        DroneTrip unset;
        check(!has_distinct_endpoints(unset), "Default-constructed (unset) trip is rejected as ill-formed");

        DroneTrip wrong_role;
        wrong_role.launch_node = 1;          // this id is a Customer, not a FlexibleLaunchPoint
        wrong_role.service_customer = 2;
        wrong_role.recovery_node = 0;
        check(has_distinct_endpoints(wrong_role),
              "Trip launching from a customer node still has distinct endpoints (identity check alone can't catch this)");
        check(!has_valid_node_roles(wrong_role, registry),
              "Trip launching from a customer node (not a flex point) fails the node-role check");

        DroneTrip unregistered_node;
        unregistered_node.launch_node = 3;
        unregistered_node.service_customer = 1;
        unregistered_node.recovery_node = 999; // never registered
        check(!has_valid_node_roles(unregistered_node, registry),
              "Trip referencing an unregistered node id fails the node-role check");
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
