#include "fixtures/small_graph_fixture.hpp"
#include "graph/distance.hpp"
#include "graph/spatial_network.hpp"
#include "graph/travel_time.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace tdr::graph;
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

void check_near(double a, double b, double tol, const std::string& description) {
    check(std::fabs(a - b) <= tol,
          description + " (got " + std::to_string(a) + ", expected " + std::to_string(b) + ")");
}

} // namespace

int main() {
    // 1. Euclidean distance on known coordinates (3-4-5 triangle).
    {
        Point2D a{0.0, 0.0};
        Point2D b{3.0, 4.0};
        check_near(euclidean_distance(a, b), 5.0, 1e-9, "Euclidean distance on a 3-4-5 triangle is exactly 5");
    }

    // 2. Symmetry of Euclidean distance.
    {
        Point2D a{1.5, -2.25};
        Point2D b{-7.0, 3.0};
        check_near(euclidean_distance(a, b), euclidean_distance(b, a), 1e-12,
                   "Euclidean distance is symmetric");
    }

    // 3. Zero self-distance.
    {
        Point2D a{42.0, -13.0};
        check_near(euclidean_distance(a, a), 0.0, 1e-12, "Distance from a point to itself is zero");
    }

    // 4. Correct truck travel time for a known distance/speed.
    {
        TruckParameters truck;
        truck.speed = 5.0;
        check_near(truck_travel_time(20.0, truck), 4.0, 1e-9,
                   "Truck travel time = distance / truck speed");
    }

    // 5. Correct drone travel time for a known distance/speed.
    {
        DroneParameters drone;
        drone.speed = 4.0;
        check_near(drone_travel_time(10.0, drone), 2.5, 1e-9,
                   "Drone travel time = distance / drone speed (raw, pre-theta)");
    }

    // Travel time safety guard: non-positive speed must not silently
    // divide-by-zero into a wrong finite number.
    {
        TruckParameters zero_speed_truck;
        zero_speed_truck.speed = 0.0;
        check(std::isinf(truck_travel_time(10.0, zero_speed_truck)),
              "Truck travel time with zero speed reports +infinity, not a silent finite value");
    }

    // 6. Construction/access of the complete distance/travel-time
    // representation (SpatialNetwork), and 7. multiple nodes with
    // different coordinates.
    NodeRegistry nodes = tdr::test_fixtures::build_small_graph_fixture();
    {
        TruckParameters truck;
        truck.speed = 5.0; // chosen so distance 5 -> time 1, for easy hand-checking
        DroneParameters drone;
        drone.speed = 2.0; // distance 4 -> time 2

        SpatialNetwork network(nodes, truck, drone);

        check(network.nodes().size() == 5, "SpatialNetwork exposes all registered nodes");
        check_near(network.truck_parameters().speed, 5.0, 1e-12, "SpatialNetwork exposes truck parameters");
        check_near(network.drone_parameters().speed, 2.0, 1e-12, "SpatialNetwork exposes drone parameters");

        check_near(network.distance(0, 1), 5.0, 1e-9, "SpatialNetwork distance depot->customer1 is 5");
        check_near(network.distance(1, 0), network.distance(0, 1), 1e-12, "SpatialNetwork distance is symmetric");
        check_near(network.distance(0, 2), 10.0, 1e-9, "SpatialNetwork distance depot->customer2 is 10");
        check_near(network.distance(0, 3), 5.0, 1e-9, "SpatialNetwork distance depot->customer3 is 5");
        check_near(network.distance(1, 2), 5.0, 1e-9, "SpatialNetwork distance customer1->customer2 is 5");
        check_near(network.distance(0, 4), 4.0, 1e-9, "SpatialNetwork distance depot->flex point is 4");

        check_near(network.truck_time(0, 1), 1.0, 1e-9, "SpatialNetwork truck time depot->customer1 is distance/speed = 1");
        check_near(network.drone_time(0, 4), 2.0, 1e-9, "SpatialNetwork drone time depot->flex point is distance/speed = 2");

        check_near(network.distance(2, 2), 0.0, 1e-12, "SpatialNetwork self-distance is zero");
    }

    // 8. Small deterministic graph fixture, reused via build_distance_matrix
    // over an explicit ordered id list (the kind of thing Phase G's
    // truck-only TSP stage will need over {depot, customers}).
    {
        TruckParameters truck;
        truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 1.0;
        SpatialNetwork network(nodes, truck, drone);

        std::vector<NodeId> depot_and_customers = {0, 1, 2, 3};
        DistanceMatrix matrix = network.build_distance_matrix(depot_and_customers);

        check(matrix.size() == 4, "Distance matrix has one row per requested id");
        check(matrix[0].size() == 4, "Distance matrix rows have one column per requested id");
        check_near(matrix[0][0], 0.0, 1e-12, "Distance matrix diagonal is zero");
        check_near(matrix[0][1], 5.0, 1e-9, "Distance matrix [depot][customer1] matches direct distance()");
        check_near(matrix[1][2], 5.0, 1e-9, "Distance matrix [customer1][customer2] matches direct distance()");
        check_near(matrix[0][1], matrix[1][0], 1e-12, "Distance matrix is symmetric");

        DistanceMatrix truck_matrix = network.build_truck_time_matrix(depot_and_customers);
        check_near(truck_matrix[0][1], 5.0, 1e-9, "Truck-time matrix entry matches distance / truck speed");

        DistanceMatrix drone_matrix = network.build_drone_time_matrix(depot_and_customers);
        check_near(drone_matrix[0][1], 5.0, 1e-9, "Drone-time matrix entry matches distance / drone speed");
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
