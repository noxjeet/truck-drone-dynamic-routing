// Phase E tests for the centralized timing/feasibility engine.
//
// Every expected value below is computed BY HAND in this file's
// comments (and cross-checked here in code using only basic arithmetic
// -- never by calling evaluate_solution() or any of its internal
// helpers to produce the "expected" side of a check). Node layouts are
// chosen so every distance is an exact integer or a clean decimal
// (mostly 3-4-5/6-8-10/8-15-17 Pythagorean triples), so expected values
// can be verified with a calculator, not just trusted.

#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"
#include "model/solution.hpp"
#include "timing/timing_engine.hpp"

#include <cmath>
#include <exception>
#include <iostream>
#include <initializer_list>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace tdr::model;
using namespace tdr::graph;
using namespace tdr::timing;

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

// Arc ownership for the FLPs used in a fixture: {flp id, arc_from, arc_to}.
FlpDataMap flps(std::initializer_list<std::tuple<NodeId, NodeId, NodeId>> entries) {
    FlpDataMap m;
    for (const auto& [f, from, to] : entries) m[f] = FlexibleLaunchPointData{f, from, to, 0.5};
    return m;
}

bool any_reason_contains(const std::vector<std::string>& reasons, const std::string& needle) {
    for (const auto& r : reasons) {
        if (r.find(needle) != std::string::npos) return true;
    }
    return false;
}

} // namespace

int main() {
    // =====================================================================
    // 1 & 6. Truck-only route timing on a simple line; truck arrival
    // propagation along a multi-stop route.
    //
    // Depot 0=(0,0), A=1=(10,0), B=2=(25,0). truck_speed=5. Se1=2 for
    // both A and B. Route: 0 -> A -> B -> 0.
    //
    // Hand computation:
    //   arrival[0]=0, departure[0]=0
    //   0->A: dist=10, time=10/5=2            -> arrival[1]=2
    //   departure[1]=2+Se1(2)=4
    //   A->B: dist=15, time=15/5=3             -> arrival[2]=4+3=7
    //   departure[2]=7+Se1(2)=9
    //   B->0: dist=25, time=25/5=5             -> arrival[3]=9+5=14
    //   departure[3]=14 (depot, no service)
    //   objective = arrival(A) + arrival(B) = 2 + 7 = 9
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{25.0, 0.0}});

        TruckParameters truck; truck.speed = 5.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 100.0;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 2, 0};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 2.0, 0.0, 0.0};
        customers[2] = CustomerData{2, 2.0, 0.0, 0.0};

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers, FlpDataMap{});

        check(result.feasible, "Truck-only route is feasible");
        check(result.truck_timing.arrival_time.size() == 4, "Truck timing has one entry per route position");
        check_near(result.truck_timing.arrival_time[0], 0.0, 1e-9, "Truck starts at time 0");
        check_near(result.truck_timing.arrival_time[1], 2.0, 1e-9, "Truck arrival at A propagates from departure+travel time");
        check_near(result.truck_timing.departure_time[1], 4.0, 1e-9, "Truck departure at A includes Se1");
        check_near(result.truck_timing.arrival_time[2], 7.0, 1e-9, "Truck arrival at B propagates from A's departure");
        check_near(result.truck_timing.departure_time[2], 9.0, 1e-9, "Truck departure at B includes Se1");
        check_near(result.truck_timing.arrival_time[3], 14.0, 1e-9, "Truck arrival back at depot propagates correctly");
        check_near(result.objective_value, 9.0, 1e-9, "Truck-only objective is the sum of the two truck arrival times");
    }

    // =====================================================================
    // 2,3,4,5,7. A single drone trip on a simple geometry where launch
    // timing, drone arrival at the customer, and drone recovery timing
    // can all be computed by hand -- and the truck arrives at the
    // recovery node before the drone, so it must wait.
    //
    // Depot 0=(0,0), FLP 1=(10,0), Customer(drone) 2=(10,8),
    // Customer(recovery,truck-served) 3=(16,0). truck_speed=5,
    // drone_speed=2, SL=1, SR=1, Se1(3)=2, Se2(2)=3, theta=0.
    // Route: 0 -> FLP(1) -> C(3) -> 0. Trip: launch=1, customer=2, recovery=3.
    //
    //   arrival[0]=0, departure[0]=0
    //   0->1: dist=10, time=2                  -> arrival[1]=2
    //   FLP launch: departure[1]=2+SL(1)=3      -> launch_time=3
    //   outbound: dist(1,2)=8 (vertical), drone time=8/2=4
    //             -> customer_arrival=3+4=7
    //   customer_departure=7+Se2(3)=10
    //   return: dist(2,3)=sqrt(6^2+8^2)=10 (6-8-10 triple), drone time=10/2=5
    //             -> recovery_arrival(drone lands)=10+5=15
    //   1->3: dist=6, truck time=6/5=1.2        -> arrival[2]=3+1.2=4.2
    //   recovery_complete = max(4.2, 15) + SR(1) = 16
    //   service_complete  = 4.2 + Se1(2) = 6.2
    //   departure[2] = max(6.2, 16) = 16
    //   3->0: dist=16, truck time=16/5=3.2      -> arrival[3]=16+3.2=19.2
    //   endurance, constraints (36)/(37) with l^_i = launch = 3,
    //   l^_j = customer departure = 10, t_k = truck arrival at 3 = 4.2:
    //     loaded part (l^_j - l^_i) = outbound 4 + Se2 3 = 7
    //     (36) = (t_k - l^_j) + 7*(1+0) = (4.2 - 10) + 7 = 1.2
    //     (37) = 7*(1+0) + return 5 = 12      -> binding value 12
    //   [before the 29 Sep 2026 correction the code used 4 + 5 = 9, which
    //   dropped Se2 from the loaded part of (36)/(37)]
    //   truck arrives (4.2) before drone lands (15) -> truck waits
    //   objective = arrival(3)=4.2 [truck-served] + customer_arrival(2)=7 [drone-served] = 11.2
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::FlexibleLaunchPoint, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{10.0, 8.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{16.0, 0.0}});

        TruckParameters truck; truck.speed = 5.0;
        DroneParameters drone;
        drone.speed = 2.0;
        drone.launch_prep_time_sl = 1.0;
        drone.recovery_time_sr = 1.0;
        drone.endurance_seconds = 100.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;
        trip.service_customer = 2;
        trip.recovery_node = 3;
        trip.drone_id = 0;
        solution.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 3.0, 0.0}; // drone-served: Se2=3, theta=0
        customers[3] = CustomerData{3, 2.0, 0.0, 0.0}; // truck-served: Se1=2

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flps({{1, 0, 3}}));

        check(result.feasible, "Single-trip solution (truck waits) is feasible");
        check(result.drone_trip_timings.size() == 1, "One drone trip timing entry produced");
        const DroneTripTiming& t = result.drone_trip_timings[0];

        check_near(result.truck_timing.arrival_time[1], 2.0, 1e-9, "Truck arrival at the FLP");
        check_near(t.launch_time, 3.0, 1e-9, "Launch time = truck arrival at FLP + SL");
        check_near(t.outbound_flight_time_raw, 4.0, 1e-9, "Outbound raw flight time = distance/drone speed");
        check_near(t.customer_arrival_time, 7.0, 1e-9, "Drone arrival at customer = launch time + outbound flight time");
        check_near(t.customer_departure_time, 10.0, 1e-9, "Drone departs customer after Se2");
        check_near(t.return_flight_time_raw, 5.0, 1e-9, "Return raw flight time = distance/drone speed");
        check_near(t.recovery_arrival_time, 15.0, 1e-9, "Drone physical recovery arrival = customer departure + return flight time");
        check_near(t.endurance_value_36, 1.2, 1e-9, "(36): (t_k - l^_j) + (l^_j - l^_i)(1+theta) = (4.2-10) + 7 = 1.2");
        check_near(t.endurance_value_37, 12.0, 1e-9, "(37): (l^_j - l^_i)(1+theta) + d^_jk = 7 + 5 = 12 (loaded part includes Se2)");
        check_near(t.generalized_flight_time, 12.0, 1e-9, "Binding endurance value is max((36),(37)) = 12");
        check(t.endurance_feasible, "Trip is within the (generous) endurance budget");

        check_near(result.truck_timing.arrival_time[2], 4.2, 1e-9, "Truck arrival at the recovery node");
        check_near(t.recovery_complete_time, 16.0, 1e-9, "Recovery completes SR after the later (drone) arrival");
        check_near(result.truck_timing.departure_time[2], 16.0, 1e-9, "Truck departure at recovery node is gated by recovery completion, not just Se1");

        check(result.synchronization_records.size() == 1, "One synchronization record produced");
        const SynchronizationRecord& sync = result.synchronization_records[0];
        check(sync.truck_waited && !sync.drone_waited, "Truck arrives first and waits for the drone");
        check_near(sync.wait_time, 10.8, 1e-9, "Wait time is the difference between drone and truck arrival at recovery");

        check_near(result.objective_value, 11.2, 1e-9, "Objective sums truck arrival (recovery customer) and drone arrival (served customer)");
    }

    // =====================================================================
    // 8. A case where the drone arrives at the recovery node before the
    // truck, and the drone must wait.
    //
    // Depot 0=(0,0), FLP 1=(5,0), Customer(drone) 2=(15,0),
    // Customer(recovery) 3=(17,0) -- all colinear. truck_speed=1,
    // drone_speed=2, SL=SR=Se1=Se2=theta=0.
    // Route: 0 -> FLP(1) -> C(3) -> 0. Trip: launch=1, customer=2, recovery=3.
    //
    //   0->1: dist=5, time=5/1=5               -> arrival[1]=5
    //   launch_time = 5 + SL(0) = 5
    //   outbound: dist(1,2)=10, drone time=10/2=5 -> customer_arrival=5+5=10
    //   customer_departure=10+Se2(0)=10
    //   return: dist(2,3)=2, drone time=2/2=1   -> recovery_arrival(drone)=10+1=11
    //   1->3: dist=12, truck time=12/1=12       -> arrival[2]=5+12=17
    //   drone lands at 11, truck arrives at 17 -> drone arrives FIRST, waits
    //   recovery_complete = max(17,11)+SR(0) = 17
    //   endurance (36) = (t_k 17 - l^_j 10) + (10 - 5)*1 = 12  -- the drone's
    //     6-second wait for the truck counts (p15, p30); (37) = 5 + 1 = 6
    //   objective = customer_arrival(2)=10 [drone] + arrival(3)=17 [truck] = 27
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::FlexibleLaunchPoint, Point2D{5.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{15.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{17.0, 0.0}});

        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 2.0; drone.endurance_seconds = 100.0; drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;
        trip.service_customer = 2;
        trip.recovery_node = 3;
        trip.drone_id = 0;
        solution.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flps({{1, 0, 3}}));

        check(result.feasible, "Single-trip solution (drone waits) is feasible");
        const DroneTripTiming& t = result.drone_trip_timings[0];
        check_near(t.launch_time, 5.0, 1e-9, "Launch time on the colinear layout");
        check_near(t.customer_arrival_time, 10.0, 1e-9, "Drone arrival at customer");
        check_near(t.recovery_arrival_time, 11.0, 1e-9, "Drone physically lands at 11");
        check_near(result.truck_timing.arrival_time[2], 17.0, 1e-9, "Truck arrives at recovery at 17, later than the drone");

        check(result.synchronization_records.size() == 1, "One synchronization record produced");
        const SynchronizationRecord& sync = result.synchronization_records[0];
        check(sync.drone_waited && !sync.truck_waited, "Drone lands first and waits for the truck");
        check_near(sync.wait_time, 6.0, 1e-9, "Wait time is 17 - 11 = 6");
        check_near(t.recovery_complete_time, 17.0, 1e-9, "Recovery completes at the later (truck) arrival, SR=0 here");
        check_near(t.endurance_value_36, 12.0, 1e-9, "(36) counts the drone's wait until the truck arrives: (17-10) + 5 = 12");
        check_near(t.endurance_value_37, 6.0, 1e-9, "(37) = loaded 5 + return 1 = 6");

        check_near(result.objective_value, 27.0, 1e-9, "Objective sums drone arrival (10) and truck arrival (17)");
    }

    // =====================================================================
    // 9. Endurance feasibility boundary: the same drone trip is feasible
    // under a generous endurance budget and infeasible under a tight one.
    //
    // Depot 0=(-1,0), FLP 1=(0,0), Customer(drone) 2=(0,3),
    // Customer(recovery) 3=(4,0). truck_speed=1, drone_speed=1 (so raw
    // distances equal raw times), SL=SR=Se1=Se2=theta=0.
    //
    //   0->1: dist=1                            -> arrival[1]=1
    //   launch_time=1
    //   outbound: dist(1,2)=3 (vertical)         -> customer_arrival=1+3=4
    //   return: dist(2,3)=sqrt(4^2+3^2)=5 (3-4-5) -> recovery_arrival=4+5=9
    //   truck: FLP(0,0) -> C3(4,0): dist 4 -> t_k = 1 + 4 = 5
    //   (36) = (t_k 5 - l^_j 4) + 3 = 4 ; (37) = 3 + 5 = 8 -> binding 8
    //   endurance=5  -> 8 > 5  -> INFEASIBLE
    //   endurance=10 -> 8 <= 10 -> feasible
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{-1.0, 0.0}});
        nodes.add(Node{1, NodeType::FlexibleLaunchPoint, Point2D{0.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{0.0, 3.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{4.0, 0.0}});

        TruckParameters truck; truck.speed = 1.0;

        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;
        trip.service_customer = 2;
        trip.recovery_node = 3;
        trip.drone_id = 0;
        solution.drone_trips = {trip};
        const FlpDataMap flp_data = flps({{1, 0, 3}});

        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};

        {
            DroneParameters tight_drone;
            tight_drone.speed = 1.0;
            tight_drone.endurance_seconds = 5.0; // < required 8
            tight_drone.num_drones = 1;
            SpatialNetwork network(nodes, truck, tight_drone);
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flp_data);

            check_near(result.drone_trip_timings[0].generalized_flight_time, 8.0, 1e-9,
                       "Binding endurance value for the boundary test trip is (37) = 3+5 = 8");
            check(!result.drone_trip_timings[0].endurance_feasible, "Trip exceeds a 5-second endurance budget (8 > 5)");
            check(!result.drone_trip_timings[0].feasible, "Trip is marked infeasible due to endurance");
            check(!result.feasible, "Overall solution is infeasible because its only drone trip violates endurance");
            check(any_reason_contains(result.violation_reasons, "battery endurance"),
                  "Violation reasons mention exceeding battery endurance");
        }
        {
            DroneParameters generous_drone;
            generous_drone.speed = 1.0;
            generous_drone.endurance_seconds = 10.0; // >= required 8
            generous_drone.num_drones = 1;
            SpatialNetwork network(nodes, truck, generous_drone);
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flp_data);

            check(result.drone_trip_timings[0].endurance_feasible, "Same trip is feasible under a 10-second endurance budget (8 <= 10)");
            check(result.feasible, "Overall solution is feasible with the generous endurance budget");
        }
    }

    // =====================================================================
    // 10. A drone trip with an invalid launch-node role (a Customer node
    // used as if it were a flexible launch point) is rejected.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{1.0, 0.0}}); // wrongly used as "launch"
        nodes.add(Node{2, NodeType::Customer, Point2D{2.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{3.0, 0.0}});

        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 100.0;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 2, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;       // Customer, not FlexibleLaunchPoint -- invalid role
        trip.service_customer = 2;
        trip.recovery_node = 3;
        solution.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers; // deliberately empty; irrelevant to this check

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers, FlpDataMap{});

        check(!result.drone_trip_timings[0].structurally_valid, "Trip with a Customer-typed launch node fails the node-role check");
        check(!result.drone_trip_timings[0].feasible, "Trip with an invalid launch-node role is infeasible");
        check(!result.feasible, "Overall solution is infeasible due to the invalid trip");
        check(any_reason_contains(result.violation_reasons, "invalid launch/customer/recovery node id or role"),
              "Violation reasons name the invalid node role");
    }

    // =====================================================================
    // 11. Load-dependent (theta) generalized flight time changes
    // endurance feasibility on an otherwise-identical trip.
    //
    // Same geometry as the "truck waits" trip above: outbound raw = 4,
    // Se2 = 3, return raw = 5, t_k = 4.2, l^_i = 3, l^_j = 10. The loaded
    // part of (36)/(37) is l^_j - l^_i = 4 + 3 = 7 (outbound + Se2).
    // Endurance fixed at 13.
    //   theta=0:   (37) = 7*(1+0)   + 5 = 12   <= 13 -> feasible
    //   theta=0.5: (37) = 7*(1+0.5) + 5 = 15.5 >  13 -> infeasible
    //              ((36) = (4.2-10) + 10.5 = 4.7, not binding)
    // [Before the 29 Sep 2026 correction this block used e = 10 with the
    // values 9 and 11, which applied theta to the outbound flight only and
    // dropped Se2.]
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::FlexibleLaunchPoint, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{10.0, 8.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{16.0, 0.0}});

        TruckParameters truck; truck.speed = 5.0;
        DroneParameters drone;
        drone.speed = 2.0;
        drone.launch_prep_time_sl = 1.0;
        drone.recovery_time_sr = 1.0;
        drone.endurance_seconds = 13.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;
        trip.service_customer = 2;
        trip.recovery_node = 3;
        trip.drone_id = 0;
        solution.drone_trips = {trip};
        const FlpDataMap flp_data = flps({{1, 0, 3}});

        {
            std::unordered_map<NodeId, CustomerData> customers;
            customers[2] = CustomerData{2, 0.0, 3.0, 0.0}; // theta=0
            customers[3] = CustomerData{3, 2.0, 0.0, 0.0};
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flp_data);
            check_near(result.drone_trip_timings[0].generalized_flight_time, 12.0, 1e-9,
                       "theta=0: binding endurance value (37) = 7 + 5 = 12");
            check(result.drone_trip_timings[0].feasible, "theta=0: trip is feasible under endurance=13");
        }
        {
            std::unordered_map<NodeId, CustomerData> customers;
            customers[2] = CustomerData{2, 0.0, 3.0, 0.5}; // theta=0.5
            customers[3] = CustomerData{3, 2.0, 0.0, 0.0};
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flp_data);
            check_near(result.drone_trip_timings[0].generalized_flight_time, 15.5, 1e-9,
                       "theta=0.5: loaded part (outbound 4 + Se2 3) inflated to 7*1.5 = 10.5, (37) = 10.5 + 5 = 15.5");
            check_near(result.drone_trip_timings[0].endurance_value_36, 4.7, 1e-9,
                       "theta=0.5: (36) = (4.2 - 10) + 10.5 = 4.7");
            check(!result.drone_trip_timings[0].feasible, "theta=0.5: the SAME geometry is now infeasible under endurance=13");
            check_near(result.drone_trip_timings[0].customer_arrival_time, 7.0, 1e-9,
                       "theta never changes the REAL customer arrival time (still launch_time+raw outbound=7)");
        }
    }

    // =====================================================================
    // 12,13,14. A mixed truck-served + drone-served solution with two
    // drone trips, and its hand-computed customer waiting-time objective.
    //
    // Nodes: 0=Depot(0,0), 1=A,Customer(5,0) [truck-served],
    // 2=FLP1(10,0), 3=D1,Customer(10,4) [drone-served via trip1],
    // 4=B,Customer(13,0) [truck-served, recovery for trip1],
    // 5=FLP2(15,0), 6=D2,Customer(15,8) [drone-served via trip2],
    // recovery for trip2 = depot (node 0, end of route).
    // truck_speed=5, drone_speed=2, SL=SR=1, Se1(A)=Se1(B)=2,
    // Se2(D1)=Se2(D2)=1, theta=0 for both. Route:
    // 0 -> A(1) -> FLP1(2) -> B(4) -> FLP2(5) -> 0.
    //
    //   arrival[0]=0, departure[0]=0
    //   0->1: dist=5, time=1                 -> arrival[1]=1
    //   departure[1]=1+Se1(2)=3
    //   1->2: dist=5, time=1                 -> arrival[2]=3+1=4
    //   FLP1 launch(trip1): departure[2]=4+SL(1)=5  -> launch_time1=5
    //     outbound(2,3): dist=4 (vertical), dtime=4/2=2 -> cust_arrival1=5+2=7
    //     cust_departure1=7+Se2(1)=8
    //     return(3,4): dist=sqrt(3^2+4^2)=5 (3-4-5), dtime=5/2=2.5 -> recov_arrival1=8+2.5=10.5
    //   2->4: dist=3, time=3/5=0.6           -> arrival[3]=5+0.6=5.6
    //   service_complete(B)=5.6+2=7.6
    //   recovery_complete1=max(5.6,10.5)+SR(1)=11.5
    //   departure[3]=max(7.6,11.5)=11.5
    //   4->5: dist=2, time=2/5=0.4           -> arrival[4]=11.5+0.4=11.9
    //   FLP2 launch(trip2): departure[4]=11.9+SL(1)=12.9 -> launch_time2=12.9
    //     outbound(5,6): dist=8 (vertical), dtime=8/2=4 -> cust_arrival2=12.9+4=16.9
    //     cust_departure2=16.9+Se2(1)=17.9
    //     return(6,0): dist=sqrt(15^2+8^2)=17 (8-15-17), dtime=17/2=8.5 -> recov_arrival2=17.9+8.5=26.4
    //   5->0: dist=15, time=15/5=3           -> arrival[5]=12.9+3=15.9
    //   recovery_complete2=max(15.9,26.4)+SR(1)=27.4
    //   departure[5]=max(15.9,27.4)=27.4
    //
    //   objective = arrival(A)=1 + cust_arrival1(D1)=7 + arrival(B)=5.6
    //             + cust_arrival2(D2)=16.9  =  30.5
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{5.0, 0.0}});
        nodes.add(Node{2, NodeType::FlexibleLaunchPoint, Point2D{10.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{10.0, 4.0}});
        nodes.add(Node{4, NodeType::Customer, Point2D{13.0, 0.0}});
        nodes.add(Node{5, NodeType::FlexibleLaunchPoint, Point2D{15.0, 0.0}});
        nodes.add(Node{6, NodeType::Customer, Point2D{15.0, 8.0}});

        TruckParameters truck; truck.speed = 5.0;
        DroneParameters drone;
        drone.speed = 2.0;
        drone.launch_prep_time_sl = 1.0;
        drone.recovery_time_sr = 1.0;
        drone.endurance_seconds = 100.0;
        drone.num_drones = 1; // both trips are flown by the same physical drone, one after the other
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 2, 4, 5, 0};

        DroneTrip trip1;
        trip1.launch_node = 2;
        trip1.service_customer = 3;
        trip1.recovery_node = 4;
        trip1.drone_id = 0;

        DroneTrip trip2;
        trip2.launch_node = 5;
        trip2.service_customer = 6;
        trip2.recovery_node = 0;
        trip2.drone_id = 0; // relaunched at FLP2 (position 4) after recovery at B (position 3): valid reuse

        solution.drone_trips = {trip1, trip2};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 2.0, 0.0, 0.0}; // A, truck-served
        customers[3] = CustomerData{3, 0.0, 1.0, 0.0}; // D1, drone-served
        customers[4] = CustomerData{4, 2.0, 0.0, 0.0}; // B, truck-served + recovery
        customers[6] = CustomerData{6, 0.0, 1.0, 0.0}; // D2, drone-served

        // FLP1 lies on A->B (A=1, B=4); FLP2 lies on B->depot.
        SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flps({{2, 1, 4}, {5, 4, 0}}));

        check(result.feasible, "Mixed truck+drone, two-trip solution is feasible");
        check(result.drone_trip_timings.size() == 2, "Both drone trips produced a timing record");
        check(result.drone_trip_timings[0].feasible && result.drone_trip_timings[1].feasible,
              "Both drone trips are individually feasible");

        check_near(result.truck_timing.arrival_time[1], 1.0, 1e-9, "Arrival at A");
        check_near(result.truck_timing.arrival_time[2], 4.0, 1e-9, "Arrival at FLP1");
        check_near(result.drone_trip_timings[0].launch_time, 5.0, 1e-9, "Trip1 launch time");
        check_near(result.drone_trip_timings[0].customer_arrival_time, 7.0, 1e-9, "Trip1 customer arrival (D1)");
        check_near(result.drone_trip_timings[0].recovery_arrival_time, 10.5, 1e-9, "Trip1 recovery arrival");
        check_near(result.truck_timing.arrival_time[3], 5.6, 1e-9, "Arrival at B");
        check_near(result.truck_timing.departure_time[3], 11.5, 1e-9, "Departure at B gated by trip1's recovery");
        check_near(result.truck_timing.arrival_time[4], 11.9, 1e-9, "Arrival at FLP2");
        check_near(result.drone_trip_timings[1].launch_time, 12.9, 1e-9, "Trip2 launch time");
        check_near(result.drone_trip_timings[1].customer_arrival_time, 16.9, 1e-9, "Trip2 customer arrival (D2)");
        check_near(result.drone_trip_timings[1].recovery_arrival_time, 26.4, 1e-9, "Trip2 recovery arrival (at depot)");
        check_near(result.truck_timing.arrival_time[5], 15.9, 1e-9, "Arrival back at depot");
        check_near(result.truck_timing.departure_time[5], 27.4, 1e-9, "Departure at depot gated by trip2's recovery");

        // Every customer's resolution (item 13: mixed truck+drone service).
        bool a_truck = false, d1_drone = false, b_truck = false, d2_drone = false;
        for (const auto& rec : result.customer_records) {
            if (rec.customer_id == 1) a_truck = rec.served_by_truck && !rec.served_by_drone;
            if (rec.customer_id == 3) d1_drone = rec.served_by_drone && !rec.served_by_truck;
            if (rec.customer_id == 4) b_truck = rec.served_by_truck && !rec.served_by_drone;
            if (rec.customer_id == 6) d2_drone = rec.served_by_drone && !rec.served_by_truck;
        }
        check(a_truck, "Customer A is resolved as truck-served");
        check(d1_drone, "Customer D1 is resolved as drone-served");
        check(b_truck, "Customer B is resolved as truck-served");
        check(d2_drone, "Customer D2 is resolved as drone-served");

        check_near(result.objective_value, 30.5, 1e-9,
                   "Hand-computed objective: 1 (A) + 7 (D1) + 5.6 (B) + 16.9 (D2) = 30.5");
    }

    // =====================================================================
    // 15. A clearly infeasible complete solution: one customer is never
    // served by either vehicle. Expect feasible == false with a useful,
    // specific violation reason.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{5.0, 0.0}});  // served by truck
        nodes.add(Node{2, NodeType::Customer, Point2D{8.0, 0.0}});  // never served by anyone

        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 100.0;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 0}; // customer 2 is not on the route and has no drone trip

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 1.0, 0.0, 0.0};
        customers[2] = CustomerData{2, 1.0, 0.0, 0.0};

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers, FlpDataMap{});

        check(!result.feasible, "A solution with an unserved customer is infeasible");
        bool found_orphan_violation = false;
        for (const auto& rec : result.customer_records) {
            if (rec.customer_id == 2) {
                check(!rec.served_by_truck && !rec.served_by_drone, "The orphaned customer is marked as served by neither vehicle");
                check(!rec.feasible, "The orphaned customer's own record is marked infeasible");
                found_orphan_violation = !rec.violation_reason.empty();
            }
            if (rec.customer_id == 1) {
                check(rec.served_by_truck, "The other customer is still correctly resolved as truck-served");
                check_near(rec.service_arrival_time, 5.0, 1e-9, "Its service arrival time is the truck's arrival at it");
            }
        }
        check(found_orphan_violation, "The orphaned customer's record carries a specific violation reason");
        check(any_reason_contains(result.violation_reasons, "not served by the truck or any drone trip"),
              "Top-level violation reasons name the unserved customer's problem specifically");
    }

    // =====================================================================
    // Block 9 (Phase E audit regression; REWRITTEN in Stage 2, 8 Nov 2026,
    // for SR Option B / decision D1). A drone recovers at a TRUCK-SERVED
    // customer with nonzero Se1. Under D1 the departure there is
    //     l_C = max( t_C + Se1 + r*SR ,  max_u (a_u + SR) ),   r = 1 here,
    // i.e. recovery work is ADDITIVE to the truck's own service, and each
    // drone's own completion a_u + SR is an independent lower bound.
    // (Before Stage 2 the rule was max(Se1 window, max(t_C,a_u)+SR), with
    // SR running concurrently with service; sub-cases A and B changed.)
    // Three sub-cases place the drone's own arrival a_u BEFORE, DURING and
    // AFTER the truck's service window, using one fixed geometry and
    // varying only drone_speed.
    //
    // Shared geometry: Depot 0=(0,0), FLP 1=(2,0),
    // Customer(drone-served) 2=(6,3), Customer(recovery,truck-served)
    // 3=(10,0). truck_speed=1, SL=0, Se2(2)=0, theta=0, Se1(3)=6, SR=5.
    // Route: 0 -> FLP(1) -> C(3) -> 0.
    //
    // Shared truck-side hand computation (independent of drone_speed):
    //   0->1: dist=2, time=2                    -> arrival[FLP]=2
    //   FLP launch: departure[FLP]=2+SL(0)=2    -> launch_time=2
    //   1->3: dist(FLP,C)=8, time=8             -> t_C = arrival[C] = 10
    //   truck-side term = t_C + Se1 + r*SR = 10 + 6 + 1*5 = 21
    //
    // Drone-side: dist(FLP,D)=5, dist(D,C)=5 (4-3-5 triangles):
    //   customer_arrival(D) = 2 + 5/drone_speed
    //   a_u = recovery_arrival = 2 + 10/drone_speed
    //
    // Sub-case A ("before"): drone_speed=2 -> customer_arrival 4.5, a_u = 7.
    //   drone term = 7 + 5 = 12;   l_C = max(21, 12) = 21
    //   [pre-Stage-2 rule gave 16]
    // Sub-case B ("during"): drone_speed=1 -> customer_arrival 7, a_u = 12
    //   (strictly inside the service window [10, 16]).
    //   drone term = 12 + 5 = 17;  l_C = max(21, 17) = 21
    //   [pre-Stage-2 rule gave 17]
    // Sub-case C ("after"): drone_speed=0.5 -> customer_arrival 12, a_u = 22.
    //   drone term = 22 + 5 = 27;  l_C = max(21, 27) = 27
    //   [pre-Stage-2 rule gave 27: unchanged, the drone dominates]
    //
    // In every sub-case the CUSTOMER-FACING arrival times used by the
    // objective (arrival[C]=10 for the truck-served customer,
    // customer_arrival_time for the drone-served customer) are
    // completely unaffected by any of this -- confirming the objective
    // is arrival-based, never departure/recovery-based, regardless of
    // how Se1/SR interact.
    // =====================================================================
    {
        auto run_case = [](double drone_speed, double expected_departure_at_c,
                            double expected_drone_customer_arrival,
                            const std::string& label) {
            NodeRegistry nodes;
            nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
            nodes.add(Node{1, NodeType::FlexibleLaunchPoint, Point2D{2.0, 0.0}});
            nodes.add(Node{2, NodeType::Customer, Point2D{6.0, 3.0}});
            nodes.add(Node{3, NodeType::Customer, Point2D{10.0, 0.0}});

            TruckParameters truck; truck.speed = 1.0;
            DroneParameters drone;
            drone.speed = drone_speed;
            drone.launch_prep_time_sl = 0.0;
            drone.recovery_time_sr = 5.0;
            drone.endurance_seconds = 100.0; // generous; not the point of this test
            drone.num_drones = 1;
            SpatialNetwork network(nodes, truck, drone);

            Solution solution;
            solution.truck_route = {0, 1, 3, 0};
            DroneTrip trip;
            trip.launch_node = 1;
            trip.service_customer = 2;
            trip.recovery_node = 3;
            trip.drone_id = 0;
            solution.drone_trips = {trip};

            std::unordered_map<NodeId, CustomerData> customers;
            customers[2] = CustomerData{2, 0.0, 0.0, 0.0}; // drone-served: Se2=0, theta=0
            customers[3] = CustomerData{3, 6.0, 0.0, 0.0}; // truck-served: Se1=6

            SolutionEvaluationResult result = evaluate_solution(solution, network, customers, flps({{1, 0, 3}}));

            check(result.feasible, label + ": solution is feasible");
            check_near(result.truck_timing.arrival_time[2], 10.0, 1e-9,
                       label + ": truck arrival at the recovery customer is unaffected by drone speed");
            check_near(result.drone_trip_timings[0].customer_arrival_time, expected_drone_customer_arrival, 1e-9,
                       label + ": drone's own customer arrival time");
            check_near(result.truck_timing.departure_time[2], expected_departure_at_c, 1e-9,
                       label + ": departure at the recovery customer = max(t+Se1+r*SR, a_u+SR) (SR Option B)");
            check(result.drone_trip_timings[0].recovery_complete_time <= result.truck_timing.departure_time[2] + 1e-9,
                  label + ": the drone's own completion max(t,a)+SR never exceeds the truck's departure");

            // The objective must still be arrival-based for both customers,
            // regardless of the Se1/SR interaction above.
            for (const auto& rec : result.customer_records) {
                if (rec.customer_id == 3) {
                    check_near(rec.service_arrival_time, 10.0, 1e-9,
                               label + ": recovery customer's objective contribution is its (truck) arrival time, not departure");
                }
                if (rec.customer_id == 2) {
                    check_near(rec.service_arrival_time, expected_drone_customer_arrival, 1e-9,
                               label + ": drone-served customer's objective contribution is its (drone) arrival time");
                }
            }
        };

        // Sub-case A: drone recovers well BEFORE the truck's service window.
        run_case(/*drone_speed=*/2.0, /*expected_departure_at_c=*/21.0,
                  /*expected_drone_customer_arrival=*/4.5, "Recovery before truck service");

        // Sub-case B: drone recovers DURING the truck's service window.
        run_case(/*drone_speed=*/1.0, /*expected_departure_at_c=*/21.0,
                  /*expected_drone_customer_arrival=*/7.0, "Recovery during truck service");

        // Sub-case C: drone recovers well AFTER the truck's service window
        // (the drone term dominates, so the old and new rules agree here).
        run_case(/*drone_speed=*/0.5, /*expected_departure_at_c=*/27.0,
                  /*expected_drone_customer_arrival=*/12.0, "Recovery after truck service");
    }

    // =====================================================================
    // 29 SEP 2026 PAPER-CONFORMANCE REGRESSIONS (A-K). Each block cites the
    // paper rule it pins down. Page numbers are PDF pages.
    // =====================================================================

    // ---------------------------------------------------------------------
    // A. Two drones launched at the same FLP: constraint (18) holds the
    // truck for n*SL; constraint (21) lets EACH drone depart at t_i + SL
    // (no staggering -- the paper does not order drones at one FLP).
    //
    // Depot 0=(0,0), FLP 10=(10,0) in L_(0,3), C3=(18,0) truck-served,
    // drone customers 1=(14,3) and 2=(14,-3). truck/drone speed 1, SL=5,
    // SR=0, all service times 0, theta=0, |U|=2.
    //   t_FLP = 10; both drones: l^_{FLP,u} = 10 + SL = 15
    //   truck: l_FLP = 10 + 2*SL = 20                         [(18)]
    //   FLP->C3: 8 -> t_C3 = 28
    //   drones: FLP->1 = FLP->2 = 5 (3-4-5) -> t^_1 = t^_2 = 20;
    //           1->C3 = 2->C3 = 5 -> land at 25 (before the truck)
    //   objective = t^_1 20 + t^_2 20 + t_C3 28 = 68
    //   (the pre-correction code held the truck for one SL only: t_C3 = 23,
    //   objective 63)
    // ---------------------------------------------------------------------
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{10, NodeType::FlexibleLaunchPoint, Point2D{10.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{18.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{14.0, 3.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{14.0, -3.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.launch_prep_time_sl = 5.0; drone.endurance_seconds = 100.0; drone.num_drones = 2;
        SpatialNetwork network(nodes, truck, drone);
        Solution solution;
        solution.truck_route = {0, 10, 3, 0};
        DroneTrip a; a.launch_node = 10; a.service_customer = 1; a.recovery_node = 3; a.drone_id = 0;
        DroneTrip b; b.launch_node = 10; b.service_customer = 2; b.recovery_node = 3; b.drone_id = 1;
        solution.drone_trips = {a, b};
        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};
        SolutionEvaluationResult r = evaluate_solution(solution, network, customers, flps({{10, 0, 3}}));
        check(r.feasible, "A: two launches at one FLP -- solution is feasible");
        check_near(r.drone_trip_timings[0].launch_time, 15.0, 1e-9, "A: drone 0 departs at t_FLP + SL = 15 (constraint (21))");
        check_near(r.drone_trip_timings[1].launch_time, 15.0, 1e-9, "A: drone 1 ALSO departs at t_FLP + SL = 15 (no staggering)");
        check_near(r.truck_timing.departure_time[1], 20.0, 1e-9, "A: truck leaves the FLP at t_FLP + 2*SL = 20 (constraint (18))");
        check_near(r.truck_timing.arrival_time[2], 28.0, 1e-9, "A: truck reaches C3 at 20 + 8 = 28");
        check_near(r.objective_value, 68.0, 1e-9, "A: objective = 20 + 20 + 28 = 68");
    }

    // ---------------------------------------------------------------------
    // B. Endurance with Se_j^D and theta_j (constraints (36)/(37)).
    //
    // Depot 0=(0,0), FLP 1=(4,0) in L_(0,3), C3=(12,0), drone customer
    // 2=(8,3): FLP->2 = 5, 2->C3 = 5 (3-4-5). speeds 1, SL=SR=0,
    // Se2(2)=2, theta_2=1.
    //   l^_i = 4; t^_2 = 9; l^_j = 9 + 2 = 11; drone lands at 16
    //   t_k (truck at C3) = 4 + 8 = 12  (truck arrives first)
    //   loaded part l^_j - l^_i = 5 + 2 = 7, times (1+theta) = 14
    //   (36) = (12 - 11) + 14 = 15;  (37) = 14 + 5 = 19  -> binding 19
    //   pre-correction formula: 5*(1+1) + 5 = 15 (dropped Se2 and its theta)
    //   e = 17 -> infeasible (would have been feasible before); e = 19 -> feasible
    // ---------------------------------------------------------------------
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::FlexibleLaunchPoint, Point2D{4.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{8.0, 3.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{12.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip t; t.launch_node = 1; t.service_customer = 2; t.recovery_node = 3; t.drone_id = 0;
        solution.drone_trips = {t};
        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 2.0, 1.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};
        for (double e : {17.0, 19.0}) {
            DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = e; drone.num_drones = 1;
            SpatialNetwork network(nodes, truck, drone);
            SolutionEvaluationResult r = evaluate_solution(solution, network, customers, flps({{1, 0, 3}}));
            const DroneTripTiming& tt = r.drone_trip_timings[0];
            const std::string lbl = "B (e=" + std::to_string(static_cast<int>(e)) + "): ";
            check_near(tt.endurance_value_36, 15.0, 1e-9, lbl + "(36) = (t_k - l^_j) + (l^_j - l^_i)(1+theta) = 1 + 14 = 15");
            check_near(tt.endurance_value_37, 19.0, 1e-9, lbl + "(37) = (l^_j - l^_i)(1+theta) + d^_jk = 14 + 5 = 19 (Se2 inside the loaded part)");
            if (e < 19.0) {
                check(!tt.endurance_feasible && !r.feasible, lbl + "trip is infeasible: 19 > 17 (the old formula's 15 would have passed)");
            } else {
                check(tt.endurance_feasible && r.feasible, lbl + "trip is feasible at the boundary 19 <= 19");
            }
        }
    }

    // ---------------------------------------------------------------------
    // C. Drone waiting for the truck counts toward endurance (constraint
    // (36); p15 "the flight duration of a drone, while awaiting the truck,
    // must not exceed the permissible battery capacity"; p30 "including
    // waiting for the truck at the merging point").
    //
    // Depot 0=(0,0), FLP 5=(3,-4) in L_(0,4), C4=(6,-8) truck-served,
    // C3=(6,0) truck-served (recovery), drone customer 2=(3,0).
    // Route 0 -> FLP5 -> C4 -> C3 -> 0 (the drone skips C4, Fig. 5).
    // speeds 1, SL=SR=0, all service 0, theta=0.
    //   t_FLP = 5; l^_i = 5; FLP->2 = 4 -> t^_2 = 9 = l^_j; 2->C3 = 3 -> lands 12
    //   truck: FLP->C4 = 5 -> 10; C4->C3 = 8 -> t_k = 18 (drone waits 6)
    //   (36) = (18 - 9) + 4 = 13;  (37) = 4 + 3 = 7  -> binding 13
    //   pre-correction formula: 4 + 3 = 7 (ignored the 6-unit wait)
    //   e = 10 -> infeasible; e = 13 -> feasible
    // ---------------------------------------------------------------------
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{5, NodeType::FlexibleLaunchPoint, Point2D{3.0, -4.0}});
        nodes.add(Node{4, NodeType::Customer, Point2D{6.0, -8.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{6.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{3.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        Solution solution;
        solution.truck_route = {0, 5, 4, 3, 0};
        DroneTrip t; t.launch_node = 5; t.service_customer = 2; t.recovery_node = 3; t.drone_id = 0;
        solution.drone_trips = {t};
        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};
        customers[4] = CustomerData{4, 0.0, 0.0, 0.0};
        for (double e : {10.0, 13.0}) {
            DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = e; drone.num_drones = 1;
            SpatialNetwork network(nodes, truck, drone);
            SolutionEvaluationResult r = evaluate_solution(solution, network, customers, flps({{5, 0, 4}}));
            const DroneTripTiming& tt = r.drone_trip_timings[0];
            const std::string lbl = "C (e=" + std::to_string(static_cast<int>(e)) + "): ";
            check_near(tt.recovery_arrival_time, 12.0, 1e-9, lbl + "drone lands at the recovery node at 12");
            check_near(tt.truck_arrival_at_recovery, 18.0, 1e-9, lbl + "truck reaches the recovery node at 18");
            check_near(tt.endurance_value_36, 13.0, 1e-9, lbl + "(36) = (18 - 9) + 4 = 13 includes the drone's wait");
            check_near(tt.endurance_value_37, 7.0, 1e-9, lbl + "(37) = 4 + 3 = 7");
            if (e < 13.0) {
                check(!tt.endurance_feasible && !r.feasible, lbl + "infeasible: waiting pushes endurance use to 13 > 10");
            } else {
                check(tt.endurance_feasible && r.feasible, lbl + "feasible at the boundary 13 <= 13");
            }
        }
    }

    // Shared layout for D-G (physical drones, constraint (28) and |U|):
    // depot 0=(0,0); route 0 -> f11 -> C1 -> f12 -> C2 -> f13 -> C3 -> 0 on
    // the x-axis (C1=(10,0), C2=(20,0), C3=(30,0); f11=(5,0) in L_(0,1),
    // f12=(15,0) in L_(1,2), f13=(25,0) in L_(2,3)); drone customers
    // 21=(5,4), 22=(15,4), 23=(25,4). Route positions: f11=1, C1=2, f12=3,
    // C2=4, f13=5, C3=6. Endurance is generous: only identity is tested.
    auto drone_layout_eval = [](int num_drones, const std::vector<DroneTrip>& trips) {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{20.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{30.0, 0.0}});
        nodes.add(Node{11, NodeType::FlexibleLaunchPoint, Point2D{5.0, 0.0}});
        nodes.add(Node{12, NodeType::FlexibleLaunchPoint, Point2D{15.0, 0.0}});
        nodes.add(Node{13, NodeType::FlexibleLaunchPoint, Point2D{25.0, 0.0}});
        nodes.add(Node{21, NodeType::Customer, Point2D{5.0, 4.0}});
        nodes.add(Node{22, NodeType::Customer, Point2D{15.0, 4.0}});
        nodes.add(Node{23, NodeType::Customer, Point2D{25.0, 4.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 1e6; drone.num_drones = num_drones;
        SpatialNetwork network(nodes, truck, drone);
        Solution solution;
        solution.truck_route = {0, 11, 1, 12, 2, 13, 3, 0};
        solution.drone_trips = trips;
        std::unordered_map<NodeId, CustomerData> customers;
        for (NodeId c : {1, 2, 3}) customers[c] = CustomerData{c, 0.0, 0.0, 0.0};
        for (const auto& t : trips) customers[t.service_customer] = CustomerData{t.service_customer, 0.0, 0.0, 0.0};
        return evaluate_solution(solution, network, customers, flps({{11, 0, 1}, {12, 1, 2}, {13, 2, 3}}));
    };
    auto mk = [](NodeId i, NodeId j, NodeId k, int u) {
        DroneTrip t; t.launch_node = i; t.service_customer = j; t.recovery_node = k; t.drone_id = u; return t;
    };

    // D. VALID sequential reuse: ONE physical drone flies three trips
    // (f11->21->C1, f12->22->C2, f13->23->C3), each relaunch strictly after
    // the previous recovery (positions 1-2, 3-4, 5-6). 3 trips > |U| = 1 is
    // fine: the number of trips is not bounded by |U| (p9: "drones are
    // capable of multiple takeoffs and recoveries"; Fig. 3).
    {
        SolutionEvaluationResult r = drone_layout_eval(1, {mk(11, 21, 1, 0), mk(12, 22, 2, 0), mk(13, 23, 3, 0)});
        check(r.feasible, "D: one physical drone flying 3 sequential trips (|U|=1) is feasible");
        check(r.max_simultaneous_drones == 1, "D: at most 1 drone is ever in flight");
    }
    // D2. Two physical drones in the same layout, alternating: drone 0 flies f11->21->C1 and f13->23->C3, drone 1
    // flies f12->22->C2.
    {
        SolutionEvaluationResult r = drone_layout_eval(2, {mk(11, 21, 1, 0), mk(12, 22, 2, 1), mk(13, 23, 3, 0)});
        check(r.feasible, "D2: two physical drones sharing three sequential trips is feasible");
    }

    // E. INVALID overlapping reuse: drone 0 flies f11->21->C3 (in flight
    // over positions 1..6) and is relaunched at f12 (position 3) for
    // f12->22->C2 before its first trip is recovered -- forbidden by (28)
    // ("prevent the relaunching of drones without truck recovery", p15).
    // f13->23->C3 is flown by drone 1 so that no launch point is idle and
    // (28) is the only violated rule.
    {
        SolutionEvaluationResult same = drone_layout_eval(2, {mk(11, 21, 3, 0), mk(12, 22, 2, 0), mk(13, 23, 3, 1)});
        check(!same.feasible, "E: drone 0 relaunched at position 3 before its recovery at position 6 is rejected, even with |U|=2");
        check(any_reason_contains(same.violation_reasons, "constraint (28)"), "E: violation names constraint (28)");
        check(same.max_simultaneous_drones == 2 && !any_reason_contains(same.violation_reasons, "simultaneously in flight"),
              "E: the fleet COUNT (2 <= |U|=2) is fine -- it is the identity assignment that is wrong");
        SolutionEvaluationResult split = drone_layout_eval(2, {mk(11, 21, 3, 0), mk(12, 22, 2, 1), mk(13, 23, 3, 1)});
        check(split.feasible, "E: the same two overlapping trips on two DIFFERENT drones are feasible");
    }

    // F. Invalid drone ids: u must satisfy 0 <= u < |U|.
    {
        SolutionEvaluationResult too_big = drone_layout_eval(1, {mk(11, 21, 1, 1), mk(12, 22, 2, 0), mk(13, 23, 3, 0)});
        check(!too_big.feasible && !too_big.drone_trip_timings[0].drone_id_valid,
              "F: drone id 1 with |U|=1 is rejected");
        check(any_reason_contains(too_big.violation_reasons, "is not a drone of U"), "F: violation explains the invalid drone id");
        SolutionEvaluationResult unassigned =
            drone_layout_eval(1, {mk(11, 21, 1, kUnassignedDroneId), mk(12, 22, 2, 0), mk(13, 23, 3, 0)});
        check(!unassigned.feasible && any_reason_contains(unassigned.violation_reasons, "is not a drone of U"),
              "F: an unassigned drone id (-1) is rejected");
        for (const SolutionEvaluationResult* rr : {&too_big, &unassigned}) {
            check(!any_reason_contains(rr->violation_reasons, "constraint (28)") &&
                      !any_reason_contains(rr->violation_reasons, "simultaneously in flight") &&
                      !any_reason_contains(rr->violation_reasons, "constraint (6)"),
                  "F: only the invalid id (and its customer's cascade) is reported -- no (28), |U| or FLP violation");
        }
    }

    // G. More physical drones simultaneously in flight than |U|: three
    // trips all recovered at C3 (position 6), launched at positions 1, 3
    // and 5 -- at position 5 all three are airborne, |U| = 2.
    {
        SolutionEvaluationResult r = drone_layout_eval(2, {mk(11, 21, 3, 0), mk(12, 22, 3, 1), mk(13, 23, 3, 1)});
        check(!r.feasible, "G: three simultaneously airborne drones with |U|=2 is rejected");
        check(r.max_simultaneous_drones == 3, "G: the evaluator counts 3 drones in flight at once");
        check(any_reason_contains(r.violation_reasons, "3 drones are simultaneously in flight but |U| = 2"),
              "G: violation reports the |U| overflow");
    }

    // Shared layout for H-K (launch point placement): depot 0=(0,0),
    // C1=(10,0), C2=(20,10), C3=(30,0), C4=(20,-10); f100=(15,5), the
    // midpoint of arc C1->C2, i.e. f100 in L_(1,2). speeds 1, all times 0.
    auto flp_layout_eval = [](const TruckRoute& route, const std::vector<DroneTrip>& trips, const FlpDataMap& data) {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{20.0, 10.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{30.0, 0.0}});
        nodes.add(Node{4, NodeType::Customer, Point2D{20.0, -10.0}});
        nodes.add(Node{100, NodeType::FlexibleLaunchPoint, Point2D{15.0, 5.0}});
        nodes.add(Node{101, NodeType::FlexibleLaunchPoint, Point2D{12.0, 2.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 1e6; drone.num_drones = 2;
        SpatialNetwork network(nodes, truck, drone);
        Solution solution; solution.truck_route = route; solution.drone_trips = trips;
        std::unordered_map<NodeId, CustomerData> customers;
        for (NodeId c : {1, 2, 3, 4}) customers[c] = CustomerData{c, 0.0, 0.0, 0.0};
        return evaluate_solution(solution, network, customers, data);
    };
    const FlpDataMap f100_on_12 = flps({{100, 1, 2}});

    // H. FLP from an untraversed arc (p9: "If the truck does not pass
    // through arc (i,j), it cannot access the launch point in set L_ij";
    // (13)-(15)). Customer 2 is drone-served, so the truck no longer
    // drives C1->C2, yet f100 in L_(1,2) is used: route 0->1->f100->3->4->0.
    {
        SolutionEvaluationResult r = flp_layout_eval({0, 1, 100, 3, 4, 0}, {mk(100, 2, 3, 0)}, f100_on_12);
        check(!r.feasible, "H: an FLP of L_(1,2) used while the truck does not traverse (1,2) is rejected");
        check(any_reason_contains(r.violation_reasons, "belongs to L_(1,2) but the truck route passes it between 1 and 3"),
              "H: violation names the owning arc and the actual neighbours");
        // Control: the same launch point on its own arc is accepted.
        SolutionEvaluationResult ok = flp_layout_eval({0, 1, 100, 2, 3, 0}, {mk(100, 4, 3, 0)}, f100_on_12);
        check(ok.feasible, "H (control): f100 between C1 and C2, i.e. on its own arc (1,2), is accepted");
    }

    // I. FLP between two unrelated truck nodes: f100 in L_(1,2) placed
    // between C3 and C4 (arc (3,4) shares no endpoint with (1,2)).
    {
        SolutionEvaluationResult r = flp_layout_eval({0, 1, 3, 100, 4, 0}, {mk(100, 2, 4, 0)}, f100_on_12);
        check(!r.feasible, "I: an FLP of L_(1,2) placed between C3 and C4 is rejected");
        check(any_reason_contains(r.violation_reasons, "passes it between 3 and 4"), "I: violation reports the unrelated neighbours");
        SolutionEvaluationResult nodata = flp_layout_eval({0, 1, 100, 2, 3, 0}, {mk(100, 4, 3, 0)}, FlpDataMap{});
        check(!nodata.feasible && any_reason_contains(nodata.violation_reasons, "no arc-ownership data"),
              "I: an FLP without arc-ownership data is rejected (membership cannot be verified)");
    }

    // J. Idle FLP: f100 visited on its own arc but no drone launches there
    // (constraint (6): a visited launch point must launch >= 1 drone).
    {
        SolutionEvaluationResult r = flp_layout_eval({0, 1, 100, 2, 3, 4, 0}, {}, f100_on_12);
        check(!r.feasible, "J: an idle FLP on the truck route is rejected");
        check(any_reason_contains(r.violation_reasons, "constraint (6)"), "J: violation names constraint (6)");
    }

    // K. Repeated FLP (constraints (8)-(9): each launch point entered and
    // left at most once): f100 appears twice in a row on its own arc.
    // Control: two DIFFERENT launch points of the same arc in a row pass
    // the arc-membership check (both lie between C1 and C2); the second one
    // launches nothing, so only constraint (6) fires.
    {
        SolutionEvaluationResult r = flp_layout_eval({0, 1, 100, 100, 2, 3, 0}, {mk(100, 4, 3, 0)}, f100_on_12);
        check(!r.feasible, "K: an FLP visited twice is rejected");
        check(any_reason_contains(r.violation_reasons, "appears more than once"), "K: violation names constraints (8)-(9)");
        SolutionEvaluationResult same_arc = flp_layout_eval({0, 1, 101, 100, 2, 3, 0}, {mk(101, 4, 3, 0)},
                                                            flps({{100, 1, 2}, {101, 1, 2}}));
        check(!same_arc.feasible && any_reason_contains(same_arc.violation_reasons, "constraint (6)") &&
                  !any_reason_contains(same_arc.violation_reasons, "belongs to L_"),
              "K (control): consecutive FLPs of the same arc pass the arc check (only the idle one fails (6))");
    }

    // =====================================================================
    // STAGE 2 (8 Nov 2026): SR TIMING, OPTION B (decision D1).
    //     l_k = max( t_k + Se^T_k + r_k*SR ,  max_{u in R_k} (a_u + SR) )
    // with r_k drones recovered at k and a_u each drone's own arrival.
    // SR is NEVER charged at a launch FLP (there l_i = t_i + n_i*SL).
    // Every expected value below was derived by hand (shown in comments)
    // and cross-checked against the pre-Stage-2 rule where it differs.
    //
    // Layout A (truck speed 1, theta = 0, Se2 = 0, endurance 1e6):
    //   depot 0=(0,0); C1=(12,0) truck-served; C2=(6,8), C3=(9,4),
    //   C4=(6,-8) drone-served; FLP 11=(6,0) on arc (0,1).
    //   drone legs: 11->C2 = 8, C2->C1 = 10 (6-8-10); 11->C4 = 8, C4->C1 = 10;
    //   11->C3 = 5, C3->C1 = 5 (3-4-5); C2->depot = 10.
    //   Route 0,11,1,0: t(11) = 6, FLP -> C1 = 6, C1 -> depot = 12.
    // =====================================================================
    {
        const std::unordered_map<NodeId, Point2D> tblA = {
            {0, {0.0, 0.0}}, {1, {12.0, 0.0}}, {2, {6.0, 8.0}}, {3, {9.0, 4.0}}, {4, {6.0, -8.0}}, {11, {6.0, 0.0}}};
        auto mk = [](NodeId i, NodeId j, NodeId k, int u) {
            DroneTrip t; t.launch_node = i; t.service_customer = j; t.recovery_node = k; t.drone_id = u; return t;
        };
        // cust = {id, Se1, Se2}; every network customer gets data (Stage 1 rule).
        auto s2 = [&](const std::unordered_map<NodeId, Point2D>& tbl,
                      const std::vector<std::tuple<NodeId, double, double>>& cust, const std::vector<NodeId>& flp_ids,
                      const TruckRoute& route, const std::vector<DroneTrip>& trips, double truck_speed,
                      double vd, double SL, double SR, int U, const FlpDataMap& fm) {
            NodeRegistry nodes;
            nodes.add(Node{0, NodeType::Depot, tbl.at(0)});
            std::unordered_map<NodeId, CustomerData> customers;
            for (const auto& [id, se1, se2] : cust) {
                nodes.add(Node{id, NodeType::Customer, tbl.at(id)});
                customers[id] = CustomerData{id, se1, se2, 0.0};
            }
            for (NodeId f : flp_ids) nodes.add(Node{f, NodeType::FlexibleLaunchPoint, tbl.at(f)});
            TruckParameters truck; truck.speed = truck_speed;
            DroneParameters drone; drone.speed = vd; drone.launch_prep_time_sl = SL; drone.recovery_time_sr = SR;
            drone.endurance_seconds = 1e6; drone.num_drones = U;
            SpatialNetwork network(nodes, truck, drone);
            Solution solution; solution.truck_route = route; solution.drone_trips = trips;
            return evaluate_solution(solution, network, customers, fm);
        };
        auto drone_never_exceeds_truck = [&](const SolutionEvaluationResult& r, const DroneTrip& t, const TruckRoute& route, std::size_t pos) {
            (void)t; (void)route;
            bool ok = true;
            for (const auto& tt : r.drone_trip_timings) ok = ok && tt.recovery_complete_time <= r.truck_timing.departure_time[pos] + 1e-9;
            return ok;
        };
        const FlpDataMap fA = flps({{11, 0, 1}});
        const TruckRoute rA = {0, 11, 1, 0};

        // ---- S2-1. Single drone recovery; truck arrives first and waits; SR NOT at launch.
        // vd=1, SL=2, SR=3, Se1(C1)=0.  t(11)=6, launch=8, dep(11)=6+1*2=8, t(C1)=8+6=14.
        // Drone: 11->C2 8 -> cust arrival 16; C2->C1 10 -> a = 26.
        // l(C1) = max(14+0+1*3 = 17, 26+3 = 29) = 29.   [old rule: max(14, 29) = 29]
        // t(depot) = 29+12 = 41.  objective = 14 + 16 = 30.
        // recovery_complete_time (drone free) = max(14,26)+3 = 29.
        // sync: truck waited 26-14 = 12.
        {
            auto r = s2(tblA, {{1, 0.0, 0.0}, {2, 0.0, 0.0}}, {11}, rA, {mk(11, 2, 1, 0)}, 1.0, 1.0, 2.0, 3.0, 1, fA);
            check(r.feasible, "S2-1: single-drone recovery solution is feasible");
            check_near(r.truck_timing.arrival_time[1], 6.0, 1e-9, "S2-1: truck arrives at the launch FLP at 6");
            check_near(r.drone_trip_timings[0].launch_time, 8.0, 1e-9, "S2-1: launch = t_i + SL = 8");
            check_near(r.truck_timing.departure_time[1], 8.0, 1e-9, "S2-1: departure at the launch FLP = t_i + n_i*SL = 8 (no SR at launch)");
            check_near(r.truck_timing.arrival_time[2], 14.0, 1e-9, "S2-1: truck arrives at C1 at 14");
            check_near(r.drone_trip_timings[0].recovery_arrival_time, 26.0, 1e-9, "S2-1: drone arrives at C1 at a_u = 26");
            check_near(r.truck_timing.departure_time[2], 29.0, 1e-9, "S2-1: l(C1) = max(14+3, 26+3) = 29");
            check_near(r.drone_trip_timings[0].recovery_complete_time, 29.0, 1e-9, "S2-1: drone free at max(14,26)+3 = 29");
            check_near(r.truck_timing.arrival_time[3], 41.0, 1e-9, "S2-1: truck returns to the depot at 29+12 = 41");
            check_near(r.objective_value, 30.0, 1e-9, "S2-1: objective = 14 + 16 = 30");
            check(r.synchronization_records.size() == 1 && r.synchronization_records[0].truck_waited &&
                      std::fabs(r.synchronization_records[0].wait_time - 12.0) < 1e-9,
                  "S2-1: truck waits 12 for the drone");
        }

        // ---- S2-10. SR is not charged at the launch FLP: vary SR, hold everything else.
        // Same layout as S2-1 with SR = 0 and SR = 100.
        //   SR=0:   l(C1) = max(14, 26)       = 26
        //   SR=100: l(C1) = max(14+100, 126)  = 126
        // while arrival/departure/launch at the FLP are 6 / 8 / 8 in both.
        {
            auto r0 = s2(tblA, {{1, 0.0, 0.0}, {2, 0.0, 0.0}}, {11}, rA, {mk(11, 2, 1, 0)}, 1.0, 1.0, 2.0, 0.0, 1, fA);
            auto r1 = s2(tblA, {{1, 0.0, 0.0}, {2, 0.0, 0.0}}, {11}, rA, {mk(11, 2, 1, 0)}, 1.0, 1.0, 2.0, 100.0, 1, fA);
            check_near(r0.truck_timing.departure_time[1], 8.0, 1e-9, "S2-10: SR=0: departure at the launch FLP = 8");
            check_near(r1.truck_timing.departure_time[1], 8.0, 1e-9, "S2-10: SR=100: departure at the launch FLP is STILL 8");
            check_near(r1.drone_trip_timings[0].launch_time, r0.drone_trip_timings[0].launch_time, 1e-12,
                       "S2-10: launch time is independent of SR");
            check_near(r0.truck_timing.departure_time[2], 26.0, 1e-9, "S2-10: SR=0: l(C1) = max(14, 26) = 26");
            check_near(r1.truck_timing.departure_time[2], 126.0, 1e-9, "S2-10: SR=100: l(C1) = max(114, 126) = 126");
        }

        // ---- S2-2. Two drones recovered at the same node (r=2), truck service Se1=4, drones early.
        // vd=10, SL=1, SR=3, |U|=2, trips 11->C2->C1 (drone 0), 11->C4->C1 (drone 1).
        // n_i=2: dep(11) = 6+2*1 = 8, BOTH launch at 6+1 = 7 (no staggering). t(C1) = 8+6 = 14.
        // Each drone: out 8/10 = 0.8 -> cust arrival 7.8; back 10/10 = 1.0 -> a_u = 8.8.
        // l(C1) = max(14 + 4 + 2*3 = 24, 8.8+3 = 11.8) = 24.   [old rule: max(18, 17) = 18]
        // t(depot) = 24+12 = 36.  objective = 14 + 7.8 + 7.8 = 29.6.
        // Each drone's own free time max(14, 8.8)+3 = 17 <= 24.
        {
            auto r = s2(tblA, {{1, 4.0, 0.0}, {2, 0.0, 0.0}, {4, 0.0, 0.0}}, {11}, rA,
                        {mk(11, 2, 1, 0), mk(11, 4, 1, 1)}, 1.0, 10.0, 1.0, 3.0, 2, fA);
            check(r.feasible, "S2-2: two distinct drones recovered at one node is feasible");
            check(r.max_simultaneous_drones == 2, "S2-2: two drones in flight");
            check_near(r.drone_trip_timings[0].launch_time, 7.0, 1e-9, "S2-2: drone 0 launches at t+SL = 7");
            check_near(r.drone_trip_timings[1].launch_time, 7.0, 1e-9, "S2-2: drone 1 also launches at 7 (no staggering)");
            check_near(r.truck_timing.departure_time[1], 8.0, 1e-9, "S2-2: departure at the FLP = 6 + 2*SL = 8");
            check_near(r.truck_timing.arrival_time[2], 14.0, 1e-9, "S2-2: t(C1) = 14");
            check_near(r.drone_trip_timings[0].recovery_arrival_time, 8.8, 1e-9, "S2-2: a_u(drone 0) = 8.8");
            check_near(r.drone_trip_timings[1].recovery_arrival_time, 8.8, 1e-9, "S2-2: a_u(drone 1) = 8.8");
            check_near(r.truck_timing.departure_time[2], 24.0, 1e-9, "S2-2: l(C1) = 14 + Se1 4 + r*SR (2*3) = 24");
            check_near(r.drone_trip_timings[0].recovery_complete_time, 17.0, 1e-9, "S2-2: drone 0 free at max(14, 8.8)+3 = 17");
            check_near(r.truck_timing.arrival_time[3], 36.0, 1e-9, "S2-2: depot at 24+12 = 36");
            check_near(r.objective_value, 29.6, 1e-9, "S2-2: objective = 14 + 7.8 + 7.8 = 29.6");
            check(drone_never_exceeds_truck(r, mk(0, 0, 0, 0), rA, 2), "S2-2: both drones are free no later than the truck departs");
        }

        // ---- S2-3 / S2-4. One drone, truck service + recovery service; drone arrives BEFORE the truck and waits.
        // vd=10, SL=1, SR=3, Se1(C1)=4, trip 11->C2->C1.  dep(11) = 6+1 = 7, launch 7, t(C1) = 7+6 = 13.
        // cust arrival 7.8, a_u = 8.8  (< 13: drone waits 13-8.8 = 4.2).
        // l(C1) = max(13 + 4 + 1*3 = 20, 8.8+3 = 11.8) = 20.   [old rule: max(17, 16) = 17]
        // objective = 13 + 7.8 = 20.8.  t(depot) = 32.
        {
            auto r = s2(tblA, {{1, 4.0, 0.0}, {2, 0.0, 0.0}}, {11}, rA, {mk(11, 2, 1, 0)}, 1.0, 10.0, 1.0, 3.0, 1, fA);
            check(r.feasible, "S2-3: truck service plus recovery service is feasible");
            check_near(r.truck_timing.arrival_time[2], 13.0, 1e-9, "S2-3: t(C1) = 13");
            check_near(r.drone_trip_timings[0].recovery_arrival_time, 8.8, 1e-9, "S2-3: a_u = 8.8");
            check_near(r.truck_timing.departure_time[2], 20.0, 1e-9, "S2-3: l(C1) = t + Se1 + SR = 13 + 4 + 3 = 20 (SR is additive to service)");
            check_near(r.truck_timing.arrival_time[3], 32.0, 1e-9, "S2-3: depot at 20+12 = 32");
            check_near(r.objective_value, 20.8, 1e-9, "S2-3: objective = 13 + 7.8 = 20.8");
            check(r.synchronization_records.size() == 1 && r.synchronization_records[0].drone_waited &&
                      !r.synchronization_records[0].truck_waited &&
                      std::fabs(r.synchronization_records[0].wait_time - 4.2) < 1e-9,
                  "S2-4: the drone arrives first and waits 13 - 8.8 = 4.2");
        }

        // ---- S2-5. Truck arrives BEFORE the drone and waits; the drone term dominates.
        // vd=1, SL=1, SR=3, Se1(C1)=2.  dep(11) = 7, launch 7, t(C1) = 13.
        // out 8 -> cust arrival 15; back 10 -> a_u = 25.
        // l(C1) = max(13 + 2 + 3 = 18, 25+3 = 28) = 28.   [old rule: 28]
        // objective = 13 + 15 = 28.
        {
            auto r = s2(tblA, {{1, 2.0, 0.0}, {2, 0.0, 0.0}}, {11}, rA, {mk(11, 2, 1, 0)}, 1.0, 1.0, 1.0, 3.0, 1, fA);
            check(r.feasible, "S2-5: truck-arrives-first solution is feasible");
            check_near(r.drone_trip_timings[0].recovery_arrival_time, 25.0, 1e-9, "S2-5: a_u = 25");
            check_near(r.truck_timing.departure_time[2], 28.0, 1e-9, "S2-5: l(C1) = max(13+2+3, 25+3) = 28 (the drone's completion binds)");
            check_near(r.objective_value, 28.0, 1e-9, "S2-5: objective = 13 + 15 = 28");
            check(r.synchronization_records[0].truck_waited && std::fabs(r.synchronization_records[0].wait_time - 12.0) < 1e-9,
                  "S2-5: the truck waits 25 - 13 = 12");
        }

        // ---- S2-6. Several recoveries with DIFFERENT drone arrival times.
        // vd=1, SL=1, |U|=2, trips 11->C2->C1 (drone 0), 11->C3->C1 (drone 1), Se2=0.
        // launch both 7, dep(11) = 8, t(C1) = 14.
        // trip C2: out 8 -> 15, back 10 -> a = 25.   trip C3: out 5 -> 12, back 5 -> a = 17.
        // objective = 14 + 15 + 12 = 41 for every SR / Se1 below (SR never moves it here).
        //   (i)   Se1=0,  SR=4:  max(14+0+2*4 = 22, max(25,17)+4 = 29) = 29   [old: 29]
        //   (ii)  Se1=10, SR=4:  max(14+10+8 = 32, 29)                  = 32   [old: max(24,29) = 29]
        //   (iii) Se1=0,  SR=12: max(14+0+24 = 38, 25+12 = 37)          = 38   [old: 37]
        struct S26 { double se1; double sr; double want_l; const char* tag; };
        const S26 cases26[] = {{0.0, 4.0, 29.0, "S2-6(i)"}, {10.0, 4.0, 32.0, "S2-6(ii)"}, {0.0, 12.0, 38.0, "S2-6(iii)"}};
        for (const auto& c : cases26) {
            auto r = s2(tblA, {{1, c.se1, 0.0}, {2, 0.0, 0.0}, {3, 0.0, 0.0}}, {11}, rA,
                        {mk(11, 2, 1, 0), mk(11, 3, 1, 1)}, 1.0, 1.0, 1.0, c.sr, 2, fA);
            const std::string tag = c.tag;
            check(r.feasible, tag + ": feasible");
            check_near(r.drone_trip_timings[0].recovery_arrival_time, 25.0, 1e-9, tag + ": a_u(C2 trip) = 25");
            check_near(r.drone_trip_timings[1].recovery_arrival_time, 17.0, 1e-9, tag + ": a_u(C3 trip) = 17");
            check_near(r.truck_timing.departure_time[2], c.want_l, 1e-9, tag + ": l(C1) = max(t+Se1+2*SR, latest a_u + SR)");
            check_near(r.objective_value, 41.0, 1e-9, tag + ": objective 14 + 15 + 12 = 41 is independent of SR here");
            check_near(r.drone_trip_timings[0].recovery_complete_time, 25.0 + c.sr, 1e-9, tag + ": late drone free at max(14,25)+SR");
            check_near(r.drone_trip_timings[1].recovery_complete_time, 17.0 + c.sr, 1e-9, tag + ": early drone free at max(14,17)+SR");
            check(drone_never_exceeds_truck(r, mk(0, 0, 0, 0), rA, 2), tag + ": every drone is free no later than the truck departs");
        }

        // ---- S2-7. Recovery at the ENDING depot (Se^T = 0 there; its departure is reported, not in the objective).
        // trip 11->C2->depot, SL=1, SR=5, Se1(C1)=0.  dep(11)=7, launch 7, t(C1) = 13, t(depot) = 13+12 = 25.
        //   (a) vd=0.5: out 16 -> cust arrival 23; back (6,8)->(0,0)=10 / 0.5 = 20 -> a_u = 43.
        //       l(depot) = max(25+0+5 = 30, 43+5 = 48) = 48.   objective = 13 + 23 = 36.
        //   (b) vd=4:   out 2 -> 9; back 2.5 -> a_u = 11.5.
        //       l(depot) = max(25+5 = 30, 11.5+5 = 16.5) = 30. objective = 13 + 9 = 22.
        {
            auto ra = s2(tblA, {{1, 0.0, 0.0}, {2, 0.0, 0.0}}, {11}, rA, {mk(11, 2, 0, 0)}, 1.0, 0.5, 1.0, 5.0, 1, fA);
            check(ra.feasible, "S2-7(a): drone recovered at the ending depot is feasible");
            check_near(ra.truck_timing.arrival_time[3], 25.0, 1e-9, "S2-7(a): truck reaches the depot at 25");
            check_near(ra.drone_trip_timings[0].recovery_arrival_time, 43.0, 1e-9, "S2-7(a): a_u = 43");
            check_near(ra.truck_timing.departure_time[3], 48.0, 1e-9, "S2-7(a): l(depot) = max(25+5, 43+5) = 48");
            check_near(ra.objective_value, 36.0, 1e-9, "S2-7(a): objective = 13 + 23 = 36 (end-depot time excluded)");
            auto rb = s2(tblA, {{1, 0.0, 0.0}, {2, 0.0, 0.0}}, {11}, rA, {mk(11, 2, 0, 0)}, 1.0, 4.0, 1.0, 5.0, 1, fA);
            check(rb.feasible, "S2-7(b): drone recovered at the depot, drone early, is feasible");
            check_near(rb.drone_trip_timings[0].recovery_arrival_time, 11.5, 1e-9, "S2-7(b): a_u = 11.5");
            check_near(rb.truck_timing.departure_time[3], 30.0, 1e-9, "S2-7(b): l(depot) = max(25+5, 11.5+5) = 30 (truck-side term binds)");
            check_near(rb.objective_value, 22.0, 1e-9, "S2-7(b): objective = 13 + 9 = 22");
        }
    }

    // Layout B (truck speed 1, theta = 0, Se2 = 0, endurance 1e6):
    //   depot 0=(0,0); C1=(10,0), C2=(5,12), C3=(30,0), C4=(18,5), C5=(18,-5);
    //   FLP 11=(5,0) on arc (0,1); FLP 12=(18,0) on arc (1,3).
    {
        const std::unordered_map<NodeId, Point2D> tblB = {
            {0, {0.0, 0.0}}, {1, {10.0, 0.0}}, {2, {5.0, 12.0}}, {3, {30.0, 0.0}}, {4, {18.0, 5.0}}, {5, {18.0, -5.0}},
            {11, {5.0, 0.0}}, {12, {18.0, 0.0}}};
        auto mk = [](NodeId i, NodeId j, NodeId k, int u) {
            DroneTrip t; t.launch_node = i; t.service_customer = j; t.recovery_node = k; t.drone_id = u; return t;
        };
        auto s2b = [&](const std::vector<std::tuple<NodeId, double, double>>& cust, const std::vector<NodeId>& flp_ids,
                       const TruckRoute& route, const std::vector<DroneTrip>& trips, double vd, double SL, double SR, int U) {
            NodeRegistry nodes;
            nodes.add(Node{0, NodeType::Depot, tblB.at(0)});
            std::unordered_map<NodeId, CustomerData> customers;
            for (const auto& [id, se1, se2] : cust) {
                nodes.add(Node{id, NodeType::Customer, tblB.at(id)});
                customers[id] = CustomerData{id, se1, se2, 0.0};
            }
            for (NodeId f : flp_ids) nodes.add(Node{f, NodeType::FlexibleLaunchPoint, tblB.at(f)});
            TruckParameters truck; truck.speed = 1.0;
            DroneParameters drone; drone.speed = vd; drone.launch_prep_time_sl = SL; drone.recovery_time_sr = SR;
            drone.endurance_seconds = 1e6; drone.num_drones = U;
            SpatialNetwork network(nodes, truck, drone);
            Solution solution; solution.truck_route = route; solution.drone_trips = trips;
            return evaluate_solution(solution, network, customers, flps({{11, 0, 1}, {12, 1, 3}}));
        };

        // ---- S2-8. Sequential trips by the SAME physical drone; SR propagates to later customers.
        // Route 0,11,1,12,3,0; trip A 11->C2->C1 and trip B 12->C4->C3, both drone 0, |U|=1.
        // vd=5, SL=0, SR=2, Se1(C1)=3, Se1(C3)=0.
        //   t(11)=5, launch A = 5, dep(11)=5, t(C1)=5+5=10.
        //   A: 11->C2 = 12 -> 2.4 -> cust arrival 7.4; C2->C1 = 13 -> 2.6 -> a = 10.0.
        //   l(C1) = max(10+3+1*2 = 15, 10+2 = 12) = 15.            [old rule: max(13, 12) = 13]
        //   t(12) = 15+8 = 23, launch B = 23, dep(12) = 23, t(C3) = 23+12 = 35.
        //   B: 12->C4 = 5 -> 1.0 -> cust arrival 24; C4->C3 = 13 -> 2.6 -> a = 26.6.
        //   l(C3) = max(35+0+2 = 37, 26.6+2 = 28.6) = 37.          t(depot) = 37+30 = 67.
        //   objective = C1 10 + C3 35 + C2 7.4 + C4 24 = 76.4.
        //   [old rule: l(C1)=13, t(12)=21, C4 at 22, t(C3)=33 -> objective 72.4:
        //    SR at the first recovery delays everything after it.]
        {
            auto r = s2b({{1, 3.0, 0.0}, {2, 0.0, 0.0}, {3, 0.0, 0.0}, {4, 0.0, 0.0}}, {11, 12}, {0, 11, 1, 12, 3, 0},
                         {mk(11, 2, 1, 0), mk(12, 4, 3, 0)}, 5.0, 0.0, 2.0, 1);
            check(r.feasible, "S2-8: one physical drone flying two sequential trips is feasible");
            check(r.max_simultaneous_drones == 1, "S2-8: one drone in flight at a time");
            check_near(r.drone_trip_timings[0].recovery_arrival_time, 10.0, 1e-9, "S2-8: trip A lands at a = 10");
            check_near(r.truck_timing.departure_time[2], 15.0, 1e-9, "S2-8: l(C1) = 10 + 3 + 2 = 15");
            check_near(r.drone_trip_timings[1].launch_time, 23.0, 1e-9, "S2-8: trip B launches at t(12) = 23 (delayed by SR)");
            check_near(r.drone_trip_timings[1].customer_arrival_time, 24.0, 1e-9, "S2-8: C4 reached at 24");
            check_near(r.truck_timing.arrival_time[4], 35.0, 1e-9, "S2-8: t(C3) = 35");
            check_near(r.truck_timing.departure_time[4], 37.0, 1e-9, "S2-8: l(C3) = 35 + 0 + 2 = 37");
            check_near(r.truck_timing.arrival_time[5], 67.0, 1e-9, "S2-8: depot at 37+30 = 67");
            check_near(r.objective_value, 76.4, 1e-9, "S2-8: objective = 10 + 35 + 7.4 + 24 = 76.4");
        }

        // ---- S2-9. Simultaneous trips by DISTINCT drones, SL and SR both nonzero.
        // Route 0,1,12,3,0; trips 12->C4->C3 (drone 0), 12->C5->C3 (drone 1). vd=1, SL=1, SR=4, Se1(C3)=2.
        //   t(C1)=10, dep(C1)=10 (Se1=0), t(12) = 18.
        //   BOTH launch at 18+1 = 19 (no staggering); dep(12) = 18 + 2*1 = 20 (no SR at launch).
        //   t(C3) = 20+12 = 32.   drones: out 5 -> 24; back 13 -> a = 37.
        //   l(C3) = max(32 + 2 + 2*4 = 42, 37+4 = 41) = 42.   [old rule: max(34, 41) = 41]
        //   t(depot) = 42+30 = 72.   objective = 10 + 32 + 24 + 24 = 90.
        {
            auto r = s2b({{1, 0.0, 0.0}, {3, 2.0, 0.0}, {4, 0.0, 0.0}, {5, 0.0, 0.0}}, {12}, {0, 1, 12, 3, 0},
                         {mk(12, 4, 3, 0), mk(12, 5, 3, 1)}, 1.0, 1.0, 4.0, 2);
            check(r.feasible, "S2-9: two distinct drones in flight simultaneously, SL and SR nonzero, is feasible");
            check(r.max_simultaneous_drones == 2, "S2-9: two drones in flight");
            check_near(r.truck_timing.arrival_time[2], 18.0, 1e-9, "S2-9: t(12) = 18");
            check_near(r.drone_trip_timings[0].launch_time, 19.0, 1e-9, "S2-9: drone 0 launches at 19");
            check_near(r.drone_trip_timings[1].launch_time, 19.0, 1e-9, "S2-9: drone 1 launches at 19 (no staggering)");
            check_near(r.truck_timing.departure_time[2], 20.0, 1e-9, "S2-9: departure at the FLP = 18 + 2*SL = 20 (no SR at launch)");
            check_near(r.truck_timing.arrival_time[3], 32.0, 1e-9, "S2-9: t(C3) = 32");
            check_near(r.drone_trip_timings[0].recovery_arrival_time, 37.0, 1e-9, "S2-9: a_u = 37 for both");
            check_near(r.truck_timing.departure_time[3], 42.0, 1e-9, "S2-9: l(C3) = max(32+2+2*4, 37+4) = 42");
            check_near(r.truck_timing.arrival_time[4], 72.0, 1e-9, "S2-9: depot at 42+30 = 72");
            check_near(r.objective_value, 90.0, 1e-9, "S2-9: objective = 10 + 32 + 24 + 24 = 90");
        }
    }

    // =====================================================================
    // STAGE 1 (8 Nov 2026): EVALUATOR INPUT VALIDATION (S1-a ... S1-l).
    // The evaluator must report a violation -- never throw, never accept --
    // for malformed routes and malformed instance data. Every scenario below
    // uses speeds 1, SL = SR = 0, all service times 0, theta = 0, so the
    // hand-computed times are simple distances. Nothing here touches the SR,
    // launch-scheduling, endurance or objective rules.
    //
    // Coordinates (3-4-5 / 5-12-13 families): depot 0 and a second depot 9 at
    // (0,0); C1=(10,0), C2=(5,12), C3=(30,0), C4=(18,5), C5=(18,-5);
    // FLP 11=(5,0) in L_(0,1), FLP 12=(18,0) in L_(1,3).
    // =====================================================================
    {
        struct S1Out { bool threw = false; SolutionEvaluationResult r; };
        const std::unordered_map<NodeId, Point2D> pos = {
            {1, {10.0, 0.0}}, {2, {5.0, 12.0}}, {3, {30.0, 0.0}}, {4, {18.0, 5.0}}, {5, {18.0, -5.0}}};
        auto s1_eval = [&](const std::vector<NodeId>& cust, const TruckRoute& route, const std::vector<DroneTrip>& trips,
                           int num_drones, const std::vector<NodeId>& skip_data,
                           const std::unordered_map<NodeId, CustomerData>& extra_data) {
            NodeRegistry nodes;
            nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
            nodes.add(Node{9, NodeType::Depot, Point2D{0.0, 0.0}});
            for (NodeId c : cust) nodes.add(Node{c, NodeType::Customer, pos.at(c)});
            nodes.add(Node{11, NodeType::FlexibleLaunchPoint, Point2D{5.0, 0.0}});
            nodes.add(Node{12, NodeType::FlexibleLaunchPoint, Point2D{18.0, 0.0}});
            TruckParameters truck; truck.speed = 1.0;
            DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 1e6; drone.num_drones = num_drones;
            SpatialNetwork network(nodes, truck, drone);
            Solution solution; solution.truck_route = route; solution.drone_trips = trips;
            std::unordered_map<NodeId, CustomerData> customers;
            for (NodeId c : cust) {
                bool skipped = false;
                for (NodeId k : skip_data) skipped = skipped || (k == c);
                if (!skipped) customers[c] = CustomerData{c, 0.0, 0.0, 0.0};
            }
            for (const auto& kv : extra_data) customers[kv.first] = kv.second;
            S1Out out;
            try {
                out.r = evaluate_solution(solution, network, customers, flps({{11, 0, 1}, {12, 1, 3}}));
            } catch (const std::exception&) {
                out.threw = true;
            }
            return out;
        };
        const std::unordered_map<NodeId, CustomerData> no_extra;
        auto trip = [](NodeId i, NodeId j, NodeId k, int u) {
            DroneTrip t; t.launch_node = i; t.service_customer = j; t.recovery_node = k; t.drone_id = u; return t;
        };

        // S1-a. Control: a valid truck-only route. 0 -> C1 (10) -> C3 (20 more) -> 0.
        {
            S1Out o = s1_eval({1, 3}, {0, 1, 3, 0}, {}, 2, {}, no_extra);
            check(!o.threw && o.r.feasible, "S1-a (control): valid truck-only route is feasible");
            check_near(o.r.objective_value, 10.0 + 30.0, 1e-9, "S1-a (control): objective = 10 + 30 = 40");
        }

        // S1-b. Repeated truck customer.
        {
            S1Out o = s1_eval({1, 3}, {0, 1, 3, 1, 0}, {}, 2, {}, no_extra);
            check(!o.threw && !o.r.feasible, "S1-b: a customer visited twice by the truck is infeasible and does not throw");
            check(any_reason_contains(o.r.violation_reasons, "customer 1 appears more than once"),
                  "S1-b: violation names the repeated customer");
        }

        // S1-c. Depot in the middle of the route.
        {
            S1Out o = s1_eval({1, 3}, {0, 1, 0, 3, 0}, {}, 2, {}, no_extra);
            check(!o.threw && !o.r.feasible, "S1-c: a depot visit in the middle of the route is infeasible and does not throw");
            check(any_reason_contains(o.r.violation_reasons, "depot node 0 appears at interior route position 2"),
                  "S1-c: violation names the interior depot position");
        }

        // S1-d. A different depot node at the end; control: ending at a customer.
        {
            S1Out o = s1_eval({1, 3}, {0, 1, 3, 9}, {}, 2, {}, no_extra);
            check(!o.threw && !o.r.feasible, "S1-d: start depot 0 and end depot 9 is infeasible and does not throw");
            check(any_reason_contains(o.r.violation_reasons, "starts at depot 0 but ends at a different depot 9"),
                  "S1-d: violation names both depots");
            S1Out c = s1_eval({1, 3}, {0, 1, 3}, {}, 2, {}, no_extra);
            check(!c.threw && !c.r.feasible && any_reason_contains(c.r.violation_reasons, "does not end at a depot node"),
                  "S1-d (control): a route ending at a customer is rejected");
        }

        // S1-e. Unknown node id in the route (middle and first position) and in a drone trip.
        {
            S1Out mid = s1_eval({1, 3}, {0, 1, 99, 3, 0}, {}, 2, {}, no_extra);
            check(!mid.threw && !mid.r.feasible, "S1-e: unknown node id 99 in the route is infeasible and does not throw");
            check(any_reason_contains(mid.r.violation_reasons, "route position 2 refers to node id 99"),
                  "S1-e: violation names the position and the unknown id");
            S1Out first = s1_eval({1, 3}, {99, 1, 3, 0}, {}, 2, {}, no_extra);
            check(!first.threw && !first.r.feasible, "S1-e: unknown id at route position 0 does not throw");
            S1Out tr = s1_eval({1, 2, 3}, {0, 11, 1, 3, 0}, {trip(11, 2, 77, 0)}, 2, {}, no_extra);
            check(!tr.threw && !tr.r.feasible &&
                      any_reason_contains(tr.r.violation_reasons, "invalid launch/customer/recovery node id or role"),
                  "S1-e: a drone trip referencing unknown node 77 is rejected without throwing");
        }

        // S1-f. Missing CustomerData for a network customer.
        {
            S1Out o = s1_eval({1, 3}, {0, 1, 3, 0}, {}, 2, {3}, no_extra);
            check(!o.threw && !o.r.feasible, "S1-f: network customer 3 without CustomerData is infeasible, even though the truck visits it");
            check(any_reason_contains(o.r.violation_reasons, "network customer 3 has no CustomerData"),
                  "S1-f: violation names the customer without data");
            S1Out none = s1_eval({1, 3}, {0, 1, 3, 0}, {}, 2, {1, 3}, no_extra);
            check(!none.threw && !none.r.feasible &&
                      any_reason_contains(none.r.violation_reasons, "network customer 1 has no CustomerData") &&
                      any_reason_contains(none.r.violation_reasons, "network customer 3 has no CustomerData"),
                  "S1-f: an empty CustomerData map reports every network customer");
        }

        // S1-g. CustomerData referring to an invalid node.
        {
            std::unordered_map<NodeId, CustomerData> bad;
            bad[77] = CustomerData{77, 5.0, 0.0, 0.0};   // not a node of the network
            bad[0] = CustomerData{0, 5.0, 0.0, 0.0};     // a depot
            bad[11] = CustomerData{11, 5.0, 0.0, 0.0};   // a flexible launch point
            S1Out o = s1_eval({1, 3}, {0, 1, 3, 0}, {}, 2, {}, bad);
            check(!o.threw && !o.r.feasible, "S1-g: CustomerData for non-customer nodes is infeasible and does not throw");
            check(any_reason_contains(o.r.violation_reasons, "CustomerData entry 77 refers to a node id that is not in the network"),
                  "S1-g: entry for an unknown node id is reported");
            check(any_reason_contains(o.r.violation_reasons, "CustomerData entry 0 refers to a node that is a depot, not a customer"),
                  "S1-g: entry keyed by the depot is reported");
            check(any_reason_contains(o.r.violation_reasons, "CustomerData entry 11 refers to a node that is a flexible launch point, not a customer"),
                  "S1-g: entry keyed by an FLP is reported");
            check_near(o.r.truck_timing.departure_time[0], 0.0, 1e-9,
                       "S1-g: the invalid depot entry's service time (5) is NOT applied at the depot");
            check(o.r.customer_records.size() == 2, "S1-g: invalid entries create no customer-coverage records");
            std::unordered_map<NodeId, CustomerData> mism;
            mism[1] = CustomerData{5, 0.0, 0.0, 0.0};     // stored under 1 but says node 5
            S1Out m = s1_eval({1, 3}, {0, 1, 3, 0}, {}, 2, {}, mism);
            check(!m.threw && !m.r.feasible &&
                      any_reason_contains(m.r.violation_reasons, "stored under customer 1 has node_id 5"),
                  "S1-g: CustomerData whose node_id differs from its key is reported");
        }

        // S1-h. Customer 2 served by BOTH the truck and a drone trip.
        // Route 0 -> f11 -> C1 -> C2 -> C3 -> 0 with trip f11 -> C2 -> C3.
        {
            S1Out o = s1_eval({1, 2, 3}, {0, 11, 1, 2, 3, 0}, {trip(11, 2, 3, 0)}, 2, {}, no_extra);
            check(!o.threw && !o.r.feasible, "S1-h: a customer served by the truck AND a drone trip is infeasible");
            check(any_reason_contains(o.r.violation_reasons, "customer 2: customer is served by both the truck and at least one drone trip"),
                  "S1-h: violation names the customer");
        }

        // S1-i. Customer 2 served by TWO drone trips (f11 -> C2 -> C1 on drone 0, f12 -> C2 -> C3 on drone 1).
        {
            S1Out o = s1_eval({1, 2, 3}, {0, 11, 1, 12, 3, 0}, {trip(11, 2, 1, 0), trip(12, 2, 3, 1)}, 2, {}, no_extra);
            check(!o.threw && !o.r.feasible, "S1-i: a customer served by two drone trips is infeasible");
            check(any_reason_contains(o.r.violation_reasons, "customer 2: customer is served by more than one drone trip"),
                  "S1-i: violation names the customer");
        }

        // S1-j. VALID: ONE physical drone flies two sequential trips.
        // Route 0 -> f11 -> C1 -> f12 -> C3 -> 0 (positions 0..5);
        //   trip A: f11 -> C2 -> C1 (positions 1..2), trip B: f12 -> C4 -> C3 (positions 3..4), both drone 0.
        // Hand computation (speeds 1, all service times 0):
        //   t(f11) = 5, A launches at 5, reaches C2 at 5+12 = 17, lands at C1 at 17+13 = 30.
        //   truck reaches C1 at 10 and waits for the drone: l(C1) = 30.
        //   t(f12) = 30+8 = 38, B launches at 38, reaches C4 at 38+5 = 43, lands at C3 at 43+13 = 56.
        //   truck reaches C3 at 38+12 = 50.
        //   objective = C1 10 + C3 50 + C2 17 + C4 43 = 120.
        {
            S1Out o = s1_eval({1, 2, 3, 4}, {0, 11, 1, 12, 3, 0}, {trip(11, 2, 1, 0), trip(12, 4, 3, 0)}, 2, {}, no_extra);
            check(!o.threw && o.r.feasible, "S1-j: one physical drone flying two sequential trips is feasible");
            check(o.r.max_simultaneous_drones == 1, "S1-j: at most one drone is in flight at a time");
            check_near(o.r.objective_value, 120.0, 1e-9, "S1-j: hand-computed objective is 120");
            check_near(o.r.drone_trip_timings[0].customer_arrival_time, 17.0, 1e-9, "S1-j: trip A reaches C2 at 17");
            check_near(o.r.drone_trip_timings[1].customer_arrival_time, 43.0, 1e-9, "S1-j: trip B reaches C4 at 43");
            // Negative twin: the same two trips on |U| = 1 are fine, but relaunching drone 0 BEFORE its recovery is not.
            S1Out overlap = s1_eval({1, 2, 3, 4}, {0, 11, 12, 1, 3, 0}, {trip(11, 2, 1, 0), trip(12, 4, 3, 0)}, 2, {}, no_extra);
            check(!overlap.threw && !overlap.r.feasible, "S1-j (negative): the same drone relaunched at f12 before its recovery at C1 is rejected");
        }

        // S1-k. VALID: two distinct drones fly simultaneously.
        // Route 0 -> C1 -> f12 -> C3 -> 0; f12 launches C4 (drone 0) and C5 (drone 1), both recovered at C3.
        //   t(C1) = 10, t(f12) = 10+8 = 18, both drones launch at 18 (SL = 0), reach C4/C5 at 23,
        //   land at C3 at 23+13 = 36; the truck reaches C3 at 30 and waits for them.
        //   objective = C1 10 + C3 30 + C4 23 + C5 23 = 86.
        {
            S1Out o = s1_eval({1, 3, 4, 5}, {0, 1, 12, 3, 0}, {trip(12, 4, 3, 0), trip(12, 5, 3, 1)}, 2, {}, no_extra);
            check(!o.threw && o.r.feasible, "S1-k: two distinct drones in flight simultaneously are feasible with |U| = 2");
            check(o.r.max_simultaneous_drones == 2, "S1-k: the evaluator counts 2 drones in flight");
            check_near(o.r.objective_value, 86.0, 1e-9, "S1-k: hand-computed objective is 86");
            S1Out same = s1_eval({1, 3, 4, 5}, {0, 1, 12, 3, 0}, {trip(12, 4, 3, 0), trip(12, 5, 3, 0)}, 2, {}, no_extra);
            check(!same.threw && !same.r.feasible && any_reason_contains(same.r.violation_reasons, "constraint (28)"),
                  "S1-k (negative): the same two simultaneous trips on ONE drone id violate (28)");
            S1Out fleet = s1_eval({1, 3, 4, 5}, {0, 1, 12, 3, 0}, {trip(12, 4, 3, 0), trip(12, 5, 3, 1)}, 1, {}, no_extra);
            check(!fleet.threw && !fleet.r.feasible &&
                      any_reason_contains(fleet.r.violation_reasons, "2 drones are simultaneously in flight but |U| = 1"),
                  "S1-k (negative): the same trips with |U| = 1 are rejected (fleet capacity)");
        }

        // S1-l. Several violations are all reported (no early masking), and the result is deterministic.
        {
            S1Out a = s1_eval({1, 3}, {0, 1, 1, 0, 3, 9}, {}, 2, {3}, no_extra);
            S1Out b = s1_eval({1, 3}, {0, 1, 1, 0, 3, 9}, {}, 2, {3}, no_extra);
            check(!a.threw && !a.r.feasible, "S1-l: a route with several defects is infeasible and does not throw");
            check(any_reason_contains(a.r.violation_reasons, "appears more than once") &&
                      any_reason_contains(a.r.violation_reasons, "interior route position") &&
                      any_reason_contains(a.r.violation_reasons, "different depot") &&
                      any_reason_contains(a.r.violation_reasons, "has no CustomerData"),
                  "S1-l: repeated customer, interior depot, wrong end depot and missing data are all reported");
            check(a.r.violation_reasons == b.r.violation_reasons, "S1-l: the violation list is identical across repeated calls");
        }
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
