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
#include <iostream>
#include <string>
#include <unordered_map>

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

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

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
    //   generalized_flight_time (theta=0) = 4*(1+0) + 5 = 9
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
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;
        trip.service_customer = 2;
        trip.recovery_node = 3;
        solution.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 3.0, 0.0}; // drone-served: Se2=3, theta=0
        customers[3] = CustomerData{3, 2.0, 0.0, 0.0}; // truck-served: Se1=2

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

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
        check_near(t.generalized_flight_time, 9.0, 1e-9, "Generalized flight time with theta=0 equals the raw flight-time sum");
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
    //   objective = customer_arrival(2)=10 [drone] + arrival(3)=17 [truck] = 27
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::FlexibleLaunchPoint, Point2D{5.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{15.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{17.0, 0.0}});

        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 2.0; drone.endurance_seconds = 100.0;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;
        trip.service_customer = 2;
        trip.recovery_node = 3;
        solution.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

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
    //   generalized_flight_time (theta=0) = 3 + 5 = 8
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
        solution.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};

        {
            DroneParameters tight_drone;
            tight_drone.speed = 1.0;
            tight_drone.endurance_seconds = 5.0; // < required 8
            SpatialNetwork network(nodes, truck, tight_drone);
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

            check_near(result.drone_trip_timings[0].generalized_flight_time, 8.0, 1e-9,
                       "Generalized flight time for the endurance test trip is 3+5=8");
            check(!result.drone_trip_timings[0].endurance_feasible, "Trip exceeds a 5-second endurance budget (8 > 5)");
            check(!result.drone_trip_timings[0].feasible, "Trip is marked infeasible due to endurance");
            check(!result.feasible, "Overall solution is infeasible because its only drone trip violates endurance");
            check(any_reason_contains(result.violation_reasons, "exceeds battery endurance"),
                  "Violation reasons mention exceeding battery endurance");
        }
        {
            DroneParameters generous_drone;
            generous_drone.speed = 1.0;
            generous_drone.endurance_seconds = 10.0; // >= required 8
            SpatialNetwork network(nodes, truck, generous_drone);
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

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

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

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
    // return raw = 5. Endurance fixed at 10.
    //   theta=0:   generalized = 4*(1+0)   + 5 = 9  <= 10 -> feasible
    //   theta=0.5: generalized = 4*(1+0.5) + 5 = 11 >  10 -> infeasible
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
        drone.endurance_seconds = 10.0;
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 3, 0};
        DroneTrip trip;
        trip.launch_node = 1;
        trip.service_customer = 2;
        trip.recovery_node = 3;
        solution.drone_trips = {trip};

        {
            std::unordered_map<NodeId, CustomerData> customers;
            customers[2] = CustomerData{2, 0.0, 3.0, 0.0}; // theta=0
            customers[3] = CustomerData{3, 2.0, 0.0, 0.0};
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers);
            check_near(result.drone_trip_timings[0].generalized_flight_time, 9.0, 1e-9,
                       "theta=0: generalized flight time equals the raw sum (9)");
            check(result.drone_trip_timings[0].feasible, "theta=0: trip is feasible under endurance=10");
        }
        {
            std::unordered_map<NodeId, CustomerData> customers;
            customers[2] = CustomerData{2, 0.0, 3.0, 0.5}; // theta=0.5
            customers[3] = CustomerData{3, 2.0, 0.0, 0.0};
            SolutionEvaluationResult result = evaluate_solution(solution, network, customers);
            check_near(result.drone_trip_timings[0].generalized_flight_time, 11.0, 1e-9,
                       "theta=0.5: outbound leg is inflated to 4*1.5=6, generalized total = 6+5=11");
            check(!result.drone_trip_timings[0].feasible, "theta=0.5: the SAME geometry is now infeasible under endurance=10");
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
        SpatialNetwork network(nodes, truck, drone);

        Solution solution;
        solution.truck_route = {0, 1, 2, 4, 5, 0};

        DroneTrip trip1;
        trip1.launch_node = 2;
        trip1.service_customer = 3;
        trip1.recovery_node = 4;

        DroneTrip trip2;
        trip2.launch_node = 5;
        trip2.service_customer = 6;
        trip2.recovery_node = 0;

        solution.drone_trips = {trip1, trip2};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 2.0, 0.0, 0.0}; // A, truck-served
        customers[3] = CustomerData{3, 0.0, 1.0, 0.0}; // D1, drone-served
        customers[4] = CustomerData{4, 2.0, 0.0, 0.0}; // B, truck-served + recovery
        customers[6] = CustomerData{6, 0.0, 1.0, 0.0}; // D2, drone-served

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

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

        SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

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
    // Block 9 (Phase E audit regression). A drone recovers at a
    // TRUCK-SERVED customer with nonzero Se1, so the recovery (SR) and
    // the truck's own service (Se1) at that same node can overlap in
    // time. This distinguishes two readings of constraint 18
    // ("truck can't depart a node until service + drone launch/recovery
    // ops there finish"):
    //   (a) CONCURRENT (implemented): departure = max(arrival+Se1,
    //       recovery_complete) -- service and recovery are independent
    //       prerequisites that can proceed in parallel.
    //   (b) SEQUENTIAL (not implemented, shown here only for contrast):
    //       recovery cannot even START until service has fully finished,
    //       i.e. recovery_start = max(arrival+Se1, drone_arrival),
    //       departure = recovery_start + SR.
    // Three sub-cases place the drone's own (independent) arrival at the
    // recovery node BEFORE, DURING, and AFTER the truck's service
    // window, using one fixed geometry and varying only drone_speed.
    //
    // Shared geometry: Depot 0=(0,0), FLP 1=(2,0),
    // Customer(drone-served) 2=(6,3), Customer(recovery,truck-served)
    // 3=(10,0). truck_speed=1, SL=0, Se2(2)=0, theta=0, Se1(3)=6, SR=5.
    // Route: 0 -> FLP(1) -> C(3) -> 0.
    //
    // Shared truck-side hand computation (independent of drone_speed):
    //   0->1: dist=2, time=2/1=2                -> arrival[FLP]=2
    //   FLP launch: departure[FLP]=2+SL(0)=2    -> launch_time=2
    //   1->3: dist(FLP,C)=8, time=8/1=8         -> arrival[C]=2+8=10
    //   service_complete(C) = arrival[C](10) + Se1(6) = 16
    //
    // Drone-side, general form (dist(FLP,D)=5, dist(D,C)=5, both 3-4-5/
    // 4-3-5-style right triangles: D=(6,3) vs FLP=(2,0) -> (4,3) -> 5;
    // D=(6,3) vs C=(10,0) -> (4,-3) -> 5):
    //   customer_arrival(D)   = launch_time(2) + 5/drone_speed
    //   customer_departure(D) = customer_arrival + Se2(0) = same
    //   recovery_arrival(raw) = customer_departure + 5/drone_speed
    //                         = 2 + 10/drone_speed
    //
    // Sub-case A ("before" -- drone lands well before the truck even
    // arrives): drone_speed=2.
    //   customer_arrival(D) = 2 + 2.5 = 4.5
    //   recovery_arrival    = 2 + 5   = 7
    //   recovery_complete (concurrent) = max(arrival[C]=10, 7) + SR(5) = 15
    //   departure[C] (concurrent, IMPLEMENTED) = max(16, 15) = 16
    //   [contrast only, not implemented: sequential recovery_start =
    //    max(16, 7) = 16, departure = 16+5 = 21 -- differs from 16]
    //
    // Sub-case B ("during" -- drone lands while the truck is still
    // mid-service; the flagship case for this audit): drone_speed=1.
    //   customer_arrival(D) = 2 + 5  = 7
    //   recovery_arrival    = 2 + 10 = 12   (strictly inside [10, 16])
    //   recovery_complete (concurrent) = max(10, 12) + SR(5) = 17
    //   departure[C] (concurrent, IMPLEMENTED) = max(16, 17) = 17
    //   [contrast only: sequential recovery_start = max(16, 12) = 16,
    //    departure = 16+5 = 21 -- differs from 17]
    //
    // Sub-case C ("after" -- drone lands well after the truck's service
    // has already finished): drone_speed=0.5.
    //   customer_arrival(D) = 2 + 10 = 12
    //   recovery_arrival    = 2 + 20 = 22
    //   recovery_complete (concurrent) = max(10, 22) + SR(5) = 27
    //   departure[C] (concurrent, IMPLEMENTED) = max(16, 27) = 27
    //   [contrast only: sequential recovery_start = max(16, 22) = 22,
    //    departure = 22+5 = 27 -- SAME as concurrent here, because the
    //    drone itself arrives so late that service is moot either way.
    //    This is expected: the two readings can only disagree while the
    //    drone's own arrival falls inside/before the service window.]
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
            SpatialNetwork network(nodes, truck, drone);

            Solution solution;
            solution.truck_route = {0, 1, 3, 0};
            DroneTrip trip;
            trip.launch_node = 1;
            trip.service_customer = 2;
            trip.recovery_node = 3;
            solution.drone_trips = {trip};

            std::unordered_map<NodeId, CustomerData> customers;
            customers[2] = CustomerData{2, 0.0, 0.0, 0.0}; // drone-served: Se2=0, theta=0
            customers[3] = CustomerData{3, 6.0, 0.0, 0.0}; // truck-served: Se1=6

            SolutionEvaluationResult result = evaluate_solution(solution, network, customers);

            check(result.feasible, label + ": solution is feasible");
            check_near(result.truck_timing.arrival_time[2], 10.0, 1e-9,
                       label + ": truck arrival at the recovery customer is unaffected by drone speed");
            check_near(result.drone_trip_timings[0].customer_arrival_time, expected_drone_customer_arrival, 1e-9,
                       label + ": drone's own customer arrival time");
            check_near(result.truck_timing.departure_time[2], expected_departure_at_c, 1e-9,
                       label + ": departure at the recovery customer follows the CONCURRENT Se1/SR reading");

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
        run_case(/*drone_speed=*/2.0, /*expected_departure_at_c=*/16.0,
                  /*expected_drone_customer_arrival=*/4.5, "Recovery before truck service");

        // Sub-case B: drone recovers DURING the truck's service window --
        // the flagship case distinguishing concurrent vs. sequential Se1/SR.
        run_case(/*drone_speed=*/1.0, /*expected_departure_at_c=*/17.0,
                  /*expected_drone_customer_arrival=*/7.0, "Recovery during truck service");

        // Sub-case C: drone recovers well AFTER the truck's service window
        // (both readings happen to agree here -- included to show the
        // ambiguity only bites while the drone's arrival falls inside or
        // before the service window).
        run_case(/*drone_speed=*/0.5, /*expected_departure_at_c=*/27.0,
                  /*expected_drone_customer_arrival=*/12.0, "Recovery after truck service");
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
