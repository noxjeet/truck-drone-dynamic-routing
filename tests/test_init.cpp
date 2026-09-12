// Phase F tests for initial-solution construction (truck-only TSP,
// K-means clustering, FLP insertion, greedy drone-trip construction).
//
// Every expected value is computed BY HAND in comments before being
// checked. Where the truck-only TSP route itself matters, the test
// documents the ACTUAL route this codebase's deterministic
// HeldKarpSolver returns for that instance (discovered once by running
// it, then treated as a fixed, reproducible fact -- HeldKarpSolver uses
// no randomness, so this is not a flaky assumption) rather than assuming
// a specific tie-broken direction; a symmetric TSP instance always has
// at least two equal-cost optimal tours (a route and its reverse), so
// asserting "the route is one of the two costing 80" would be weaker
// than pinning the actual deterministic output and computing everything
// downstream of it by hand.

#include "graph/spatial_network.hpp"
#include "init/customer_clustering.hpp"
#include "init/drone_fleet_capacity.hpp"
#include "init/initial_solution_builder.hpp"
#include "init/truck_only_route.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/node.hpp"
#include "model/solution.hpp"
#include "timing/timing_engine.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>

using namespace tdr::model;
using namespace tdr::graph;
using namespace tdr::init;

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
    // =====================================================================
    // Block 1: k = ceil(|C| / (|U| + 2)) [PAPER, explicit formula].
    // =====================================================================
    {
        check(compute_cluster_count(10, 2) == 3, "k(10 customers, 2 drones) = ceil(10/4) = 3");
        check(compute_cluster_count(20, 2) == 5, "k(20 customers, 2 drones) = ceil(20/4) = 5");
        check(compute_cluster_count(50, 2) == 13, "k(50 customers, 2 drones) = ceil(50/4) = 13");
        check(compute_cluster_count(4, 1) == 2, "k(4 customers, 1 drone) = ceil(4/3) = 2");
        check(compute_cluster_count(0, 2) == 0, "k(0 customers) = 0 regardless of drone count");
        check(compute_cluster_count(1, 0) == 1, "k(1 customer, 0 drones) = ceil(1/2) = 1");
        check(compute_cluster_count(7, 3) == 2, "k(7 customers, 3 drones) = ceil(7/5) = 2");
    }

    // =====================================================================
    // Block 2: deterministic K-means on a hand-worked 4-point line, plus
    // an "obvious clusters" well-separated case and edge cases.
    //
    // Customers (id -> coords): 1=(10,0), 2=(20,0), 3=(30,0), 4=(40,0).
    // k=2. Seeding: sorted ids=[1,2,3,4], n=4, effective_k=2.
    //   centroid0 = ids[floor(0*4/2)] = ids[0] = 1 -> (10,0)
    //   centroid1 = ids[floor(1*4/2)] = ids[2] = 3 -> (30,0)
    // Iteration 1: assign by nearest centroid (10,0)/(30,0):
    //   1(10,0): d0=0, d1=20 -> cluster0
    //   2(20,0): d0=10, d1=10 -> TIE -> lowest index -> cluster0
    //   3(30,0): d0=20, d1=0 -> cluster1
    //   4(40,0): d0=30, d1=10 -> cluster1
    //   any_change=true (all were -1) -> recompute centroids:
    //   cluster0 mean(10,20)=(15,0); cluster1 mean(30,40)=(35,0)
    // Iteration 2: assign by (15,0)/(35,0): same assignment as iteration 1
    //   (1,2->0; 3,4->1) -> any_change=false -> recompute centroids: same
    //   (15,0)/(35,0), max_shift=0 -> converged after iterations_run=2.
    // =====================================================================
    {
        std::unordered_map<NodeId, Point2D> coords;
        coords[1] = Point2D{10.0, 0.0};
        coords[2] = Point2D{20.0, 0.0};
        coords[3] = Point2D{30.0, 0.0};
        coords[4] = Point2D{40.0, 0.0};

        KMeansResult r1 = run_kmeans(coords, 2);
        check(r1.converged, "K-means converges on the 4-point line");
        check(r1.iterations_run == 2, "K-means converges in exactly 2 iterations (hand-traced)");
        check(r1.cluster_count == 2, "K-means reports cluster_count == 2");
        check(r1.assignment.at(1) == r1.assignment.at(2), "Customers 1 and 2 land in the same cluster");
        check(r1.assignment.at(3) == r1.assignment.at(4), "Customers 3 and 4 land in the same cluster");
        check(r1.assignment.at(1) != r1.assignment.at(3), "The two pairs land in different clusters");
        const int c01 = r1.assignment.at(1);
        const int c34 = r1.assignment.at(3);
        check_near(r1.centroids[c01].x, 15.0, 1e-9, "Cluster {1,2}'s centroid x = mean(10,20) = 15");
        check_near(r1.centroids[c34].x, 35.0, 1e-9, "Cluster {3,4}'s centroid x = mean(30,40) = 35");

        KMeansResult r2 = run_kmeans(coords, 2);
        check(r1.assignment == r2.assignment, "K-means is deterministic: identical assignment across repeated calls");
        check(r1.iterations_run == r2.iterations_run, "K-means is deterministic: identical iteration count across repeated calls");
    }

    // Obvious well-separated clusters: two tight groups far apart.
    {
        std::unordered_map<NodeId, Point2D> coords;
        coords[1] = Point2D{0.0, 0.0};
        coords[2] = Point2D{1.0, 0.0};
        coords[3] = Point2D{0.0, 1.0};
        coords[10] = Point2D{1000.0, 1000.0};
        coords[11] = Point2D{1001.0, 1000.0};
        coords[12] = Point2D{1000.0, 1001.0};

        KMeansResult r = run_kmeans(coords, 2);
        check(r.assignment.at(1) == r.assignment.at(2) && r.assignment.at(2) == r.assignment.at(3),
              "The near-origin trio all land in the same cluster");
        check(r.assignment.at(10) == r.assignment.at(11) && r.assignment.at(11) == r.assignment.at(12),
              "The far-away trio all land in the same cluster");
        check(r.assignment.at(1) != r.assignment.at(10), "The two obviously-separated groups land in different clusters");
    }

    // Edge cases.
    {
        std::unordered_map<NodeId, Point2D> empty_coords;
        KMeansResult r_empty = run_kmeans(empty_coords, 3);
        check(r_empty.cluster_count == 0, "K-means on an empty customer set reports cluster_count == 0");

        std::unordered_map<NodeId, Point2D> two_points;
        two_points[1] = Point2D{0.0, 0.0};
        two_points[2] = Point2D{5.0, 5.0};
        KMeansResult r_over = run_kmeans(two_points, 10);
        check(r_over.cluster_count == 2, "K-means clamps k > n down to n (2 points, k=10 requested)");

        KMeansResult r_zero = run_kmeans(two_points, 0);
        check(r_zero.cluster_count == 0, "K-means with k=0 reports cluster_count == 0, no crash");
    }

    // =====================================================================
    // Block 3 (F1): truck-only route on the same 4-point line. For points
    // on a line, the optimal round-trip tour visits them in order out to
    // the farthest one and returns directly -- cost = 2 * 40 = 80. This
    // is a robust, direction-independent invariant (unlike a specific
    // node order, which ties with its own reverse under any symmetric
    // TSP instance) so it is asserted directly, along with route
    // structural properties that also do not depend on direction.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{20.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{30.0, 0.0}});
        nodes.add(Node{4, NodeType::Customer, Point2D{40.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 1.0;
        SpatialNetwork network(nodes, truck, drone);

        TruckOnlyRouteResult r = build_truck_only_route(0, {1, 2, 3, 4}, network);
        check(r.tsp_result.succeeded, "F1: truck-only TSP succeeds on a 5-node instance");
        check(r.tsp_result.exact, "F1: n=5 is well within Held-Karp's exact range -- result is a certified optimum, not heuristic");
        check_near(r.tsp_result.cost, 80.0, 1e-9, "F1: optimal truck-only tour cost on the 4-point line is 2*40=80");
        check(r.route.front() == 0 && r.route.back() == 0, "F1: route starts and ends at the depot");
        check(r.route.size() == 6, "F1: route has depot + 4 customers + depot = 6 entries");
        for (NodeId id : {1, 2, 3, 4}) {
            check(std::find(r.route.begin(), r.route.end(), id) != r.route.end(),
                  "F1: customer " + std::to_string(id) + " appears somewhere in the truck-only route");
        }
    }

    // =====================================================================
    // Block 4 (F4 endurance rejection): a single customer far enough from
    // its only candidate launch point, under a deliberately tight
    // endurance budget, that no feasible drone trip can be built for it.
    // Depot=(0,0), Customer=1=(100,0). One arc (depot,1), FLP at its
    // midpoint (50,0). truck_speed=1, drone_speed=1, endurance=10.
    //   outbound: dist(50,0)-(100,0)=50 -> 50s
    //   return:   dist(100,0)-(0,0) [recovery=depot] = 100 -> 100s
    //   generalized_flight_time (theta=0) = 150 >> 10 -> infeasible
    // Expected: the builder leaves the customer TRUCK-SERVED (no drone
    // trip at all), and the resulting trivial solution is fully feasible
    // with objective = truck arrival at the customer = 100.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{100.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 1.0;
        drone.endurance_seconds = 10.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};

        InitialSolutionResult result = build_initial_solution(0, {1}, customers, network);

        check(result.solution.drone_trips.empty(),
              "F4 endurance rejection: no drone trip is constructed for the endurance-infeasible customer");
        check(result.solution.truck_route.size() == 3 && result.solution.truck_route[1] == 1,
              "F4 endurance rejection: the customer remains on the truck route (truck-served)");
        check(result.feasible, "F4 endurance rejection: the resulting truck-only solution is fully feasible");
        check_near(result.objective_value, 100.0, 1e-9,
                   "F4 endurance rejection: objective is the truck's arrival time at the customer (100)");
    }

    // =====================================================================
    // Block 5 (recovery-before-launch rejection): Phase F's own candidate
    // generation can never propose a trip whose recovery position
    // precedes its launch position (recovery is always found by scanning
    // FORWARD from the launch position -- see initial_solution_builder.hpp
    // F4). What this test verifies instead is the feasibility gate F4
    // depends on: that tdr::timing::evaluate_solution (unchanged from
    // Phase E, reused here rather than reimplemented) correctly rejects
    // such a trip if one were ever proposed, so F4's reliance on it is
    // sound. Route: depot(0) -> custA(1, recovery target) -> FLP(2) ->
    // custB(3, drone-served, NOT on the truck route) -> depot(0). Trip:
    // launch=2 (position 2), service=3, recovery=1 (position 1, BEFORE
    // the launch position).
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{5.0, 0.0}});
        nodes.add(Node{2, NodeType::FlexibleLaunchPoint, Point2D{10.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{15.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 1000.0;
        SpatialNetwork network(nodes, truck, drone);

        Solution sol;
        sol.truck_route = {0, 1, 2, 0}; // custB (3) is drone-served, not in the truck route
        DroneTrip trip;
        trip.launch_node = 2;
        trip.service_customer = 3;
        trip.recovery_node = 1; // position 1, before launch's position 2
        sol.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};

        tdr::timing::SolutionEvaluationResult eval = tdr::timing::evaluate_solution(sol, network, customers);
        check(!eval.drone_trip_timings[0].on_route,
              "Recovery-before-launch: the feasibility gate marks the trip as not on-route");
        check(!eval.drone_trip_timings[0].feasible,
              "Recovery-before-launch: the feasibility gate marks the trip infeasible");
        check(!eval.feasible, "Recovery-before-launch: the overall trial solution is infeasible");
    }

    // =====================================================================
    // Block 6: full end-to-end initial-solution construction on 4 colinear
    // customers, 1 drone. Covers F1-F4 together, plus items 5 ("simple
    // feasible drone trip construction"), 8 ("duplicate customer-service
    // prevention"), 9 ("all customers served exactly once"), and 10
    // ("final initial solution validated through the Phase E engine").
    //
    // Depot=0=(0,0), customers 1=(10,0), 2=(20,0), 3=(30,0), 4=(40,0).
    // truck_speed=1, drone_speed=2, SL=SR=Se1=Se2=theta=0, endurance=100
    // (generous -- not the point of this test), num_drones=1.
    //
    // F1: HeldKarpSolver is exact and fully deterministic (no RNG). For
    // this instance it returns r0 = [0,4,3,2,1,0] (cost 80 = 2*40, the
    // known-optimal cost for colinear points -- this specific direction,
    // among the two equal-cost optimal tours, was observed by running
    // this exact frozen solver once and is reproduced by it every time).
    //
    // F2: k=ceil(4/(1+2))=2. K-means (Block 2's own logic, seeded from
    // the sorted customer ids [1,2,3,4], independent of r0's direction)
    // gives cluster0={1,2}, cluster1={3,4}.
    //
    // F3: one FLP per arc of r0: arc(0,4)->(20,0); arc(4,3)->(35,0);
    // arc(3,2)->(25,0); arc(2,1)->(15,0); arc(1,0)->(5,0) [unused, enters
    // the depot]. Launch candidates: customer4->(20,0); customer3->(35,0);
    // customer2->(25,0); customer1->(15,0).
    //
    // F4 cluster0 (customers 1,2), available_drones=1:
    //   candidate 1: launch(15,0)->cust1, recovery=depot (nothing truck-
    //     served follows customer1 in r0 before the closing depot).
    //     launch_time=65, customer_arrival=67.5 (score).
    //   candidate 2: launch(25,0)->cust2, recovery=customer1.
    //     launch_time=55, customer_arrival=57.5 (score).
    //   BestRoute (lowest score) picks customer 2: score 57.5 < 67.5.
    //   Committed. available_drones -> 0. Customer 1 remains truck-served
    //   (and becomes recovery-locked).
    //
    // F4 cluster1 (customers 3,4), available_drones resets to 1:
    //   candidate 4: launch(20,0)->cust4, recovery=customer3.
    //     launch_time=20, customer_arrival=30 (score).
    //   candidate 3: launch(35,0)->cust3, recovery=customer1 (the next
    //     later Customer/Depot after arc entering cust3 in the route
    //     WITHOUT customer4 removed -- customer1 is still on the route in
    //     THIS independent trial). launch_time=45, customer_arrival=47.5.
    //   BestRoute picks customer 4: score 30 < 47.5. Committed.
    //   Customer 3 remains truck-served (and becomes recovery-locked).
    //
    // Final route: [0, FLP(20,0), 3, FLP(25,0), 1, 0].
    // Final trips: (launch=FLP(25,0), customer=2, recovery=1),
    //              (launch=FLP(20,0), customer=4, recovery=3).
    // Final timing (forward pass, hand-computed):
    //   arrival[0]=0
    //   FLP(20,0): arrival=20, departure=20 (launch_time for trip(4)=20)
    //     drone(4): outbound |20-40|/2=10 -> arrival(4)=30; departure=30;
    //       return |40-30|/2=5 -> recovery_arrival=35
    //   node3: arrival=departure[FLP20](20)+|20-30|=30;
    //     recovery_complete=max(30,35)+0=35; service_complete=30+0=30;
    //     departure[3]=max(30,35)=35
    //   FLP(25,0): arrival=departure[3](35)+|30-25|=40; departure=40
    //     (launch_time for trip(2)=40)
    //     drone(2): outbound |25-20|/2=2.5 -> arrival(2)=42.5;
    //       departure=42.5; return |20-10|/2=5 -> recovery_arrival=47.5
    //   node1: arrival=departure[FLP25](40)+|25-10|=55;
    //     recovery_complete=max(55,47.5)+0=55; service_complete=55+0=55;
    //     departure[1]=max(55,55)=55
    //   depot: arrival=departure[1](55)+|10-0|=65
    //   Objective = 30 (cust4,drone) + 30 (cust3,truck) + 42.5 (cust2,drone)
    //             + 55 (cust1,truck) = 157.5
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{20.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{30.0, 0.0}});
        nodes.add(Node{4, NodeType::Customer, Point2D{40.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 2.0;
        drone.endurance_seconds = 100.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        std::unordered_map<NodeId, CustomerData> customers;
        for (NodeId id : {1, 2, 3, 4}) customers[id] = CustomerData{id, 0.0, 0.0, 0.0};

        InitialSolutionResult result = build_initial_solution(0, {1, 2, 3, 4}, customers, network);

        check(result.tsp_result.exact, "Block 6: F1 tour is exact (n=5, well within Held-Karp's range)");
        check_near(result.tsp_result.cost, 80.0, 1e-9, "Block 6: F1 optimal cost is 80");
        check(result.truck_only_route.size() == 6, "Block 6: F1 route has 6 entries (depot+4 customers+depot)");

        check(result.clustering.cluster_count == 2, "Block 6: F2 produces 2 clusters");
        check(result.clustering.assignment.at(1) == result.clustering.assignment.at(2),
              "Block 6: F2 groups customers 1 and 2 together");
        check(result.clustering.assignment.at(3) == result.clustering.assignment.at(4),
              "Block 6: F2 groups customers 3 and 4 together");

        check(result.candidate_flps.size() == 5, "Block 6: F3 generates one FLP per arc of the 5-node route (5 arcs)");
        bool flp_geometry_ok = true;
        for (std::size_t i = 0; i + 1 < result.truck_only_route.size(); ++i) {
            const Point2D& a = nodes.at(result.truck_only_route[i]).coordinates;
            const Point2D& b = nodes.at(result.truck_only_route[i + 1]).coordinates;
            const auto& gen = result.candidate_flps[i];
            if (gen.data.arc_from != result.truck_only_route[i] || gen.data.arc_to != result.truck_only_route[i + 1]) flp_geometry_ok = false;
            if (std::fabs(gen.node.coordinates.x - (a.x + b.x) / 2.0) > 1e-9) flp_geometry_ok = false;
            if (std::fabs(gen.node.coordinates.y - (a.y + b.y) / 2.0) > 1e-9) flp_geometry_ok = false;
        }
        check(flp_geometry_ok, "Block 6: F3 every candidate FLP sits at the midpoint of its recorded arc, in route order");

        check(result.solution.drone_trips.size() == 2, "Block 6: F4 constructs exactly 2 drone trips");
        bool trip_for_2 = false, trip_for_4 = false;
        for (const auto& t : result.solution.drone_trips) {
            if (t.service_customer == 2) { trip_for_2 = (t.recovery_node == 1); }
            if (t.service_customer == 4) { trip_for_4 = (t.recovery_node == 3); }
        }
        check(trip_for_2, "Block 6: customer 2 is drone-served, recovering at customer 1");
        check(trip_for_4, "Block 6: customer 4 is drone-served, recovering at customer 3");

        check(result.feasible, "Block 6: the final initial solution is feasible");
        check_near(result.objective_value, 157.5, 1e-9, "Block 6: hand-computed objective is 157.5");

        int truck_served = 0, drone_served = 0;
        std::unordered_map<NodeId, int> seen_count;
        for (const auto& rec : result.evaluation.customer_records) {
            seen_count[rec.customer_id]++;
            if (rec.served_by_truck) ++truck_served;
            if (rec.served_by_drone) ++drone_served;
            check(!(rec.served_by_truck && rec.served_by_drone),
                  "Block 6: customer " + std::to_string(rec.customer_id) + " is not marked served by both vehicles");
        }
        check(result.evaluation.customer_records.size() == 4, "Block 6: exactly 4 customer records are produced (no duplicates, no omissions)");
        for (NodeId id : {1, 2, 3, 4}) {
            check(seen_count[id] == 1, "Block 6: customer " + std::to_string(id) + " has exactly one service record");
        }
        check(truck_served == 2 && drone_served == 2, "Block 6: 2 customers truck-served and 2 drone-served, every customer served exactly once");

        // Item 10: re-evaluate the FINAL solution independently (a fresh
        // network built from the result's own node_registry) and confirm
        // it agrees with the builder's own final evaluation -- the
        // builder must not report anything Phase E itself would not.
        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers);
        check(independent_eval.feasible == result.evaluation.feasible,
              "Block 6: independently re-running the Phase E engine agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-9,
                   "Block 6: independently re-running the Phase E engine agrees on the objective value");
    }

    // =====================================================================
    // Block 7 (post-approval correctness patch, Issue 1): direct unit
    // tests of check_drone_fleet_capacity() -- the pure sweep-line
    // arithmetic F4 now uses to decide whether accepting a candidate
    // trip would ever require more drones "in the air" at once than the
    // fleet has. These use hand-built DroneTripTiming values (only
    // launch_time/recovery_complete_time matter to this function) rather
    // than a full solution, so the arithmetic itself is pinned
    // independently of any clustering/TSP behavior.
    // =====================================================================
    {
        using tdr::init::check_drone_fleet_capacity;
        using tdr::timing::DroneTripTiming;

        auto make = [](double launch, double recover) {
            DroneTripTiming t;
            t.launch_time = launch;
            t.recovery_complete_time = recover;
            return t;
        };

        // No trips at all: vacuously feasible regardless of fleet size.
        {
            auto r = check_drone_fleet_capacity({}, 0);
            check(r.feasible && r.max_simultaneous_drones == 0, "Block 7: zero trips is vacuously feasible");
        }
        // Two intervals that only TOUCH at the boundary (one ends exactly
        // when the other starts) do not count as overlapping -- a drone
        // freed at time T is available for a new launch at time T.
        {
            std::vector<DroneTripTiming> t{make(0.0, 10.0), make(10.0, 20.0)};
            auto r = check_drone_fleet_capacity(t, 1);
            check(r.max_simultaneous_drones == 1, "Block 7: touching-boundary intervals never overlap (max=1)");
            check(r.feasible, "Block 7: touching-boundary intervals are feasible with 1 drone");
        }
        // Two genuinely overlapping intervals ([0,10] and [5,15]) need 2
        // drones: infeasible with 1, feasible with 2.
        {
            std::vector<DroneTripTiming> t{make(0.0, 10.0), make(5.0, 15.0)};
            auto r1 = check_drone_fleet_capacity(t, 1);
            check(r1.max_simultaneous_drones == 2, "Block 7: overlapping pair needs 2 simultaneous drones");
            check(!r1.feasible, "Block 7: overlapping pair is infeasible with only 1 drone");
            auto r2 = check_drone_fleet_capacity(t, 2);
            check(r2.feasible, "Block 7: overlapping pair is feasible with 2 drones");
        }
        // Three intervals staggered so consecutive pairs overlap but all
        // three are never simultaneously active: [0,10], [5,15], [12,20].
        // A&B overlap on [5,10]; B&C overlap on [12,15]; A and C never
        // overlap (10 <= 12). Max simultaneous = 2, not 3.
        {
            std::vector<DroneTripTiming> t{make(0.0, 10.0), make(5.0, 15.0), make(12.0, 20.0)};
            auto r = check_drone_fleet_capacity(t, 2);
            check(r.max_simultaneous_drones == 2, "Block 7: staggered triple never needs more than 2 simultaneously");
            check(r.feasible, "Block 7: staggered triple is feasible with 2 drones");
            auto r1 = check_drone_fleet_capacity(t, 1);
            check(!r1.feasible, "Block 7: staggered triple is infeasible with only 1 drone");
        }
    }

    // =====================================================================
    // Block 8 (post-approval correctness patch, Issue 1): a genuine
    // end-to-end instance, run through the ACTUAL build_initial_solution
    // pipeline (real K-means + real TSP, no hand-picked cluster labels),
    // where the drone-fleet cap correctly rejects a DIFFERENT cluster's
    // candidate because accepting it would require 2 drones
    // simultaneously in the air, with only num_drones=1 available.
    //
    // Depot=(0,0); customers 1=(10,5), 2=(20,0), 3=(30,5), 4=(40,0).
    // truck_speed=1, drone_speed=2, SL=SR=Se1=Se2=theta=0,
    // endurance=1000 (generous -- not the point here), num_drones=1.
    //
    // F1: HeldKarpSolver (exact, deterministic, no RNG) returns
    // r0 = [0,2,4,3,1,0], cost = 20+20+sqrt(125)+20+sqrt(125)
    //    = 60 + 2*sqrt(125) = 82.360679774998 (observed once by running
    // this exact frozen solver; reproduced identically every time).
    //
    // F2: k=ceil(4/(1+2))=2. K-means groups cluster0={1,2} (near x=10-20)
    // and cluster1={3,4} (near x=30-40) -- the small y perturbation on
    // customers 1,3 does not change which points are mutually closest.
    //
    // F3: one FLP per arc of r0 (points_per_arc=1): arc(0,2)->(10,0)
    // [customer2]; arc(2,4)->(30,0) [customer4]; arc(4,3)->(35,2.5)
    // [customer3]; arc(3,1)->(20,5) [customer1]; arc(1,0)->(5,2.5)
    // [depot, unused].
    //
    // F4 cluster0={1,2} is processed FIRST (ascending cluster index).
    // Round 1, evaluated against the empty accepted-trip set:
    //   candidate for customer2 (launch (10,0), recovery=customer4 --
    //     the next Customer/Depot node after customer2's position in r0):
    //     launch_time=10, customer_arrival=10+|10-20|/2=15.
    //   candidate for customer1 (launch (20,5), recovery=depot):
    //     launch_time = 20(0->2)+20(2->4)+sqrt(125)(4->3)+10(3->launch)
    //       = 61.180339887499; customer_arrival = +5 = 66.180339887499.
    //   BestRoute picks the lower score: customer2 (15 < 66.18).
    //   Committed (fleet check on a single trip trivially passes).
    //   Customer4 becomes recovery-locked.
    // Round 2: only customer1 remains (customer2 converted). Its trial
    //   (accepted=[trip2]) has interval [61.180339887499, 81.795868...]
    //   vs trip2's [10,40] -- disjoint, fleet check passes. Committed.
    //   Cluster0 exhausted (both customers converted).
    //
    // F4 cluster1={3,4}: customer4 is recovery-locked (skipped).
    //   candidate for customer3 (launch (35,2.5), recovery=depot -- the
    //     node that WAS customer1's own next stop is now customer1's own
    //     launch FLP, an FLP type, so the search skips it and reaches the
    //     depot): building this trial (accepted=[trip2,trip1] +
    //     candidate3) and re-evaluating the WHOLE trial from scratch
    //     (Phase E always does) gives, because trip1's own launch point
    //     now sits downstream of a SHORTER path through customer3's own
    //     launch FLP instead of through customer3 directly:
    //       trip1 (recomputed within this trial): launch_time =
    //         40 (trip2's own recovery_complete, unchanged)
    //         + sqrt(31.25) (node4->launch3, 5.590169943749)
    //         + sqrt(231.25) (launch3->launch1, 15.206906325746)
    //         = 60.797076269495; recovery_complete (at the shared
    //         ending depot) = 81.412604397583.
    //       trip3 (candidate): launch_time = 45.590169943749;
    //         recovery_complete (ALSO the ending depot, since customer1's
    //         slot is now an FLP too) = 81.412604397583.
    //     trip3's interval [45.590169943749, 81.412604397583] OVERLAPS
    //     trip1's [60.797076269495, 81.412604397583] (60.797... falls
    //     inside trip3's window) -- max_simultaneous_drones = 2 > 1 =
    //     fleet_size. check_drone_fleet_capacity rejects this candidate.
    //     [Before this patch, F4 would have reset "available_drones" to
    //     1 fresh at the start of cluster1's processing and accepted this
    //     candidate on structural/endurance grounds alone, silently
    //     producing a solution needing 2 simultaneous drones from a
    //     1-drone fleet -- exactly the bug this patch fixes.]
    //   No other candidate exists this round (customer4 still locked) ->
    //   feasible_this_round is empty -> cluster1 is exhausted with ZERO
    //   trips. Customers 3 and 4 remain truck-served.
    //
    // Final route: [0, launch(customer2), customer4, customer3,
    // launch(customer1), 0] -- the two ACCEPTED trips are customer2's
    // and customer1's; customer3 and customer4 stay plain truck-served
    // nodes. Final (fully re-evaluated) timings:
    //   trip(cust=2): launch_time=10, customer_arrival=15,
    //     recovery_complete=40 (recovery=customer4, still a plain
    //     truck-served node in the final solution): truck arrival at
    //     (10,0)=10; outbound |10-20|/2=5 -> arrival=15; return
    //     |20-40|/2=10 -> drone_recovery_arrival=25; truck arrival at
    //     customer4=10+30=40; recovery_complete=max(40,25)+0=40.
    //   customer4 (truck-served): arrival = 40 (departure[customer4] is
    //     then also 40, gated by this same recovery_complete).
    //   customer3 (truck-served): arrival = 40 + sqrt(125)
    //     = 51.180339887499 (the direct customer4->customer3 leg, since
    //     customer3 itself was never converted).
    //   trip(cust=1): launch_time = 51.180339887499 (departure[customer3])
    //     + 10 (customer3->launch1 leg, (30,5)->(20,5)) = 61.180339887499;
    //     customer_arrival=66.180339887499; recovery_complete
    //     =81.795868015587 -- identical to the standalone per-candidate
    //     derivation above, since nothing upstream of customer1's launch
    //     changed between the trial and the final solution (customer3 was
    //     never converted either way).
    //   Objective = 15 (cust2,drone) + 51.180339887499 (cust3,truck)
    //             + 66.180339887499 (cust1,drone) + 40 (cust4,truck)
    //             = 172.360679774998.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 5.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{20.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{30.0, 5.0}});
        nodes.add(Node{4, NodeType::Customer, Point2D{40.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 2.0;
        drone.endurance_seconds = 1000.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        std::unordered_map<NodeId, CustomerData> customers;
        for (NodeId id : {1, 2, 3, 4}) customers[id] = CustomerData{id, 0.0, 0.0, 0.0};

        InitialSolutionResult result = build_initial_solution(0, {1, 2, 3, 4}, customers, network);

        check(result.tsp_result.exact, "Block 8: F1 tour is exact");
        check_near(result.tsp_result.cost, 60.0 + 2.0 * std::sqrt(125.0), 1e-9, "Block 8: F1 optimal cost matches hand formula");

        check(result.clustering.cluster_count == 2, "Block 8: F2 produces 2 clusters");
        auto cluster_of = [&](NodeId id) { return result.clustering.assignment.at(id); };
        check(cluster_of(1) == cluster_of(2) && cluster_of(3) == cluster_of(4) && cluster_of(1) != cluster_of(3),
              "Block 8: F2 groups {1,2} and {3,4} into two DIFFERENT clusters");

        // Item 1 + item 2: multiple clusters competed for the single
        // drone, and the cross-cluster time conflict was detected --
        // cluster1's customers (3,4) end up with ZERO accepted drone
        // trips (customer4 recovery-locked, customer3's only candidate
        // rejected for exceeding the fleet's simultaneous capacity),
        // while cluster0's customers (1,2) both got their trips.
        check(result.solution.drone_trips.size() == 2, "Block 8: exactly 2 drone trips accepted overall (not 3 or 4)");
        bool customer3_is_drone = false, customer4_is_drone = false;
        bool customer1_is_drone = false, customer2_is_drone = false;
        for (const auto& t : result.solution.drone_trips) {
            if (t.service_customer == 1) customer1_is_drone = true;
            if (t.service_customer == 2) customer2_is_drone = true;
            if (t.service_customer == 3) customer3_is_drone = true;
            if (t.service_customer == 4) customer4_is_drone = true;
        }
        check(customer1_is_drone && customer2_is_drone, "Block 8: cluster0's customers 1 and 2 are both drone-served");
        check(!customer3_is_drone && !customer4_is_drone,
              "Block 8: cluster1's customers 3 and 4 remain truck-served (fleet-capacity conflict blocked customer3, "
              "recovery-locking blocked customer4)");

        // Item 2, directly: verify that the ACCEPTED solution never
        // requires more than 1 drone simultaneously (the fleet cap is
        // actually respected in the delivered result, not just "checked
        // and ignored").
        auto final_capacity = check_drone_fleet_capacity(result.evaluation.drone_trip_timings, 1);
        check(final_capacity.feasible && final_capacity.max_simultaneous_drones <= 1,
              "Block 8: the final accepted solution never needs more than 1 simultaneous drone");

        check(result.feasible, "Block 8: the final initial solution is feasible");
        check_near(result.objective_value, 172.360679774998, 1e-6,
                   "Block 8: hand-computed objective is 172.360679774998");

        // Item 4: independent Phase E re-evaluation from a rebuilt
        // network agrees with the builder's own stored evaluation.
        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers);
        check(independent_eval.feasible == result.feasible,
              "Block 8: independent Phase E re-evaluation agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-6,
                   "Block 8: independent Phase E re-evaluation agrees on the objective value");
    }

    // =====================================================================
    // Block 9 (post-approval correctness patch, Issue 1): two genuinely
    // DIFFERENT clusters, far enough apart that their two accepted drone
    // trips do NOT overlap in time -- proving the single drone in the
    // fleet is correctly REUSED (not blocked) once its first trip has
    // completed. This is the same instance shape used informally while
    // designing the fix; every number below is confirmed against the
    // implementation's own deterministic output.
    //
    // Depot=(0,0); customers 1=(10,0), 2=(20,0), 3=(500,0), 4=(510,0).
    // truck_speed=1, drone_speed=2, SL=SR=Se1=Se2=theta=0,
    // endurance=1000, num_drones=1.
    //
    // F1: r0 = [0,4,3,2,1,0], cost = 2*510 = 1020 (colinear, all
    // customers on one side of the depot -- the well-known "there and
    // back" optimum; this specific direction is HeldKarpSolver's own
    // deterministic tie-break, as in Block 6).
    // F2: k=ceil(4/3)=2; cluster0={1,2}, cluster1={3,4} (contiguous
    // proximity groups, as in Block 6/8).
    // F3: FLP candidates at arc midpoints: (255,0)->customer4;
    // (505,0)->customer3; (260,0)->customer2; (15,0)->customer1.
    // F4: cluster0 accepts customer2 (recovery=customer1); cluster1
    // accepts customer4 (recovery=customer3) -- full derivation and the
    // exact final numbers below match the implementation's own
    // deterministic, independently-recomputed output:
    //   trip(customer4): launch_time=255, recovery_complete=500.
    //   trip(customer2): launch_time=740, recovery_complete=990.
    // 500 <= 740: the two trips' busy intervals do NOT overlap, so a
    // single physical drone suffices for both -- exactly the "reuse
    // after the previous operation has completed" behavior the review
    // required, confirmed end-to-end through the real construction
    // pipeline (not just the isolated check_drone_fleet_capacity unit
    // tests in Block 7).
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{20.0, 0.0}});
        nodes.add(Node{3, NodeType::Customer, Point2D{500.0, 0.0}});
        nodes.add(Node{4, NodeType::Customer, Point2D{510.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 2.0;
        drone.endurance_seconds = 1000.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        std::unordered_map<NodeId, CustomerData> customers;
        for (NodeId id : {1, 2, 3, 4}) customers[id] = CustomerData{id, 0.0, 0.0, 0.0};

        InitialSolutionResult result = build_initial_solution(0, {1, 2, 3, 4}, customers, network);

        auto cluster_of = [&](NodeId id) { return result.clustering.assignment.at(id); };
        check(cluster_of(1) == cluster_of(2) && cluster_of(3) == cluster_of(4) && cluster_of(1) != cluster_of(3),
              "Block 9: customers split into two genuinely different clusters, {1,2} and {3,4}");

        check(result.solution.drone_trips.size() == 2, "Block 9: both clusters' best candidates are accepted (2 trips)");

        double launch_a = -1.0, recover_a = -1.0, launch_b = -1.0, recover_b = -1.0;
        for (std::size_t i = 0; i < result.solution.drone_trips.size(); ++i) {
            const auto& t = result.solution.drone_trips[i];
            const auto& tm = result.evaluation.drone_trip_timings[i];
            if (t.service_customer == 4) { launch_a = tm.launch_time; recover_a = tm.recovery_complete_time; }
            if (t.service_customer == 2) { launch_b = tm.launch_time; recover_b = tm.recovery_complete_time; }
        }
        check_near(launch_a, 255.0, 1e-9, "Block 9: customer4's trip launches at 255");
        check_near(recover_a, 500.0, 1e-9, "Block 9: customer4's trip completes recovery at 500");
        check_near(launch_b, 740.0, 1e-9, "Block 9: customer2's trip launches at 740");
        check_near(recover_b, 990.0, 1e-9, "Block 9: customer2's trip completes recovery at 990");
        check(recover_a <= launch_b, "Block 9: customer4's trip fully completes before customer2's trip launches (no overlap)");

        auto capacity = check_drone_fleet_capacity(result.evaluation.drone_trip_timings, 1);
        check(capacity.max_simultaneous_drones == 1,
              "Block 9: the two different-cluster trips never require more than 1 simultaneous drone (sequential reuse)");
        check(capacity.feasible, "Block 9: sequential reuse is feasible with exactly 1 drone");

        check(result.feasible, "Block 9: the final initial solution is feasible");
        check_near(result.objective_value, 2732.5, 1e-6, "Block 9: hand-computed objective is 2732.5");

        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers);
        check(independent_eval.feasible == result.feasible,
              "Block 9: independent Phase E re-evaluation agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-6,
                   "Block 9: independent Phase E re-evaluation agrees on the objective value");
    }

    // =====================================================================
    // Block 10 (post-approval correctness patch, Issue 2): FLP candidate
    // coverage. Proves the launch-candidate lookup considers EVERY FLP
    // generated on a customer's entering arc, not just the single
    // closest one -- i.e. that the fix in initial_solution_builder.hpp
    // (launch_candidates_for now maps to a vector, and F4 tries every
    // entry) actually changes behavior versus the pre-patch single-
    // candidate lookup, rather than being a no-op refactor.
    //
    // Depot=(0,0), single customer 1=(90,0). truck_speed=1,
    // drone_speed=2 (FASTER than the truck -- see below for why this
    // matters), SL=SR=Se1=Se2=theta=0, endurance=1000 (generous -- not
    // the point), num_drones=1, points_per_arc=2.
    //
    // F1: r0=[0,1,0] (trivial). F3 generates 2 FLPs per arc; on arc
    // (0->1) [the one entering the customer]: id=2 at ratio 1/3 = (30,0)
    // and id=3 at ratio 2/3 = (60,0).
    //
    // Because drone_speed(2) > truck_speed(1), launching FARTHER from
    // the customer (lower ratio) gives an EARLIER customer arrival, not
    // a later one: arrival(x) = x (truck time to reach the launch point)
    // + (90-x)/2 (drone flight from there) = 45 + x/2, which INCREASES
    // with x. So the theoretically-nearest-to-customer candidate (60,0)
    // is actually the WORSE choice here, and the farther one (30,0) is
    // BestRoute's correct pick:
    //   candidate @ (30,0): launch_time=30, arrival=30+(90-30)/2=60.
    //   candidate @ (60,0): launch_time=60, arrival=60+(90-60)/2=75.
    // If the lookup still kept only the single closest same-arc
    // candidate (the pre-patch behavior, which always kept the
    // highest-ratio point), the accepted launch node would necessarily
    // be id=3 (60,0) with objective 75 -- it would never even see id=2.
    // With the fix, BOTH are tried and BestRoute correctly picks the
    // lower-scoring one: id=2 (30,0), objective 60.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{90.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 2.0;
        drone.endurance_seconds = 1000.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};

        InitialSolutionPolicy policy;
        policy.flp_policy.points_per_arc = 2;
        InitialSolutionResult result = build_initial_solution(0, {1}, customers, network, policy);

        // 2 arcs in r0 (0->1, 1->0) x 2 points per arc = 4 candidate FLPs
        // total, but only the 2 on arc (0->1) are candidates for
        // customer 1 -- proving neither is silently dropped.
        check(result.candidate_flps.size() == 4, "Block 10: F3 generates 4 candidate FLPs total (2 arcs x 2 per arc)");
        int same_arc_candidates = 0;
        for (const auto& g : result.candidate_flps) {
            if (g.data.arc_to == 1) ++same_arc_candidates;
        }
        check(same_arc_candidates == 2, "Block 10: customer 1 has 2 candidate launch points on its entering arc, not 1");

        check(result.solution.drone_trips.size() == 1, "Block 10: customer 1 is drone-served");
        const NodeId chosen_launch = result.solution.drone_trips.front().launch_node;
        check(chosen_launch == 2,
              "Block 10: the FARTHER candidate (id=2, (30,0)) is chosen, proving the closer one (id=3) was not the only "
              "candidate considered");
        check_near(result.node_registry.at(chosen_launch).coordinates.x, 30.0, 1e-9,
                   "Block 10: the chosen launch point sits at x=30, not x=60");

        check(result.feasible, "Block 10: the resulting solution is feasible");
        check_near(result.objective_value, 60.0, 1e-9,
                   "Block 10: objective is 60 (the farther candidate's arrival time), not 75 (the closer candidate's)");

        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers);
        check(independent_eval.feasible == result.feasible,
              "Block 10: independent Phase E re-evaluation agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-9,
                   "Block 10: independent Phase E re-evaluation agrees on the objective value");
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
