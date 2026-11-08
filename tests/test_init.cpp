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
#include <vector>

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

// Test-fixture FLP policy: eq. (41) structure n = floor(D/d) with a d chosen
// by the TEST. The paper gives no main-experiment d; none of the values
// used in this file is a paper value.
InitialSolutionPolicy unit_division_policy(double d) {
    InitialSolutionPolicy policy;
    policy.flp_policy.count_rule = tdr::flp::FlpCountRule::UnitDivisionLength;
    policy.flp_policy.unit_division_length = d;
    return policy;
}

// Test L helper: the structural invariants the corrected Phase F must
// guarantee, checked DIRECTLY on the result (not only via the evaluator):
//  - every launch point on the route belongs to L_(a,b) of the truck nodes
//    a / b immediately around it (p9; (13)-(15));
//  - no launch point is visited twice ((8)-(9));
//  - every visited launch point launches at least one drone ((6));
//  - every trip launches at an FLP of the route and carries a physical
//    drone id 0 <= u < |U|;
//  - no physical drone is relaunched before its previous trip's recovery
//    position ((28), route-order form);
//  - the builder's own evaluation is feasible.
void check_phase_f_invariants(const InitialSolutionResult& r, int num_drones, const std::string& lbl) {
    const auto& route = r.solution.truck_route;
    const auto is_flp = [&](NodeId v) { return r.node_registry.at(v).type == NodeType::FlexibleLaunchPoint; };
    bool membership = true, once = true, launches = true, ids = true, reuse = true, launch_on_route = true;
    std::unordered_map<NodeId, int> count;
    for (std::size_t p = 0; p < route.size(); ++p) {
        if (!is_flp(route[p])) continue;
        if (++count[route[p]] > 1) once = false;
        NodeId a = kInvalidNodeId, b = kInvalidNodeId;
        for (std::size_t q = p; q-- > 0;) { if (!is_flp(route[q])) { a = route[q]; break; } }
        for (std::size_t q = p + 1; q < route.size(); ++q) { if (!is_flp(route[q])) { b = route[q]; break; } }
        const auto it = r.flp_data.find(route[p]);
        if (it == r.flp_data.end() || it->second.arc_from != a || it->second.arc_to != b) membership = false;
        bool any = false;
        for (const auto& t : r.solution.drone_trips) any = any || t.launch_node == route[p];
        if (!any) launches = false;
    }
    struct Interval { int u; std::size_t lp; std::size_t rp; };
    std::vector<Interval> intervals;
    for (const auto& t : r.solution.drone_trips) {
        if (t.drone_id < 0 || t.drone_id >= num_drones) ids = false;
        const auto lp = std::find(route.begin(), route.end(), t.launch_node);
        if (lp == route.end() || !is_flp(t.launch_node)) { launch_on_route = false; continue; }
        const auto rp = std::find(lp + 1, route.end(), t.recovery_node);
        if (rp == route.end()) { launch_on_route = false; continue; }
        intervals.push_back(Interval{t.drone_id, static_cast<std::size_t>(lp - route.begin()),
                                     static_cast<std::size_t>(rp - route.begin())});
    }
    for (std::size_t x = 0; x < intervals.size(); ++x) {
        for (std::size_t y = 0; y < intervals.size(); ++y) {
            if (x == y || intervals[x].u != intervals[y].u) continue;
            if (intervals[x].lp < intervals[y].lp && intervals[y].lp <= intervals[x].rp) reuse = false;
        }
    }
    check(membership, lbl + " (test L): every route launch point belongs to L_(a,b) of the truck nodes a, b around it");
    check(once, lbl + " (test L): no launch point is visited twice (constraints (8)-(9))");
    check(launches, lbl + " (test L): every visited launch point launches a drone (constraint (6))");
    check(launch_on_route, lbl + " (test L): every trip launches at a route FLP and recovers later on the route");
    check(ids, lbl + " (test L): every trip carries a physical drone id 0 <= u < |U|");
    check(reuse, lbl + " (test L): no physical drone is relaunched before its previous recovery (constraint (28))");
    check(r.evaluation.max_simultaneous_drones <= num_drones, lbl + " (test L): never more than |U| drones in flight");
    check(r.feasible && r.evaluation.violation_reasons.empty(), lbl + " (test L): the builder's own evaluation is feasible");
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
    // Conventions for Blocks 4-12 (paper-conformance corrections, 29 Sep
    // 2026). F3 now generates L_(a,b) for EVERY ordered pair of truck nodes
    // (depot + customers) with n = floor(D/d) points at ratios m/(n+1)
    // (eq. (41) structure). The paper does NOT give the main-experiment d,
    // so there is no default: every block below sets a TEST-FIXTURE d
    // chosen only to keep the hand arithmetic small. None of these d values
    // is a paper value. FLP ids start at max(depot, customers) + 1 and are
    // assigned arc by arc in the order (a, b) for a, b in [depot, sorted
    // customers], a != b. F4's launch candidates for customer j are the
    // points of L_(a,b), where a / b are the truck nodes before / after j
    // (the arc the route traverses once j is removed); recovery is b.
    // Every expected value was derived by hand and cross-checked against an
    // independent Python re-implementation of F3/F4 and constraints
    // (6), (8)-(9), (13)-(15)/p9, (18), (21), (28), (36), (37) and |U|.
    // =====================================================================

    // =====================================================================
    // Block 4 (F4 endurance rejection). Depot 0=(0,0), customers 1=(50,0),
    // 2=(100,0). truck=drone speed 1, e=10, |U|=1, SL=SR=Se=theta=0,
    // test-fixture d=25.
    // F1: r0 = [0,2,1,0] (frozen deterministic solver; F1 is unchanged).
    // F2: k = ceil(2/3) = 1, one cluster {1,2}.
    // F3: floor(D/25): |01|=50 -> 2, |02|=100 -> 4, |12|=50 -> 2, per
    //     direction 8 -> 16 launch points.
    // F4: customer 1 (a=2, b=0): L_(2,0) = x in {80,60,40,20}, recovery at
    //     the depot: the return leg alone is d^(1,0) = 50 > e, so (37)
    //     >= 50 > 10 for every candidate. Customer 2 (a=0, b=1): L_(0,1) =
    //     x in {16.7, 33.3}: the outbound leg alone is >= 66.7 > 10.
    //     Every candidate violates (36)/(37); nothing is converted.
    // Result: the truck-only r0; objective = 100 (cust 2) + 150 (cust 1).
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{50.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{100.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 1.0;
        drone.endurance_seconds = 10.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};

        InitialSolutionResult result = build_initial_solution(0, {1, 2}, customers, network, unit_division_policy(25.0));

        check(result.candidate_flps.size() == 16, "Block 4: F3 generates floor(D/25) points on each of the 6 ordered arcs (16)");
        check(result.solution.drone_trips.empty(),
              "Block 4 (endurance rejection): no drone trip is constructed -- every candidate violates (36)/(37)");
        check(result.solution.truck_route == result.truck_only_route,
              "Block 4 (endurance rejection): the truck route is r0 unchanged (every customer truck-served)");
        check(result.truck_only_route == TruckRoute({0, 2, 1, 0}), "Block 4: r0 = [0,2,1,0]");
        check(result.feasible, "Block 4 (endurance rejection): the resulting truck-only solution is feasible");
        check_near(result.objective_value, 250.0, 1e-9, "Block 4 (endurance rejection): objective = 100 + 150 = 250");
    }

    // =====================================================================
    // Block 5 (recovery-before-launch rejection): Phase F's own candidate
    // generation can never propose a trip whose recovery position
    // precedes its launch position (recovery is always found by scanning
    // FORWARD from the launch position -- see initial_solution_builder.hpp
    // F4). What this test verifies instead is the feasibility gate F4
    // depends on: that tdr::timing::evaluate_solution (reused here rather
    // than reimplemented) correctly rejects
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
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 1000.0; drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        Solution sol;
        sol.truck_route = {0, 1, 2, 0}; // custB (3) is drone-served, not in the truck route
        DroneTrip trip;
        trip.launch_node = 2;
        trip.service_customer = 3;
        trip.recovery_node = 1; // position 1, before launch's position 2
        trip.drone_id = 0;
        sol.drone_trips = {trip};

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};
        customers[3] = CustomerData{3, 0.0, 0.0, 0.0};

        tdr::timing::SolutionEvaluationResult eval = tdr::timing::evaluate_solution(sol, network, customers, {{2, FlexibleLaunchPointData{2, 1, 0, 0.5}}});
        check(!eval.drone_trip_timings[0].on_route,
              "Recovery-before-launch: the feasibility gate marks the trip as not on-route");
        check(!eval.drone_trip_timings[0].feasible,
              "Recovery-before-launch: the feasibility gate marks the trip infeasible");
        check(!eval.feasible, "Recovery-before-launch: the overall trial solution is infeasible");
    }

    // =====================================================================
    // Block 6: full end-to-end construction on 4 colinear customers, 1
    // drone. Depot 0=(0,0), customers 1..4 at x = 10, 20, 30, 40 (y=0).
    // truck speed 1, drone speed 2, SL=SR=Se=theta=0, e=100, |U|=1,
    // test-fixture d=10.
    //
    // F1: r0 = [0,4,3,2,1,0], cost 80 (frozen deterministic solver).
    // F2: k=2: cluster0 = {1,2}, cluster1 = {3,4}.
    // F3: floor(D/10) on each of the 20 ordered arcs. Per direction:
    //     0-1..0-4: 1+2+3+4; 1-2..1-4: 1+2+3; 2-3, 2-4: 1+2; 3-4: 1 -> 20,
    //     so 40 launch points, ids 5..44. The ones that matter:
    //     L_(0,3) = {8, 9, 10} at x = 7.5, 15, 22.5;
    //     L_(2,0) = {22, 23} at x = 13.33, 6.67;
    //     L_(3,1) = {31, 32} at x = 23.33, 16.67.
    //
    // F4 cluster0, round 1 (route r0):
    //   j=1 (a=2, b=0): f22: truck at f22 = 40+10+10+6.667 = 66.667,
    //     t_1 = 66.667 + 3.333/2 = 68.333;  f23: 73.333 + 1.667 = 75.
    //   j=2 (a=3, b=1): f31: truck at f31 = 40+10+6.667 = 56.667,
    //     t_2 = 56.667 + 3.333/2 = 58.333;  f32: 63.333 + 1.667 = 65.
    //   BestRoute: j=2 via f31 (58.333), recovery at customer 1 (locked).
    //   Round 2: nothing left (1 locked, 2 converted).
    // F4 cluster1 (route [0,4,3,31,1,0]):
    //   j=3 (a=4, b=1 -- f31 is skipped as an FLP): a trip over route
    //     positions 2..4 overlaps trip(2) at positions 3..4, so it needs 2
    //     drones > |U|=1 (and f31, owned by L_(3,1), would then sit between
    //     4 and 1): rejected.
    //   j=4 (a=0, b=3): f8: t_4 = 7.5 + 32.5/2 = 23.75; f9: 15 + 12.5 =
    //     27.5; f10: 22.5 + 8.75 = 31.25. BestRoute: f8.
    //     Endurance of f8 -> 4 -> 3: l^_i = 7.5, l^_j = 23.75, loaded 16.25;
    //     t_3 = 7.5 + 22.5 = 30; (36) = (30 - 23.75) + 16.25 = 22.5;
    //     (37) = 16.25 + 10/2 = 21.25; both <= 100.
    //
    // Final route [0, 8, 3, 31, 1, 0]; trips (8 -> 4 -> 3) and (31 -> 2 -> 1),
    // BOTH flown by drone 0 (positions 1..2, then 3..4: sequential reuse).
    // Timing: f8 7.5; C3 30 (drone landed 28.75 and waited); f31 36.667;
    // t_2 = 38.333, lands at C1 at 43.333; truck at C1 50; depot 60.
    // Objective = 23.75 (4) + 30 (3) + 38.333 (2) + 50 (1) = 142.0833...
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

        InitialSolutionResult result = build_initial_solution(0, {1, 2, 3, 4}, customers, network, unit_division_policy(10.0));

        check(result.tsp_result.exact, "Block 6: F1 tour is exact (n=5, well within Held-Karp's range)");
        check_near(result.tsp_result.cost, 80.0, 1e-9, "Block 6: F1 optimal cost is 80");
        check(result.truck_only_route == TruckRoute({0, 4, 3, 2, 1, 0}), "Block 6: r0 = [0,4,3,2,1,0]");

        check(result.clustering.cluster_count == 2, "Block 6: F2 produces 2 clusters");
        check(result.clustering.assignment.at(1) == result.clustering.assignment.at(2),
              "Block 6: F2 groups customers 1 and 2 together");
        check(result.clustering.assignment.at(3) == result.clustering.assignment.at(4),
              "Block 6: F2 groups customers 3 and 4 together");

        check(result.candidate_flps.size() == 40, "Block 6: F3 generates sum floor(D/10) = 40 launch points over the 20 ordered arcs");
        bool flp_geometry_ok = true;
        for (const auto& gen : result.candidate_flps) {
            const Point2D& a = nodes.at(gen.data.arc_from).coordinates;
            const Point2D& b = nodes.at(gen.data.arc_to).coordinates;
            const double len = std::hypot(b.x - a.x, b.y - a.y);
            const int n = static_cast<int>(std::floor(len / 10.0 + 1e-9));
            const double m = gen.data.position_ratio * (n + 1);
            if (std::fabs(m - std::round(m)) > 1e-9 || std::round(m) < 1 || std::round(m) > n) flp_geometry_ok = false;
            if (std::fabs(gen.node.coordinates.x - (a.x + gen.data.position_ratio * (b.x - a.x))) > 1e-9) flp_geometry_ok = false;
            if (std::fabs(gen.node.coordinates.y - (a.y + gen.data.position_ratio * (b.y - a.y))) > 1e-9) flp_geometry_ok = false;
            if (result.flp_data.at(gen.node.id).arc_from != gen.data.arc_from ||
                result.flp_data.at(gen.node.id).arc_to != gen.data.arc_to) flp_geometry_ok = false;
        }
        check(flp_geometry_ok, "Block 6: F3 every launch point sits at ratio m/(n+1), n = floor(D/d), of its own arc");
        check(result.flp_data.at(8).arc_from == 0 && result.flp_data.at(8).arc_to == 3 &&
                  std::fabs(result.node_registry.at(8).coordinates.x - 7.5) < 1e-9,
              "Block 6: FLP 8 is the first point of L_(0,3), at x = 7.5");
        check(result.flp_data.at(31).arc_from == 3 && result.flp_data.at(31).arc_to == 1 &&
                  std::fabs(result.node_registry.at(31).coordinates.x - 70.0 / 3.0) < 1e-9,
              "Block 6: FLP 31 is the first point of L_(3,1), at x = 23.33");

        check(result.solution.truck_route == TruckRoute({0, 8, 3, 31, 1, 0}), "Block 6: final route is [0, 8, 3, 31, 1, 0]");
        check(result.solution.drone_trips.size() == 2, "Block 6: F4 constructs exactly 2 drone trips");
        bool trip_for_2 = false, trip_for_4 = false;
        for (const auto& t : result.solution.drone_trips) {
            if (t.service_customer == 2) trip_for_2 = (t.launch_node == 31 && t.recovery_node == 1 && t.drone_id == 0);
            if (t.service_customer == 4) trip_for_4 = (t.launch_node == 8 && t.recovery_node == 3 && t.drone_id == 0);
        }
        check(trip_for_2, "Block 6: customer 2 is served 31 -> 2 -> 1 by drone 0");
        check(trip_for_4, "Block 6: customer 4 is served 8 -> 4 -> 3 by drone 0 (the same physical drone, reused)");

        check(result.feasible, "Block 6: the final initial solution is feasible");
        check_near(result.objective_value, 23.75 + 30.0 + 115.0 / 3.0 + 50.0, 1e-9,
                   "Block 6: hand-computed objective is 142.0833");
        for (std::size_t i = 0; i < result.solution.drone_trips.size(); ++i) {
            if (result.solution.drone_trips[i].service_customer != 4) continue;
            const auto& tm = result.evaluation.drone_trip_timings[i];
            check_near(tm.endurance_value_36, 22.5, 1e-9, "Block 6: trip 8->4->3 (36) = 22.5");
            check_near(tm.endurance_value_37, 21.25, 1e-9, "Block 6: trip 8->4->3 (37) = 21.25");
        }

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

        check_phase_f_invariants(result, 1, "Block 6");

        // Item 10: re-evaluate the FINAL solution independently (a fresh
        // network built from the result's own node_registry and the result's
        // own arc-ownership data) and confirm it agrees with the builder.
        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers, result.flp_data);
        check(independent_eval.feasible == result.evaluation.feasible,
              "Block 6: independently re-running the Phase E engine agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-9,
                   "Block 6: independently re-running the Phase E engine agrees on the objective value");
    }

    // =====================================================================
    // Block 7: direct unit tests of check_drone_fleet_capacity() -- a pure
    // time-interval sweep-line utility. Since the 29 Sep 2026 correction
    // F4 NO LONGER uses it: fleet limits are now enforced through explicit
    // physical drone ids and the evaluator's route-order (28) and |U|
    // checks. The utility is retained (and its arithmetic pinned here)
    // only as a standalone helper; see DECISIONS.md.
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
    // Block 8: a candidate of a DIFFERENT cluster is blocked because it
    // would need a second drone simultaneously in the air (|U| = 1). Runs
    // the ACTUAL pipeline (real K-means + real TSP).
    //
    // Depot=(0,0); customers 1=(10,5), 2=(20,0), 3=(30,5), 4=(40,0).
    // truck speed 1, drone speed 2, SL=SR=Se=theta=0, e=1000, |U|=1,
    // test-fixture d=10.
    // F1: r0 = [0,2,4,3,1,0], cost 60 + 2*sqrt(125). F2: {1,2}, {3,4}.
    // F3: per direction 0-1 (11.18) 1, 0-2 (20) 2, 0-3 (30.41) 3, 0-4 (40)
    //     4, 1-2 1, 1-3 (20) 2, 1-4 (30.41) 3, 2-3 1, 2-4 (20) 2, 3-4 1 ->
    //     20; 40 in all. L_(0,4) = {11..14} at x = 8, 16, 24, 32;
    //     L_(3,0) = {28, 29, 30} at (22.5,3.75), (15,2.5), (7.5,1.25).
    // F4 cluster0, round 1 (route r0):
    //   j=2 (a=0, b=4): f11: t_2 = 8 + 12/2 = 14 (f12: 18, f13: 26, f14: 38).
    //   j=1 (a=3, b=0): f28: 65.06, f29: 69.18, f30: 76.24.
    //   BestRoute: j=2 via f11 (14); customer 4 becomes recovery-locked.
    // Round 2 (route [0,11,4,3,1,0]): j=1 via f28: truck at f28 =
    //   8 + 32 + sqrt(125) + sqrt(57.8125) = 58.7838; t_1 = 58.7838 +
    //   sqrt(157.8125)/2 = 65.0650. Accepted (recovery at the depot).
    // F4 cluster1 (route [0,11,4,3,28,0]): 4 is locked. j=3 (a=4, b=0,
    //   f28 skipped): the trip would span positions 2..5 and overlap trip(1)
    //   at positions 4..5 -> 2 drones > |U| = 1: rejected. Cluster1 ends
    //   with no trip.
    // Final route [0, 11, 4, 3, 28, 0]; objective = 14 (2) + 40 (4) +
    // (40 + sqrt(125)) (3) + 65.0650 (1) = 170.2453052010712.
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

        InitialSolutionResult result = build_initial_solution(0, {1, 2, 3, 4}, customers, network, unit_division_policy(10.0));

        check(result.tsp_result.exact, "Block 8: F1 tour is exact");
        check_near(result.tsp_result.cost, 60.0 + 2.0 * std::sqrt(125.0), 1e-9, "Block 8: F1 optimal cost matches hand formula");
        check(result.truck_only_route == TruckRoute({0, 2, 4, 3, 1, 0}), "Block 8: r0 = [0,2,4,3,1,0]");

        check(result.clustering.cluster_count == 2, "Block 8: F2 produces 2 clusters");
        auto cluster_of = [&](NodeId id) { return result.clustering.assignment.at(id); };
        check(cluster_of(1) == cluster_of(2) && cluster_of(3) == cluster_of(4) && cluster_of(1) != cluster_of(3),
              "Block 8: F2 groups {1,2} and {3,4} into two DIFFERENT clusters");

        check(result.candidate_flps.size() == 40, "Block 8: F3 generates 40 launch points");
        check(result.solution.truck_route == TruckRoute({0, 11, 4, 3, 28, 0}), "Block 8: final route is [0, 11, 4, 3, 28, 0]");
        check(result.solution.drone_trips.size() == 2, "Block 8: exactly 2 drone trips accepted overall");
        bool customer1_is_drone = false, customer2_is_drone = false, customer3_is_drone = false, customer4_is_drone = false;
        for (const auto& t : result.solution.drone_trips) {
            if (t.service_customer == 1) customer1_is_drone = (t.launch_node == 28 && t.recovery_node == 0);
            if (t.service_customer == 2) customer2_is_drone = (t.launch_node == 11 && t.recovery_node == 4);
            if (t.service_customer == 3) customer3_is_drone = true;
            if (t.service_customer == 4) customer4_is_drone = true;
        }
        check(customer1_is_drone && customer2_is_drone, "Block 8: cluster0's customers 1 (28->1->0) and 2 (11->2->4) are drone-served");
        check(!customer3_is_drone && !customer4_is_drone,
              "Block 8: cluster1's customers 3 and 4 remain truck-served (customer 3 would need a 2nd simultaneous "
              "drone, customer 4 is recovery-locked)");
        check(result.evaluation.max_simultaneous_drones <= 1,
              "Block 8: the delivered solution never has more than |U| = 1 drone in flight");

        check(result.feasible, "Block 8: the final initial solution is feasible");
        check_near(result.objective_value, 170.2453052010712, 1e-9, "Block 8: hand-computed objective is 170.2453052010712");
        check_phase_f_invariants(result, 1, "Block 8");

        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers, result.flp_data);
        check(independent_eval.feasible == result.feasible,
              "Block 8: independent Phase E re-evaluation agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-9,
                   "Block 8: independent Phase E re-evaluation agrees on the objective value");
    }

    // =====================================================================
    // Block 9: two DIFFERENT clusters far apart; the single physical drone
    // is REUSED (drone 0 flies both trips, one after the other).
    //
    // Depot=(0,0); customers 1=(10,0), 2=(20,0), 3=(500,0), 4=(510,0).
    // truck speed 1, drone speed 2, SL=SR=Se=theta=0, e=1000, |U|=1,
    // test-fixture d=100.
    // F1: r0 = [0,4,3,2,1,0], cost 1020. F2: {1,2}, {3,4}.
    // F3: per direction 0-3 5, 0-4 5, 1-3 4, 1-4 5, 2-3 4, 2-4 4 (the
    //     short arcs 0-1, 0-2, 1-2, 3-4 get none) -> 27; 54 in all.
    //     L_(0,3) = {5..9} at x = 83.33, 166.67, 250, 333.33, 416.67;
    //     L_(3,1) = {37..40} at x = 402, 304, 206, 108; L_(2,0) is empty.
    // F4 cluster0 (route r0): j=1 (a=2, b=0): L_(2,0) empty, no candidate.
    //   j=2 (a=3, b=1): f37: truck at f37 = 510 + 10 + 98 = 618, t_2 =
    //   618 + 382/2 = 809 (f38 858, f39 907, f40 956). Accept f37;
    //   customer 1 locked.
    // F4 cluster1 (route [0,4,3,37,1,0]): j=3 (a=4, b=1): overlaps trip(2)
    //   -> 2 drones > 1: rejected. j=4 (a=0, b=3): f5: t_4 = 83.333 +
    //   426.667/2 = 296.667 (the other four are later). Accept f5.
    // Final route [0, 5, 3, 37, 1, 0]:
    //   trip 5->4->3: launch 83.333, lands 301.667, truck at C3 at 500,
    //     recovery completes at 500 (route positions 1..2);
    //   trip 37->2->1: launch 500 + 98 = 598, t_2 = 789, lands 794,
    //     truck at C1 at 990, recovery completes at 990 (positions 3..4).
    //   500 <= 598: drone 0 is back on the truck before it is relaunched.
    // Objective = 296.667 + 500 + 789 + 990 = 2575.6667.
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

        InitialSolutionResult result = build_initial_solution(0, {1, 2, 3, 4}, customers, network, unit_division_policy(100.0));

        auto cluster_of = [&](NodeId id) { return result.clustering.assignment.at(id); };
        check(cluster_of(1) == cluster_of(2) && cluster_of(3) == cluster_of(4) && cluster_of(1) != cluster_of(3),
              "Block 9: customers split into two genuinely different clusters, {1,2} and {3,4}");
        check(result.candidate_flps.size() == 54, "Block 9: F3 generates 54 launch points (none on the short arcs)");
        check(result.solution.truck_route == TruckRoute({0, 5, 3, 37, 1, 0}), "Block 9: final route is [0, 5, 3, 37, 1, 0]");
        check(result.solution.drone_trips.size() == 2, "Block 9: both clusters' best candidates are accepted (2 trips)");

        double launch_a = -1.0, recover_a = -1.0, launch_b = -1.0, recover_b = -1.0;
        int drone_a = -1, drone_b = -1;
        for (std::size_t i = 0; i < result.solution.drone_trips.size(); ++i) {
            const auto& t = result.solution.drone_trips[i];
            const auto& tm = result.evaluation.drone_trip_timings[i];
            if (t.service_customer == 4) { launch_a = tm.launch_time; recover_a = tm.recovery_complete_time; drone_a = t.drone_id; }
            if (t.service_customer == 2) { launch_b = tm.launch_time; recover_b = tm.recovery_complete_time; drone_b = t.drone_id; }
        }
        check_near(launch_a, 250.0 / 3.0, 1e-9, "Block 9: customer4's trip launches at 83.333");
        check_near(recover_a, 500.0, 1e-9, "Block 9: customer4's trip completes recovery at 500");
        check_near(launch_b, 598.0, 1e-9, "Block 9: customer2's trip launches at 598");
        check_near(recover_b, 990.0, 1e-9, "Block 9: customer2's trip completes recovery at 990");
        check(recover_a <= launch_b, "Block 9: customer4's trip fully completes before customer2's trip launches (no overlap)");
        check(drone_a == 0 && drone_b == 0, "Block 9: both trips are flown by the same physical drone u = 0 (sequential reuse)");
        check(result.evaluation.max_simultaneous_drones == 1,
              "Block 9: the two different-cluster trips never have more than 1 drone in flight");

        check(result.feasible, "Block 9: the final initial solution is feasible");
        check_near(result.objective_value, 296.0 + 2.0 / 3.0 + 500.0 + 789.0 + 990.0, 1e-9,
                   "Block 9: hand-computed objective is 2575.6667");
        check_phase_f_invariants(result, 1, "Block 9");

        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers, result.flp_data);
        check(independent_eval.feasible == result.feasible,
              "Block 9: independent Phase E re-evaluation agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-9,
                   "Block 9: independent Phase E re-evaluation agrees on the objective value");
    }

    // =====================================================================
    // Block 10: EVERY launch point of the bridging arc is a candidate, and
    // the best one is not simply the first or last generated.
    //
    // Depot=(0,0), customers 1=(90,0), 2=(180,0). truck speed 1, drone
    // speed 0.5 (SLOWER than the truck), SL=SR=Se=theta=0, e=1000, |U|=1,
    // test-fixture d=45.
    // F1: r0 = [0,2,1,0]. F2: one cluster {1,2}.
    // F3: 0-1 (90) 2, 0-2 (180) 4, 1-2 (90) 2 per direction -> 16, ids
    //     3..18: L_(0,1) = {3, 4} at x = 30, 60; L_(2,0) = {13..16} at
    //     x = 144, 108, 72, 36.
    // F4: j=1 (a=2, b=0): truck reaches x at 180 + (180 - x); drone
    //     reaches customer 1 at that + |x - 90|/0.5:
    //       f13 (144): 216 + 108 = 324;  f14 (108): 252 + 36 = 288;
    //       f15 (72):  288 + 36  = 324;  f16 (36):  324 + 108 = 432.
    //     j=2 (a=0, b=1): f3 (30): 30 + 300 = 330; f4 (60): 60 + 240 = 300.
    //   BestRoute: j=1 via f14 (288) -- the SECOND point of L_(2,0).
    //   Endurance f14: loaded 36, return 90/0.5 = 180, truck at depot 360:
    //   (36) = (360 - 288) + 36 = 108, (37) = 36 + 180 = 216 <= 1000.
    //   Round 2: customer 2 now sits between the depot and f14, so a = b =
    //   depot and no arc exists; done.
    // Final route [0, 2, 14, 0]; objective = 180 (2) + 288 (1) = 468.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{90.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{180.0, 0.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 0.5;
        drone.endurance_seconds = 1000.0;
        drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);

        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};

        InitialSolutionResult result = build_initial_solution(0, {1, 2}, customers, network, unit_division_policy(45.0));

        check(result.truck_only_route == TruckRoute({0, 2, 1, 0}), "Block 10: r0 = [0,2,1,0]");
        check(result.candidate_flps.size() == 16, "Block 10: F3 generates 16 launch points");
        int bridging_arc_points = 0;
        for (const auto& g : result.candidate_flps) {
            if (g.data.arc_from == 2 && g.data.arc_to == 0) ++bridging_arc_points;
        }
        check(bridging_arc_points == 4, "Block 10: customer 1's bridging arc (2,0) carries 4 candidate launch points");

        check(result.solution.drone_trips.size() == 1, "Block 10: customer 1 is drone-served");
        const NodeId chosen_launch = result.solution.drone_trips.front().launch_node;
        check(chosen_launch == 14, "Block 10: the SECOND point of L_(2,0) (id 14) is chosen -- all four were evaluated");
        check_near(result.node_registry.at(chosen_launch).coordinates.x, 108.0, 1e-9,
                   "Block 10: the chosen launch point sits at x = 108");
        check(result.solution.truck_route == TruckRoute({0, 2, 14, 0}), "Block 10: final route is [0, 2, 14, 0]");

        check(result.feasible, "Block 10: the resulting solution is feasible");
        check_near(result.objective_value, 468.0, 1e-9, "Block 10: objective is 180 + 288 = 468");
        check_phase_f_invariants(result, 1, "Block 10");

        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers, result.flp_data);
        check(independent_eval.feasible == result.feasible,
              "Block 10: independent Phase E re-evaluation agrees on feasibility");
        check_near(independent_eval.objective_value, result.objective_value, 1e-9,
                   "Block 10: independent Phase E re-evaluation agrees on the objective value");
    }

    // =====================================================================
    // Block 11 = test L: a Phase F initial solution with valid FLP-arc
    // membership on a non-trivial instance (7 customers, |U| = 2, non-zero
    // SL, SR, Se^T, Se^D and theta). Depot (0,0); customers 1=(10,30),
    // 2=(25,5), 3=(40,35), 4=(55,10), 5=(70,40), 6=(15,-20), 7=(45,-25).
    // truck speed 1, drone speed 1.5, SL=1, SR=1, Se^T=2, Se^D=1,
    // theta=0.2, e=60, test-fixture d=8.
    // Expected (independent reference implementation): r0 =
    // [0,6,7,4,5,3,1,2,0]; 292 launch points; final route
    // [0, 39, 7, 290, 5, 3, 117, 2, 0] with trips 39->6->7, 117->1->2,
    // 290->4->5.
    // STAGE 2 (8 Nov 2026, SR Option B): the route and trips are UNCHANGED,
    // but the objective moved from 820.1434959367223 (pre-Stage-2 SR rule)
    // to 828.143495936722. Both figures were reproduced by a stand-alone
    // Python evaluator that shares no code with the C++ one (old rule ->
    // 820.1434959367, Option B -> 828.1434959367). The +8 is SR charged
    // additively (t + Se^T + r*SR) at the recovery customers 7, 5 and 2
    // instead of concurrently with their service.
    // Before the correction, F3 generated launch points only on r0's arcs
    // and F4 could leave a point of L_(a,j) in the route after j was
    // removed, i.e. between a and some unrelated node.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        const std::vector<Point2D> pts{{10, 30}, {25, 5}, {40, 35}, {55, 10}, {70, 40}, {15, -20}, {45, -25}};
        std::vector<NodeId> ids;
        std::unordered_map<NodeId, CustomerData> customers;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const NodeId id = static_cast<NodeId>(i + 1);
            nodes.add(Node{id, NodeType::Customer, pts[i]});
            ids.push_back(id);
            customers[id] = CustomerData{id, 2.0, 1.0, 0.2};
        }
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone;
        drone.speed = 1.5;
        drone.launch_prep_time_sl = 1.0;
        drone.recovery_time_sr = 1.0;
        drone.endurance_seconds = 60.0;
        drone.num_drones = 2;
        SpatialNetwork network(nodes, truck, drone);

        InitialSolutionResult result = build_initial_solution(0, ids, customers, network, unit_division_policy(8.0));

        check(result.truck_only_route == TruckRoute({0, 6, 7, 4, 5, 3, 1, 2, 0}), "L: r0 = [0,6,7,4,5,3,1,2,0]");
        check(result.candidate_flps.size() == 292, "L: F3 generates 292 launch points over the 56 ordered arcs");
        check(result.solution.truck_route == TruckRoute({0, 39, 7, 290, 5, 3, 117, 2, 0}),
              "L: final route is [0, 39, 7, 290, 5, 3, 117, 2, 0]");
        check(result.solution.drone_trips.size() == 3, "L: three customers are converted to drone service");
        check_phase_f_invariants(result, 2, "L");
        check_near(result.objective_value, 828.143495936722, 1e-9, "L: objective matches the independent reference under SR Option B (828.1435)");

        // Every route launch point is on the arc between its truck
        // neighbours, and that arc is one the final truck route traverses.
        bool arcs_traversed = true;
        const auto& route = result.solution.truck_route;
        std::vector<NodeId> truck_only;
        for (NodeId v : route) {
            if (result.node_registry.at(v).type != NodeType::FlexibleLaunchPoint) truck_only.push_back(v);
        }
        for (NodeId v : route) {
            if (result.node_registry.at(v).type != NodeType::FlexibleLaunchPoint) continue;
            const auto& data = result.flp_data.at(v);
            bool found = false;
            for (std::size_t q = 0; q + 1 < truck_only.size(); ++q) {
                if (truck_only[q] == data.arc_from && truck_only[q + 1] == data.arc_to) found = true;
            }
            if (!found) arcs_traversed = false;
        }
        check(arcs_traversed, "L: the owning arc (i,j) of every used launch point is an arc of the final truck route");

        SpatialNetwork rebuilt_network(result.node_registry, truck, drone);
        tdr::timing::SolutionEvaluationResult independent_eval =
            tdr::timing::evaluate_solution(result.solution, rebuilt_network, customers, result.flp_data);
        check(independent_eval.feasible && independent_eval.violation_reasons.empty(),
              "L: independent Phase E re-evaluation (with arc-membership checks) finds no violation");
    }

    // =====================================================================
    // Block 12: F3 refuses to run without a configured d (no invented
    // paper default). The default InitialSolutionPolicy leaves d = 0.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0.0, 0.0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10.0, 0.0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{20.0, 5.0}});
        TruckParameters truck; truck.speed = 1.0;
        DroneParameters drone; drone.speed = 1.0; drone.endurance_seconds = 100.0; drone.num_drones = 1;
        SpatialNetwork network(nodes, truck, drone);
        std::unordered_map<NodeId, CustomerData> customers;
        customers[1] = CustomerData{1, 0.0, 0.0, 0.0};
        customers[2] = CustomerData{2, 0.0, 0.0, 0.0};

        InitialSolutionResult result = build_initial_solution(0, {1, 2}, customers, network);
        check(!result.feasible, "Block 12: with no d configured the builder reports no feasible solution");
        bool diag = false;
        for (const auto& s : result.diagnostics) {
            if (s.rfind("F3: FLP unit division length d is not configured", 0) == 0) diag = true;
        }
        check(diag, "Block 12: the diagnostic states that d is not configured (the paper gives no main-experiment d)");
        check(result.candidate_flps.empty() && result.solution.drone_trips.empty(),
              "Block 12: no launch points and no drone trips are produced");
    }

    std::cout << "\n" << (g_failures == 0 ? "ALL TESTS PASSED" : std::to_string(g_failures) + " TEST(S) FAILED") << "\n";
    return g_failures == 0 ? 0 : 1;
}
