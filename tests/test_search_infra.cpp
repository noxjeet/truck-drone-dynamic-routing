// M1 tests: static-reproduction infrastructure (RNG, config, instance
// generator, FLP arc index, shared drone-id assignment, route operations,
// candidate window). Every expected value is derived by hand in comments or
// from an independent Python reference (RNG vectors); none is produced by
// calling the code under test to fill in the "expected" side.

#include "config/static_config.hpp"
#include "flp/flexible_launch_point_generator.hpp"
#include "flp/flp_arc_index.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/node.hpp"
#include "model/solution.hpp"
#include "search/candidate_window.hpp"
#include "search/drone_assignment.hpp"
#include "search/route_ops.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <set>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

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

DroneTrip trip(int drone, NodeId launch, NodeId customer, NodeId recovery) {
    DroneTrip t;
    t.drone_id = drone;
    t.launch_node = launch;
    t.service_customer = customer;
    t.recovery_node = recovery;
    return t;
}

std::uint64_t fnv(std::uint64_t h, std::uint64_t v) { h ^= v; h *= 0x100000001b3ULL; return h; }

} // namespace

int main() {
    // =====================================================================
    // 1. RNG. Vectors are from an independent pure-Python implementation of
    // the published algorithms (SplitMix64: the seed-0 outputs are also the
    // widely published reference values; xoshiro256** seeded through
    // SplitMix64).
    // =====================================================================
    {
        tdr::util::SplitMix64 sm(0);
        check(sm.next() == 0xe220a8397b1dcdafULL, "SplitMix64(0) output 1 = e220a8397b1dcdaf (published reference)");
        check(sm.next() == 0x6e789e6aa1b965f4ULL, "SplitMix64(0) output 2 = 6e789e6aa1b965f4 (published reference)");
        check(sm.next() == 0x06c45d188009454fULL, "SplitMix64(0) output 3 = 06c45d188009454f (published reference)");

        tdr::util::Rng r(1);
        check(r.next_u64() == 0xb3f2af6d0fc710c5ULL, "xoshiro256** seed 1: output 1");
        check(r.next_u64() == 0x853b559647364ceaULL, "xoshiro256** seed 1: output 2");
        check(r.next_u64() == 0x92f89756082a4514ULL, "xoshiro256** seed 1: output 3");
        check(r.next_u64() == 0x642e1c7bc266a3a7ULL, "xoshiro256** seed 1: output 4");

        tdr::util::Rng u(12345);
        check_near(u.uniform01(), 0.7438081631565894, 1e-15, "uniform01 seed 12345, draw 1");
        check_near(u.uniform01(), 0.13004553462783452, 1e-15, "uniform01 seed 12345, draw 2");

        tdr::util::Rng ui(99);
        const std::vector<int> expect{0, 4, 4, 5, -3, -5, -4, -1};
        bool all = true;
        for (int e : expect) all = all && (ui.uniform_int(-5, 5) == e);
        check(all, "uniform_int(-5,5) seed 99 reproduces the Python reference sequence 0,4,4,5,-3,-5,-4,-1");

        tdr::util::Rng sh(7);
        std::vector<int> v{0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        sh.shuffle(v);
        check(v == std::vector<int>({8, 3, 9, 0, 7, 2, 1, 6, 5, 4}), "shuffle(0..9) seed 7 = 8,3,9,0,7,2,1,6,5,4 (Python reference)");

        // Statistical sanity (not a proof): 60000 draws from {0..5}; each
        // bucket expects 10000; a 5% band is > 5 standard deviations (sd = 91).
        tdr::util::Rng sd(2026);
        int bucket[6] = {0, 0, 0, 0, 0, 0};
        bool in_range = true;
        for (int i = 0; i < 60000; ++i) {
            const auto x = sd.uniform_int(0, 5);
            if (x < 0 || x > 5) { in_range = false; continue; }
            ++bucket[x];
        }
        bool balanced = in_range;
        for (int b : bucket) balanced = balanced && std::abs(b - 10000) < 500;
        check(balanced, "uniform_int(0,5): always in range and each value within 5% of its expected count");

        tdr::util::Rng same(5);
        check(same.uniform_int(7, 7) == 7, "uniform_int(7,7) = 7");
        tdr::util::Rng m(5);
        double lo = 1.0, hi = 0.0;
        for (int i = 0; i < 10000; ++i) { const double x = m.uniform01(); lo = std::min(lo, x); hi = std::max(hi, x); }
        check(lo >= 0.0 && hi < 1.0, "uniform01 stays within [0,1)");

        // Determinism: identical seed -> identical stream; reseed restarts it.
        tdr::util::Rng a(31337), b(31337);
        bool same_stream = true;
        for (int i = 0; i < 100; ++i) same_stream = same_stream && (a.next_u64() == b.next_u64());
        check(same_stream, "two generators with the same seed produce identical streams");
        a.reseed(31337);
        tdr::util::Rng c(31337);
        check(a.next_u64() == c.next_u64(), "reseed(s) restarts the stream exactly");

        // derive_seed vectors (Python reference).
        using tdr::util::SeedStream;
        check(tdr::util::derive_seed(20261109ULL, SeedStream::Instance, 0) == 0x5ffca0db06b1a925ULL, "derive_seed(20261109, Instance, 0)");
        check(tdr::util::derive_seed(20261109ULL, SeedStream::Instance, 1) == 0x5d23561b091f0b05ULL, "derive_seed(20261109, Instance, 1)");
        check(tdr::util::derive_seed(20261109ULL, SeedStream::InitialSolution, 0) == 0x9bdb510ea00ea6b6ULL, "derive_seed(20261109, InitialSolution, 0)");
        check(tdr::util::derive_seed(1, SeedStream::Vns, 3) != tdr::util::derive_seed(1, SeedStream::Shake, 3),
              "different streams give different seeds");
        check(tdr::util::derive_seed(1, SeedStream::Vns, 3) == tdr::util::derive_seed(1, SeedStream::Vns, 3),
              "derive_seed is a pure function");
    }

    // =====================================================================
    // 2. Configuration: defaults and validation.
    // =====================================================================
    {
        tdr::config::StaticConfig c;
        check(tdr::config::validate(c).empty(), "default StaticConfig is valid");
        check(c.instance.area_side_m == 60000.0, "default area 60 km [PAPER Sec 6.1]");
        check(c.vehicle.truck_speed_mps == 15.0 && c.vehicle.drone_speed_mps == 12.0, "default speeds 15 / 12 m/s [PAPER Sec 6.1]");
        check(c.vehicle.endurance_s == 1800.0, "default endurance 1800 s [PAPER Sec 6.1]");
        check(c.vehicle.launch_prep_time_s == 60.0 && c.vehicle.recovery_time_s == 40.0, "default SL = 60, SR = 40 [PAPER Sec 6.1]");
        check(c.vehicle.num_drones == 2, "default fleet |U| = 2 [PAPER Sec 6.1]");
        check(c.instance.truck_service_time_s == 30.0 && c.instance.drone_service_time_s == 30.0 && c.instance.theta == 1.0,
              "default service time 30 s, theta 1 [PAPER Sec 6.1]");
        check(c.instance.depot_placement == tdr::config::DepotPlacement::Centre, "default depot = centre of the square [D9]");
        check(c.neighbourhood.adjacency_window == 3 && c.neighbourhood.n1_repeats == 2 && c.neighbourhood.n2_repeats == 2,
              "default adjacency window 3, N1/N2 repeats 2 [D1, D2]");

        const auto bad = [&](auto mutate, const std::string& what) {
            tdr::config::StaticConfig x;
            mutate(x);
            check(!tdr::config::validate(x).empty(), "validate rejects " + what);
        };
        bad([](auto& x) { x.instance.num_customers = 0; }, "num_customers = 0");
        bad([](auto& x) { x.instance.area_side_m = -1; }, "negative area");
        bad([](auto& x) { x.instance.theta = -0.5; }, "negative theta");
        bad([](auto& x) { x.instance.dataset_index = -1; }, "negative dataset index");
        bad([](auto& x) { x.vehicle.truck_speed_mps = 0; }, "zero truck speed");
        bad([](auto& x) { x.vehicle.drone_speed_mps = std::nan(""); }, "NaN drone speed");
        bad([](auto& x) { x.vehicle.endurance_s = 0; }, "zero endurance");
        bad([](auto& x) { x.vehicle.num_drones = 0; }, "zero drones");
        bad([](auto& x) { x.flp.unit_division_length_m = 0; }, "d = 0");
        bad([](auto& x) { x.neighbourhood.adjacency_window = 0; }, "adjacency window 0");
        bad([](auto& x) { x.neighbourhood.n1_repeats = 0; }, "n1_repeats 0");
        bad([](auto& x) {
            x.instance.depot_placement = tdr::config::DepotPlacement::Explicit;
            x.instance.depot_x_m = 70000.0;
        }, "explicit depot outside the square");
    }

    // =====================================================================
    // 3. Instance generator.
    // =====================================================================
    {
        tdr::config::StaticConfig c;
        c.instance.num_customers = 10;
        const auto r1 = tdr::instance::generate_static_instance(c);
        const auto r2 = tdr::instance::generate_static_instance(c);
        check(r1.ok && r2.ok, "instance generation succeeds for the default config");
        const auto& i1 = r1.instance;
        const auto& i2 = r2.instance;
        check(i1.customers.size() == 10 && i1.nodes.size() == 11, "10 customers + depot");
        check(i1.depot == 0 && i1.customers.front() == 1 && i1.customers.back() == 10, "ids: depot 0, customers 1..10");

        bool identical = true;
        for (NodeId id : i1.customers) {
            identical = identical && i1.nodes.at(id).coordinates.x == i2.nodes.at(id).coordinates.x &&
                        i1.nodes.at(id).coordinates.y == i2.nodes.at(id).coordinates.y;
        }
        check(identical, "same config -> bit-identical coordinates");

        // Pinned from the Python reference: seed 20261109, dataset 0.
        check_near(i1.nodes.at(1).coordinates.x, 28073.594620679673, 1e-6, "customer 1 x (reference)");
        check_near(i1.nodes.at(1).coordinates.y, 16573.68124145301, 1e-6, "customer 1 y (reference)");
        check_near(i1.nodes.at(2).coordinates.x, 40198.4578268745, 1e-6, "customer 2 x (reference)");
        check_near(i1.nodes.at(2).coordinates.y, 7508.471312076066, 1e-6, "customer 2 y (reference)");

        tdr::config::StaticConfig c1 = c;
        c1.instance.dataset_index = 1;
        const auto r3 = tdr::instance::generate_static_instance(c1);
        check_near(r3.instance.nodes.at(1).coordinates.x, 47826.2730454663, 1e-6, "dataset 1 customer 1 x (reference)");
        check_near(r3.instance.nodes.at(1).coordinates.y, 38375.493186327534, 1e-6, "dataset 1 customer 1 y (reference)");
        check(r3.instance.instance_seed != i1.instance_seed, "different dataset index -> different instance seed");

        // The customer-count prefix property: the first 10 customers of a
        // 20-customer instance equal the 10-customer instance (draw order is
        // fixed), so growing n never reshuffles earlier customers.
        tdr::config::StaticConfig c20 = c;
        c20.instance.num_customers = 20;
        const auto r20 = tdr::instance::generate_static_instance(c20);
        bool prefix = true;
        for (NodeId id : i1.customers) prefix = prefix && r20.instance.nodes.at(id).coordinates.x == i1.nodes.at(id).coordinates.x;
        check(prefix, "first 10 customers of a 20-customer instance equal the 10-customer instance");

        // Range and parameters.
        bool in_square = true;
        for (const auto& kv : r20.instance.nodes.all()) {
            const auto& p = kv.second.coordinates;
            in_square = in_square && p.x >= 0.0 && p.x <= 60000.0 && p.y >= 0.0 && p.y <= 60000.0;
        }
        check(in_square, "all coordinates lie in the 60 km square");
        check(i1.nodes.at(0).coordinates.x == 30000.0 && i1.nodes.at(0).coordinates.y == 30000.0, "depot at the centre (30000, 30000)");
        check(i1.truck.speed == 15.0 && i1.drone.speed == 12.0 && i1.drone.endurance_seconds == 1800.0 &&
                  i1.drone.launch_prep_time_sl == 60.0 && i1.drone.recovery_time_sr == 40.0 && i1.drone.num_drones == 2,
              "vehicle parameters copied from the config");
        const auto& cd = i1.customer_data.at(4);
        check(cd.node_id == 4 && cd.truck_service_time == 30.0 && cd.drone_service_time == 30.0 && cd.generalized_flight_time_theta == 1.0,
              "CustomerData: Se1 = Se2 = 30 s, theta = 1");

        tdr::config::StaticConfig cc = c;
        cc.instance.depot_placement = tdr::config::DepotPlacement::Corner;
        check(tdr::instance::generate_static_instance(cc).instance.nodes.at(0).coordinates.x == 0.0, "corner depot at the origin");
        tdr::config::StaticConfig ce = c;
        ce.instance.depot_placement = tdr::config::DepotPlacement::Explicit;
        ce.instance.depot_x_m = 1234.0;
        ce.instance.depot_y_m = 4321.0;
        const auto re = tdr::instance::generate_static_instance(ce).instance;
        check(re.nodes.at(0).coordinates.x == 1234.0 && re.nodes.at(0).coordinates.y == 4321.0, "explicit depot honoured");
        check(re.nodes.at(1).coordinates.x == i1.nodes.at(1).coordinates.x, "depot placement does not change the customer draws");

        tdr::config::StaticConfig badc = c;
        badc.instance.num_customers = -3;
        const auto rb = tdr::instance::generate_static_instance(badc);
        check(!rb.ok && !rb.errors.empty(), "invalid config -> ok = false with a message, no throw");

        const auto net = i1.make_network();
        check(net.nodes().size() == 11, "make_network() wraps depot + customers");
    }

    // =====================================================================
    // 4. FlpArcIndex.
    //
    // Nodes: 0=(0,0), 1=(10,0), 2=(0,25); d = 5 (eq. (41) n = floor(D/d)).
    //   |01| = 10 -> n = 2, ratios 1/3, 2/3; |02| = 25 -> n = 5;
    //   |12| = sqrt(100+625) = 26.93 -> floor(5.39) = 5; |10| = 10 -> 2.
    // Arc order (0,1),(0,2),(1,2),(1,0); ids start at 3:
    //   (0,1): 3,4 | (0,2): 5..9 | (1,2): 10..14 | (1,0): 15,16.
    // =====================================================================
    {
        NodeRegistry nodes;
        nodes.add(Node{0, NodeType::Depot, Point2D{0, 0}});
        nodes.add(Node{1, NodeType::Customer, Point2D{10, 0}});
        nodes.add(Node{2, NodeType::Customer, Point2D{0, 25}});
        tdr::flp::FlpGenerationPolicy pol;
        pol.unit_division_length = 5.0;
        const auto gen = tdr::flp::generate_flexible_launch_points_on_arcs({{0, 1}, {0, 2}, {1, 2}, {1, 0}}, nodes, pol, 3);
        std::unordered_map<NodeId, FlexibleLaunchPointData> data;
        for (const auto& p : gen.points) data[p.node.id] = p.data;
        const tdr::flp::FlpArcIndex idx(data);
        check(idx.flp_count() == 14, "index holds 2+5+5+2 = 14 FLPs");
        check(idx.nonempty_arc_count() == 4, "4 non-empty arcs");
        check(idx.flps_on_arc(0, 1) == std::vector<NodeId>({3, 4}), "L(0,1) = {3,4} ordered by ratio");
        check(idx.flps_on_arc(0, 2) == std::vector<NodeId>({5, 6, 7, 8, 9}), "L(0,2) = {5..9}");
        check(idx.flps_on_arc(1, 0) == std::vector<NodeId>({15, 16}), "L(1,0) = {15,16}: directed arcs are distinct from L(0,1)");
        check(idx.flps_on_arc(2, 0).empty(), "arc (2,0) was never generated -> empty");
        check(idx.flps_on_arc(0, 1) != idx.flps_on_arc(1, 0), "L(0,1) != L(1,0)");
        check(idx.arc_of(7) && idx.arc_of(7)->first == 0 && idx.arc_of(7)->second == 2, "arc_of(7) = (0,2)");
        check(!idx.arc_of(99).has_value() && !idx.contains(99) && idx.contains(16), "unknown FLP has no arc; known FLP is contained");

        // The order must not depend on unordered_map iteration order: rebuild
        // from a map filled in reverse.
        std::unordered_map<NodeId, FlexibleLaunchPointData> rev;
        for (auto it = gen.points.rbegin(); it != gen.points.rend(); ++it) rev[it->node.id] = it->data;
        const tdr::flp::FlpArcIndex idx2(rev);
        check(idx2.flps_on_arc(0, 2) == idx.flps_on_arc(0, 2) && idx2.flps_on_arc(1, 2) == idx.flps_on_arc(1, 2),
              "per-arc order is independent of insertion order");
    }
    {
        // Against the real eager pool of a generated instance.
        tdr::config::StaticConfig c;
        c.instance.num_customers = 6;
        const auto gi = tdr::instance::generate_static_instance(c);
        const auto net = gi.instance.make_network();
        tdr::init::InitialSolutionPolicy pol;
        pol.flp_policy.unit_division_length = c.flp.unit_division_length_m;
        const auto res = tdr::init::build_initial_solution(gi.instance.depot, gi.instance.customers, gi.instance.customer_data, net, pol);
        const tdr::flp::FlpArcIndex idx(res.flp_data);
        check(idx.flp_count() == res.candidate_flps.size(), "index size equals the eager pool size");
        std::vector<NodeId> truck_nodes{0};
        for (NodeId id : gi.instance.customers) truck_nodes.push_back(id);
        bool counts_ok = true, order_ok = true;
        std::size_t total = 0;
        for (NodeId a : truck_nodes) {
            for (NodeId b : truck_nodes) {
                if (a == b) continue;
                const double dist = net.distance(a, b);
                const int expected = tdr::flp::flp_count_for_arc(dist, pol.flp_policy);
                const auto& got = idx.flps_on_arc(a, b);
                counts_ok = counts_ok && static_cast<int>(got.size()) == expected;
                total += got.size();
                for (std::size_t k = 1; k < got.size(); ++k) {
                    order_ok = order_ok && res.flp_data.at(got[k - 1]).position_ratio < res.flp_data.at(got[k]).position_ratio;
                }
            }
        }
        check(counts_ok, "per-arc counts equal floor(D/d) for every ordered pair of truck nodes");
        check(order_ok, "FLPs on every arc are strictly ordered by ratio");
        check(total == res.candidate_flps.size(), "every pool FLP is on exactly one indexed arc");
    }

    // =====================================================================
    // 5. assign_drone_ids_by_route_order after the move.
    // Route (positions): 0:D0, 1:F10, 2:C1, 3:F11, 4:C2, 5:F12, 6:C3, 7:D0.
    // =====================================================================
    {
        Solution s;
        s.truck_route = {0, 10, 1, 11, 2, 12, 3, 0};

        // (a) disjoint: A launches F10 (pos1) recovers C1 (pos2); B launches
        //     F11 (pos3) recovers C2 (pos4). B's launch 3 > A's recovery 2.
        s.drone_trips = {trip(-1, 10, 21, 1), trip(-1, 11, 22, 2)};
        check(assign_drone_ids_by_route_order(s, 2) && s.drone_trips[0].drone_id == 0 && s.drone_trips[1].drone_id == 0,
              "disjoint trips share drone 0");
        // (b) overlapping: A: F10(1)..C2(4); B: F11(3)..C3(6): 3 <= 4 -> need 2 drones.
        s.drone_trips = {trip(-1, 10, 21, 2), trip(-1, 11, 22, 3)};
        check(assign_drone_ids_by_route_order(s, 2) && s.drone_trips[0].drone_id == 0 && s.drone_trips[1].drone_id == 1,
              "overlapping trips take drones 0 and 1");
        check(!assign_drone_ids_by_route_order(s, 1), "overlapping trips with |U| = 1 -> false");
        check(!assign_drone_ids_by_route_order(s, 0), "|U| = 0 with placed trips -> false");
        // (c) processing is by launch position, not vector order.
        s.drone_trips = {trip(-1, 11, 22, 3), trip(-1, 10, 21, 2)};
        assign_drone_ids_by_route_order(s, 2);
        check(s.drone_trips[1].drone_id == 0 && s.drone_trips[0].drone_id == 1, "earlier launch gets drone 0 regardless of vector order");
        // (d) unlocatable launch -> drone 0, ignored for the others.
        s.drone_trips = {trip(-1, 99, 21, 1), trip(-1, 10, 22, 2)};
        check(assign_drone_ids_by_route_order(s, 1) && s.drone_trips[0].drone_id == 0 && s.drone_trips[1].drone_id == 0,
              "trip with a launch node not on the route gets drone 0 and does not block the others");
        // (e) recovery before launch -> unlocatable -> drone 0.
        s.drone_trips = {trip(-1, 12, 21, 1)};
        check(assign_drone_ids_by_route_order(s, 1) && s.drone_trips[0].drone_id == 0, "recovery before launch -> drone 0");
        // (f) the old name still resolves to the same function.
        s.drone_trips = {trip(-1, 10, 21, 2), trip(-1, 11, 22, 3)};
        check(tdr::init::detail::assign_drone_ids_by_route_order(s, 2) && s.drone_trips[1].drone_id == 1,
              "tdr::init::detail::assign_drone_ids_by_route_order forwards to the shared implementation");
        // (g) a drone is reusable after recovery: A F10(1)..C1(2), B F11(3)..C2(4), C F12(5)..C3(6) on one drone.
        s.drone_trips = {trip(-1, 10, 21, 1), trip(-1, 11, 22, 2), trip(-1, 12, 23, 3)};
        check(assign_drone_ids_by_route_order(s, 1) && s.drone_trips[2].drone_id == 0, "three sequential trips on a single drone");
    }
    {
        // Regression pin: 20,000 random cases, checksum computed with the
        // PRE-MOVE implementation (tdr::init::detail, baseline headers at
        // commit cc37e49) and the same generator. Any behavioural change of
        // the function changes this value.
        tdr::util::Rng g(424242);
        std::uint64_t h = 0xcbf29ce484222325ULL;
        long falses = 0;
        for (int it = 0; it < 20000; ++it) {
            Solution s;
            const int ncust = static_cast<int>(g.uniform_int(2, 12));
            const int nflp = static_cast<int>(g.uniform_int(0, 10));
            std::vector<NodeId> mid;
            for (int i = 1; i <= ncust; ++i) mid.push_back(i);
            for (int f = 0; f < nflp; ++f) mid.push_back(100 + f);
            g.shuffle(mid);
            s.truck_route.push_back(0);
            for (NodeId v : mid) s.truck_route.push_back(v);
            s.truck_route.push_back(0);
            const int ntrips = static_cast<int>(g.uniform_int(0, 9));
            for (int t = 0; t < ntrips; ++t) {
                DroneTrip d;
                d.launch_node = nflp ? 100 + static_cast<int>(g.uniform_int(0, nflp - 1)) : static_cast<int>(g.uniform_int(1, ncust));
                d.service_customer = static_cast<int>(g.uniform_int(1, ncust));
                d.recovery_node = (g.uniform_int(0, 9) == 0) ? 0 : static_cast<int>(g.uniform_int(1, ncust));
                if (g.uniform_int(0, 19) == 0) d.launch_node = 999;
                d.drone_id = static_cast<int>(g.uniform_int(-1, 3));
                s.drone_trips.push_back(d);
            }
            const int U = static_cast<int>(g.uniform_int(0, 4));
            const bool ok = assign_drone_ids_by_route_order(s, U);
            h = fnv(h, ok ? 1 : 0);
            h = fnv(h, static_cast<std::uint64_t>(U));
            for (const auto& d : s.drone_trips) h = fnv(h, static_cast<std::uint64_t>(d.drone_id + 7));
            if (!ok) ++falses;
        }
        check(h == 0xd32e63262ac4e064ULL, "20,000-case behavioural checksum equals the pre-move value 0xd32e63262ac4e064");
        check(falses == 7722, "the pinned run contains 7722 infeasible (false) cases, so both outcomes are exercised");
    }

    // =====================================================================
    // 6. Route operations.
    // Registry: depot 0; customers 1..5; FLPs 10,11,12,13,14.
    // Route R = [0, F10, 1, F11, F12, 2, 3, 0]
    //   positions: 0:D0 1:F10 2:C1 3:F11 4:F12 5:C2 6:C3 7:D0
    //   skeleton (non-FLP): s=0:D0@0, s=1:C1@2, s=2:C2@5, s=3:C3@6, s=4:D0@7
    //   gaps 0..3; FLP positions 1 -> gap 0; 3,4 -> gap 1.
    // Ownership: F10:(0,1) F11:(1,2) F12:(1,2) F13:(3,0); F14 has none.
    // =====================================================================
    NodeRegistry nodes;
    nodes.add(Node{0, NodeType::Depot, Point2D{0, 0}});
    for (int i = 1; i <= 5; ++i) nodes.add(Node{i, NodeType::Customer, Point2D{static_cast<double>(i), 0}});
    for (int f = 10; f <= 14; ++f) nodes.add(Node{f, NodeType::FlexibleLaunchPoint, Point2D{0.5 * f, 1}});
    std::unordered_map<NodeId, FlexibleLaunchPointData> fd;
    fd[10] = FlexibleLaunchPointData{10, 0, 1, 0.5};
    fd[11] = FlexibleLaunchPointData{11, 1, 2, 0.3};
    fd[12] = FlexibleLaunchPointData{12, 1, 2, 0.6};
    fd[13] = FlexibleLaunchPointData{13, 3, 0, 0.5};

    {
        const TruckRoute R{0, 10, 1, 11, 12, 2, 3, 0};
        const Skeleton sk = make_skeleton(R, nodes);
        check(sk.node == std::vector<NodeId>({0, 1, 2, 3, 0}), "skeleton nodes = 0,1,2,3,0");
        check(sk.route_position == std::vector<std::size_t>({0, 2, 5, 6, 7}), "skeleton raw positions = 0,2,5,6,7");
        check(sk.gap_count() == 4, "4 gaps");
        check(gap_of_route_position(sk, 1) == std::optional<std::size_t>(0), "F10 (pos 1) is in gap 0");
        check(gap_of_route_position(sk, 3) == std::optional<std::size_t>(1) && gap_of_route_position(sk, 4) == std::optional<std::size_t>(1),
              "F11/F12 (pos 3,4) are in gap 1");
        check(!gap_of_route_position(sk, 2).has_value() && !gap_of_route_position(sk, 7).has_value(), "skeleton positions are in no gap");
        check(flps_in_gap(R, sk, 0) == std::vector<NodeId>({10}), "flps_in_gap(0) = {10}");
        check(flps_in_gap(R, sk, 1) == std::vector<NodeId>({11, 12}), "flps_in_gap(1) = {11,12}");
        check(flps_in_gap(R, sk, 2).empty() && flps_in_gap(R, sk, 9).empty(), "empty / out-of-range gap -> no FLPs");
        check(insertion_position_for_gap(sk, 1) == std::optional<std::size_t>(5), "insertion into gap 1 lands before C2 at raw position 5");
        check(insertion_position_for_gap(sk, 3) == std::optional<std::size_t>(7), "insertion into the last gap lands before the end depot (7)");
        check(!insertion_position_for_gap(sk, 4).has_value(), "gap 4 does not exist");

        check(flps_off_their_arc(R, nodes, fd).empty(), "a consistent route has no foreign FLPs");

        // Removing customer 2 (pos 5): F11,F12 now lie between 1 and 3.
        TruckRoute R2 = R;
        check(remove_from_route(R2, 2) == std::optional<std::size_t>(5), "remove_from_route(2) returns its position 5");
        check(R2 == TruckRoute({0, 10, 1, 11, 12, 3, 0}), "route after removal");
        const auto off = flps_off_their_arc(R2, nodes, fd);
        check(off.size() == 2 && off[0].flp == 11 && off[1].flp == 12, "removing C2 makes F11 and F12 foreign");
        check(off.size() == 2 && off[0].actual_from == 1 && off[0].actual_to == 3 && off[0].has_owner &&
                  off[0].owner_from == 1 && off[0].owner_to == 2,
              "foreign F11: route puts it on (1,3) but it belongs to (1,2)");
        check(off.size() == 2 && off[0].route_position == 3 && off[1].route_position == 4, "foreign FLP route positions 3 and 4");
        check(!remove_from_route(R2, 2).has_value(), "removing an absent node -> nullopt, route unchanged");

        // FLP of a foreign arc, an FLP without ownership data, a trailing FLP.
        TruckRoute R3{0, 13, 1, 2, 0};
        const auto off3 = flps_off_their_arc(R3, nodes, fd);
        check(off3.size() == 1 && off3[0].flp == 13 && off3[0].actual_from == 0 && off3[0].actual_to == 1, "F13 (owner (3,0)) placed on arc (0,1) is foreign");
        TruckRoute R4{0, 14, 1, 0};
        const auto off4 = flps_off_their_arc(R4, nodes, fd);
        check(off4.size() == 1 && off4[0].flp == 14 && !off4[0].has_owner, "FLP with no ownership data is reported with has_owner = false");
        TruckRoute R5{0, 1, 10};
        const auto off5 = flps_off_their_arc(R5, nodes, fd);
        check(off5.size() == 1 && off5[0].actual_to == kInvalidNodeId, "trailing FLP (no next truck node) is reported");

        // Trips.
        Solution s;
        s.truck_route = R;
        s.drone_trips = {trip(0, 10, 4, 1), trip(1, 11, 5, 3)};
        check(trips_launching_from(s, 10) == std::vector<std::size_t>({0}) && trips_launching_from(s, 11) == std::vector<std::size_t>({1}) &&
                  trips_launching_from(s, 12).empty(),
              "trips_launching_from");
        check(trips_serving(s, 4) == std::vector<std::size_t>({0}) && trips_serving(s, 1).empty(), "trips_serving");
        check(trips_recovering_at(s, 1) == std::vector<std::size_t>({0}) && trips_recovering_at(s, 3) == std::vector<std::size_t>({1}),
              "trips_recovering_at");
        const ServiceMap sm = classify_service(s, nodes);
        check(sm.truck_customers == std::vector<NodeId>({1, 2, 3}), "truck customers in route order = 1,2,3");
        check(sm.drone_trip_of.size() == 2 && sm.drone_trip_of.at(4) == 0 && sm.drone_trip_of.at(5) == 1, "drone customers 4->trip 0, 5->trip 1");

        // Unused FLPs: F12 has no trip.
        check(unused_flps_in_route(s, nodes) == std::vector<NodeId>({12}), "F12 is the only unused FLP");
        Solution s2 = s;
        const auto dropped = drop_unused_flps(s2, nodes);
        check(dropped == std::vector<NodeId>({12}), "drop_unused_flps returns the removed FLPs");
        check(s2.truck_route == TruckRoute({0, 10, 1, 11, 2, 3, 0}), "only F12 left the route; used FLPs and customers untouched");
        check(s2.drone_trips.size() == 2, "drop_unused_flps never touches trips");
        check(drop_unused_flps(s2, nodes).empty(), "drop_unused_flps is idempotent");

        erase_trip(s2, 0);
        check(s2.drone_trips.size() == 1 && s2.drone_trips[0].service_customer == 5, "erase_trip removes exactly that trip and keeps the rest in order");
        check(unused_flps_in_route(s2, nodes) == std::vector<NodeId>({10}), "after erasing the trip, F10 is reported unused");
    }
    {
        // lowest_free_drone_for_trip on [0,F10,1,F11,2,3,0]:
        //   pos 0:D0 1:F10 2:C1 3:F11 4:C2 5:C3 6:D0.
        Solution s;
        s.truck_route = {0, 10, 1, 11, 2, 3, 0};
        // Overlapping: t0 F10(1)..C2(4), t1 F11(3)..C3(5).
        s.drone_trips = {trip(0, 10, 4, 2), trip(0, 11, 5, 3)};
        check(lowest_free_drone_for_trip(s, 1, 2) == std::optional<int>(1), "t1 overlaps t0 on drone 0 -> lowest free is drone 1");
        check(lowest_free_drone_for_trip(s, 0, 2) == std::optional<int>(1), "symmetrically for t0");
        check(!lowest_free_drone_for_trip(s, 1, 1).has_value(), "single drone, overlapping -> none");
        s.drone_trips[0].drone_id = 1;
        check(lowest_free_drone_for_trip(s, 1, 2) == std::optional<int>(0), "if t0 flies drone 1, t1 may use drone 0");
        check(s.drone_trips[0].drone_id == 1, "lowest_free_drone_for_trip does not modify other trips (identity preserved)");
        // Disjoint: t0 F10(1)..C1(2); t1 F11(3)..C3(5).
        s.drone_trips = {trip(0, 10, 4, 1), trip(0, 11, 5, 3)};
        check(lowest_free_drone_for_trip(s, 1, 1) == std::optional<int>(0), "disjoint trips may share the only drone");
        s.drone_trips.push_back(trip(0, 99, 5, 3));
        check(!lowest_free_drone_for_trip(s, 2, 2).has_value(), "trip whose launch is not on the route -> nullopt");
        s.drone_trips.back() = trip(-1, 11, 5, 3);
        check(lowest_free_drone_for_trip(s, 2, 2) == std::optional<int>(1), "other trips with invalid ids are ignored; duplicate trip sees t1 on drone 0 -> drone 1");
    }

    // =====================================================================
    // 7. Candidate window. Route [0,F10,1,F11,2,3,0]; trips: customer 4 from
    // F10 (gap 0), customer 5 from F11 (gap 1).
    //   skeleton: s=0:D0, s=1:C1, s=2:C2, s=3:C3, s=4:D0
    //   half-steps: C1=2, C2=4, C3=6, drone C4 = 2*0+1 = 1, drone C5 = 2*1+1 = 3.
    //   gap g has coordinate 2g+1: gaps 0..3 -> 1,3,5,7.
    // =====================================================================
    {
        Solution s;
        s.truck_route = {0, 10, 1, 11, 2, 3, 0};
        s.drone_trips = {trip(0, 10, 4, 1), trip(1, 11, 5, 3)};
        const PositionIndex pi(s, nodes);
        check(pi.half_step(1) == std::optional<int>(2) && pi.half_step(2) == std::optional<int>(4) && pi.half_step(3) == std::optional<int>(6),
              "truck customers: h = 2*skeleton index");
        check(pi.half_step(4) == std::optional<int>(1) && pi.half_step(5) == std::optional<int>(3), "drone customers: h = 2*gap+1");
        check(!pi.half_step(99).has_value() && !pi.half_step(0).has_value(), "unknown node and the depot have no half-step");
        check(pi.adjacent(1, 2, 1) && !pi.adjacent(1, 3, 1) && pi.adjacent(1, 3, 2), "truck adjacency: |2-4|=2<=2 yes; |2-6|=4>2 no; with w=2 yes");
        check(!pi.adjacent(4, 3, 1) && pi.adjacent(4, 3, 3), "|1-6| = 5 > 2 (w=1) but <= 6 (w=3)");
        check(pi.neighbours(2, 1) == std::vector<NodeId>({5, 1, 3}), "neighbours(C2, w=1): d=1 C5, d=2 C1, d=2 C3 (C4 at d=3 excluded)");
        check(pi.neighbours(1, 1) == std::vector<NodeId>({4, 5, 2}), "neighbours(C1, w=1) = C4(d=1), C5(d=1), C2(d=2)");
        check(pi.neighbours(2, 3).size() == 4, "w=3 covers all other 4 customers");
        check(pi.neighbours(99, 3).empty(), "unknown customer has no neighbours");
        check(pi.gaps_within(4, 1) == std::vector<std::size_t>({1, 2}), "gaps within w=1 of h=4: gap 1 (3) and gap 2 (5)");
        check(pi.gaps_within(4, 3) == std::vector<std::size_t>({0, 1, 2, 3}), "w=3: every gap");
        check(pi.skeleton().size() == 5, "PositionIndex exposes its skeleton");

        // A drone trip whose launch FLP is not on the route is simply unplaced.
        Solution bad = s;
        bad.drone_trips.push_back(trip(0, 99, 5, 3));
        const PositionIndex pb(bad, nodes);
        check(pb.half_step(5) == std::optional<int>(3), "an unplaceable extra trip does not displace an already placed customer");
        Solution bad2;
        bad2.truck_route = {0, 1, 0};
        bad2.drone_trips = {trip(0, 99, 4, 1)};
        check(!PositionIndex(bad2, nodes).half_step(4).has_value(), "drone customer with a missing launch FLP has no position");
    }

    std::cout << (g_failures == 0 ? "\nALL TESTS PASSED\n" : "\nSOME TESTS FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
