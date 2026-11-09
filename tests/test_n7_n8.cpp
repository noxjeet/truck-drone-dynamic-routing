// M3 tests: neighbourhoods N7 (truck node delete / drone node insert) and N8 (drone node
// delete / truck node insert). HAND-DERIVED fixtures on a line, so every expected number
// below was worked out on paper and is not read back from the code under test:
//
//   * depot 0 at x = 0; customer Ci at x = 1000 i metres; truck 10 m/s, drone 20 m/s
//     (10 m/s in the tie fixtures), SL = SR = 0, service times 0, theta = 0, d = 600 m
//     => floor(D/600) FLPs per arc (1000 m: 1 FLP at the midpoint; 2000 m: 3 at 1/4, 2/4,
//     3/4; 5000 m: 8 at k/9).
//   * FLP pool = every ordered pair of truck nodes (the eager pool).
//   * truck arrival at x metres along the line, no waiting: x / 10 s; a truck that reaches
//     a recovery node before its drone waits for it (SR = 0).
//
// Independent cross-checks use the FROZEN reference evaluator (tdr::timing_ref), not the
// production one. M1/M2 behaviour is covered by test_n4_n5.cpp and is not repeated here.

#include "flp/flexible_launch_point_generator.hpp"
#include "flp/flp_arc_index.hpp"
#include "graph/spatial_network.hpp"
#include "model/customer.hpp"
#include "model/drone_trip.hpp"
#include "model/node.hpp"
#include "model/parameters.hpp"
#include "model/solution.hpp"
#include "reference/timing_engine_reference.hpp"
#include "search/candidate_window.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/neighbourhood_n5.hpp"
#include "search/neighbourhood_n7.hpp"
#include "search/neighbourhood_n8.hpp"
#include "search/route_ops.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

using namespace tdr::model;
using namespace tdr::search;

namespace {

int g_failures = 0;
int g_passes = 0;

void check(bool condition, const std::string& description) {
    if (condition) {
        ++g_passes;
        std::cout << "[PASS] " << description << "\n";
    } else {
        ++g_failures;
        std::cout << "[FAIL] " << description << "\n";
    }
}

void check_near(double a, double b, double tol, const std::string& description) {
    check(std::fabs(a - b) <= tol, description + " (got " + std::to_string(a) + ", expected " + std::to_string(b) + ")");
}

struct Fix {
    NodeRegistry nodes;
    std::unordered_map<NodeId, CustomerData> cd;
    tdr::timing::FlpDataMap flp;
    std::optional<tdr::graph::SpatialNetwork> net;
    tdr::flp::FlpArcIndex idx;
    int n = 0;

    SearchContext ctx(int w) const { return SearchContext{*net, cd, flp, idx, w}; }
    NodeId flp_at(NodeId a, NodeId b, std::size_t k) const { return idx.flps_on_arc(a, b).at(k); }
    double flp_x(NodeId f) const { return nodes.at(f).coordinates.x; }
};

std::unique_ptr<Fix> make_line(int n, int drones, double endurance = 3600.0, double d = 600.0, double spacing = 1000.0, double drone_speed = 20.0) {
    auto f = std::make_unique<Fix>();
    f->n = n;
    f->nodes.add(Node{0, NodeType::Depot, Point2D{0, 0}});
    for (int i = 1; i <= n; ++i) f->nodes.add(Node{i, NodeType::Customer, Point2D{spacing * i, 0}});
    std::vector<tdr::flp::Arc> arcs;
    for (int a = 0; a <= n; ++a)
        for (int b = 0; b <= n; ++b)
            if (a != b) arcs.emplace_back(a, b);
    tdr::flp::FlpGenerationPolicy pol;
    pol.unit_division_length = d;
    const auto gen = tdr::flp::generate_flexible_launch_points_on_arcs(arcs, f->nodes, pol, 100);
    for (const auto& p : gen.points) {
        f->nodes.add(p.node);
        f->flp[p.node.id] = p.data;
    }
    for (int i = 1; i <= n; ++i) {
        CustomerData c;
        c.node_id = i;
        c.truck_service_time = 0;
        c.drone_service_time = 0;
        c.generalized_flight_time_theta = 0;
        f->cd[i] = c;
    }
    TruckParameters tp;
    tp.speed = 10;
    DroneParameters dp;
    dp.speed = drone_speed;
    dp.endurance_seconds = endurance;
    dp.launch_prep_time_sl = 0;
    dp.recovery_time_sr = 0;
    dp.num_drones = drones;
    f->net.emplace(f->nodes, tp, dp);
    f->idx = tdr::flp::FlpArcIndex(f->flp);
    return f;
}

DroneTrip trip(int drone, NodeId launch, NodeId customer, NodeId recovery) {
    DroneTrip t;
    t.drone_id = drone;
    t.launch_node = launch;
    t.service_customer = customer;
    t.recovery_node = recovery;
    return t;
}

bool same_solution(const Solution& a, const Solution& b) {
    if (a.truck_route != b.truck_route || a.drone_trips.size() != b.drone_trips.size()) return false;
    for (std::size_t i = 0; i < a.drone_trips.size(); ++i) {
        const auto &x = a.drone_trips[i], &y = b.drone_trips[i];
        if (x.drone_id != y.drone_id || x.launch_node != y.launch_node || x.service_customer != y.service_customer ||
            x.recovery_node != y.recovery_node)
            return false;
    }
    return true;
}

// Each customer 1..n appears exactly once, either on the truck route or as a trip's service customer.
bool exactly_once(const Solution& s, int n) {
    std::vector<int> count(static_cast<std::size_t>(n) + 1, 0);
    for (NodeId v : s.truck_route) if (v >= 1 && v <= n) ++count[static_cast<std::size_t>(v)];
    for (const auto& t : s.drone_trips) if (t.service_customer >= 1 && t.service_customer <= n) ++count[static_cast<std::size_t>(t.service_customer)];
    for (int i = 1; i <= n; ++i) if (count[static_cast<std::size_t>(i)] != 1) return false;
    return true;
}

// INDEPENDENT re-implementation of "some FLP entry is not between the endpoints of its own arc".
bool has_foreign_flp(const TruckRoute& r, const Fix& f) {
    NodeId last_real = kInvalidNodeId;
    std::vector<NodeId> pending;
    for (NodeId v : r) {
        if (f.nodes.at(v).type == NodeType::FlexibleLaunchPoint) {
            pending.push_back(v);
        } else {
            for (NodeId p : pending) {
                const auto& d = f.flp.at(p);
                if (d.arc_from != last_real || d.arc_to != v) return true;
            }
            pending.clear();
            last_real = v;
        }
    }
    return !pending.empty();
}

double ref_objective(const Solution& s, const Fix& f, bool* feasible) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *f.net, f.cd, f.flp);
    if (feasible) *feasible = e.feasible;
    return e.objective_value;
}

} // namespace

namespace {

// Same eager-pool fixture as make_line but with customers at arbitrary points (customer i is node i, 1-based).
std::unique_ptr<Fix> make_points(const std::vector<Point2D>& pts, int drones, double endurance = 3600.0, double d = 600.0) {
    auto f = std::make_unique<Fix>();
    const int n = static_cast<int>(pts.size());
    f->n = n;
    f->nodes.add(Node{0, NodeType::Depot, Point2D{0, 0}});
    for (int i = 1; i <= n; ++i) f->nodes.add(Node{i, NodeType::Customer, pts[static_cast<std::size_t>(i - 1)]});
    std::vector<tdr::flp::Arc> arcs;
    for (int a = 0; a <= n; ++a)
        for (int b = 0; b <= n; ++b)
            if (a != b) arcs.emplace_back(a, b);
    tdr::flp::FlpGenerationPolicy pol;
    pol.unit_division_length = d;
    const auto gen = tdr::flp::generate_flexible_launch_points_on_arcs(arcs, f->nodes, pol, 100);
    for (const auto& p : gen.points) {
        f->nodes.add(p.node);
        f->flp[p.node.id] = p.data;
    }
    for (int i = 1; i <= n; ++i) {
        CustomerData c;
        c.node_id = i;
        c.truck_service_time = 0;
        c.drone_service_time = 0;
        c.generalized_flight_time_theta = 0;
        f->cd[i] = c;
    }
    TruckParameters tp;
    tp.speed = 10;
    DroneParameters dp;
    dp.speed = 20;
    dp.endurance_seconds = endurance;
    dp.launch_prep_time_sl = 0;
    dp.recovery_time_sr = 0;
    dp.num_drones = drones;
    f->net.emplace(f->nodes, tp, dp);
    f->idx = tdr::flp::FlpArcIndex(f->flp);
    return f;
}

std::size_t pos_of(const TruckRoute& r, NodeId v) {
    for (std::size_t i = 0; i < r.size(); ++i) if (r[i] == v) return i;
    return r.size();
}

// Every record of an N7/N8 result: exactly-once coverage, no FLP off its arc, the production objective equals the
// FROZEN reference evaluator's, and feasibility agrees (a rejected-by-evaluator record must be infeasible there).
void check_records_against_reference(const OperatorResult& res, const Fix& f, const std::string& tag) {
    bool cover = true, arcs = true, obj = true, verdict = true;
    for (const auto& rec : res.log) {
        if (rec.solution.truck_route.empty()) continue;
        // an N7 candidate rejected before the new trip exists shows the route WITHOUT the deleted customer (it is not a complete solution)
        const bool partial = rec.move.kind == MoveKind::N7TruckToDrone && (rec.reason == RejectReason::AffectedTrips || rec.reason == RejectReason::NoReplacementFlp);
        if (!partial) cover = cover && exactly_once(rec.solution, f.n);
        arcs = arcs && !has_foreign_flp(rec.solution.truck_route, f) == (rec.reason != RejectReason::AffectedTrips);
        if (rec.evaluated) {
            bool feas = false;
            const double o = ref_objective(rec.solution, f, &feas);
            obj = obj && std::fabs(o - rec.objective) <= 1e-9 * std::max(1.0, std::fabs(o));
            verdict = verdict && (feas == (rec.reason == RejectReason::None));
        }
    }
    check(cover, tag + ": exactly-once coverage in every record");
    check(arcs, tag + ": FLP off its arc <=> AffectedTrips (independent check)");
    check(obj, tag + ": production objective == frozen reference objective in every evaluated record");
    check(verdict, tag + ": production verdict == frozen reference verdict in every evaluated record");
    check(res.accepted + res.rejected_total() == res.generated, tag + ": accepted + rejected == generated (one outcome per candidate)");
    check(res.duplicates_skipped == 0, tag + ": no duplicates are ever produced");
    check(res.log.size() == res.generated, tag + ": one record per generated candidate");
}

} // namespace

int main() {
    // =====================================================================
    // N7-1. All-truck line, n = 5, |U| = 2, route [0,1,2,3,4,5,0], base objective 100+...+500 = 1500.
    //       Anchor C3. Resulting skeleton [0,1,2,4,5,0]; arcs by gap:
    //         g0 (0,1) 1 FLP @500 | g1 (1,2) 1 FLP @1500 | g2 (2,4) 3 FLPs @2500,3000,3500 (the merged arc)
    //         g3 (4,5) 1 FLP @4500 | g4 (5,0) 8 FLPs (x = 5000 - 5000k/9)
    //       w = 3: every gap is eligible (original half-steps 1,3,6,9,11 vs 2 s_t = 6: all within 6)  => 14 candidates.
    //       Trip (f, C3, y) with y = arc end (the next truck node).
    //       Hand-derived objectives (launch at x_f/10 s, drone 20 m/s, SL = SR = 0, truck waits at y for the drone):
    //         g2 f=2500: C3 at 250+25 = 275, y=C4 at 325 < truck 400:           100+200+400+500+275 = 1475
    //         g2 f=3000: C3 at 300, y at 350 < 400:                              100+200+400+500+300 = 1500
    //         g2 f=3500: C3 at 350+25 = 375, y at 425 > 400, truck waits 25:     100+200+400+(500+25)+375 = 1600
    //         g1 f=1500: C3 at 150+75 = 225, y=C2 at 275 > 200, truck waits 75:  100+200+(400+75)+(500+75)+225 = 1575
    //         g3 f=4500: C3 at 450+75 = 525, y=C5 at 625 > 500 (no later node):  100+200+400+500+525 = 1725
    //         g0 f=500 : C3 at 50+125 = 175, y=C1 at 275 > 100, truck waits 175: 100+(200+175)+(400+175)+(500+175)+175 = 1900
    // =====================================================================
    {
        auto f = make_line(5, 2);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 0};
        const Solution before = base;
        N7Options o;
        o.record_candidates = true;
        const auto res = n7_truck_node_delete_drone_insert(base, f->ctx(3), 3, o);

        const std::vector<std::pair<NodeId, NodeId>> arcs = {{0, 1}, {1, 2}, {2, 4}, {4, 5}, {5, 0}};
        const std::vector<std::size_t> expected_counts = {1, 1, 3, 1, 8};
        bool fixture_ok = true;
        for (std::size_t g = 0; g < arcs.size(); ++g) fixture_ok = fixture_ok && f->idx.flps_on_arc(arcs[g].first, arcs[g].second).size() == expected_counts[g];
        check(fixture_ok, "N7-1 fixture sanity: the arcs of the resulting skeleton hold 1, 1, 3, 1, 8 FLPs");
        check(res.status == OperatorStatus::Ok && res.op == OperatorId::N7 && res.anchor_customer == 3, "N7-1 status ok, operator id, anchor customer");
        check_near(res.base_objective, 1500.0, 1e-9, "N7-1 base objective");
        check(res.generated == 14 && res.log.size() == 14, "N7-1 w=3: 14 candidates (1+1+3+1+8)");

        std::size_t idx = 0;
        bool order_ok = true, moves_ok = true, trips_ok = true, route_ok = true;
        for (std::size_t g = 0; g < arcs.size(); ++g) {
            const auto& fl = f->idx.flps_on_arc(arcs[g].first, arcs[g].second);
            for (NodeId fid : fl) {
                if (idx >= res.log.size()) { order_ok = false; continue; }
                const auto& rec = res.log[idx++];
                moves_ok = moves_ok && rec.move.kind == MoveKind::N7TruckToDrone && rec.move.customer == 3 && rec.move.gap == g && rec.move.launch == fid && rec.move.recovery == arcs[g].second;
                trips_ok = trips_ok && rec.solution.drone_trips.size() == 1 && rec.solution.drone_trips[0].service_customer == 3 && rec.solution.drone_trips[0].launch_node == fid &&
                           rec.solution.drone_trips[0].recovery_node == arcs[g].second && rec.solution.drone_trips[0].drone_id == 0;
                route_ok = route_ok && pos_of(rec.solution.truck_route, 3) == rec.solution.truck_route.size() && pos_of(rec.solution.truck_route, fid) < (arcs[g].second == 0 ? rec.solution.truck_route.size() - 1 : pos_of(rec.solution.truck_route, arcs[g].second)) &&
                           rec.solution.truck_route.size() == 7;
            }
        }
        check(order_ok && moves_ok, "N7-1 candidate order: gaps ascending, FLPs in ratio order; move descriptors (gap, launch, recovery = arc end)");
        check(trips_ok, "N7-1 the new trip is (f, C3, arc end) on drone 0 and is the only trip");
        check(route_ok, "N7-1 C3 left the truck route; the launch FLP lies before its recovery node");
        check_near(res.log[2].objective, 1475.0, 1e-9, "N7-1 g2 f=2500 -> 1475 (hand-derived)");
        check_near(res.log[3].objective, 1500.0, 1e-9, "N7-1 g2 f=3000 -> 1500");
        check_near(res.log[4].objective, 1600.0, 1e-9, "N7-1 g2 f=3500 -> 1600");
        check_near(res.log[1].objective, 1575.0, 1e-9, "N7-1 g1 f=1500 -> 1575");
        check_near(res.log[5].objective, 1725.0, 1e-9, "N7-1 g3 f=4500 -> 1725");
        check_near(res.log[0].objective, 1900.0, 1e-9, "N7-1 g0 f=500 -> 1900");
        check(res.best.has_value() && res.best->move.launch == f->flp_at(2, 4, 0) && res.improves(), "N7-1 best = first FLP of the merged arc, and it improves on the base");
        if (res.best) check_near(res.best->objective, 1475.0, 1e-9, "N7-1 best objective 1475");
        check(same_solution(base, before), "N7-1 the input solution is unchanged");
        check(res.evaluated == res.generated, "N7-1 every candidate went to the evaluator (no free-drone or affected rejections)");
        check_records_against_reference(res, *f, "N7-1");

        // determinism
        const auto res2 = n7_truck_node_delete_drone_insert(base, f->ctx(3), 3, o);
        bool same = res2.log.size() == res.log.size();
        for (std::size_t i = 0; same && i < res.log.size(); ++i) same = same_solution(res.log[i].solution, res2.log[i].solution) && res.log[i].objective == res2.log[i].objective && res.log[i].reason == res2.log[i].reason;
        check(same && res2.best && res.best && same_solution(res.best->solution, res2.best->solution), "N7-1 deterministic: repeated calls give identical candidates in identical order");

        // evaluate_candidates = false: construction only
        N7Options g;
        g.evaluate_candidates = false;
        const auto r3 = n7_truck_node_delete_drone_insert(base, f->ctx(3), 3, g);
        check(r3.generated == 14 && r3.evaluated == 0 && r3.accepted == 0 && !r3.best, "N7-1 evaluate_candidates=false: 14 generated, none evaluated or accepted");
    }

    // =====================================================================
    // N7-2. Window boundaries on the same fixture (anchor C3, s_t = 3, original half-steps of the 5 resulting gaps: 1,3,6,9,11).
    //       eligible iff |h - 6| <= 2w:  w=0 -> {g2}; w=1 -> {g2}; w=2 -> {g1,g2,g3}; w=3 -> all five; w=9 -> all five.
    // =====================================================================
    {
        auto f = make_line(5, 2);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 0};
        struct Case { int w; std::set<std::size_t> gaps; std::size_t n; };
        const std::vector<Case> cases = {{0, {2}, 3}, {1, {2}, 3}, {2, {1, 2, 3}, 5}, {3, {0, 1, 2, 3, 4}, 14}, {9, {0, 1, 2, 3, 4}, 14}};
        for (const auto& c : cases) {
            N7Options o;
            o.record_candidates = true;
            const auto res = n7_truck_node_delete_drone_insert(base, f->ctx(c.w), 3, o);
            std::set<std::size_t> gaps;
            for (const auto& rec : res.log) gaps.insert(rec.move.gap);
            check(res.generated == c.n && gaps == c.gaps, "N7-2 w=" + std::to_string(c.w) + ": " + std::to_string(c.n) + " candidates on exactly the expected gaps (boundary inclusive: |h-6| <= 2w)");
        }
    }

    // =====================================================================
    // N7-3. Depot boundaries. Anchor C1 (s_t = 1): resulting skeleton [0,2,3,4,5,0], merged arc (0,2) = gap 0 (3 FLPs @500,1000,1500).
    //       Original half-steps: g0 merged = 2; g1 = 2*1+3 = 5; g2 = 7 ... vs 2 s_t = 2: w=1 -> {g0}; w=2 -> {g0,g1} (|5-2| = 3 <= 4).
    //       Anchor C5 (s_t = 5, the last interior customer): skeleton [0,1,2,3,4,0], merged arc (4,0) = gap 4, 4000 m -> 6 FLPs,
    //       recovery = the DEPOT (id 0); w=1 -> only the merged gap.
    // =====================================================================
    {
        auto f = make_line(5, 2);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 0};
        N7Options o;
        o.record_candidates = true;
        const auto a1 = n7_truck_node_delete_drone_insert(base, f->ctx(1), 1, o);
        bool all_g0 = true;
        for (const auto& rec : a1.log) all_g0 = all_g0 && rec.move.gap == 0 && rec.move.recovery == 2;
        check(a1.generated == 3 && all_g0, "N7-3 anchor C1, w=1: the 3 FLPs of the merged arc (0,2), recovery C2");
        const auto a2 = n7_truck_node_delete_drone_insert(base, f->ctx(2), 1, o);
        std::set<std::size_t> gp;
        for (const auto& rec : a2.log) gp.insert(rec.move.gap);
        check(a2.generated == 4 && gp == std::set<std::size_t>({0, 1}), "N7-3 anchor C1, w=2: merged arc (3 FLPs) + the next arc (1 FLP)");
        check_records_against_reference(a1, *f, "N7-3 C1");

        const auto a5 = n7_truck_node_delete_drone_insert(base, f->ctx(1), 5, o);
        bool depot_rec = true;
        for (const auto& rec : a5.log)
            depot_rec = depot_rec && rec.move.gap == 4 && rec.move.recovery == 0 && rec.solution.drone_trips.size() == 1 && rec.solution.drone_trips[0].recovery_node == 0 &&
                        rec.solution.truck_route.back() == 0 && pos_of(rec.solution.truck_route, rec.move.launch) < rec.solution.truck_route.size() - 1;
        check(a5.generated == 6 && depot_rec, "N7-3 anchor C5, w=1: the 6 FLPs of the arc (C4, depot); recovery at the depot, launch before it");
        check_records_against_reference(a5, *f, "N7-3 C5");
    }

    // =====================================================================
    // N7-4. No FLP anywhere (d = 5000 m > every arc): one NoReplacementFlp candidate, nothing evaluated. Invalid anchors.
    // =====================================================================
    {
        auto f = make_line(4, 2, 3600.0, 5000.0);
        check(f->idx.flp_count() == 0, "N7-4 fixture sanity: the pool has no FLP");
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 0};
        N7Options o;
        o.record_candidates = true;
        const auto res = n7_truck_node_delete_drone_insert(base, f->ctx(3), 2, o);
        check(res.status == OperatorStatus::Ok && res.generated == 1 && res.rejected[static_cast<std::size_t>(RejectReason::NoReplacementFlp)] == 1 && res.evaluated == 0 && res.accepted == 0 && !res.best,
              "N7-4 no FLP on any eligible arc: ONE NoReplacementFlp candidate, not evaluated, no best");
        check(res.log.size() == 1 && res.log[0].reason == RejectReason::NoReplacementFlp && res.log[0].move.customer == 2, "N7-4 the record names the anchor");
        check_records_against_reference(res, *f, "N7-4");
    }
    {
        auto f = make_line(5, 2);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 0};
        for (NodeId bad : {0, 99}) {
            const auto r = n7_truck_node_delete_drone_insert(base, f->ctx(3), bad);
            check(r.status == OperatorStatus::InvalidAnchor && r.generated == 0, "N7-4 anchor " + std::to_string(bad) + " (depot / unknown node) -> InvalidAnchor, nothing generated");
        }
        Solution bad_base;
        bad_base.truck_route = {0, 1, 2, 4, 5, 0}; // C3 unserved: not evaluator-feasible
        const auto r2 = n7_truck_node_delete_drone_insert(bad_base, f->ctx(3), 2);
        check(r2.status == OperatorStatus::InputInfeasible && r2.generated == 0, "N7-4 infeasible input -> InputInfeasible, nothing generated");
        // a drone-served customer is not a truck anchor
        TruckRoute r = {0, 1, f->flp_at(1, 3, 1), 3, 4, 5, 0};
        Solution withtrip;
        withtrip.truck_route = r;
        withtrip.drone_trips = {trip(0, f->flp_at(1, 3, 1), 2, 3)};
        const auto r3 = n7_truck_node_delete_drone_insert(withtrip, f->ctx(3), 2);
        check(r3.status == OperatorStatus::InvalidAnchor, "N7-4 a drone-served customer is not a valid N7 anchor");
        // random overload: no interior truck customer
        Solution empty_route;
        empty_route.truck_route = {0, 0};
        tdr::util::Rng rr(3);
        const auto r4 = n7_truck_node_delete_drone_insert(empty_route, f->ctx(3), rr);
        check(r4.status == OperatorStatus::NoEligibleAnchor, "N7-4 random overload with no interior truck customer -> NoEligibleAnchor");
    }

    // =====================================================================
    // N7-5. One trip. n = 5, route [0,1,F,3,4,5,0] with F = the middle FLP of L(1,3) (x = 2000), trip T0 = (F, C2, C3), |U| = 2.
    //       Base: C1 100, C2 (drone: launch 200, no flight to C2) 200, C3 300 (drone back at 250), C4 400, C5 500 = 1500.
    //   * anchor C3: it is T0's recovery AND F's arc (1,3) vanishes -> ONE AffectedTrips (trip 0), never evaluated.
    //   * anchor C1: F now sits between the depot and C3 -> AffectedTrips (trip 0).
    //   * anchor C4 (unaffected): resulting skeleton [0,1,3,5,0], w=3 -> all four gaps: (0,1) 1 FLP, (1,3) 3 FLPs (the middle one IS F: reuse),
    //     (3,5) 3 FLPs, (5,0) 8 FLPs = 15 candidates. New trip (f, C4, y).
    //     g2 f=3500: C4 at 375, y=C5 at 425 < 500:  100+200+300+500+375 = 1475;  f=4000: 1500;  f=4500: 475, 525 > 500 (last node): 100+200+300+500+475 = 1575
    //     g1 f=1500 (before F, drone 1: its interval [f,C3] overlaps T0's): launch 150, C4 at 275, y=C3 at 325 > 300, truck waits 25:
    //          100+200+300+(500+25)+275 = 1400
    //     g1 f=F (reused, drone 1): launch 200, C4 at 300, y=C3 at 350, truck waits 50:                      100+200+300+550+300 = 1450
    //     g1 f=2500 (after F, drone 1): launch 250, C4 at 325, y at 375, waits 75:                           100+200+300+575+325 = 1500
    // =====================================================================
    {
        auto f = make_line(5, 2);
        const NodeId F = f->flp_at(1, 3, 1);
        check_near(f->flp_x(F), 2000.0, 1e-6, "N7-5 fixture sanity: F is at x = 2000");
        Solution base;
        base.truck_route = {0, 1, F, 3, 4, 5, 0};
        base.drone_trips = {trip(0, F, 2, 3)};
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 1500.0, 1e-9, "N7-5 base objective 1500 (reference evaluator)");
        check(feas, "N7-5 the base is feasible");
        const Solution before = base;
        N7Options o;
        o.record_candidates = true;

        const auto a3 = n7_truck_node_delete_drone_insert(base, f->ctx(3), 3, o);
        check(a3.status == OperatorStatus::Ok && a3.generated == 1 && a3.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 1 && a3.evaluated == 0 && !a3.best && a3.log.size() == 1 &&
                  a3.log[0].affected_trips == std::vector<std::size_t>({0}),
              "N7-5 anchor C3 (T0's recovery, F's arc vanishes): ONE AffectedTrips naming trip 0, never evaluated");
        const auto a1 = n7_truck_node_delete_drone_insert(base, f->ctx(3), 1, o);
        check(a1.generated == 1 && a1.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 1 && a1.evaluated == 0 && a1.log[0].affected_trips == std::vector<std::size_t>({0}),
              "N7-5 anchor C1 (F lands on a foreign arc): ONE AffectedTrips naming trip 0");

        const auto res = n7_truck_node_delete_drone_insert(base, f->ctx(3), 4, o);
        check(res.status == OperatorStatus::Ok && res.generated == 15 && res.log.size() == 15, "N7-5 anchor C4 (unaffected): 15 candidates (1+3+3+8)");
        bool t0_unchanged = true, appended = true, ids_ok = true;
        for (const auto& rec : res.log) {
            t0_unchanged = t0_unchanged && rec.solution.drone_trips.size() == 2 && rec.solution.drone_trips[0].launch_node == F && rec.solution.drone_trips[0].service_customer == 2 &&
                           rec.solution.drone_trips[0].recovery_node == 3 && rec.solution.drone_trips[0].drone_id == 0;
            appended = appended && rec.solution.drone_trips[1].service_customer == 4;
            const bool overlaps_t0 = rec.move.gap == 1;
            if (rec.reason == RejectReason::None) ids_ok = ids_ok && rec.solution.drone_trips[1].drone_id == (overlaps_t0 ? 1 : 0);
        }
        check(t0_unchanged && appended, "N7-5 the unaffected trip T0 is carried unchanged (launch, customer, recovery, drone id); the new trip is appended");
        check(ids_ok, "N7-5 the new trip gets the LOWEST FREE drone: 1 where it overlaps T0 (gap (1,3)), else 0");
        // log index: g0 -> 0 ; g1 -> 1,2,3 (f1500, F, f2500) ; g2 -> 4,5,6 ; g3 -> 7..14
        check(res.log[2].move.launch == F && std::count(res.log[2].solution.truck_route.begin(), res.log[2].solution.truck_route.end(), F) == 1,
              "N7-5 an FLP already on the route is REUSED (second launch), it appears once in the route");
        check(res.log[2].solution.drone_trips[0].launch_node == res.log[2].solution.drone_trips[1].launch_node, "N7-5 reuse: both trips launch from F");
        check_near(res.log[1].objective, 1400.0, 1e-9, "N7-5 g1 f=1500 (before F, drone 1) -> 1400");
        check_near(res.log[2].objective, 1450.0, 1e-9, "N7-5 g1 f=F reused (drone 1) -> 1450");
        check_near(res.log[3].objective, 1500.0, 1e-9, "N7-5 g1 f=2500 (after F, drone 1) -> 1500");
        check_near(res.log[4].objective, 1475.0, 1e-9, "N7-5 g2 f=3500 -> 1475");
        check_near(res.log[5].objective, 1500.0, 1e-9, "N7-5 g2 f=4000 -> 1500");
        check_near(res.log[6].objective, 1575.0, 1e-9, "N7-5 g2 f=4500 -> 1575");
        check(res.best && res.best->move.launch == f->flp_at(1, 3, 0) && res.best->move.drone_id == 1 && res.improves(), "N7-5 best = f=1500 on drone 1 (1400 < base 1500)");
        if (res.best) check_near(res.best->objective, 1400.0, 1e-9, "N7-5 best objective 1400");
        check(same_solution(base, before), "N7-5 the input solution is unchanged");
        check_records_against_reference(res, *f, "N7-5");
        check_records_against_reference(a3, *f, "N7-5 C3");

        // |U| = 1: every candidate on arc (1,3) overlaps T0 -> NoFreeDrone (not evaluated); the rest are unchanged.
        auto f1 = make_line(5, 1);
        const NodeId F1 = f1->flp_at(1, 3, 1);
        Solution b1;
        b1.truck_route = {0, 1, F1, 3, 4, 5, 0};
        b1.drone_trips = {trip(0, F1, 2, 3)};
        const auto u1 = n7_truck_node_delete_drone_insert(b1, f1->ctx(3), 4, o);
        check(u1.generated == 15 && u1.rejected[static_cast<std::size_t>(RejectReason::NoFreeDrone)] == 3 && u1.evaluated == 12, "N7-5 |U|=1: the 3 overlapping candidates are NoFreeDrone and never evaluated; 12 evaluated");
        bool nf = true;
        for (std::size_t i : {1u, 2u, 3u}) nf = nf && u1.log[i].reason == RejectReason::NoFreeDrone && !u1.log[i].evaluated;
        check(nf, "N7-5 |U|=1: they are exactly the candidates on arc (1,3)");
        check(u1.best && u1.best->move.launch == f1->flp_at(3, 5, 0), "N7-5 |U|=1: best = f=3500 on (3,5) (1475)");
        check_records_against_reference(u1, *f1, "N7-5 |U|=1");
    }

    // =====================================================================
    // N7-6. Two trips (route [0,1,F,3,G,5,0], T0 = (F,C2,C3), T1 = (G,C4,C5), G = x 4000), |U| = 1 (intervals are disjoint).
    //   anchor C1 -> trip 0 affected;  C3 -> trips 0 and 1 (recovery of T0, both arcs vanish);  C5 -> trip 1 (recovery + arc (3,5) vanishes).
    //   A third base with n = 6 gives an unaffected anchor next to nothing: [0,1,F,3,4,5,6,0], anchor C5 -> candidates exist, T0 untouched.
    // =====================================================================
    {
        auto f = make_line(5, 1);
        const NodeId F = f->flp_at(1, 3, 1), G = f->flp_at(3, 5, 1);
        Solution base;
        base.truck_route = {0, 1, F, 3, G, 5, 0};
        base.drone_trips = {trip(0, F, 2, 3), trip(0, G, 4, 5)};
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 1500.0, 1e-9, "N7-6 base objective 1500");
        check(feas, "N7-6 the two-trip base is feasible");
        N7Options o;
        o.record_candidates = true;
        const std::vector<std::pair<NodeId, std::vector<std::size_t>>> expect = {{1, {0}}, {3, {0, 1}}, {5, {1}}};
        for (const auto& e : expect) {
            const auto r = n7_truck_node_delete_drone_insert(base, f->ctx(3), e.first, o);
            check(r.generated == 1 && r.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 1 && r.log[0].affected_trips == e.second && r.evaluated == 0,
                  "N7-6 anchor C" + std::to_string(e.first) + ": AffectedTrips naming exactly the disturbed trips");
        }
        auto f6 = make_line(6, 2);
        const NodeId F6 = f6->flp_at(1, 3, 1);
        Solution b6;
        b6.truck_route = {0, 1, F6, 3, 4, 5, 6, 0};
        b6.drone_trips = {trip(0, F6, 2, 3)};
        const auto r6 = n7_truck_node_delete_drone_insert(b6, f6->ctx(3), 5, o);
        check(r6.status == OperatorStatus::Ok && r6.generated > 3 && r6.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 0, "N7-6 anchor C5 of the 6-customer base touches no FLP: candidates are generated");
        check_records_against_reference(r6, *f6, "N7-6 n=6");
    }

    // =====================================================================
    // N7-7. Endurance. All-truck n = 5, anchor C3, w = 3, endurance 200 s.
    //   g0 f=500: flight = 125 (to C3) + 100 (to C1) = 225 > 200            -> eval-endurance
    //   g1 f=1500: flight 125, rendezvous C2 at 275, on air 125             -> accepted (1575)
    //   g2: flights 75 / 50 / 75 and waits <= 75                            -> accepted (1475, 1500, 1600)
    //   g3 f=4500: flight 175 <= 200, on air 625-450 = 175                  -> accepted (1725)
    //   g4: for the 8 FLPs (x_k = 5000-5000k/9) either flight > 200 (k >= 6: |x-3000|/20 + 150 > 200) or on air = 1000 - (500+55.6k) > 200 (k <= 5)
    //       -> eval-endurance for all 8.        accepted 5, endurance 9, total 14.
    // =====================================================================
    {
        auto f = make_line(5, 2, 200.0);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 0};
        N7Options o;
        o.record_candidates = true;
        const auto res = n7_truck_node_delete_drone_insert(base, f->ctx(3), 3, o);
        check(res.generated == 14 && res.accepted == 5 && res.rejected[static_cast<std::size_t>(RejectReason::EvalEndurance)] == 9 && res.rejected_total() == 9, "N7-7 endurance 200 s: 5 accepted, 9 eval-endurance, nothing else");
        bool g4 = true;
        for (std::size_t i = 6; i < 14; ++i) g4 = g4 && res.log[i].reason == RejectReason::EvalEndurance;
        check(res.log[0].reason == RejectReason::EvalEndurance && g4 && res.log[1].reason == RejectReason::None && res.log[5].reason == RejectReason::None, "N7-7 the rejected ones are g0 and all of g4");
        check(res.best && std::fabs(res.best->objective - 1475.0) < 1e-9, "N7-7 best still 1475");
        check_records_against_reference(res, *f, "N7-7");
    }

    // =====================================================================
    // N7-8. Strict < and first-generated tie-break. vd = vt = 10 m/s, all-truck n = 5, anchor C3, w = 3.
    //   g2 f=2500: C3 at 300, y=C4 at 400 = truck: 100+200+400+500+300 = 1500;  f=3000: C3 at 300, y at 400: 1500 (EXACT tie);  f=3500: 1700.
    //   Every other candidate is larger (g0, g1, g3, g4 delay the truck or the drone). The FIRST generated (f=2500) must be kept.
    // =====================================================================
    {
        auto f = make_line(5, 2, 3600.0, 600.0, 1000.0, 10.0);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 0};
        N7Options o;
        o.record_candidates = true;
        const auto res = n7_truck_node_delete_drone_insert(base, f->ctx(3), 3, o);
        check_near(res.log[2].objective, 1500.0, 1e-12, "N7-8 g2 f=2500 -> 1500");
        check_near(res.log[3].objective, 1500.0, 1e-12, "N7-8 g2 f=3000 -> 1500 (exact tie with the previous one)");
        check_near(res.log[4].objective, 1700.0, 1e-12, "N7-8 g2 f=3500 -> 1700");
        double min_other = 1e18;
        for (std::size_t i = 0; i < res.log.size(); ++i) if (i != 2 && i != 3 && res.log[i].reason == RejectReason::None) min_other = std::min(min_other, res.log[i].objective);
        check(min_other > 1500.0 + 1e-9, "N7-8 every other candidate is strictly worse than 1500 (the tie is exactly between two)");
        check(res.best && res.best->move.launch == f->flp_at(2, 4, 0) && res.best->objective == res.log[2].objective, "N7-8 tie -> the FIRST generated candidate (f=2500) is kept");
        check(!res.improves(), "N7-8 equal to the base objective -> improves() is false (strict <)");
    }

    // =====================================================================
    // N7-9. Random overload: one uniform draw over the interior truck customers (route order); same seed, same result.
    // =====================================================================
    {
        auto f = make_line(5, 2);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 0};
        bool det = true, matches = true, one_draw = true;
        for (std::uint64_t seed = 1; seed <= 12; ++seed) {
            tdr::util::Rng a(seed), b(seed), c(seed);
            const auto ra = n7_truck_node_delete_drone_insert(base, f->ctx(3), a);
            const auto rb = n7_truck_node_delete_drone_insert(base, f->ctx(3), b);
            const NodeId expect_anchor = static_cast<NodeId>(1 + c.uniform_index(5));
            const auto rd = n7_truck_node_delete_drone_insert(base, f->ctx(3), expect_anchor);
            det = det && ra.anchor_customer == rb.anchor_customer && ra.generated == rb.generated && ra.accepted == rb.accepted;
            matches = matches && ra.anchor_customer == expect_anchor && ra.generated == rd.generated && ra.best && rd.best && ra.best->objective == rd.best->objective;
            one_draw = one_draw && a.next_u64() == c.next_u64();
        }
        check(det, "N7-9 the same seed gives the same anchor and result");
        check(matches, "N7-9 the anchor is the uniform_index(#interior truck customers)-th customer in route order");
        check(one_draw, "N7-9 exactly one RNG draw is consumed");
    }

    // =====================================================================
    // N7-10. A trip whose RECOVERY is not the end of its launch arc. n = 5, route [0,1,F,2,3,4,5,0], F = the FLP of L(1,2) (x = 1500),
    //        T0 = (F, C5, recovery C3): the launch FLP is two skeleton nodes before the recovery node.
    //   * anchor C3 (T0's recovery, but F's arc (1,2) is untouched): the ONLY disturbance is the vanished recovery node ->
    //     ONE AffectedTrips naming trip 0, never evaluated. (Distinguishes the recovery rule from the launch-FLP rule.)
    //   * anchor C4: neither F's arc nor T0's recovery is touched -> no AffectedTrips; candidates are generated and judged.
    // =====================================================================
    {
        auto f = make_line(5, 2);
        check(f->idx.flps_on_arc(1, 2).size() == 1, "N7-10 fixture sanity: arc (1,2) holds exactly one FLP");
        const NodeId F = f->flp_at(1, 2, 0);
        Solution base;
        base.truck_route = {0, 1, F, 2, 3, 4, 0};
        base.drone_trips = {trip(0, F, 5, 3)};
        bool feas = false;
        const double bo = ref_objective(base, *f, &feas);
        check(feas && bo > 0, "N7-10 the base (recovery two nodes after the launch arc) is feasible in the reference evaluator");
        const Solution before = base;
        N7Options o;
        o.record_candidates = true;
        const auto a3 = n7_truck_node_delete_drone_insert(base, f->ctx(3), 3, o);
        check(a3.status == OperatorStatus::Ok && a3.generated == 1 && a3.log.size() == 1 && a3.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 1 && a3.evaluated == 0 && !a3.best &&
                  a3.log[0].reason == RejectReason::AffectedTrips && a3.log[0].affected_trips == std::vector<std::size_t>({0}),
              "N7-10 anchor C3 (recovery node of T0, F's arc intact): ONE AffectedTrips naming trip 0, never evaluated");
        check(!has_foreign_flp(a3.log[0].solution.truck_route, *f), "N7-10 ... and the route shown has no foreign FLP: the recovery rule alone caused the rejection");
        const auto a4 = n7_truck_node_delete_drone_insert(base, f->ctx(3), 4, o);
        check(a4.status == OperatorStatus::Ok && a4.generated > 1 && a4.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 0 && a4.evaluated > 0,
              "N7-10 anchor C4 (touches neither F's arc nor the recovery): candidates are generated and evaluated, none AffectedTrips");
        check_records_against_reference(a4, *f, "N7-10 C4");
        check(same_solution(base, before), "N7-10 the input solution is unchanged");
    }

    // =====================================================================
    // N8-1. Dissolve one trip, insert its customer at EVERY gap (D5). n = 5, |U| = 2,
    //       route [0,1,F,3,4,5,0], T0 = (F, C2, C3), F = x 2000. Base objective 1500.
    //       Dissolving T0 drops F (its only trip); skeleton [0,1,3,4,5,0] has gaps 0..4, none holds an FLP -> 5 valid insertions of C2:
    //         g0 [0,2,1,3,4,5,0]: 200 + 300 + 500 + 600 + 700 = 2300       g1 [0,1,2,3,4,5,0]: 100+200+300+400+500 = 1500
    //         g2 [0,1,3,2,4,5,0]: 100 + 300 + 400 + 600 + 700 = 2100       g3 [0,1,3,4,2,5,0]: 100+300+400+600+900 = 2300
    //         g4 [0,1,3,4,5,2,0]: 100 + 300 + 400 + 500 + 800 = 2100
    //       best = g1 (1500), equal to the base -> not an improvement (strict <).
    // =====================================================================
    {
        auto f = make_line(5, 2);
        const NodeId F = f->flp_at(1, 3, 1);
        Solution base;
        base.truck_route = {0, 1, F, 3, 4, 5, 0};
        base.drone_trips = {trip(0, F, 2, 3)};
        const Solution before = base;
        N8Options o;
        o.record_candidates = true;
        const auto res = n8_drone_node_delete_truck_insert(base, f->ctx(3), 0, o);
        check(res.status == OperatorStatus::Ok && res.op == OperatorId::N8 && res.anchor_trip == 0 && res.anchor_customer == 2, "N8-1 status ok, operator id, anchor trip and customer");
        check_near(res.base_objective, 1500.0, 1e-9, "N8-1 base objective");
        check(res.generated == 5 && res.log.size() == 5 && res.accepted == 5 && res.evaluated == 5, "N8-1 five insertion gaps, all valid and evaluated");
        const std::vector<double> expect = {2300.0, 1500.0, 2100.0, 2300.0, 2100.0};
        const std::vector<TruckRoute> routes = {{0, 2, 1, 3, 4, 5, 0}, {0, 1, 2, 3, 4, 5, 0}, {0, 1, 3, 2, 4, 5, 0}, {0, 1, 3, 4, 2, 5, 0}, {0, 1, 3, 4, 5, 2, 0}};
        for (std::size_t g = 0; g < 5; ++g) {
            check(res.log[g].move.kind == MoveKind::N8DroneToTruck && res.log[g].move.gap == g && res.log[g].move.customer == 2 && res.log[g].move.replaced_trip == 0, "N8-1 gap " + std::to_string(g) + ": move descriptor");
            check(res.log[g].solution.truck_route == routes[g] && res.log[g].solution.drone_trips.empty(), "N8-1 gap " + std::to_string(g) + ": C2 inserted before skeleton node g+1; trip dissolved");
            check_near(res.log[g].objective, expect[g], 1e-9, "N8-1 gap " + std::to_string(g) + " objective (hand-derived)");
            check(res.log[g].flps_dropped == std::vector<NodeId>({F}), "N8-1 gap " + std::to_string(g) + ": F dropped (recorded)");
        }
        check(res.best && res.best->move.gap == 1 && std::fabs(res.best->objective - 1500.0) < 1e-9 && !res.improves(), "N8-1 best = gap 1 (1500); equal to the base, so no strict improvement");
        check(res.best && res.best->flps_dropped == std::vector<NodeId>({F}), "N8-1 the best candidate carries flps_dropped");
        check(res.best && res.best->move.kind == MoveKind::N8DroneToTruck && res.best->move.replaced_trip == 0 && res.best->move.customer == res.anchor_customer, "N8-1 the best candidate's move names the dissolved trip, its customer and the operator kind");
        check(same_solution(base, before), "N8-1 the input solution is unchanged");
        check_records_against_reference(res, *f, "N8-1");
        std::set<TruckRoute> distinct;
        for (const auto& rec : res.log) distinct.insert(rec.solution.truck_route);
        check(distinct.size() == res.log.size(), "N8-1 distinct gaps give distinct routes (no duplicates exist)");

        // D5: insertion is NOT restricted to the D1 window; the optional window restricts it.
        // c = C2 has half-step 2*1+1 = 3 (launch FLP in gap 1). |2g+1-3| <= 2w: w=0 -> {1}; w=1 -> {0,1,2}; w=2 -> {0,1,2,3}.
        for (const auto& c : std::vector<std::pair<int, std::set<std::size_t>>>{{0, {1}}, {1, {0, 1, 2}}, {2, {0, 1, 2, 3}}}) {
            N8Options ow = o;
            ow.insertion_window = c.first;
            const auto r = n8_drone_node_delete_truck_insert(base, f->ctx(1), 0, ow);
            std::set<std::size_t> gaps;
            for (const auto& rec : r.log) gaps.insert(rec.move.gap);
            check(gaps == c.second, "N8-1 insertion_window=" + std::to_string(c.first) + " restricts to the D1 window of the customer's half-step");
        }
        const auto unrestricted = n8_drone_node_delete_truck_insert(base, f->ctx(1), 0, o); // adjacency_window = 1 must NOT limit N8
        check(unrestricted.generated == 5, "N8-1 default: all 5 gaps although the adjacency window is only 1 (D5, not silently restricted)");
        check(unrestricted.log[4].reason == RejectReason::None && unrestricted.log[4].evaluated, "N8-1 the gap outside the D1 window (gap 4) is still generated, evaluated and accepted");

        // evaluate_candidates = false
        N8Options g;
        g.evaluate_candidates = false;
        const auto r3 = n8_drone_node_delete_truck_insert(base, f->ctx(3), 0, g);
        check(r3.generated == 5 && r3.evaluated == 0 && r3.accepted == 0 && !r3.best, "N8-1 evaluate_candidates=false: generated, none evaluated");
        // determinism
        const auto r4 = n8_drone_node_delete_truck_insert(base, f->ctx(3), 0, o);
        bool same = r4.log.size() == res.log.size();
        for (std::size_t i = 0; same && i < res.log.size(); ++i) same = r4.log[i].solution.truck_route == res.log[i].solution.truck_route && r4.log[i].objective == res.log[i].objective;
        check(same, "N8-1 deterministic: repeated calls give identical candidates in identical order");
    }

    // =====================================================================
    // N8-2. Affected trips, endurance, several trips. n = 6, |U| = 2, endurance 500 s, route [0,F,1,3,4,5,G,0],
    //       F = the FLP of L(0,1) (x = 500), G = first FLP of L(5,0) (x = 5000-5000/9 = 4444.4);
    //       T0 = (F, C2, C4), T1 = (G, C6, depot). Skeleton [0,1,3,4,5,0]; F in gap 0, G in gap 4... G sits after C5: gap 4 (5,0).
    //       Base: T0 launch 50, C2 at 125, rendezvous at C4 400 (on air 350 <= 500). T1 launch 555.6, C6 at 633.3, depot 933 < 1000 (on air 444 <= 500).
    //       Base objective 100+125+300+400+500+633.33 = 2058.33.
    //       N8 on T1: dissolve T1 (G dropped), insert C6:
    //         g0: F of T0 is in gap 0 -> AffectedTrips (trip 0), not evaluated
    //         g1 [0,F,1,6,3,4,5,0]: C6 at 600, C3 at 900, C4 at 1000 -> T0 on air 1000-50 = 950 > 500           -> eval-endurance
    //         g2 [0,F,1,3,6,4,5,0]: C3 300, C6 600, C4 800 -> T0 on air 750 > 500                               -> eval-endurance
    //         g3 [0,F,1,3,4,6,5,0]: C4 400 (T0 intact), C6 600, C5 700:  100+125+300+400+600+700 = 2225        -> accepted
    //         g4 [0,F,1,3,4,5,6,0]: C5 500, C6 600:                      100+125+300+400+500+600 = 2025        -> accepted (best)
    //       2025 < 2058.33: an improvement.
    // =====================================================================
    {
        auto f = make_line(6, 2, 500.0);
        const NodeId F = f->flp_at(0, 1, 0), G = f->flp_at(5, 0, 0);
        check_near(f->flp_x(F), 500.0, 1e-6, "N8-2 fixture sanity: F at x = 500");
        check_near(f->flp_x(G), 5000.0 - 5000.0 / 9.0, 1e-6, "N8-2 fixture sanity: G at x = 4444.4");
        Solution base;
        base.truck_route = {0, F, 1, 3, 4, 5, G, 0};
        base.drone_trips = {trip(0, F, 2, 4), trip(0, G, 6, 0)};
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 100.0 + 125.0 + 300.0 + 400.0 + 500.0 + 6000.0 * 0 + (500.0 + 5000.0 / 9.0 / 10.0 + (1555.5555555555557 / 20.0)), 1e-6, "N8-2 base objective 2058.33 (reference evaluator)");
        check(feas, "N8-2 the base is feasible");
        const Solution before = base;
        N8Options o;
        o.record_candidates = true;
        const auto res = n8_drone_node_delete_truck_insert(base, f->ctx(3), 1, o);
        check(res.status == OperatorStatus::Ok && res.generated == 5 && res.anchor_customer == 6, "N8-2 trip 1 (C6): 5 insertion gaps");
        check(res.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 1 && res.rejected[static_cast<std::size_t>(RejectReason::EvalEndurance)] == 2 && res.accepted == 2 && res.evaluated == 4,
              "N8-2 one AffectedTrips (gap 0), two eval-endurance (gaps 1, 2), two accepted; 4 evaluator calls");
        check(res.log[0].reason == RejectReason::AffectedTrips && !res.log[0].evaluated && res.log[0].affected_trips == std::vector<std::size_t>({0}), "N8-2 gap 0 holds F: AffectedTrips naming base trip 0, not evaluated");
        check(res.log[1].reason == RejectReason::EvalEndurance && res.log[2].reason == RejectReason::EvalEndurance, "N8-2 gaps 1 and 2 delay T0's recovery: eval-endurance");
        check_near(res.log[3].objective, 2225.0, 1e-9, "N8-2 gap 3 -> 2225");
        check_near(res.log[4].objective, 2025.0, 1e-9, "N8-2 gap 4 -> 2025");
        check(res.best && res.best->move.gap == 4 && res.improves() && res.best->flps_dropped == std::vector<NodeId>({G}), "N8-2 best = gap 4 (2025 < base), G dropped");
        bool t0 = true;
        for (const auto& rec : res.log) t0 = t0 && rec.solution.drone_trips.size() == 1 && rec.solution.drone_trips[0].launch_node == F && rec.solution.drone_trips[0].service_customer == 2 && rec.solution.drone_trips[0].recovery_node == 4 && rec.solution.drone_trips[0].drone_id == 0;
        check(t0, "N8-2 the kept trip T0 is unchanged in every record (launch, customer, recovery, drone id): nothing is repaired");
        check(res.rejected[static_cast<std::size_t>(RejectReason::NoFreeDrone)] == 0 && res.rejected[static_cast<std::size_t>(RejectReason::EvalFleetCapacity)] == 0, "N8-2 N8 never needs a free drone (one trip fewer)");
        check(same_solution(base, before), "N8-2 the input solution is unchanged");
        check_records_against_reference(res, *f, "N8-2");

        // dissolving T0 instead: C2 inserted; T1's FLP G sits in gap 4 -> that insertion is affected
        const auto r0 = n8_drone_node_delete_truck_insert(base, f->ctx(3), 0, o);
        check(r0.generated == 5 && r0.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 1 && r0.log[4].reason == RejectReason::AffectedTrips && r0.log[4].affected_trips == std::vector<std::size_t>({1}),
              "N8-2 dissolving T0: the gap holding G is AffectedTrips naming base trip 1");
        check_records_against_reference(r0, *f, "N8-2 T0");
    }

    // =====================================================================
    // N8-3. FLP shared by two trips and a single truck customer. n = 4, |U| = 2, route [0,F,2,G,0], F = x 1000 on L(0,2), G = x 1500 on L(2,0);
    //       T0 = (F,C1,C2) d0, T1 = (F,C3,C2) d1, T2 = (G,C4,depot) d0. Skeleton [0,2,0]: gaps 0 (F) and 1 (G).
    //   N8 on T0: F stays (T1 still uses it) -> gap 0 foreign; G -> gap 1 foreign:  ZERO valid positions, two AffectedTrips ([1] and [2]), no best.
    //   N8 on T2: G dropped; gap 0 holds F (trips 0,1) -> AffectedTrips [0,1]; gap 1 free -> [0,F,2,4,0]:
    //       C2 at 200 (T1 recovers at 250: truck waits), C4 at 250+200 = 450, C1 100, C3 200:  200+450+100+200 = 950.
    // =====================================================================
    {
        auto f = make_line(4, 2);
        const NodeId F = f->flp_at(0, 2, 1), G = f->flp_at(2, 0, 0);
        check_near(f->flp_x(F), 1000.0, 1e-6, "N8-3 fixture sanity: F at x = 1000");
        check_near(f->flp_x(G), 1500.0, 1e-6, "N8-3 fixture sanity: G at x = 1500");
        Solution base;
        base.truck_route = {0, F, 2, G, 0};
        base.drone_trips = {trip(0, F, 1, 2), trip(1, F, 3, 2), trip(0, G, 4, 0)};
        bool feas = false;
        ref_objective(base, *f, &feas);
        check(feas, "N8-3 the base (two launches from one FLP, three trips) is feasible");
        N8Options o;
        o.record_candidates = true;
        const auto r0 = n8_drone_node_delete_truck_insert(base, f->ctx(3), 0, o);
        check(r0.generated == 2 && r0.accepted == 0 && !r0.best && r0.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 2 && r0.evaluated == 0, "N8-3 dissolving T0: no valid insertion position (both gaps hold FLPs of other trips)");
        check(r0.log[0].affected_trips == std::vector<std::size_t>({1}) && r0.log[1].affected_trips == std::vector<std::size_t>({2}) && r0.log[0].flps_dropped.empty(), "N8-3 the shared FLP F is NOT dropped; trips named: gap 0 -> trip 1, gap 1 -> trip 2");
        const auto r2 = n8_drone_node_delete_truck_insert(base, f->ctx(3), 2, o);
        check(r2.generated == 2 && r2.log[0].reason == RejectReason::AffectedTrips && r2.log[0].affected_trips == std::vector<std::size_t>({0, 1}) && r2.log[1].reason == RejectReason::None && r2.accepted == 1, "N8-3 dissolving T2: gap 0 AffectedTrips naming trips 0 and 1; gap 1 valid");
        check_near(r2.log[1].objective, 950.0, 1e-9, "N8-3 gap 1 objective 950 (hand-derived)");
        check(r2.log[1].flps_dropped == std::vector<NodeId>({G}) && r2.best && r2.best->move.gap == 1, "N8-3 G dropped, best = gap 1");
        check_records_against_reference(r0, *f, "N8-3 T0");
        check_records_against_reference(r2, *f, "N8-3 T2");
    }

    // =====================================================================
    // N8-4. Strict < tie. 2-D fixture: depot (0,0), C1 (1000,0), C2 (0,1000); T0 = (F, C2, C1), F = x 500 on L(0,1).
    //       Dissolve T0, insert C2: gap 0 [0,2,1,0]: 100 + (100 + s/10) ; gap 1 [0,1,2,0]: 100 + (100 + s/10), s = |C1C2| -> identical operations,
    //       an exact tie. The FIRST (gap 0) must be kept.
    // =====================================================================
    {
        auto f = make_points({Point2D{1000, 0}, Point2D{0, 1000}}, 1);
        const NodeId F = f->flp_at(0, 1, 0);
        Solution base;
        base.truck_route = {0, F, 1, 0};
        base.drone_trips = {trip(0, F, 2, 1)};
        bool feas = false;
        ref_objective(base, *f, &feas);
        check(feas, "N8-4 the 2-D base is feasible");
        N8Options o;
        o.record_candidates = true;
        const auto res = n8_drone_node_delete_truck_insert(base, f->ctx(3), 0, o);
        check(res.generated == 2 && res.accepted == 2, "N8-4 two valid insertions");
        check(res.log[0].objective == res.log[1].objective, "N8-4 the two insertions have an EXACTLY equal objective (341.4)");
        check(res.best && res.best->move.gap == 0 && res.best->solution.truck_route == TruckRoute({0, 2, 1, 0}), "N8-4 tie -> the first generated candidate (gap 0) is kept");
        check_near(res.log[0].objective, 200.0 + std::sqrt(2.0e6) / 10.0, 1e-9, "N8-4 objective = 200 + 100 sqrt(2) / 10 * 10 (hand-derived)");
    }

    // =====================================================================
    // N8-5. Anchors: out-of-range trip, no trips (random overload), infeasible input, Rng overload = one draw over the trip indices.
    // =====================================================================
    {
        auto f = make_line(6, 2, 500.0);
        const NodeId F = f->flp_at(0, 1, 0), G = f->flp_at(5, 0, 0);
        Solution base;
        base.truck_route = {0, F, 1, 3, 4, 5, G, 0};
        base.drone_trips = {trip(0, F, 2, 4), trip(0, G, 6, 0)};
        const auto bad = n8_drone_node_delete_truck_insert(base, f->ctx(3), 2);
        check(bad.status == OperatorStatus::InvalidAnchor && bad.generated == 0, "N8-5 trip index out of range -> InvalidAnchor");
        Solution none;
        none.truck_route = {0, 1, 2, 3, 4, 5, 6, 0};
        tdr::util::Rng r0(1);
        const auto ne = n8_drone_node_delete_truck_insert(none, f->ctx(3), r0);
        check(ne.status == OperatorStatus::NoEligibleAnchor, "N8-5 random overload with no trips -> NoEligibleAnchor");
        Solution infeasible = base;
        infeasible.drone_trips.pop_back(); // C6 unserved
        const auto inf = n8_drone_node_delete_truck_insert(infeasible, f->ctx(3), 0);
        check(inf.status == OperatorStatus::InputInfeasible && inf.generated == 0, "N8-5 infeasible input -> InputInfeasible");
        bool det = true, matches = true, one_draw = true;
        for (std::uint64_t seed = 1; seed <= 12; ++seed) {
            tdr::util::Rng a(seed), b(seed), c(seed);
            N8Options o;
            const auto ra = n8_drone_node_delete_truck_insert(base, f->ctx(3), a, o);
            const auto rb = n8_drone_node_delete_truck_insert(base, f->ctx(3), b, o);
            const std::size_t expect_trip = c.uniform_index(2);
            const auto rd = n8_drone_node_delete_truck_insert(base, f->ctx(3), expect_trip, o);
            det = det && ra.anchor_trip == rb.anchor_trip && ra.generated == rb.generated && ra.accepted == rb.accepted;
            matches = matches && ra.anchor_trip == expect_trip && ra.generated == rd.generated && ra.accepted == rd.accepted;
            one_draw = one_draw && a.next_u64() == c.next_u64();
        }
        check(det && matches && one_draw, "N8-5 same seed -> same trip; the trip is uniform_index(#trips); exactly one RNG draw");
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
