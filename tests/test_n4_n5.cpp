// M2 tests: neighbourhoods N4 (truck service node relocate) and N5 (T-D service
// node swap). HAND-DERIVED fixtures on a line, so every expected number below was
// worked out on paper and is not read back from the code under test:
//
//   * depot 0 at x = 0; customer Ci at x = 1000 i metres; truck 10 m/s, drone 20 m/s,
//     SL = SR = 0, service times 0, theta = 0, d = 600 m  =>  floor(D/600) FLPs per arc
//     (an arc of 1000 m has 1 FLP at its midpoint, 2000 m has 3 at 1/4, 2/4, 3/4, ...).
//   * FLP pool = every ordered pair of truck nodes (the eager pool).
//   * truck arrival at x metres along the line, no waiting: x / 10 s.
//
// Independent cross-checks use the FROZEN reference evaluator (tdr::timing_ref),
// not the production one.

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

int main() {
    // =====================================================================
    // 1. New helpers.
    // =====================================================================
    {
        // flp_insertion_position: gap holds FLPs of one arc in ratio order.
        auto f = make_line(3, 2);
        const NodeId a = f->flp_at(1, 3, 0), b = f->flp_at(1, 3, 1), c = f->flp_at(1, 3, 2); // L(1,3): ratios 1/4, 2/4, 3/4
        check(f->flp.at(a).position_ratio < f->flp.at(b).position_ratio && f->flp.at(b).position_ratio < f->flp.at(c).position_ratio,
              "L(1,3) = 3 FLPs in increasing ratio order (fixture sanity)");
        TruckRoute r = {0, 1, b, 3, 0};
        const Skeleton sk = make_skeleton(r, f->nodes); // skeleton: 0@0, 1@1, 3@3, 0@4 ; gap 1 holds {b}
        check(flp_insertion_position(r, sk, 1, a, f->flp) == std::optional<std::size_t>(2), "FLP with a smaller ratio goes BEFORE the one in the gap (position 2)");
        check(flp_insertion_position(r, sk, 1, c, f->flp) == std::optional<std::size_t>(3), "FLP with a larger ratio goes AFTER the one in the gap (position 3)");
        TruckRoute empty_gap = {0, 1, 3, 0};
        const Skeleton sk2 = make_skeleton(empty_gap, f->nodes);
        check(flp_insertion_position(empty_gap, sk2, 1, a, f->flp) == std::optional<std::size_t>(2), "empty gap: insert directly after the gap's left node");
        check(!flp_insertion_position(r, sk, 9, a, f->flp).has_value(), "gap index out of range -> nullopt");
        check(!flp_insertion_position(r, sk, 1, 999, f->flp).has_value(), "FLP without ownership data -> nullopt");
        TruckRoute two = {0, 1, a, c, 3, 0}; // insert b between a and c
        const Skeleton sk3 = make_skeleton(two, f->nodes);
        check(flp_insertion_position(two, sk3, 1, b, f->flp) == std::optional<std::size_t>(3), "middle FLP lands between its two neighbours");

        // gaps_within_window equals the PositionIndex member and the closed-form interval.
        Solution s;
        s.truck_route = {0, 1, 2, 3, 0};
        const PositionIndex pi(s, f->nodes);
        bool same = true;
        for (int h = 0; h <= 8; ++h)
            for (int w = 0; w <= 4; ++w) same = same && pi.gaps_within(h, w) == gaps_within_window(pi.skeleton(), h, w);
        check(same, "gaps_within_window == PositionIndex::gaps_within for every (h, w) tested");
        check(gaps_within_window(pi.skeleton(), 4, 1) == std::vector<std::size_t>({1, 2}), "h=4 (customer at s=2), w=1 -> gaps {1,2}");
        check(gaps_within_window(pi.skeleton(), 2, 3) == std::vector<std::size_t>({0, 1, 2, 3}), "window clipped to the existing gaps 0..3");
        check(gaps_within_window(pi.skeleton(), 4, 0).empty(), "w=0 at an even half-step -> no gap");
        // Odd half-step (a drone customer launched in gap 1): the boundary |2g+1-h| <= 2w is INCLUSIVE.
        check(gaps_within_window(pi.skeleton(), 3, 1) == std::vector<std::size_t>({0, 1, 2}), "odd h=3, w=1 -> gaps {0,1,2}: gaps at distance exactly 2w are included");
        check(gaps_within_window(pi.skeleton(), 3, 0) == std::vector<std::size_t>({1}), "odd h=3, w=0 -> exactly its own gap {1}");
        check(gaps_within_window(pi.skeleton(), 5, 1) == std::vector<std::size_t>({1, 2, 3}), "odd h=5, w=1 -> gaps {1,2,3}");
    }

    // =====================================================================
    // 2. N4 on an all-truck line (no trips, so no trip can be affected).
    //    n = 6, route [0,1,2,3,4,5,6,0]; skeleton index of Ci is i; gaps 0..6.
    //    Objective of a route = sum of arrival times (end depot excluded);
    //    base = 100+200+...+600 = 2100.
    // =====================================================================
    {
        auto f = make_line(6, 2);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 4, 5, 6, 0};
        const Solution base_copy = base;
        N4Options opt;
        opt.record_candidates = true;

        auto r = n4_truck_service_node_relocate(base, f->ctx(1), 3, opt);
        check(r.status == OperatorStatus::Ok && r.op == OperatorId::N4 && r.anchor_customer == 3, "N4 C3 w=1: status ok");
        check_near(r.base_objective, 2100, 1e-9, "N4 base objective = 100+...+600 = 2100");
        check(r.generated == 2 && r.evaluated == 2 && r.accepted == 2 && r.duplicates_skipped == 0 && r.rejected_total() == 0,
              "N4 C3 w=1: relocate has no gap (both gaps touch C3); exchange with C2 and C4 -> 2 candidates, both accepted");
        check(r.log.size() == 2 && r.log[0].move.kind == MoveKind::N4Exchange && r.log[0].move.partner == 2 && r.log[1].move.partner == 4,
              "N4 C3 w=1: partners nearest-first, tie by id (C2 then C4)");
        check(r.log[0].solution.truck_route == TruckRoute({0, 1, 3, 2, 4, 5, 6, 0}) && r.log[1].solution.truck_route == TruckRoute({0, 1, 2, 4, 3, 5, 6, 0}),
              "N4 C3 w=1: candidate routes [0,1,3,2,4,5,6,0] and [0,1,2,4,3,5,6,0]");
        check_near(r.log[0].objective, 2900, 1e-9, "N4 exchange(C3,C2): 100+300+400+600+700+800 = 2900");
        check_near(r.log[1].objective, 2700, 1e-9, "N4 exchange(C3,C4): 100+200+400+500+700+800 = 2700");
        check(r.best && r.best->move.partner == 4 && std::fabs(r.best->objective - 2700) < 1e-9, "N4 C3 w=1: best = exchange with C4 (2700)");
        check(!r.improves(), "N4 C3 w=1: best (2700) does not improve on 2100");
        check(same_solution(base, base_copy), "N4 does not modify its input");

        r = n4_truck_service_node_relocate(base, f->ctx(2), 3, opt);
        check(r.generated == 4 && r.duplicates_skipped == 2 && r.accepted == 4, "N4 C3 w=2: 4 candidates, 2 duplicates (exchange C2/C4 repeat relocations into gaps 1 and 4)");
        check(r.log.size() == 4 && r.log[0].move.kind == MoveKind::N4Relocate && r.log[0].move.gap == 1 && r.log[1].move.kind == MoveKind::N4Relocate && r.log[1].move.gap == 4,
              "N4 C3 w=2: relocations into original gaps 1 and 4 (gaps 2,3 touch C3), ascending");
        check_near(r.log[0].objective, 2900, 1e-9, "w=2 relocate gap1 [0,1,3,2,4,5,6,0] = 2900");
        check_near(r.log[1].objective, 2700, 1e-9, "w=2 relocate gap4 [0,1,2,4,3,5,6,0] = 2700");
        check(r.log[2].move.partner == 1 && r.log[3].move.partner == 5, "w=2 exchanges with C1 then C5 (distance 4, tie by id)");
        check_near(r.log[2].objective, 3900, 1e-9, "w=2 exchange(C3,C1) [0,3,2,1,4,5,6,0]: 300+400+500+800+900+1000 = 3900");
        check_near(r.log[3].objective, 3100, 1e-9, "w=2 exchange(C3,C5) [0,1,2,5,4,3,6,0]: 100+200+500+600+700+1000 = 3100");
        check(r.best && std::fabs(r.best->objective - 2700) < 1e-9 && r.best->move.kind == MoveKind::N4Relocate, "w=2 best = relocation into gap 4 (2700), generated before the equal-routed exchange");

        r = n4_truck_service_node_relocate(base, f->ctx(3), 3, opt);
        check(r.generated == 7 && r.duplicates_skipped == 2 && r.accepted == 7, "N4 C3 w=3: 4 relocations + exchanges with C1,C5,C6 = 7, 2 duplicates (C2,C4)");
        std::vector<std::size_t> gaps;
        for (const auto& c : r.log) if (c.move.kind == MoveKind::N4Relocate) gaps.push_back(c.move.gap);
        check(gaps == std::vector<std::size_t>({0, 1, 4, 5}), "w=3: relocation gaps {0,1,4,5} (window ends exactly at s-w .. s+w-1)");
        check_near(r.log[0].objective, 4100, 1e-9, "w=3 relocate gap0 [0,3,1,2,4,5,6,0] = 300+500+600+800+900+1000 = 4100");
        check_near(r.log[3].objective, 2900, 1e-9, "w=3 relocate gap5 [0,1,2,4,5,3,6,0] = 100+200+400+500+700+1000 = 2900");
        check_near(r.log[6].objective, 3700, 1e-9, "w=3 exchange(C3,C6) [0,1,2,6,4,5,3,0] = 100+200+600+800+900+1100 = 3700 (partner at the window edge s-3)");

        // modes
        N4Options only_reloc = opt;
        only_reloc.mode = N4Mode::Relocate;
        r = n4_truck_service_node_relocate(base, f->ctx(2), 3, only_reloc);
        check(r.generated == 2 && r.duplicates_skipped == 0 && r.log[0].move.kind == MoveKind::N4Relocate && r.log[1].move.kind == MoveKind::N4Relocate, "mode Relocate: 2 relocations, no exchange");
        N4Options only_exch = opt;
        only_exch.mode = N4Mode::Exchange;
        r = n4_truck_service_node_relocate(base, f->ctx(2), 3, only_exch);
        check(r.generated == 4 && r.duplicates_skipped == 0 && r.log[0].move.kind == MoveKind::N4Exchange, "mode Exchange: 4 exchanges (C2,C4,C1,C5), no relocation");

        // depot boundaries
        r = n4_truck_service_node_relocate(base, f->ctx(3), 1, opt);
        gaps.clear();
        for (const auto& c : r.log) if (c.move.kind == MoveKind::N4Relocate) gaps.push_back(c.move.gap);
        check(gaps == std::vector<std::size_t>({2, 3}), "N4 C1 (first customer) w=3: only right-hand gaps {2,3}; nothing before the start depot");
        check(r.generated == 4 && r.duplicates_skipped == 1, "N4 C1 w=3: relocations into gaps 2,3 + exchanges C3,C4 (C2 duplicates gap 2)");
        check_near(r.log[0].objective, 3100, 1e-9, "C1 -> gap2 [0,2,1,3,4,5,6,0] = 200+300+500+600+700+800 = 3100");
        check_near(r.log[1].objective, 3700, 1e-9, "C1 -> gap3 [0,2,3,1,4,5,6,0] = 200+300+500+800+900+1000 = 3700");
        check_near(r.log[3].objective, 5300, 1e-9, "exchange(C1,C4) [0,4,2,3,1,5,6,0] = 400+600+700+900+1300+1400 = 5300 (partner s=4 is exactly 3 away)");
        r = n4_truck_service_node_relocate(base, f->ctx(3), 6, opt);
        gaps.clear();
        for (const auto& c : r.log) if (c.move.kind == MoveKind::N4Relocate) gaps.push_back(c.move.gap);
        check(gaps == std::vector<std::size_t>({3, 4}), "N4 C6 (last customer) w=3: only left-hand gaps {3,4}; nothing after the end depot");
        check_near(r.log[1].objective, 2300, 1e-9, "C6 -> gap4 [0,1,2,3,4,6,5,0] = 100+200+300+400+600+700 = 2300");
        check(r.best && std::fabs(r.best->objective - 2300) < 1e-9 && !r.improves(), "C6 best candidate 2300 > base 2100: the operator reports it but improves() is false");

        // w = 0
        r = n4_truck_service_node_relocate(base, f->ctx(0), 3, opt);
        check(r.status == OperatorStatus::Ok && r.generated == 0 && !r.has_candidate(), "N4 w=0: no gap and no partner -> nothing generated, no candidate");
    }

    // short routes
    {
        auto f = make_line(2, 2);
        Solution one;
        one.truck_route = {0, 1, 2, 0};
        N4Options opt;
        opt.record_candidates = true;
        auto r = n4_truck_service_node_relocate(one, f->ctx(3), 1, opt);
        check(r.generated == 1 && r.duplicates_skipped == 1 && r.log[0].solution.truck_route == TruckRoute({0, 2, 1, 0}),
              "N4 two customers: the only distinct neighbour route is [0,2,1,0] (relocation and exchange coincide)");
        auto g = make_line(1, 2);
        Solution single;
        single.truck_route = {0, 1, 0};
        r = n4_truck_service_node_relocate(single, g->ctx(3), 1, opt);
        check(r.status == OperatorStatus::Ok && r.generated == 0 && !r.has_candidate() && r.rejected_total() == 0,
              "N4 single customer: no candidate exists (status Ok, nothing generated, nothing rejected)");
    }

    // =====================================================================
    // 3. N4 with a drone trip: affected / unaffected trips.
    //    n = 5, route [0, C1, Fa, C2, C4, C5, 0], Fa = the FLP of L(C1,C2)
    //    (x = 1500), trip (drone 0): Fa -> C3 -> C4.  Skeleton: 0:D 1:C1 2:C2 3:C4 4:C5 5:D.
    //    Hand timeline: C1 100, Fa 150, launch 150, C3 225, drone back at C4 at 275,
    //    C2 200, C4 400, C5 500.  Objective = 100+200+400+500+225 = 1425.
    // =====================================================================
    {
        auto f = make_line(5, 2);
        const NodeId fa = f->flp_at(1, 2, 0);
        check(std::fabs(f->flp_x(fa) - 1500) < 1e-9, "fixture: Fa lies at x = 1500 on arc (C1,C2)");
        Solution base;
        base.truck_route = {0, 1, fa, 2, 4, 5, 0};
        base.drone_trips = {trip(0, fa, 3, 4)};
        const Solution base_copy = base;
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 1425, 1e-9, "base objective (reference evaluator) = 1425 as derived by hand");
        check(feas, "base solution is feasible");

        N4Options opt;
        opt.record_candidates = true;
        opt.affected_trips = AffectedTripPolicy::Reject; // the original M2 policy, pinned: every assertion below is the pre-follow-up one
        // anchor C1: every candidate touches the gap that holds Fa
        auto r = n4_truck_service_node_relocate(base, f->ctx(3), 1, opt);
        check_near(r.base_objective, 1425, 1e-9, "N4 operator reports base objective 1425");
        check(r.generated > 0 && r.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == r.generated && r.evaluated == 0 && !r.has_candidate(),
              "N4 C1: EVERY candidate leaves Fa off its arc -> all rejected as affected-trips, none evaluated, none accepted");
        bool names_trip0 = !r.log.empty();
        for (const auto& c : r.log) names_trip0 = names_trip0 && c.reason == RejectReason::AffectedTrips && c.affected_trips == std::vector<std::size_t>({0});
        check(names_trip0, "N4 C1: each rejection names the affected trip (index 0)");
        bool evaluator_agrees = true;
        for (const auto& c : r.log) {
            bool fe = true;
            ref_objective(c.solution, *f, &fe);
            evaluator_agrees = evaluator_agrees && !fe && has_foreign_flp(c.solution.truck_route, *f);
        }
        check(evaluator_agrees, "N4 C1: the reference evaluator also rejects every affected candidate (no feasible candidate was hidden)");

        // anchor C5: gap 1 and C2/C1 exchanges affected, gap 2 / exchange C4 unaffected (duplicates of each other)
        r = n4_truck_service_node_relocate(base, f->ctx(3), 5, opt);
        check(r.generated == 4 && r.duplicates_skipped == 1 && r.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 3 && r.evaluated == 1 && r.accepted == 1,
              "N4 C5 w=3: 4 candidates (1 duplicate skipped): 3 affected, 1 evaluated and accepted");
        check(r.best && r.best->solution.truck_route == TruckRoute({0, 1, fa, 2, 5, 4, 0}) && same_solution(Solution{{}, base.drone_trips}, Solution{{}, r.best->solution.drone_trips}),
              "N4 C5: the accepted candidate is [0,C1,Fa,C2,C5,C4,0] and the trip is untouched (unaffected trip preserved verbatim)");
        check_near(r.best->objective, 1625, 1e-9, "N4 C5 accepted candidate objective: 100+200+500+600+225 = 1625");
        check(!r.improves(), "1625 > 1425: not an improvement");
        check(same_solution(base, base_copy), "N4 with a trip: input unchanged");

        // anchor C4 = the trip's recovery node: gap0 puts the recovery before the launch (evaluator rejects), gap4 ok.
        r = n4_truck_service_node_relocate(base, f->ctx(3), 4, opt);
        // gaps {0,1,4}; exchanges C2,C5 (nearest, duplicate gaps 1 and 4) and C1 -> 4 distinct candidates, 2 duplicates.
        check(r.generated == 4 && r.duplicates_skipped == 2 && r.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 2 &&
                  r.rejected[static_cast<std::size_t>(RejectReason::EvalTripOrder)] == 1 && r.evaluated == 2 && r.accepted == 1,
              "N4 C4 (recovery node) w=3: 4 distinct candidates: 2 affected (gap1, exch C1), 1 trip-order rejection (gap0), 1 accepted (gap4)");
        const auto& first = r.log.front();
        check(first.move.kind == MoveKind::N4Relocate && first.move.gap == 0 && first.evaluated && first.reason == RejectReason::EvalTripOrder,
              "REGRESSION: N4 C4 -> gap 0 puts recovery before launch; reported as eval-trip-order (root cause), not the rule-(6) cascade 'eval-flp-unused' (got " + std::string(reject_reason_name(first.reason)) + ")");
        check(r.best && r.best->solution.truck_route == TruckRoute({0, 1, fa, 2, 5, 4, 0}) && std::fabs(r.best->objective - 1625) < 1e-9, "N4 C4: best = [0,C1,Fa,C2,C5,C4,0] (1625)");

        // anchors that are not interior truck customers
        r = n4_truck_service_node_relocate(base, f->ctx(3), 3, opt);
        check(r.status == OperatorStatus::InvalidAnchor && r.generated == 0, "N4 anchor C3 (drone-served) -> InvalidAnchor");
        r = n4_truck_service_node_relocate(base, f->ctx(3), 0, opt);
        check(r.status == OperatorStatus::InvalidAnchor, "N4 anchor = depot -> InvalidAnchor");
        r = n4_truck_service_node_relocate(base, f->ctx(3), 999, opt);
        check(r.status == OperatorStatus::InvalidAnchor, "N4 unknown anchor -> InvalidAnchor");

        // evaluate_candidates = false: construction + affected check only
        N4Options dry = opt;
        dry.evaluate_candidates = false;
        const auto full = n4_truck_service_node_relocate(base, f->ctx(3), 5, opt);
        const auto d = n4_truck_service_node_relocate(base, f->ctx(3), 5, dry);
        check(d.generated == full.generated && d.evaluated == 0 && d.accepted == 0 && !d.has_candidate() && d.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 3,
              "evaluate_candidates=false: same candidates generated, none evaluated or accepted");
    }

    // infeasible input
    {
        auto f = make_line(5, 2);
        const NodeId fa = f->flp_at(1, 2, 0);
        Solution bad;
        bad.truck_route = {0, 1, fa, 2, 4, 5, 0};
        bad.drone_trips = {trip(0, fa, 3, 1)}; // recovery C1 precedes the launch
        auto r4 = n4_truck_service_node_relocate(bad, f->ctx(3), 2);
        check(r4.status == OperatorStatus::InputInfeasible && r4.generated == 0 && !r4.has_candidate(), "N4 on an infeasible input -> InputInfeasible, nothing generated");
        auto r5 = n5_td_service_node_swap(bad, f->ctx(3), 0);
        check(r5.status == OperatorStatus::InputInfeasible && r5.generated == 0, "N5 on an infeasible input -> InputInfeasible, nothing generated");
        Solution bad2;
        bad2.truck_route = {0, 1, 2, 4, 5, 0}; // customer C3 not served at all
        check(n4_truck_service_node_relocate(bad2, f->ctx(3), 2).status == OperatorStatus::InputInfeasible, "N4: an input that omits a customer is InputInfeasible");
    }


    // =====================================================================
    // 3b. N4 CONSERVATIVE RE-ANCHORING (AffectedTripPolicy::Reanchor, the default).
    //     A stale launch FLP (off its arc after the route change) is replaced by an FLP of the arc it now sits on;
    //     drone customer, drone id and recovery node are preserved; unaffected trips are untouched; a recovery that
    //     is no longer after its launch is rejected as invalid-recovery and never repaired.
    //     All numbers are derived by hand (line fixture, truck 10 m/s; drone 20 m/s unless stated; SL = SR = 0).
    // =====================================================================
    {
        const auto reloc = [](const OperatorResult& r, long gap) -> const CandidateRecord* {
            for (const auto& c : r.log) if (c.move.kind == MoveKind::N4Relocate && static_cast<long>(c.move.gap) == gap) return &c;
            return nullptr;
        };
        const auto exch = [](const OperatorResult& r, NodeId partner) -> const CandidateRecord* {
            for (const auto& c : r.log) if (c.move.kind == MoveKind::N4Exchange && c.move.partner == partner) return &c;
            return nullptr;
        };
        const auto rej = [](const OperatorResult& r, RejectReason why) { return r.rejected[static_cast<std::size_t>(why)]; };
        // drone customer, drone id and recovery of every trip are preserved (only the launch may differ)
        const auto trips_preserved = [](const Solution& a, const Solution& b) {
            if (a.drone_trips.size() != b.drone_trips.size()) return false;
            for (std::size_t i = 0; i < a.drone_trips.size(); ++i)
                if (a.drone_trips[i].drone_id != b.drone_trips[i].drone_id || a.drone_trips[i].service_customer != b.drone_trips[i].service_customer ||
                    a.drone_trips[i].recovery_node != b.drone_trips[i].recovery_node)
                    return false;
            return true;
        };

        // ---------------- Fixture A: d = 520 => 1000 m arc: 1 FLP, 3000 m: 5, 4000 m: 7 ----------------
        // base [0,C1,Fa,C2,C4,C5,0], Fa on (C1,C2) at x = 1500, trip (drone 0) Fa -> C3 -> C4; base objective 1425.
        auto f = make_line(5, 2, 3600.0, 520.0);
        const NodeId fa = f->flp_at(1, 2, 0);
        Solution base;
        base.truck_route = {0, 1, fa, 2, 4, 5, 0};
        base.drone_trips = {trip(0, fa, 3, 4)};
        const Solution base_copy = base;
        N4Options opt;
        opt.record_candidates = true;
        check(opt.affected_trips == AffectedTripPolicy::Reanchor, "N4Options defaults to AffectedTripPolicy::Reanchor");
        check(opt.max_reanchor_variants == 100, "(cap 1) default re-anchor variant cap is 100 (approved 9 Nov 2026; an empirical implementation choice, not a paper parameter)");
        const auto r = n4_truck_service_node_relocate(base, f->ctx(3), 5, opt);
        check(r.status == OperatorStatus::Ok && r.generated == 4 && r.duplicates_skipped == 1 && r.log.size() == 4,
              "Reanchor, anchor C5 w=3: 4 candidates (relocate gap1, gap2; exchange C2, C1), 1 duplicate skipped");
        check(same_solution(base, base_copy), "Reanchor: the input solution is unchanged (test 14)");

        // (1) one affected trip re-anchored
        const auto* g1 = reloc(r, 1);
        check(g1 && g1->reason == RejectReason::None && g1->evaluated && g1->reanchored.size() == 1 && g1->reanchored[0].trip == 0 &&
                  g1->reanchored[0].from == fa && g1->reanchored[0].to == f->flp_at(1, 5, 0),
              "(1) relocate C5 -> gap 1: Fa is stale on arc (C1,C5); trip 0 re-anchored Fa -> first FLP of L(C1,C5)");
        check(g1 && g1->solution.truck_route == TruckRoute({0, 1, f->flp_at(1, 5, 0), 5, 2, 4, 0}), "(1) the stale Fa is gone from the route, the replacement sits on arc (C1,C5)");
        check(g1 && g1->solution.drone_trips.size() == 1 && g1->solution.drone_trips[0].launch_node == f->flp_at(1, 5, 0), "(1) the trip now launches from the replacement FLP");
        check(g1 && g1->variants_generated == 7 && g1->variants_evaluated == 7, "(1) |L(C1,C5)| = 7: 7 variants generated and evaluated (4000 m / 520 m)");
        check(!has_foreign_flp(g1->solution.truck_route, *f), "(1) the re-anchored candidate has no FLP off its arc");

        // (11) several valid FLPs: deterministic selection against hand-derived objectives
        //   truck C1 100, C5 500, C2 800, C4 1000 => 2400; drone C3 at f/10 + |3000-f|/20.
        bool all_variants_ok = true;
        double best_hand = 1e18;
        std::size_t best_k = 99;
        for (std::size_t k = 0; k < 7; ++k) {
            const NodeId fk = f->flp_at(1, 5, k);
            const double x = f->flp_x(fk);
            Solution v;
            v.truck_route = {0, 1, fk, 5, 2, 4, 0};
            v.drone_trips = {trip(0, fk, 3, 4)};
            bool fe = false;
            const double o = ref_objective(v, *f, &fe);
            const double hand = 2400.0 + x / 10.0 + std::fabs(3000.0 - x) / 20.0;
            all_variants_ok = all_variants_ok && fe && std::fabs(o - hand) < 1e-9;
            if (o < best_hand) { best_hand = o; best_k = k; }
        }
        check(all_variants_ok, "(11) all 7 replacement variants are feasible and match the hand formula 2400 + f/10 + |3000-f|/20 (frozen reference evaluator)");
        check(best_k == 0 && g1 && std::fabs(g1->objective - best_hand) < 1e-9 && std::fabs(g1->objective - 2625) < 1e-9,
              "(11) the minimum is the first FLP (x = 1500): 2625; the operator reports exactly that variant");

        // (3) unaffected / (15) existing behaviour: gap 2 keeps Fa on its arc, no re-anchoring, same trip bytes
        const auto* g2 = reloc(r, 2);
        check(g2 && g2->reason == RejectReason::None && g2->reanchored.empty() && g2->variants_generated == 0 && std::fabs(g2->objective - 1625) < 1e-9 &&
                  g2->solution.truck_route == TruckRoute({0, 1, fa, 2, 5, 4, 0}) && same_solution(Solution{{}, base.drone_trips}, Solution{{}, g2->solution.drone_trips}),
              "(3) relocate C5 -> gap 2: Fa stays valid; the trip is byte-for-byte unchanged, nothing re-anchored; objective 1625");
        const auto* e2 = exch(r, 2);
        check(e2 && e2->reason == RejectReason::None && e2->reanchored.size() == 1 && e2->solution.truck_route == TruckRoute({0, 1, f->flp_at(1, 5, 0), 5, 4, 2, 0}) &&
                  std::fabs(e2->objective - 2225) < 1e-9 && e2->variants_generated == 7,
              "exchange C5<->C2 -> [0,C1,F,C5,C4,C2,0]: re-anchored to the first FLP of L(C1,C5); objective 100+500+600+800+225 = 2225");
        const auto* e1 = exch(r, 1);
        check(e1 && e1->reason == RejectReason::None && e1->reanchored.size() == 1 && e1->reanchored[0].to == f->flp_at(5, 2, 0) && e1->variants_generated == 5 &&
                  e1->solution.truck_route == TruckRoute({0, 5, f->flp_at(5, 2, 0), 2, 4, 1, 0}) && std::fabs(e1->objective - 4225) < 1e-9,
              "exchange C5<->C1 -> [0,C5,F,C2,C4,C1,0]: re-anchored on L(C5,C2) (5 FLPs, first x = 4500); objective 3600 + 625 = 4225");

        // (4)(5)(13) invariants over every candidate
        bool preserved = true, once = true, none_foreign = true, ref_agrees = true;
        for (const auto& c : r.log) {
            preserved = preserved && trips_preserved(base, c.solution);
            once = once && exactly_once(c.solution, 5);
            if (c.reason == RejectReason::None) {
                none_foreign = none_foreign && !has_foreign_flp(c.solution.truck_route, *f);
                bool fe = false;
                const double o = ref_objective(c.solution, *f, &fe);
                ref_agrees = ref_agrees && fe && std::fabs(o - c.objective) < 1e-9;
            }
        }
        check(preserved, "(4)(5) every candidate keeps each trip's drone id, drone customer and recovery node");
        check(once, "(13) every candidate serves each customer exactly once");
        check(none_foreign, "every accepted candidate has all FLPs on their arcs");
        check(ref_agrees, "every accepted candidate is feasible under the FROZEN reference evaluator with the same objective");
        check(r.best && r.best->move.kind == MoveKind::N4Relocate && r.best->move.gap == 2 && std::fabs(r.best->objective - 1625) < 1e-9 && !r.improves(),
              "best candidate = the minimum 1625 (not an improvement over 1425)");

        // diagnostics
        check(r.stale_launch_candidates == 3 && r.reanchored_candidates == 3 && r.trips_reanchored == 3 && r.variants_generated == 19 && r.evaluated == 20 && r.accepted == 4 &&
                  r.rejected_total() == 0,
              "diagnostics: 3 stale candidates, 3 re-anchored, 3 trips re-anchored, 19 variants, 20 evaluator calls, 4 accepted, 0 rejected");
        check(r.rejected_total() + r.accepted == r.generated, "diagnostics: every generated candidate is accepted or rejected exactly once (no double counting)");

        // determinism
        {
            const auto again = n4_truck_service_node_relocate(base, f->ctx(3), 5, opt);
            bool same = again.log.size() == r.log.size() && again.accepted == r.accepted && again.variants_generated == r.variants_generated;
            for (std::size_t i = 0; same && i < r.log.size(); ++i)
                same = same_solution(r.log[i].solution, again.log[i].solution) && r.log[i].objective == again.log[i].objective && r.log[i].reason == again.log[i].reason;
            check(same, "Reanchor is deterministic: a second identical call yields identical records");
        }

        // evaluate_candidates = false under Reanchor
        {
            N4Options dry = opt;
            dry.evaluate_candidates = false;
            const auto d = n4_truck_service_node_relocate(base, f->ctx(3), 5, dry);
            check(d.generated == 4 && d.evaluated == 0 && d.accepted == 0 && !d.has_candidate() && d.rejected_total() == 0 && d.variants_generated == 19 && d.stale_launch_candidates == 3,
                  "Reanchor with evaluate_candidates=false: same candidates, 19 variants built, nothing evaluated, accepted or rejected");
        }

        // ---------------- Reject policy still reports AffectedTrips on the same fixture (15) ----------------
        {
            N4Options rj = opt;
            rj.affected_trips = AffectedTripPolicy::Reject;
            const auto q = n4_truck_service_node_relocate(base, f->ctx(3), 5, rj);
            check(q.generated == 4 && rej(q, RejectReason::AffectedTrips) == 3 && q.evaluated == 1 && q.accepted == 1 && q.variants_generated == 0 && q.reanchored_candidates == 0,
                  "(15) Reject policy on the same fixture: 3 AffectedTrips, 1 evaluated and accepted, no variants");
        }

        // ---------------- (7) endurance fails after a valid replacement (E = 300 s) ----------------
        {
            auto fe5 = make_line(5, 2, 300.0, 520.0);
            const NodeId fae = fe5->flp_at(1, 2, 0);
            Solution b5;
            b5.truck_route = {0, 1, fae, 2, 4, 5, 0};
            b5.drone_trips = {trip(0, fae, 3, 4)};
            bool fe = false;
            ref_objective(b5, *fe5, &fe);
            check(fe, "(7) endurance fixture: base feasible (flight 125 s <= 300 s)");
            const auto q = n4_truck_service_node_relocate(b5, fe5->ctx(3), 5, opt);
            const auto* c = reloc(q, 1);
            // C4 is reached at t = 1000 after the move; the drone waits for the truck there. The latest launch on L(C1,C5) is at x = 4500, t = 450,
            // so the flight lasts at least 550 s > 300 s for EVERY replacement.
            check(c && c->reason == RejectReason::EvalEndurance && c->variants_generated == 7 && c->variants_evaluated == 7 && c->evaluated && c->reanchored.size() == 1,
                  "(7) relocate C5 -> gap 1: replacements exist (7 variants), every one violates endurance -> eval-endurance, not accepted");
            bool every_infeasible = true;
            for (std::size_t k = 0; k < 7; ++k) {
                const NodeId fk = fe5->flp_at(1, 5, k);
                Solution v;
                v.truck_route = {0, 1, fk, 5, 2, 4, 0};
                v.drone_trips = {trip(0, fk, 3, 4)};
                bool ok = true;
                ref_objective(v, *fe5, &ok);
                every_infeasible = every_infeasible && !ok;
            }
            check(every_infeasible, "(7) the frozen reference evaluator also rejects all 7 variants");
            check(rej(q, RejectReason::EvalEndurance) >= 1 && rej(q, RejectReason::NoReplacementFlp) == 0, "(7) counted once, as an evaluator rejection (not as no-replacement)");
            const auto* x = exch(q, 2);
            // exchange C5<->C2: C4 reached at t = 600; feasibility needs launch >= 300 s, i.e. x >= 3000
            check(x && x->reason == RejectReason::None && x->reanchored.size() == 1 && fe5->flp_x(x->reanchored[0].to) >= 3000.0 - 1e-9,
                  "(7) a different candidate with a later launch is accepted: only FLPs with x >= 3000 satisfy endurance");
            bool rb = true;
            if (x) { bool ok = false; const double o = ref_objective(x->solution, *fe5, &ok); rb = ok && std::fabs(o - x->objective) < 1e-9; }
            check(rb, "(7) that accepted candidate agrees with the reference evaluator");
        }

        // ---------------- (6) no replacement FLP: d = 1500 => 1000 m arcs have NO FLP ----------------
        {
            auto f4 = make_line(5, 2, 3600.0, 1500.0);
            check(f4->idx.flps_on_arc(1, 2).empty() && f4->idx.flps_on_arc(1, 3).size() == 1 && f4->idx.flps_on_arc(1, 4).size() == 2, "(6) fixture: |L(C1,C2)| = 0, |L(C1,C3)| = 1, |L(C1,C4)| = 2");
            const NodeId fl = f4->flp_at(1, 3, 0);
            Solution b4;
            b4.truck_route = {0, 1, fl, 3, 2, 4, 0};
            b4.drone_trips = {trip(0, fl, 5, 4)};
            bool fe = false;
            check_near(ref_objective(b4, *f4, &fe), 1750, 1e-9, "(6) base objective 1750 (hand: 100+300+400+600... derived)");
            const Solution b4_copy = b4;
            const auto q = n4_truck_service_node_relocate(b4, f4->ctx(3), 3, opt);
            check(q.generated == 5 && rej(q, RejectReason::NoReplacementFlp) == 3 && q.accepted == 2 && q.evaluated == 3 && q.rejected_total() == 3,
                  "(6) anchor C3: removing C3 leaves Fl on arc (C1,C2) which has no FLP -> 3 relocations rejected as no-replacement-flp; 2 exchanges accepted");
            bool never_evaluated = true, intact = true;
            for (const auto& c : q.log)
                if (c.reason == RejectReason::NoReplacementFlp) {
                    never_evaluated = never_evaluated && !c.evaluated && c.variants_generated == 0 && c.reanchored.empty();
                    intact = intact && same_solution(Solution{{}, b4.drone_trips}, Solution{{}, c.solution.drone_trips}) && c.affected_trips == std::vector<std::size_t>({0});
                }
            check(never_evaluated, "(6) no-replacement candidates are never evaluated and carry no re-anchoring");
            check(intact, "(6)(10) the trip is left exactly as it was (not dissolved, not altered) and the record names trip 0");
            const auto* p4 = exch(q, 4);
            // exchange C3<->C4: [0,C1,F,C4,C2,C3,0]: truck 100+400+600+700 = 1800; F on L(C1,C4) at x=2000 or 3000: C5 at 350 / 400 -> 2150
            check(p4 && p4->reason == RejectReason::None && p4->variants_generated == 2 && std::fabs(p4->objective - 2150) < 1e-9 && p4->reanchored.size() == 1 &&
                      std::fabs(f4->flp_x(p4->reanchored[0].to) - 2000.0) < 1e-9,
                  "(11) exchange C3<->C4: 2 replacement FLPs (x = 2000, 3000); the x = 2000 launch wins: 1800 + 350 = 2150");
            const auto* p1 = exch(q, 1);
            check(p1 && p1->reason == RejectReason::None && p1->variants_generated == 1 && std::fabs(p1->objective - 2750) < 1e-9, "exchange C3<->C1: single replacement FLP (x = 2000): 2200 + 550 = 2750");
            check(same_solution(b4, b4_copy), "(6) input unchanged");
        }

        // ---------------- (2) several affected trips, re-anchored deterministically ----------------
        // base [0,C1,Fa,C2,Fb,C3,0]; Fa on (C1,C2) x=1500, Fb on (C2,C3) x=2500;
        // trips: A (drone 0) Fa -> C4 -> C3,  B (drone 1) Fb -> C5 -> C3.  Base objective 100+200+300+275+375 = 1250.
        auto f2 = make_line(5, 2);
        const NodeId a2 = f2->flp_at(1, 2, 0), b2 = f2->flp_at(2, 3, 0);
        Solution m;
        m.truck_route = {0, 1, a2, 2, b2, 3, 0};
        m.drone_trips = {trip(0, a2, 4, 3), trip(1, b2, 5, 3)};
        const Solution m_copy = m;
        {
            bool fe = false;
            check_near(ref_objective(m, *f2, &fe), 1250, 1e-9, "(2) two-trip base objective 1250 (hand)");
            const auto q = n4_truck_service_node_relocate(m, f2->ctx(3), 2, opt);
            check(q.generated == 4 && q.log.size() == 4, "(2) anchor C2: 4 candidates");
            const NodeId l13 = f2->flp_at(1, 3, 0);
            const auto* c0 = reloc(q, 0);
            // [0,C2,C1,F,C3,0]: both stale FLPs now sit on (C1,C3) {x = 1500, 2000, 2500}; 3 x 3 = 9 variants; best is (1500,1500):
            //  C2 200, C1 300, C3 500; F at t=350: C4 at 475, C5 at 525  => 200+300+500+475+525 = 2000
            check(c0 && c0->reason == RejectReason::None && c0->reanchored.size() == 2 && c0->reanchored[0].trip == 0 && c0->reanchored[1].trip == 1 &&
                      c0->reanchored[0].to == l13 && c0->reanchored[1].to == l13 && c0->variants_generated == 9 && c0->variants_evaluated == 9 &&
                      std::fabs(c0->objective - 2000) < 1e-9 && c0->solution.truck_route == TruckRoute({0, 2, 1, l13, 3, 0}),
                  "(2) relocate C2 -> gap 0: BOTH trips stale, re-anchored in trip order, 3 x 3 = 9 joint variants, best (1500,1500) = 2000, one shared FLP node");
            const auto* c3 = reloc(q, 3);
            check(c3 && c3->reason == RejectReason::None && c3->reanchored.size() == 2 && c3->variants_generated == 9 && std::fabs(c3->objective - 1525) < 1e-9 &&
                      c3->solution.truck_route == TruckRoute({0, 1, l13, 3, 2, 0}),
                  "(2) relocate C2 -> gap 3: both trips re-anchored on (C1,C3): 100+300+525+275+325 = 1525 (truck waits for drone B at C3)");
            const auto* x1 = exch(q, 1);
            check(x1 && x1->reason == RejectReason::None && x1->reanchored.size() == 2 && x1->reanchored[0].to == f2->flp_at(2, 1, 0) && x1->reanchored[1].to == l13 &&
                      x1->variants_generated == 3 && std::fabs(x1->objective - 1900) < 1e-9,
                  "(2) exchange C2<->C1: trip A on L(C2,C1) (1 FLP), trip B on L(C1,C3) (3 FLPs): 1 x 3 variants, best 200+300+500+375+525 = 1900");
            // (9)(10) exchange C2<->C3: trip B's recovery C3 is now BEFORE its launch FLP -> invalid recovery, never repaired
            const auto* x3 = exch(q, 3);
            check(x3 && x3->reason == RejectReason::InvalidRecovery && !x3->evaluated && x3->variants_generated == 0 && x3->reanchored.empty() && x3->affected_trips == std::vector<std::size_t>({1}),
                  "(9) exchange C2<->C3: trip B recovers at C3 which now precedes its launch -> invalid-recovery (names trip 1), not evaluated");
            check(x3 && x3->solution.drone_trips[0].launch_node == a2 && x3->solution.drone_trips[1].launch_node == b2 && x3->solution.truck_route == TruckRoute({0, 1, a2, 3, b2, 2, 0}),
                  "(10) the invalid-recovery candidate is shown UNREPAIRED: stale launches and the route as the move produced it");
            check(x3 && !x3->solution.drone_trips.empty() && [&] { bool ok = true; ref_objective(x3->solution, *f2, &ok); return !ok; }(),
                  "(10) the frozen reference evaluator also finds that unrepaired candidate infeasible");
            check(q.stale_launch_candidates == 4 && q.reanchored_candidates == 3 && q.trips_reanchored == 6 && q.variants_generated == 21 && q.evaluated == 21 &&
                      rej(q, RejectReason::InvalidRecovery) == 1 && q.rejected_total() == 1 && q.accepted == 3,
                  "(2) diagnostics: 4 stale, 3 re-anchored, 6 trips, 21 variants, 21 evaluator calls, 1 invalid-recovery");
            bool preserved2 = true, once2 = true;
            for (const auto& c : q.log) { preserved2 = preserved2 && trips_preserved(m, c.solution); once2 = once2 && exactly_once(c.solution, 5); }
            check(preserved2 && once2, "(4)(5)(13) two-trip fixture: ids, customers, recoveries preserved; exactly-once coverage");
            check(same_solution(m, m_copy), "(14) two-trip fixture: input unchanged");

            // (2) determinism
            const auto q2 = n4_truck_service_node_relocate(m, f2->ctx(3), 2, opt);
            bool same = q2.log.size() == q.log.size();
            for (std::size_t i = 0; same && i < q.log.size(); ++i)
                same = same_solution(q.log[i].solution, q2.log[i].solution) && q.log[i].objective == q2.log[i].objective && q.log[i].reanchored.size() == q2.log[i].reanchored.size();
            check(same, "(2) multiple-trip re-anchoring is deterministic across repeated calls");

            // budget: 9 joint combinations in the two relocations, 3 in the exchange (budget-bounded, never a rejection)
            N4Options cap = opt;
            cap.max_reanchor_variants = 8;
            const auto qc = n4_truck_service_node_relocate(m, f2->ctx(3), 2, cap);
            check(qc.accepted == 3 && qc.rejected_total() == 1 && rej(qc, RejectReason::InvalidRecovery) == 1 && qc.variants_generated == 19 && qc.evaluated == 19 && qc.cap_reached_candidates == 2,
                  "cap 8: the two 9-combination candidates are cut at 8 combinations (cap reached in exactly 2 candidates), still evaluated and accepted; the 3-combination one is complete");
            cap.max_reanchor_variants = 9;
            const auto qc9 = n4_truck_service_node_relocate(m, f2->ctx(3), 2, cap);
            check(qc9.accepted == 3 && qc9.cap_reached_candidates == 0 && qc9.variants_generated == 21, "cap 9 (exactly the combinations): not reached, nothing truncated");
            cap.max_reanchor_variants = 0;
            const auto qc0 = n4_truck_service_node_relocate(m, f2->ctx(3), 2, cap);
            check(qc0.accepted == 3 && qc0.variants_generated == 21 && qc0.cap_reached_candidates == 0, "cap 0 = unlimited (exhaustive)");

            // (3) unaffected trip stays byte-for-byte: anchor C1
            const auto q1 = n4_truck_service_node_relocate(m, f2->ctx(3), 1, opt);
            const auto* d3 = reloc(q1, 3);
            check(d3 && d3->reason == RejectReason::None && d3->reanchored.size() == 1 && d3->reanchored[0].trip == 0 && d3->reanchored[0].to == f2->flp_at(0, 2, 0) &&
                      d3->solution.drone_trips[1].launch_node == b2 && d3->solution.drone_trips[1].drone_id == 1 && d3->solution.drone_trips[1].service_customer == 5 &&
                      d3->solution.drone_trips[1].recovery_node == 3 && std::fabs(d3->objective - 1775) < 1e-9 &&
                      d3->solution.truck_route == TruckRoute({0, f2->flp_at(0, 2, 0), 2, b2, 3, 1, 0}),
                  "(3) relocate C1 -> gap 3: only trip A (stale on (C0,C2)) is re-anchored to x = 500; trip B and Fb are byte-for-byte unchanged; objective 1775");
            const auto* d2 = reloc(q1, 2);
            check(d2 && d2->reason == RejectReason::None && d2->reanchored.size() == 2 && std::fabs(d2->objective - 1650) < 1e-9 &&
                      d2->solution.drone_trips[0].launch_node == f2->flp_at(0, 2, 0) && d2->solution.drone_trips[1].launch_node == f2->flp_at(2, 1, 0),
                  "relocate C1 -> gap 2: A on L(C0,C2) first FLP, B on L(C2,C1): 200+300+500+225+425 = 1650");
        }

        // ---------------- (8) fleet capacity fails after a valid replacement ----------------
        {
            auto ff = make_line(5, 1);
            const NodeId f0 = ff->flp_at(0, 1, 0), fbb = ff->flp_at(3, 0, 0);
            Solution b6;
            b6.truck_route = {0, f0, 1, 2, 3, fbb, 0};
            b6.drone_trips = {trip(0, f0, 4, 2), trip(0, fbb, 5, 0)};
            bool fe = false;
            ref_objective(b6, *ff, &fe);
            check(fe, "(8) fleet fixture: base feasible with ONE drone (A: Fa0 -> C4 -> C2, then B: Fb -> C5 -> depot, sequential)");
            const auto q = n4_truck_service_node_relocate(b6, ff->ctx(3), 2, opt);
            const auto* c = reloc(q, 3);
            check(c && c->reason == RejectReason::EvalFleetCapacity && c->variants_generated == 1 && c->variants_evaluated == 1 && c->evaluated && c->affected_trips.empty(),
                  "(8) relocate C2 -> gap 3: Fb is stale, ONE replacement exists on L(C3,C2); A (recovering at C2) is still in flight when B launches -> eval-fleet-capacity");
            Solution hand;
            hand.truck_route = {0, f0, 1, 3, ff->flp_at(3, 2, 0), 2, 0};
            hand.drone_trips = {trip(0, f0, 4, 2), trip(0, ff->flp_at(3, 2, 0), 5, 0)};
            bool ok = true;
            ref_objective(hand, *ff, &ok);
            check(!ok, "(8) the frozen reference evaluator rejects that exact re-anchored solution");
            const auto* c0 = reloc(q, 0);
            check(c0 && c0->reason == RejectReason::None && c0->reanchored.size() == 1 && c0->reanchored[0].trip == 0 && c0->variants_generated == 3,
                  "(8) relocate C2 -> gap 0 on the same fixture is accepted: A re-anchored on L(C0,C2) (3 FLPs), still sequential with B");
            check(rej(q, RejectReason::EvalFleetCapacity) >= 1 && rej(q, RejectReason::NoReplacementFlp) == 0, "(8) rejection counted once, under the evaluator reason");

            // with two drones (ids 0 and 1) the identical candidate is feasible
            auto ff2 = make_line(5, 2);
            const NodeId g0 = ff2->flp_at(0, 1, 0), gb = ff2->flp_at(3, 0, 0);
            Solution b7;
            b7.truck_route = {0, g0, 1, 2, 3, gb, 0};
            b7.drone_trips = {trip(0, g0, 4, 2), trip(1, gb, 5, 0)};
            const auto q7 = n4_truck_service_node_relocate(b7, ff2->ctx(3), 2, opt);
            const auto* c7 = reloc(q7, 3);
            check(c7 && c7->reason == RejectReason::None && c7->reanchored.size() == 1, "(8) with |U| = 2 (ids 0 and 1) the same relocation is accepted: the rejection above was the fleet limit, not the re-anchoring");
        }

        // ---------------- (9)(10) recovery invalidated ----------------
        {
            // base [0,C1,Fa,C2,C4,C5,0]; trip Fa -> C3 -> C2 (recovery = the next truck node)
            auto fr = make_line(5, 2);
            const NodeId fr_a = fr->flp_at(1, 2, 0);
            Solution b;
            b.truck_route = {0, 1, fr_a, 2, 4, 5, 0};
            b.drone_trips = {trip(0, fr_a, 3, 2)};
            const Solution bc = b;
            bool fe = false;
            ref_objective(b, *fr, &fe);
            check(fe, "(9) recovery fixture feasible");
            const auto q = n4_truck_service_node_relocate(b, fr->ctx(3), 2, opt);
            const auto* g0 = reloc(q, 0);
            const auto* e1 = exch(q, 1);
            check(g0 && g0->reason == RejectReason::InvalidRecovery && !g0->evaluated && g0->variants_generated == 0 && g0->solution.truck_route == TruckRoute({0, 2, 1, fr_a, 4, 5, 0}) &&
                      g0->affected_trips == std::vector<std::size_t>({0}) && same_solution(Solution{{}, b.drone_trips}, Solution{{}, g0->solution.drone_trips}),
                  "(9)(10) relocate C2 -> gap 0: recovery C2 is now before the launch; reported as invalid-recovery; trip and stale FLP left untouched (no repair)");
            check(e1 && e1->reason == RejectReason::InvalidRecovery && !e1->evaluated && e1->solution.truck_route == TruckRoute({0, 2, fr_a, 1, 4, 5, 0}),
                  "(9) exchange C2<->C1: the recovery precedes the launch again -> invalid-recovery");
            check(rej(q, RejectReason::InvalidRecovery) == 2 && q.rejected_total() == 2 && q.accepted == 3 && q.generated == 5,
                  "(9) counted once each: 2 invalid-recovery, 3 accepted, 0 other rejections");
            // changing the recovery would 'fix' it, so prove that the operator did not: the candidate is infeasible as produced
            bool both_infeasible = true;
            for (const auto* c : {g0, e1}) { bool ok = true; if (c) ref_objective(c->solution, *fr, &ok); both_infeasible = both_infeasible && !ok; }
            check(both_infeasible, "(10) both unrepaired candidates are infeasible under the frozen reference evaluator");
            // the accepted ones keep the recovery C2 even though it is no longer the next node
            const auto* g3 = reloc(q, 3);
            check(g3 && g3->reason == RejectReason::None && g3->reanchored.size() == 1 && g3->solution.drone_trips[0].recovery_node == 2 && g3->solution.drone_trips[0].service_customer == 3 &&
                      g3->solution.drone_trips[0].drone_id == 0 && g3->variants_generated == 5,
                  "(5) accepted relocation keeps recovery = C2 (no longer adjacent to the launch) and the customer; 5 FLPs on L(C1,C4)");
            check(same_solution(b, bc), "(14) input unchanged");
            bool once = true;
            for (const auto& c : q.log) once = once && exactly_once(c.solution, 5);
            check(once, "(13) exactly-once coverage in every record, rejected ones included");
        }

        // ---------------- (12) exact ties between replacement FLPs: drone as fast as the truck ----------------
        {
            auto ft = make_line(5, 2, 3600.0, 520.0, 1000.0, 10.0);
            const NodeId ta = ft->flp_at(1, 2, 0);
            Solution b;
            b.truck_route = {0, 1, ta, 2, 4, 5, 0};
            b.drone_trips = {trip(0, ta, 3, 4)};
            bool fe = false;
            check_near(ref_objective(b, *ft, &fe), 1500, 1e-9, "(12) tie fixture (vd = vt = 10): base objective 1500");
            const auto q = n4_truck_service_node_relocate(b, ft->ctx(3), 5, opt);
            const auto* c = reloc(q, 1);
            // drone customer time f/10 + |3000-f|/10 = 300 for every x <= 3000: x = 1500, 2000, 2500, 3000 tie exactly; 3500, 4000, 4500 give 400, 500, 600 + ... -> 2800, 2900, 3000
            const double expect[7] = {2700, 2700, 2700, 2700, 2800, 2900, 3000};
            bool hand = true;
            for (std::size_t k = 0; k < 7; ++k) {
                const NodeId fk = ft->flp_at(1, 5, k);
                Solution v;
                v.truck_route = {0, 1, fk, 5, 2, 4, 0};
                v.drone_trips = {trip(0, fk, 3, 4)};
                bool ok = false;
                hand = hand && std::fabs(ref_objective(v, *ft, &ok) - expect[k]) < 1e-9 && ok;
            }
            check(hand, "(12) the 7 replacement variants give 2700,2700,2700,2700,2800,2900,3000 (hand + frozen reference): a 4-way exact tie");
            check(c && c->reason == RejectReason::None && c->reanchored.size() == 1 && c->reanchored[0].to == ft->flp_at(1, 5, 0) && c->objective == 2700.0 && c->variants_generated == 7,
                  "(12)(17) among the 4 equal replacements the FIRST generated (lowest ratio, x = 1500) is kept: strict '<', no epsilon");
        }

        // ---------------- (16) N5 untouched by the N4 policy ----------------
        {
            N5Options o5;
            o5.record_candidates = true;
            const auto a = n5_td_service_node_swap(base, f->ctx(3), 0, o5);
            const auto b = n5_td_service_node_swap(base, f->ctx(3), 0, o5);
            bool same = a.log.size() == b.log.size() && a.generated == b.generated && a.accepted == b.accepted;
            for (std::size_t i = 0; same && i < a.log.size(); ++i) same = a.log[i].objective == b.log[i].objective && a.log[i].reason == b.log[i].reason;
            check(same && a.generated > 0 && a.reanchored_candidates == 0 && a.variants_generated == 0 && a.stale_launch_candidates == 0,
                  "(16) N5 is deterministic and reports no re-anchoring (the policy is N4-only)");
        }
    }


    // =====================================================================
    // 3c. BUDGET-BOUNDED joint re-anchoring: N4Options::max_reanchor_variants (default 100; 0 = unlimited).
    //     The combinations are enumerated in a fixed order (stale trips by index, the last one fastest; FLPs by
    //     ratio) and the search stops after `cap` of them. A finite cap is NOT exhaustive. Nothing here asserts
    //     that cap 100 equals exhaustive enumeration.
    // =====================================================================
    {
        const auto reloc = [](const OperatorResult& r, long gap) -> const CandidateRecord* {
            for (const auto& c : r.log) if (c.move.kind == MoveKind::N4Relocate && static_cast<long>(c.move.gap) == gap) return &c;
            return nullptr;
        };
        const auto rej = [](const OperatorResult& r, RejectReason why) { return r.rejected[static_cast<std::size_t>(why)]; };
        const auto run = [](const Fix& f, const Solution& b, NodeId anchor, std::size_t cap, AffectedTripPolicy pol = AffectedTripPolicy::Reanchor) {
            N4Options o;
            o.record_candidates = true;
            o.max_reanchor_variants = cap;
            o.affected_trips = pol;
            return n4_truck_service_node_relocate(b, f.ctx(3), anchor, o);
        };

        // ---- endurance ladder: E = 300 s; on L(C1,C5) / L(C5,C2) only the later FLPs (x >= 3000) are feasible, hand-derived ----
        //   exchange C5<->C2: route [0,C1,F,C5,C4,C2,0], C4 reached at t = 600, drone waits there: duration = 600 - launch <= 300 => launch >= 300 s => x >= 3000 = the 4th FLP of 7
        //   exchange C5<->C1: route [0,C5,F,C2,C4,C1,0], C4 reached at t = 1000: launch >= 700 s => x <= 3000 = the 4th FLP of 5 (x = 4500, 4000, 3500, 3000, 2500)
        //   relocate C5 -> gap 1: C4 at t = 1000 but launches happen at x/10 <= 450 s: no FLP is feasible.
        auto fe = make_line(5, 2, 300.0, 520.0);
        const NodeId fae = fe->flp_at(1, 2, 0);
        Solution be;
        be.truck_route = {0, 1, fae, 2, 4, 5, 0};
        be.drone_trips = {trip(0, fae, 3, 4)};
        const Solution be_copy = be;
        {
            bool fe_ok = false;
            ref_objective(be, *fe, &fe_ok);
            check(fe_ok, "cap fixture (E = 300 s): base feasible");
            const auto r1 = run(*fe, be, 5, 1), r3 = run(*fe, be, 5, 3), r4 = run(*fe, be, 5, 4), r100 = run(*fe, be, 5, 100), r10k = run(*fe, be, 5, 10000), r0 = run(*fe, be, 5, 0);
            // (3) cap 1: only the first combination of each stale candidate is reached
            check(r1.variants_generated == 3 && r1.evaluated == 4 && r1.cap_reached_candidates == 3 && r1.accepted == 0 && r1.rejected_total() == 4,
                  "(3) cap 1: 3 stale candidates evaluate ONLY their first combination (3 combinations, 4 evaluator calls with the unaffected candidate), cap reached in all 3, nothing feasible");
            bool one_each = true;
            for (const auto& c : r1.log) if (!c.reanchored.empty()) one_each = one_each && c.variants_generated == 1 && c.variants_evaluated == 1 && c.cap_reached && c.combinations_total > 1 && c.reanchored[0].to == fe->flp_at(c.move.kind == MoveKind::N4Exchange && c.move.partner == 1 ? 5 : 1, c.move.kind == MoveKind::N4Exchange && c.move.partner == 1 ? 2 : 5, 0);
            check(one_each, "(3) cap 1: the one combination reached is the FIRST FLP (lowest ratio) of the resulting arc");
            // (6) cap reached before exhaustive enumeration: the search is incomplete, and the result shows it
            check(r3.cap_reached_candidates == 3 && r3.variants_generated == 9 && r3.accepted == 0,
                  "(6) cap 3: the feasible FLPs are the 4th of each arc, so the budget ends before them: 0 accepted although the exhaustive search accepts 2");
            check(r4.cap_reached_candidates == 3 && r4.variants_generated == 12 && r4.accepted == 2 && std::fabs(r4.log[2].objective - 2300) < 1e-9 && std::fabs(r4.log[3].objective - 4300) < 1e-9,
                  "(6) cap 4: the 4th combination is reached: exchange C2 accepted (2300), exchange C1 accepted (4300); cap still reached (7 > 4, 5 > 4)");
            // (4)(5) cap 100 and cap larger than the total
            check(r100.cap_reached_candidates == 0 && r100.variants_generated == 19 && r100.accepted == 2, "(4) cap 100: all 7 + 7 + 5 combinations reached, cap not reached");
            check(r10k.variants_generated == r100.variants_generated && r0.variants_generated == r100.variants_generated && r10k.accepted == 2 && r0.accepted == 2 && r10k.cap_reached_candidates == 0 && r0.cap_reached_candidates == 0,
                  "(5) cap larger than the total (10000) and cap 0 (unlimited) give the identical exhaustive result");
            bool same_best = r100.best && r10k.best && same_solution(r100.best->solution, r10k.best->solution) && r100.best->objective == r10k.best->objective;
            check(same_best, "(5) cap 100 vs cap 10000: identical best candidate on this fixture");
            // exact-fit: cap == number of combinations is not 'reached'
            const auto r7 = run(*fe, be, 5, 7), r5 = run(*fe, be, 5, 5);
            check(r7.cap_reached_candidates == 0 && r5.cap_reached_candidates == 2, "cap equal to the largest candidate's combinations (7): not reached; cap 5: reached in the two 7-combination candidates only");
            check(r5.log[3].variants_generated == 5 && !r5.log[3].cap_reached && r5.log[3].combinations_total == 5, "the 5-combination candidate under cap 5 is complete (cap_reached false, total 5)");
            // each candidate exactly one outcome, cap or not
            bool once = true;
            for (const auto* r : {&r1, &r3, &r4, &r5, &r7, &r100, &r0}) once = once && r->accepted + r->rejected_total() == r->generated;
            check(once, "diagnostics: accepted + rejected == generated for every cap (the cap flag is not an outcome, no double counting)");
            check(rej(r1, RejectReason::EvalEndurance) == 4 && rej(r4, RejectReason::EvalEndurance) == 2, "cap-truncated candidates rejected by the evaluator are reported under the evaluator's reason (eval-endurance), not as a cap rejection");
            // (7) determinism
            const auto r4b = run(*fe, be, 5, 4);
            bool det = r4b.log.size() == r4.log.size();
            for (std::size_t i = 0; det && i < r4.log.size(); ++i)
                det = same_solution(r4.log[i].solution, r4b.log[i].solution) && r4.log[i].objective == r4b.log[i].objective && r4.log[i].variants_generated == r4b.log[i].variants_generated && r4.log[i].cap_reached == r4b.log[i].cap_reached;
            check(det, "(7) a truncated search is deterministic: identical records on a second call");
            // (2) a user-configured cap overrides the default
            N4Options dflt;
            const auto rd = n4_truck_service_node_relocate(be, fe->ctx(3), 5, [&] { N4Options o; o.record_candidates = true; return o; }());
            check(dflt.max_reanchor_variants == 100 && rd.variants_generated == r100.variants_generated && r3.variants_generated != rd.variants_generated,
                  "(2) leaving the option alone behaves as cap 100; setting 3 changes the work done (9 vs 19 combinations)");
            // (10)(11)(14) preserved
            bool preserved = true, once_cov = true;
            for (const auto* r : {&r1, &r3, &r4, &r100})
                for (const auto& c : r->log) {
                    once_cov = once_cov && exactly_once(c.solution, 5);
                    preserved = preserved && c.solution.drone_trips.size() == 1 && c.solution.drone_trips[0].drone_id == 0 && c.solution.drone_trips[0].service_customer == 3 && c.solution.drone_trips[0].recovery_node == 4;
                }
            check(preserved, "(10)(11) under every cap the recovery node (C4), drone id (0) and drone customer (C3) are unchanged");
            check(once_cov, "exactly-once coverage under every cap");
            check(same_solution(be, be_copy), "(14) input unchanged by all the capped runs");
            // (12) the Reject policy ignores the cap entirely
            const auto j1 = run(*fe, be, 5, 1, AffectedTripPolicy::Reject), j100 = run(*fe, be, 5, 100, AffectedTripPolicy::Reject);
            bool rj_same = j1.generated == j100.generated && j1.rejected == j100.rejected && j1.accepted == j100.accepted && j1.variants_generated == 0 && j1.cap_reached_candidates == 0 && j1.log.size() == j100.log.size();
            for (std::size_t i = 0; rj_same && i < j1.log.size(); ++i) rj_same = same_solution(j1.log[i].solution, j100.log[i].solution) && j1.log[i].reason == j100.log[i].reason;
            check(rj_same && rej(j1, RejectReason::AffectedTrips) == 3, "(12) Reject policy: identical results for cap 1 and cap 100, 3 AffectedTrips, no variants, cap never reached");
        }

        // ---- ties: vd = vt (four-way exact tie at 2700 over the first four FLPs of L(C1,C5)); first generated must win under every cap ----
        {
            auto ft = make_line(5, 2, 3600.0, 520.0, 1000.0, 10.0);
            const NodeId ta = ft->flp_at(1, 2, 0);
            Solution b;
            b.truck_route = {0, 1, ta, 2, 4, 5, 0};
            b.drone_trips = {trip(0, ta, 3, 4)};
            bool ok = true, reached_ok = true;
            for (std::size_t cap : {1u, 2u, 4u, 5u, 7u, 100u}) {
                const auto q = run(*ft, b, 5, cap);
                const auto* c = reloc(q, 1);
                ok = ok && c && c->reason == RejectReason::None && c->objective == 2700.0 && c->reanchored.size() == 1 && c->reanchored[0].to == ft->flp_at(1, 5, 0);
                reached_ok = reached_ok && c && (c->cap_reached == (cap < 7));
            }
            check(ok, "(9) four exactly tied replacements: the FIRST generated (x = 1500) is kept for every cap 1, 2, 4, 5, 7, 100 (strict <)");
            check(reached_ok, "cap_reached is true exactly when cap < 7 for the 7-combination candidate");
            const auto q5 = run(*ft, b, 5, 5);
            const auto* c5 = reloc(q5, 1);
            check(c5 && c5->variants_generated == 5 && c5->objective == 2700.0, "cap 5: five combinations reached (2700 x 4, then 2800); the best is still the first 2700");
        }

        // ---- two stale trips (R2 fixture): budget over a 3 x 3 joint space; the order is last stale trip fastest ----
        // Relocate C2 -> gap 0: both trips on L(C1,C3) = {x 1500, 2000, 2500}; the combinations in order are
        // (A,B) = (1500,1500) (1500,2000) (1500,2500) (2000,1500) ...; the optimum is the first, so cap 1 already finds 2000.
        {
            auto f2 = make_line(5, 2);
            const NodeId a2 = f2->flp_at(1, 2, 0), b2 = f2->flp_at(2, 3, 0);
            Solution m;
            m.truck_route = {0, 1, a2, 2, b2, 3, 0};
            m.drone_trips = {trip(0, a2, 4, 3), trip(1, b2, 5, 3)};
            const auto q1 = run(*f2, m, 2, 1), q3 = run(*f2, m, 2, 3), q9 = run(*f2, m, 2, 9);
            const auto *g1 = reloc(q1, 0), *g3 = reloc(q3, 0), *g9 = reloc(q9, 0);
            check(g1 && g3 && g9 && g1->variants_generated == 1 && g3->variants_generated == 3 && g9->variants_generated == 9 && g1->cap_reached && g3->cap_reached && !g9->cap_reached &&
                      g1->combinations_total == 9 && g9->combinations_total == 9,
                  "two stale trips (3 x 3): cap 1 / 3 / 9 reach 1 / 3 / 9 combinations; total reported as 9; reached for 1 and 3 only");
            check(g1->reanchored.size() == 2 && g1->reanchored[0].to == f2->flp_at(1, 3, 0) && g1->reanchored[1].to == f2->flp_at(1, 3, 0) && std::fabs(g1->objective - 2000) < 1e-9 && std::fabs(g9->objective - 2000) < 1e-9,
                  "two stale trips: the first combination (1500,1500) is also the optimum 2000, so cap 1 equals the exhaustive result here");
        }

        // ---- enumeration ORDER (stale trips by index, the LAST varying fastest), made observable by endurance ----
        // E = 260 s. Trip 0 (drone 0) is the LONG trip Fb -> C5 -> C3, trip 1 (drone 1) the short one Fa -> C4 -> C3 (base feasible: 225 s and 175 s).
        // Relocate C2 -> gap 0 gives [0,C2,C1,F,F,C3,0] with BOTH stale launches on L(C1,C3) = {x 1500, 2000, 2500}; truck: C2 200, C1 300, C3 500.
        //   trip 0 (C5): duration = 275 / 250 / 225 s for x = 1500 / 2000 / 2500  -> only x0 >= 2000 satisfies E = 260
        //   trip 1 (C4): duration = 175 / 150 / 125 s                             -> every x1 is feasible
        // Combination order (x0, x1): (1500,1500) (1500,2000) (1500,2500) (2000,1500) ...  A budget of 3 therefore reaches ONLY x0 = 1500: no feasible combination;
        // the 4th combination (2000,1500) is feasible, objective 200 + 300 + 500 + C5 (200 + 200 + 150 = 550) + C4 (200 + 150 + 125 = 475) = 2025.
        {
            auto fo = make_line(5, 2, 260.0);
            const NodeId a2 = fo->flp_at(1, 2, 0), b2 = fo->flp_at(2, 3, 0);
            Solution m;
            m.truck_route = {0, 1, a2, 2, b2, 3, 0};
            m.drone_trips = {trip(0, b2, 5, 3), trip(1, a2, 4, 3)};
            bool fe_ok = false;
            ref_objective(m, *fo, &fe_ok);
            check(fe_ok, "order fixture (E = 260 s): base feasible (long trip from Fb: 225 s, short trip from Fa: 175 s)");
            const NodeId l0 = fo->flp_at(1, 3, 0), l1 = fo->flp_at(1, 3, 1);
            const auto q3 = run(*fo, m, 2, 3), q4 = run(*fo, m, 2, 4), q9 = run(*fo, m, 2, 9), q1 = run(*fo, m, 2, 1);
            const auto *c3 = reloc(q3, 0), *c4 = reloc(q4, 0), *c9 = reloc(q9, 0), *c1 = reloc(q1, 0);
            check(c3 && c3->variants_generated == 3 && c3->cap_reached && c3->reason == RejectReason::EvalEndurance,
                  "(6) budget 3: the first three combinations (1500, x1) all violate endurance: rejected by the evaluator (eval-endurance), cap reached");
            check(c1 && c1->reason == RejectReason::EvalEndurance && c1->variants_generated == 1, "budget 1: only (1500,1500): eval-endurance");
            check(c4 && c4->variants_generated == 4 && c4->cap_reached && c4->reason == RejectReason::None, "(6) budget 4: the 4th combination (2000,1500) is feasible -> accepted; the order is last-stale-trip-fastest");
            check(c4 && c4->reanchored.size() == 2 && c4->reanchored[0].trip == 0 && c4->reanchored[0].to == l1 && c4->reanchored[1].trip == 1 && c4->reanchored[1].to == l0,
                  "budget 4: trip 0 -> second FLP (x = 2000), trip 1 -> first FLP (x = 1500)");
            check(c9 && c9->variants_generated == 9 && !c9->cap_reached && c9->reason == RejectReason::None && c9->reanchored.size() == 2 && c9->reanchored[0].to == l1 && c9->reanchored[1].to == l0,
                  "exhaustive (budget 9): the same combination is the optimum (2000,1500)");
            if (c9) {
                bool ok = false;
                const double o = ref_objective(c9->solution, *fo, &ok);
                check(ok && std::fabs(o - c9->objective) < 1e-9 && std::fabs(c9->objective - 2025) < 1e-9, "the exhaustive optimum is the hand-derived 2025 and agrees with the frozen reference evaluator");
            }
        }
    }

    // =====================================================================
    // 4. N5. Same line, n = 5; base trip A = (drone 0): Fa -> C3 -> C4  (objective 1425).
    //    Drone customer C3, launch gap 1 => h = 3.  Truck customers C1 (s=1), C2 (s=2), C4 (s=3), C5 (s=4).
    // =====================================================================
    {
        auto f = make_line(5, 2);
        const NodeId fa = f->flp_at(1, 2, 0);
        Solution base;
        base.truck_route = {0, 1, fa, 2, 4, 5, 0};
        base.drone_trips = {trip(0, fa, 3, 4)};
        const Solution base_copy = base;
        N5Options opt;
        opt.record_candidates = true;

        auto r = n5_td_service_node_swap(base, f->ctx(1), 0, opt);
        check(r.status == OperatorStatus::Ok && r.op == OperatorId::N5 && r.anchor_trip == 0 && r.anchor_customer == 3, "N5 trip 0 (customer C3): status ok");
        check_near(r.base_objective, 1425, 1e-9, "N5 base objective 1425");
        // partners within w = 1: |2s-3| <= 2 -> C1 (s=1) and C2 (s=2), both at distance 1: C1 first (tie by id).
        // Partner C1: route [0,C3,C2,C4,C5,0] -> gaps {0,1}: L(0,C3) = 5 FLPs (x=500..2500), L(C3,C2) = 1 FLP (x=2500)  -> 6
        // Partner C2: route [0,C1,C3,C4,C5,0] -> gaps {1,2}: L(C1,C3) = 3 FLPs (x=1500,2000,2500), L(C3,C4) = 1 FLP (x=3500) -> 4
        check(r.generated == 10 && r.evaluated == 10 && r.accepted == 10 && r.rejected_total() == 0, "N5 w=1: 6 + 4 = 10 candidates, all feasible");
        check(r.log.size() == 10, "N5 w=1: all 10 candidates logged");
        std::vector<NodeId> partner_order;
        for (const auto& c : r.log) if (partner_order.empty() || partner_order.back() != c.move.customer) partner_order.push_back(c.move.customer);
        check(partner_order == std::vector<NodeId>({1, 2}), "N5 w=1: partners C1 then C2 (nearest first, tie by id); C4 (distance 3 half-steps) is outside the window");
        const std::vector<double> expect = {2075, 2100, 2175, 2400, 2700, 2575, 1475, 1500, 1625, 1850};
        bool values_ok = r.log.size() == expect.size();
        for (std::size_t i = 0; values_ok && i < expect.size(); ++i) values_ok = std::fabs(r.log[i].objective - expect[i]) < 1e-9;
        check(values_ok, "N5 w=1: candidate objectives 2075,2100,2175,2400,2700,2575 (partner C1) and 1475,1500,1625,1850 (partner C2) as derived by hand");
        bool ref_ok = true;
        for (const auto& c : r.log) { bool fe = false; const double o = ref_objective(c.solution, *f, &fe); ref_ok = ref_ok && fe && std::fabs(o - c.objective) < 1e-9; }
        check(ref_ok, "N5 w=1: every candidate is also feasible with the same objective under the frozen reference evaluator");
        check(r.best && std::fabs(r.best->objective - 1475) < 1e-9, "N5 w=1: best = 1475");
        check(r.best->move.customer == 2 && r.best->move.partner == 3 && r.best->move.replaced_trip == 0, "best: C2 becomes drone-served, C3 moves to the truck, trip 0 replaced");
        const NodeId f1500 = f->flp_at(1, 3, 0);
        check(r.best->move.launch == f1500 && r.best->move.recovery == 3 && r.best->move.drone_id == 0, "best trip: launch = first FLP of L(C1,C3) (x=1500), recovery = C3 (the next truck node), drone 0");
        check(r.best->solution.truck_route == TruckRoute({0, 1, f1500, 3, 4, 5, 0}), "best route [0,C1,F1500,C3,C4,C5,0]: Fa dropped, new FLP inserted in its gap, C3 in C2's slot");
        check(r.best->solution.drone_trips.size() == 1 && r.best->solution.drone_trips[0].launch_node == f1500 && r.best->solution.drone_trips[0].service_customer == 2 &&
                  r.best->solution.drone_trips[0].recovery_node == 3 && r.best->solution.drone_trips[0].drone_id == 0,
              "best trip is (drone 0, F1500, C2, C3)");
        check(r.best->flps_dropped == std::vector<NodeId>({fa}), "the dissolved trip's launch FLP Fa (used by no other trip) is reported as dropped");
        check(!r.improves(), "1475 > 1425: not an improvement");
        check(same_solution(base, base_copy), "N5 does not modify its input");

        // every candidate: recovery = next truck node after the launch FLP; exactly-once; FLP on its own arc
        bool next_node = true, once = true, arcs = true;
        for (const auto& c : r.log) {
            const Skeleton sk = make_skeleton(c.solution.truck_route, f->nodes);
            const auto& nt = c.solution.drone_trips.back();
            std::size_t lp = 0;
            for (std::size_t p = 0; p < c.solution.truck_route.size(); ++p) if (c.solution.truck_route[p] == nt.launch_node) lp = p;
            NodeId nxt = kInvalidNodeId;
            for (std::size_t p = lp + 1; p < c.solution.truck_route.size(); ++p) if (f->nodes.at(c.solution.truck_route[p]).type != NodeType::FlexibleLaunchPoint) { nxt = c.solution.truck_route[p]; break; }
            next_node = next_node && nxt == nt.recovery_node;
            once = once && exactly_once(c.solution, 5);
            arcs = arcs && !has_foreign_flp(c.solution.truck_route, *f);
            (void)sk;
        }
        check(next_node, "N5: every new trip recovers at the NEXT TRUCK NODE after its launch FLP (D4)");
        check(once, "N5: every candidate serves each of the 5 customers exactly once");
        check(arcs, "N5: no candidate leaves an FLP on a foreign arc");

        // w = 2 enlarges partners (adds C4) and the launch windows: 9 + 6 + 13 = 28 candidates (derived in the comments of the test plan)
        auto r2 = n5_td_service_node_swap(base, f->ctx(2), 0, opt);
        std::size_t by_partner[6] = {0, 0, 0, 0, 0, 0};
        for (const auto& c : r2.log) ++by_partner[static_cast<std::size_t>(c.move.customer)];
        check(r2.generated == 28 && by_partner[1] == 9 && by_partner[2] == 6 && by_partner[4] == 13, "N5 w=2: partner C1 -> 9, C2 -> 6, C4 -> 13 candidates (28 in total)");
        bool depot_recovery = false;
        for (const auto& c : r2.log) depot_recovery = depot_recovery || (c.move.customer == 4 && c.move.recovery == 0);
        check(depot_recovery, "N5 w=2: arcs into the END depot are launch arcs (recovery at the depot, depot boundary)");
        bool depot_start = false;
        for (const auto& c : r.log) depot_start = depot_start || (c.move.customer == 1 && f->flp.at(c.move.launch).arc_from == 0);
        check(depot_start, "N5 w=1, partner C1: arc (depot, C3) is a launch arc (start-depot boundary)");

        // evaluate_candidates = false
        N5Options dry = opt;
        dry.evaluate_candidates = false;
        const auto d = n5_td_service_node_swap(base, f->ctx(1), 0, dry);
        check(d.generated == 10 && d.evaluated == 0 && d.accepted == 0 && !d.has_candidate(), "N5 evaluate_candidates=false: 10 constructed, none evaluated, none accepted");

        // invalid anchor
        check(n5_td_service_node_swap(base, f->ctx(1), 7, opt).status == OperatorStatus::InvalidAnchor, "N5 trip index out of range -> InvalidAnchor");
    }

    // N5: recovery of the base trip at C2 and a short endurance, so that long new flights are rejected.
    //    Base: Fa -> C3 -> C2 (Fa at x=1500): launch 150, C3 at 225, back at C2 at 275, truck at C2 at 200 -> waits.
    //    endurance needed: max(E36,E37) = 125.  e = 140 keeps the base feasible.
    {
        auto f = make_line(5, 2, 140.0);
        const NodeId fa = f->flp_at(1, 2, 0);
        Solution base;
        base.truck_route = {0, 1, fa, 2, 4, 5, 0};
        base.drone_trips = {trip(0, fa, 3, 2)};
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 1575, 1e-9, "endurance fixture: base objective 100+200+475+575+225 = 1575");
        check(feas, "endurance fixture: base feasible with e = 140 (needs 125)");
        N5Options opt;
        opt.record_candidates = true;
        const auto r = n5_td_service_node_swap(base, f->ctx(1), 0, opt);
        // hand endurance values: partner C2 -> 150,100,75,175 ; partner C1 -> 250,200,150,150,175,125 ; e = 140
        check(r.generated == 10 && r.accepted == 3 && r.rejected[static_cast<std::size_t>(RejectReason::EvalEndurance)] == 7 && r.rejected_total() == 7,
              "N5 e=140: 3 candidates accepted (E=100,75,125), 7 rejected by endurance (E=150,175,250,200,150,150,175)");
        std::vector<RejectReason> reasons;
        for (const auto& c : r.log) reasons.push_back(c.reason);
        const std::vector<RejectReason> E = RejectReason::EvalEndurance == RejectReason::None ? reasons : std::vector<RejectReason>{
            RejectReason::EvalEndurance, RejectReason::EvalEndurance, RejectReason::EvalEndurance, RejectReason::EvalEndurance, RejectReason::EvalEndurance, RejectReason::None,
            RejectReason::EvalEndurance, RejectReason::None, RejectReason::None, RejectReason::EvalEndurance};
        check(reasons == E, "N5 e=140: per-candidate verdicts in generation order (hand-derived pattern)");
        check(r.best && std::fabs(r.best->objective - 1500) < 1e-9, "N5 e=140: best feasible = 1500 (partner C2, FLP x=2000)");
        bool rejected_infeasible_in_ref = true;
        for (const auto& c : r.log) { if (c.reason == RejectReason::EvalEndurance) { bool fe = true; ref_objective(c.solution, *f, &fe); rejected_infeasible_in_ref = rejected_infeasible_in_ref && !fe; } }
        check(rejected_infeasible_in_ref, "every endurance rejection is also infeasible under the frozen reference evaluator");
    }

    // =====================================================================
    // 5. Fleet capacity (|U| = 1) and reuse of a launch FLP already on the route.
    //    n = 5; drone customers C4 (trip A) and C5 (trip B); route [0, Fa0, C1, C2, C3, Fb, 0].
    //    A = (drone 0): Fa0 -> C4 -> C3, interval [Fa0 .. C3];  B = (drone 0): Fb -> C5 -> depot, interval [Fb .. 0].
    //    The two intervals are disjoint, so one drone suffices.
    // =====================================================================
    {
        auto f = make_line(5, 1);
        const NodeId fa0 = f->flp_at(0, 1, 0);  // x = 500
        const NodeId fb = f->flp_at(3, 0, 0);   // L(C3,0): 5 FLPs, ratio 1/6 -> x = 2500
        check(std::fabs(f->flp_x(fa0) - 500) < 1e-9 && std::fabs(f->flp_x(fb) - 2500) < 1e-9, "fixture: Fa0 at x=500, Fb at x=2500");
        Solution base;
        base.truck_route = {0, fa0, 1, 2, 3, fb, 0};
        base.drone_trips = {trip(0, fa0, 4, 3), trip(0, fb, 5, 0)};
        bool feas = false;
        ref_objective(base, *f, &feas);
        check(feas, "fleet fixture: base feasible with a single drone (trips are sequential)");

        N5Options opt;
        opt.record_candidates = true;
        const auto r = n5_td_service_node_swap(base, f->ctx(2), 1, opt);
        // trip B (customer C5, launch gap 3 => h=7): partners with |2s-7| <= 4 -> C3 (s=3), C2 (s=2).
        //  C3: trip A recovers at C3 -> affected.  C2: unaffected; new-trip gaps {0,1,2,3}:
        //  gap0 L(0,C1)=1 (Fa0: reuse) + gap1 L(C1,C5)=6 + gap2 L(C5,C3)=3 -> all overlap A's interval -> NoFreeDrone (10);
        //  gap3 L(C3,0)=5 launches after A has been recovered -> evaluated.
        check(r.generated == 16 && r.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)] == 1 && r.rejected[static_cast<std::size_t>(RejectReason::NoFreeDrone)] == 10 && r.evaluated == 5,
              "N5 |U|=1: 1 affected swap (C3: trip A recovers there), 10 NoFreeDrone (new trips inside A's interval), 5 evaluated");
        bool reuse_seen = false, nofree_ok = true;
        for (const auto& c : r.log) {
            if (c.reason == RejectReason::NoFreeDrone) {
                reuse_seen = reuse_seen || c.move.launch == fa0;
                nofree_ok = nofree_ok && !c.evaluated;
            }
        }
        check(reuse_seen, "N5: reusing a launch FLP that is already on the route (Fa0, a second launch there) is generated, and rejected only because no drone is free");
        check(nofree_ok, "NoFreeDrone candidates are never sent to the evaluator");
        std::size_t affected_idx_ok = 0;
        for (const auto& c : r.log) if (c.reason == RejectReason::AffectedTrips && c.affected_trips == std::vector<std::size_t>({0})) ++affected_idx_ok;
        check(affected_idx_ok == 1, "the affected swap names trip A (index 0) as the trip whose recovery node leaves the truck route");
        // Fb is re-selected (dropped with the dissolved trip, then inserted again): at least one accepted-or-evaluated candidate launches from Fb
        bool fb_back = false;
        for (const auto& c : r.log) fb_back = fb_back || (c.evaluated && c.move.launch == fb);
        check(fb_back, "the dissolved trip's own launch FLP (Fb) can be re-selected for the new trip");
        // with two drones the same swap has no NoFreeDrone rejection at all for the gap-3 family and fewer overall
        auto f2 = make_line(5, 2);
        Solution base2 = base;
        base2.drone_trips = {trip(0, fa0, 4, 3), trip(0, fb, 5, 0)};
        const auto r2 = n5_td_service_node_swap(base2, f2->ctx(2), 1, opt);
        check(r2.rejected[static_cast<std::size_t>(RejectReason::NoFreeDrone)] == 0, "with |U| = 2 the overlapping new trips find the second drone: no NoFreeDrone rejections");
        bool second_drone_used = false;
        for (const auto& c : r2.log) second_drone_used = second_drone_used || (c.evaluated && c.move.drone_id == 1);
        check(second_drone_used, "with |U| = 2 some new trip is flown by drone 1 (lowest FREE id), existing trips keep their ids");
        bool ids_kept = true;
        for (const auto& c : r2.log) if (c.evaluated) ids_kept = ids_kept && c.solution.drone_trips.front().drone_id == 0;
        check(ids_kept, "the unaffected trip A keeps drone id 0 in every candidate (no silent reassignment)");
    }

    // short route N5
    {
        auto f = make_line(2, 2);
        const NodeId fa = f->flp_at(0, 1, 0);
        Solution base;
        base.truck_route = {0, fa, 1, 0};
        base.drone_trips = {trip(0, fa, 2, 1)};
        bool feas = false;
        ref_objective(base, *f, &feas);
        check(feas, "short-route fixture feasible");
        N5Options opt;
        opt.record_candidates = true;
        const auto r = n5_td_service_node_swap(base, f->ctx(3), 0, opt);
        // only truck customer C1 (s=1). d=C2 takes its slot: route [0,C2,0]; gaps (0,C2): 3 FLPs, (C2,0): 3 FLPs -> 6 candidates.
        check(r.generated == 6 && r.log.size() == 6, "N5 one truck customer: swap with C1 gives route [0,C2,0] and 3 + 3 launch candidates");
        bool routes_ok = true;
        for (const auto& c : r.log) {
            std::vector<NodeId> core;
            for (NodeId v : c.solution.truck_route) if (f->nodes.at(v).type != NodeType::FlexibleLaunchPoint) core.push_back(v);
            routes_ok = routes_ok && core == std::vector<NodeId>({0, 2, 0});
        }
        check(routes_ok, "every short-route candidate has the skeleton [0,C2,0]");
    }

    // =====================================================================
    // 5b. Ties: strict '<', the FIRST generated candidate wins (deterministic, documented).
    // =====================================================================
    {
        // N4: n = 3 on the line. Relocating C1 into gap 2 gives [0,C2,C1,C3,0] (200+300+500 = 1000) and into gap 3
        // gives [0,C2,C3,C1,0] (200+300+500 = 1000): an EXACT tie (all arrival times are integers).
        auto f = make_line(3, 2);
        Solution base;
        base.truck_route = {0, 1, 2, 3, 0};
        N4Options opt;
        opt.record_candidates = true;
        const auto r = n4_truck_service_node_relocate(base, f->ctx(3), 1, opt);
        std::size_t ties = 0;
        for (const auto& c : r.log) if (c.reason == RejectReason::None && c.objective == 1000.0) ++ties;
        check(ties == 2 && r.best && r.best->objective == 1000.0, "N4 tie fixture: two distinct candidates share the optimum 1000 exactly");
        check(r.best && r.best->move.kind == MoveKind::N4Relocate && r.best->move.gap == 2 && r.best->solution.truck_route == TruckRoute({0, 2, 1, 3, 0}),
              "N4 tie: the FIRST generated candidate (gap 2) is kept, not the later equal one (gap 3)");
    }
    {
        // N5: n = 4; base route [0,F,C1,C3,C4,0] with F on arc (0,C1) at x = 500, trip F -> C2 -> C1 (objective 1075).
        // Swap with C1: C2 joins the truck, C1 is launched at x = 500 on arc (0,C2): 200+300+400 + (50+25) = 975.
        // Swap with C3: C3 is launched at x = 2500 on arc (C2,C4): 100+200+400 + (250+25) = 975.  Mirror symmetry => exact tie.
        auto f = make_line(4, 2);
        const NodeId fl = f->flp_at(0, 1, 0);
        Solution base;
        base.truck_route = {0, fl, 1, 3, 4, 0};
        base.drone_trips = {trip(0, fl, 2, 1)};
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 1075, 1e-9, "N5 tie fixture: base objective 1075 as derived by hand");
        N5Options opt;
        opt.record_candidates = true;
        const auto r = n5_td_service_node_swap(base, f->ctx(2), 0, opt);
        std::size_t ties = 0;
        for (const auto& c : r.log) if (c.reason == RejectReason::None && c.objective == 975.0) ++ties;
        check(ties == 2 && r.best && r.best->objective == 975.0, "N5 tie fixture: two distinct candidates share the optimum 975 exactly");
        check(r.best && r.best->move.customer == 1 && r.best->move.launch == f->flp_at(0, 2, 0) && r.best->move.recovery == 2,
              "N5 tie: the FIRST generated candidate (partner C1, nearest first) is kept, not the later equal one (partner C3)");
        bool second = false;
        for (const auto& c : r.log) second = second || (c.reason == RejectReason::None && c.move.customer == 3 && c.objective == 975.0 && c.move.launch == f->flp_at(2, 4, 0));
        check(second, "N5 tie: the later equal candidate (C3 launched at x = 2500) exists and was not preferred");
    }

    // =====================================================================
    // 5c. classify_evaluation, fed real evaluator results (the operators pre-empt fleet violations with NoFreeDrone,
    //     so the evaluator-level branches are pinned here directly).
    //     Route [0,C1,Fa,C2,Fb,C3,0]; trips A: Fa->C4->C3 and B: Fb->C5->C3 overlap on [Fb, C3].
    // =====================================================================
    {
        const auto classify = [](int U, int id_a, int id_b, bool swap_recoveries, bool* feasible) {
            auto f = make_line(5, U);
            const NodeId fa = f->flp_at(1, 2, 0), fb = f->flp_at(2, 3, 0);
            Solution s;
            s.truck_route = {0, 1, fa, 2, fb, 3, 0};
            s.drone_trips = {trip(id_a, fa, 4, 3), trip(id_b, fb, 5, 3)};
            if (swap_recoveries) s.drone_trips[0].recovery_node = 1; // recovery C1 precedes the launch Fa
            const auto e = tdr::timing::evaluate_solution(s, *f->net, f->cd, f->flp);
            if (feasible) *feasible = e.feasible;
            return classify_evaluation(e, U).reason;
        };
        bool feas = true;
        check(classify(2, 0, 1, false, &feas) == RejectReason::None && feas, "classify: two overlapping trips on two different drones, |U| = 2 -> feasible");
        check(classify(2, 0, 0, false, &feas) == RejectReason::EvalDroneSequencing && !feas, "classify: overlapping trips on the SAME drone id, |U| = 2 -> eval-drone-sequencing (constraint (28))");
        check(classify(1, 0, 0, false, &feas) == RejectReason::EvalFleetCapacity && !feas, "classify: two trips in flight at once with |U| = 1 -> eval-fleet-capacity");
        check(classify(2, 0, 1, true, &feas) == RejectReason::EvalTripOrder && !feas, "classify: recovery before launch -> eval-trip-order (root cause), not the rule-(6) cascade");
    }

    // =====================================================================
    // 6. Randomised entry points: deterministic under a fixed seed, honest about "nothing to do".
    // =====================================================================
    {
        auto f = make_line(6, 2);
        const NodeId fa = f->flp_at(1, 2, 0);
        Solution base;
        base.truck_route = {0, 1, fa, 2, 4, 5, 6, 0};
        base.drone_trips = {trip(0, fa, 3, 4)};
        bool feas = false;
        ref_objective(base, *f, &feas);
        check(feas, "random fixture feasible");
        std::set<NodeId> anchors4;
        std::set<std::size_t> dummy;
        bool det4 = true;
        for (std::uint64_t seed = 1; seed <= 24; ++seed) {
            tdr::util::Rng a(seed), b(seed);
            const auto r1 = n4_truck_service_node_relocate(base, f->ctx(3), a);
            const auto r2 = n4_truck_service_node_relocate(base, f->ctx(3), b);
            det4 = det4 && r1.anchor_customer == r2.anchor_customer && r1.generated == r2.generated && r1.accepted == r2.accepted &&
                   r1.has_candidate() == r2.has_candidate() && (!r1.has_candidate() || same_solution(r1.best->solution, r2.best->solution));
            anchors4.insert(r1.anchor_customer);
        }
        check(det4, "N4 random anchor: identical results for equal seeds (24 seeds)");
        check(anchors4.size() >= 3, "N4 random anchor: different seeds pick different truck customers (>= 3 distinct of 4 in 24 seeds)");
        bool det5 = true;
        for (std::uint64_t seed = 1; seed <= 10; ++seed) {
            tdr::util::Rng a(seed), b(seed);
            const auto r1 = n5_td_service_node_swap(base, f->ctx(3), a);
            const auto r2 = n5_td_service_node_swap(base, f->ctx(3), b);
            det5 = det5 && r1.anchor_trip == r2.anchor_trip && r1.generated == r2.generated && r1.has_candidate() == r2.has_candidate() &&
                   (!r1.has_candidate() || same_solution(r1.best->solution, r2.best->solution));
        }
        check(det5, "N5 random trip: identical results for equal seeds");

        Solution all_truck;
        all_truck.truck_route = {0, 1, 2, 3, 4, 5, 6, 0};
        tdr::util::Rng rr(5);
        const auto none = n5_td_service_node_swap(all_truck, f->ctx(3), rr);
        check(none.status == OperatorStatus::NoEligibleAnchor && !none.has_candidate() && none.generated == 0, "N5 random on a trip-free solution -> NoEligibleAnchor");
        auto empty = make_line(0, 1);
        Solution depot_only;
        depot_only.truck_route = {0, 0};
        tdr::util::Rng r0(5);
        const auto none4 = n4_truck_service_node_relocate(depot_only, empty->ctx(3), r0);
        check(none4.status == OperatorStatus::NoEligibleAnchor && !none4.has_candidate(), "N4 random with no truck customer -> NoEligibleAnchor");
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
