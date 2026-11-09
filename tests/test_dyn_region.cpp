// DM7 region builder: construction, closure (extend / freeze), drone-trip straddling, empty/full regions, impact vs optimisation region,
// protected-element checks, V1-V4 property tests on random plans, closure idempotence, determinism, P2 tail shift through the evaluator.
// No optimiser is involved (DM8/DM9 are out of scope).
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <vector>

#include "dynamic/region_builder.hpp"
#include "dynamic/runner.hpp"

using namespace tdr;
using namespace tdr::dynamic;
using tdr::model::DroneTrip;
using tdr::model::Node;
using tdr::model::NodeType;
using tdr::model::Solution;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

namespace {

// Registry: depot 0; customers 1..7 on a line; drone customers 20, 21, 22; FLPs 1000..1003.
model::NodeRegistry registry() {
    model::NodeRegistry n;
    n.add(Node{0, NodeType::Depot, {0, 0}});
    for (int i = 1; i <= 7; ++i) n.add(Node{i, NodeType::Customer, {1000.0 * i, 0}});
    for (int i : {20, 21, 22}) n.add(Node{i, NodeType::Customer, {1000.0 * (i - 19), 500}});
    for (int f = 1000; f <= 1003; ++f) n.add(Node{f, NodeType::FlexibleLaunchPoint, {1000.0 * (f - 999), 100}});
    return n;
}

// route 0 c1 c2 F1000 c3 c4 F1001 c5 c6 c7 0 ; skeleton 0..8 = 0,1,2,3,4,5,6,7,depot
// trip A: (F1000 -> 20 -> c4)  launch gap 2, recovery skel 4 ; trip B: (F1001 -> 21 -> c6) launch gap 4, recovery skel 6
Solution base_plan() {
    Solution s;
    s.truck_route = {0, 1, 2, 1000, 3, 4, 1001, 5, 6, 7, 0};
    s.drone_trips = {DroneTrip{0, 1000, 20, 4}, DroneTrip{1, 1001, 21, 6}};
    return s;
}

Region build(const RegionBuilder& b, const Solution& s, std::size_t cut, NodeId r, const model::NodeRegistry& n) {
    const PlanLayout L = layout_of(s, n);
    return b.build(RegionContext{L, n, cut, r});
}
bool clean(const Region& R, const Solution& s, const model::NodeRegistry& n) { return validate_region(R, s, n).empty(); }

void test_layout() {
    const auto n = registry(); const Solution s = base_plan();
    const PlanLayout L = layout_of(s, n);
    CHECK(L.ok && L.sk.size() == 9 && L.last() == 8);
    CHECK(L.trips.size() == 2 && L.trips[0].launch_gap == 2 && L.trips[0].recovery_skel == 4 && L.trips[1].launch_gap == 4 && L.trips[1].recovery_skel == 6);
    Solution bad = s; bad.drone_trips[0].launch_node = 3; // launch is not an FLP entry
    CHECK(!layout_of(bad, n).ok);
    Solution bad2 = s; bad2.drone_trips[0].recovery_node = 2; // recovery before the launch
    CHECK(!layout_of(bad2, n).ok);
    Solution tail = s; tail.drone_trips[1].recovery_node = 0; // recovery at the end depot
    const PlanLayout Lt = layout_of(tail, n);
    CHECK(Lt.ok && Lt.trips[1].recovery_skel == 8);
}

void test_basic_and_boundaries() {
    const auto n = registry(); const Solution s = base_plan();
    RegionR3 r3; r3.w = 0;
    // r = c3 (skel 3): core [2,4]; trip A launches in gap 2 and is recovered at 4 -> already closed, no extension
    Region R = build(r3, s, 0, 3, n);
    CHECK(R.valid && !R.empty && R.lo == 2 && R.hi == 4 && !R.closure_extended && R.frozen_trips.empty());
    CHECK(R.free_trips == std::vector<std::size_t>{0});
    CHECK((R.free_customers == std::vector<NodeId>{3, 20}));
    CHECK((R.pinned_nodes == std::vector<NodeId>{2, 4}));
    CHECK(clean(R, s, n));
    // margin: w = 1 widens by one skeleton node on each side
    r3.w = 1; R = build(r3, s, 0, 3, n);
    CHECK(R.lo == 1 && R.hi == 6 && R.closure_extended && clean(R, s, n)); // margin [1,5]; trip B (launch gap 4, recovery 6) extends hi
    // clipping at the cut (V1) and at the end depot
    r3.w = 5; R = build(r3, s, 2, 3, n);
    CHECK(R.lo == 2 && R.hi == 8 && clean(R, s, n));   // lo clipped at cut 2, hi at the end depot
    R = build(r3, s, 0, 3, n);
    CHECK(R.lo == 0 && R.hi == 8 && clean(R, s, n));
    // cut exactly at r-1 is allowed (a = B)
    r3.w = 0; R = build(r3, s, 2, 3, n);
    CHECK(R.valid && R.lo == 2);
    // r at or before the cut, r absent, r served by a drone: invalid with a reason, never a guess
    CHECK(!build(r3, s, 3, 3, n).valid);
    CHECK(!build(r3, s, 5, 3, n).valid);
    CHECK(!build(r3, s, 0, 99, n).valid);
    CHECK(!build(r3, s, 0, 20, n).valid);
    CHECK(!build(r3, s, 8, 3, n).valid); // cut leaves no editable gap
    // r next to the end depot: r = c7 (skel 7), core [6,8]
    R = build(r3, s, 0, 7, n);
    CHECK(R.valid && R.hi == 8 && clean(R, s, n));
}

void test_closure_extension() {
    const auto n = registry(); const Solution s = base_plan();
    RegionR3 r3; r3.w = 0;
    // r = c4 (skel 4): core [3,5]. Trip A (launch gap 2, recovery 4) is launched left of lo and recovered inside -> extend lo to 2.
    // Trip B launches in gap 4 (inside) and is recovered at 6 -> extend hi to 6. Fixpoint [2,6].
    Region R = build(r3, s, 0, 4, n);
    CHECK(R.valid && R.lo == 2 && R.hi == 6 && R.closure_extended && R.frozen_trips.empty() && R.core_lo == 3 && R.core_hi == 5);
    CHECK((R.free_trips == std::vector<std::size_t>{0, 1}));
    CHECK((R.free_customers == std::vector<NodeId>{3, 4, 5, 20, 21}));
    CHECK(clean(R, s, n));
    // the closed region is never spanned at the boundaries: no non-frozen trip crosses lo or hi
    const PlanLayout L = layout_of(s, n);
    for (const auto& t : L.trips) { CHECK(!(t.launch_gap < R.lo && t.recovery_skel > R.lo)); CHECK(!(t.launch_gap < R.hi && t.recovery_skel > R.hi && t.launch_gap >= R.lo)); }
    // chained closure: trip B recovered at the end depot pulls hi to the end
    Solution c = s; c.drone_trips[1].recovery_node = 0;
    R = build(r3, c, 0, 4, n);
    CHECK(R.valid && R.hi == 8 && clean(R, c, n));
    // R5 full tail
    RegionR5 r5; R = build(r5, s, 0, 4, n);
    CHECK(R.valid && R.lo == 0 && R.hi == 8 && R.free_trips.size() == 2 && clean(R, s, n));
    CHECK((R.free_customers == std::vector<NodeId>{1, 2, 3, 4, 5, 6, 7, 20, 21}));
    R = build(r5, s, 2, 4, n);
    CHECK(R.lo == 2 && R.free_trips.size() == 2 && clean(R, s, n));
}

void test_freeze_with_non_clean_cut() {
    // A clean cut never leaves a trip spanning it, so freezing cannot occur. Here the cut (skel 3) is DELIBERATELY non-clean: trip A
    // (launch gap 2) is airborne across it. The region may not reach behind the cut, so the trip is FROZEN (kept, recovery pinned), never cut through.
    const auto n = registry(); const Solution s = base_plan();
    RegionR3 r3; r3.w = 0;
    Region R = build(r3, s, 3, 4, n);
    CHECK(R.valid && (R.frozen_trips == std::vector<std::size_t>{0}));
    CHECK(R.lo >= 3);
    CHECK(R.boundary_spanned_by_frozen);                                         // explicit flag, never silent
    CHECK(!R.is_free(20));                                                       // its customer is not free
    CHECK(std::find(R.pinned_nodes.begin(), R.pinned_nodes.end(), 4) != R.pinned_nodes.end());
    CHECK(clean(R, s, n));
    // the free trip set excludes the frozen trip
    CHECK(std::find(R.free_trips.begin(), R.free_trips.end(), 0) == R.free_trips.end());
}

void test_r0_r1_r4_and_empty() {
    const auto n = registry(); const Solution s = base_plan();
    RegionR0 r0; Region R = build(r0, s, 0, 4, n);
    CHECK(R.valid && R.empty && R.free_customers.empty() && R.free_trips.empty() && clean(R, s, n));
    // preservation for the empty region: nothing may change
    CHECK(check_region_preserved(s, s, R, n).empty());
    Solution t = s; std::swap(t.truck_route[1], t.truck_route[2]);
    CHECK(!check_region_preserved(s, t, R, n).empty());
    Solution u = s; u.drone_trips[0].service_customer = 22;
    CHECK(!check_region_preserved(s, u, R, n).empty());
    // R4: fixed interval, same closure
    RegionR4 r4; r4.h = 1; R = build(r4, s, 0, 4, n);
    CHECK(R.valid && R.lo <= 2 && R.hi >= 6 && clean(R, s, n));
    // R1: k nearest editable customers (comparator); k = 0 -> the insertion gaps only (+ closure)
    RegionR1 r1; r1.k = 0; R = build(r1, s, 0, 4, n);
    CHECK(R.valid && clean(R, s, n) && R.lo <= 3 && R.hi >= 5);
    r1.k = 3; const Region R1b = build(r1, s, 0, 4, n);
    CHECK(R1b.valid && clean(R1b, s, n) && R1b.free_customers.size() >= R.free_customers.size());
}

void test_impact_vs_region() {
    const auto n = registry(); const Solution s = base_plan();
    const PlanLayout L = layout_of(s, n);
    // r = c3 (skel 3): impact = r's old gap (2,4) -> everything from skeleton 4 on, plus drone customers launched at/after it
    ImpactSet I = impact_of(L, n, 3);
    CHECK(I.valid && I.node_a == 2 && I.node_b == 4);
    CHECK((I.shifted_customers == std::vector<NodeId>{4, 5, 6, 7, 21}));
    CHECK(I.shifted_trips == std::vector<std::size_t>{1});
    // trip A: launched at gap 2 (before node 4), recovered at node 4 -> its recovery is AT b: recovery_pos >= pb -> straddling the insertion
    CHECK(I.straddling_trips == std::vector<std::size_t>{0});
    CHECK(I.straddling_customers == std::vector<NodeId>{20});
    RegionR3 r3; r3.w = 0;
    const Region R = build(r3, s, 0, 3, n);
    const RegionReport rep = describe(R, I);
    // impact is a suffix; the region is bounded: the tail behind hi is impacted but protected; c3 and trip A's customer are free but not shifted
    CHECK(rep.impact_customers == 5 && rep.free_customers == 2);
    CHECK(rep.impacted_not_free == 5 && rep.free_and_impacted == 0 && rep.free_not_impacted == 2);
    // the two sets are different objects: a wider region covers more of the impact set but the impact set does not change
    RegionR5 r5; const Region F = build(r5, s, 0, 3, n);
    const RegionReport repF = describe(F, I);
    CHECK(repF.impacted_not_free == 0 && repF.free_and_impacted == 5);
    CHECK(impact_of(L, n, 3).shifted_customers == I.shifted_customers); // independent of any region setting
    // invalid requests
    CHECK(!impact_of(L, n, 0).valid && !impact_of(L, n, 20).valid && !impact_of(L, n, 99).valid);
}

void test_preservation() {
    const auto n = registry(); const Solution s = base_plan();
    RegionR3 r3; r3.w = 0; const Region R = build(r3, s, 0, 4, n); // [2,6] free {3,4,5,20,21}
    CHECK(check_region_preserved(s, s, R, n).empty());
    // permuting the free interior (positions strictly between pos(lo) and pos(hi)) is allowed
    Solution ok = s; std::swap(ok.truck_route[4], ok.truck_route[5]); // c3 <-> c4 inside the region (FLP 1000 stays at position 3, trips untouched)
    CHECK(check_region_preserved(s, ok, R, n).empty());
    // moving a protected customer into the region
    Solution bad = s; std::swap(bad.truck_route[2], bad.truck_route[4]); // c2 (the boundary lo) <-> c3
    CHECK(!check_region_preserved(s, bad, R, n).empty());
    // moving the boundary
    Solution bad2 = s; std::swap(bad2.truck_route[8], bad2.truck_route[9]); // c6 (hi) <-> c7
    CHECK(!check_region_preserved(s, bad2, R, n).empty());
    // a protected trip changed
    RegionR3 narrow; narrow.w = 0; const Region Rn = build(narrow, s, 0, 3, n); // [2,4]: trip B is protected
    Solution bt = s; bt.drone_trips[1].service_customer = 22;
    CHECK(!check_region_preserved(s, bt, Rn, n).empty());
    // coverage lost
    Solution cov = s; cov.drone_trips.pop_back();
    CHECK(!check_region_preserved(s, cov, Rn, n).empty());
    // prefix changed
    Solution pre = s; std::swap(pre.truck_route[1], pre.truck_route[2]);
    CHECK(!check_region_preserved(s, pre, Rn, n).empty());
}

// random plans: route of m customers (line), t random trips (distinct FLPs placed in the launch gap, recovery a few nodes later)
struct RandPlan { model::NodeRegistry nodes; Solution sol; std::size_t m = 0; };
RandPlan random_plan(std::mt19937_64& g) {
    RandPlan P; P.m = 5 + g() % 10;
    P.nodes.add(Node{0, NodeType::Depot, {0, 0}});
    for (std::size_t i = 1; i <= P.m; ++i) P.nodes.add(Node{static_cast<NodeId>(i), NodeType::Customer, {1000.0 * i, 0}});
    const std::size_t T = g() % 4;
    struct Tr { std::size_t gap, rec; NodeId flp, cust; };
    std::vector<Tr> trs;
    for (std::size_t t = 0; t < T; ++t) {
        const std::size_t gap = g() % (P.m); // gap g between skeleton nodes g and g+1, g in 0..m
        const std::size_t rec = std::min(P.m + 1, gap + 1 + g() % 3);
        const NodeId flp = static_cast<NodeId>(1000 + t), cust = static_cast<NodeId>(100 + t);
        P.nodes.add(Node{flp, NodeType::FlexibleLaunchPoint, {0, 50}}); P.nodes.add(Node{cust, NodeType::Customer, {0, 500}});
        trs.push_back({gap, rec, flp, cust});
    }
    // route: depot, customers 1..m with FLPs inserted after skeleton node `gap`, depot
    P.sol.truck_route.push_back(0);
    for (std::size_t s = 1; s <= P.m + 1; ++s) {
        for (const auto& t : trs) if (t.gap + 1 == s) P.sol.truck_route.push_back(t.flp);
        P.sol.truck_route.push_back(s <= P.m ? static_cast<NodeId>(s) : 0);
    }
    // FLPs of gap 0 are inserted before skeleton node 1 (after the depot) - handled by t.gap + 1 == s with s = 1
    int drone = 0;
    for (const auto& t : trs) P.sol.drone_trips.push_back(DroneTrip{drone++ % 2, t.flp, t.cust, t.rec <= P.m ? static_cast<NodeId>(t.rec) : 0});
    return P;
}

void test_properties() {
    std::mt19937_64 g(20261109ULL);
    int built = 0, extended = 0, frozen = 0, invalid = 0;
    for (int it = 0; it < 600; ++it) {
        RandPlan P = random_plan(g);
        const PlanLayout L = layout_of(P.sol, P.nodes);
        if (!L.ok) { ++invalid; continue; }
        const std::size_t cut = g() % (L.last() - 1);
        for (std::size_t k = cut + 1; k < L.last(); ++k) {
            const NodeId r = L.sk.node[k];
            for (int w : {0, 1, 3}) {
                RegionR3 b3; b3.w = w;
                const Region R = b3.build(RegionContext{L, P.nodes, cut, r});
                CHECK(R.valid);
                if (!R.valid) continue;
                ++built; extended += R.closure_extended; frozen += !R.frozen_trips.empty();
                const auto v = validate_region(R, P.sol, P.nodes);
                CHECK(v.empty());
                if (!v.empty()) for (const auto& x : v) std::printf("   violation: %s (it=%d cut=%zu k=%zu w=%d)\n", x.c_str(), it, cut, k, w);
                CHECK(R.lo >= cut && R.hi <= L.last() && R.lo < R.hi);
                CHECK((R.lo <= k - 1 && R.hi >= k + 1) || R.lo == cut || R.hi == L.last()); // the insertion gaps are inside unless clipped
                CHECK(R.is_free(r));
                for (auto ti : R.frozen_trips) CHECK(L.trips[ti].launch_gap < cut); // freezing happens only for a trip airborne across the cut
                // idempotence: closing the final region again changes nothing
                std::size_t lo = R.lo, hi = R.hi; auto fr = R.frozen_trips; bool ext = false;
                detail_region::close(L, cut, lo, hi, fr, ext);
                CHECK(lo == R.lo && hi == R.hi && fr == R.frozen_trips && !ext);
                // monotone in the margin: a larger margin never shrinks the region
                RegionR3 bw; bw.w = w + 1;
                const Region R2 = bw.build(RegionContext{L, P.nodes, cut, r});
                CHECK(R2.lo <= R.lo && R2.hi >= R.hi);
                // determinism
                CHECK(b3.build(RegionContext{L, P.nodes, cut, r}) == R);
                // identity optimiser result is always preserved
                CHECK(check_region_preserved(P.sol, P.sol, R, P.nodes).empty());
            }
            RegionR5 b5; const Region F = b5.build(RegionContext{L, P.nodes, cut, r});
            CHECK(F.valid && F.lo == cut && F.hi == L.last() && validate_region(F, P.sol, P.nodes).empty());
            RegionR0 b0; const Region E = b0.build(RegionContext{L, P.nodes, cut, r});
            CHECK(E.valid && E.empty && validate_region(E, P.sol, P.nodes).empty());
        }
    }
    CHECK(built > 1000 && extended > 0);   // the closure is actually exercised
    CHECK(frozen > 0);                     // random cuts are NOT clean, so trips spanning the cut are frozen (see the check inside the loop)
    std::printf("   property: %d regions built, %d closure-extended, %d with frozen trips, %d plans rejected by layout\n", built, extended, frozen, invalid);
}

// P2 (DM0 §4.3): with a trip-closed region whose right boundary is a skeleton node not spanned by a trip, permuting the region interior changes the time at
// hi by some delta and shifts EVERY customer behind hi by exactly that delta. Drone-free world so the evaluator is exact.
void test_p2_tail_shift() {
    model::NodeRegistry n; n.add(Node{0, NodeType::Depot, {0, 0}});
    World::CustomerMap c;
    const double xy[8][2] = {{0, 0}, {9000, 3000}, {2000, 8000}, {14000, 1000}, {6000, -7000}, {18000, 6000}, {11000, 11000}, {25000, 0}};
    for (int i = 1; i <= 7; ++i) { n.add(Node{i, NodeType::Customer, {xy[i][0], xy[i][1]}}); model::CustomerData d; d.node_id = i; d.truck_service_time = 30; d.drone_service_time = 30; d.generalized_flight_time_theta = 1; c[i] = d; }
    auto w = World::create(n, c, timing::FlpDataMap{}, model::TruckParameters{15.0}, model::DroneParameters{12.0, 1800.0, 60.0, 40.0, 2});
    Solution S; S.truck_route = {0, 1, 2, 3, 4, 5, 6, 7, 0};
    auto base = InstalledPlan::make(S, w, 0.0, 1);
    CHECK(base.has_value());
    if (!base) return;
    RegionR3 r3; r3.w = 1;
    const PlanLayout L = layout_of(S, w->nodes());
    const Region R = r3.build(RegionContext{L, w->nodes(), 0, 4}); // skeleton 4, w=1 -> interval [2,6]
    CHECK(R.valid && R.lo == 2 && R.hi == 6);
    Solution T = S; std::reverse(T.truck_route.begin() + 3, T.truck_route.begin() + 6); // reverse nodes 3,4,5 (strictly inside [2,6])
    CHECK(check_region_preserved(S, T, R, w->nodes()).empty());
    auto moved = InstalledPlan::make(T, w, 0.0, 2);
    CHECK(moved.has_value());
    if (!moved) return;
    const auto e0 = eta_map(base->eval()), e1 = eta_map(moved->eval());
    const double delta = e1.at(6) - e0.at(6);            // 6 is the boundary node hi
    CHECK(std::fabs(delta) > 1.0);                       // the reordering actually changed something
    for (int tail : {7}) CHECK(std::fabs((e1.at(tail) - e0.at(tail)) - delta) <= 1e-9 * std::max(1.0, std::fabs(e0.at(tail))));
    for (int pre : {1, 2}) CHECK(std::fabs(e1.at(pre) - e0.at(pre)) <= 1e-12 + 1e-9 * std::fabs(e0.at(pre))); // prefix and boundary lo unchanged
    // end-to-end: objective change = interior change + tail_count * delta
    double d_int = 0; for (int i : {3, 4, 5}) d_int += e1.at(i) - e0.at(i);
    const double d_hi = e1.at(6) - e0.at(6), d_tail = 1 * delta;
    CHECK(std::fabs((moved->objective() - base->objective()) - (d_int + d_hi + d_tail)) <= 1e-7 * std::fabs(base->objective()));
    // integration: build the region from an InstalledPlan through the convenience wrapper
    const Cut cut{0, 0, 0.0};
    const Region R2 = build_region(r3, *base, cut, 4);
    CHECK(R2 == R);
    CHECK(impact_of(*base, 4).valid);
}

// ======================================= RIC core: half-step margin, P6 reach, P3 cost ranking =======================================

Request mkreq(std::uint32_t i, double x, double y) { Request r; r.index = i; r.time = 0; r.location = {x, y}; r.se1 = 30; r.se2 = 30; r.theta = 1; return r; }
model::DroneParameters drone_e(double e) { return model::DroneParameters{12.0, e, 60.0, 40.0, 2}; }
model::CustomerData cdata() { model::CustomerData d; d.truck_service_time = 30; d.drone_service_time = 30; d.generalized_flight_time_theta = 1; return d; }

struct Pt { double x, y; };
// plain truck world: depot 0 and customers 1..n at the given points (+ optional FLP nodes appended with explicit ids and arcs)
std::shared_ptr<const World> mkworld(const std::vector<Pt>& pts, double e, const std::vector<std::pair<NodeId, Pt>>& flps = {}, const std::vector<model::FlexibleLaunchPointData>& fdata = {}) {
    model::NodeRegistry n; World::CustomerMap c; timing::FlpDataMap f;
    n.add(Node{0, NodeType::Depot, {pts[0].x, pts[0].y}});
    for (std::size_t i = 1; i < pts.size(); ++i) { n.add(Node{static_cast<NodeId>(i), NodeType::Customer, {pts[i].x, pts[i].y}}); auto d = cdata(); d.node_id = static_cast<NodeId>(i); c[static_cast<NodeId>(i)] = d; }
    for (const auto& fl : flps) n.add(Node{fl.first, NodeType::FlexibleLaunchPoint, {fl.second.x, fl.second.y}});
    for (const auto& fd : fdata) f[fd.node_id] = fd;
    return World::create(n, c, f, model::TruckParameters{15.0}, drone_e(e));
}
InstalledPlan base_of(const std::shared_ptr<const World>& w, std::vector<NodeId> route) { Solution s; s.truck_route = std::move(route); auto p = InstalledPlan::make(s, w, 0.0, 1); return *p; }

// ---- (1) margin reconciliation with the static window (D1) ----
void test_half_step_margin() {
    // the static rule |h1 - h2| <= 2w: gaps_within_window(sk, 2g+1, w) = [g-w, g+w]  (w truck nodes, 2w half-steps), NOT |dh| <= 3
    const auto n = registry();
    Solution s = base_plan();
    const PlanLayout L = layout_of(s, n);
    for (int w : {0, 1, 2, 3}) {
        for (std::size_t g = 0; g + 1 < L.sk.size(); ++g) {
            const auto gs = tdr::search::gaps_within_window(L.sk, 2 * static_cast<int>(g) + 1, w);
            const std::size_t lo = g > static_cast<std::size_t>(w) ? g - w : 0, hi = std::min(L.sk.gap_count() - 1, g + static_cast<std::size_t>(w));
            CHECK(!gs.empty() && gs.front() == lo && gs.back() == hi);
        }
    }
    // literal "w = 3 half-steps" (|dh| <= 3 <=> window 1.5 nodes) is a DIFFERENT, smaller set than the static w = 3 (|dh| <= 6): the code follows the source
    const auto w3 = tdr::search::gaps_within_window(L.sk, 2 * 4 + 1, 3), w1 = tdr::search::gaps_within_window(L.sk, 2 * 4 + 1, 1);
    CHECK(w3.size() == 7 && w1.size() == 3);
    // region margin equals the static window on both sides of the core hull, clipped at the cut and the end depot
    RegionR3 r3; r3.w = 3;
    for (std::size_t k = 1; k < L.last(); ++k) {
        const Region R = r3.build(RegionContext{L, n, 0, L.sk.node[k]});
        const std::size_t want_lo = k - 1 > 3 ? k - 1 - 3 : 0, want_hi = std::min(L.last(), k + 1 + 3);
        CHECK(R.valid && R.core_lo == k - 1 && R.core_hi == k + 1);
        CHECK(R.lo <= want_lo && R.hi >= want_hi); // closure may only enlarge
    }
    // V5 property: every customer ADJACENT (static PositionIndex, window w) to r lies inside the region
    std::mt19937_64 g(77);
    int checked = 0;
    for (int it = 0; it < 300; ++it) {
        RandPlan P = random_plan(g);
        const PlanLayout PL = layout_of(P.sol, P.nodes);
        if (!PL.ok) continue;
        const tdr::search::PositionIndex pi(P.sol, P.nodes);
        for (std::size_t k = 1; k < PL.last(); ++k) {
            const NodeId r = PL.sk.node[k];
            for (int w : {1, 2, 3}) {
                RegionR3 b; b.w = w;
                const Region R = b.build(RegionContext{PL, P.nodes, 0, r});
                for (NodeId nb : pi.neighbours(r, w)) {
                    const auto h = *pi.half_step(nb);
                    bool inside;
                    if (h % 2 == 0) { const std::size_t sidx = static_cast<std::size_t>(h / 2); inside = sidx >= R.lo && sidx <= R.hi; }   // truck customer
                    else { const std::size_t gg = static_cast<std::size_t>((h - 1) / 2); inside = gg >= R.lo && gg < R.hi; }             // drone customer: launch gap
                    CHECK(inside); ++checked;
                }
            }
        }
    }
    CHECK(checked > 500);
}

// ---- (2) P6 reach: formula, boundary inclusion/exclusion against the evaluator, regimes P and F ----
void test_reach_formula() {
    CHECK(std::fabs(drone_reach_radius(drone_e(1800), 1, 30) - 6960.0) < 1e-9);   // regime P (paper parameters), DM0 §4.5
    CHECK(std::fabs(drone_reach_radius(drone_e(3600), 1, 30) - 14160.0) < 1e-9);  // regime F illustration, DM0 §12.1
    // theta/Se2 enter: theta = 0, Se2 = 0 -> v e / 2
    CHECK(std::fabs(drone_reach_radius(drone_e(1800), 0, 0) - 12.0 * 1800 / 2) < 1e-9);
    CHECK(std::fabs(dist_point_segment({0, 5}, {-1, 0}, {1, 0}) - 5.0) < 1e-12 && std::fabs(dist_point_segment({3, 4}, {0, 0}, {1, 0}) - std::hypot(2.0, 4.0)) < 1e-12);
}

// base: depot(0,0) c1(10000,0) c2(20000,0), FLP 1000 coincident with c1 on arc (0,c1) (ratio 1). Trip (1000, r, 1): launch and recovery both at c1 -> (37)=(36)=3h/12+60.
bool trip_feasible_at(double h, double e) {
    const auto w0 = mkworld({{0, 0}, {10000, 0}, {20000, 0}}, e, {{1000, {10000, 0}}}, {model::FlexibleLaunchPointData{1000, 0, 1, 1.0}});
    const auto wr = w0->with_request(100, mkreq(1, 10000, h));
    Solution s; s.truck_route = {0, 1000, 1, 2, 0}; s.drone_trips = {DroneTrip{0, 1000, 100, 1}};
    return wr->evaluate(s).feasible;
}
bool reach_ok_at(double h, double e, std::size_t gap) {
    const auto w0 = mkworld({{0, 0}, {10000, 0}, {20000, 0}}, e);
    const InstalledPlan base = base_of(w0, {0, 1, 2, 0});
    const PlanLayout L = layout_of(base.solution(), w0->nodes());
    for (const auto& r : drone_reach_gaps(base, L, 0, {10000, h}, 30, 1)) if (r.gap == gap) return r.ok;
    return false;
}
void test_reach_boundary_evaluator() {
    // regime P: boundary r_D* = 6960 m (evaluator and predicate agree on both sides)
    CHECK(trip_feasible_at(6960.0 - 1e-3, 1800) && reach_ok_at(6960.0 - 1e-3, 1800, 0));
    CHECK(!trip_feasible_at(6960.0 + 1e-3, 1800) && !reach_ok_at(6960.0 + 1e-3, 1800, 0));
    CHECK(trip_feasible_at(3000, 1800) && reach_ok_at(3000, 1800, 0));
    CHECK(!trip_feasible_at(9000, 1800) && !reach_ok_at(9000, 1800, 0));
    // regime F (e = 3600): boundary 14160 m; the SAME point 10000 m away is infeasible/excluded in P and feasible/included in F
    CHECK(trip_feasible_at(14160.0 - 1e-3, 3600) && reach_ok_at(14160.0 - 1e-3, 3600, 0));
    CHECK(!trip_feasible_at(14160.0 + 1e-3, 3600) && !reach_ok_at(14160.0 + 1e-3, 3600, 0));
    CHECK(!trip_feasible_at(10000, 1800) && !reach_ok_at(10000, 1800, 0));
    CHECK(trip_feasible_at(10000, 3600) && reach_ok_at(10000, 3600, 0));
    // the far gap (c2, depot): rho = 5000 but the only recovery behind it is the depot, 11180 m away -> lhs = 1825 s > 1800: excluded in P, included in F
    CHECK(!reach_ok_at(5000, 1800, 2) && reach_ok_at(5000, 3600, 2) && reach_ok_at(5000, 1800, 0));
    // no drones in the fleet -> never reach-feasible
    {
        model::NodeRegistry n; World::CustomerMap c;
        n.add(Node{0, NodeType::Depot, {0, 0}}); n.add(Node{1, NodeType::Customer, {10000, 0}}); { auto d = cdata(); d.node_id = 1; c[1] = d; }
        auto w = World::create(n, c, timing::FlpDataMap{}, model::TruckParameters{15.0}, model::DroneParameters{12.0, 1800, 60, 40, 0});
        const InstalledPlan b = base_of(w, {0, 1, 0});
        const PlanLayout L = layout_of(b.solution(), w->nodes());
        for (const auto& r : drone_reach_gaps(b, L, 0, {10000, 100}, 30, 1)) CHECK(!r.ok);
    }
}

// Evaluator fuzz: whenever the evaluator accepts a drone trip (launch FLP on gap g's arc, recovery at a later skeleton node), the predicate for gap g holds (no false exclusion)
void test_reach_no_false_exclusion() {
    std::mt19937_64 g(31337);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    int feasible = 0, excluded = 0, total = 0;
    for (double e : {1800.0, 3600.0}) {
        for (int it = 0; it < 500; ++it) {
            const std::size_t m = 3 + g() % 3; // customers
            std::vector<Pt> pts{{30000 * U(g), 30000 * U(g)}};
            for (std::size_t i = 1; i <= m; ++i) pts.push_back({30000 * U(g), 30000 * U(g)});
            std::vector<NodeId> route{0}; for (std::size_t i = 1; i <= m; ++i) route.push_back(static_cast<NodeId>(i)); route.push_back(0);
            const std::size_t gap = g() % (m + 1);                     // arc (route[gap], route[gap+1]) of the truck-only route
            const std::size_t rec = gap + 1 + g() % (m + 1 - gap);     // recovery skeleton index in gap+1..m+1
            const double fr = U(g);
            const Pt A = pts[route[gap]], B = pts[route[gap + 1]];
            const Pt Fp{A.x + fr * (B.x - A.x), A.y + fr * (B.y - A.y)};
            const Pt X{30000 * U(g), 30000 * U(g)};
            auto w0 = mkworld(pts, e, {{1000, Fp}}, {model::FlexibleLaunchPointData{1000, route[gap], route[gap + 1], fr}});
            const auto wr = w0->with_request(100, mkreq(1, X.x, X.y));
            Solution s; s.truck_route = route; s.truck_route.insert(s.truck_route.begin() + static_cast<std::ptrdiff_t>(gap + 1), 1000);
            s.drone_trips = {DroneTrip{0, 1000, 100, rec <= m ? static_cast<NodeId>(rec) : 0}};
            const bool ok = wr->evaluate(s).feasible;
            const InstalledPlan base = base_of(w0, route);
            const PlanLayout L = layout_of(base.solution(), w0->nodes());
            const auto reach = drone_reach_gaps(base, L, 0, {X.x, X.y}, 30, 1);
            ++total;
            if (ok) { ++feasible; CHECK(reach[gap].ok); }  // necessary condition
            if (!reach[gap].ok) ++excluded;
        }
    }
    CHECK(feasible > 40 && excluded > 100);   // the test has power in both directions
    std::printf("   reach fuzz: %d trials, %d evaluator-feasible (all included), %d gaps excluded by reach\n", total, feasible, excluded);
}

// ---- (3) P3 insertion cost against the evaluator, and its ranking ----
void test_cost_ranking_truck_only() {
    std::mt19937_64 g(4242);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    int exact_cmp = 0, rank_cmp = 0;
    for (int it = 0; it < 200; ++it) {
        const std::size_t m = 3 + g() % 5;
        std::vector<Pt> pts{{30000 * U(g), 30000 * U(g)}};
        for (std::size_t i = 1; i <= m; ++i) pts.push_back({30000 * U(g), 30000 * U(g)});
        std::vector<NodeId> route{0}; for (std::size_t i = 1; i <= m; ++i) route.push_back(static_cast<NodeId>(i)); route.push_back(0);
        auto w0 = mkworld(pts, 1800);
        const InstalledPlan base = base_of(w0, route);
        const Pt X{30000 * U(g), 30000 * U(g)};
        const auto req = mkreq(1, X.x, X.y);
        const auto cw = w0->with_request(100, req);
        const PlanLayout L = layout_of(base.solution(), w0->nodes());
        const auto costs = insertion_costs(base, L, 0, {X.x, X.y}, 30);
        const InsertionResult ins = cheapest_truck_insertion(base, Cut{0, 0, 0.0}, 100, cw);
        CHECK(costs.size() == ins.trials.size());
        std::vector<std::pair<double, std::size_t>> by_eval;
        for (std::size_t i = 0; i < costs.size(); ++i) {
            CHECK(costs[i].p3_exact && !costs[i].straddled && !costs[i].has_flps);
            CHECK(ins.trials[i].feasible);
            const double d_eval = ins.trials[i].objective - base.objective();
            CHECK(std::fabs(costs[i].cost - d_eval) <= 1e-9 * std::max(1.0, std::fabs(d_eval)));   // P3 equality on truck-only plans
            ++exact_cmp;
            by_eval.push_back({ins.trials[i].objective, i});
        }
        std::stable_sort(by_eval.begin(), by_eval.end(), [](auto& a, auto& b) { return a.first < b.first; });
        const auto rank = rank_gaps_by_cost(costs);
        // identical ordering up to numerically tied costs
        for (std::size_t i = 0; i < rank.size(); ++i) { CHECK(std::fabs(costs[rank[i] - 0].cost - (by_eval[i].first - base.objective())) <= 1e-6 * std::max(1.0, std::fabs(by_eval[i].first))); ++rank_cmp; }
        CHECK(rank.front() == ins.trials[*ins.best].gap.skel || std::fabs(costs[rank.front()].cost - costs[ins.trials[*ins.best].gap.skel].cost) <= 1e-9 * std::fabs(costs[rank.front()].cost));
    }
    CHECK(exact_cmp > 800 && rank_cmp > 800);
}

// CE-4 world: P3 is exact only on gaps without FLPs / launches / straddling; the others are FLAGGED (detected, not hidden) and the evaluator may reject them
void test_cost_flags_with_trip() {
    model::NodeRegistry n; World::CustomerMap c; timing::FlpDataMap f;
    n.add(Node{0, NodeType::Depot, {-3000, 0}}); n.add(Node{1, NodeType::Customer, {6000, 0}}); n.add(Node{2, NodeType::Customer, {12000, 0}});
    n.add(Node{3, NodeType::Customer, {6000, 2000}}); n.add(Node{4, NodeType::Customer, {12000, -4000}}); n.add(Node{1000, NodeType::FlexibleLaunchPoint, {0, 0}});
    for (int i : {1, 2, 3, 4}) { auto d = cdata(); d.node_id = i; c[i] = d; }
    f[1000] = model::FlexibleLaunchPointData{1000, 0, 1, 0.5};
    auto w0 = World::create(n, c, f, model::TruckParameters{15.0}, drone_e(1800));
    Solution s; s.truck_route = {0, 1000, 1, 2, 4, 0}; s.drone_trips = {DroneTrip{0, 1000, 3, 2}};
    auto base = InstalledPlan::make(s, w0, 0.0, 1);
    CHECK(base.has_value());
    if (!base) return;
    const PlanLayout L = layout_of(base->solution(), w0->nodes());
    const Pt X{9000, 4758};
    const auto costs = insertion_costs(*base, L, 0, {X.x, X.y}, 30);
    const auto cw = w0->with_request(100, mkreq(1, X.x, X.y));
    const InsertionResult ins = cheapest_truck_insertion(*base, Cut{0, 0, 0.0}, 100, cw);
    CHECK(costs.size() == 4 && ins.trials.size() == 4);
    CHECK(costs[0].has_flps && !costs[0].p3_exact && !ins.trials[0].feasible);       // gap (depot,c1): launch FLP on the split arc (CE-2) -> evaluator-infeasible, flagged
    CHECK(costs[1].straddled && !costs[1].p3_exact);                                  // gap (c1,c2): straddled by the trip (CE-1/CE-4), flagged
    CHECK(costs[2].p3_exact && costs[3].p3_exact);                                    // after the recovery: exact
    for (int i : {2, 3}) { CHECK(ins.trials[i].feasible); const double d = ins.trials[i].objective - base->objective(); CHECK(std::fabs(costs[i].cost - d) <= 1e-9 * std::max(1.0, std::fabs(d))); }
    // m_g counts the drone customer when it is launched behind b: gap (depot,c1) has b=c1, trip launched at position 1 < P(c1)=2 -> not behind; customers behind b: c1,c2,c4 = 3
    CHECK(costs[0].m == 3 && costs[2].m == 1 + 0 && costs[3].m == 0);
}

// ---- (4) RIC core integration: best-cost gap U drone-reach gaps U insertion gaps; hull; margin; closure; impact set separate ----
struct Cand { std::shared_ptr<const World> cw; std::optional<InstalledPlan> plan; InsertionResult ins; };
Cand candidate_of(const InstalledPlan& base, NodeId rid, const Request& rq) {
    Cand c; c.cw = base.world().with_request(rid, rq);
    c.ins = cheapest_truck_insertion(base, Cut{0, 0, 0.0}, rid, c.cw);
    if (c.ins.best) c.plan = InstalledPlan::make(c.ins.solution, c.cw, 0.0, 2);
    return c;
}

// plan with a far reach gap whose membership toggles at the 1e-3 m boundary
//   r=(0,0); route D(-40000,-2000) a(-2000,-500) b(2000,-500) q1(0,3000) q2(0,nu) D; gap (q1,q2): rho=3000, nu -> condition nu <= 14880 (regime P)
Cand far_reach(double nu, double e, std::optional<InstalledPlan>& base_out) {
    auto w0 = mkworld({{-40000, -2000}, {-2000, -500}, {2000, -500}, {0, 3000}, {0, nu}}, e);
    base_out = base_of(w0, {0, 1, 2, 3, 4, 0});
    return candidate_of(*base_out, 100, mkreq(1, 0, 0));
}

void test_ric_core() {
    std::optional<InstalledPlan> base;
    Cand c = far_reach(14880.0 - 1e-3, 1800, base);
    CHECK(c.plan.has_value());
    if (!c.plan) return;
    const PlanLayout CL = layout_of(c.plan->solution(), c.cw->nodes());
    const std::size_t k = *CL.skel_of_node(100);
    CHECK(k == 2);                                          // r is cheapest between a and b (gap 1 of the base plan)
    RegionR3 ric = RegionR3::ric(0);                        // w = 0 isolates the core
    RicInputs in{&*base, c.cw.get()};
    Region R = ric.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in});
    CHECK(R.valid && R.ric.used && clean(R, c.plan->solution(), c.cw->nodes()));
    CHECK(R.ric.cost_rank.front() == 1 && R.ric.cost_core_gaps == std::vector<std::size_t>{1}); // best P3 cost gap = (a,b)
    // reach gaps (base coords) include the far gap 3 = (q1,q2) at nu = 14880 - 1e-3 and the near ones
    CHECK(std::find(R.ric.reach_core_gaps.begin(), R.ric.reach_core_gaps.end(), 3) != R.ric.reach_core_gaps.end());
    // candidate coordinates: base gap 1 -> {1,2}; base gap 3 -> 4 ; base gap 0 -> 0 ; base gap 2 -> 3
    CHECK((R.ric.core_gaps == std::vector<std::size_t>{0, 1, 2, 3, 4}));
    CHECK(R.core_lo == 0 && R.core_hi == 5 && R.lo == 0 && R.hi == 5);
    const Region with_boundary = R;
    // immediately beyond the boundary the far gap leaves the core
    Cand c2 = far_reach(14880.0 + 1e-3, 1800, base);
    CHECK(c2.plan.has_value());
    const PlanLayout CL2 = layout_of(c2.plan->solution(), c2.cw->nodes());
    RicInputs in2{&*base, c2.cw.get()};
    const Region R2 = ric.build(RegionContext{CL2, c2.cw->nodes(), 0, 100, &in2});
    CHECK(R2.valid && std::find(R2.ric.reach_core_gaps.begin(), R2.ric.reach_core_gaps.end(), 3) == R2.ric.reach_core_gaps.end());
    CHECK((R2.ric.core_gaps == std::vector<std::size_t>{0, 1, 2, 3}));
    CHECK(R2.hi == 4 && with_boundary.hi == 5 && R2.hi < with_boundary.hi);   // inclusion at the boundary changes the region; exclusion just beyond does not include it
    // evaluator agrees on the same two sides: a trip launched at q1 and recovered at q2 serving r is feasible at the inside value and infeasible beyond
    auto trip_ok = [&](double nu) {
        auto w0 = mkworld({{-40000, -2000}, {-2000, -500}, {2000, -500}, {0, 3000}, {0, nu}}, 1800, {{1000, {0, 3000}}}, {model::FlexibleLaunchPointData{1000, 3, 4, 0.0}});
        const auto wr = w0->with_request(100, mkreq(1, 0, 0));
        Solution s; s.truck_route = {0, 1, 2, 3, 1000, 4, 0}; s.drone_trips = {DroneTrip{0, 1000, 100, 4}};
        return wr->evaluate(s).feasible;
    };
    CHECK(trip_ok(14880.0 - 1e-3) && !trip_ok(14880.0 + 1e-3));
    // regime F: the same geometry with e = 3600 puts the far gap (and more) in reach at nu = 14880 + 1e-3
    Cand c3 = far_reach(14880.0 + 1e-3, 3600, base);
    CHECK(c3.plan.has_value());
    const PlanLayout CL3 = layout_of(c3.plan->solution(), c3.cw->nodes());
    RicInputs in3{&*base, c3.cw.get()};
    const Region R3f = ric.build(RegionContext{CL3, c3.cw->nodes(), 0, 100, &in3});
    CHECK(R3f.valid && std::find(R3f.ric.reach_core_gaps.begin(), R3f.ric.reach_core_gaps.end(), 3) != R3f.ric.reach_core_gaps.end());
    CHECK(R3f.hi >= R2.hi && R3f.hi == 5);
    // approved A1 default: unlimited reach gaps, one cost gap, not flagged restricted; reach list ordered by increasing rho (ties -> lower gap)
    CHECK(ric.n_reach == 0 && ric.n_cost == 1 && !R.restricted_variant && std::string(ric.name()) == "R3-ric-core");
    {
        std::size_t nok = 0; for (const auto& r : R.ric.reach) nok += r.ok;
        CHECK(R.ric.reach_core_gaps.size() == nok);                      // ALL reach-feasible gaps, no truncation
        for (std::size_t i = 1; i < R.ric.reach_core_gaps.size(); ++i) {
            const auto &a = R.ric.reach[R.ric.reach_core_gaps[i - 1] - R.ric.reach.front().gap], &b = R.ric.reach[R.ric.reach_core_gaps[i] - R.ric.reach.front().gap];
            CHECK(a.rho < b.rho || (a.rho == b.rho && a.gap < b.gap));
        }
    }
    // n_reach cap: keep only the nearest reach gap -> core = best gap U nearest reach gap
    RegionR3 capped = RegionR3::ric(0); capped.n_reach = 1;
    const Region Rc = capped.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in});
    CHECK(Rc.valid && Rc.ric.reach_core_gaps.size() == 1 && Rc.hi < with_boundary.hi);
    CHECK(Rc.restricted_variant && std::string(capped.name()) == "R3-ric-core-RESTRICTED-variant");
    RegionR3 two = RegionR3::ric(0); two.n_cost = 2;
    CHECK(two.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in}).restricted_variant);
    // never narrower than the insertion-gaps core (monotone), margin only enlarges
    RegionR3 legacy; legacy.w = 0;
    const Region Rl = legacy.build(RegionContext{CL, c.cw->nodes(), 0, 100});
    CHECK(R.lo <= Rl.lo && R.hi >= Rl.hi);
    RegionR3 wide = RegionR3::ric(1);
    const Region Rw = wide.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in});
    CHECK(Rw.lo <= R.lo && Rw.hi >= R.hi && Rw.valid && clean(Rw, c.plan->solution(), c.cw->nodes()));
    // determinism
    CHECK(ric.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in}) == R);
    // missing inputs / inconsistent candidate are reported, never guessed
    CHECK(!ric.build(RegionContext{CL, c.cw->nodes(), 0, 100}).valid);
    RicInputs wrong{&*base, c.cw.get()};
    Solution tampered = c.plan->solution();
    { auto i1 = std::find(tampered.truck_route.begin(), tampered.truck_route.end(), 1), i2 = std::find(tampered.truck_route.begin(), tampered.truck_route.end(), 2); std::iter_swap(i1, i2); } // base nodes reordered: not base + r
    const PlanLayout TL = layout_of(tampered, c.cw->nodes());
    CHECK(!ric.build(RegionContext{TL, c.cw->nodes(), 0, 100, &wrong}).valid);
    // cut: the core never crosses behind the cut (V1) even if reach gaps exist behind it
    const Region Rcut = ric.build(RegionContext{CL, c.cw->nodes(), 1, 100, &in});
    CHECK(Rcut.valid && Rcut.lo >= 1 && clean(Rcut, c.plan->solution(), c.cw->nodes()));
    CHECK(std::all_of(Rcut.ric.core_gaps.begin(), Rcut.ric.core_gaps.end(), [](std::size_t g) { return g >= 1; }));
}

// degrades to the insertion-gaps core when nothing is in reach and the cost-best gap is where r was inserted (truck-only P3 is exact)
void test_ric_degrades() {
    std::mt19937_64 g(99);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    int same = 0, wider = 0;
    for (int it = 0; it < 150; ++it) {
        const std::size_t m = 4 + g() % 4;
        std::vector<Pt> pts{{60000 * U(g), 60000 * U(g)}};
        for (std::size_t i = 1; i <= m; ++i) pts.push_back({60000 * U(g), 60000 * U(g)});
        std::vector<NodeId> route{0}; for (std::size_t i = 1; i <= m; ++i) route.push_back(static_cast<NodeId>(i)); route.push_back(0);
        auto w0 = mkworld(pts, 1800);
        const InstalledPlan base = base_of(w0, route);
        const Pt X{60000 * U(g), 60000 * U(g)};
        const Cand c = candidate_of(base, 100, mkreq(1, X.x, X.y));
        if (!c.plan) continue;
        const PlanLayout CL = layout_of(c.plan->solution(), c.cw->nodes());
        RicInputs in{&base, c.cw.get()};
        for (int w : {0, 2, 3}) {
            RegionR3 ric = RegionR3::ric(w), legacy; legacy.w = w;
            const Region R = ric.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in});
            const Region Rl = legacy.build(RegionContext{CL, c.cw->nodes(), 0, 100});
            CHECK(R.valid && clean(R, c.plan->solution(), c.cw->nodes()) && R.is_free(100));
            CHECK(R.lo <= Rl.lo && R.hi >= Rl.hi);                    // RIC is a superset of the gaps-only core
            // the cost-best gap IS where the evaluator-cheapest insertion put r (truck-only, P3 exact)
            const auto& cg = R.ric.cost_core_gaps;
            CHECK(cg.size() == 1 && cg[0] + 1 == *CL.skel_of_node(100));
            bool no_reach = R.ric.reach_core_gaps.empty();
            if (no_reach) { CHECK(R == Rl || (R.lo == Rl.lo && R.hi == Rl.hi)); ++same; } else if (R.lo < Rl.lo || R.hi > Rl.hi) ++wider;
        }
    }
    CHECK(same > 20 && wider > 20);   // both situations occur
}

// impact set vs optimisation region stay separate objects and the RIC region never redefines the impact set
void test_ric_impact_separation() {
    std::optional<InstalledPlan> base;
    Cand c = far_reach(14880.0 - 1e-3, 1800, base);
    CHECK(c.plan.has_value());
    if (!c.plan) return;
    const PlanLayout CL = layout_of(c.plan->solution(), c.cw->nodes());
    RicInputs in{&*base, c.cw.get()};
    const ImpactSet I0 = impact_of(CL, c.cw->nodes(), 100);
    for (int w : {0, 1, 3}) {
        for (int nr : {0, 1}) {
            RegionR3 ric = RegionR3::ric(w); ric.n_reach = nr;
            const Region R = ric.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in});
            const ImpactSet I = impact_of(CL, c.cw->nodes(), 100);
            CHECK(I.valid && I.shifted_customers == I0.shifted_customers && I.node_a == I0.node_a && I.node_b == I0.node_b);   // independent of the region
            CHECK(R.valid && clean(R, c.plan->solution(), c.cw->nodes()));
        }
    }
    RegionR3 narrow = RegionR3::ric(0); narrow.n_reach = 1;
    RegionR3 broad = RegionR3::ric(0);
    const RegionReport a = describe(narrow.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in}), I0), b = describe(broad.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in}), I0);
    CHECK(a.impact_customers == b.impact_customers);          // same impact
    CHECK(b.free_customers >= a.free_customers);              // different optimisation region
    // protected elements outside the region are preserved under the identity result and detected when changed
    const Region R = broad.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in});
    CHECK(check_region_preserved(c.plan->solution(), c.plan->solution(), R, c.cw->nodes()).empty());
    RegionR3 small = RegionR3::ric(0); small.n_reach = 1;
    const Region Rs = small.build(RegionContext{CL, c.cw->nodes(), 0, 100, &in});
    if (Rs.hi + 1 < CL.sk.size()) {
        Solution bad = c.plan->solution();
        const std::size_t p1 = CL.sk.route_position[Rs.hi], p2 = CL.sk.route_position[Rs.hi + 1];
        if (p2 < bad.truck_route.size() - 1) { std::swap(bad.truck_route[p1], bad.truck_route[p2]); CHECK(!check_region_preserved(c.plan->solution(), bad, Rs, c.cw->nodes()).empty()); }
    }
}

// RIC with drone trips: closure still applies on top of the larger core (trip-closure semantics unchanged)
void test_ric_with_trips_closure() {
    model::NodeRegistry n; World::CustomerMap c; timing::FlpDataMap f;
    n.add(Node{0, NodeType::Depot, {0, 0}});
    for (int i = 1; i <= 6; ++i) { n.add(Node{i, NodeType::Customer, {9000.0 * i, (i % 2) * 2000.0}}); auto d = cdata(); d.node_id = i; c[i] = d; }
    n.add(Node{7, NodeType::Customer, {9000.0 * 3, 5000}}); { auto d = cdata(); d.node_id = 7; c[7] = d; }
    n.add(Node{1000, NodeType::FlexibleLaunchPoint, {9000.0 * 2.5, 1000}});
    f[1000] = model::FlexibleLaunchPointData{1000, 2, 3, 0.5};
    auto w0 = World::create(n, c, f, model::TruckParameters{15.0}, drone_e(3600));
    Solution s; s.truck_route = {0, 1, 2, 1000, 3, 4, 5, 6, 0}; s.drone_trips = {DroneTrip{0, 1000, 7, 4}};
    auto base = InstalledPlan::make(s, w0, 0.0, 1);
    CHECK(base.has_value());
    if (!base) return;
    const Cand cand = candidate_of(*base, 100, mkreq(1, 9000.0 * 4.5, 1500));
    CHECK(cand.plan.has_value());
    if (!cand.plan) return;
    const PlanLayout CL = layout_of(cand.plan->solution(), cand.cw->nodes());
    RicInputs in{&*base, cand.cw.get()};
    for (int w : {0, 1, 3}) {
        const Region R = RegionR3::ric(w).build(RegionContext{CL, cand.cw->nodes(), 0, 100, &in});
        CHECK(R.valid && clean(R, cand.plan->solution(), cand.cw->nodes()));
        for (const auto& t : CL.trips) CHECK(!(t.launch_gap < R.lo && t.recovery_skel > R.lo) && !(t.launch_gap >= R.lo && t.launch_gap < R.hi && t.recovery_skel > R.hi));
        // closure idempotence on the final region
        std::size_t lo = R.lo, hi = R.hi; auto fr = R.frozen_trips; bool ext = false;
        detail_region::close(CL, 0, lo, hi, fr, ext);
        CHECK(lo == R.lo && hi == R.hi && !ext);
    }
}

} // namespace

int main() {
    test_layout();
    test_basic_and_boundaries();
    test_closure_extension();
    test_freeze_with_non_clean_cut();
    test_r0_r1_r4_and_empty();
    test_impact_vs_region();
    test_preservation();
    test_properties();
    test_p2_tail_shift();
    test_half_step_margin();
    test_reach_formula();
    test_reach_boundary_evaluator();
    test_reach_no_false_exclusion();
    test_cost_ranking_truck_only();
    test_cost_flags_with_trip();
    test_ric_core();
    test_ric_degrades();
    test_ric_impact_separation();
    test_ric_with_trips_closure();
    std::printf("DM7 region tests: %d checks, %d failures\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
