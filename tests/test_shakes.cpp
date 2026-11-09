// M6 unit tests: the four shaking operators (Random_Reverse, Random_Exchange, Random_Shift, Random_Drone_Remove), docs/static/shakes.md.
// Hand-derived fixtures on a line (truck 10 m/s, drone 20 m/s, SL = SR = 0, service times 0, e = 3600): explicit-parameter calls
// (deterministic cores) with hand-derived routes, dissolved trips and reasons; the randomised reinsertion draw is predicted by replaying
// the documented draw order on a fresh Rng. Every result is also passed through an independent invariant checker and compared with the
// FROZEN reference evaluator.
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
#include "search/shakes.hpp"
#include "search/route_ops.hpp"
#include "timing/timing_engine.hpp"
#include "util/rng.hpp"

#include <algorithm>
#include <array>
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

// Each customer 1..n appears exactly once, either on the truck route or as a trip's service customer.
bool exactly_once(const Solution& s, int n) {
    std::vector<int> count(static_cast<std::size_t>(n) + 1, 0);
    for (NodeId v : s.truck_route) if (v >= 1 && v <= n) ++count[static_cast<std::size_t>(v)];
    for (const auto& t : s.drone_trips) if (t.service_customer >= 1 && t.service_customer <= n) ++count[static_cast<std::size_t>(t.service_customer)];
    for (int i = 1; i <= n; ++i) if (count[static_cast<std::size_t>(i)] != 1) return false;
    return true;
}

double ref_objective(const Solution& s, const Fix& f, bool* feasible) {
    const auto e = tdr::timing_ref::evaluate_solution(s, *f.net, f.cd, f.flp);
    if (feasible) *feasible = e.feasible;
    return e.objective_value;
}

} // namespace

namespace {

using Sk = std::vector<NodeId>;

bool is_flp_id(const Fix& f, NodeId v) { return f.nodes.at(v).type == NodeType::FlexibleLaunchPoint; }

Sk skel(const TruckRoute& r, const Fix& f) {
    Sk s;
    for (NodeId v : r) if (!is_flp_id(f, v)) s.push_back(v);
    return s;
}

std::set<NodeId> flps_of(const TruckRoute& r, const Fix& f) {
    std::set<NodeId> s;
    for (NodeId v : r) if (is_flp_id(f, v)) s.insert(v);
    return s;
}

bool foreign_flp(const TruckRoute& r, const Fix& f) {
    NodeId last = kInvalidNodeId;
    std::vector<NodeId> pending;
    for (NodeId v : r) {
        if (is_flp_id(f, v)) { pending.push_back(v); continue; }
        for (NodeId p : pending) { const auto& d = f.flp.at(p); if (d.arc_from != last || d.arc_to != v) return true; }
        pending.clear();
        last = v;
    }
    return !pending.empty();
}

Sk without(Sk s, NodeId v) { s.erase(std::remove(s.begin(), s.end(), v), s.end()); return s; }

bool same_trip(const DroneTrip& a, const DroneTrip& b) { return a.drone_id == b.drone_id && a.launch_node == b.launch_node && a.service_customer == b.service_customer && a.recovery_node == b.recovery_node; }

// Invariants every OK result must satisfy (independent of the production helpers).
void check_shake(const ShakeResult& r, const Solution& base, const Fix& f, const std::string& tag) {
    if (r.status != ShakeStatus::Ok) { check(r.solution.truck_route.empty() && r.solution.drone_trips.empty(), tag + ": a non-OK result carries no solution"); return; }
    const Solution& s = r.solution;
    check(exactly_once(s, f.n), tag + ": exactly-once coverage");
    check(!foreign_flp(s.truck_route, f), tag + ": no FLP off its arc");
    bool no_idle = true;
    for (NodeId v : flps_of(s.truck_route, f)) { bool used = false; for (const auto& t : s.drone_trips) used = used || t.launch_node == v; no_idle = no_idle && used; }
    for (const auto& t : s.drone_trips) no_idle = no_idle && flps_of(s.truck_route, f).count(t.launch_node) == 1;
    check(no_idle, tag + ": every route FLP is a trip's launch and every launch is on the route (rule (6))");
    // the input trips are partitioned into preserved / removed / dissolved, each exactly once
    std::vector<int> seen(base.drone_trips.size(), 0);
    for (auto i : r.preserved) ++seen[i];
    for (const auto& x : r.removed) ++seen[x.trip];
    for (const auto& x : r.dissolved) ++seen[x.trip];
    bool part = true;
    for (int c : seen) part = part && c == 1;
    check(part, tag + ": every input trip is exactly one of preserved / removed / dissolved");
    bool kept = r.preserved.size() == s.drone_trips.size();
    for (std::size_t k = 0; kept && k < r.preserved.size(); ++k) kept = same_trip(s.drone_trips[k], base.drone_trips[r.preserved[k]]) && (k == 0 || r.preserved[k - 1] < r.preserved[k]);
    check(kept, tag + ": preserved trips are untouched (drone id, launch, customer, recovery) and keep their relative order");
    bool cust = true;
    for (const auto& x : r.dissolved) cust = cust && std::count(s.truck_route.begin(), s.truck_route.end(), x.customer) == 1;
    for (const auto& x : r.removed) cust = cust && std::count(s.truck_route.begin(), s.truck_route.end(), x.customer) == 1;
    check(cust, tag + ": every dissolved or removed trip's customer is now served by the truck");
    // FLP accounting: base FLPs = still on the route + lost with their arc + dropped idle, disjoint
    std::set<NodeId> all = flps_of(base.truck_route, f), got = flps_of(s.truck_route, f);
    std::set<NodeId> lost(r.flps_lost_with_arcs.begin(), r.flps_lost_with_arcs.end()), drop(r.flps_dropped.begin(), r.flps_dropped.end());
    std::set<NodeId> uni = got;
    uni.insert(lost.begin(), lost.end());
    uni.insert(drop.begin(), drop.end());
    check(uni == all && got.size() + lost.size() + drop.size() == all.size() && lost.size() == r.flps_lost_with_arcs.size() && drop.size() == r.flps_dropped.size(), tag + ": FLP accounting: base FLPs = on the route + lost with their arc + dropped (disjoint)");
    check(r.dissolved.size() + r.removed.size() + r.preserved.size() == base.drone_trips.size(), tag + ": dissolved + removed + preserved == input trips");
    if (r.evaluated) {
        bool feas = false;
        const double o = ref_objective(s, f, &feas);
        check(feas == r.feasible && std::fabs(o - r.objective) <= 1e-9 * std::max(1.0, std::fabs(o)), tag + ": verdict and objective equal the frozen reference's");
    }
}

// Fixture Q: n = 6, truck [0,1,2,3,4,F45,5,0] (m = 5), one trip (d0, F45 = x 4500, C6, recovery C5). Objective 2025.
struct FixQ { std::unique_ptr<Fix> f; Solution base; NodeId F45 = 0; };
FixQ make_Q() {
    FixQ q;
    q.f = make_line(6, 1);
    q.F45 = q.f->flp_at(4, 5, 0);
    q.base.truck_route = {0, 1, 2, 3, 4, q.F45, 5, 0};
    q.base.drone_trips = {trip(0, q.F45, 6, 5)};
    return q;
}

// Fixture P: n = 7, |U| = 2, truck [0,1,2,F23,3,5,F56,6,0] (m = 5); T1 = (d0, F23, C4, rec C5), T2 = (d1, F56, C7, rec C6). Objective 2650.
struct FixP { std::unique_ptr<Fix> f; Solution base; NodeId F23 = 0, F56 = 0; };
FixP make_P() {
    FixP p;
    p.f = make_line(7, 2);
    p.F23 = p.f->flp_at(2, 3, 0);
    p.F56 = p.f->flp_at(5, 6, 0);
    p.base.truck_route = {0, 1, 2, p.F23, 3, 5, p.F56, 6, 0};
    p.base.drone_trips = {trip(0, p.F23, 4, 5), trip(1, p.F56, 7, 6)};
    return p;
}

// Fixture F: n = 4, truck [0,F_a,1,F_b,0] (m = 1); T_x = (d0, F_a, C2, rec C1), T_y = (d1, F_a, C3, rec depot), T_z = (d2, F_b, C4, rec depot); every gap holds an FLP.
struct FixF { std::unique_ptr<Fix> f; Solution base; NodeId Fa = 0, Fb = 0; };
FixF make_F() {
    FixF x;
    x.f = make_line(4, 3);
    x.Fa = x.f->flp_at(0, 1, 0);
    x.Fb = x.f->flp_at(1, 0, 0);
    x.base.truck_route = {0, x.Fa, 1, x.Fb, 0};
    x.base.drone_trips = {trip(0, x.Fa, 2, 1), trip(1, x.Fa, 3, 0), trip(2, x.Fb, 4, 0)};
    return x;
}

// All gaps free: insert the customers one by one the way the contract draws (replay on a fresh Rng).
Sk replay_free_inserts(Sk s, const std::vector<NodeId>& customers, std::uint64_t seed) {
    tdr::util::Rng r(seed);
    for (NodeId c : customers) {
        const std::size_t g = r.uniform_index(s.size() - 1);
        s.insert(s.begin() + static_cast<std::ptrdiff_t>(g + 1), c);
    }
    return s;
}

} // namespace

int main() {
    // ------------------------------------------------------------------------------------------------ fixtures sanity
    auto Q = make_Q();
    auto P = make_P();
    auto F = make_F();
    {
        bool fe = false;
        check_near(ref_objective(Q.base, *Q.f, &fe), 2025.0, 1e-9, "Q base objective (hand-derived 2025, frozen reference)");
        check(fe, "Q base feasible");
        check_near(ref_objective(P.base, *P.f, &fe), 2650.0, 1e-9, "P base objective (hand-derived 2650, frozen reference)");
        check(fe, "P base feasible");
        ref_objective(F.base, *F.f, &fe);
        check(fe, "F base feasible");
        check(P.f->idx.flps_on_arc(2, 3).size() == 1 && P.f->idx.flps_on_arc(5, 6).size() == 1 && Q.f->idx.flps_on_arc(4, 5).size() == 1 && F.f->idx.flps_on_arc(0, 1).size() == 1 && F.f->idx.flps_on_arc(1, 0).size() == 1, "fixture sanity: one FLP on each trip arc");
    }
    const ShakeOptions opt;
    check(opt.max_segment_length == 3 && opt.max_shift == 3 && opt.evaluate_result, "default options: L_max = D_max = 3, evaluate_result on (assumption A22)");

    // ------------------------------------------------------------------------------------------------ Q: no trip invalidated
    {
        tdr::util::Rng rng(1);
        const Solution before = Q.base;
        const auto r = shake_reverse(Q.base, Q.f->ctx(3), 1, 2, rng, opt);
        check(r.status == ShakeStatus::Ok && r.kind == ShakeKind::Reverse && r.seg_a == 1 && r.seg_b == 2, "Q reverse(1,2): status, kind, parameters");
        check(skel(r.solution.truck_route, *Q.f) == Sk({0, 2, 1, 3, 4, 5, 0}), "Q reverse(1,2): skeleton [0,2,1,3,4,5,0]");
        check(r.solution.truck_route == TruckRoute({0, 2, 1, 3, 4, Q.F45, 5, 0}), "Q reverse(1,2): the surviving arc (4,5) keeps its FLP: route [0,2,1,3,4,F45,5,0]");
        check(r.dissolved.empty() && r.removed.empty() && r.preserved == std::vector<std::size_t>({0}) && r.flps_lost_with_arcs.empty() && r.flps_dropped.empty() && r.reinsertions.empty(), "Q reverse(1,2): nothing dissolved, removed, lost, dropped or reinserted; the trip is preserved");
        check(r.evaluated && r.feasible && std::fabs(r.objective - 3025.0) < 1e-9, "Q reverse(1,2): objective 3025 (hand-derived), feasible");
        check(std::fabs(r.base_objective - 2025.0) < 1e-9, "Q: base objective reported (2025)");
        check_shake(r, Q.base, *Q.f, "Q reverse(1,2)");
        check(Q.base.truck_route == before.truck_route && Q.base.drone_trips.size() == 1, "Q: the input solution is unchanged");
        const auto r2 = shake_reverse(Q.base, Q.f->ctx(3), 1, 5, rng, opt);
        check(without(skel(r2.solution.truck_route, *Q.f), 6) == Sk({0, 5, 4, 3, 2, 1, 0}) && r2.dissolved.size() == 1 && r2.dissolved[0].reason == DissolveReason::LaunchArcVanished && r2.flps_lost_with_arcs == std::vector<NodeId>({Q.F45}),
              "Q reverse(1,5): the whole interior reversed; arc (4,5) becomes (5,4): launch arc vanished, FLP lost with its arc");
        check_shake(r2, Q.base, *Q.f, "Q reverse(1,5)");
        // exchange keeping the arc intact: segments [1,2] and [4,5]
        const auto e1 = shake_exchange(Q.base, Q.f->ctx(3), 1, 4, 2, rng, opt);
        check(skel(e1.solution.truck_route, *Q.f) == Sk({0, 4, 5, 3, 1, 2, 0}) && e1.dissolved.empty() && e1.preserved == std::vector<std::size_t>({0}), "Q exchange([1,2],[4,5]): [0,4,5,3,1,2,0]; arc (4,5) is inside a moved segment, so it survives and the trip is preserved");
        check(e1.solution.truck_route == TruckRoute({0, 4, Q.F45, 5, 3, 1, 2, 0}), "Q exchange([1,2],[4,5]): the FLP travels with its arc");
        check_shake(e1, Q.base, *Q.f, "Q exchange([1,2],[4,5])");
        const auto e2 = shake_exchange(Q.base, Q.f->ctx(3), 1, 3, 2, rng, opt);
        check(without(skel(e2.solution.truck_route, *Q.f), 6) == Sk({0, 3, 4, 1, 2, 5, 0}) && e2.dissolved.size() == 1 && e2.dissolved[0].reason == DissolveReason::LaunchArcVanished, "Q exchange([1,2],[3,4]): [0,3,4,1,2,5,0]; arc (4,5) vanished");
        check_shake(e2, Q.base, *Q.f, "Q exchange([1,2],[3,4])");
        // shift
        const auto s1 = shake_shift(Q.base, Q.f->ctx(3), 1, 1, 4, rng, opt);
        check(skel(s1.solution.truck_route, *Q.f) == Sk({0, 2, 3, 4, 5, 1, 0}) && s1.dissolved.empty() && s1.solution.truck_route == TruckRoute({0, 2, 3, 4, Q.F45, 5, 1, 0}), "Q shift(a=1,L=1,+4): node 1 moves to the end [0,2,3,4,5,1,0]; arc (4,5) and the trip survive");
        check_shake(s1, Q.base, *Q.f, "Q shift(1,1,+4)");
        const auto s2 = shake_shift(Q.base, Q.f->ctx(3), 4, 2, -2, rng, opt);
        check(skel(s2.solution.truck_route, *Q.f) == Sk({0, 1, 4, 5, 2, 3, 0}) && s2.dissolved.empty(), "Q shift(a=4,L=2,-2): segment [4,5] moves back 2 nodes: [0,1,4,5,2,3,0]; the arc (4,5) survives");
        check_shake(s2, Q.base, *Q.f, "Q shift(4,2,-2)");
        const auto s3 = shake_shift(Q.base, Q.f->ctx(3), 2, 2, 2, rng, opt);
        check(skel(s3.solution.truck_route, *Q.f) == Sk({0, 1, 4, 5, 2, 3, 0}), "Q shift(a=2,L=2,+2): the two jumped nodes go in front: same skeleton [0,1,4,5,2,3,0]");
    }

    // ------------------------------------------------------------------------------------------------ P: dissolution, reasons, reinsertion
    {
        const Fix& f = *P.f;
        const auto S = [&](const Solution& s) { return skel(s.truck_route, f); };
        for (std::uint64_t seed : {1ULL, 2ULL, 3ULL, 7ULL, 11ULL}) {
            const std::string t = "P seed " + std::to_string(seed);
            // reverse(1,2): [0,2,1,3,5,6,0]: T1's arc (2,3) vanishes; T2's arc (5,6) survives
            tdr::util::Rng rng(seed);
            const auto r = shake_reverse(P.base, f.ctx(3), 1, 2, rng, opt);
            check(r.status == ShakeStatus::Ok && r.dissolved.size() == 1 && r.dissolved[0].trip == 0 && r.dissolved[0].reason == DissolveReason::LaunchArcVanished && r.dissolved[0].customer == 4 && r.dissolved[0].launch == P.F23 && r.dissolved[0].recovery == 5 && r.dissolved[0].drone_id == 0,
                  t + " reverse(1,2): T1 dissolved with LaunchArcVanished (record: trip, customer, launch, recovery, drone)");
            check(r.preserved == std::vector<std::size_t>({1}) && r.flps_lost_with_arcs == std::vector<NodeId>({P.F23}) && r.flps_dropped.empty(), t + " reverse(1,2): T2 preserved; F23 lost with its arc; nothing dropped");
            tdr::util::Rng pr(seed);
                // free gaps of [0,2,1,3,5,6,0] with F56 in gap 4 are {0,1,2,3,5}
            const std::size_t fg[5] = {0, 1, 2, 3, 5};
            const std::size_t g = fg[pr.uniform_index(5)];
            Sk want = {0, 2, 1, 3, 5, 6, 0};
            want.insert(want.begin() + static_cast<std::ptrdiff_t>(g + 1), 4);
            check(r.reinsertions.size() == 1 && r.reinsertions[0].customer == 4 && r.reinsertions[0].free_gaps == 5 && !r.reinsertions[0].fallback && r.reinsertions[0].after == Sk({0, 2, 1, 3, 5, 6, 0})[g], t + " reverse(1,2): one reinsertion into one of the 5 free gaps (draw replayed)");
            check(S(r.solution) == want && r.solution.drone_trips.size() == 1 && same_trip(r.solution.drone_trips[0], P.base.drone_trips[1]), t + " reverse(1,2): resulting skeleton as predicted; T2 untouched");
            check_shake(r, P.base, f, t + " reverse(1,2)");
        }
        {
            // exchange([1],[4]): [0,5,2,3,1,6,0]: T1 launch arc (2,3) survives but its recovery C5 is now before it (R2); T2's arc (5,6) vanished (R1)
            tdr::util::Rng rng(5);
            const auto r = shake_exchange(P.base, f.ctx(3), 1, 4, 1, rng, opt);
            check(r.status == ShakeStatus::Ok && r.dissolved.size() == 2 && r.dissolved[0].trip == 0 && r.dissolved[0].reason == DissolveReason::RecoveryNotAfterLaunch && r.dissolved[1].trip == 1 && r.dissolved[1].reason == DissolveReason::LaunchArcVanished,
                  "P exchange([1],[4]): T1 RecoveryNotAfterLaunch (its launch arc survived), T2 LaunchArcVanished, in input-trip order");
            check(r.flps_lost_with_arcs == std::vector<NodeId>({P.F56}) && r.flps_dropped == std::vector<NodeId>({P.F23}) && r.preserved.empty() && r.solution.drone_trips.empty(), "P exchange([1],[4]): F56 lost with its arc; F23 (arc survived, trip dissolved) dropped as idle; no trip left");
            const Sk want = replay_free_inserts(Sk({0, 5, 2, 3, 1, 6, 0}), {4, 7}, 5);
            check(skel(r.solution.truck_route, f) == want && r.solution.truck_route == want && r.reinsertions.size() == 2 && r.reinsertions[0].free_gaps == 6 && r.reinsertions[1].free_gaps == 7, "P exchange([1],[4]): both customers reinserted (4 then 7) into free gaps (draws replayed); route is FLP-free");
            check_shake(r, P.base, f, "P exchange([1],[4])");
            check(r.dissolved_with(DissolveReason::RecoveryNotAfterLaunch) == 1 && r.dissolved_with(DissolveReason::LaunchArcVanished) == 1 && r.dissolved_with(DissolveReason::ArcSplitByReinsertion) == 0, "P exchange: reason counts");
        }
        {
            tdr::util::Rng rng(6);
            const auto r = shake_shift(P.base, f.ctx(3), 1, 2, 1, rng, opt); // [0,3,1,2,5,6,0]
            check(without(S(r.solution), 4) == Sk({0, 3, 1, 2, 5, 6, 0}) && r.dissolved.size() == 1 && r.dissolved[0].trip == 0 && r.preserved == std::vector<std::size_t>({1}), "P shift(1,2,+1): [0,3,1,2,5,6,0]: T1 dissolved, T2 preserved");
            Sk base = {0, 3, 1, 2, 5, 6, 0};
            tdr::util::Rng pr(6);
            const std::size_t fg[5] = {0, 1, 2, 3, 5};
            base.insert(base.begin() + static_cast<std::ptrdiff_t>(fg[pr.uniform_index(5)] + 1), 4);
            check(S(r.solution) == base, "P shift(1,2,+1): customer 4 reinserted as predicted");
            check_shake(r, P.base, f, "P shift(1,2,+1)");
        }
        {
            tdr::util::Rng rng(8);
            const auto r = shake_reverse(P.base, f.ctx(3), 2, 4, rng, opt); // [0,1,5,3,2,6,0]
            check(r.dissolved.size() == 2 && r.dissolved[0].reason == DissolveReason::LaunchArcVanished && r.dissolved[1].reason == DissolveReason::LaunchArcVanished && r.preserved.empty() && r.flps_lost_with_arcs.size() == 2, "P reverse(2,4): both trips lose their launch arcs");
            check_shake(r, P.base, f, "P reverse(2,4)");
            const auto r2 = shake_shift(P.base, f.ctx(3), 3, 2, -1, rng, opt); // [0,1,3,5,2,6,0]
            check(without(without(skel(r2.solution.truck_route, f), 4), 7) == Sk({0, 1, 3, 5, 2, 6, 0}) && r2.dissolved.size() == 2, "P shift(3,2,-1): [0,1,3,5,2,6,0] before reinsertion; both trips dissolved, both customers reinserted");
            check_shake(r2, P.base, f, "P shift(3,2,-1)");
        }
        // drone remove
        for (std::uint64_t seed : {1ULL, 4ULL, 9ULL}) {
            tdr::util::Rng rng(seed);
            const auto r = shake_drone_remove(P.base, f.ctx(3), 0, rng, opt);
            const std::string t = "P remove drone 0 seed " + std::to_string(seed);
            check(r.status == ShakeStatus::Ok && r.kind == ShakeKind::DroneRemove && r.drone == 0 && r.removed.size() == 1 && r.removed[0].trip == 0 && r.removed[0].customer == 4 && r.dissolved.empty() && r.preserved == std::vector<std::size_t>({1}), t + ": T1 removed by the shake (not 'dissolved'), T2 preserved");
            check(r.flps_dropped == std::vector<NodeId>({P.F23}) && r.flps_lost_with_arcs.empty(), t + ": F23 dropped explicitly as idle");
            // skeleton [0,1,2,3,5,6,0], F56 in gap 4: free {0,1,2,3,5}
            tdr::util::Rng pr(seed);
            const std::size_t fg[5] = {0, 1, 2, 3, 5};
            Sk want = {0, 1, 2, 3, 5, 6, 0};
            want.insert(want.begin() + static_cast<std::ptrdiff_t>(fg[pr.uniform_index(5)] + 1), 4);
            check(skel(r.solution.truck_route, f) == want && r.reinsertions.size() == 1 && r.reinsertions[0].free_gaps == 5 && !r.reinsertions[0].fallback, t + ": customer 4 reinserted into a free gap (draw replayed), route skeleton otherwise unchanged");
            check_shake(r, P.base, f, t);
        }
        {
            tdr::util::Rng rng(2);
            const auto r = shake_drone_remove(P.base, f.ctx(3), 1, rng, opt);
            check(r.removed.size() == 1 && r.removed[0].customer == 7 && r.flps_dropped == std::vector<NodeId>({P.F56}) && r.reinsertions.size() == 1 && r.reinsertions[0].free_gaps == 5, "P remove drone 1: T2 removed, F56 dropped, 5 free gaps (all but the one holding F23)");
            check_shake(r, P.base, f, "P remove drone 1");
            check(shake_drone_remove(P.base, f.ctx(3), 2, rng, opt).status == ShakeStatus::InvalidParameters, "P remove a drone without trips: InvalidParameters");
        }
    }

    // ------------------------------------------------------------------------------------------------ F: no free gap -> fallback
    {
        const Fix& f = *F.f;
        bool all = true, order = true;
        std::set<int> g_seen;
        for (std::uint64_t seed = 1; seed <= 40; ++seed) {
            tdr::util::Rng rng(seed), pr(seed);
            const auto r = shake_drone_remove(F.base, f.ctx(3), 0, rng, opt);
            const std::size_t g = pr.uniform_index(2);
            const std::size_t r2 = pr.uniform_index(2);
            g_seen.insert(static_cast<int>(g));
            all = all && r.status == ShakeStatus::Ok && r.removed.size() == 1 && r.removed[0].customer == 2 && r.dissolved.size() == 1 && r.dissolved[0].reason == DissolveReason::ArcSplitByReinsertion && r.reinsertions.size() == 2 && r.reinsertions[0].fallback && r.reinsertions[0].free_gaps == 0 && !r.reinsertions[1].fallback && r.reinsertions[1].free_gaps == 2;
            Sk want;
            if (g == 0) {
                // T_y (launch F_a in gap 0) dissolved: queue [2,3]; skeleton [0,2,1,0]; free gaps {0,1}
                all = all && r.dissolved[0].trip == 1 && r.dissolved[0].customer == 3 && r.flps_dropped == std::vector<NodeId>({F.Fa}) && r.preserved == std::vector<std::size_t>({2});
                want = (r2 == 0) ? Sk({0, 3, 2, 1, 0}) : Sk({0, 2, 3, 1, 0});
            } else {
                // T_z (launch F_b in gap 1) dissolved: queue [2,4]; skeleton [0,1,2,0]; free gaps {1,2}
                all = all && r.dissolved[0].trip == 2 && r.dissolved[0].customer == 4 && r.flps_dropped == std::vector<NodeId>({F.Fb}) && r.preserved == std::vector<std::size_t>({1});
                want = (r2 == 0) ? Sk({0, 1, 4, 2, 0}) : Sk({0, 1, 2, 4, 0});
            }
            order = order && skel(r.solution.truck_route, f) == want;
            check_shake(r, F.base, f, "F fallback seed " + std::to_string(seed));
        }
        check(all, "F remove drone 0 (its FLP is shared, every gap holds an FLP): fallback draw, ArcSplitByReinsertion dissolves exactly the trips launching in the drawn gap, FLP dropped, second customer into a free gap (40 seeds, replayed)");
        check(order, "F fallback: resulting skeletons as predicted (40 seeds)");
        check(g_seen.size() == 2, "F fallback: both gaps are drawn over the seeds (non-vacuous)");
        tdr::util::Rng rng(3);
        check(shake_reverse(F.base, f.ctx(3), 1, 2, rng, opt).status == ShakeStatus::InvalidParameters, "F (m = 1): explicit reverse(1,2) is InvalidParameters");
        check(shake_random_reverse(F.base, f.ctx(3), rng, opt).status == ShakeStatus::NotApplicable && shake_random_exchange(F.base, f.ctx(3), rng, opt).status == ShakeStatus::NotApplicable && shake_random_shift(F.base, f.ctx(3), rng, opt).status == ShakeStatus::NotApplicable, "F (m = 1): the three route shakes are NotApplicable");
    }

    // ------------------------------------------------------------------------------------------------ parameter validation, statuses, flags
    {
        const Fix& f = *Q.f;
        tdr::util::Rng rng(1);
        const auto bad = [&](ShakeResult r) { return r.status == ShakeStatus::InvalidParameters; };
        check(bad(shake_reverse(Q.base, f.ctx(3), 2, 2, rng)) && bad(shake_reverse(Q.base, f.ctx(3), 0, 2, rng)) && bad(shake_reverse(Q.base, f.ctx(3), 3, 2, rng)) && bad(shake_reverse(Q.base, f.ctx(3), 1, 6, rng)), "reverse: a == b, a == 0, a > b, b > m are InvalidParameters");
        check(bad(shake_exchange(Q.base, f.ctx(3), 1, 2, 2, rng)) && bad(shake_exchange(Q.base, f.ctx(3), 1, 3, 0, rng)) && bad(shake_exchange(Q.base, f.ctx(3), 2, 5, 2, rng)) && bad(shake_exchange(Q.base, f.ctx(3), 3, 1, 1, rng)), "exchange: overlapping, zero-length, beyond the end, unordered segments are InvalidParameters");
        check(!bad(shake_exchange(Q.base, f.ctx(3), 1, 2, 1, rng)), "exchange: adjacent segments are allowed");
        check(bad(shake_shift(Q.base, f.ctx(3), 1, 1, 0, rng)) && bad(shake_shift(Q.base, f.ctx(3), 1, 1, -1, rng)) && bad(shake_shift(Q.base, f.ctx(3), 4, 2, 1, rng)) && bad(shake_shift(Q.base, f.ctx(3), 1, 5, 1, rng)) && !bad(shake_shift(Q.base, f.ctx(3), 1, 4, 1, rng)), "shift: zero, out-of-range and whole-route moves are InvalidParameters, a legal edge move is not");
        Solution omitted = Q.base;
        omitted.drone_trips.clear();
        omitted.truck_route = {0, 1, 2, 0};
        check(shake_reverse(omitted, f.ctx(3), 1, 2, rng).status == ShakeStatus::InputInfeasible && shake_random_drone_remove(omitted, f.ctx(3), rng).status == ShakeStatus::InputInfeasible && shake(omitted, f.ctx(3), rng).status == ShakeStatus::InputInfeasible, "an infeasible input (omitted customers) is InputInfeasible for every entry point, nothing generated");
        Solution alltruck;
        alltruck.truck_route = {0, 1, 2, 3, 4, 5, 6, 0};
        check(shake_random_drone_remove(alltruck, f.ctx(3), rng).status == ShakeStatus::NotApplicable, "Random_Drone_Remove without any trip is NotApplicable");
        const auto rr = shake_random_reverse(alltruck, f.ctx(3), rng);
        check(rr.status == ShakeStatus::Ok && rr.solution.drone_trips.empty() && rr.dissolved.empty() && rr.reinsertions.empty() && rr.preserved.empty() && exactly_once(rr.solution, 6), "an all-truck solution is shaken without any dissolution");
        // flags
        tdr::util::Rng a(1), b(1);
        const auto with = shake_reverse(Q.base, f.ctx(3), 1, 5, a, ShakeOptions{});
        ShakeOptions noeval;
        noeval.evaluate_result = false;
        const auto wo = shake_reverse(Q.base, f.ctx(3), 1, 5, b, noeval);
        check(with.evaluated && !wo.evaluated && with.solution.truck_route == wo.solution.truck_route && with.dissolved.size() == wo.dissolved.size() && a.next_u64() == b.next_u64(), "evaluate_result = false: same solution, same draws, no judgement");
        // determinism
        tdr::util::Rng c(9), d(9);
        const auto x = shake_exchange(P.base, P.f->ctx(3), 1, 4, 1, c), y = shake_exchange(P.base, P.f->ctx(3), 1, 4, 1, d);
        check(x.solution.truck_route == y.solution.truck_route && x.objective == y.objective && x.reinsertions.size() == y.reinsertions.size(), "deterministic: same seed, same result");
    }

    // ------------------------------------------------------------------------------------------------ randomised entries: draw order, support, options
    {
        const Fix& f = *Q.f;
        bool replay = true, draws = true;
        std::set<std::pair<int, int>> rev_pairs;
        std::set<std::array<int, 3>> ex_combos, sh_combos;
        std::set<int> sh_dirs;
        for (std::uint64_t seed = 1; seed <= 4000; ++seed) {
            {
                tdr::util::Rng r1(seed), r2(seed);
                const auto a = shake_random_reverse(Q.base, f.ctx(3), r1);
                r2.uniform_index(5); r2.uniform_index(4);
                const auto b = shake_reverse(Q.base, f.ctx(3), a.seg_a, a.seg_b, r2);
                replay = replay && a.solution.truck_route == b.solution.truck_route && a.seg_a < a.seg_b && a.seg_b <= 5;
                draws = draws && r1.next_u64() == r2.next_u64();
                rev_pairs.insert({static_cast<int>(a.seg_a), static_cast<int>(a.seg_b)});
            }
            {
                tdr::util::Rng r1(seed), r2(seed);
                const auto a = shake_random_exchange(Q.base, f.ctx(3), r1);
                r2.uniform_int(1, 2); r2.uniform_index(a.seg_len == 1 ? 10 : 3);
                const auto b = shake_exchange(Q.base, f.ctx(3), a.seg_a, a.seg_b, a.seg_len, r2);
                replay = replay && a.solution.truck_route == b.solution.truck_route && a.seg_len >= 1 && a.seg_len <= 2;
                draws = draws && r1.next_u64() == r2.next_u64();
                ex_combos.insert({static_cast<int>(a.seg_a), static_cast<int>(a.seg_b), static_cast<int>(a.seg_len)});
            }
            {
                tdr::util::Rng r1(seed), r2(seed);
                const auto a = shake_random_shift(Q.base, f.ctx(3), r1);
                const long L = static_cast<long>(a.seg_len), A = static_cast<long>(a.seg_a);
                r2.uniform_int(1, 3); r2.uniform_int(1, 5 - L + 1);
                const std::size_t nd = (A > 1 ? 1 : 0) + (A + L - 1 < 5 ? 1 : 0);
                r2.uniform_index(nd);
                const long room = a.shift < 0 ? A - 1 : 5 - (A + L - 1);
                r2.uniform_int(1, std::min<long>(3, room));
                const auto b = shake_shift(Q.base, f.ctx(3), a.seg_a, a.seg_len, a.shift, r2);
                replay = replay && a.solution.truck_route == b.solution.truck_route && L >= 1 && L <= 3 && std::labs(a.shift) >= 1 && std::labs(a.shift) <= 3 && a.shift != 0;
                draws = draws && r1.next_u64() == r2.next_u64();
                sh_combos.insert({static_cast<int>(a.seg_a), static_cast<int>(a.seg_len), static_cast<int>(a.shift)});
                sh_dirs.insert(a.shift > 0 ? 1 : -1);
            }
        }
        check(replay, "random entries: the result equals the explicit call with the recorded parameters after exactly the documented draws");
        check(draws, "random entries: the documented number of draws is consumed (RNG state equal after replay)");
        check(rev_pairs.size() == 10, "Random_Reverse over 4000 seeds: all 10 unordered pairs of 5 nodes occur");
        std::size_t want_ex = 0, want_sh = 0;
        for (int L = 1; L <= 2; ++L) for (int a = 1; a + 2 * L - 1 <= 5; ++a) for (int b = a + L; b + L - 1 <= 5; ++b) ++want_ex;
        for (int L = 1; L <= 3; ++L) for (int a = 1; a + L - 1 <= 5; ++a) for (int d = -3; d <= 3; ++d) if (d != 0 && (d > 0 ? a + L - 1 + d <= 5 : a + d >= 1)) ++want_sh;
        check(ex_combos.size() == want_ex && want_ex == 13, "Random_Exchange: all 13 (a, b, L) combinations of m = 5, L <= 2 occur and no other");
        check(sh_combos.size() == want_sh && sh_dirs.size() == 2, "Random_Shift: every legal (a, L, delta) with L <= 3 and |delta| <= 3 occurs (" + std::to_string(want_sh) + ") and both directions");
        // options
        bool lens = true, shifts = true;
        ShakeOptions o1;
        o1.max_segment_length = 1;
        o1.max_shift = 1;
        for (std::uint64_t seed = 1; seed <= 300; ++seed) {
            tdr::util::Rng r(seed);
            const auto e = shake_random_exchange(Q.base, f.ctx(3), r, o1);
            const auto s = shake_random_shift(Q.base, f.ctx(3), r, o1);
            lens = lens && e.seg_len == 1 && s.seg_len == 1;
            shifts = shifts && std::labs(s.shift) == 1;
        }
        check(lens && shifts, "options: max_segment_length = 1 and max_shift = 1 restrict the draws");
        ShakeOptions o0;
        o0.max_segment_length = 0;
        o0.max_shift = -4;
        tdr::util::Rng r0(3);
        const auto z = shake_random_shift(Q.base, f.ctx(3), r0, o0);
        check(z.status == ShakeStatus::Ok && z.seg_len == 1 && std::labs(z.shift) == 1, "options: values below 1 act as 1");
        // Random_Drone_Remove: the drone id is uniform over ids that fly a trip
        std::set<int> ids;
        for (std::uint64_t seed = 1; seed <= 60; ++seed) {
            tdr::util::Rng r(seed), r2(seed);
            const auto a = shake_random_drone_remove(P.base, P.f->ctx(3), r);
            const int id = static_cast<int>(r2.uniform_index(2));
            ids.insert(a.drone);
            check(a.drone == id && a.status == ShakeStatus::Ok, "random drone remove seed " + std::to_string(seed) + ": the drone is the uniform_index(#drone ids with trips)-th");
        }
        check(ids == std::set<int>({0, 1}), "Random_Drone_Remove: both drones are drawn");
        // shake(): first draw selects the kind uniformly
        std::set<int> kinds;
        bool first = true;
        for (std::uint64_t seed = 1; seed <= 200; ++seed) {
            tdr::util::Rng r(seed), r2(seed);
            const auto a = shake(P.base, P.f->ctx(3), r);
            const std::size_t k = r2.uniform_index(4);
            kinds.insert(static_cast<int>(k));
            first = first && static_cast<std::size_t>(a.kind) == k && a.status == ShakeStatus::Ok;
        }
        check(first && kinds.size() == 4, "shake(): the first draw selects the kind uniformly among the four (200 seeds, all four occur)");
        Solution alltruck;
        alltruck.truck_route = {0, 1, 2, 3, 4, 5, 6, 0};
        bool napp = true, seen_na = false, seen_ok = false;
        for (std::uint64_t seed = 1; seed <= 100; ++seed) {
            tdr::util::Rng r(seed), r2(seed);
            const std::size_t k = r2.uniform_index(4);
            const auto a = shake(alltruck, Q.f->ctx(3), r);
            if (k == 3) { napp = napp && a.status == ShakeStatus::NotApplicable && a.kind == ShakeKind::DroneRemove; seen_na = true; }
            else { napp = napp && a.status == ShakeStatus::Ok; seen_ok = true; }
        }
        check(napp && seen_na && seen_ok, "shake() on an all-truck solution: a drawn Random_Drone_Remove is NotApplicable (no redraw, no substitution), the others act");
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
