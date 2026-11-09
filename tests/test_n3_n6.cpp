// M4 tests: neighbourhoods N3 (drone service node relocate: consecutive trips of one drone) and N6 (D-D service node
// swap: adjacent trips of other drones). HAND-DERIVED fixtures on a line, so every expected number below was worked out
// on paper and is not read back from the code under test:
//
//   * depot 0 at x = 0; customer Ci at x = 1000 i metres; truck 10 m/s, drone 20 m/s unless stated, SL = SR = 0, service
//     times 0, theta = 0, d = 600 m => floor(D/600) FLPs per arc (1000 m: 1 FLP at the midpoint; 2000 m: 3 FLPs at
//     1/4, 2/4, 3/4 of the arc; 3000 m: 5 at k/6).
//   * FLP pool = every ordered pair of truck nodes (the eager pool).
//   * truck arrival at x metres along the line, no waiting: x / 10 s; a truck that reaches a recovery node before its
//     drone waits for it (SR = 0), which delays everything after that node.
//   * endurance (36) = (t_k - l_j) + loaded, (37) = loaded + return flight (timing_engine.hpp), loaded = outbound flight.
//
// Independent cross-checks use the FROZEN reference evaluator (tdr::timing_ref), not the production one. M1-M3
// behaviour is covered by test_n4_n5.cpp and test_n7_n8.cpp and is not repeated here.

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
#include "search/neighbourhood_n3.hpp"
#include "search/neighbourhood_n6.hpp"
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

constexpr double kTol = 1e-9;

// The two trips exchanged their drone customers; the route, every launch/recovery node, every drone id and all other
// trips are byte-identical (the whole of the N3/N6 transformation).
bool exchanged_only(const Solution& base, const Solution& cand, std::size_t i, std::size_t j) {
    if (cand.truck_route != base.truck_route || cand.drone_trips.size() != base.drone_trips.size()) return false;
    for (std::size_t k = 0; k < base.drone_trips.size(); ++k) {
        const auto &b = base.drone_trips[k], &c = cand.drone_trips[k];
        if (b.drone_id != c.drone_id || b.launch_node != c.launch_node || b.recovery_node != c.recovery_node) return false;
        const NodeId expect = k == i ? base.drone_trips[j].service_customer : (k == j ? base.drone_trips[i].service_customer : b.service_customer);
        if (c.service_customer != expect) return false;
    }
    return true;
}

std::vector<std::size_t> partners_of(const OperatorResult& r) {
    std::vector<std::size_t> out;
    for (const auto& rec : r.log) out.push_back(rec.move.partner_trip);
    return out;
}

using Idx = std::vector<std::size_t>;

// Index of the first record with the lowest feasible objective (strict <, first generated wins), or log.size().
std::size_t first_best(const OperatorResult& r) {
    std::size_t best = r.log.size();
    for (std::size_t i = 0; i < r.log.size(); ++i)
        if (r.log[i].reason == RejectReason::None && (best == r.log.size() || r.log[i].objective < r.log[best].objective)) best = i;
    return best;
}

std::size_t construction_reasons(const OperatorResult& r) { // reasons that cannot occur: nothing is disturbed
    std::size_t n = 0;
    for (RejectReason x : {RejectReason::AffectedTrips, RejectReason::NoFreeDrone, RejectReason::NoReplacementFlp, RejectReason::InvalidRecovery, RejectReason::EvalFlpArc, RejectReason::EvalFlpUnused})
        n += r.rejected[static_cast<std::size_t>(x)];
    return n;
}

// Every record of an N3/N6 result against the FROZEN reference evaluator and the model.
void check_records(const OperatorResult& res, const Solution& base, const Fix& f, const std::string& tag) {
    bool swap_only = true, cover = true, arcs = true, obj = true, verdict = true, fields = true, reasons = true;
    std::set<std::vector<NodeId>> sigs;
    for (const auto& rec : res.log) {
        const std::size_t i = rec.move.replaced_trip, j = rec.move.partner_trip;
        swap_only = swap_only && i < base.drone_trips.size() && j < base.drone_trips.size() && i != j && exchanged_only(base, rec.solution, i, j);
        cover = cover && exactly_once(rec.solution, f.n);
        arcs = arcs && !has_foreign_flp(rec.solution.truck_route, f);
        fields = fields && i < base.drone_trips.size() && j < base.drone_trips.size() && rec.move.customer == base.drone_trips[i].service_customer &&
                 rec.move.partner == base.drone_trips[j].service_customer && (rec.move.kind == MoveKind::N3Exchange || rec.move.kind == MoveKind::N6Swap);
        std::vector<NodeId> sig;
        for (const auto& t : rec.solution.drone_trips) sig.push_back(t.service_customer);
        sigs.insert(sig);
        if (rec.evaluated) {
            bool feas = false;
            const double o = ref_objective(rec.solution, f, &feas);
            obj = obj && std::fabs(o - rec.objective) <= 1e-9 * std::max(1.0, std::fabs(o));
            verdict = verdict && (feas == (rec.reason == RejectReason::None));
        }
        reasons = reasons && (rec.reason == RejectReason::None || rec.reason == RejectReason::EvalEndurance || rec.reason == RejectReason::EvalFleetCapacity ||
                              rec.reason == RejectReason::EvalDroneSequencing || rec.reason == RejectReason::EvalTripOrder || rec.reason == RejectReason::EvalOther);
    }
    check(swap_only, tag + ": every candidate = the two trips' customers exchanged, nothing else changed (route, launch, recovery, drone ids)");
    check(cover, tag + ": exactly-once customer coverage in every record");
    check(arcs, tag + ": no FLP leaves its arc in any record (independent check)");
    check(fields, tag + ": move descriptors name the two trips and their original customers");
    check(obj, tag + ": production objective == frozen reference objective in every evaluated record");
    check(verdict, tag + ": production verdict == frozen reference verdict in every evaluated record");
    check(reasons && construction_reasons(res) == 0, tag + ": only evaluator reasons occur (AffectedTrips/NoFreeDrone/NoReplacementFlp/InvalidRecovery/FlpArc/FlpUnused = 0)");
    check(res.accepted + res.rejected_total() == res.generated, tag + ": accepted + rejected == generated (one outcome per candidate)");
    check(res.evaluated == res.generated, tag + ": every candidate is evaluated");
    check(res.duplicates_skipped == 0 && sigs.size() == res.log.size() && res.log.size() == res.generated, tag + ": no duplicate candidates; one record per generated candidate");
    const std::size_t fb = first_best(res);
    if (fb == res.log.size()) {
        check(!res.best && res.accepted == 0, tag + ": no feasible candidate -> no best");
    } else {
        check(res.best && res.best->objective == res.log[fb].objective && res.best->move.partner_trip == res.log[fb].move.partner_trip && same_solution(res.best->solution, res.log[fb].solution),
              tag + ": best = the first-generated lowest feasible candidate (strict <)");
    }
}

// ---- fixtures ----

// n = 7, truck skeleton [0,1,3,5,7,0], drone customers C2, C4, C6, route [0,F1,1,3,F2,5,F3,7,0],
//   F1 = x 500 (arc (0,1)), F2 = x 3500 (arc (3,5), 1st of 3), F3 = x 5500 (arc (5,7), 1st of 3).
//   T1 = (F1, C2, C3), T2 = (F2, C4, C5), T3 = (F3, C6, C7). Trip VECTOR order is [T3, T1, T2] (so index != drone-sequence order).
//   Half-steps (skeleton 0,1,3,5,7,0 -> gaps g0..g4): T1 in g0 -> h = 1, T2 in g2 -> h = 5, T3 in g3 -> h = 7.
struct Chain {
    std::unique_ptr<Fix> f;
    Solution base;
    NodeId F1 = 0, F2 = 0, F3 = 0;
};
constexpr std::size_t kT3 = 0, kT1 = 1, kT2 = 2;

// `order` lists the trips (0 = T1, 1 = T2, 2 = T3) in vector order; the default is [T3, T1, T2].
Chain make_chain(int units, double drone_speed, int d1, int d2, int d3, std::array<int, 3> order = {2, 0, 1}) {
    Chain c;
    c.f = make_line(7, units, 3600.0, 600.0, 1000.0, drone_speed);
    c.F1 = c.f->flp_at(0, 1, 0);
    c.F2 = c.f->flp_at(3, 5, 0);
    c.F3 = c.f->flp_at(5, 7, 0);
    c.base.truck_route = {0, c.F1, 1, 3, c.F2, 5, c.F3, 7, 0};
    const DroneTrip all[3] = {trip(d1, c.F1, 2, 3), trip(d2, c.F2, 4, 5), trip(d3, c.F3, 6, 7)};
    for (int k : order) c.base.drone_trips.push_back(all[k]);
    return c;
}

} // namespace

int main() {
    // =====================================================================
    // N3-1. Fig. 10c: two consecutive trips of ONE drone exchange their customers. n = 5, |U| = 1, endurance 3600,
    //       route [0,Fa,1,3,Fb,5,0]; Fa = FLP of L(0,1) (x 500), Fb = first FLP of L(3,5) (x 3500).
    //       Base: A = (Fa, C4, C3), B = (Fb, C2, C5)  -- "{C1,C4,C3} and {C3,C2,C5}".
    //         truck C1 100, C3 300, C5 500 = 900.
    //         A: launch 50, C4 (x 4000) +175 = 225, back to C3 +50 = 275 < 300.   B: launch 350, C2 (x 2000) +75 = 425, back to C5 +150 = 575 > 500 (truck waits; depot only).
    //         base objective 900 + 225 + 425 = 1550.
    //       Exchange: A = (Fa, C2, C3): 50 + 75 = 125, back +50 = 175.  B = (Fb, C4, C5): 350 + 25 = 375, back +50 = 425 < 500.
    //         objective 900 + 125 + 375 = 1400  (< 1550: an improvement).
    // =====================================================================
    {
        auto f = make_line(5, 1);
        const NodeId Fa = f->flp_at(0, 1, 0), Fb = f->flp_at(3, 5, 0);
        check(f->idx.flps_on_arc(0, 1).size() == 1 && f->idx.flps_on_arc(3, 5).size() == 3 && std::fabs(f->flp_x(Fa) - 500) < 1e-9 && std::fabs(f->flp_x(Fb) - 3500) < 1e-9, "N3-1 fixture sanity: Fa at x 500, Fb at x 3500");
        Solution base;
        base.truck_route = {0, Fa, 1, 3, Fb, 5, 0};
        base.drone_trips = {trip(0, Fa, 4, 3), trip(0, Fb, 2, 5)};
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 1550.0, kTol, "N3-1 base objective (hand-derived 1550, frozen reference)");
        check(feas, "N3-1 base is feasible in the frozen reference");
        const Solution before = base;
        N3Options o;
        o.record_candidates = true;

        const auto r0 = n3_drone_service_node_relocate(base, f->ctx(3), 0, o);
        check(r0.status == OperatorStatus::Ok && r0.op == OperatorId::N3 && r0.anchor_trip == 0 && r0.anchor_customer == 4, "N3-1 anchor A: status, operator id, anchor trip and customer");
        check_near(r0.base_objective, 1550.0, kTol, "N3-1 base objective reported");
        check(r0.generated == 1 && r0.evaluated == 1 && r0.accepted == 1 && r0.log.size() == 1 && r0.rejected_total() == 0, "N3-1 exactly one candidate (A has no predecessor; B is its successor), accepted");
        check(r0.log[0].move.kind == MoveKind::N3Exchange && r0.log[0].move.replaced_trip == 0 && r0.log[0].move.partner_trip == 1 && r0.log[0].move.customer == 4 && r0.log[0].move.partner == 2, "N3-1 move descriptor: A (C4) <-> B (C2)");
        check_near(r0.log[0].objective, 1400.0, kTol, "N3-1 exchanged objective (hand-derived 1400)");
        check(r0.best && r0.improves() && std::fabs(r0.best->objective - 1400.0) < kTol, "N3-1 the exchange is a strict improvement (1400 < 1550)");
        Solution expect = base;
        expect.drone_trips[0].service_customer = 2;
        expect.drone_trips[1].service_customer = 4;
        check(r0.best && same_solution(r0.best->solution, expect), "N3-1 result: A = (Fa, C2, C3), B = (Fb, C4, C5); route, FLPs, recoveries, drone ids unchanged");
        check_records(r0, base, *f, "N3-1 anchor A");

        const auto r1 = n3_drone_service_node_relocate(base, f->ctx(3), 1, o);
        check(r1.status == OperatorStatus::Ok && r1.anchor_trip == 1 && r1.anchor_customer == 2 && r1.generated == 1 && r1.log.size() == 1, "N3-1 anchor B: one candidate (its predecessor A)");
        check(r1.log[0].move.replaced_trip == 1 && r1.log[0].move.partner_trip == 0 && r1.log[0].move.customer == 2 && r1.log[0].move.partner == 4, "N3-1 anchor B: move descriptor B (C2) <-> A (C4)");
        check_near(r1.log[0].objective, 1400.0, kTol, "N3-1 anchor B: the same exchange, the same objective");
        check(r1.best && same_solution(r1.best->solution, expect), "N3-1 anchor B: the same resulting solution");
        check_records(r1, base, *f, "N3-1 anchor B");
        check(same_solution(base, before), "N3-1 the input solution is unchanged");
    }

    // =====================================================================
    // N3-2. THREE trips of one drone, consecutive vs non-consecutive. n = 7, |U| = 1, route [0,F1,1,3,F2,5,F3,7,0],
    //       F1 x 500, F2 x 3500, F3 x 5500;  T1 = (F1, C2, C3), T2 = (F2, C4, C5), T3 = (F3, C6, C7).
    //       Trip VECTOR order is [T3, T1, T2] so that vector adjacency differs from the drone's sequence (T1, T2, T3).
    //       Base: truck 100+300+500+700 = 1600; T1: 50+75 = 125 (back 175 < 300); T2: 350+25 = 375 (back 425 < 500);
    //             T3: 550+25 = 575 (back 625 < 700) => drones 1075; objective 2675.
    //       T1<->T2 (T1 serves C4, T2 serves C2): T1: 50+175 = 225 (back 275 < 300).  T2: launch 350, C2 +75 = 425, back to C5 +150 = 575 > 500
    //            => the truck waits 75 at C5: F3 at 625, C7 at 775.  T3: launch 625, C6 650, back 700 < 775.
    //            objective: truck 100+300+500+775 = 1675; drones 225+425+650 = 1300; total 2975.
    //       T2<->T3 (T2 serves C6, T3 serves C4): T2: 350 + 125 = 475, back to C5 +50 = 525 > 500 => truck waits 25: F3 at 575, C7 at 725.
    //            T3: launch 575, C4 (1500 m) +75 = 650, back to C7 +150 = 800 > 725 (depot only).
    //            objective: truck 100+300+500+725 = 1625; drones 125+475+650 = 1250; total 2875.
    // =====================================================================
    {
        auto c = make_chain(1, 20.0, 0, 0, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        check(std::fabs(f.flp_x(c.F1) - 500) < 1e-9 && std::fabs(f.flp_x(c.F2) - 3500) < 1e-9 && std::fabs(f.flp_x(c.F3) - 5500) < 1e-9, "N3-2 fixture sanity: F1 x 500, F2 x 3500, F3 x 5500");
        bool feas = false;
        check_near(ref_objective(base, f, &feas), 2675.0, kTol, "N3-2 base objective (hand-derived 2675)");
        check(feas, "N3-2 base feasible in the frozen reference");
        check(detail_n3::drone_sequence(base, 0) == Idx({kT1, kT2, kT3}), "N3-2 drone 0's sequence is T1, T2, T3 (launch order), not the vector order T3, T1, T2");

        N3Options o;
        o.record_candidates = true;
        const auto mid = n3_drone_service_node_relocate(base, f.ctx(3), kT2, o);
        check(mid.status == OperatorStatus::Ok && mid.anchor_trip == kT2 && mid.anchor_customer == 4 && mid.generated == 2 && mid.accepted == 2, "N3-2 middle trip T2: two candidates, both feasible");
        check(partners_of(mid) == Idx({kT1, kT3}), "N3-2 order: predecessor T1 first, then successor T3 (by drone sequence, not by vector index)");
        check_near(mid.log[0].objective, 2975.0, kTol, "N3-2 T2<->T1 objective (hand-derived 2975)");
        check_near(mid.log[1].objective, 2875.0, kTol, "N3-2 T2<->T3 objective (hand-derived 2875)");
        check(mid.best && mid.best->move.partner_trip == kT3 && std::fabs(mid.best->objective - 2875.0) < kTol, "N3-2 best = the second candidate (strictly lower), not the first");
        check(!mid.improves(), "N3-2 neither exchange improves on 2675");
        check_records(mid, base, f, "N3-2 T2");

        const auto lo = n3_drone_service_node_relocate(base, f.ctx(3), kT1, o);
        check(partners_of(lo) == Idx({kT2}) && lo.generated == 1, "N3-2 first trip T1: only its successor T2 (the NON-consecutive T3 is never exchanged)");
        check_near(lo.log[0].objective, 2975.0, kTol, "N3-2 T1<->T2 objective 2975");
        check_records(lo, base, f, "N3-2 T1");
        const auto hi = n3_drone_service_node_relocate(base, f.ctx(3), kT3, o);
        check(partners_of(hi) == Idx({kT2}) && hi.generated == 1, "N3-2 last trip T3: only its predecessor T2 (T1 is not consecutive)");
        check_near(hi.log[0].objective, 2875.0, kTol, "N3-2 T3<->T2 objective 2875");
        check_records(hi, base, f, "N3-2 T3");

        // N3 does not use the D1 window (adjacency is the drone's sequence, D3)
        bool window_free = true;
        for (int w : {0, 1, 2, 3, 10}) {
            const auto r = n3_drone_service_node_relocate(base, f.ctx(w), kT2, o);
            window_free = window_free && partners_of(r) == Idx({kT1, kT3}) && r.log[0].objective == mid.log[0].objective && r.log[1].objective == mid.log[1].objective;
        }
        check(window_free, "N3-2 the adjacency window (0, 1, 2, 3, 10) has no effect on N3");
        check(same_solution(base, before), "N3-2 the input solution is unchanged");
    }

    // =====================================================================
    // N3-3. TWO drones: only trips of the SAME drone, consecutive in THAT drone's sequence. Same chain, T1 and T3 on drone 0, T2 on drone 1.
    //       Drone 0's sequence is T1, T3 (T2 lies between them in the route but belongs to drone 1).
    //       T1<->T3 (T1 serves C6, T3 serves C2): T1: launch 50, C6 +275 = 325, back to C3 +150 = 475 > 300 => the truck waits 175 at C3
    //            (departs 475): F2 at 525, C5 at 675, F3 at 725, C7 at 875.  T2 (drone 1): launch 525, C4 550, back 600 < 675.
    //            T3: launch 725, C2 (3500 m) +175 = 900, back to C7 +250 = 1150 (> 875, depot only).
    //            objective: truck 100+300+675+875 = 1950; drones 325+550+900 = 1775; total 3725.
    // =====================================================================
    {
        auto c = make_chain(2, 20.0, 0, 1, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        bool feas = false;
        check_near(ref_objective(base, f, &feas), 2675.0, kTol, "N3-3 base objective 2675 (drone ids do not change the timing)");
        check(feas, "N3-3 base feasible in the frozen reference (|U| = 2)");
        check(detail_n3::drone_sequence(base, 0) == Idx({kT1, kT3}) && detail_n3::drone_sequence(base, 1) == Idx({kT2}), "N3-3 drone sequences: drone 0 = T1, T3; drone 1 = T2");
        N3Options o;
        o.record_candidates = true;
        const auto a1 = n3_drone_service_node_relocate(base, f.ctx(3), kT1, o);
        check(partners_of(a1) == Idx({kT3}) && a1.generated == 1, "N3-3 T1: its only partner is T3 (consecutive on drone 0, although T2 lies between them in the route)");
        check_near(a1.log[0].objective, 3725.0, kTol, "N3-3 T1<->T3 objective (hand-derived 3725)");
        check(a1.accepted == 1 && a1.best && !a1.improves(), "N3-3 feasible but worse than the base");
        check_records(a1, base, f, "N3-3 T1");
        const auto a3 = n3_drone_service_node_relocate(base, f.ctx(3), kT3, o);
        check(partners_of(a3) == Idx({kT1}) && a3.generated == 1 && std::fabs(a3.log[0].objective - 3725.0) < kTol, "N3-3 T3: its only partner is T1 (predecessor on drone 0)");
        check_records(a3, base, f, "N3-3 T3");
        const auto a2 = n3_drone_service_node_relocate(base, f.ctx(3), kT2, o);
        check(a2.status == OperatorStatus::Ok && a2.generated == 0 && a2.evaluated == 0 && a2.log.empty() && !a2.best && a2.anchor_customer == 4, "N3-3 T2 is alone on drone 1: zero candidates, status Ok (a different drone's trip is never a partner)");
        check_near(a2.base_objective, 2675.0, kTol, "N3-3 T2: the base objective is still reported");
        bool same_drone_only = true;
        for (std::size_t k = 0; k < base.drone_trips.size(); ++k) {
            const auto r = n3_drone_service_node_relocate(base, f.ctx(3), k, o);
            for (const auto& rec : r.log) same_drone_only = same_drone_only && base.drone_trips[rec.move.partner_trip].drone_id == base.drone_trips[k].drone_id;
        }
        check(same_drone_only, "N3-3 every partner of every anchor belongs to the anchor's own drone");
        check(same_solution(base, before), "N3-3 the input solution is unchanged");
    }

    // =====================================================================
    // N3-4. Strict < and first-generated tie-breaking. Same chain, one drone, drone speed 100 m/s (no waiting anywhere).
    //       Base: truck 1600; T1: 50 + 1500/100 = 65; T2: 350 + 5 = 355; T3: 550 + 5 = 555 => 975; objective 2575.
    //       T1<->T2: T1 serves C4: 50+35 = 85; T2 serves C2: 350+15 = 365; T3 555 => 1005 => 2605.
    //       T2<->T3: T2 serves C6: 350+25 = 375; T3 serves C4: 550+15 = 565; T1 65   => 1005 => 2605.   EXACT tie.
    // =====================================================================
    {
        auto c = make_chain(1, 100.0, 0, 0, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        N3Options o;
        o.record_candidates = true;
        check_near(ref_objective(base, f, nullptr), 2575.0, kTol, "N3-4 base objective (hand-derived 2575)");
        const auto r = n3_drone_service_node_relocate(base, f.ctx(3), kT2, o);
        check(r.generated == 2 && partners_of(r) == Idx({kT1, kT3}), "N3-4 two candidates: predecessor, successor");
        check_near(r.log[0].objective, 2605.0, kTol, "N3-4 T2<->T1 objective 2605");
        check_near(r.log[1].objective, 2605.0, kTol, "N3-4 T2<->T3 objective 2605");
        check(r.log[0].objective == r.log[1].objective, "N3-4 the two objectives are EXACTLY equal (no tolerance anywhere)");
        check(r.best && r.best->move.partner_trip == kT1 && same_solution(r.best->solution, r.log[0].solution), "N3-4 tie -> the FIRST generated candidate (predecessor) is kept");
        check(!r.improves(), "N3-4 2605 > 2575: no improvement");
        check_records(r, base, f, "N3-4");
        check(same_solution(base, before), "N3-4 the input solution is unchanged");
    }

    // =====================================================================
    // N3-4b. The same exact tie with the trips stored in DRONE order [T1, T2, T3] (indices 0, 1, 2): anchor T2 = index 1 has the predecessor at index 0
    //        and the successor at index 2, so the SECOND tied candidate has a LARGER trip index than the anchor. First generated still wins.
    // =====================================================================
    {
        auto c = make_chain(1, 100.0, 0, 0, 0, {0, 1, 2});
        Fix& f = *c.f;
        const Solution& base = c.base;
        N3Options o;
        o.record_candidates = true;
        const auto r = n3_drone_service_node_relocate(base, f.ctx(3), 1, o);
        check(r.generated == 2 && partners_of(r) == Idx({0, 2}) && r.log[0].objective == r.log[1].objective && std::fabs(r.log[0].objective - 2605.0) < kTol, "N3-4b predecessor (index 0) and successor (index 2) tie exactly at 2605");
        check(r.best && r.best->move.partner_trip == 0 && same_solution(r.best->solution, r.log[0].solution), "N3-4b tie -> the first generated candidate (index 0), not the one with the larger trip index");
        check_records(r, base, f, "N3-4b");
    }

    // =====================================================================
    // N3-5. Endurance failure and its boundary. n = 5, |U| = 1, route [0,Fa,1,3,Fb,5,0], Fa x 500, Fb = third FLP of L(3,5) (x 4500).
    //       A = (Fa, C2, C3), B = (Fb, C4, C5).
    //       Base: A: 50 + 75 = 125, back +50 = 175; (36) = (300-125) + 75 = 250, (37) = 75+50 = 125.  B: launch 450, C4 +25 = 475, back +50 = 525 > 500;
    //             (36) = (500-475) + 25 = 50, (37) = 25 + 50 = 75.  objective 900 + 125 + 475 = 1500.
    //       A<->B: A serves C4: 50+175 = 225, back 275; (36) = 75 + 175 = 250, (37) = 175 + 50 = 225.
    //              B serves C2: launch 450, C2 (2500 m) +125 = 575, back (3000 m) +150 = 725; (36) = (500-575) + 125 = 50, (37) = 125 + 150 = 275.
    //              objective (best effort) 900 + 225 + 575 = 1700; feasible iff e >= 275.
    // =====================================================================
    {
        const auto run = [&](double e, bool* base_feasible) {
            auto f = make_line(5, 1, e);
            const NodeId Fa = f->flp_at(0, 1, 0), Fb = f->flp_at(3, 5, 2);
            Solution base;
            base.truck_route = {0, Fa, 1, 3, Fb, 5, 0};
            base.drone_trips = {trip(0, Fa, 2, 3), trip(0, Fb, 4, 5)};
            N3Options o;
            o.record_candidates = true;
            const auto r = n3_drone_service_node_relocate(base, f->ctx(3), 0, o);
            if (base_feasible) *base_feasible = r.status == OperatorStatus::Ok;
            check_records(r, base, *f, "N3-5 e=" + std::to_string(static_cast<int>(e)));
            return r;
        };
        bool bf = false;
        const auto tight = run(260.0, &bf);
        check(bf, "N3-5 the base is feasible at e = 260");
        check(tight.generated == 1 && tight.evaluated == 1 && tight.accepted == 0 && !tight.best, "N3-5 e = 260: the only candidate is rejected, no best");
        check(tight.rejected[static_cast<std::size_t>(RejectReason::EvalEndurance)] == 1 && tight.rejected_total() == 1 && tight.log[0].reason == RejectReason::EvalEndurance, "N3-5 e = 260: rejected as EvalEndurance (and only that)");
        check_near(tight.log[0].objective, 1700.0, kTol, "N3-5 the rejected candidate's best-effort objective is 1700 (hand-derived)");
        check(run(274.99, nullptr).accepted == 0, "N3-5 e = 274.99 < 275: still rejected");
        const auto exact = run(275.0, nullptr);
        check(exact.accepted == 1 && exact.best && std::fabs(exact.best->objective - 1700.0) < kTol, "N3-5 e = 275 (= (37), inclusive bound): accepted, objective 1700");
        const auto loose = run(3600.0, nullptr);
        check(loose.accepted == 1 && std::fabs(loose.best->objective - 1700.0) < kTol && !loose.improves(), "N3-5 e = 3600: accepted with the same objective 1700 -> the rejection at 260 was caused by endurance alone");
    }

    // =====================================================================
    // N3-6. Recovery at the END DEPOT is preserved. n = 4, |U| = 1, route [0,Fa,1,3,Fb,0], Fa x 500, Fb = first FLP of L(3,0) (x 2500).
    //       A = (Fa, C2, C3), B = (Fb, C4, depot).
    //       Base: truck 100 + 300; A: 125 (back 175); B: launch 350, C4 (1500 m) +75 = 425 (back 4000 m +200 = 625 > 600, depot only) => 950.
    //       A<->B: A serves C4: 50+175 = 225 (back 275 < 300); B serves C2: launch 350, +25 = 375, back (2000 m) +100 = 475 < 600 => 100+300+225+375 = 1000.
    // =====================================================================
    {
        auto f = make_line(4, 1);
        const NodeId Fa = f->flp_at(0, 1, 0), Fb = f->flp_at(3, 0, 0);
        check(f->idx.flps_on_arc(3, 0).size() == 5 && std::fabs(f->flp_x(Fb) - 2500) < 1e-9, "N3-6 fixture sanity: Fb at x 2500 on L(3,0)");
        Solution base;
        base.truck_route = {0, Fa, 1, 3, Fb, 0};
        base.drone_trips = {trip(0, Fa, 2, 3), trip(0, Fb, 4, 0)};
        const Solution before = base;
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 950.0, kTol, "N3-6 base objective 950");
        check(feas, "N3-6 base feasible in the frozen reference");
        N3Options o;
        o.record_candidates = true;
        const auto r = n3_drone_service_node_relocate(base, f->ctx(3), 0, o);
        check(r.generated == 1 && r.accepted == 1 && std::fabs(r.log[0].objective - 1000.0) < kTol, "N3-6 A<->B accepted, objective 1000 (hand-derived)");
        check(r.best && r.best->solution.drone_trips[1].recovery_node == 0 && r.best->solution.drone_trips[1].launch_node == Fb && r.best->solution.drone_trips[0].recovery_node == 3, "N3-6 the depot recovery of B and all launch nodes are unchanged");
        check_records(r, base, *f, "N3-6");
        check(same_solution(base, before), "N3-6 the input solution is unchanged");
    }

    // =====================================================================
    // N3-7. Short routes and boundaries: a lone trip, a trip-free solution, an out-of-range anchor, infeasible inputs.
    //       n = 3: route [0,Fa,1,2,0], Fa x 500, A = (Fa, C3, C2): launch 50, C3 +125 = 175, back (1000 m) +50 = 225 > 200 (truck waits); objective 100+200+175 = 475.
    // =====================================================================
    {
        auto f = make_line(3, 1);
        const NodeId Fa = f->flp_at(0, 1, 0);
        Solution lone;
        lone.truck_route = {0, Fa, 1, 2, 0};
        lone.drone_trips = {trip(0, Fa, 3, 2)};
        const Solution before = lone;
        check_near(ref_objective(lone, *f, nullptr), 475.0, kTol, "N3-7 lone-trip base objective 475");
        const auto r = n3_drone_service_node_relocate(lone, f->ctx(3), 0);
        check(r.status == OperatorStatus::Ok && r.generated == 0 && r.evaluated == 0 && !r.best && r.anchor_customer == 3, "N3-7 a lone trip has no partner: zero candidates, status Ok");
        check_near(r.base_objective, 475.0, kTol, "N3-7 base objective reported");
        const auto bad = n3_drone_service_node_relocate(lone, f->ctx(3), 1);
        check(bad.status == OperatorStatus::InvalidAnchor && bad.generated == 0, "N3-7 trip index == #trips -> InvalidAnchor, nothing generated");
        check(n3_drone_service_node_relocate(lone, f->ctx(3), 999).status == OperatorStatus::InvalidAnchor, "N3-7 a far out-of-range trip index -> InvalidAnchor");
        Solution none;
        none.truck_route = {0, 1, 2, 3, 0};
        check(n3_drone_service_node_relocate(none, f->ctx(3), 0).status == OperatorStatus::InvalidAnchor, "N3-7 no trips: index 0 -> InvalidAnchor");
        tdr::util::Rng r0(1);
        const auto ne = n3_drone_service_node_relocate(none, f->ctx(3), r0);
        check(ne.status == OperatorStatus::NoEligibleAnchor && ne.generated == 0 && ne.op == OperatorId::N3, "N3-7 Rng overload with no trips -> NoEligibleAnchor");

        auto c = make_chain(1, 20.0, 0, 0, 0);
        Solution omitted = c.base;
        omitted.drone_trips.erase(omitted.drone_trips.begin() + static_cast<std::ptrdiff_t>(kT3)); // C6 unserved (and F3 unused)
        const auto o1 = n3_drone_service_node_relocate(omitted, c.f->ctx(3), 0);
        check(o1.status == OperatorStatus::InputInfeasible && o1.generated == 0 && o1.evaluated == 0 && !o1.best, "N3-7 omitted customer (C6 unserved) -> InputInfeasible, nothing generated");
        Solution dup = c.base;
        dup.drone_trips.push_back(trip(0, c.F1, 2, 3)); // C2 served twice
        const auto o2 = n3_drone_service_node_relocate(dup, c.f->ctx(3), 0);
        check(o2.status == OperatorStatus::InputInfeasible && o2.generated == 0, "N3-7 duplicate customer (C2 served by two trips) -> InputInfeasible, nothing generated");
        check(same_solution(lone, before), "N3-7 the input solution is unchanged");
    }

    // =====================================================================
    // N3-8. Counting flags, determinism and the randomised overload.
    // =====================================================================
    {
        auto c = make_chain(1, 20.0, 0, 0, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        N3Options none;
        none.evaluate_candidates = false;
        none.record_candidates = true;
        const auto g = n3_drone_service_node_relocate(base, f.ctx(3), kT2, none);
        check(g.generated == 2 && g.evaluated == 0 && g.accepted == 0 && g.rejected_total() == 0 && !g.best && g.log.size() == 2 && !g.log[0].evaluated, "N3-8 evaluate_candidates=false: generated, none evaluated, none accepted, no best");
        const auto q = n3_drone_service_node_relocate(base, f.ctx(3), kT2); // record_candidates defaults to false
        check(q.log.empty() && q.generated == 2 && q.accepted == 2 && q.best && std::fabs(q.best->objective - 2875.0) < kTol, "N3-8 record_candidates=false: no log, same counts and best");
        N3Options rec;
        rec.record_candidates = true;
        const auto a = n3_drone_service_node_relocate(base, f.ctx(3), kT2, rec), b = n3_drone_service_node_relocate(base, f.ctx(3), kT2, rec);
        bool same = a.log.size() == b.log.size() && a.generated == b.generated && a.accepted == b.accepted;
        for (std::size_t i = 0; same && i < a.log.size(); ++i) same = same_solution(a.log[i].solution, b.log[i].solution) && a.log[i].objective == b.log[i].objective && a.log[i].move.partner_trip == b.log[i].move.partner_trip;
        check(same, "N3-8 deterministic: repeated calls give identical candidates in identical order");

        bool det = true, matches = true, one_draw = true, covered = true;
        std::set<std::size_t> anchors;
        for (std::uint64_t seed = 1; seed <= 40; ++seed) {
            tdr::util::Rng x(seed), y(seed), z(seed);
            const auto rx = n3_drone_service_node_relocate(base, f.ctx(3), x);
            const auto ry = n3_drone_service_node_relocate(base, f.ctx(3), y);
            const std::size_t expect = z.uniform_index(base.drone_trips.size());
            const auto rd = n3_drone_service_node_relocate(base, f.ctx(3), expect);
            det = det && rx.anchor_trip == ry.anchor_trip && rx.generated == ry.generated && rx.accepted == ry.accepted;
            matches = matches && rx.anchor_trip == expect && rx.generated == rd.generated && rx.best && rd.best && rx.best->objective == rd.best->objective;
            one_draw = one_draw && x.next_u64() == z.next_u64();
            anchors.insert(rx.anchor_trip);
        }
        covered = anchors.size() == base.drone_trips.size();
        check(det, "N3-8 the same seed gives the same anchor and result");
        check(matches, "N3-8 the anchor is the uniform_index(#trips)-th trip (vector index)");
        check(one_draw, "N3-8 exactly one RNG draw is consumed");
        check(covered, "N3-8 over 40 seeds every trip is drawn at least once (non-vacuous)");
        check(same_solution(base, before), "N3-8 the input solution is unchanged");
    }

    // =====================================================================
    // N6-1. Partners are the drone customers of OTHER trips inside the D1 window (A8, revised at the M4 sign-off: trips of the anchor's own drone are partners by default; include_same_drone = false restricts to other drones).
    //       This block pins the RESTRICTED option (o) and the explicit +same option; the default is pinned in N6-1b. Chain with T1, T3 on drone 0 and T2 on drone 1 (|U| = 2).
    //       Half-steps (skeleton 0,1,3,5,7,0): T1 h = 1 (gap 0), T2 h = 5 (gap 2), T3 h = 7 (gap 3); adjacent iff |dh| <= 2w.
    //       Truck customers in the window (C1 h 2, C3 h 4, C5 h 6) are never partners.
    //       Objectives (same exchanges as N3-2/N3-3; drone ids do not change the timing):  T1<->T2 2975, T2<->T3 2875, T1<->T3 3725.
    // =====================================================================
    {
        auto c = make_chain(2, 20.0, 0, 1, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        N6Options o;
        o.record_candidates = true;
        o.include_same_drone = false; // the restricted option

        // anchor T1: dh to T2 = 4, to T3 = 6
        struct Row { int w; bool same; Idx partners; };
        const std::vector<Row> rows = {{0, false, {}}, {1, false, {}}, {2, false, {kT2}}, {3, false, {kT2}}, {2, true, {kT2}}, {3, true, {kT2, kT3}}};
        for (const auto& row : rows) {
            N6Options oo = o;
            oo.include_same_drone = row.same;
            const auto r = n6_drone_drone_swap(base, f.ctx(row.w), kT1, oo);
            const std::string tag = "N6-1 anchor T1 w=" + std::to_string(row.w) + (row.same ? " +same-drone" : "");
            check(r.status == OperatorStatus::Ok && r.op == OperatorId::N6 && r.anchor_trip == kT1 && r.anchor_customer == 2, tag + ": status, operator id, anchor");
            check(partners_of(r) == row.partners && r.generated == row.partners.size(), tag + ": partner set and order");
            check_records(r, base, f, tag);
        }
        const auto t1 = n6_drone_drone_swap(base, f.ctx(3), kT1, o);
        check_near(t1.log[0].objective, 2975.0, kTol, "N6-1 T1<->T2 objective (hand-derived 2975)");
        check(t1.best && t1.best->move.kind == MoveKind::N6Swap && t1.best->move.partner_trip == kT2 && t1.best->move.customer == 2 && t1.best->move.partner == 4, "N6-1 best = T1<->T2, move descriptor C2 <-> C4");
        N6Options same = o;
        same.include_same_drone = true;
        const auto t1s = n6_drone_drone_swap(base, f.ctx(3), kT1, same);
        check_near(t1s.log[0].objective, 2975.0, kTol, "N6-1 +same-drone: T1<->T2 2975");
        check_near(t1s.log[1].objective, 3725.0, kTol, "N6-1 +same-drone: T1<->T3 3725 (hand-derived)");
        check(t1s.best && t1s.best->move.partner_trip == kT2, "N6-1 +same-drone: best is still T1<->T2");

        // truck customers are inside the window but are never partners
        const PositionIndex pi(base, f.nodes);
        const auto near = pi.neighbours(2, 3);
        check(near == std::vector<NodeId>({1, 3, 4, 5, 6}), "N6-1 fixture sanity: C2's window neighbours (w=3) are C1,C3,C4,C5,C6, three of them truck customers");
        const auto t1w = n6_drone_drone_swap(base, f.ctx(3), kT1, o);
        bool no_truck = true;
        for (const auto& rec : t1w.log) no_truck = no_truck && (rec.move.partner == 4 || rec.move.partner == 6);
        check(no_truck && t1w.generated == 1, "N6-1 truck customers C1, C3, C5 inside the window are skipped (that is N5)");

        // anchor T2: dh to T3 = 2, to T1 = 4; both partners are on drone 0
        const std::vector<Row> rows2 = {{0, false, {}}, {1, false, {kT3}}, {2, false, {kT3, kT1}}, {3, false, {kT3, kT1}}};
        for (const auto& row : rows2) {
            const auto r = n6_drone_drone_swap(base, f.ctx(row.w), kT2, o);
            const std::string tag = "N6-1 anchor T2 w=" + std::to_string(row.w);
            check(partners_of(r) == row.partners && r.generated == row.partners.size(), tag + ": partner set, nearest first (T3 at dh 2 before T1 at dh 4)");
            check_records(r, base, f, tag);
        }
        const auto t2 = n6_drone_drone_swap(base, f.ctx(3), kT2, o);
        check_near(t2.log[0].objective, 2875.0, kTol, "N6-1 T2<->T3 objective (hand-derived 2875)");
        check_near(t2.log[1].objective, 2975.0, kTol, "N6-1 T2<->T1 objective (hand-derived 2975)");
        check(t2.best && t2.best->move.partner_trip == kT3 && std::fabs(t2.best->objective - 2875.0) < kTol && !t2.improves(), "N6-1 best = T2<->T3 (2875); not an improvement on 2675");

        // anchor T3 (h 7, drone 0): T2 at dh 2 (drone 1); T1 at dh 6 is on the same drone
        const auto t3 = n6_drone_drone_swap(base, f.ctx(3), kT3, o);
        check(partners_of(t3) == Idx({kT2}) && std::fabs(t3.log[0].objective - 2875.0) < kTol, "N6-1 anchor T3 (restricted): only T2 (T1 is on its own drone)");
        const auto t3s = n6_drone_drone_swap(base, f.ctx(3), kT3, same);
        check(partners_of(t3s) == Idx({kT2, kT1}) && std::fabs(t3s.log[1].objective - 3725.0) < kTol, "N6-1 anchor T3 +same-drone: T2 (dh 2) then T1 (dh 6, 3725)");
        check_records(t3, base, f, "N6-1 anchor T3");
        check_records(t3s, base, f, "N6-1 anchor T3 +same-drone");

        // restricted N6 and N3 never produce the same candidate (disjoint partner sets); the default N6 overlaps N3 (accepted at the M4 sign-off, see N6-1b)
        const auto n3r = n3_drone_service_node_relocate(base, f.ctx(3), kT1, N3Options{});
        const auto n6r = n6_drone_drone_swap(base, f.ctx(3), kT1, o);
        N3Options n3o;
        n3o.record_candidates = true;
        const auto n3rec = n3_drone_service_node_relocate(base, f.ctx(3), kT1, n3o);
        check(partners_of(n3rec) == Idx({kT3}) && partners_of(n6r) == Idx({kT2}) && n3r.generated == 1, "N3/N6 for anchor T1: N3 exchanges with T3 (same drone), restricted N6 with T2 (other drone): disjoint");
        check(same_solution(base, before), "N6-1 the input solution is unchanged");
    }

    // =====================================================================
    // N6-1b. The DEFAULT (A8, revised 9 Nov 2026): every other trip within the window is a partner, own-drone trips included; the overlap with N3 is accepted.
    //        Same chain as N6-1 (T1, T3 on drone 0, T2 on drone 1; h 1, 5, 7). Hand-derived objectives as in N6-1: T1<->T2 2975, T1<->T3 3725, T2<->T3 2875.
    // =====================================================================
    {
        check(N6Options{}.include_same_drone, "N6-1b the default is include_same_drone = true (any other trip)");
        auto c = make_chain(2, 20.0, 0, 1, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        N6Options d; // default options
        d.record_candidates = true;
        N6Options same = d;
        same.include_same_drone = true;
        N6Options restricted = d;
        restricted.include_same_drone = false;

        const auto t1 = n6_drone_drone_swap(base, f.ctx(3), kT1, d);
        check(partners_of(t1) == Idx({kT2, kT3}) && t1.generated == 2, "N6-1b anchor T1 (default, w=3): T2 (dh 4, other drone) then T3 (dh 6, SAME drone)");
        check_near(t1.log[0].objective, 2975.0, kTol, "N6-1b T1<->T2 2975");
        check_near(t1.log[1].objective, 3725.0, kTol, "N6-1b T1<->T3 3725 (the same exchange as N3's)");
        check(t1.best && t1.best->move.partner_trip == kT2 && std::fabs(t1.best->objective - 2975.0) < kTol, "N6-1b best = T1<->T2 (strict <, first generated)");
        check_records(t1, base, f, "N6-1b anchor T1 default");
        // the default is exactly the explicit +same option, and differs from the restricted option by the same-drone partner
        bool eq = true;
        for (std::size_t k = 0; k < base.drone_trips.size(); ++k)
            for (int w = 0; w <= 3; ++w) {
                const auto a = n6_drone_drone_swap(base, f.ctx(w), k, d), b = n6_drone_drone_swap(base, f.ctx(w), k, same);
                eq = eq && partners_of(a) == partners_of(b) && a.generated == b.generated && a.accepted == b.accepted;
            }
        check(eq, "N6-1b default == explicit include_same_drone = true, for every anchor and w = 0..3");
        const auto t3 = n6_drone_drone_swap(base, f.ctx(3), kT3, d);
        check(partners_of(t3) == Idx({kT2, kT1}), "N6-1b anchor T3 (default): T2 (dh 2) then T1 (dh 6, same drone)");
        check(partners_of(n6_drone_drone_swap(base, f.ctx(3), kT3, restricted)) == Idx({kT2}), "N6-1b restricted: T1 (same drone) dropped");
        // overlap with N3 is accepted: the exchange T1<->T3 comes out of both
        N3Options n3o;
        n3o.record_candidates = true;
        const auto n3rec = n3_drone_service_node_relocate(base, f.ctx(3), kT1, n3o);
        bool overlap = false;
        for (const auto& a : t1.log)
            for (const auto& b : n3rec.log) overlap = overlap || (a.move.partner_trip == b.move.partner_trip && same_solution(a.solution, b.solution));
        check(overlap, "N6-1b N3 and N6 (default) both generate the T1<->T3 exchange: the same solution (overlap accepted)");
        check(same_solution(base, before), "N6-1b the input solution is unchanged");
    }

    // =====================================================================
    // N6-2. Strict < and first-generated tie-breaking, nearest-first order. Chain at 100 m/s drone speed, T1, T3 on drone 0, T2 on drone 1.
    //       Base 2575 (N3-4). Anchor T2: T3 first (dh 2), T1 second (dh 4); T2<->T3 = 2605, T2<->T1 = 2605: an exact tie.
    // =====================================================================
    {
        auto c = make_chain(2, 100.0, 0, 1, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        N6Options o;
        o.record_candidates = true;
        check_near(ref_objective(base, f, nullptr), 2575.0, kTol, "N6-2 base objective 2575");
        const auto r = n6_drone_drone_swap(base, f.ctx(3), kT2, o);
        check(partners_of(r) == Idx({kT3, kT1}) && r.generated == 2, "N6-2 order: nearest partner (T3) first, then T1");
        check(r.log[0].objective == r.log[1].objective && std::fabs(r.log[0].objective - 2605.0) < kTol, "N6-2 the two objectives are exactly equal (2605)");
        check(r.best && r.best->move.partner_trip == kT3 && same_solution(r.best->solution, r.log[0].solution), "N6-2 tie -> the FIRST generated candidate (T3, the nearer partner; not the lower trip index) is kept");
        check(!r.improves(), "N6-2 2605 > 2575: no improvement");
        check_records(r, base, f, "N6-2");
        check(same_solution(base, before), "N6-2 the input solution is unchanged");
    }

    // =====================================================================
    // N6-2b. The same exact tie with the trips stored as [T3, T2, T1] (indices 0, 1, 2): anchor T2 = index 1; the nearer partner T3 is index 0 (first
    //        generated), the farther partner T1 is index 2 (> the anchor's index). The first generated candidate must win.
    // =====================================================================
    {
        auto c = make_chain(2, 100.0, 0, 1, 0, {2, 1, 0});
        Fix& f = *c.f;
        const Solution& base = c.base;
        N6Options o;
        o.record_candidates = true;
        const auto r = n6_drone_drone_swap(base, f.ctx(3), 1, o);
        check(r.generated == 2 && partners_of(r) == Idx({0, 2}) && r.log[0].objective == r.log[1].objective && std::fabs(r.log[0].objective - 2605.0) < kTol, "N6-2b partners T3 (index 0, nearer) then T1 (index 2) tie exactly at 2605");
        check(r.best && r.best->move.partner_trip == 0 && same_solution(r.best->solution, r.log[0].solution), "N6-2b tie -> the first generated candidate (index 0)");
        check_records(r, base, f, "N6-2b");
    }

    // =====================================================================
    // N6-3. Three drones; partners in the SAME gap (dh = 0), ordering by customer id, a tie with the base. n = 5, d = 1000 m, |U| = 3,
    //       route [0,Fa,1,Fb,Fc,5,0]: Fa x 500 (L(0,1)), Fb x 1800, Fc x 2600 (L(1,5): 4 FLPs at 1800, 2600, 3400, 4200). Skeleton [0,1,5,0]: Ta in gap 0 (h 1), Tb and Tc in gap 1 (h 3).
    //       Ta = (drone 0, Fa, C2, C5), Tb = (drone 1, Fb, C3, C5), Tc = (drone 2, Fc, C4, C5); vector order [Tc, Ta, Tb] (indices 0, 1, 2).
    //       Base: truck 100 + 500; Ta: 50+75 = 125; Tb: 180+60 = 240; Tc: 260+70 = 330 => 600 + 695 = 1295 (all back before 500).
    //       Ta<->Tb: Ta serves C3 50+125 = 175; Tb serves C2 180+10 = 190; Tc 330 => 1295.
    //       Ta<->Tc: Ta serves C4 50+175 = 225; Tc serves C2 260+30 = 290; Tb 240    => 1355.
    //       Tb<->Tc: Tb serves C4 180+110 = 290; Tc serves C3 260+20 = 280; Ta 125    => 1295.
    // =====================================================================
    {
        auto f = make_line(5, 3, 3600.0, 1000.0);
        check(f->idx.flps_on_arc(0, 1).size() == 1 && f->idx.flps_on_arc(1, 5).size() == 4, "N6-3 fixture sanity: 1 FLP on L(0,1), 4 on L(1,5)");
        const NodeId Fa = f->flp_at(0, 1, 0), Fb = f->flp_at(1, 5, 0), Fc = f->flp_at(1, 5, 1);
        check(std::fabs(f->flp_x(Fa) - 500) < 1e-9 && std::fabs(f->flp_x(Fb) - 1800) < 1e-9 && std::fabs(f->flp_x(Fc) - 2600) < 1e-9, "N6-3 fixture sanity: Fa 500, Fb 1800, Fc 2600");
        Solution base;
        base.truck_route = {0, Fa, 1, Fb, Fc, 5, 0};
        base.drone_trips = {trip(2, Fc, 4, 5), trip(0, Fa, 2, 5), trip(1, Fb, 3, 5)};
        constexpr std::size_t Tc = 0, Ta = 1, Tb = 2;
        const Solution before = base;
        bool feas = false;
        check_near(ref_objective(base, *f, &feas), 1295.0, kTol, "N6-3 base objective (hand-derived 1295)");
        check(feas, "N6-3 base feasible in the frozen reference (3 drones, 3 trips in flight at once)");
        N6Options o;
        o.record_candidates = true;

        const auto a0 = n6_drone_drone_swap(base, f->ctx(0), Ta, o);
        check(a0.status == OperatorStatus::Ok && a0.generated == 0, "N6-3 anchor Ta, w=0: Tb, Tc are in the next gap (dh 2) -> no candidate");
        for (int w : {1, 3}) {
            const auto r = n6_drone_drone_swap(base, f->ctx(w), Ta, o);
            check(partners_of(r) == Idx({Tb, Tc}), "N6-3 anchor Ta, w=" + std::to_string(w) + ": equal distance (dh 2) -> ordered by customer id (C3 = Tb before C4 = Tc), not by trip index");
            check_near(r.log[0].objective, 1295.0, kTol, "N6-3 Ta<->Tb objective 1295");
            check_near(r.log[1].objective, 1355.0, kTol, "N6-3 Ta<->Tc objective 1355");
            check(r.best && r.best->move.partner_trip == Tb && !r.improves(), "N6-3 best = Ta<->Tb, equal to the base objective -> not an improvement (strict <)");
            check_records(r, base, *f, "N6-3 Ta w=" + std::to_string(w));
        }
        // anchor Tb: Tc at dh 0, Ta at dh 2; the swaps Tb<->Tc and Tb<->Ta both give 1295
        const auto b0 = n6_drone_drone_swap(base, f->ctx(0), Tb, o);
        check(partners_of(b0) == Idx({Tc}), "N6-3 anchor Tb, w=0: only Tc (same gap, dh 0 is inside every window)");
        const auto b3 = n6_drone_drone_swap(base, f->ctx(3), Tb, o);
        check(partners_of(b3) == Idx({Tc, Ta}), "N6-3 anchor Tb, w=3: Tc (dh 0) before Ta (dh 2)");
        check(b3.log[0].objective == b3.log[1].objective && std::fabs(b3.log[0].objective - 1295.0) < kTol, "N6-3 both swaps give exactly 1295");
        check(b3.best && b3.best->move.partner_trip == Tc, "N6-3 tie between candidates -> the first generated (Tc) is kept");
        check_records(b3, base, *f, "N6-3 Tb w=3");
        const auto c3 = n6_drone_drone_swap(base, f->ctx(3), Tc, o);
        check(partners_of(c3) == Idx({Tb, Ta}) && std::fabs(c3.log[0].objective - 1295.0) < kTol && std::fabs(c3.log[1].objective - 1355.0) < kTol, "N6-3 anchor Tc: Tb (dh 0), Ta (dh 2); 1295 and 1355");
        check_records(c3, base, *f, "N6-3 Tc w=3");
        check(same_solution(base, before), "N6-3 the input solution is unchanged");
    }

    // =====================================================================
    // N6-4. Endurance failure and the window/drone filters on the N3-5 geometry (e = 260; B launches from x 4500), now with A on drone 0 and B on drone 1.
    //       A<->B violates (37) of B (275 > 260): EvalEndurance; objective 1700.
    // =====================================================================
    {
        const auto build = [&](double e, int b_drone, std::unique_ptr<Fix>& f, Solution& base) {
            f = make_line(5, 2, e);
            const NodeId Fa = f->flp_at(0, 1, 0), Fb = f->flp_at(3, 5, 2);
            base = Solution{};
            base.truck_route = {0, Fa, 1, 3, Fb, 5, 0};
            base.drone_trips = {trip(0, Fa, 2, 3), trip(b_drone, Fb, 4, 5)};
        };
        std::unique_ptr<Fix> f;
        Solution base;
        build(260.0, 1, f, base);
        const Solution before = base;
        N6Options o;
        o.record_candidates = true;
        const auto r = n6_drone_drone_swap(base, f->ctx(3), 0, o);
        check(r.generated == 1 && r.evaluated == 1 && r.accepted == 0 && !r.best && r.rejected[static_cast<std::size_t>(RejectReason::EvalEndurance)] == 1 && r.rejected_total() == 1, "N6-4 e = 260: rejected as EvalEndurance, no best");
        check_near(r.log[0].objective, 1700.0, kTol, "N6-4 best-effort objective 1700");
        check_records(r, base, *f, "N6-4 e=260");
        check(n6_drone_drone_swap(base, f->ctx(1), 0, o).generated == 0, "N6-4 w = 1: B is 4 half-steps away -> not adjacent");
        check(n6_drone_drone_swap(base, f->ctx(2), 0, o).generated == 1, "N6-4 w = 2: B is adjacent (dh 4 <= 4)");
        std::unique_ptr<Fix> f2;
        Solution b2;
        build(3600.0, 1, f2, b2);
        const auto ok = n6_drone_drone_swap(b2, f2->ctx(3), 0, o);
        check(ok.accepted == 1 && std::fabs(ok.best->objective - 1700.0) < kTol, "N6-4 e = 3600: accepted, 1700 -> the rejection was caused by endurance alone");
        // same drone: admitted by default (A8 revised), excluded by the restricted option
        std::unique_ptr<Fix> f3;
        Solution b3;
        build(3600.0, 0, f3, b3);
        N6Options restricted = o;
        restricted.include_same_drone = false;
        check(n6_drone_drone_swap(b3, f3->ctx(3), 0, restricted).generated == 0, "N6-4 both trips on drone 0: no partner with the restricted option");
        const auto rs = n6_drone_drone_swap(b3, f3->ctx(3), 0, o); // default options
        check(rs.generated == 1 && rs.accepted == 1 && std::fabs(rs.best->objective - 1700.0) < kTol, "N6-4 default: the same-drone partner is admitted, the same exchange as N3's, objective 1700");
        check_records(rs, b3, *f3, "N6-4 same-drone");
        check(same_solution(base, before), "N6-4 the input solution is unchanged");
    }

    // =====================================================================
    // N6-5. Recovery at the END DEPOT is preserved; short routes. The N3-6 geometry with A on drone 0 and B on drone 1 (objective 950 -> 1000).
    // =====================================================================
    {
        auto f = make_line(4, 2);
        const NodeId Fa = f->flp_at(0, 1, 0), Fb = f->flp_at(3, 0, 0);
        Solution base;
        base.truck_route = {0, Fa, 1, 3, Fb, 0};
        base.drone_trips = {trip(0, Fa, 2, 3), trip(1, Fb, 4, 0)};
        const Solution before = base;
        N6Options o;
        o.record_candidates = true;
        check(n6_drone_drone_swap(base, f->ctx(1), 0, o).generated == 0, "N6-5 w = 1: Fb is in gap 2 (h 5), Fa in gap 0 (h 1): dh 4 > 2");
        const auto r = n6_drone_drone_swap(base, f->ctx(2), 0, o);
        check(r.generated == 1 && r.accepted == 1 && std::fabs(r.log[0].objective - 1000.0) < kTol, "N6-5 w = 2: A<->B accepted, objective 1000");
        check(r.best && r.best->solution.drone_trips[1].recovery_node == 0 && r.best->solution.drone_trips[1].drone_id == 1 && r.best->solution.drone_trips[0].drone_id == 0, "N6-5 depot recovery and drone ids unchanged");
        check_records(r, base, *f, "N6-5");

        // lone trip: no partner
        auto f1 = make_line(3, 2);
        const NodeId F1a = f1->flp_at(0, 1, 0);
        Solution lone;
        lone.truck_route = {0, F1a, 1, 2, 0};
        lone.drone_trips = {trip(0, F1a, 3, 2)};
        const auto lr = n6_drone_drone_swap(lone, f1->ctx(3), 0);
        check(lr.status == OperatorStatus::Ok && lr.generated == 0 && !lr.best && lr.anchor_customer == 3, "N6-5 a lone trip has no partner: zero candidates, status Ok");
        check(same_solution(base, before), "N6-5 the input solution is unchanged");
    }

    // =====================================================================
    // N6-6. All trips on ONE drone: no partner with the restricted option; by default (and with include_same_drone) the middle trip sees the N3 pair, ordered nearest first.
    // =====================================================================
    {
        auto c = make_chain(1, 20.0, 0, 0, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        N6Options o;
        o.record_candidates = true;
        N6Options restricted = o;
        restricted.include_same_drone = false;
        bool none = true, some = false;
        for (std::size_t k = 0; k < base.drone_trips.size(); ++k) {
            none = none && n6_drone_drone_swap(base, f.ctx(3), k, restricted).generated == 0;
            some = some || n6_drone_drone_swap(base, f.ctx(3), k, o).generated > 0;
        }
        check(none, "N6-6 every anchor of a one-drone solution: zero candidates with the restricted option");
        check(some, "N6-6 the default gives candidates on a one-drone solution (A8 revised)");
        const auto r = n6_drone_drone_swap(base, f.ctx(3), kT2, o);
        check(partners_of(r) == Idx({kT3, kT1}) && std::fabs(r.log[0].objective - 2875.0) < kTol && std::fabs(r.log[1].objective - 2975.0) < kTol, "N6-6 default, anchor T2: T3 (2875) then T1 (2975): N3's pair, N6's nearest-first order");
        check_records(r, base, f, "N6-6");
        check(same_solution(base, before), "N6-6 the input solution is unchanged");
    }

    // =====================================================================
    // N6-7. Errors, flags, determinism and the randomised overload.
    // =====================================================================
    {
        auto c = make_chain(2, 20.0, 0, 1, 0);
        Fix& f = *c.f;
        const Solution& base = c.base;
        const Solution before = base;
        check(n6_drone_drone_swap(base, f.ctx(3), base.drone_trips.size()).status == OperatorStatus::InvalidAnchor, "N6-7 trip index == #trips -> InvalidAnchor");
        Solution none;
        none.truck_route = {0, 1, 2, 3, 4, 5, 6, 7, 0};
        check(n6_drone_drone_swap(none, f.ctx(3), 0).status == OperatorStatus::InvalidAnchor, "N6-7 no trips: index 0 -> InvalidAnchor");
        tdr::util::Rng r0(1);
        const auto ne = n6_drone_drone_swap(none, f.ctx(3), r0);
        check(ne.status == OperatorStatus::NoEligibleAnchor && ne.generated == 0 && ne.op == OperatorId::N6, "N6-7 Rng overload with no trips -> NoEligibleAnchor");
        Solution omitted = base;
        omitted.drone_trips.erase(omitted.drone_trips.begin() + static_cast<std::ptrdiff_t>(kT3));
        check(n6_drone_drone_swap(omitted, f.ctx(3), 0).status == OperatorStatus::InputInfeasible, "N6-7 omitted customer -> InputInfeasible, nothing generated");
        Solution dup = base;
        dup.drone_trips.push_back(trip(1, c.F2, 4, 5));
        const auto di = n6_drone_drone_swap(dup, f.ctx(3), 0);
        check(di.status == OperatorStatus::InputInfeasible && di.generated == 0, "N6-7 duplicate customer -> InputInfeasible, nothing generated");

        N6Options gen;
        gen.evaluate_candidates = false;
        gen.record_candidates = true;
        const auto g = n6_drone_drone_swap(base, f.ctx(3), kT2, gen);
        check(g.generated == 2 && g.evaluated == 0 && g.accepted == 0 && g.rejected_total() == 0 && !g.best && g.log.size() == 2 && !g.log[0].evaluated, "N6-7 evaluate_candidates=false: generated, none evaluated, none accepted, no best");
        const auto q = n6_drone_drone_swap(base, f.ctx(3), kT2);
        check(q.log.empty() && q.generated == 2 && q.accepted == 2 && q.best && std::fabs(q.best->objective - 2875.0) < kTol, "N6-7 record_candidates=false: no log, same counts and best");
        N6Options rec;
        rec.record_candidates = true;
        const auto a = n6_drone_drone_swap(base, f.ctx(3), kT2, rec), b = n6_drone_drone_swap(base, f.ctx(3), kT2, rec);
        bool same = a.log.size() == b.log.size();
        for (std::size_t i = 0; same && i < a.log.size(); ++i) same = same_solution(a.log[i].solution, b.log[i].solution) && a.log[i].objective == b.log[i].objective && a.log[i].move.partner_trip == b.log[i].move.partner_trip;
        check(same, "N6-7 deterministic: repeated calls give identical candidates in identical order");

        bool det = true, matches = true, one_draw = true;
        std::set<std::size_t> anchors;
        for (std::uint64_t seed = 1; seed <= 40; ++seed) {
            tdr::util::Rng x(seed), y(seed), z(seed);
            const auto rx = n6_drone_drone_swap(base, f.ctx(3), x);
            const auto ry = n6_drone_drone_swap(base, f.ctx(3), y);
            const std::size_t expect = z.uniform_index(base.drone_trips.size());
            const auto rd = n6_drone_drone_swap(base, f.ctx(3), expect);
            det = det && rx.anchor_trip == ry.anchor_trip && rx.generated == ry.generated && rx.accepted == ry.accepted;
            matches = matches && rx.anchor_trip == expect && rx.generated == rd.generated && rx.best.has_value() == rd.best.has_value() && (!rx.best || rx.best->objective == rd.best->objective);
            one_draw = one_draw && x.next_u64() == z.next_u64();
            anchors.insert(rx.anchor_trip);
        }
        check(det, "N6-7 the same seed gives the same anchor and result");
        check(matches, "N6-7 the anchor is the uniform_index(#trips)-th trip (vector index)");
        check(one_draw, "N6-7 exactly one RNG draw is consumed");
        check(anchors.size() == base.drone_trips.size(), "N6-7 over 40 seeds every trip is drawn at least once (non-vacuous)");
        check(same_solution(base, before), "N6-7 the input solution is unchanged");
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
