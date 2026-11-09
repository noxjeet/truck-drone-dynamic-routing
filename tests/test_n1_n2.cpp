// M5 tests: neighbourhoods N1 (launch node relocate) and N2 (recover node relocate). HAND-DERIVED fixtures on a line, so every
// expected number below was worked out on paper and is not read back from the code under test:
//
//   * depot 0 at x = 0; customer Ci at x = 1000 i metres; truck 10 m/s, drone 20 m/s unless stated, SL = SR = 0, service
//     times 0, theta = 0, d = 600 m => floor(D/600) FLPs per arc (1000 m: 1 FLP at the midpoint; 2000 m: 3 FLPs at
//     1/4, 2/4, 3/4 of the arc; 4000 m: 6 FLPs at k/7; 5000 m: 8 FLPs at k/9). FLP pool = every ordered pair of truck nodes.
//   * truck arrival at x metres along the line, no waiting: x / 10 s; a truck that reaches a recovery node before its drone
//     waits for it (SR = 0), which delays everything AFTER that node (the node's own arrival time is the arrival).
//   * drone customer arrival = launch time + flight / speed; endurance (36) = (t_k - l_j) + loaded, (37) = loaded + return
//     flight (timing_engine.hpp), loaded = outbound flight; both <= e (inclusive).
//
// Independent cross-checks use the FROZEN reference evaluator (tdr::timing_ref), not the production one. M1-M4 behaviour is
// covered by the earlier test programs and is not repeated here.

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
#include "search/neighbourhood_n1.hpp"
#include "search/neighbourhood_n2.hpp"
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

constexpr double kTol = 1e-9;
using Idx = std::vector<std::size_t>;

std::size_t R(RejectReason r) { return static_cast<std::size_t>(r); }

// ---- independent route helpers (no production helper is called) ----
std::vector<NodeId> skeleton_of(const TruckRoute& r, const Fix& f) {
    std::vector<NodeId> out;
    for (NodeId v : r) if (f.nodes.at(v).type != NodeType::FlexibleLaunchPoint) out.push_back(v);
    return out;
}
std::optional<std::size_t> index_of(const TruckRoute& r, NodeId v) {
    for (std::size_t i = 0; i < r.size(); ++i) if (r[i] == v) return i;
    return std::nullopt;
}
// "recovery strictly after launch on the route" (depot id 0 occurs first and last, so the END depot counts)
bool recovery_after_launch(const TruckRoute& r, const DroneTrip& t) {
    const auto lp = index_of(r, t.launch_node);
    if (!lp) return false;
    for (std::size_t i = *lp + 1; i < r.size(); ++i) if (r[i] == t.recovery_node) return true;
    return false;
}
std::set<NodeId> flps_in_route(const TruckRoute& r, const Fix& f) {
    std::set<NodeId> out;
    for (NodeId v : r) if (f.nodes.at(v).type == NodeType::FlexibleLaunchPoint) out.insert(v);
    return out;
}
bool launch_used(const Solution& s, NodeId flp) {
    for (const auto& t : s.drone_trips) if (t.launch_node == flp) return true;
    return false;
}

std::size_t first_best(const OperatorResult& r) {
    std::size_t best = r.log.size();
    for (std::size_t i = 0; i < r.log.size(); ++i)
        if (r.log[i].reason == RejectReason::None && (best == r.log.size() || r.log[i].objective < r.log[best].objective)) best = i;
    return best;
}

std::size_t impossible_reasons(const OperatorResult& r) { // reasons that cannot occur in N1/N2
    std::size_t n = 0;
    for (RejectReason x : {RejectReason::AffectedTrips, RejectReason::NoFreeDrone, RejectReason::NoReplacementFlp, RejectReason::EvalFlpArc, RejectReason::EvalFlpUnused})
        n += r.rejected[R(x)];
    return n;
}

// The new nodes of the candidates, in generation order.
std::vector<NodeId> launches_of(const OperatorResult& r) { std::vector<NodeId> v; for (const auto& c : r.log) v.push_back(c.move.launch); return v; }
std::vector<NodeId> recoveries_of(const OperatorResult& r) { std::vector<NodeId> v; for (const auto& c : r.log) v.push_back(c.move.recovery); return v; }
std::vector<RejectReason> reasons_of(const OperatorResult& r) { std::vector<RejectReason> v; for (const auto& c : r.log) v.push_back(c.reason); return v; }

// Every record of an N1 / N2 result against the FROZEN reference evaluator and the model.
void check_records(const OperatorResult& res, const Solution& base, const Fix& f, const std::string& tag, MoveKind kind, std::size_t trip) {
    const bool n1 = kind == MoveKind::N1LaunchRelocate;
    bool only = true, cover = true, arcs = true, obj = true, verdict = true, fields = true, reasons = true, relation = true, flpset = true, dropped = true;
    std::set<std::string> sigs;
    for (const auto& rec : res.log) {
        const Solution& c = rec.solution;
        const DroneTrip& bt = base.drone_trips[trip];
        // what changed: exactly one node of exactly one trip (and, for N1, the route's FLP entries)
        only = only && c.drone_trips.size() == base.drone_trips.size();
        for (std::size_t k = 0; only && k < base.drone_trips.size(); ++k) {
            const auto &b = base.drone_trips[k], &x = c.drone_trips[k];
            if (k != trip) { only = only && b.drone_id == x.drone_id && b.launch_node == x.launch_node && b.service_customer == x.service_customer && b.recovery_node == x.recovery_node; continue; }
            only = only && x.drone_id == b.drone_id && x.service_customer == b.service_customer &&
                   (n1 ? (x.recovery_node == b.recovery_node && x.launch_node != b.launch_node) : (x.launch_node == b.launch_node && x.recovery_node != b.recovery_node));
        }
        if (n1) only = only && skeleton_of(c.truck_route, f) == skeleton_of(base.truck_route, f);
        else only = only && c.truck_route == base.truck_route;
        cover = cover && exactly_once(c, f.n);
        // FLP bookkeeping: every FLP of the route is some trip's launch, every launch is on the route, nothing else changed
        std::set<NodeId> expect = flps_in_route(base.truck_route, f);
        if (n1) {
            const bool old_used = launch_used(c, bt.launch_node);
            if (!old_used) expect.erase(bt.launch_node);
            expect.insert(c.drone_trips[trip].launch_node);
            dropped = dropped && (rec.flps_dropped == (old_used ? std::vector<NodeId>{} : std::vector<NodeId>{bt.launch_node}));
        } else {
            dropped = dropped && rec.flps_dropped.empty();
        }
        flpset = flpset && flps_in_route(c.truck_route, f) == expect;
        for (NodeId v : c.truck_route) if (f.nodes.at(v).type == NodeType::FlexibleLaunchPoint) flpset = flpset && launch_used(c, v);
        for (const auto& t : c.drone_trips) flpset = flpset && index_of(c.truck_route, t.launch_node).has_value();
        // FLPs on their arcs (independent)
        {
            NodeId last_real = kInvalidNodeId;
            std::vector<NodeId> pending;
            bool ok = true;
            for (NodeId v : c.truck_route) {
                if (f.nodes.at(v).type == NodeType::FlexibleLaunchPoint) pending.push_back(v);
                else { for (NodeId p : pending) { const auto& d = f.flp.at(p); ok = ok && d.arc_from == last_real && d.arc_to == v; } pending.clear(); last_real = v; }
            }
            arcs = arcs && ok && pending.empty();
        }
        // descriptor
        fields = fields && rec.move.kind == kind && rec.move.replaced_trip == trip && rec.move.customer == bt.service_customer && rec.move.drone_id == bt.drone_id &&
                 rec.move.replaced_node == (n1 ? bt.launch_node : bt.recovery_node) &&
                 (n1 ? (rec.move.launch == c.drone_trips[trip].launch_node && rec.move.recovery == bt.recovery_node)
                     : (rec.move.recovery == c.drone_trips[trip].recovery_node && rec.move.launch == bt.launch_node));
        // the relationship "recovery after launch": InvalidRecovery <=> violated (independent), and only then is the candidate not evaluated
        const bool ok_rel = recovery_after_launch(c.truck_route, c.drone_trips[trip]);
        relation = relation && ((rec.reason == RejectReason::InvalidRecovery) == !ok_rel) && (rec.evaluated == ok_rel || !true);
        if (rec.reason == RejectReason::InvalidRecovery) relation = relation && !rec.evaluated && rec.affected_trips == std::vector<std::size_t>{trip};
        std::string sig;
        for (NodeId v : c.truck_route) sig += std::to_string(v) + ",";
        sig += "|";
        for (const auto& t : c.drone_trips) sig += std::to_string(t.drone_id) + ":" + std::to_string(t.launch_node) + ">" + std::to_string(t.service_customer) + ">" + std::to_string(t.recovery_node) + ";";
        sigs.insert(sig);
        if (rec.evaluated) {
            bool feas = false;
            const double o = ref_objective(c, f, &feas);
            obj = obj && std::fabs(o - rec.objective) <= 1e-9 * std::max(1.0, std::fabs(o));
            verdict = verdict && (feas == (rec.reason == RejectReason::None));
        }
        reasons = reasons && (rec.reason == RejectReason::None || rec.reason == RejectReason::InvalidRecovery || rec.reason == RejectReason::EvalEndurance ||
                              rec.reason == RejectReason::EvalFleetCapacity || rec.reason == RejectReason::EvalDroneSequencing || rec.reason == RejectReason::EvalTripOrder ||
                              rec.reason == RejectReason::EvalOther);
    }
    check(only, tag + ": every candidate changes exactly one node of exactly the anchor trip (drone id, customer, the other node, all other trips, the skeleton)");
    check(cover, tag + ": exactly-once customer coverage in every record");
    check(arcs, tag + ": no FLP leaves its arc in any record (independent check)");
    check(flpset, tag + ": the route's FLPs are exactly the trips' launches (no idle FLP, no missing launch; the old launch dropped only if unused) -- independent");
    check(dropped, tag + ": flps_dropped is the old launch iff no trip uses it any more (N1), empty (N2)");
    check(fields, tag + ": move descriptors (kind, anchor trip, customer, drone id, old and new node)");
    check(relation, tag + ": InvalidRecovery <=> recovery not after launch on the candidate's route (independent); such candidates are not evaluated");
    check(obj, tag + ": production objective == frozen reference objective in every evaluated record");
    check(verdict, tag + ": production verdict == frozen reference verdict in every evaluated record");
    check(reasons && impossible_reasons(res) == 0, tag + ": only InvalidRecovery and evaluator reasons occur (AffectedTrips/NoFreeDrone/NoReplacementFlp/FlpArc/FlpUnused = 0)");
    check(res.accepted + res.rejected_total() == res.generated, tag + ": accepted + rejected == generated (one outcome per candidate)");
    check(res.evaluated + res.rejected[R(RejectReason::InvalidRecovery)] == res.generated, tag + ": every candidate is either evaluated or an InvalidRecovery (never both)");
    check(res.duplicates_skipped == 0 && sigs.size() == res.log.size() && res.log.size() == res.generated, tag + ": no duplicate candidates; one record per generated candidate");
    const std::size_t fb = first_best(res);
    if (fb == res.log.size()) check(!res.best && res.accepted == 0, tag + ": no feasible candidate -> no best");
    else check(res.best && res.best->objective == res.log[fb].objective && res.best->move.launch == res.log[fb].move.launch && res.best->move.recovery == res.log[fb].move.recovery &&
                   res.best->solution.truck_route == res.log[fb].solution.truck_route, tag + ": best = the first-generated lowest feasible candidate (strict <)");
}

// ---- fixtures ----

// A: n = 4, skeleton [0,1,2,4,0], drone customer C3, trip (F3000, C3, C4); F3000 = 2nd FLP of arc (2,4) (x 3000).
struct FixA { std::unique_ptr<Fix> f; Solution base; NodeId F3000 = 0; };
FixA make_A(double endurance, int units = 1) {
    FixA a;
    a.f = make_line(4, units, endurance);
    a.F3000 = a.f->flp_at(2, 4, 1);
    a.base.truck_route = {0, 1, 2, a.F3000, 4, 0};
    a.base.drone_trips = {trip(0, a.F3000, 3, 4)};
    return a;
}

// C: n = 5, skeleton [0,1,2,4,0], route [0,1,2,F,4,0], F = x 3000 shared; T1 = (drone 0, F, C3, C4), T2 = (drone 1, F, C5, depot).
struct FixC { std::unique_ptr<Fix> f; Solution base; NodeId F3000 = 0; };
FixC make_C() {
    FixC c;
    c.f = make_line(5, 2);
    c.F3000 = c.f->flp_at(2, 4, 1);
    c.base.truck_route = {0, 1, 2, c.F3000, 4, 0};
    c.base.drone_trips = {trip(0, c.F3000, 3, 4), trip(1, c.F3000, 5, 0)};
    return c;
}

// S: n = 5, skeleton [0,1,3,5,0], drone customers C2, C4, route [0,1,F2000,3,F4000,5,0]; T1 = (d1, F2000, C2, C3), T2 = (d2, F4000, C4, C5).
// Trip VECTOR order is [T2, T1] (index != route order).
constexpr std::size_t kS2 = 0, kS1 = 1;
struct FixS { std::unique_ptr<Fix> f; Solution base; NodeId F2000 = 0, F4000 = 0; };
FixS make_S(int units, int d1, int d2) {
    FixS s;
    s.f = make_line(5, units);
    s.F2000 = s.f->flp_at(1, 3, 1);
    s.F4000 = s.f->flp_at(3, 5, 1);
    s.base.truck_route = {0, 1, s.F2000, 3, s.F4000, 5, 0};
    s.base.drone_trips = {trip(d2, s.F4000, 4, 5), trip(d1, s.F2000, 2, 3)};
    return s;
}

// W: n = 5, drone speed 10 m/s, skeleton [0,1,2,4,5,0], route [0,1,2,F3500,4,5,0]; trip (F3500, C3, C4); F3500 = 3rd FLP of arc (2,4).
struct FixW { std::unique_ptr<Fix> f; Solution base; NodeId F3500 = 0; };
FixW make_W(double endurance) {
    FixW w;
    w.f = make_line(5, 1, endurance, 600.0, 1000.0, 10.0);
    w.F3500 = w.f->flp_at(2, 4, 2);
    w.base.truck_route = {0, 1, 2, w.F3500, 4, 5, 0};
    w.base.drone_trips = {trip(0, w.F3500, 3, 4)};
    return w;
}

} // namespace

int main() {
    const MoveKind K1 = MoveKind::N1LaunchRelocate, K2 = MoveKind::N2RecoveryRelocate;

    // =====================================================================
    // N1-A. Fixture A (n = 4, |U| = 1, e = 3600): skeleton [0,1,2,4,0] (s0..s4, gaps g0 (0,1), g1 (1,2), g2 (2,4), g3 (4,0)), C3 drone-served,
    //       trip (F3000, C3, C4), F3000 in g2. Base: truck C1 100, C2 200, C4 400; launch 300, C3 arrival 300 (flight 0), back to C4 +50 = 350 < 400.
    //       Objective 100 + 200 + 400 + 300 = 1000.  Launch at x gives l = x/10, C3 at l + (3000-x)/20 = 150 + x/20 (C4 400 unchanged).
    //         F500  (g0): 175 -> 875     F1500 (g1): 225 -> 925     F2500 (g2): 275 -> 975     F3500 (g2): 375 -> 1075 (back at C4 425 > 400: truck waits, depot only)
    //       Arc (4,0) = 4000 m holds 6 FLPs, all AFTER the recovery C4 -> InvalidRecovery.  Window: gaps g with |g - g0'| <= w, g0' = 2.
    // =====================================================================
    {
        auto a = make_A(3600.0);
        Fix& f = *a.f;
        bool feas = false;
        check_near(ref_objective(a.base, f, &feas), 1000.0, kTol, "N1-A base objective (hand-derived 1000, frozen reference)");
        check(feas, "N1-A base is feasible in the frozen reference");
        const NodeId F500 = f.flp_at(0, 1, 0), F1500 = f.flp_at(1, 2, 0), F2500 = f.flp_at(2, 4, 0), F3500 = f.flp_at(2, 4, 2);
        check(f.idx.flps_on_arc(0, 1).size() == 1 && f.idx.flps_on_arc(1, 2).size() == 1 && f.idx.flps_on_arc(2, 4).size() == 3 && f.idx.flps_on_arc(4, 0).size() == 6 &&
                  std::fabs(f.flp_x(F500) - 500) < kTol && std::fabs(f.flp_x(F1500) - 1500) < kTol && std::fabs(f.flp_x(F2500) - 2500) < kTol && std::fabs(f.flp_x(a.F3000) - 3000) < kTol && std::fabs(f.flp_x(F3500) - 3500) < kTol,
              "N1-A fixture sanity: 1 / 1 / 3 / 6 FLPs on the arcs (0,1) (1,2) (2,4) (4,0), at x 500, 1500, 2500 / 3000 / 3500");
        const Solution before = a.base;
        N1Options o;
        o.record_candidates = true;

        // w = 0: only the other FLPs of the launch arc, in arc order (F2500, then F3500; the current F3000 is excluded)
        const auto r0 = n1_launch_node_relocate(a.base, f.ctx(0), 0, o);
        check(r0.status == OperatorStatus::Ok && r0.op == OperatorId::N1 && r0.anchor_trip == 0 && r0.anchor_customer == 3, "N1-A w=0: status, operator id, anchor trip and customer");
        check_near(r0.base_objective, 1000.0, kTol, "N1-A base objective reported");
        check(launches_of(r0) == std::vector<NodeId>({F2500, F3500}) && r0.generated == 2 && r0.evaluated == 2 && r0.accepted == 2, "N1-A w=0: candidates F2500, F3500 in arc order; the current launch is excluded");
        check_near(r0.log[0].objective, 975.0, kTol, "N1-A F2500 objective (hand-derived 975)");
        check_near(r0.log[1].objective, 1075.0, kTol, "N1-A F3500 objective (hand-derived 1075)");
        check(r0.best && r0.improves() && r0.best->move.launch == F2500 && std::fabs(r0.best->objective - 975.0) < kTol, "N1-A w=0: best = F2500 (975 < 1000), a strict improvement");
        check(r0.best && r0.best->solution.truck_route == TruckRoute({0, 1, 2, F2500, 4, 0}) && r0.best->flps_dropped == std::vector<NodeId>({a.F3000}), "N1-A w=0: result route [0,1,2,F2500,4,0]; the old launch F3000 (used by no other trip) is dropped and reported");
        check(r0.best && r0.best->solution.drone_trips[0].launch_node == F2500 && r0.best->solution.drone_trips[0].recovery_node == 4 && r0.best->solution.drone_trips[0].drone_id == 0 && r0.best->solution.drone_trips[0].service_customer == 3,
              "N1-A w=0: trip = (drone 0, F2500, C3, C4): only the launch changed");
        check(r0.log[0].move.kind == K1 && r0.log[0].move.replaced_node == a.F3000 && r0.log[0].move.gap == 2 && r0.log[0].move.launch == F2500, "N1-A descriptor: old launch F3000, new F2500, gap 2");
        check_records(r0, a.base, f, "N1-A w=0", K1, 0);

        // w = 1: gaps g1, g2, g3 -> F1500, F2500, F3500, then the six FLPs of arc (4,0) (after the recovery: InvalidRecovery, not evaluated)
        const auto r1 = n1_launch_node_relocate(a.base, f.ctx(1), 0, o);
        const auto& fl40 = f.idx.flps_on_arc(4, 0);
        std::vector<NodeId> want1 = {F1500, F2500, F3500};
        want1.insert(want1.end(), fl40.begin(), fl40.end());
        check(launches_of(r1) == want1 && r1.generated == 9 && r1.evaluated == 3 && r1.accepted == 3 && r1.rejected[R(RejectReason::InvalidRecovery)] == 6 && r1.rejected_total() == 6, "N1-A w=1: gap order g1, g2, g3; F1500, F2500, F3500 evaluated; the 6 FLPs after C4 are InvalidRecovery");
        check_near(r1.log[0].objective, 925.0, kTol, "N1-A F1500 objective (hand-derived 925)");
        check(r1.best && r1.best->move.launch == F1500 && std::fabs(r1.best->objective - 925.0) < kTol, "N1-A w=1: best = F1500 (925)");
        check(r1.log[3].reason == RejectReason::InvalidRecovery && !r1.log[3].evaluated && r1.log[3].affected_trips == std::vector<std::size_t>({0}) && !r1.log[3].solution.truck_route.empty(),
              "N1-A InvalidRecovery candidates are named (trip 0), not evaluated, and shown unrepaired (recovery still C4)");
        check(r1.log[3].solution.drone_trips[0].recovery_node == 4, "N1-A the recovery node is never changed to repair the order");
        check_records(r1, a.base, f, "N1-A w=1", K1, 0);

        // w = 3: all four gaps; g0 (the depot arc) first
        const auto r3 = n1_launch_node_relocate(a.base, f.ctx(3), 0, o);
        std::vector<NodeId> want3 = {F500, F1500, F2500, F3500};
        want3.insert(want3.end(), fl40.begin(), fl40.end());
        check(launches_of(r3) == want3 && r3.generated == 10 && r3.accepted == 4, "N1-A w=3: g0 (depot arc) F500 first, then F1500, F2500, F3500, then the invalid six");
        check_near(r3.log[0].objective, 875.0, kTol, "N1-A F500 objective (hand-derived 875)");
        check(r3.best && r3.best->move.launch == F500 && r3.best->solution.truck_route == TruckRoute({0, F500, 1, 2, 4, 0}) && std::fabs(r3.best->objective - 875.0) < kTol, "N1-A w=3: best = F500 (875), route [0,F500,1,2,4,0]");
        check_records(r3, a.base, f, "N1-A w=3", K1, 0);
        // a window larger than the route is harmless
        const auto r9 = n1_launch_node_relocate(a.base, f.ctx(9), 0, o);
        check(launches_of(r9) == want3, "N1-A w=9: the window is clipped to the existing gaps (same candidates as w=3)");

        // flags
        N1Options gen = o;
        gen.evaluate_candidates = false;
        const auto g1 = n1_launch_node_relocate(a.base, f.ctx(1), 0, gen);
        check(g1.generated == 9 && g1.evaluated == 0 && g1.accepted == 0 && !g1.best && g1.rejected[R(RejectReason::InvalidRecovery)] == 6 && g1.rejected_total() == 6 && !g1.log[0].evaluated,
              "N1-A evaluate_candidates=false: 9 generated, none evaluated, none accepted, the 6 structural rejections still counted, no best");
        const auto q = n1_launch_node_relocate(a.base, f.ctx(1), 0);
        check(q.log.empty() && q.generated == 9 && q.accepted == 3 && q.best && std::fabs(q.best->objective - 925.0) < kTol, "N1-A record_candidates=false: no log, same counts and best");
        const auto d1 = n1_launch_node_relocate(a.base, f.ctx(3), 0, o), d2 = n1_launch_node_relocate(a.base, f.ctx(3), 0, o);
        bool det = d1.generated == d2.generated && d1.log.size() == d2.log.size();
        for (std::size_t i = 0; det && i < d1.log.size(); ++i) det = d1.log[i].solution.truck_route == d2.log[i].solution.truck_route && d1.log[i].objective == d2.log[i].objective && d1.log[i].reason == d2.log[i].reason;
        check(det, "N1-A deterministic: repeated calls give identical candidates in identical order");
        check(a.base.truck_route == before.truck_route && a.base.drone_trips.size() == 1 && a.base.drone_trips[0].launch_node == before.drone_trips[0].launch_node, "N1-A the input solution is unchanged");
    }

    // =====================================================================
    // N1-B. Endurance, inclusive boundary. Fixture A, w = 1: (36) = t_k - l_i (C4 at 400): F1500 250, F2500 150, F3500 50 (= (400-375) + 25); (37) = loaded + return:
    //       F1500 75 + 50 = 125, F2500 25 + 50, F3500 25 + 50.  Base (F3000): (36) 100, (37) 50.  e = 250: F1500 accepted (equality); e = 249.99: rejected.
    //       w = 3 adds F500: (36) 350 -> rejected at e = 250 and e = 200.
    // =====================================================================
    {
        N1Options o;
        o.record_candidates = true;
        {
            auto a = make_A(250.0);
            const auto r = n1_launch_node_relocate(a.base, a.f->ctx(1), 0, o);
            check(r.accepted == 3 && r.rejected_total() == 6 && r.best && std::fabs(r.best->objective - 925.0) < kTol, "N1-B e = 250: F1500 is accepted at (36) = 250 (inclusive bound)");
            check_records(r, a.base, *a.f, "N1-B e=250", K1, 0);
        }
        {
            auto a = make_A(249.99);
            const auto r = n1_launch_node_relocate(a.base, a.f->ctx(1), 0, o);
            check(r.accepted == 2 && r.rejected[R(RejectReason::EvalEndurance)] == 1 && r.log[0].reason == RejectReason::EvalEndurance && r.best && r.best->move.launch == a.f->flp_at(2, 4, 0), "N1-B e = 249.99: F1500 rejected as EvalEndurance, best = F2500");
            check_near(r.log[0].objective, 925.0, kTol, "N1-B a rejected candidate still carries its best-effort objective (925)");
            check_records(r, a.base, *a.f, "N1-B e=249.99", K1, 0);
        }
        {
            auto a = make_A(200.0);
            const auto r = n1_launch_node_relocate(a.base, a.f->ctx(3), 0, o);
            check(r.rejected[R(RejectReason::EvalEndurance)] == 2 && r.accepted == 2 && r.rejected[R(RejectReason::InvalidRecovery)] == 6 && r.best && std::fabs(r.best->objective - 975.0) < kTol,
                  "N1-B e = 200, w = 3: F500 (350) and F1500 (250) are EvalEndurance, F2500 (150) and F3500 accepted, best 975");
            check_records(r, a.base, *a.f, "N1-B e=200", K1, 0);
        }
    }

    // =====================================================================
    // N1-C. A shared launch FLP and an FLP not yet on the route. Fixture C (n = 5, |U| = 2): skeleton [0,1,2,4,0], T1 = (drone 0, F3000, C3, C4), T2 = (drone 1, F3000, C5, depot).
    //       Base: C1 100, C2 200, C4 400; launch 300 for both; C3 300 (T1 back at C4 350), C5 300 + 2000/20 = 400 (T2 back at the depot 400 + ... 650 < 800).  1400.
    //       Anchor T1, w = 0: F2500 -> C3 275, objective 1375 (route [0,1,2,F2500,F3000,4,0]); F3500 -> launch 350, C3 375, back at C4 425 > 400 (truck waits, depot only) -> 1475 (route [...,F3000,F3500,4,0]).
    //       Anchor T2, w = 0: F2500 -> C5 250 + 125 = 375 -> 1375; F3500 -> 350 + 75 = 425 -> 1425.  F3000 stays on the route in all four (the other trip launches there): nothing dropped.
    // =====================================================================
    {
        auto c = make_C();
        Fix& f = *c.f;
        const NodeId F2500 = f.flp_at(2, 4, 0), F3500 = f.flp_at(2, 4, 2);
        check_near(ref_objective(c.base, f, nullptr), 1400.0, kTol, "N1-C base objective (hand-derived 1400)");
        N1Options o;
        o.record_candidates = true;
        const auto t1 = n1_launch_node_relocate(c.base, f.ctx(0), 0, o);
        check(launches_of(t1) == std::vector<NodeId>({F2500, F3500}) && t1.accepted == 2, "N1-C anchor T1: candidates F2500, F3500 (F3000 is the current launch)");
        check_near(t1.log[0].objective, 1375.0, kTol, "N1-C T1 -> F2500: 1375");
        check_near(t1.log[1].objective, 1475.0, kTol, "N1-C T1 -> F3500: 1475");
        check(t1.log[0].solution.truck_route == TruckRoute({0, 1, 2, F2500, c.F3000, 4, 0}) && t1.log[1].solution.truck_route == TruckRoute({0, 1, 2, c.F3000, F3500, 4, 0}),
              "N1-C the new FLP is inserted into its gap in arc (ratio) order; the shared F3000 stays");
        check(t1.log[0].flps_dropped.empty() && t1.log[1].flps_dropped.empty(), "N1-C the old launch is shared with T2: nothing is dropped");
        check(t1.best && t1.best->move.launch == F2500 && t1.improves(), "N1-C anchor T1: best F2500 (1375 < 1400)");
        check_records(t1, c.base, f, "N1-C anchor T1", K1, 0);
        const auto t2 = n1_launch_node_relocate(c.base, f.ctx(0), 1, o);
        check(launches_of(t2) == std::vector<NodeId>({F2500, F3500}) && t2.accepted == 2, "N1-C anchor T2: candidates F2500, F3500");
        check_near(t2.log[0].objective, 1375.0, kTol, "N1-C T2 -> F2500: 1375");
        check_near(t2.log[1].objective, 1425.0, kTol, "N1-C T2 -> F3500: 1425");
        check(t2.log[0].flps_dropped.empty() && t2.log[1].flps_dropped.empty(), "N1-C T2: F3000 is still T1's launch, nothing is dropped");
        check_records(t2, c.base, f, "N1-C anchor T2", K1, 1);
        // an FLP already on the route and used by another trip is a legal (shared) target: T1 moved from F2500 onto T2's F3000 is covered by N1-D below
    }

    // =====================================================================
    // N1-D. Drone sequencing (28) and fleet capacity. Fixture S (n = 5): skeleton [0,1,3,5,0], T1 = (d1, F2000 in g1, C2, C3), T2 = (d2, F4000 in g2, C4, C5); vector [T2, T1].
    //       Base: C1 100, C3 300, C5 500; T1 launch 200, C2 200, back at C3 250; T2 launch 400, C4 400, back at C5 450.  Objective 100 + 300 + 500 + 200 + 400 = 1500.
    //       Anchor T2 (vector index 0), w = 1: gaps g1, g2, g3.
    //         g1 = arc (1,3): F1500 -> launch 150, C4 150 + 2500/20 = 275 -> 1375; F2000 (shared with T1) -> 200 + 100 = 300 -> 1400; F2500 -> 250 + 75 = 325 -> 1425
    //         g2 = arc (3,5): F3500 -> 350 + 25 = 375 -> 1475; F4500 -> 450 + 25 = 475 -> 1575 (back at C5 525 > 500: the truck waits, depot only)
    //         g3 = arc (5,0): 8 FLPs, after the recovery C5 -> InvalidRecovery.
    //       With different drones all five are feasible; with ONE drone for both trips the three g1 launches overlap T1's flight -> EvalDroneSequencing (28); with |U| = 1 the same
    //       overlap is EvalFleetCapacity (two drones in flight).
    // =====================================================================
    {
        N1Options o;
        o.record_candidates = true;
        {
            auto s = make_S(2, 0, 1);
            Fix& f = *s.f;
            check(f.idx.flps_on_arc(5, 0).size() == 8 && f.idx.flps_on_arc(1, 3).size() == 3 && f.idx.flps_on_arc(3, 5).size() == 3, "N1-D fixture sanity: 3 / 3 / 8 FLPs on arcs (1,3) (3,5) (5,0)");
            check_near(ref_objective(s.base, f, nullptr), 1500.0, kTol, "N1-D base objective (hand-derived 1500)");
            const NodeId F1500 = f.flp_at(1, 3, 0), F2500 = f.flp_at(1, 3, 2), F3500 = f.flp_at(3, 5, 0), F4500 = f.flp_at(3, 5, 2);
            const auto r = n1_launch_node_relocate(s.base, f.ctx(1), kS2, o);
            std::vector<NodeId> want = {F1500, s.F2000, F2500, F3500, F4500};
            const auto& fl50 = f.idx.flps_on_arc(5, 0);
            want.insert(want.end(), fl50.begin(), fl50.end());
            check(r.status == OperatorStatus::Ok && r.anchor_trip == kS2 && r.anchor_customer == 4 && launches_of(r) == want && r.generated == 13 && r.accepted == 5 && r.rejected[R(RejectReason::InvalidRecovery)] == 8,
                  "N1-D different drones, anchor T2 (vector index 0), w=1: 5 feasible, 8 InvalidRecovery");
            const double obj[5] = {1375, 1400, 1425, 1475, 1575};
            bool ok = true;
            for (int i = 0; i < 5; ++i) ok = ok && std::fabs(r.log[static_cast<std::size_t>(i)].objective - obj[i]) < kTol;
            check(ok, "N1-D objectives 1375, 1400 (shared F2000), 1425, 1475, 1575 (hand-derived)");
            check(r.best && r.best->move.launch == F1500 && r.improves() && r.log[1].solution.truck_route == TruckRoute({0, 1, s.F2000, 3, 5, 0}) && r.log[1].flps_dropped == std::vector<NodeId>({s.F4000}),
                  "N1-D best F1500; launching from T1's F2000 is a legal shared launch (route = base without T2's old F4000)");
            check_records(r, s.base, f, "N1-D different drones", K1, kS2);
            // anchor T1 in the same fixture: its launch moves within g0..g2 (w = 1); the first candidate is F500's neighbour on arc (1,3)... covered by the oracle; here only the anchor bookkeeping
            const auto r1 = n1_launch_node_relocate(s.base, f.ctx(1), kS1, o);
            check(r1.status == OperatorStatus::Ok && r1.anchor_trip == kS1 && r1.anchor_customer == 2, "N1-D anchor T1 (vector index 1): the anchor is the vector index, not the route order");
            check_records(r1, s.base, f, "N1-D anchor T1", K1, kS1);
        }
        {
            auto s = make_S(2, 0, 0);
            const auto r = n1_launch_node_relocate(s.base, s.f->ctx(1), kS2, o);
            check(r.rejected[R(RejectReason::EvalDroneSequencing)] == 3 && r.accepted == 2 && r.rejected[R(RejectReason::InvalidRecovery)] == 8 && r.generated == 13,
                  "N1-D one drone for both trips: the three g1 launches overlap T1's flight -> EvalDroneSequencing (28); g2 launches stay feasible");
            check(r.log[0].reason == RejectReason::EvalDroneSequencing && r.log[2].reason == RejectReason::EvalDroneSequencing && r.log[3].reason == RejectReason::None && r.best && r.best->move.launch == s.f->flp_at(3, 5, 0) && std::fabs(r.best->objective - 1475.0) < kTol,
                  "N1-D one drone: best = F3500 (1475); the drone id is NOT re-chosen to dodge the violation");
            check_records(r, s.base, *s.f, "N1-D one drone", K1, kS2);
        }
        {
            auto s = make_S(1, 0, 0);
            const auto r = n1_launch_node_relocate(s.base, s.f->ctx(1), kS2, o);
            check(r.rejected[R(RejectReason::EvalFleetCapacity)] == 3 && r.rejected[R(RejectReason::EvalDroneSequencing)] == 0 && r.accepted == 2, "N1-D |U| = 1: the same overlap is EvalFleetCapacity (two drones in flight, the evaluator's classification)");
            check_records(r, s.base, *s.f, "N1-D |U|=1", K1, kS2);
        }
    }

    // =====================================================================
    // N1-E. No alternative FLP in the window. n = 4, skeleton [0,1,2,4,0], trip (F1500, C3, C4); arc (1,2) = 1000 m holds ONE FLP.
    //       w = 0: zero candidates, status Ok.  w = 1 adds g0 (F500) and g2 (F2500, F3000, F3500): launch 150 -> C3 225 (objective 925) is the base;
    //       F500: 50 + 125 = 175 -> 875; F2500 975; F3000 1000; F3500 1075.
    // =====================================================================
    {
        auto f = make_line(4, 1);
        const NodeId F1500 = f->flp_at(1, 2, 0), F500 = f->flp_at(0, 1, 0);
        Solution base;
        base.truck_route = {0, 1, F1500, 2, 4, 0};
        base.drone_trips = {trip(0, F1500, 3, 4)};
        check_near(ref_objective(base, *f, nullptr), 925.0, kTol, "N1-E base objective (hand-derived 925)");
        N1Options o;
        o.record_candidates = true;
        const auto r0 = n1_launch_node_relocate(base, f->ctx(0), 0, o);
        check(r0.status == OperatorStatus::Ok && r0.generated == 0 && !r0.best && r0.rejected_total() == 0 && r0.log.empty(), "N1-E w = 0: the launch arc holds no other FLP -> zero candidates, status Ok");
        const auto r1 = n1_launch_node_relocate(base, f->ctx(1), 0, o);
        check(launches_of(r1) == std::vector<NodeId>({F500, f->flp_at(2, 4, 0), f->flp_at(2, 4, 1), f->flp_at(2, 4, 2)}) && r1.accepted == 4, "N1-E w = 1: g0 (F500), then g2 (F2500, F3000, F3500)");
        const double obj[4] = {875, 975, 1000, 1075};
        bool ok = true;
        for (int i = 0; i < 4; ++i) ok = ok && std::fabs(r1.log[static_cast<std::size_t>(i)].objective - obj[i]) < kTol;
        check(ok && r1.best && r1.best->move.launch == F500, "N1-E objectives 875, 975, 1000, 1075; best F500");
        check_records(r1, base, *f, "N1-E w=1", K1, 0);
    }

    // =====================================================================
    // N2-A. Fixture W (n = 5, drone 10 m/s, |U| = 1): skeleton [0,1,2,4,5,0] (s0..s5), route [0,1,2,F3500,4,5,0], trip (F3500, C3, C4), launch gap g2.
    //       Base: C1 100, C2 200, F 350, C4 400; launch 350, C3 at 350 + 500/10 = 400; back at C4 400 + 1000/10 = 500, the truck (400) waits 100, C5 at 600.
    //       Objective 100 + 200 + 400 + 600 + 400 = 1700.   (36) = 50, (37) = 150.
    //       Recovery candidates s with |s - 3| <= w, s >= 1, ascending: s1 (C1) and s2 (C2) lie BEFORE the launch (need s >= g2 + 1 = 3) -> InvalidRecovery.
    //         s4 = C5: back at 400 + 2000/10 = 600, the truck (500) waits at C5, nothing after but the depot: 100 + 200 + 400 + 500 + 400 = 1600; (36) 150, (37) 250.
    //         s5 = depot: back at 400 + 300 = 700 < 1000: no wait: 1600 (an EXACT tie with C5: the first generated, C5, wins); (36) 650, (37) 350.
    //       The start depot (s0) is never a candidate.
    // =====================================================================
    {
        auto w = make_W(3600.0);
        Fix& f = *w.f;
        bool feas = false;
        check_near(ref_objective(w.base, f, &feas), 1700.0, kTol, "N2-A base objective (hand-derived 1700, frozen reference)");
        check(feas, "N2-A base is feasible");
        const Solution before = w.base;
        N2Options o;
        o.record_candidates = true;
        const auto r0 = n2_recover_node_relocate(w.base, f.ctx(0), 0, o);
        check(r0.status == OperatorStatus::Ok && r0.op == OperatorId::N2 && r0.anchor_trip == 0 && r0.anchor_customer == 3 && r0.generated == 0 && !r0.best && r0.log.empty(), "N2-A w=0: no other node in the window -> zero candidates, status Ok");
        check_near(r0.base_objective, 1700.0, kTol, "N2-A base objective reported");
        const auto r1 = n2_recover_node_relocate(w.base, f.ctx(1), 0, o);
        check(recoveries_of(r1) == std::vector<NodeId>({2, 5}) && r1.generated == 2 && r1.evaluated == 1 && r1.accepted == 1 && r1.rejected[R(RejectReason::InvalidRecovery)] == 1,
              "N2-A w=1: s2 (C2, before the launch: InvalidRecovery) and s4 (C5, evaluated)");
        check(r1.log[0].reason == RejectReason::InvalidRecovery && !r1.log[0].evaluated && r1.log[0].affected_trips == std::vector<std::size_t>({0}) && r1.log[0].solution.drone_trips[0].launch_node == w.F3500, "N2-A the invalid candidate is named, not evaluated, shown unrepaired");
        check_near(r1.log[1].objective, 1600.0, kTol, "N2-A recovery at C5: 1600 (hand-derived)");
        const auto r3 = n2_recover_node_relocate(w.base, f.ctx(3), 0, o);
        check(recoveries_of(r3) == std::vector<NodeId>({1, 2, 5, 0}) && r3.generated == 4 && r3.accepted == 2 && r3.rejected[R(RejectReason::InvalidRecovery)] == 2, "N2-A w=3: ascending C1, C2 (both invalid), C5, end depot; the start depot is not a candidate");
        check(reasons_of(r3) == std::vector<RejectReason>({RejectReason::InvalidRecovery, RejectReason::InvalidRecovery, RejectReason::None, RejectReason::None}), "N2-A outcomes in order");
        check_near(r3.log[2].objective, 1600.0, kTol, "N2-A C5: 1600");
        check_near(r3.log[3].objective, 1600.0, kTol, "N2-A end depot: 1600 (the truck does not wait)");
        check(r3.log[2].objective == r3.log[3].objective, "N2-A the two objectives are exactly equal (tie)");
        check(r3.best && r3.best->move.recovery == 5 && r3.best->move.replaced_node == 4 && r3.improves() && std::fabs(r3.best->objective - 1600.0) < kTol, "N2-A tie -> the FIRST generated candidate (C5, not the depot) is kept; 1600 < 1700 is an improvement");
        check(r3.best && r3.best->solution.truck_route == w.base.truck_route && r3.best->solution.drone_trips[0].recovery_node == 5 && r3.best->solution.drone_trips[0].launch_node == w.F3500 && r3.best->solution.drone_trips[0].drone_id == 0,
              "N2-A result: route untouched, trip = (drone 0, F3500, C3, C5): only the recovery changed");
        check(r3.best && r3.best->flps_dropped.empty() && r3.log[3].move.recovery == 0 && r3.log[3].solution.drone_trips[0].recovery_node == 0, "N2-A nothing is dropped; the end depot is a legal recovery");
        check_records(r1, w.base, f, "N2-A w=1", K2, 0);
        check_records(r3, w.base, f, "N2-A w=3", K2, 0);
        const auto r9 = n2_recover_node_relocate(w.base, f.ctx(9), 0, o);
        check(recoveries_of(r9) == recoveries_of(r3), "N2-A w=9: the window is clipped to the route (same candidates as w=3)");
        N2Options gen = o;
        gen.evaluate_candidates = false;
        const auto g = n2_recover_node_relocate(w.base, f.ctx(3), 0, gen);
        check(g.generated == 4 && g.evaluated == 0 && g.accepted == 0 && !g.best && g.rejected_total() == 2, "N2-A evaluate_candidates=false: 4 generated, none evaluated or accepted, the 2 structural rejections counted");
        const auto q = n2_recover_node_relocate(w.base, f.ctx(3), 0);
        check(q.log.empty() && q.generated == 4 && q.accepted == 2 && q.best && q.best->move.recovery == 5, "N2-A record_candidates=false: no log, same counts and best");
        const auto d1 = n2_recover_node_relocate(w.base, f.ctx(3), 0, o), d2 = n2_recover_node_relocate(w.base, f.ctx(3), 0, o);
        bool det = d1.log.size() == d2.log.size();
        for (std::size_t i = 0; det && i < d1.log.size(); ++i) det = d1.log[i].move.recovery == d2.log[i].move.recovery && d1.log[i].objective == d2.log[i].objective && d1.log[i].reason == d2.log[i].reason;
        check(det, "N2-A deterministic: repeated calls give identical candidates in identical order");
        check(w.base.truck_route == before.truck_route && w.base.drone_trips[0].recovery_node == 4, "N2-A the input solution is unchanged");
    }

    // =====================================================================
    // N2-B. Endurance, inclusive boundary (fixture W, w = 3): C5 has (37) = 50 + 200 = 250, the depot (36) = 650.  e = 250: C5 accepted (equality), depot rejected;
    //       e = 249.99: both rejected, no best.  Base: (36) 50, (37) 150 (feasible at both).
    // =====================================================================
    {
        N2Options o;
        o.record_candidates = true;
        {
            auto w = make_W(250.0);
            const auto r = n2_recover_node_relocate(w.base, w.f->ctx(3), 0, o);
            check(r.accepted == 1 && r.rejected[R(RejectReason::EvalEndurance)] == 1 && r.rejected[R(RejectReason::InvalidRecovery)] == 2 && r.best && r.best->move.recovery == 5 && std::fabs(r.best->objective - 1600.0) < kTol, "N2-B e = 250: C5 accepted at (37) = 250 (inclusive), the depot rejected as EvalEndurance ((36) = 650)");
            check_near(r.log[3].objective, 1600.0, kTol, "N2-B the rejected depot candidate keeps its best-effort objective (1600)");
            check_records(r, w.base, *w.f, "N2-B e=250", K2, 0);
        }
        {
            auto w = make_W(249.99);
            const auto r = n2_recover_node_relocate(w.base, w.f->ctx(3), 0, o);
            check(r.accepted == 0 && !r.best && r.rejected[R(RejectReason::EvalEndurance)] == 2 && r.rejected[R(RejectReason::InvalidRecovery)] == 2 && r.generated == 4, "N2-B e = 249.99: both evaluated candidates rejected as EvalEndurance, no best");
            check_records(r, w.base, *w.f, "N2-B e=249.99", K2, 0);
        }
    }

    // =====================================================================
    // N2-C. Drone sequencing (28) and fleet capacity (fixture S, anchor T1 = vector index 1, w = 1, skeleton [0,1,3,5,0], T1's recovery C3 = s2, launch gap g1).
    //       Candidates: s1 (C1, before the launch: InvalidRecovery) and s3 (C5): T1 would be recovered at C5 and so overlap T2's flight.
    //       Different drones: feasible, objective 1500 (T1 back at C5 200 + 3000/20 = 350 < 500; no wait anywhere).
    //       One drone for both trips: EvalDroneSequencing.  |U| = 1: EvalFleetCapacity.   w = 2 adds s4 = the end depot (200 + 100 = 300 < 1000): 1500, a tie with C5 -> C5 wins.
    // =====================================================================
    {
        N2Options o;
        o.record_candidates = true;
        {
            auto s = make_S(2, 0, 1);
            const auto r = n2_recover_node_relocate(s.base, s.f->ctx(1), kS1, o);
            check(r.status == OperatorStatus::Ok && r.anchor_trip == kS1 && r.anchor_customer == 2 && recoveries_of(r) == std::vector<NodeId>({1, 5}) && r.accepted == 1 && r.rejected[R(RejectReason::InvalidRecovery)] == 1,
                  "N2-C different drones, anchor T1 (vector index 1): C1 invalid, C5 accepted");
            check_near(r.log[1].objective, 1500.0, kTol, "N2-C T1 recovered at C5: 1500 (hand-derived)");
            check(r.best && !r.improves(), "N2-C 1500 == base: accepted but no strict improvement");
            check_records(r, s.base, *s.f, "N2-C different drones", K2, kS1);
            const auto r2 = n2_recover_node_relocate(s.base, s.f->ctx(2), kS1, o);
            check(recoveries_of(r2) == std::vector<NodeId>({1, 5, 0}) && r2.accepted == 2 && r2.best && r2.best->move.recovery == 5 && r2.log[1].objective == r2.log[2].objective && std::fabs(r2.log[2].objective - 1500.0) < kTol,
                  "N2-C w=2: C5 then the end depot, both 1500, the first generated (C5) wins the tie");
            check_records(r2, s.base, *s.f, "N2-C w=2", K2, kS1);
        }
        {
            auto s = make_S(2, 0, 0);
            const auto r = n2_recover_node_relocate(s.base, s.f->ctx(1), kS1, o);
            check(r.rejected[R(RejectReason::EvalDroneSequencing)] == 1 && r.accepted == 0 && !r.best && r.log[1].reason == RejectReason::EvalDroneSequencing, "N2-C one drone for both trips: recovering T1 at C5 overlaps T2 -> EvalDroneSequencing (28); the drone id is not re-chosen");
            check_records(r, s.base, *s.f, "N2-C one drone", K2, kS1);
        }
        {
            auto s = make_S(1, 0, 0);
            const auto r = n2_recover_node_relocate(s.base, s.f->ctx(1), kS1, o);
            check(r.rejected[R(RejectReason::EvalFleetCapacity)] == 1 && r.rejected[R(RejectReason::EvalDroneSequencing)] == 0, "N2-C |U| = 1: the same overlap is EvalFleetCapacity");
            check_records(r, s.base, *s.f, "N2-C |U|=1", K2, kS1);
        }
    }

    // =====================================================================
    // N2-D. A trip that recovers at the END DEPOT (fixture C, anchor T2 = index 1, w = 1): skeleton [0,1,2,4,0], s_k = 4 (last), only s3 = C4 is in the window and is after the launch (g2).
    //       T2 back at C4: 400 + 1000/20 = 450; the truck (400) waits 50 (depot only): objective 1400, equal to the base.
    // =====================================================================
    {
        auto c = make_C();
        N2Options o;
        o.record_candidates = true;
        const auto r = n2_recover_node_relocate(c.base, c.f->ctx(1), 1, o);
        check(r.status == OperatorStatus::Ok && r.anchor_customer == 5 && recoveries_of(r) == std::vector<NodeId>({4}) && r.accepted == 1, "N2-D depot-recovery trip: the only candidate in the window is C4");
        check_near(r.log[0].objective, 1400.0, kTol, "N2-D recovery at C4: 1400 (hand-derived)");
        check(r.best && !r.improves() && r.best->move.replaced_node == 0, "N2-D accepted, not an improvement; the replaced node is the depot");
        check_records(r, c.base, *c.f, "N2-D", K2, 1);
        const auto r3 = n2_recover_node_relocate(c.base, c.f->ctx(3), 1, o);
        check(recoveries_of(r3) == std::vector<NodeId>({1, 2, 4}) && r3.rejected[R(RejectReason::InvalidRecovery)] == 2, "N2-D w=3: C1 and C2 lie before the launch (InvalidRecovery), C4 is valid");
        check_records(r3, c.base, *c.f, "N2-D w=3", K2, 1);
    }

    // =====================================================================
    // Errors, the randomised overloads, trivial and infeasible inputs (both operators).
    // =====================================================================
    {
        auto a = make_A(3600.0);
        Fix& f = *a.f;
        check(n1_launch_node_relocate(a.base, f.ctx(3), 1).status == OperatorStatus::InvalidAnchor, "N1 trip index == #trips -> InvalidAnchor");
        check(n2_recover_node_relocate(a.base, f.ctx(3), 7).status == OperatorStatus::InvalidAnchor, "N2 trip index out of range -> InvalidAnchor");
        Solution none;
        none.truck_route = {0, 1, 2, 3, 4, 0};
        check(n1_launch_node_relocate(none, f.ctx(3), 0).status == OperatorStatus::InvalidAnchor && n2_recover_node_relocate(none, f.ctx(3), 0).status == OperatorStatus::InvalidAnchor, "N1/N2 no trips: index 0 -> InvalidAnchor");
        tdr::util::Rng r0(1);
        const auto n1e = n1_launch_node_relocate(none, f.ctx(3), r0);
        const auto n2e = n2_recover_node_relocate(none, f.ctx(3), r0);
        check(n1e.status == OperatorStatus::NoEligibleAnchor && n2e.status == OperatorStatus::NoEligibleAnchor && n1e.generated == 0 && n2e.generated == 0 && n1e.op == OperatorId::N1 && n2e.op == OperatorId::N2, "N1/N2 Rng overload with no trips -> NoEligibleAnchor");
        Solution omitted = a.base;
        omitted.drone_trips.clear();
        omitted.truck_route = {0, 1, 2, 0};
        check(n1_launch_node_relocate(omitted, f.ctx(3), 0).status == OperatorStatus::InputInfeasible && n2_recover_node_relocate(omitted, f.ctx(3), 0).status == OperatorStatus::InputInfeasible, "N1/N2 omitted customers: InputInfeasible, nothing generated");
        Solution dup = a.base;
        dup.truck_route = {0, 1, 2, 3, a.F3000, 4, 0}; // C3 is also drone-served
        const auto d1 = n1_launch_node_relocate(dup, f.ctx(3), 0), d2 = n2_recover_node_relocate(dup, f.ctx(3), 0);
        check(d1.status == OperatorStatus::InputInfeasible && d1.generated == 0 && d2.status == OperatorStatus::InputInfeasible && d2.generated == 0, "N1/N2 duplicated customer: InputInfeasible, nothing generated");
        Solution recycled = a.base;
        recycled.drone_trips[0].recovery_node = 2; // recovery before the launch: the input itself violates the relationship
        check(n1_launch_node_relocate(recycled, f.ctx(3), 0).status == OperatorStatus::InputInfeasible && n2_recover_node_relocate(recycled, f.ctx(3), 0).status == OperatorStatus::InputInfeasible, "N1/N2 input with recovery before launch: InputInfeasible");

        // the randomised overloads: one uniform_index draw, anchor = vector index, every trip reachable
        auto c = make_C();
        bool det = true, matches = true, one_draw = true;
        std::set<std::size_t> anchors1, anchors2;
        for (std::uint64_t seed = 1; seed <= 40; ++seed) {
            tdr::util::Rng x(seed), y(seed), z(seed), x2(seed), z2(seed);
            const auto a1 = n1_launch_node_relocate(c.base, c.f->ctx(1), x), b1 = n1_launch_node_relocate(c.base, c.f->ctx(1), y);
            const std::size_t e1 = z.uniform_index(c.base.drone_trips.size());
            const auto d = n1_launch_node_relocate(c.base, c.f->ctx(1), e1);
            det = det && a1.anchor_trip == b1.anchor_trip && a1.generated == b1.generated && a1.accepted == b1.accepted;
            matches = matches && a1.anchor_trip == e1 && a1.generated == d.generated && a1.accepted == d.accepted && a1.op == OperatorId::N1;
            one_draw = one_draw && x.next_u64() == z.next_u64();
            anchors1.insert(a1.anchor_trip);
            const auto a2 = n2_recover_node_relocate(c.base, c.f->ctx(1), x2);
            const std::size_t e2 = z2.uniform_index(c.base.drone_trips.size());
            const auto d2b = n2_recover_node_relocate(c.base, c.f->ctx(1), e2);
            matches = matches && a2.anchor_trip == e2 && a2.generated == d2b.generated && a2.accepted == d2b.accepted && a2.op == OperatorId::N2;
            one_draw = one_draw && x2.next_u64() == z2.next_u64();
            anchors2.insert(a2.anchor_trip);
        }
        check(det, "N1 the same seed gives the same anchor and result");
        check(matches, "N1/N2 the anchor is the uniform_index(#trips)-th trip (vector index)");
        check(one_draw, "N1/N2 exactly one RNG draw is consumed");
        check(anchors1.size() == 2 && anchors2.size() == 2, "N1/N2 over 40 seeds both trips are drawn (non-vacuous)");
    }

    std::cout << "\n" << g_passes << " passed, " << g_failures << " failed\n";
    if (g_failures == 0) std::cout << "ALL TESTS PASSED\n";
    return g_failures == 0 ? 0 : 1;
}
