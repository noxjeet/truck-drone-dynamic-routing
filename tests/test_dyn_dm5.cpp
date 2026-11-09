// DM5: G3 scan region, promise predicate Phi (PM1 window W, PM2 tolerance E), U1 admission chain, FIFO/queue interaction.
// Scope of every G2' statement here: single TRUCK insertions into editable gaps on the plan the region is built on (no new drone trip, no
// re-anchoring) - the tested move class. Nothing below supports a claim about G2' beyond that class. No wall-clock quantity is asserted.
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <vector>

#include "dynamic/runner.hpp"
#include "dynamic/trace.hpp"

using namespace tdr;
using namespace tdr::dynamic;
using tdr::model::DroneTrip;
using tdr::model::Node;
using tdr::model::NodeType;
using tdr::model::Solution;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { ++g_checks; if (!(cond)) { ++g_fail; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

namespace {

constexpr double kVT = 15.0;

// Drone-free line world: depot (0,0), customers 1..3 at x = 20000, 40000, 60000 on the x axis; route 0,1,2,3,0.
std::shared_ptr<const World> line_world(Solution& sol) {
    model::NodeRegistry n;
    n.add(Node{0, NodeType::Depot, {0, 0}});
    World::CustomerMap c;
    for (int i : {1, 2, 3}) {
        n.add(Node{i, NodeType::Customer, {20000.0 * i, 0}});
        model::CustomerData d; d.node_id = i; d.truck_service_time = 30; d.drone_service_time = 30; d.generalized_flight_time_theta = 1; c[i] = d;
    }
    sol.truck_route = {0, 1, 2, 3, 0};
    sol.drone_trips.clear();
    return World::create(n, c, timing::FlpDataMap{}, model::TruckParameters{kVT}, model::DroneParameters{12.0, 1800.0, 60.0, 40.0, 2});
}

// CE-4 world (DM1): depot (-3000,0), FLP f=1000 at (0,0), c1=1 (6000,0), c2=2 (12000,0), X=3 (6000,2000), c3=4 (12000,-4000);
// route depot-f-c1-c2-c3-depot, trip (f, X, c2). Gaps (depot,c1) [launch FLP on the arc: stale-FLP infeasible, CE-2] and (c1,c2) are STRADDLED.
std::shared_ptr<const World> trip_world(Solution& sol) {
    model::NodeRegistry n;
    n.add(Node{0, NodeType::Depot, {-3000, 0}});
    n.add(Node{1, NodeType::Customer, {6000, 0}});
    n.add(Node{2, NodeType::Customer, {12000, 0}});
    n.add(Node{3, NodeType::Customer, {6000, 2000}});
    n.add(Node{4, NodeType::Customer, {12000, -4000}});
    n.add(Node{1000, NodeType::FlexibleLaunchPoint, {0, 0}});
    World::CustomerMap c;
    for (int i : {1, 2, 3, 4}) { model::CustomerData d; d.node_id = i; d.truck_service_time = 30; d.drone_service_time = 30; d.generalized_flight_time_theta = 1; c[i] = d; }
    timing::FlpDataMap f; f[1000] = model::FlexibleLaunchPointData{1000, 0, 1, 0.5};
    sol.truck_route = {0, 1000, 1, 2, 4, 0};
    sol.drone_trips = {DroneTrip{0, 1000, 3, 2}};
    return World::create(n, c, f, model::TruckParameters{kVT}, model::DroneParameters{12.0, 1800.0, 60.0, 40.0, 2});
}

Request mk(std::uint32_t i, double t, double x, double y) { Request r; r.index = i; r.time = t; r.location = {x, y}; r.se1 = 30; r.se2 = 30; r.theta = 1; return r; }
PromiseParams pm(double W, double Eb, double En) { PromiseParams p; p.enabled = true; p.W = W; p.E_base = Eb; p.E_new = En; return p; }
Scenario scen(std::vector<Request> rs) { Scenario s; s.ok = true; s.n0 = 3; s.t0_ref = 5000; s.horizon = 1e7; s.requests = std::move(rs); return s; }

struct Eval { bool feasible = false; std::unordered_map<model::NodeId, double> eta; };
Eval eval_of(const InstalledPlan& base, const std::shared_ptr<const World>& w, const model::Solution& s) {
    Eval e;
    auto p = InstalledPlan::make(s, w, 0.0, base.version() + 1);
    if (p) { e.feasible = true; e.eta = eta_map(p->eval()); }
    return e;
}
bool tolerance_ok(const Eval& e, const PromiseLedger& L, const PromiseParams& pp) {
    for (const auto& kv : L.all()) {
        const auto it = e.eta.find(kv.first);
        if (it == e.eta.end() || it->second > kv.second.first_promised_eta + pp.tolerance(kv.second.klass) + 1e-9) return false;
    }
    return true;
}

// ---- T-a: Phi unit tests --------------------------------------------------------------------------------------------------------------
void test_phi_unit() {
    Solution sol; const auto w = line_world(sol);
    const auto base = InstalledPlan::make(sol, w, 0.0, 0); CHECK(base.has_value());
    const auto L = PromiseLedger::from_base_plan(*base);
    CHECK(L.size() == 3 && L.find(1)->klass == Klass::Base && L.find(1)->first_promised_eta == eta_map(base->eval()).at(1));
    // PM0: Phi reduces to evaluator feasibility
    PromiseParams off;
    const auto rq = mk(1, 0.0, 30000, 5000);
    const auto cid = customer_id(1);
    const auto tw = w->with_request(cid, rq);
    const auto cut = clean_cut(*base, 10.0); CHECK(cut.has_value());
    const auto ins = cheapest_truck_insertion(*base, *cut, cid, tw);
    CHECK(ins.best.has_value() && ins.trials.size() == 3); // gaps (1,2), (2,3), (3,0) behind the cut at customer 1
    const auto cand = InstalledPlan::make(ins.solution, tw, 10.0, 1); CHECK(cand.has_value());
    CHECK(phi(*cand, L, off, cid, 0.0).ok);
    // hand value of the detour into gap (1,2): x=(30000,5000): 2*sqrt(10000^2+5000^2)=22360.68; (22360.68-20000)/15 + 30 = 187.38 s
    const double hand = (2.0 * std::sqrt(1.0e8 + 2.5e7) - 20000.0) / kVT + 30.0;
    for (const auto& t : ins.trials) {
        if (!t.feasible || t.gap.a_pos != 1) continue;
        const auto e = eval_of(*base, tw, t.solution);
        CHECK(std::fabs((e.eta.at(2) - eta_map(base->eval()).at(2)) - hand) < 1e-6 && std::fabs((e.eta.at(3) - eta_map(base->eval()).at(3)) - hand) < 1e-6);
        CHECK(std::fabs(e.eta.at(1) - eta_map(base->eval()).at(1)) < 1e-9); // customer before the gap is untouched
        // tolerance boundary: E just below / just above the detour
        CHECK(!phi(*InstalledPlan::make(t.solution, tw, 10.0, 1), L, pm(1e9, hand - 0.01, hand - 0.01), cid, 0.0).ok);
        CHECK(phi(*InstalledPlan::make(t.solution, tw, 10.0, 1), L, pm(1e9, hand + 0.01, hand + 0.01), cid, 0.0).ok);
        // class split: base tolerance binds although the new-customer tolerance is huge; the reverse split passes (all promised customers here are base)
        CHECK(!phi(*InstalledPlan::make(t.solution, tw, 10.0, 1), L, pm(1e9, hand - 0.01, 1e9), cid, 0.0).ok);
        CHECK(phi(*InstalledPlan::make(t.solution, tw, 10.0, 1), L, pm(1e9, hand + 0.01, 0.0), cid, 0.0).ok);
        // response window: ETA_r vs tau_r + W
        const double eta_r = e.eta.at(cid);
        CHECK(phi(*InstalledPlan::make(t.solution, tw, 10.0, 1), L, pm(eta_r + 0.01 - 5.0, 1e9, 1e9), cid, 5.0).ok);   // tau_r = 5: bound = eta_r + 0.01
        CHECK(!phi(*InstalledPlan::make(t.solution, tw, 10.0, 1), L, pm(eta_r - 0.01 - 5.0, 1e9, 1e9), cid, 5.0).ok);  // bound = eta_r - 0.01
        const auto r = phi(*InstalledPlan::make(t.solution, tw, 10.0, 1), L, pm(eta_r - 0.01, 1e9, 1e9), cid, 0.0);
        CHECK(!r.ok && r.reason.find("window") != std::string::npos);
    }
    // a candidate that drops a promised customer cannot satisfy Phi (guards ledger/plan mismatch)
    PromiseLedger L2 = L; L2.set(999, Klass::New, 1.0, 0.0);
    CHECK(!phi(*cand, L2, pm(1e9, 1e9, 1e9), cid, 0.0).ok);
}

// ---- T-b: per-gap necessary condition: single truck insertion into gap g passing PM2  =>  x in E_g(delta_hat_max(g)) ------------------
struct FuzzOut { int feasible_ok = 0, inside = 0, false_excl = 0, naive_false_excl = 0, straddled_ok = 0; };
FuzzOut fuzz_gap_ellipse(const InstalledPlan& P, const Cut& cut, const PromiseLedger& L, const PromiseParams& pp, double extent, int N, unsigned seed, const G3Variant& var = {}) {
    FuzzOut o;
    std::mt19937_64 g(seed);
    std::uniform_real_distribution<double> U(-extent * 0.1, extent * 1.1);
    const auto& route = P.solution().truck_route;
    for (int k = 0; k < N; ++k) {
        const Request rq = mk(1, 0.0, U(g), U(g));
        const auto cid = customer_id(1);
        const auto tw = P.world_ptr()->with_request(cid, rq);
        const auto ins = cheapest_truck_insertion(P, cut, cid, tw);
        for (const auto& t : ins.trials) {
            if (!t.feasible) continue;
            const Eval e = eval_of(P, tw, t.solution);
            if (!e.feasible || !tolerance_ok(e, L, pp)) continue;
            ++o.feasible_ok;
            if (t.straddled) ++o.straddled_ok;
            const double dmax = delta_hat_max(P, t.gap, L, pp, var);
            const auto& nodes = P.world().nodes();
            const bool in = in_ellipse(nodes.at(route[t.gap.a_pos]).coordinates, nodes.at(route[t.gap.b_pos]).coordinates, rq.location, dmax, P.world().truck().speed, rq.se1);
            if (in) ++o.inside; else ++o.false_excl;
        }
    }
    return o;
}

void test_g3_gap_ellipse() {
    Solution sol; const auto w = line_world(sol);
    const auto base = InstalledPlan::make(sol, w, 0.0, 0);
    const auto L = PromiseLedger::from_base_plan(*base);
    const auto cut = clean_cut(*base, 10.0);
    CHECK(cut.has_value() && cut->route_pos == 1);
    for (double E : {0.0, 60.0, 200.0, 600.0, 3000.0}) {
        const auto o = fuzz_gap_ellipse(*base, *cut, L, pm(1e12, E, E), 90000.0, 150, 11);
        CHECK(o.false_excl == 0);
        CHECK(o.feasible_ok > 0 && o.inside == o.feasible_ok);
    }
    // the ellipse is tight on the line world: for gap (1,2) a point just OUTSIDE the ellipse has no tolerance-passing insertion into that gap.
    // E = 300: boundary h solves 2*sqrt(10000^2 + h^2) = 20000 + 15*(300-30) -> h = 6677.2
    const double E = 300.0, hb = std::sqrt(std::pow((20000.0 + kVT * (E - 30.0)) / 2.0, 2) - 1.0e8);
    CHECK(std::fabs(hb - 6678.4) < 0.1);
    PromiseParams pp = pm(1e12, E, E);
    const auto& route = base->solution().truck_route;
    const auto A = base->world().nodes().at(route[1]).coordinates, B = base->world().nodes().at(route[2]).coordinates;
    CHECK(in_ellipse(A, B, {30000, hb - 5.0}, E, kVT, 30.0) && !in_ellipse(A, B, {30000, hb + 5.0}, E, kVT, 30.0));
    for (double h : {hb - 5.0, hb + 5.0}) {
        const Request rq = mk(1, 0.0, 30000, h);
        const auto tw = base->world_ptr()->with_request(customer_id(1), rq);
        const auto ins = cheapest_truck_insertion(*base, *cut, customer_id(1), tw);
        bool gap12_ok = false;
        for (const auto& t : ins.trials) if (t.feasible && t.gap.a_pos == 1) gap12_ok = tolerance_ok(eval_of(*base, tw, t.solution), L, pp);
        CHECK(gap12_ok == (h < hb)); // exact agreement at +-5 m around the analytic boundary
    }
    // delta_hat_max: gap (1,2) has tail {2,3} -> E (no depletion); gap (2,3) tail {3} -> E; end gap (3,0): empty tail -> +inf
    const auto gaps = editable_gaps(*base, *cut);
    CHECK(gaps.size() == 3);
    CHECK(delta_hat_max(*base, gaps[0], L, pp) == E && delta_hat_max(*base, gaps[1], L, pp) == E && std::isinf(delta_hat_max(*base, gaps[2], L, pp)));
    // PM0: every slack is +inf
    CHECK(std::isinf(delta_hat_max(*base, gaps[0], L, PromiseParams{})));
    // depletion: after consuming 250 s of customer 3's slack its slack is E - 250; bound of gaps before it shrinks accordingly
    PromiseLedger L3 = L; L3.set(3, Klass::Base, L.find(3)->first_promised_eta - 250.0, 0.0); // pretend 3 already runs 250 s late vs its promise
    CHECK(std::fabs(delta_hat_max(*base, gaps[0], L3, pp) - (E - 250.0)) < 1e-9 && std::fabs(delta_hat_max(*base, gaps[1], L3, pp) - (E - 250.0)) < 1e-9);
}

// ---- T-c: G0 vs G3 -----------------------------------------------------------------------------------------------------------------
void test_g0_vs_g3() {
    Solution sol; const auto w = line_world(sol);
    const auto base = InstalledPlan::make(sol, w, 0.0, 0);
    const auto L = PromiseLedger::from_base_plan(*base);
    ScanAll g0; ScanG3 g3; ScanNone none;
    // PM0: G3 = G0 (all slacks +inf)
    PromiseParams off;
    std::mt19937_64 g(5);
    std::uniform_real_distribution<double> U(-5000, 65000);
    bool eq = true;
    for (int k = 0; k < 100; ++k) { const Request rq = mk(1, 0.0, U(g), U(g)); const ScanContext c{*base, L, off, 10.0, rq}; eq = eq && g3.contains(c) == g0.contains(c) && !none.contains(c); }
    CHECK(eq);
    // PM1+PM2 with E = 0 on a DRONE-FREE plan: the end gap (3,0) is editable and delays no promised customer, so G3 still equals G0.
    // (Finding recorded in docs: with tolerance-only slack G3 shrinks below G0 only when no gap is editable (empty region), or by the tube-less
    // ellipse term in plans where the end gap is not available; the informative shrinkage is G3c.)
    const PromiseParams tight = pm(1e12, 0.0, 0.0);
    bool all_in = true;
    for (int k = 0; k < 100; ++k) { const Request rq = mk(1, 0.0, U(g), U(g)); const ScanContext c{*base, L, tight, 10.0, rq}; all_in = all_in && g3.contains(c); }
    CHECK(all_in);
    // no editable gap: theta beyond the last customer -> G3 empty (G0 would classify the request as stale at decision time)
    { const Request rq = mk(1, 0.0, 1000, 1000); const ScanContext c{*base, L, tight, 1.0e6, rq}; CHECK(!g3.contains(c) && g0.contains(c)); }
    // G3c is explicitly NON-conservative: with E = infinity G2' is the whole area (every point has a feasible end insertion), G3c is not.
    ScanG3c g3c; g3c.varsigma = 0.25; g3c.ell = 20000.0; // tube of 5 km
    const Request far = mk(1, 0.0, 30000, 40000), near = mk(2, 0.0, 30000, 2000);
    const ScanContext cf{*base, L, tight, 10.0, far}, cn{*base, L, tight, 10.0, near};
    CHECK(g3.contains(cf) && !g3c.contains(cf) && g3c.contains(cn));
    const auto cut = clean_cut(*base, 10.0);
    const auto tw = base->world_ptr()->with_request(customer_id(1), far);
    const auto ins = cheapest_truck_insertion(*base, *cut, customer_id(1), tw);
    CHECK(ins.best.has_value()); // far point has a feasible truck insertion that G3c excludes: G2' is NOT a subset of G3c
}

// ---- T-d: straddled gaps, CE-2/CE-4 style world -----------------------------------------------------------------------------------------
void test_straddled() {
    Solution sol; const auto w = trip_world(sol);
    const auto base = InstalledPlan::make(sol, w, 0.0, 0); CHECK(base.has_value());
    const auto L = PromiseLedger::from_base_plan(*base);
    const auto cut = clean_cut(*base, 0.0); CHECK(cut.has_value() && cut->route_pos == 0);
    const auto gaps = editable_gaps(*base, *cut);
    CHECK(gaps.size() == 4);
    const auto spans = trip_spans(base->solution());
    CHECK(is_straddled(spans, gaps[0]) && is_straddled(spans, gaps[1]) && !is_straddled(spans, gaps[2]) && !is_straddled(spans, gaps[3]));
    // delta_hat_max of a straddled gap is +inf by the approved U8 rule; the naive finite bound (tail set only) is finite
    // DM1 CE-4 setting: only c3 (node 4) carries a finite PM2 slack (250 s); every other promised customer has effectively unlimited slack
    PromiseLedger L4;
    for (const auto& kv : L.all()) L4.set(kv.first, Klass::Base, kv.first == 4 ? kv.second.first_promised_eta : kv.second.first_promised_eta + 1e9, 0.0);
    const PromiseParams pp = pm(1e12, 250.0, 250.0);
    CHECK(std::isinf(delta_hat_max(*base, gaps[1], L4, pp)) && std::isinf(delta_hat_max(*base, gaps[0], L4, pp)));
    G3Variant naive; naive.relax_straddled = false;
    CHECK(std::fabs(delta_hat_max(*base, gaps[1], L4, pp, naive) - 250.0) < 1e-9); // tail {c2, c3}: min slack = c3's
    CHECK(std::isinf(delta_hat_max(*base, gaps[3], L, pp)));                      // end gap, empty tail
    // CE-2 behaviour: every insertion into the gap that holds the launch FLP is evaluator-infeasible (stale FLP) and is recorded, never repaired
    {
        const Request rq = mk(1, 0.0, 3000, 3000);
        const auto tw = base->world_ptr()->with_request(customer_id(1), rq);
        const auto ins = cheapest_truck_insertion(*base, *cut, customer_id(1), tw);
        CHECK(ins.trials.size() == 4 && ins.trials[0].has_flps && !ins.trials[0].feasible && ins.trials[1].straddled);
    }
    // hand CE-4 insertion r = (9000, 4758): delta_g = 380.0 s in gap (c1,c2); shift at c3 is 155.9 s (absorbed by the trip), NOT delta_g
    {
        const Request rq = mk(1, 0.0, 9000, 4758);
        const auto tw = base->world_ptr()->with_request(customer_id(1), rq);
        const auto ins = cheapest_truck_insertion(*base, *cut, customer_id(1), tw);
        const auto eb = eta_map(base->eval());
        const auto e = eval_of(*base, tw, ins.trials[1].solution);
        const double delta = (std::hypot(3000.0, 4758.0) + std::hypot(3000.0, 4758.0) - 6000.0) / kVT + 30.0;
        CHECK(std::fabs(delta - 380.0) < 0.1 && ins.trials[1].feasible && std::fabs((e.eta.at(4) - eb.at(4)) - 155.9) < 0.2 && (e.eta.at(4) - eb.at(4)) < delta);
        // with PM2 slack 250 at c3 the insertion passes Phi, the naive finite bound (250 < delta_g) excludes it, the retained +inf bound includes it
        CHECK(tolerance_ok(e, L4, pp));
        const auto& route = base->solution().truck_route; const auto& nodes = base->world().nodes();
        const auto A = nodes.at(route[gaps[1].a_pos]).coordinates, B = nodes.at(route[gaps[1].b_pos]).coordinates;
        CHECK(!in_ellipse(A, B, rq.location, 250.0, kVT, 30.0));                                  // naive bound: false exclusion
        CHECK(in_ellipse(A, B, rq.location, delta_hat_max(*base, gaps[1], L4, pp), kVT, 30.0));     // retained rule: included
    }
    // fuzz: the retained rule has no false exclusion; the naive bound is a negative control that DOES exclude feasible, tolerance-passing insertions
    int fe = 0, naive_fe = 0, ok = 0;
    for (double E : {20.0, 60.0, 150.0, 250.0, 400.0}) {
        const PromiseParams q = pm(1e12, E, E);
        PromiseLedger Lq; // finite slack E only at c3, as in DM1 T4b/CE-4
        for (const auto& kv : L.all()) Lq.set(kv.first, Klass::Base, kv.first == 4 ? kv.second.first_promised_eta : kv.second.first_promised_eta + 1e9, 0.0);
        const auto a = fuzz_gap_ellipse(*base, *cut, Lq, q, 15000.0, 400, 21);
        const auto b = fuzz_gap_ellipse(*base, *cut, Lq, q, 15000.0, 400, 21, naive);
        fe += a.false_excl; naive_fe += b.false_excl; ok += a.straddled_ok;
    }
    CHECK(fe == 0);
    CHECK(ok > 0);       // straddled insertions that pass PM2 exist in the sample
    CHECK(naive_fe > 0); // negative control: the naive finite bound would have excluded some of them (CE-4)
    std::printf("   [info] straddled gaps: feasible+PM2 straddled insertions %d, naive false exclusions %d, retained-rule false exclusions %d\n", ok, naive_fe, fe);
}

// ---- T-e: simulator: U1 chain outcomes, W, E, FIFO, expiry, Stale ------------------------------------------------------------------
struct Fx {
    Solution sol; std::shared_ptr<const World> w; std::optional<InstalledPlan> base;
    Fx() { w = line_world(sol); base = InstalledPlan::make(sol, w, 0.0, 0); }
    SimResult run(const Scenario& s, double omega, const PromiseParams& pp, const ScanRegion& scan, const AdmissionPolicy& adm, const Optimizer* opt = nullptr) const {
        SimOptions so; so.omega = omega; so.scan = &scan; so.admission = &adm; so.promise = pp; so.optimizer = opt;
        return simulate(*base, s, so);
    }
};

void test_u1_chain() {
    Fx f; ScanAll g0; AdmitAPM apm; AdmitA0 a0;
    const Request rq = mk(1, 0.0, 30000, 5000); // detours gaps (1,2) and (2,3) only; the end gap (3,0) detours nobody
    const double hand = (2.0 * std::sqrt(1.0e8 + 2.5e7) - 20000.0) / kVT + 30.0; // 187.38 s into gap (1,2)
    // identify the cheapest and the min-ETA gaps from the evaluator (used only to place thresholds)
    const auto cid = customer_id(1);
    const auto tw = f.base->world_ptr()->with_request(cid, rq);
    const auto cut = clean_cut(*f.base, 10.0);
    const auto ins = cheapest_truck_insertion(*f.base, *cut, cid, tw);
    CHECK(ins.best.has_value() && ins.trials[*ins.best].gap.a_pos == 1); // the cheapest insertion is into gap (1,2)
    double eta_min = 1e18, eta_end = 0, eta_cheapest = eval_of(*f.base, tw, ins.solution).eta.at(cid);
    for (const auto& t : ins.trials) if (t.feasible) { const double e = eval_of(*f.base, tw, t.solution).eta.at(cid); eta_min = std::min(eta_min, e); if (t.gap.a_pos == 3) eta_end = e; }
    CHECK(eta_end > eta_cheapest && std::fabs(eta_min - eta_cheapest) < 1e-9);

    // PM0 (A0) baseline: accepted, cheapest plan
    const SimResult r0 = f.run(scen({rq}), 10.0, PromiseParams{}, g0, a0);
    CHECK(r0.ok && r0.records[0].outcome == Outcome::Accepted && r0.records[0].stage == Stage::None);
    // A-PM with infinite W and E is identical to A0 (Phi reduces to feasibility)
    const SimResult ri = f.run(scen({rq}), 10.0, pm(kInfinity, kInfinity, kInfinity), g0, apm);
    CHECK(ri.ok && ri.records[0].outcome == Outcome::Accepted && ri.records[0].stage == Stage::Primary && ri.final_objective == r0.final_objective);
    // (1) Primary: E above the detour
    const SimResult p1 = f.run(scen({rq}), 10.0, pm(1e9, hand + 1.0, hand + 1.0), g0, apm);
    CHECK(p1.ok && p1.records[0].outcome == Outcome::Accepted && p1.records[0].stage == Stage::Primary && p1.final_objective == r0.final_objective);
    CHECK(p1.shift_base.violated == 0 && p1.shift_base.max <= hand + 1.0 + 1e-9 && std::fabs(p1.shift_base.max - hand) < 1e-6);
    // (2) Fallback: E below the detour of the cheapest and of gap (2,3); only the end gap passes
    const SimResult fb = f.run(scen({rq}), 10.0, pm(1e9, hand - 1.0, hand - 1.0), g0, apm);
    CHECK(fb.ok && fb.records[0].outcome == Outcome::Accepted && fb.records[0].stage == Stage::Fallback && fb.records[0].phi_primary_failed);
    CHECK(fb.records[0].eta_at_install > eta_cheapest && std::fabs(fb.records[0].eta_at_install - eta_end) < 1e-6 && fb.final_objective > r0.final_objective);
    CHECK(fb.shift_base.max < 1e-9 && fb.stage_fallback == 1 && fb.stage_primary == 0 && fb.records[0].note.find("tolerance") != std::string::npos);
    // (3) reject-promise: tolerance excludes the cheap gaps AND the window excludes the end gap (W between eta_min and eta_end)
    const double W3 = 0.5 * (eta_min + eta_end);
    const SimResult rj = f.run(scen({rq}), 10.0, pm(W3, hand - 1.0, hand - 1.0), g0, apm);
    CHECK(rj.ok && rj.records[0].outcome == Outcome::RejectedPromise && rj.final_plan->version() == 0 && rj.final_objective == rj.base_objective && rj.records[0].phi_primary_failed);
    // the same window with a tolerant E: accepted (primary), so the rejection above is due to the COMBINATION
    const SimResult ok3 = f.run(scen({rq}), 10.0, pm(W3, hand + 1.0, hand + 1.0), g0, apm);
    CHECK(ok3.records[0].outcome == Outcome::Accepted && ok3.records[0].stage == Stage::Primary);
    // (4) rejected-no-insertion is distinct from rejected-promise: nothing editable but a cut at the end is Stale; no insertion at all -> see Stale test below
    // response window W alone: boundary at eta_min (tau_r = 0)
    CHECK(f.run(scen({rq}), 10.0, pm(eta_min + 0.01, 1e9, 1e9), g0, apm).records[0].outcome == Outcome::Accepted);
    CHECK(f.run(scen({rq}), 10.0, pm(eta_min - 0.01, 1e9, 1e9), g0, apm).records[0].outcome == Outcome::RejectedPromise);
    // an optimiser hook that proposes a worse plan first (the most-delaying insertion) cannot change WHETHER the request is accepted [P15]
    struct WorstFirst final : Optimizer {
        std::optional<model::Solution> optimise(const AdmissionInput& in, const model::Solution&) const override {
            std::size_t w = 0; double worst = -1;
            for (std::size_t i = 0; i < in.ins.trials.size(); ++i) if (in.ins.trials[i].feasible && in.ins.trials[i].objective > worst) { worst = in.ins.trials[i].objective; w = i; }
            return in.ins.trials[w].solution;
        }
    } worst;
    for (double E : {hand - 1.0, hand + 1.0, 1e9}) {
        for (double W : {W3, 1e9}) {
            const bool a = f.run(scen({rq}), 10.0, pm(W, E, E), g0, apm).records[0].outcome == Outcome::Accepted;
            const bool b = f.run(scen({rq}), 10.0, pm(W, E, E), g0, apm, &worst).records[0].outcome == Outcome::Accepted;
            CHECK(a == b || (b && !a)); // acceptance set with an optimiser contains the identity-optimiser set
        }
    }
    // ... and an installed optimiser plan still satisfies Phi (I7 re-checked inside the simulator)
    const SimResult w1 = f.run(scen({rq}), 10.0, pm(1e9, hand + 1.0, hand + 1.0), g0, apm, &worst);
    CHECK(w1.ok && w1.records[0].outcome == Outcome::Accepted && w1.shift_base.violated == 0);
}

void test_fifo_expiry_stale() {
    Fx f; ScanAll g0; AdmitAPM apm;
    // omega = 900, W = 4500. Plan: customer arrivals 1333.3, 2696.7, 4060.0, depot 5393.3.
    // r1 tau=0 starts 0 installs 900 (cut at customer 1, t=1333); r2 tau=10 waits: at s=900, s+omega = 1800 <= 10+4500: decided (starts 900, installs 1800).
    // r3 tau=20: s=1800, s+omega = 2700 <= 4520: decided (start 1800, install 2700). r4 tau=30: s=2700, 3600 <= 4530: decided.
    // r5 tau=40: s=3600, s+omega = 4500 <= 4540: decided. r6 tau=50: s=4500, 5400 > 50+4500: EXPIRED, no omega consumed.
    // r7 tau=4600: arrives while r5 is being decided; dequeued at s=4500 (r6 dropped, no omega) -> starts at 4500 (not 5400).
    std::vector<Request> rs;
    for (int i = 1; i <= 6; ++i) rs.push_back(mk(i, 10.0 * (i - 1) + (i == 1 ? 0.0 : 0.0), 70000.0 + 1000.0 * i, 3000.0));
    const SimResult R = f.run(scen(rs), 900.0, pm(4500.0, 1e12, 1e12), g0, apm);
    CHECK(R.ok);
    // FIFO: decision starts are non-decreasing in index order and each start = max(arrival, previous install) over decided requests
    double prev_install = 0.0; bool fifo = true; int expired = 0;
    for (const auto& d : R.records) {
        if (d.outcome == Outcome::Expired) { ++expired; fifo = fifo && d.start == d.install; continue; }
        fifo = fifo && std::fabs(d.start - std::max(d.arrival, prev_install)) < 1e-9 && std::fabs(d.install - (d.start + 900.0)) < 1e-9;
        prev_install = d.install;
    }
    CHECK(fifo);
    CHECK(expired >= 1);
    // an expired request consumed no optimiser time: the next decided request starts at the install of the previous decided one
    for (std::size_t i = 1; i < R.records.size(); ++i)
        if (R.records[i].outcome == Outcome::Expired) {
            CHECK(R.records[i].start == R.records[i - 1].install || R.records[i].start >= R.records[i - 1].install);
        }
    // Expiry is determined by tau (arrival), not by the decision start: same scenario with W = 4500 and arrivals all at 0 gives expiry when s+omega > W
    std::vector<Request> same;
    for (int i = 1; i <= 8; ++i) same.push_back(mk(i, 0.0, 70000.0, 1000.0 * i));
    const SimResult S = f.run(scen(same), 900.0, pm(4500.0, 1e12, 1e12), g0, apm);
    // starts 0,900,1800,2700,3600 satisfy s+900 <= 4500; start 4500 does not -> requests 6..8 expire (they are dropped at s = 4500)
    int dec = 0, exp = 0;
    for (const auto& d : S.records) { if (d.outcome == Outcome::Expired) { ++exp; CHECK(d.start == 4500.0); } else ++dec; }
    CHECK(dec == 5 && exp == 3 && S.counts[static_cast<int>(Outcome::Expired)] == 3);
    // with PM0 nothing ever expires, however long the queue
    AdmitA0 a0;
    const SimResult N = f.run(scen(same), 900.0, PromiseParams{}, g0, a0);
    CHECK(N.counts[static_cast<int>(Outcome::Expired)] == 0);
    // convention C2: a decision with no editable gap is Stale and STILL consumes omega (r after the cut has reached the end depot)
    std::vector<Request> late = {mk(1, 0.0, 70000, 0), mk(2, 1.0, 71000, 0), mk(3, 2.0, 72000, 0)};
    const SimResult T = f.run(scen(late), 3000.0, PromiseParams{}, g0, a0); // theta = 3000, 6000, 9000: cut moves to customer 3 then past the last customer
    int stale = 0; for (const auto& d : T.records) if (d.outcome == Outcome::Stale) { ++stale; CHECK(d.install - d.start == 3000.0); }
    CHECK(stale >= 1);
    // convention C1: a rejected decision (reject-promise) also occupies omega
    const Request rq = mk(1, 0.0, 30000, 5000);
    const SimResult RJ = f.run(scen({rq, mk(2, 1.0, 31000, 5000)}), 10.0, pm(1e9, 0.0, 0.0), g0, apm); // E=0: gaps (1,2),(2,3) rejected, end gap accepted; use W to force rejection
    CHECK(RJ.ok);
    const SimResult RJ2 = f.run(scen({rq, mk(2, 1.0, 31000, 5000)}), 10.0, pm(500.0, 0.0, 0.0), g0, apm);
    CHECK(RJ2.records[0].outcome == Outcome::RejectedPromise && RJ2.records[0].install - RJ2.records[0].start == 10.0 && RJ2.records[1].start == 10.0);
}

void test_first_promise_and_classes() {
    Fx f; ScanAll g0; AdmitAPM apm;
    // r1 is accepted with its ETA as first promise; r2 (arriving later) is subject to the tolerance of r1's FIRST promise (no ratchet)
    const Request r1 = mk(1, 0.0, 70000, 0), r2 = mk(2, 1.0, 30000, 5000);
    const SimResult R = f.run(scen({r1, r2}), 10.0, pm(1e9, 100.0, 100.0), g0, apm);
    CHECK(R.ok && R.shift_base.violated == 0 && R.shift_new.violated == 0 && R.shift_base.max <= 100.0 + 1e-9 && R.shift_new.max <= 100.0 + 1e-9);
    CHECK(R.ledger_size == 3 + static_cast<std::size_t>(R.counts[static_cast<int>(Outcome::Accepted)]));
    // class split: base customers frozen (E_base = 0) while new customers may drift: nothing before the new customers' position is ever delayed
    const SimResult S = f.run(scen({r1, r2}), 10.0, pm(1e9, 0.0, 1e9), g0, apm);
    CHECK(S.ok && S.shift_base.max < 1e-9 && S.shift_base.violated == 0);
}

// ---- T-e2: the missing rejection path under A0: a trip recovered at the END depot makes a distant end insertion break (36) -------------
void test_rejected_no_insertion() {
    model::NodeRegistry n; n.add(Node{0, NodeType::Depot, {0, 0}}); n.add(Node{1, NodeType::Customer, {12000, 0}}); n.add(Node{2, NodeType::Customer, {6000, 3000}});
    n.add(Node{1000, NodeType::FlexibleLaunchPoint, {6000, 0}});
    World::CustomerMap c; for (int i : {1, 2}) { model::CustomerData d; d.node_id = i; d.truck_service_time = 30; d.drone_service_time = 30; d.generalized_flight_time_theta = 1; c[i] = d; }
    timing::FlpDataMap f; f[1000] = model::FlexibleLaunchPointData{1000, 0, 1, 0.5};
    Solution sol; sol.truck_route = {0, 1000, 1, 0}; sol.drone_trips = {DroneTrip{0, 1000, 2, 0}}; // trip (f, X, END DEPOT): (36) = 1510 s of e = 1800 s
    const auto w = World::create(n, c, f, model::TruckParameters{kVT}, model::DroneParameters{12.0, 1800.0, 60.0, 40.0, 2});
    const auto base = InstalledPlan::make(sol, w, 0.0, 0); CHECK(base.has_value());
    const auto e36 = base->eval().drone_trip_timings[0].endurance_value_36;
    CHECK(std::fabs(e36 - 1510.0) < 1e-6);
    Scenario sc = scen({mk(1, 0.0, 9000, -30000), mk(2, 0.0, 9000, 0)}); sc.n0 = 2;
    ScanAll g0; AdmitA0 a0; AdmitAPM apm;
    SimOptions so; so.omega = 0.0; so.scan = &g0; so.admission = &a0;      // omega = 0 keeps the cut at the depot (a cut behind the launch FLP is spanned)
    const SimResult R = simulate(*base, sc, so);
    // r1: far from the route: the only evaluator-feasible gap does not exist (detour pushes the depot arrival and breaks (36)) -> rejected-no-insertion
    CHECK(R.ok && R.records[0].outcome == Outcome::RejectedNoInsertion && R.records[0].gaps_feasible == 0 && R.records[0].gaps_tried == 2 && R.final_plan->version() <= 1);
    // r2: a +30 s service insertion on the way back fits the 290 s of endurance margin... but r1's slot is gone only if r1 had been accepted; here it was not
    CHECK(R.records[1].outcome == Outcome::Stale || R.records[1].outcome == Outcome::Accepted);
    // the same single far request under A-PM is ALSO rejected-no-insertion (not rejected-promise): Phi is never reached
    SimOptions sp = so; sp.admission = &apm; sp.promise = pm(1e9, 1e9, 1e9);
    Scenario one = scen({mk(1, 0.0, 9000, -30000)}); one.n0 = 2;
    const SimResult P = simulate(*base, one, sp);
    CHECK(P.ok && P.records[0].outcome == Outcome::RejectedNoInsertion && P.counts[static_cast<int>(Outcome::RejectedPromise)] == 0);
    // a near request is accepted
    Scenario near1 = scen({mk(1, 0.0, 9000, 0)}); near1.n0 = 2;
    CHECK(simulate(*base, near1, so).records[0].outcome == Outcome::Accepted);
}

// ---- T-f: end-to-end properties on a real base plan ---------------------------------------------------------------------------------
void test_end_to_end() {
    DynamicConfig c; c.base.base.instance.num_customers = 10; c.base.vns.iter_max = 150; c.rho = 1.0; c.omega = 60.0; c.scenarios = 3; c.regime = 'F';
    // (1) G0/A0 baseline is unchanged by DM5 (numbers recorded before DM5 for n=10, rho=1, vns-iters=300)
    DynamicConfig b = c; b.base.vns.iter_max = 300; b.scenarios = 1; b.regime = 'P'; b.omega = 10.0;
    const DynamicRun B = run_dynamic(b);
    CHECK(B.ok && std::fabs(B.runs[0].sim.base_objective - 50766.2) < 0.1 && std::fabs(B.runs[0].sim.final_objective - 200716.0) < 1.0);
    // (2) A-PM with W = E = inf equals A0, outcome by outcome, and G3 = G0 under PM0
    DynamicConfig a0 = c, apm = c, g3 = c;
    apm.admission = "apm"; apm.w_tilde = 1e9; apm.eps_tilde = 1e9;
    g3.scan = "g3";
    const DynamicRun R0 = run_dynamic(a0), RA = run_dynamic(apm), R3 = run_dynamic(g3);
    CHECK(R0.ok && RA.ok && R3.ok);
    bool same = R0.runs.size() == RA.runs.size();
    for (std::size_t k = 0; same && k < R0.runs.size(); ++k)
        for (std::size_t i = 0; same && i < R0.runs[k].sim.records.size(); ++i)
            same = R0.runs[k].sim.records[i].outcome == RA.runs[k].sim.records[i].outcome && R0.runs[k].sim.records[i].delta_objective == RA.runs[k].sim.records[i].delta_objective &&
                   R0.runs[k].sim.records[i].outcome == R3.runs[k].sim.records[i].outcome;
    CHECK(same);
    // (3) scenarios are untouched by the policy (pairing)
    for (std::size_t k = 0; k < R0.runs.size(); ++k) CHECK(R0.runs[k].scenario.digest == RA.runs[k].scenario.digest);
    // (4) A-PM with finite W, E: zero tolerance violations, I7 holds (sim.ok), partition of outcomes
    for (double eps : {0.0, 0.5, 2.0}) {
        DynamicConfig d = c; d.admission = "apm"; d.scan = "g3"; d.w_tilde = 2.0; d.eps_tilde = eps; d.scenarios = 3;
        const DynamicRun D = run_dynamic(d);
        CHECK(D.ok && D.promise.enabled);
        for (const auto& r : D.runs) {
            CHECK(r.sim.ok && r.sim.shift_base.violated == 0 && r.sim.shift_new.violated == 0);
            CHECK(r.sim.shift_base.max <= D.promise.E_base + 1e-6 && r.sim.shift_new.max <= D.promise.E_new + 1e-6);
            std::size_t tot = 0; for (auto n : r.sim.counts) tot += n;
            CHECK(tot == r.scenario.requests.size());
            CHECK(r.sim.stage_primary + r.sim.stage_fallback == r.sim.counts[static_cast<int>(Outcome::Accepted)]);
        }
    }
    // (5) validator: W must exceed omega; unknown policies rejected
    DynamicConfig v = c; v.admission = "apm"; v.w_tilde = 1e-3; CHECK(!validate(v).empty());
    DynamicConfig u = c; u.scan = "g3c"; u.admission = "apm"; CHECK(validate(u).empty());
    // (6) replay: byte-identical outputs
    DynamicConfig d = c; d.admission = "apm"; d.scan = "g3c"; d.eps_tilde = 0.5; d.scenarios = 2;
    const DynamicRun D1 = run_dynamic(d), D2 = run_dynamic(d);
    const auto o1 = render_outputs(D1.cfg, D1.base, D1.runs, D1.promise), o2 = render_outputs(D2.cfg, D2.base, D2.runs, D2.promise);
    bool eq = o1.size() == o2.size();
    for (std::size_t i = 0; eq && i < o1.size(); ++i) eq = o1[i].content == o2[i].content;
    CHECK(D1.ok && eq);
}

// ---- T-g: every outcome is represented correctly in the output files (DM6) -------------------------------------------------------
void test_output_representation() {
    DynamicConfig c; c.base.base.instance.num_customers = 20; c.base.vns.iter_max = 100; c.rho = 2.0; c.omega = 600.0; c.scenarios = 2; c.admission = "apm"; c.scan = "g3"; c.w_tilde = 1.5; c.eps_tilde = 2.0;
    const DynamicRun R = run_dynamic(c);
    CHECK(R.ok);
    const auto files = render_outputs(R.cfg, R.base, R.runs, R.promise);
    std::size_t seen_kinds = 0;
    for (std::size_t k = 0; k < R.runs.size(); ++k) {
        const std::string csv = requests_csv(R.runs[k]);
        std::size_t tally[6] = {0, 0, 0, 0, 0, 0}, rows = 0, pos = csv.find('\n') + 1;
        while (pos < csv.size()) {
            const std::size_t e = csv.find('\n', pos);
            const std::string line = csv.substr(pos, e - pos);
            std::size_t col = 0, st = 0; std::string field;
            for (std::size_t i = 0; i <= line.size(); ++i) if (i == line.size() || line[i] == ',') { if (col == 5) field = line.substr(st, i - st); ++col; st = i + 1; }
            for (int o = 0; o < 6; ++o) if (field == outcome_name(static_cast<Outcome>(o))) ++tally[o];
            ++rows; pos = e + 1;
        }
        CHECK(rows == R.runs[k].scenario.requests.size());
        for (int o = 0; o < 6; ++o) { CHECK(tally[o] == R.runs[k].sim.counts[o]); if (tally[o]) ++seen_kinds; }
        // out-of-scan and expired requests never carry a decision: start == install, no cut, no objective change
        for (const auto& d : R.runs[k].sim.records)
            if (d.outcome == Outcome::OutOfScan || d.outcome == Outcome::Expired) CHECK(d.start == d.install && !d.had_cut && d.delta_objective == 0.0 && d.eta_at_install == 0.0);
            else if (d.outcome == Outcome::Stale) CHECK(!d.had_cut && std::fabs(d.install - d.start - 600.0) < 1e-6);
            else if (d.outcome == Outcome::Accepted) CHECK(d.had_cut && std::fabs(d.install - d.start - 600.0) < 1e-6 && d.eta_at_install >= d.install - 1e-6);
            else CHECK(std::fabs(d.install - d.start - 600.0) < 1e-6 && d.delta_objective == 0.0);
    }
    CHECK(seen_kinds >= 3);
    bool json_has = false; for (const auto& f : files) if (f.name == "results.json") json_has = f.content.find("\"rejected-promise\"") != std::string::npos && f.content.find("\"expired\"") != std::string::npos && f.content.find("promise_enabled") != std::string::npos;
    CHECK(json_has);
    // human-readable trace (demo): deterministic, read-only, labels the optimiser and the prototype, and shows one line per outcome present
    const std::string t1 = trace_text(R.cfg, R.base, R.runs[0], R.promise), t2 = trace_text(R.cfg, R.base, R.runs[0], R.promise);
    CHECK(t1 == t2 && t1.find("IDENTITY") != std::string::npos && t1.find("truck-only insertion") != std::string::npos && t1.find("INITIAL PLAN") != std::string::npos && t1.find("FINAL PLAN") != std::string::npos);
    const auto& cn = R.runs[0].sim.counts;
    CHECK((cn[static_cast<int>(Outcome::Expired)] == 0) == (t1.find("EXPIRED  r") == std::string::npos));
    CHECK((cn[static_cast<int>(Outcome::Stale)] == 0) == (t1.find("decision ends: STALE") == std::string::npos));
    CHECK((cn[static_cast<int>(Outcome::RejectedPromise)] == 0) == (t1.find("REJECTED-PROMISE") == std::string::npos));
    CHECK((cn[static_cast<int>(Outcome::Accepted)] == 0) == (t1.find("decision ends: ACCEPTED") == std::string::npos));
    CHECK((cn[static_cast<int>(Outcome::OutOfScan)] == 0) == (t1.find("OUT OF SCAN") == std::string::npos));
}

} // namespace

int main() {
    test_phi_unit();
    test_g3_gap_ellipse();
    test_g0_vs_g3();
    test_straddled();
    test_u1_chain();
    test_fifo_expiry_stale();
    test_first_promise_and_classes();
    test_rejected_no_insertion();
    test_end_to_end();
    test_output_representation();
    std::printf("test_dyn_dm5: %d checks, %d failures\n", g_checks, g_fail);
    return g_fail == 0 ? 0 : 1;
}
