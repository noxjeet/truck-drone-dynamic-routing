#pragma once

// Dynamic extension: scan regions (DM0 §6). Eligibility is evaluated ONCE, at arrival, against the last installed plan (and its promise
// ledger); out-of-scan requests are lost, never queued (D3, D9).
//   G0   : the whole area (reference arm).                       ScanNone : reject-all baseline B0 (everything is out of scan).
//   G3   : union over editable gaps g of the detour ellipse E_g(delta_hat_max(g))  UNION  Tube_line   (NO finite cap).
//          delta_hat_max(g) = min slack over the tail set T(g) if g is not straddled, +infinity if g is straddled (U8, APPROVED:
//          the conservative infinite bound is retained; a sharper bound is deferred).
//   G3c  : G3 INTERSECT a tube of radius varsigma*l around the editable route. EXPLICITLY NON-CONSERVATIVE: it can exclude requests that
//          have a feasible single insertion; it is a demand restriction, not a filter, and is never claimed to satisfy G2' subset G3c.
// Scope of the validated claim: G2'(truck insertion, no re-anchoring, no new drone trip) is a subset of G3 on the plan the region is built
// on (math_spec §2, DM1 probes T3/T4 [HAND]+[OBS]). Move (b) (new drone trip) and re-anchoring are NOT covered by any test.
// With PM0 (A0) every slack is +infinity, so G3 = G0.

#include <algorithm>
#include <cmath>
#include <vector>

#include "dynamic/cursor.hpp"
#include "dynamic/promise.hpp"
#include "dynamic/request.hpp"

namespace tdr::dynamic {

struct ScanContext {
    const InstalledPlan& plan;
    const PromiseLedger& ledger;
    const PromiseParams& promise;
    double omega;
    const Request& request;
};

struct ScanRegion {
    virtual ~ScanRegion() = default;
    virtual bool contains(const ScanContext& c) const = 0;
    virtual const char* name() const = 0;
};

struct ScanAll final : ScanRegion { // G0
    bool contains(const ScanContext&) const override { return true; }
    const char* name() const override { return "g0"; }
};
struct ScanNone final : ScanRegion { // baseline B0
    bool contains(const ScanContext&) const override { return false; }
    const char* name() const override { return "none"; }
};

// ---- G3 building blocks (exposed so the tests can build negative controls) -------------------------------------------------------
struct G3Variant {
    bool relax_straddled = true;             // false = naive bound: finite min over the tail set even on straddled gaps (known to be WRONG, CE-4)
    bool include_recovered_before_a = false; // true  = tail set that also contains customers of trips recovered at or before a (over-conservative)
    bool include_straddling_customers = false;
};

inline std::vector<tdr::model::NodeId> tail_set(const InstalledPlan& P, const Gap& g, const G3Variant& v = {}) {
    std::vector<tdr::model::NodeId> out;
    const auto& route = P.solution().truck_route;
    for (std::size_t i = g.b_pos; i < route.size(); ++i)
        if (P.world().nodes().at(route[i]).type == tdr::model::NodeType::Customer) out.push_back(route[i]);
    for (const auto& t : trip_spans(P.solution())) {
        if (t.launch >= g.b_pos) out.push_back(t.customer);
        else if (v.include_straddling_customers && t.recovery >= g.b_pos) out.push_back(t.customer);
        else if (v.include_recovered_before_a && t.recovery <= g.a_pos) out.push_back(t.customer);
    }
    return out;
}

inline double delta_hat_max(const InstalledPlan& P, const Gap& g, const PromiseLedger& L, const PromiseParams& pp, const G3Variant& v = {}) {
    if (!pp.enabled) return kInfinity;
    if (v.relax_straddled && is_straddled(trip_spans(P.solution()), g)) return kInfinity;
    const auto eta = eta_map(P.eval());
    double m = kInfinity;
    for (auto id : tail_set(P, g, v)) m = std::min(m, slack_of(eta, L, pp, id));
    return m;
}

inline double dist(const tdr::model::Point2D& a, const tdr::model::Point2D& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// P5: x can be inserted into arc (a, b) with detour time <= dmax iff |ax| + |xb| <= |ab| + vT (dmax - Se1); empty when dmax < Se1.
inline bool in_ellipse(const tdr::model::Point2D& a, const tdr::model::Point2D& b, const tdr::model::Point2D& x, double dmax, double vT, double se1) {
    if (std::isinf(dmax)) return true;
    if (dmax < se1) return false;
    const double lhs = dist(a, x) + dist(x, b), rhs = dist(a, b) + vT * (dmax - se1);
    return lhs <= rhs + 1e-9 * std::max(1.0, std::fabs(rhs)); // inclusive within 1e-9 relative
}

inline bool g3_ellipse_contains(const InstalledPlan& P, const Cut& cut, const PromiseLedger& L, const PromiseParams& pp, const Request& rq, const G3Variant& v = {}) {
    const auto& nodes = P.world().nodes();
    const auto& route = P.solution().truck_route;
    for (const Gap& g : editable_gaps(P, cut)) {
        const double dmax = delta_hat_max(P, g, L, pp, v);
        if (in_ellipse(nodes.at(route[g.a_pos]).coordinates, nodes.at(route[g.b_pos]).coordinates, rq.location, dmax, P.world().truck().speed, rq.se1)) return true;
    }
    return false;
}

// distance from x to the editable route polyline (skeleton nodes from the cut node to the end depot)
inline double distance_to_editable_route(const InstalledPlan& P, const Cut& cut, const tdr::model::Point2D& x) {
    const auto sk = tdr::search::make_skeleton(P.solution().truck_route, P.world().nodes());
    double best = kInfinity;
    for (std::size_t s = cut.skel_index; s + 1 < sk.size(); ++s) {
        const auto a = P.world().nodes().at(sk.node[s]).coordinates, b = P.world().nodes().at(sk.node[s + 1]).coordinates;
        const double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
        double t = L2 > 0 ? ((x.x - a.x) * dx + (x.y - a.y) * dy) / L2 : 0.0;
        t = std::max(0.0, std::min(1.0, t));
        best = std::min(best, dist(x, tdr::model::Point2D{a.x + t * dx, a.y + t * dy}));
    }
    return best;
}

// Tube_line (DM0 §4.5): (1+theta) rho_line / vD + (1+theta) Se2 + nu / vD <= e, nu = distance to the nearest editable skeleton node that can
// be a recovery node (skeleton index > cut). Necessary for a NEW DRONE TRIP (move (b)); not exercised by any DM5 test.
inline bool g3_tube_contains(const InstalledPlan& P, const Cut& cut, const Request& rq) {
    const auto sk = tdr::search::make_skeleton(P.solution().truck_route, P.world().nodes());
    double nu = kInfinity;
    for (std::size_t s = cut.skel_index + 1; s < sk.size(); ++s) nu = std::min(nu, dist(rq.location, P.world().nodes().at(sk.node[s]).coordinates));
    const double rho = distance_to_editable_route(P, cut, rq.location);
    const auto& d = P.world().drone();
    const double lhs = (1.0 + rq.theta) * rho / d.speed + (1.0 + rq.theta) * rq.se2 + nu / d.speed;
    return lhs <= d.endurance_seconds + 1e-9 * std::max(1.0, d.endurance_seconds);
}

struct ScanG3 final : ScanRegion {
    bool contains(const ScanContext& c) const override {
        const auto cut = clean_cut(c.plan, c.request.time + c.omega); // the earliest cut any later decision can have: its editable gaps are a subset
        if (!cut) return false;                                        // nothing editable: empty region (G0 would classify it as stale at decision time)
        return g3_ellipse_contains(c.plan, *cut, c.ledger, c.promise, c.request) || g3_tube_contains(c.plan, *cut, c.request);
    }
    const char* name() const override { return "g3"; }
};

struct ScanG3c final : ScanRegion { // non-conservative comparator
    double varsigma = 1.0; // tube radius in units of l = L / sqrt(n0)  [PROPOSED placeholder, not confirmed]
    double ell = 0.0;      // l in metres (set by the runner)
    bool contains(const ScanContext& c) const override {
        if (!ScanG3{}.contains(c)) return false;
        const auto cut = clean_cut(c.plan, c.request.time + c.omega);
        return distance_to_editable_route(c.plan, *cut, c.request.location) <= varsigma * ell;
    }
    const char* name() const override { return "g3c"; }
};

} // namespace tdr::dynamic
