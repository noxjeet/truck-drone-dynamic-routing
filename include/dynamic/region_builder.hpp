#pragma once

// Dynamic extension (DM7): region builder (DM0 §8.1, §8.6; roadmap §5.1-5.5). IMPLEMENTS regions and closure ONLY. It does not optimise, does not
// call the scope hook (DM8) and does not run the certificate loop (DM9); no stagnation rule exists here (withdrawn, C9).
//
// TWO DIFFERENT OBJECTS (DM0 §4.6) - never mixed:
//   ImpactSet   : plan elements POTENTIALLY AFFECTED by inserting r. Determined by the insertion point, independent of any optimiser setting.
//                 Always a route suffix (r and everything behind it); exact under P2/P3, only an upper bound for trips straddling the gap.
//   Region      : the OPTIMISATION region = plan elements the local optimiser is PERMITTED to change. A design choice (R0..R5). Contiguous skeleton
//                 interval [lo, hi] with FIXED boundary nodes, trip-closed, never reaching behind the clean cut.
//
// Coordinates: skeleton nodes (non-FLP route entries) are numbered 0..last (route_ops.hpp). A gap g is the arc between skeleton nodes g and g+1.
// A region is the skeleton interval [lo, hi]; its gaps are lo..hi-1; the nodes lo and hi are the fixed boundary nodes (pinned: they keep their
// identity and position, only the time at them changes).
//   free truck customer  : skeleton node strictly inside, lo < s < hi
//   free trip            : not frozen and launch gap in [lo, hi-1]; closure guarantees its recovery skeleton index is <= hi
// Validity (DM0 §8.1):  V1 lo >= cut. V2 trip closure. V3 no non-frozen trip spans hi (nor lo). V4 boundaries are skeleton (non-FLP) nodes and the
// FLPs of free trips lie on gaps inside the region (FLP ownership is a property of the skeleton arc, so it is kept by construction).
// V5 operators anchored in the core cannot leave the region. MARGIN SEMANTICS (reconciled with search/candidate_window.hpp, D1):
//   half-step coordinate: truck customer at skeleton s -> h = 2s; drone customer launched in gap g -> h = 2g+1; gap g -> h = 2g+1.
//   Adjacent iff |h1 - h2| <= 2w, where w = NeighbourhoodConfig::adjacency_window (default 3). So the static parameter w=3 means a half-step
//   DISTANCE of 2w = 6 (three truck nodes); it is NOT "3 half-steps" (|dh| <= 3 would be 1.5 truck nodes and would be a different, smaller
//   window). The spec's phrase "w = 3 half-steps" cites candidate_window.hpp as its authority; the source, not the phrase, is followed here.
//   The margin is computed THROUGH tdr::search::gaps_within_window on the half-step coordinate of the extreme core gaps (never by re-deriving
//   the rule), and tests pin it to the static function. A truck-node count is only the consequence (|g'-g| <= w), not the definition.
//
// Closure (monotone, bounded => terminates): for every non-frozen trip that TOUCHES the region (launch gap inside [lo, hi-1], or launched left of lo
// and recovered strictly right of lo) enlarge to [min(lo, launch gap), max(hi, recovery)] if that keeps lo >= cut; otherwise FREEZE the trip (kept
// as is, its recovery node pinned, its customer not free, reported in `frozen_trips`). With a CLEAN cut (no trip spans it) a launch gap is never
// left of the cut, so freezing cannot occur; it is implemented as the explicit, reported fallback required by the spec and is exercised by a test
// that passes a deliberately non-clean cut.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include "dynamic/cursor.hpp"
#include "dynamic/plan.hpp"
#include "graph/travel_time.hpp"
#include "search/candidate_window.hpp"

namespace tdr::dynamic {

using tdr::model::NodeId;

// ---- skeleton view of a plan --------------------------------------------------------------------------------------------------------
struct TripInfo {
    std::size_t index = 0;                // index into Solution::drone_trips
    NodeId launch_node = tdr::model::kInvalidNodeId, customer = tdr::model::kInvalidNodeId, recovery_node = tdr::model::kInvalidNodeId;
    std::size_t launch_pos = kNoPos, recovery_pos = kNoPos;   // raw route positions (end-depot recovery = last index)
    std::size_t launch_gap = kNoPos, recovery_skel = kNoPos;  // skeleton coordinates
};

struct PlanLayout {
    bool ok = true;
    std::string why;
    tdr::search::Skeleton sk;
    std::vector<std::size_t> skel_of_pos; // kNoPos for FLP entries
    std::vector<TripInfo> trips;
    std::size_t last() const { return sk.size() - 1; }
    std::optional<std::size_t> skel_of_node(NodeId id) const {
        for (std::size_t s = 0; s < sk.size(); ++s) if (sk.node[s] == id) return s;
        return std::nullopt;
    }
};

inline PlanLayout layout_of(const tdr::model::Solution& sol, const tdr::model::NodeRegistry& nodes) {
    PlanLayout L;
    const auto& route = sol.truck_route;
    L.sk = tdr::search::make_skeleton(route, nodes);
    if (L.sk.size() < 2) { L.ok = false; L.why = "fewer than two skeleton nodes"; return L; }
    L.skel_of_pos.assign(route.size(), kNoPos);
    for (std::size_t s = 0; s < L.sk.size(); ++s) L.skel_of_pos[L.sk.route_position[s]] = s;
    for (std::size_t t = 0; t < sol.drone_trips.size(); ++t) {
        const auto& d = sol.drone_trips[t];
        TripInfo ti;
        ti.index = t; ti.launch_node = d.launch_node; ti.customer = d.service_customer; ti.recovery_node = d.recovery_node;
        for (std::size_t p = 0; p < route.size() && ti.launch_pos == kNoPos; ++p) if (route[p] == d.launch_node) ti.launch_pos = p;
        if (d.recovery_node == route.front()) ti.recovery_pos = route.size() - 1;
        else for (std::size_t p = 0; p < route.size() && ti.recovery_pos == kNoPos; ++p) if (route[p] == d.recovery_node) ti.recovery_pos = p;
        if (ti.launch_pos == kNoPos || L.skel_of_pos[ti.launch_pos] != kNoPos) { L.ok = false; L.why = "trip " + std::to_string(t) + ": launch node is not an FLP entry of the route"; return L; }
        if (ti.recovery_pos == kNoPos || L.skel_of_pos[ti.recovery_pos] == kNoPos) { L.ok = false; L.why = "trip " + std::to_string(t) + ": recovery node is not a skeleton node of the route"; return L; }
        std::size_t g = 0;
        while (g + 1 < L.sk.size() && L.sk.route_position[g + 1] < ti.launch_pos) ++g;
        ti.launch_gap = g;
        ti.recovery_skel = L.skel_of_pos[ti.recovery_pos];
        if (ti.recovery_skel <= ti.launch_gap) { L.ok = false; L.why = "trip " + std::to_string(t) + ": recovery is not behind the launch gap"; return L; }
        L.trips.push_back(ti);
    }
    return L;
}

// ---- region ---------------------------------------------------------------------------------------------------------------------------
enum class RegionKind { None /*R0*/, Knn /*R1*/, Ric /*R3, RIC core*/, Interval /*R4*/, FullTail /*R5*/ };
inline const char* region_kind_name(RegionKind k) {
    switch (k) {
        case RegionKind::None: return "R0-none";
        case RegionKind::Knn: return "R1-knn";
        case RegionKind::Ric: return "R3-ric";
        case RegionKind::Interval: return "R4-interval";
        case RegionKind::FullTail: return "R5-full-tail";
    }
    return "?";
}

struct GapReach { std::size_t gap = 0; double rho = 0, nu = 0, lhs = 0; bool ok = false; };            // gap in BASE-plan coordinates
struct GapCost { std::size_t gap = 0; double eta_r = 0, delta = 0, cost = 0; std::size_t m = 0; bool p3_exact = false, straddled = false, has_flps = false; };
struct RicDiag {
    bool used = false;
    std::vector<GapCost> costs;                // every editable gap of the base plan, in gap order
    std::vector<std::size_t> cost_rank;        // gap indices (base coordinates), ascending P3 cost, ties -> lower gap
    std::vector<GapReach> reach;               // every editable gap of the base plan, in gap order
    std::vector<std::size_t> cost_core_gaps;   // base-coordinate gaps taken into the core by cost rank
    std::vector<std::size_t> reach_core_gaps;  // base-coordinate gaps taken into the core by drone reach
    std::vector<std::size_t> core_gaps;        // union, candidate-plan coordinates (insertion gaps included), ascending
    std::size_t insertion_skel = 0;            // skeleton index k of r in the candidate plan
    std::size_t window = 0;                    // margin (w + ring) in the static window unit
};

struct Region {
    bool valid = false;
    std::string why;                      // reason when !valid
    RegionKind kind = RegionKind::None;
    std::size_t cut_skel = 0, last_skel = 0;
    bool empty = true;                    // R0: nothing may change (the insertion itself is not an optimisation)
    std::size_t lo = 0, hi = 0;           // fixed boundary skeleton nodes (meaningful iff !empty)
    std::size_t core_lo = 0, core_hi = 0; // core hull in skeleton nodes, before margin and closure
    std::vector<NodeId> free_customers;   // ascending id: interior truck customers + customers of free trips
    std::vector<std::size_t> free_trips;  // ascending trip index
    std::vector<std::size_t> frozen_trips;
    std::vector<NodeId> pinned_nodes;     // boundary nodes lo, hi, then recovery nodes of frozen trips (deduplicated)
    std::size_t closure_rounds = 0;
    bool closure_extended = false;        // closure enlarged the interval beyond core + margin
    bool boundary_spanned_by_frozen = false; // explicit flag: a frozen airborne trip crosses a boundary (never silent)
    bool restricted_variant = false;         // true iff built with a finite n_reach or n_cost > 1 (experimental variant, not the default RIC policy)
    RicDiag ric;                             // filled only by the RIC core (RegionR3 with CoreMode::Ric)

    bool is_free(NodeId c) const { return std::binary_search(free_customers.begin(), free_customers.end(), c); }
    bool operator==(const Region& o) const {
        return valid == o.valid && kind == o.kind && empty == o.empty && lo == o.lo && hi == o.hi && core_lo == o.core_lo && core_hi == o.core_hi && free_customers == o.free_customers &&
               free_trips == o.free_trips && frozen_trips == o.frozen_trips && pinned_nodes == o.pinned_nodes && closure_rounds == o.closure_rounds;
    }
};

namespace detail_region {

inline bool touches(const TripInfo& t, std::size_t lo, std::size_t hi) {
    return (t.launch_gap >= lo && t.launch_gap < hi) || (t.launch_gap < lo && t.recovery_skel > lo);
}

// Fixpoint closure. Returns the number of rounds used. Monotone: lo only decreases, hi only increases, the frozen set only grows.
inline std::size_t close(const PlanLayout& L, std::size_t cut, std::size_t& lo, std::size_t& hi, std::vector<std::size_t>& frozen, bool& extended) {
    std::size_t rounds = 0;
    for (bool changed = true; changed;) {
        changed = false;
        ++rounds;
        for (const auto& t : L.trips) {
            if (std::find(frozen.begin(), frozen.end(), t.index) != frozen.end() || !touches(t, lo, hi)) continue;
            const std::size_t nlo = std::min(lo, t.launch_gap), nhi = std::max(hi, t.recovery_skel);
            if (nlo < cut) { frozen.push_back(t.index); changed = true; continue; }
            if (nlo < lo || nhi > hi) { lo = nlo; hi = nhi; extended = true; changed = true; }
        }
    }
    std::sort(frozen.begin(), frozen.end());
    return rounds;
}

inline Region finish(const PlanLayout& L, const tdr::model::NodeRegistry& nodes, RegionKind kind, std::size_t cut, std::size_t lo, std::size_t hi, std::size_t core_lo, std::size_t core_hi) {
    Region R;
    R.kind = kind; R.cut_skel = cut; R.last_skel = L.last(); R.core_lo = core_lo; R.core_hi = core_hi;
    const std::size_t lo0 = lo, hi0 = hi;
    R.closure_rounds = close(L, cut, lo, hi, R.frozen_trips, R.closure_extended);
    (void)lo0; (void)hi0;
    R.lo = lo; R.hi = hi; R.empty = false;
    for (std::size_t s = lo + 1; s < hi; ++s) {
        const NodeId id = L.sk.node[s];
        if (nodes.contains(id) && nodes.at(id).type == tdr::model::NodeType::Customer) R.free_customers.push_back(id);
    }
    for (const auto& t : L.trips) {
        const bool frozen = std::find(R.frozen_trips.begin(), R.frozen_trips.end(), t.index) != R.frozen_trips.end();
        if (frozen) {
            if (std::find(R.pinned_nodes.begin(), R.pinned_nodes.end(), t.recovery_node) == R.pinned_nodes.end()) R.pinned_nodes.push_back(t.recovery_node);
            if ((t.launch_gap < lo && t.recovery_skel > lo) || (t.launch_gap < hi && t.recovery_skel > hi)) R.boundary_spanned_by_frozen = true;
            continue;
        }
        if (t.launch_gap >= lo && t.launch_gap < hi) { R.free_trips.push_back(t.index); R.free_customers.push_back(t.customer); }
    }
    std::sort(R.free_customers.begin(), R.free_customers.end());
    R.free_customers.erase(std::unique(R.free_customers.begin(), R.free_customers.end()), R.free_customers.end());
    R.pinned_nodes.insert(R.pinned_nodes.begin(), {L.sk.node[lo], L.sk.node[hi]});
    R.valid = true;
    return R;
}

inline std::size_t sub_clip(std::size_t a, std::size_t b, std::size_t floor_) { return a > b + floor_ ? a - b : floor_; } // max(floor_, a - b)

inline Region invalid(RegionKind kind, std::string why) { Region R; R.kind = kind; R.why = std::move(why); return R; }

} // namespace detail_region

// ---- builders -------------------------------------------------------------------------------------------------------------------------
// Extra inputs of the RIC core only: the BASE plan S (before the insertion, bound to its own World, which does not contain r) supplies the timing used by
// the P3 cost; the candidate's world supplies r's location and Se1/Se2/theta. R0/R1/R4/R5 and the gaps-only R3 ignore it.
struct RicInputs {
    const InstalledPlan* base = nullptr;
    const World* candidate_world = nullptr;
};

struct RegionContext {
    const PlanLayout& layout;
    const tdr::model::NodeRegistry& nodes;
    std::size_t cut_skel;  // skeleton index of the cut B (from clean_cut, or explicit)
    NodeId request;        // the inserted customer r (a truck customer of the candidate plan S+, behind the cut)
    const RicInputs* ric = nullptr;
};


// ---- P6 drone reach and P3 insertion cost, per editable gap of the BASE plan (DM0 §4.5, §4.3 P3, roadmap §5.4 R3) -------------------------
// All formulas are those of the approved spec; what the spec leaves open is listed in docs/dynamic/architecture_dynamic.md (DM7 addendum 2).

// Tight necessary one-way radius r_D* = v_D [e - (1+theta) Se2] / (2 + theta) (DM0 §4.5); computed from the regime's parameters, never hard-coded.
inline double drone_reach_radius(const tdr::model::DroneParameters& d, double theta, double se2) {
    return d.speed * (d.endurance_seconds - (1.0 + theta) * se2) / (2.0 + theta);
}

inline double dist_point(const tdr::model::Point2D& a, const tdr::model::Point2D& b) { return std::hypot(a.x - b.x, a.y - b.y); }
inline double dist_point_segment(const tdr::model::Point2D& p, const tdr::model::Point2D& a, const tdr::model::Point2D& b) {
    const double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
    double t = L2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    return dist_point(p, {a.x + t * dx, a.y + t * dy});
}

// Per-gap instantiation of the spec's insertion-into-the-route reach inequality (DM0 §4.5, "Insertion into the current route (sharper)"):
//   (1+theta) rho_g / v_D + (1+theta) Se2 + nu_g / v_D <= e
// with rho_g = distance from x to the SEGMENT of gap g (the launch FLP lies on that arc, so rho_g lower-bounds d_ij) and nu_g = distance from x to the
// nearest editable skeleton node strictly behind the launch gap (nodes g+1..end depot; a recovery is a truck node after the launch), a lower bound of d_jk.
// It is a NECESSARY condition for a drone trip serving x that launches on gap g (never sufficient: it ignores (36), FLP discreteness, fleet size).
// Inclusive within 1e-9 relative. [IMPLEMENTATION DECISION: the spec states the inequality with rho_line/nu over the whole editable route; restricting it
// to one gap (segment g, recovery nodes behind g) is the same inequality applied per arc, flagged as such.]
inline std::vector<GapReach> drone_reach_gaps(const InstalledPlan& base, const PlanLayout& L, std::size_t cut_skel, const tdr::model::Point2D& x, double se2, double theta) {
    std::vector<GapReach> out;
    const auto& nodes = base.world().nodes();
    const auto& d = base.world().drone();
    for (std::size_t g = cut_skel; g + 1 < L.sk.size(); ++g) {
        GapReach r; r.gap = g;
        r.rho = dist_point_segment(x, nodes.at(L.sk.node[g]).coordinates, nodes.at(L.sk.node[g + 1]).coordinates);
        r.nu = std::numeric_limits<double>::infinity();
        for (std::size_t k = g + 1; k < L.sk.size(); ++k) r.nu = std::min(r.nu, dist_point(x, nodes.at(L.sk.node[k]).coordinates));
        r.lhs = (1.0 + theta) * (r.rho / d.speed + se2) + r.nu / d.speed;
        r.ok = d.num_drones > 0 && d.speed > 0 && r.lhs <= d.endurance_seconds * (1.0 + 1e-9);
        out.push_back(r);
    }
    return out;
}

// P3 cost of inserting x as a TRUCK customer into gap g=(a,b) (DM0 §4.3 P3): cost = ETA_r + m_g * delta_g,
//   ETA_r = departure_a + tau_ar ; delta_g = tau_ar + Se1 + tau_rb - tau_ab ; m_g = truck customers at skeleton >= b + drone customers launched behind b.
// Equality with the evaluator holds only when `p3_exact` (no FLP entry on the arc, no trip launched on/after the gap, no trip straddling it); otherwise the
// value is a ranking heuristic and the flag says so. Departure times come from the base plan's evaluation.
inline std::vector<GapCost> insertion_costs(const InstalledPlan& base, const PlanLayout& L, std::size_t cut_skel, const tdr::model::Point2D& x, double se1) {
    std::vector<GapCost> out;
    const auto& w = base.world();
    const auto& nodes = w.nodes();
    const auto spans = trip_spans(base.solution());
    const auto& dep = base.eval().truck_timing.departure_time;
    auto tau = [&](double dist) { return tdr::graph::truck_travel_time(dist, w.truck()); };
    for (std::size_t g = cut_skel; g + 1 < L.sk.size(); ++g) {
        GapCost c; c.gap = g;
        const auto &pa = nodes.at(L.sk.node[g]).coordinates, &pb = nodes.at(L.sk.node[g + 1]).coordinates;
        const double tar = tau(dist_point(pa, x)), trb = tau(dist_point(x, pb)), tab = tau(dist_point(pa, pb));
        const std::size_t a_pos = L.sk.route_position[g], b_pos = L.sk.route_position[g + 1];
        c.delta = tar + se1 + trb - tab;
        c.eta_r = (a_pos < dep.size() ? dep[a_pos] : 0.0) + tar;
        for (std::size_t s = g + 1; s < L.sk.size(); ++s) if (nodes.contains(L.sk.node[s]) && nodes.at(L.sk.node[s]).type == tdr::model::NodeType::Customer) ++c.m;
        bool launched_after_a = false;
        for (const auto& t : spans) { if (t.launch > b_pos) ++c.m; if (t.launch > a_pos) launched_after_a = true; }
        Gap gp{g, a_pos, b_pos};
        c.straddled = is_straddled(spans, gp);
        c.has_flps = gap_has_flps(base.solution(), gp);
        c.p3_exact = !c.has_flps && !launched_after_a && !c.straddled;
        c.cost = c.eta_r + static_cast<double>(c.m) * c.delta;
        out.push_back(c);
    }
    return out;
}

inline std::vector<std::size_t> rank_gaps_by_cost(const std::vector<GapCost>& costs) {
    std::vector<std::size_t> idx(costs.size());
    for (std::size_t i = 0; i < idx.size(); ++i) idx[i] = i;
    std::stable_sort(idx.begin(), idx.end(), [&](std::size_t a, std::size_t b) { return costs[a].cost < costs[b].cost; }); // ties -> lower gap (stable on gap order)
    std::vector<std::size_t> gaps; for (auto i : idx) gaps.push_back(costs[i].gap);
    return gaps;
}

namespace detail_region {
// Margin through the STATIC window function on the half-step coordinate of the extreme core gaps; result clipped to [cut, end depot] in nodes.
inline void window_hull(const PlanLayout& L, std::size_t cut, std::size_t gmin, std::size_t gmax, std::size_t window, std::size_t& lo, std::size_t& hi) {
    const auto lo_gaps = tdr::search::gaps_within_window(L.sk, 2 * static_cast<int>(gmin) + 1, static_cast<int>(window));
    const auto hi_gaps = tdr::search::gaps_within_window(L.sk, 2 * static_cast<int>(gmax) + 1, static_cast<int>(window));
    lo = std::max(cut, lo_gaps.empty() ? gmin : lo_gaps.front());
    hi = std::min(L.last(), (hi_gaps.empty() ? gmax : hi_gaps.back()) + 1);
}
// base-plan gap -> candidate-plan gaps (r inserted at skeleton k: the base gap k-1 splits into k-1 and k)
inline void map_base_gap(std::size_t g, std::size_t k, std::vector<std::size_t>& out) {
    if (g + 1 < k) out.push_back(g);
    else if (g + 1 == k) { out.push_back(k - 1); out.push_back(k); }
    else out.push_back(g + 1);
}
inline bool candidate_is_base_plus_r(const PlanLayout& cand, const PlanLayout& base, std::size_t k) {
    if (cand.sk.size() != base.sk.size() + 1) return false;
    for (std::size_t s = 0, b = 0; s < cand.sk.size(); ++s) { if (s == k) continue; if (cand.sk.node[s] != base.sk.node[b++]) return false; }
    return true;
}
} // namespace detail_region

struct RegionBuilder {
    virtual ~RegionBuilder() = default;
    virtual Region build(const RegionContext& c) const = 0;
    virtual const char* name() const = 0;
};

namespace detail_region {
// common preconditions: candidate decodes, r is a truck customer strictly behind the cut
inline std::optional<std::string> precondition(const RegionContext& c, std::size_t& k) {
    if (!c.layout.ok) return "layout: " + c.layout.why;
    if (c.cut_skel + 1 >= c.layout.sk.size()) return "the cut leaves no editable gap";
    const auto ks = c.layout.skel_of_node(c.request);
    if (!ks) return "the request is not a truck customer of the candidate plan (served by a drone, or absent)";
    if (*ks <= c.cut_skel) return "the request lies at or before the cut";
    k = *ks;
    return std::nullopt;
}
inline Region around_insertion(RegionKind kind, const RegionContext& c, std::size_t margin) {
    std::size_t k = 0;
    if (auto e = precondition(c, k)) return invalid(kind, *e);
    // core = the two gaps created by the insertion: skeleton nodes [k-1, k+1]; margin in truck nodes on both sides, clipped to [cut, end depot]
    const std::size_t lo = sub_clip(k - 1, margin, c.cut_skel), hi = std::min(k + 1 + margin, c.layout.last());
    return finish(c.layout, c.nodes, kind, c.cut_skel, std::max(lo, c.cut_skel), hi, k - 1, k + 1);
}
} // namespace detail_region

struct RegionR0 final : RegionBuilder { // no re-optimisation: the empty region (insertion only)
    Region build(const RegionContext& c) const override {
        std::size_t k = 0;
        if (auto e = detail_region::precondition(c, k)) return detail_region::invalid(RegionKind::None, *e);
        Region R; R.valid = true; R.kind = RegionKind::None; R.empty = true; R.cut_skel = c.cut_skel; R.last_skel = c.layout.last(); R.lo = R.hi = k; R.core_lo = k - 1; R.core_hi = k + 1;
        return R;
    }
    const char* name() const override { return "R0-none"; }
};

enum class CoreMode {
    InsertionGaps, // legacy/default: core = the two gaps created by the insertion (no ranking, no reach). Behaviour preserved from the first DM7 increment.
    Ric            // RIC core (DM0 §8.6 / roadmap §5.4 R3): best-cost gap(s) U drone-reach launch gaps U the insertion gaps; hull; margin w; closure
};

struct RegionR3 final : RegionBuilder { // `ring` widens the margin by one window unit per ring (adaptive-ring POLICY is DM9; the primitive lives here)
    int w = 3;      // static adjacency window (NeighbourhoodConfig::adjacency_window); margin in the static window unit, see the header comment
    int ring = 0;
    CoreMode core = CoreMode::InsertionGaps;
    // DEFAULT RIC POLICY (approved, A1/A3): n_cost = 1 (the best P3-cost gap) and n_reach = 0 = UNLIMITED (every reach-feasible gap, listed by increasing rho_g, ties to the
    // lower gap). Any n_reach > 0 or n_cost > 1 is a RESTRICTED EXPERIMENTAL VARIANT, not the RIC policy; such regions carry restricted_variant = true.
    int n_cost = 1;
    int n_reach = 0;
    static RegionR3 ric(int w = 3) { RegionR3 r; r.w = w; r.core = CoreMode::Ric; return r; }

    Region build(const RegionContext& c) const override {
        const std::size_t window = static_cast<std::size_t>(std::max(0, w) + std::max(0, ring));
        std::size_t k = 0;
        if (auto e = detail_region::precondition(c, k)) return detail_region::invalid(RegionKind::Ric, *e);
        const auto& L = c.layout;
        std::vector<std::size_t> core = {k - 1, k};
        RicDiag diag;
        if (core_mode_is_ric()) {
            if (!c.ric || !c.ric->base || !c.ric->candidate_world) return detail_region::invalid(RegionKind::Ric, "RIC core needs the base plan and the candidate world (RicInputs)");
            const InstalledPlan& base = *c.ric->base;
            const PlanLayout BL = layout_of(base.solution(), base.world().nodes());
            if (!BL.ok) return detail_region::invalid(RegionKind::Ric, "base layout: " + BL.why);
            if (!detail_region::candidate_is_base_plus_r(L, BL, k)) return detail_region::invalid(RegionKind::Ric, "the candidate is not the base plan plus the request (skeleton mismatch)");
            const auto cit = c.ric->candidate_world->customers().find(c.request);
            if (cit == c.ric->candidate_world->customers().end()) return detail_region::invalid(RegionKind::Ric, "the request has no customer data in the candidate world");
            const auto x = c.nodes.at(c.request).coordinates;
            diag.used = true; diag.insertion_skel = k; diag.window = window;
            diag.costs = insertion_costs(base, BL, c.cut_skel, x, cit->second.truck_service_time);
            diag.cost_rank = rank_gaps_by_cost(diag.costs);
            diag.reach = drone_reach_gaps(base, BL, c.cut_skel, x, cit->second.drone_service_time, cit->second.generalized_flight_time_theta);
            for (std::size_t i = 0; i < diag.cost_rank.size() && i < static_cast<std::size_t>(std::max(0, n_cost)); ++i) diag.cost_core_gaps.push_back(diag.cost_rank[i]);
            std::vector<GapReach> ok_reach;
            for (const auto& r : diag.reach) if (r.ok) ok_reach.push_back(r);
            std::stable_sort(ok_reach.begin(), ok_reach.end(), [](const GapReach& a, const GapReach& b) { return a.rho < b.rho; }); // best = nearest launch arc; ties -> lower gap
            for (std::size_t i = 0; i < ok_reach.size() && (n_reach <= 0 || i < static_cast<std::size_t>(n_reach)); ++i) diag.reach_core_gaps.push_back(ok_reach[i].gap);
            for (auto g : diag.cost_core_gaps) detail_region::map_base_gap(g, k, core);
            for (auto g : diag.reach_core_gaps) detail_region::map_base_gap(g, k, core);
        }
        std::sort(core.begin(), core.end()); core.erase(std::unique(core.begin(), core.end()), core.end());
        diag.core_gaps = core;
        const std::size_t gmin = std::max(core.front(), c.cut_skel), gmax = core.back();
        std::size_t lo = 0, hi = 0;
        detail_region::window_hull(L, c.cut_skel, gmin, gmax, window, lo, hi);
        Region R = detail_region::finish(L, c.nodes, RegionKind::Ric, c.cut_skel, lo, hi, gmin, std::min(gmax + 1, L.last()));
        R.ric = std::move(diag);
        R.restricted_variant = core_mode_is_ric() && (n_reach > 0 || n_cost > 1);
        return R;
    }
    const char* name() const override { return core == CoreMode::Ric ? ((n_reach > 0 || n_cost > 1) ? "R3-ric-core-RESTRICTED-variant" : "R3-ric-core") : "R3-ric"; }
private:
    bool core_mode_is_ric() const { return core == CoreMode::Ric; }
};

struct RegionR4 final : RegionBuilder { // fixed interval of h skeleton nodes around the insertion gaps (comparator)
    int h = 3;
    Region build(const RegionContext& c) const override { return detail_region::around_insertion(RegionKind::Interval, c, static_cast<std::size_t>(std::max(0, h))); }
    const char* name() const override { return "R4-interval"; }
};

struct RegionR5 final : RegionBuilder { // full editable tail: [cut, end depot]
    Region build(const RegionContext& c) const override {
        std::size_t k = 0;
        if (auto e = detail_region::precondition(c, k)) return detail_region::invalid(RegionKind::FullTail, *e);
        return detail_region::finish(c.layout, c.nodes, RegionKind::FullTail, c.cut_skel, c.cut_skel, c.layout.last(), c.cut_skel, c.layout.last());
    }
    const char* name() const override { return "R5-full-tail"; }
};

struct RegionR1 final : RegionBuilder { // k nearest editable customers (Euclid, ties by id) + r, hull in half-steps, closure (comparator)
    int k = 3;
    Region build(const RegionContext& c) const override {
        std::size_t ks = 0;
        if (auto e = detail_region::precondition(c, ks)) return detail_region::invalid(RegionKind::Knn, *e);
        const auto& L = c.layout;
        const auto xr = c.nodes.at(c.request).coordinates;
        struct Cand { double d; NodeId id; std::size_t h; };
        std::vector<Cand> cs;
        auto dist2 = [&](NodeId id) { const auto p = c.nodes.at(id).coordinates; return (p.x - xr.x) * (p.x - xr.x) + (p.y - xr.y) * (p.y - xr.y); };
        for (std::size_t s = c.cut_skel + 1; s < L.sk.size(); ++s) { // editable truck customers (behind the cut), excluding r
            const NodeId id = L.sk.node[s];
            if (id != c.request && c.nodes.contains(id) && c.nodes.at(id).type == tdr::model::NodeType::Customer) cs.push_back({dist2(id), id, 2 * s});
        }
        for (const auto& t : L.trips) if (t.launch_gap >= c.cut_skel) cs.push_back({dist2(t.customer), t.customer, 2 * t.launch_gap + 1}); // editable drone customers
        std::sort(cs.begin(), cs.end(), [](const Cand& a, const Cand& b) { return a.d != b.d ? a.d < b.d : a.id < b.id; });
        std::size_t hmin = 2 * ks, hmax = 2 * ks;
        for (std::size_t i = 0; i < cs.size() && i < static_cast<std::size_t>(std::max(0, k)); ++i) { hmin = std::min(hmin, cs[i].h); hmax = std::max(hmax, cs[i].h); }
        const std::size_t lo = std::max<std::size_t>((hmin - 1) / 2, c.cut_skel), hi = std::min(hmax / 2 + 1, L.last()); // 2 lo < h_min, h_max < 2 hi
        return detail_region::finish(L, c.nodes, RegionKind::Knn, c.cut_skel, lo, hi, lo, hi);
    }
    const char* name() const override { return "R1-knn"; }
};

// ---- impact set -----------------------------------------------------------------------------------------------------------------------
struct ImpactSet {
    bool valid = false;
    std::string why;
    NodeId request = tdr::model::kInvalidNodeId;
    std::size_t node_a = 0, node_b = 0;          // skeleton neighbours of r in S+: the old gap (a, b)
    std::vector<NodeId> shifted_customers;       // ETA shifts by <= delta_g: truck customers at skeleton >= b and drone customers of trips launched at/after b (exact by P2/P3)
    std::vector<std::size_t> shifted_trips;      // trips launched at or after b
    std::vector<std::size_t> straddling_trips;   // launched before b, recovered at or after b: the truck waits/does not wait for the drone; endurance (36) may break (CE-1)
    std::vector<NodeId> straddling_customers;    // their drone customers: ETA NOT shifted by the insertion, but the trip's feasibility is potentially affected
};

inline ImpactSet impact_of(const PlanLayout& L, const tdr::model::NodeRegistry& nodes, NodeId r) {
    ImpactSet I;
    I.request = r;
    if (!L.ok) { I.why = "layout: " + L.why; return I; }
    const auto ks = L.skel_of_node(r);
    if (!ks || *ks == 0 || *ks + 1 >= L.sk.size()) { I.why = "the request is not an interior truck customer of the candidate plan"; return I; }
    I.node_a = *ks - 1; I.node_b = *ks + 1;
    const std::size_t pb = L.sk.route_position[I.node_b];
    for (std::size_t s = I.node_b; s < L.sk.size(); ++s) {
        const NodeId id = L.sk.node[s];
        if (nodes.contains(id) && nodes.at(id).type == tdr::model::NodeType::Customer) I.shifted_customers.push_back(id);
    }
    for (const auto& t : L.trips) {
        if (t.launch_pos >= pb) { I.shifted_trips.push_back(t.index); I.shifted_customers.push_back(t.customer); }
        else if (t.recovery_pos >= pb) { I.straddling_trips.push_back(t.index); I.straddling_customers.push_back(t.customer); }
    }
    std::sort(I.shifted_customers.begin(), I.shifted_customers.end());
    std::sort(I.straddling_customers.begin(), I.straddling_customers.end());
    I.valid = true;
    return I;
}

struct RegionReport {
    std::size_t free_customers = 0, free_trips = 0, frozen_trips = 0;
    std::size_t impact_customers = 0, impact_straddling_trips = 0;
    std::size_t free_and_impacted = 0;      // may be re-sequenced AND shifted
    std::size_t impacted_not_free = 0;      // shifted but protected (e.g. the tail behind hi): the effect the region does not let the optimiser undo
    std::size_t free_not_impacted = 0;      // re-sequenceable but not delayed by the insertion (customers between lo and r)
    double region_to_impact_ratio = 0.0;    // free_customers / impact_customers (0 if impact empty)
};

inline RegionReport describe(const Region& R, const ImpactSet& I) {
    RegionReport o;
    o.free_customers = R.free_customers.size(); o.free_trips = R.free_trips.size(); o.frozen_trips = R.frozen_trips.size();
    o.impact_customers = I.shifted_customers.size(); o.impact_straddling_trips = I.straddling_trips.size();
    for (NodeId c : I.shifted_customers) (R.is_free(c) ? o.free_and_impacted : o.impacted_not_free)++;
    o.free_not_impacted = R.free_customers.size() - o.free_and_impacted;
    o.region_to_impact_ratio = o.impact_customers ? static_cast<double>(o.free_customers) / static_cast<double>(o.impact_customers) : 0.0;
    return o;
}

// ---- independent validity check (V1-V4 and free-set consistency), computed from RAW route positions, not from the builder's coordinates ----
inline std::vector<std::string> validate_region(const Region& R, const tdr::model::Solution& sol, const tdr::model::NodeRegistry& nodes) {
    std::vector<std::string> v;
    if (!R.valid) { v.push_back("region is not valid: " + R.why); return v; }
    if (R.empty) { if (!R.free_customers.empty() || !R.free_trips.empty()) v.push_back("empty region has free elements"); return v; }
    const auto& route = sol.truck_route;
    const auto sk = tdr::search::make_skeleton(route, nodes);
    if (R.hi >= sk.size() || R.lo >= R.hi) { v.push_back("boundary indices out of range"); return v; }
    if (R.lo < R.cut_skel) v.push_back("V1: lo is behind the cut");
    const std::size_t plo = sk.route_position[R.lo], phi = sk.route_position[R.hi];
    if (tdr::search::is_flp(nodes, route[plo]) || tdr::search::is_flp(nodes, route[phi])) v.push_back("V4: a boundary is an FLP entry");
    const auto trips = trip_spans(sol);
    for (const auto& t : trips) {
        const bool frozen = std::find(R.frozen_trips.begin(), R.frozen_trips.end(), t.trip_index) != R.frozen_trips.end();
        const bool free = std::find(R.free_trips.begin(), R.free_trips.end(), t.trip_index) != R.free_trips.end();
        const bool launch_in = t.launch > plo && t.launch < phi;
        if (frozen) {
            if (free) v.push_back("trip is both free and frozen");
            if (std::find(R.free_customers.begin(), R.free_customers.end(), t.customer) != R.free_customers.end()) v.push_back("V2: a frozen trip's customer is free");
            continue;
        }
        if (launch_in && t.recovery > phi) v.push_back("V2/V3: trip launched inside the region is recovered behind hi (hi is spanned)");
        if (launch_in != free) v.push_back("V2: free-trip set disagrees with the launch positions");
        if (t.launch < plo && t.recovery > plo) v.push_back("V2: a trip spans the left boundary lo");
        if (t.launch < phi && phi < t.recovery && !launch_in && t.launch > plo) v.push_back("V3: trip spans hi");
        if (free && (t.launch <= plo || t.recovery > phi)) v.push_back("V4: a free trip's FLP or recovery is outside the region");
    }
    // free customers = interior truck customers + customers of free trips (exactly)
    std::vector<NodeId> expect;
    for (std::size_t s = R.lo + 1; s < R.hi; ++s) if (nodes.contains(sk.node[s]) && nodes.at(sk.node[s]).type == tdr::model::NodeType::Customer) expect.push_back(sk.node[s]);
    for (auto ti : R.free_trips) expect.push_back(sol.drone_trips[ti].service_customer);
    std::sort(expect.begin(), expect.end());
    expect.erase(std::unique(expect.begin(), expect.end()), expect.end());
    if (expect != R.free_customers) v.push_back("free-customer set is not (interior truck customers + free-trip customers)");
    return v;
}

// ---- protection check: what an optimiser may NOT have changed (used by DM9; independent of how the new plan was produced) ----------------
// `before` is the candidate plan the region was built on. Reported violations are structural facts, never repaired.
inline std::vector<std::string> check_region_preserved(const tdr::model::Solution& before, const tdr::model::Solution& after, const Region& R, const tdr::model::NodeRegistry& nodes) {
    std::vector<std::string> v;
    if (!R.valid) { v.push_back("invalid region"); return v; }
    const auto& rb = before.truck_route; const auto& ra = after.truck_route;
    if (R.empty) { // R0: nothing may change at all
        if (rb != ra) v.push_back("empty region: the truck route changed");
        if (before.drone_trips.size() != after.drone_trips.size()) v.push_back("empty region: the trip set changed");
        else for (std::size_t i = 0; i < before.drone_trips.size(); ++i) {
            const auto &x = before.drone_trips[i], &y = after.drone_trips[i];
            if (x.drone_id != y.drone_id || x.launch_node != y.launch_node || x.service_customer != y.service_customer || x.recovery_node != y.recovery_node) v.push_back("empty region: a trip changed");
        }
        return v;
    }
    const auto sk = tdr::search::make_skeleton(rb, nodes);
    const std::size_t plo = sk.route_position[R.lo], phi = sk.route_position[R.hi];
    if (ra.size() < (rb.size() - phi) + plo + 1) { v.push_back("route too short to contain the protected prefix and suffix"); return v; }
    for (std::size_t p = 0; p <= plo; ++p) if (rb[p] != ra[p]) { v.push_back("protected prefix (through boundary lo) changed at route position " + std::to_string(p)); break; }
    const std::size_t tailn = rb.size() - phi; // boundary hi and everything behind it
    for (std::size_t i = 0; i < tailn; ++i) if (rb[phi + i] != ra[ra.size() - tailn + i]) { v.push_back("protected suffix (from boundary hi) changed"); break; }
    // trips: every non-free (protected or frozen) trip of `before` must be present unchanged in `after`
    auto same = [](const tdr::model::DroneTrip& x, const tdr::model::DroneTrip& y) { return x.drone_id == y.drone_id && x.launch_node == y.launch_node && x.service_customer == y.service_customer && x.recovery_node == y.recovery_node; };
    std::vector<bool> used(after.drone_trips.size(), false);
    for (std::size_t t = 0; t < before.drone_trips.size(); ++t) {
        if (std::find(R.free_trips.begin(), R.free_trips.end(), t) != R.free_trips.end()) continue;
        bool found = false;
        for (std::size_t u = 0; u < after.drone_trips.size() && !found; ++u) if (!used[u] && same(before.drone_trips[t], after.drone_trips[u])) { used[u] = true; found = true; }
        if (!found) v.push_back("protected trip " + std::to_string(t) + " is missing or changed");
    }
    // everything else must be region-internal: its customer is free, its FLP and recovery lie inside (plo..phi) of the new route
    auto pos_in = [&](NodeId id) { for (std::size_t p = 0; p < ra.size(); ++p) if (ra[p] == id) return p; return kNoPos; };
    const std::size_t nlo = plo, nhi = ra.size() - tailn;
    for (std::size_t u = 0; u < after.drone_trips.size(); ++u) {
        if (used[u]) continue;
        const auto& d = after.drone_trips[u];
        if (!R.is_free(d.service_customer)) v.push_back("a non-protected trip serves a customer that is not free (" + std::to_string(d.service_customer) + ")");
        const std::size_t lp = pos_in(d.launch_node), rp = d.recovery_node == ra.front() ? ra.size() - 1 : pos_in(d.recovery_node);
        if (lp == kNoPos || lp <= nlo || lp >= nhi) v.push_back("a changed trip launches outside the region");
        if (rp == kNoPos || rp <= nlo || rp > nhi) v.push_back("a changed trip is recovered outside the region");
    }
    // customers: the middle of the new route may contain only free customers; coverage is unchanged
    for (std::size_t p = nlo + 1; p < nhi; ++p) {
        if (nodes.contains(ra[p]) && nodes.at(ra[p]).type == tdr::model::NodeType::Customer && !R.is_free(ra[p])) v.push_back("a protected customer moved into the region: " + std::to_string(ra[p]));
    }
    std::vector<NodeId> cb, ca;
    for (auto id : rb) if (nodes.contains(id) && nodes.at(id).type == tdr::model::NodeType::Customer) cb.push_back(id);
    for (auto id : ra) if (nodes.contains(id) && nodes.at(id).type == tdr::model::NodeType::Customer) ca.push_back(id);
    for (const auto& t : before.drone_trips) cb.push_back(t.service_customer);
    for (const auto& t : after.drone_trips) ca.push_back(t.service_customer);
    std::sort(cb.begin(), cb.end()); std::sort(ca.begin(), ca.end());
    if (cb != ca) v.push_back("the set of served customers changed (coverage)");
    return v;
}

// Convenience wrapper on an InstalledPlan (the candidate S+ on its tentative World) and the clean cut of the decision.
inline Region build_region(const RegionBuilder& b, const InstalledPlan& candidate, const Cut& cut, NodeId request) {
    const PlanLayout L = layout_of(candidate.solution(), candidate.world().nodes());
    return b.build(RegionContext{L, candidate.world().nodes(), cut.skel_index, request});
}
inline ImpactSet impact_of(const InstalledPlan& candidate, NodeId request) { return impact_of(layout_of(candidate.solution(), candidate.world().nodes()), candidate.world().nodes(), request); }

} // namespace tdr::dynamic
