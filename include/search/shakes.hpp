#pragma once

// The four shaking operators of Sec 5.4 (M6, 9 Nov 2026): Random_Reverse, Random_Exchange, Random_Shift, Random_Drone_Remove.
// Contracts: docs/static/shakes.md (written before this file). Summary:
//  * S1--S3 permute the truck SKELETON (interior truck-served customers); the depots are fixed. FLPs follow their directed arcs:
//    an arc that survives as a consecutive pair keeps its FLPs and trips; a vanished arc's FLPs leave the route.
//  * D6: only trips whose launch/recovery relationship became invalid (launch arc vanished; recovery not strictly after the launch)
//    are dissolved; every other trip is preserved untouched; count and reason are recorded; nothing is repaired or re-anchored.
//  * S4 removes every trip of one randomly chosen drone (recorded as REMOVED by the shake, not as dissolved).
//  * Customers returned to the truck are inserted into a uniformly drawn FREE gap (no FLP in it); if no gap is free, a uniformly
//    drawn gap is used and the trips launching in it are dissolved (ArcSplitByReinsertion).
//  * The result is judged by the evaluator and returned as judged (it may be infeasible); it is never altered.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "search/move_types.hpp"
#include "search/route_ops.hpp"
#include "util/rng.hpp"

namespace tdr::search {

enum class ShakeKind { Reverse, Exchange, Shift, DroneRemove };

inline const char* shake_kind_name(ShakeKind k) {
    switch (k) {
        case ShakeKind::Reverse: return "random-reverse";
        case ShakeKind::Exchange: return "random-exchange";
        case ShakeKind::Shift: return "random-shift";
        case ShakeKind::DroneRemove: return "random-drone-remove";
    }
    return "?";
}

enum class ShakeStatus { Ok, InputInfeasible, NotApplicable, InvalidParameters };

inline const char* shake_status_name(ShakeStatus s) {
    switch (s) {
        case ShakeStatus::Ok: return "ok";
        case ShakeStatus::InputInfeasible: return "input-infeasible";
        case ShakeStatus::NotApplicable: return "not-applicable";
        case ShakeStatus::InvalidParameters: return "invalid-parameters";
    }
    return "?";
}

enum class DissolveReason { LaunchArcVanished, RecoveryNotAfterLaunch, ArcSplitByReinsertion };

inline const char* dissolve_reason_name(DissolveReason r) {
    switch (r) {
        case DissolveReason::LaunchArcVanished: return "launch-arc-vanished";
        case DissolveReason::RecoveryNotAfterLaunch: return "recovery-not-after-launch";
        case DissolveReason::ArcSplitByReinsertion: return "arc-split-by-reinsertion";
    }
    return "?";
}

struct ShakeOptions {
    int max_segment_length = 3; // L_max of Random_Exchange / Random_Shift (assumption A22; values < 1 act as 1)
    int max_shift = 3;          // D_max of Random_Shift ("several nodes", assumption A22; values < 1 act as 1)
    bool evaluate_result = true; // false: skip the evaluator judgement of the result (benchmarking the construction)
};

struct DissolvedTrip {
    std::size_t trip = kNoIndex; // index into the INPUT solution's trips (ArcSplit...: of the trip that existed when it was dissolved, mapped to the input index)
    int drone_id = 0;
    tdr::model::NodeId customer = tdr::model::kInvalidNodeId;
    tdr::model::NodeId launch = tdr::model::kInvalidNodeId;
    tdr::model::NodeId recovery = tdr::model::kInvalidNodeId;
    DissolveReason reason = DissolveReason::LaunchArcVanished;
};

struct RemovedTrip { // S4: removed by the shake itself
    std::size_t trip = kNoIndex;
    int drone_id = 0;
    tdr::model::NodeId customer = tdr::model::kInvalidNodeId;
    tdr::model::NodeId launch = tdr::model::kInvalidNodeId;
    tdr::model::NodeId recovery = tdr::model::kInvalidNodeId;
};

struct Reinsertion {
    tdr::model::NodeId customer = tdr::model::kInvalidNodeId;
    tdr::model::NodeId after = tdr::model::kInvalidNodeId; // inserted immediately after this skeleton node (the gap held no FLP after the clean-up)
    std::size_t free_gaps = 0;                              // number of free gaps the draw was made from (0 -> fallback)
    bool fallback = false;                                  // no free gap: a uniformly drawn gap, its trips dissolved
};

struct ShakeResult {
    ShakeKind kind = ShakeKind::Reverse;
    ShakeStatus status = ShakeStatus::Ok;
    tdr::model::Solution solution;           // the shaken solution (empty unless status == Ok)
    // drawn parameters (1-based interior skeleton indices; 0 = unused)
    std::size_t seg_a = 0, seg_b = 0, seg_len = 0;
    long shift = 0;                          // S3: signed distance, > 0 forward
    int drone = -1;                          // S4
    // D6 record
    std::vector<DissolvedTrip> dissolved;    // invalid trips, in the order they were dissolved
    std::vector<RemovedTrip> removed;        // S4: the chosen drone's trips
    std::vector<std::size_t> preserved;      // input indices of the trips kept untouched, in solution order
    std::vector<tdr::model::NodeId> flps_lost_with_arcs; // S1--S3: FLPs of the input route whose arc vanished (left the route with it)
    std::vector<tdr::model::NodeId> flps_dropped;        // idle FLPs removed explicitly (rule (6))
    std::vector<Reinsertion> reinsertions;
    // evaluator judgement of the result
    bool evaluated = false;
    bool feasible = false;
    double objective = 0.0;
    double base_objective = 0.0;
    RejectReason reason = RejectReason::None;

    std::size_t dissolved_with(DissolveReason r) const {
        std::size_t n = 0;
        for (const auto& d : dissolved) n += d.reason == r;
        return n;
    }
};

namespace detail_shake {

using tdr::model::NodeId;
using tdr::model::Solution;

inline std::uint64_t arc_key(NodeId a, NodeId b) { return (static_cast<std::uint64_t>(a) << 32) ^ static_cast<std::uint64_t>(b); }

inline bool begin(const Solution& base, const SearchContext& ctx, ShakeKind kind, ShakeResult& res) {
    res.kind = kind;
    // [DM8] shakes have no scope support (deferred): a scoped call is refused BEFORE any evaluation, never silently run unscoped.
    if (ctx.scope != nullptr) { res.status = ShakeStatus::InvalidParameters; return false; }
    const auto e = tdr::timing::evaluate_solution(base, ctx.network, ctx.customers, ctx.flp_data);
    if (!e.feasible) { res.status = ShakeStatus::InputInfeasible; return false; }
    res.base_objective = e.objective_value;
    return true;
}

inline std::vector<NodeId> interior(const Solution& base, const SearchContext& ctx) {
    const Skeleton sk = make_skeleton(base.truck_route, ctx.nodes());
    std::vector<NodeId> mid;
    for (std::size_t s = 1; s + 1 < sk.size(); ++s) mid.push_back(sk.node[s]);
    return mid;
}

// Rebuild the route for a permuted interior: depot, interior..., depot; each gap gets the base route's FLPs of that directed arc, in
// their old (ratio) order. Records the FLPs whose arc vanished.
inline Solution rebuild(const Solution& base, const SearchContext& ctx, const std::vector<NodeId>& mid, ShakeResult& res) {
    std::unordered_map<std::uint64_t, std::vector<NodeId>> by_arc;
    for (NodeId v : base.truck_route)
        if (is_flp(ctx.nodes(), v)) {
            const auto& d = ctx.flp_data.at(v);
            by_arc[arc_key(d.arc_from, d.arc_to)].push_back(v);
        }
    const Skeleton old = make_skeleton(base.truck_route, ctx.nodes());
    std::vector<NodeId> sk;
    sk.push_back(old.node.front());
    for (NodeId v : mid) sk.push_back(v);
    sk.push_back(old.node.back());
    Solution out = base;
    out.truck_route.clear();
    std::size_t kept = 0;
    for (std::size_t k = 0; k < sk.size(); ++k) {
        out.truck_route.push_back(sk[k]);
        if (k + 1 == sk.size()) break;
        const auto it = by_arc.find(arc_key(sk[k], sk[k + 1]));
        if (it == by_arc.end()) continue;
        for (NodeId f : it->second) { out.truck_route.push_back(f); ++kept; }
    }
    std::vector<NodeId> on_new;
    for (NodeId v : out.truck_route) if (is_flp(ctx.nodes(), v)) on_new.push_back(v);
    for (NodeId v : base.truck_route)
        if (is_flp(ctx.nodes(), v) && std::find(on_new.begin(), on_new.end(), v) == on_new.end()) res.flps_lost_with_arcs.push_back(v);
    (void)kept;
    return out;
}

inline std::size_t route_index(const tdr::model::TruckRoute& r, NodeId v, std::size_t from = 0) {
    for (std::size_t i = from; i < r.size(); ++i) if (r[i] == v) return i;
    return r.size();
}

// Remove the trips flagged in `drop` (stable), recording origin bookkeeping handled by the caller.
inline void erase_flagged(Solution& cur, std::vector<std::size_t>& origin, const std::vector<bool>& drop) {
    std::vector<tdr::model::DroneTrip> t;
    std::vector<std::size_t> o;
    for (std::size_t k = 0; k < cur.drone_trips.size(); ++k)
        if (!drop[k]) { t.push_back(cur.drone_trips[k]); o.push_back(origin[k]); }
    cur.drone_trips = std::move(t);
    origin = std::move(o);
}

// Common back end. `cur` carries the (possibly permuted) route and ALL input trips; `removed_drone` >= 0 selects S4's removal;
// otherwise the trips invalid on the new route are dissolved (D6).
inline void settle(Solution cur, const Solution& base, const SearchContext& ctx, int removed_drone, tdr::util::Rng& rng, const ShakeOptions& opt, ShakeResult& res) {
    std::vector<std::size_t> origin(cur.drone_trips.size());
    for (std::size_t k = 0; k < origin.size(); ++k) origin[k] = k;
    std::vector<NodeId> queue; // customers returned to the truck, in order
    {
        std::vector<bool> drop(cur.drone_trips.size(), false);
        for (std::size_t k = 0; k < cur.drone_trips.size(); ++k) {
            const auto& t = cur.drone_trips[k];
            if (removed_drone >= 0) {
                if (t.drone_id == removed_drone) {
                    drop[k] = true;
                    res.removed.push_back(RemovedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node});
                    queue.push_back(t.service_customer);
                }
                continue;
            }
            const std::size_t lp = route_index(cur.truck_route, t.launch_node);
            DissolveReason why{};
            bool bad = false;
            if (lp == cur.truck_route.size()) { bad = true; why = DissolveReason::LaunchArcVanished; }
            else if (route_index(cur.truck_route, t.recovery_node, lp + 1) == cur.truck_route.size()) { bad = true; why = DissolveReason::RecoveryNotAfterLaunch; }
            if (bad) {
                drop[k] = true;
                res.dissolved.push_back(DissolvedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node, why});
                queue.push_back(t.service_customer);
            }
        }
        erase_flagged(cur, origin, drop);
    }
    for (NodeId f : drop_unused_flps(cur, ctx.nodes())) res.flps_dropped.push_back(f);

    for (std::size_t qi = 0; qi < queue.size(); ++qi) {
        const NodeId c = queue[qi];
        Skeleton sk = make_skeleton(cur.truck_route, ctx.nodes());
        const std::size_t gaps = sk.gap_count();
        std::vector<std::size_t> free_gaps;
        for (std::size_t g = 0; g < gaps; ++g) if (sk.route_position[g + 1] == sk.route_position[g] + 1) free_gaps.push_back(g);
        Reinsertion ri;
        ri.customer = c;
        ri.free_gaps = free_gaps.size();
        std::size_t g = 0;
        if (!free_gaps.empty()) {
            g = free_gaps[rng.uniform_index(free_gaps.size())];
        } else {
            ri.fallback = true;
            g = rng.uniform_index(gaps);
            std::vector<NodeId> in_gap;
            for (std::size_t p = sk.route_position[g] + 1; p < sk.route_position[g + 1]; ++p) in_gap.push_back(cur.truck_route[p]);
            std::vector<bool> drop(cur.drone_trips.size(), false);
            for (std::size_t k = 0; k < cur.drone_trips.size(); ++k) {
                const auto& t = cur.drone_trips[k];
                if (std::find(in_gap.begin(), in_gap.end(), t.launch_node) == in_gap.end()) continue;
                drop[k] = true;
                res.dissolved.push_back(DissolvedTrip{origin[k], t.drone_id, t.service_customer, t.launch_node, t.recovery_node, DissolveReason::ArcSplitByReinsertion});
                queue.push_back(t.service_customer);
            }
            erase_flagged(cur, origin, drop);
            for (NodeId f : in_gap) res.flps_dropped.push_back(f);
            cur.truck_route.erase(cur.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[g] + 1), cur.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[g + 1]));
            sk = make_skeleton(cur.truck_route, ctx.nodes());
        }
        ri.after = sk.node[g];
        cur.truck_route.insert(cur.truck_route.begin() + static_cast<std::ptrdiff_t>(sk.route_position[g] + 1), c);
        res.reinsertions.push_back(ri);
    }
    res.preserved = origin;
    res.solution = std::move(cur);
    res.status = ShakeStatus::Ok;
    if (opt.evaluate_result) {
        const auto e = tdr::timing::evaluate_solution(res.solution, ctx.network, ctx.customers, ctx.flp_data);
        const EvalVerdict v = classify_evaluation(e, ctx.num_drones());
        res.evaluated = true;
        res.feasible = v.feasible;
        res.objective = v.objective;
        res.reason = v.reason;
    }
    (void)base;
}

inline long lmax(const ShakeOptions& o) { return std::max(1, o.max_segment_length); }
inline long dmax(const ShakeOptions& o) { return std::max(1, o.max_shift); }

} // namespace detail_shake

// ---------------------------------------------------------------------------------------------------------------- S1
// a < b: 1-based interior indices; the segment s_a..s_b (both included) is reversed. Explicit parameters when `given`, else drawn.
inline ShakeResult shake_reverse(const tdr::model::Solution& base, const SearchContext& ctx, std::size_t a, std::size_t b, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult res;
    if (!detail_shake::begin(base, ctx, ShakeKind::Reverse, res)) return res;
    auto mid = detail_shake::interior(base, ctx);
    const std::size_t m = mid.size();
    if (a < 1 || a >= b || b > m) { res.status = ShakeStatus::InvalidParameters; return res; }
    res.seg_a = a; res.seg_b = b;
    std::reverse(mid.begin() + static_cast<std::ptrdiff_t>(a - 1), mid.begin() + static_cast<std::ptrdiff_t>(b));
    detail_shake::settle(detail_shake::rebuild(base, ctx, mid, res), base, ctx, -1, rng, opt, res);
    return res;
}

inline ShakeResult shake_random_reverse(const tdr::model::Solution& base, const SearchContext& ctx, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult none;
    if (!detail_shake::begin(base, ctx, ShakeKind::Reverse, none)) return none;
    const std::size_t m = detail_shake::interior(base, ctx).size();
    if (m < 2) { none.status = ShakeStatus::NotApplicable; return none; }
    const std::size_t x = rng.uniform_index(m);
    std::size_t y = rng.uniform_index(m - 1);
    if (y >= x) ++y;
    return shake_reverse(base, ctx, std::min(x, y) + 1, std::max(x, y) + 1, rng, opt);
}

// ---------------------------------------------------------------------------------------------------------------- S2
// segments [a, a+L-1] and [b, b+L-1], a + L <= b, b + L - 1 <= m.
inline ShakeResult shake_exchange(const tdr::model::Solution& base, const SearchContext& ctx, std::size_t a, std::size_t b, std::size_t len, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult res;
    if (!detail_shake::begin(base, ctx, ShakeKind::Exchange, res)) return res;
    auto mid = detail_shake::interior(base, ctx);
    const std::size_t m = mid.size();
    if (len < 1 || a < 1 || a + len > b || b + len - 1 > m) { res.status = ShakeStatus::InvalidParameters; return res; }
    res.seg_a = a; res.seg_b = b; res.seg_len = len;
    std::swap_ranges(mid.begin() + static_cast<std::ptrdiff_t>(a - 1), mid.begin() + static_cast<std::ptrdiff_t>(a - 1 + len), mid.begin() + static_cast<std::ptrdiff_t>(b - 1));
    detail_shake::settle(detail_shake::rebuild(base, ctx, mid, res), base, ctx, -1, rng, opt, res);
    return res;
}

inline ShakeResult shake_random_exchange(const tdr::model::Solution& base, const SearchContext& ctx, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult none;
    if (!detail_shake::begin(base, ctx, ShakeKind::Exchange, none)) return none;
    const std::size_t m = detail_shake::interior(base, ctx).size();
    if (m < 2) { none.status = ShakeStatus::NotApplicable; return none; }
    const std::size_t len = static_cast<std::size_t>(rng.uniform_int(1, std::min<long>(detail_shake::lmax(opt), static_cast<long>(m / 2))));
    std::size_t total = 0;
    for (std::size_t a = 1; a + 2 * len - 1 <= m; ++a) total += m - 2 * len - a + 2;
    std::size_t r = rng.uniform_index(total);
    for (std::size_t a = 1; a + 2 * len - 1 <= m; ++a) {
        const std::size_t cnt = m - 2 * len - a + 2;
        if (r < cnt) return shake_exchange(base, ctx, a, a + len + r, len, rng, opt);
        r -= cnt;
    }
    return none; // unreachable
}

// ---------------------------------------------------------------------------------------------------------------- S3
// the segment [a, a+L-1] moves |delta| nodes forward (delta > 0, towards the end depot) or backward (delta < 0).
inline ShakeResult shake_shift(const tdr::model::Solution& base, const SearchContext& ctx, std::size_t a, std::size_t len, long delta, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult res;
    if (!detail_shake::begin(base, ctx, ShakeKind::Shift, res)) return res;
    auto mid = detail_shake::interior(base, ctx);
    const long m = static_cast<long>(mid.size());
    const long A = static_cast<long>(a), L = static_cast<long>(len);
    const bool ok = len >= 1 && a >= 1 && A + L - 1 <= m && delta != 0 && (delta > 0 ? A + L - 1 + delta <= m : A + delta >= 1);
    if (!ok) { res.status = ShakeStatus::InvalidParameters; return res; }
    res.seg_a = a; res.seg_len = len; res.shift = delta;
    const auto at = [&](long i) { return mid.begin() + static_cast<std::ptrdiff_t>(i); };
    if (delta > 0) std::rotate(at(A - 1), at(A - 1 + L), at(A - 1 + L + delta));
    else std::rotate(at(A - 1 + delta), at(A - 1), at(A - 1 + L));
    detail_shake::settle(detail_shake::rebuild(base, ctx, mid, res), base, ctx, -1, rng, opt, res);
    return res;
}

inline ShakeResult shake_random_shift(const tdr::model::Solution& base, const SearchContext& ctx, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult none;
    if (!detail_shake::begin(base, ctx, ShakeKind::Shift, none)) return none;
    const long m = static_cast<long>(detail_shake::interior(base, ctx).size());
    if (m < 2) { none.status = ShakeStatus::NotApplicable; return none; }
    const long len = rng.uniform_int(1, std::min<long>(detail_shake::lmax(opt), m - 1));
    const long a = rng.uniform_int(1, m - len + 1);
    std::vector<int> dirs; // -1 backward, +1 forward
    if (a > 1) dirs.push_back(-1);
    if (a + len - 1 < m) dirs.push_back(1);
    const int dir = dirs[rng.uniform_index(dirs.size())];
    const long room = dir < 0 ? a - 1 : m - (a + len - 1);
    const long d = rng.uniform_int(1, std::min<long>(detail_shake::dmax(opt), room));
    return shake_shift(base, ctx, static_cast<std::size_t>(a), static_cast<std::size_t>(len), dir * d, rng, opt);
}

// ---------------------------------------------------------------------------------------------------------------- S4
inline ShakeResult shake_drone_remove(const tdr::model::Solution& base, const SearchContext& ctx, int drone, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult res;
    if (!detail_shake::begin(base, ctx, ShakeKind::DroneRemove, res)) return res;
    bool has = false;
    for (const auto& t : base.drone_trips) has = has || t.drone_id == drone;
    if (!has) { res.status = ShakeStatus::InvalidParameters; return res; }
    res.drone = drone;
    detail_shake::settle(base, base, ctx, drone, rng, opt, res);
    return res;
}

inline ShakeResult shake_random_drone_remove(const tdr::model::Solution& base, const SearchContext& ctx, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    ShakeResult none;
    if (!detail_shake::begin(base, ctx, ShakeKind::DroneRemove, none)) return none;
    std::vector<int> ids;
    for (const auto& t : base.drone_trips) ids.push_back(t.drone_id);
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
    if (ids.empty()) { none.status = ShakeStatus::NotApplicable; return none; }
    return shake_drone_remove(base, ctx, ids[rng.uniform_index(ids.size())], rng, opt);
}

// "VNS randomly selects a certain neighborhood for shaking" (D6: uniform among the four). A drawn shake that cannot act gives
// NotApplicable: no redraw, no substitution.
inline ShakeResult shake(const tdr::model::Solution& base, const SearchContext& ctx, tdr::util::Rng& rng, const ShakeOptions& opt = {}) {
    switch (rng.uniform_index(4)) {
        case 0: return shake_random_reverse(base, ctx, rng, opt);
        case 1: return shake_random_exchange(base, ctx, rng, opt);
        case 2: return shake_random_shift(base, ctx, rng, opt);
        default: return shake_random_drone_remove(base, ctx, rng, opt);
    }
}

} // namespace tdr::search
