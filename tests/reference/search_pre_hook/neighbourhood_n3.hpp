#pragma once

// N3 -- drone service node relocate (M4, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p21, Fig. 10c): "repositioning the drone service node of the drone trip. The operation requires
// searching adjacent trip of the original drone trip"; the figure turns trips {C1,C4,C3} and {C3,C2,C5} into
// {C1,C2,C3} and {C3,C4,C5}: the two drone customers are exchanged, the truck route and every launch/recovery node stay.
// [DECISION D3] N3 concerns consecutive trips belonging to the same drone.
//
// CONTRACT (full text, assumptions and unresolved points: docs/static/operators_N3_N6.md)
//   input  : an evaluator-feasible Solution and a trip index i (chosen by the caller, or drawn from an Rng in the
//            randomised overload: uniform over the trip indices, the N5/N8 convention).
//   1. The trips of the anchor's drone are ordered by the route position of their launch FLP (ties: recovery
//      position, then trip index). The CONSECUTIVE trips of the anchor are its predecessor and its successor in that
//      order (either may be missing).
//   2. Candidates, in this order: (anchor, predecessor), (anchor, successor). Each exchanges the two trips' drone
//      customers; launch FLPs, recovery nodes, drone ids, the truck route and all other trips are untouched.
//      The D1 adjacency window is NOT used: D3 defines adjacency by the drone's own sequence.
//   3. Nothing is disturbed (the route does not change), so AffectedTrips / NoReplacementFlp / InvalidRecovery /
//      NoFreeDrone cannot occur; feasibility is the evaluator's verdict (endurance, fleet capacity, (28), ...),
//      classified, never overridden.
//   4. Best feasible candidate: strict <, first generated wins ties. No duplicates are possible (duplicates_skipped == 0).
//      An anchor alone on its drone yields no candidate (status Ok, generated == 0).

#include <algorithm>
#include <cstddef>
#include <tuple>
#include <vector>

#include "search_pre_hook/move_types.hpp"
#include "search_pre_hook/trip_exchange.hpp"
#include "util/rng.hpp"

namespace tdr::search_ref {

struct N3Options : OperatorOptions {};

namespace detail_n3 {

// Trip indices of drone `drone`, in the order of the drone's sequence (launch position, recovery position, index).
inline std::vector<std::size_t> drone_sequence(const tdr::model::Solution& s, int drone) {
    struct Placed { std::size_t launch, recovery, index; };
    std::vector<Placed> placed;
    for (std::size_t k = 0; k < s.drone_trips.size(); ++k) {
        const auto& t = s.drone_trips[k];
        if (t.drone_id != drone) continue;
        const auto lp = tdr::timing::detail::find_route_position(s.truck_route, t.launch_node);
        const std::size_t l = lp ? *lp : s.truck_route.size();
        const auto rp = lp ? tdr::timing::detail::find_route_position_from(s.truck_route, t.recovery_node, *lp + 1) : std::nullopt;
        placed.push_back(Placed{l, rp ? *rp : s.truck_route.size(), k});
    }
    std::sort(placed.begin(), placed.end(), [](const Placed& a, const Placed& b) { return std::tie(a.launch, a.recovery, a.index) < std::tie(b.launch, b.recovery, b.index); });
    std::vector<std::size_t> out;
    for (const auto& p : placed) out.push_back(p.index);
    return out;
}

} // namespace detail_n3

// Deterministic given the trip.
inline OperatorResult n3_drone_service_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx,
                                                     std::size_t trip_index, const N3Options& opt = {}) {
    OperatorResult res;
    if (!detail_exchange::begin(base, ctx, OperatorId::N3, trip_index, res)) return res;

    const std::vector<std::size_t> seq = detail_n3::drone_sequence(base, base.drone_trips[trip_index].drone_id);
    const auto at = std::find(seq.begin(), seq.end(), trip_index);
    if (at != seq.begin()) detail_exchange::judge(base, ctx, opt, MoveKind::N3Exchange, trip_index, *(at - 1), res); // predecessor
    if (at + 1 != seq.end()) detail_exchange::judge(base, ctx, opt, MoveKind::N3Exchange, trip_index, *(at + 1), res); // successor
    return res;
}

// "Randomly selecting a drone trip": uniform over trip indices (one RNG draw).
inline OperatorResult n3_drone_service_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx,
                                                     tdr::util::Rng& rng, const N3Options& opt = {}) {
    if (base.drone_trips.empty()) {
        OperatorResult res;
        res.op = OperatorId::N3;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n3_drone_service_node_relocate(base, ctx, rng.uniform_index(base.drone_trips.size()), opt);
}

} // namespace tdr::search_ref
