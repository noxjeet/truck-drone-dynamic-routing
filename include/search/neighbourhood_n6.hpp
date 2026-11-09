#pragma once

// N6 -- D-D service node swap (M4, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p22, Fig. 11b): "randomly selecting a drone trip and searching adjacent drone service node of other
// drone trips"; the caption exchanges C2 and C4 between an orange (drone 1) and a green (drone 2) trip.
//
// CONTRACT (full text, assumptions and unresolved points: docs/static/operators_N3_N6.md)
//   input  : an evaluator-feasible Solution and a trip index i (chosen by the caller, or drawn from an Rng in the
//            randomised overload: uniform over the trip indices).
//   1. [D1, A10] Partner customers = the drone customers of OTHER trips whose half-step coordinate (2g+1, g = gap of
//      the trip's launch FLP; PositionIndex) lies within 2w of the anchor's, nearest first, ties by customer id
//      (PositionIndex::neighbours, the order N5 uses for its adjacent truck customers).
//   2. [A8, revised 9 Nov 2026 at the M4 sign-off] Every OTHER trip is a partner by default, trips of the anchor's own
//      drone included (the N3/N6 candidate overlap is accepted). N6Options::include_same_drone = false restricts the
//      partners to trips of a different drone.
//   3. [A9] Each partner gives one candidate: the two trips' drone customers are exchanged; launch FLPs, recovery
//      nodes, drone ids, the truck route and all other trips are untouched.
//   4. Nothing is disturbed, so AffectedTrips / NoReplacementFlp / InvalidRecovery / NoFreeDrone cannot occur;
//      feasibility is the evaluator's verdict, classified, never overridden.
//   5. Best feasible candidate: strict <, first generated wins ties. No duplicates are possible (duplicates_skipped == 0).

#include <cstddef>

#include "search/candidate_window.hpp"
#include "search/move_types.hpp"
#include "search/route_ops.hpp"
#include "search/trip_exchange.hpp"
#include "util/rng.hpp"

namespace tdr::search {

struct N6Options : OperatorOptions {
    bool include_same_drone = true; // true (A8, default): any other trip; false: only trips of a different drone
};

// Deterministic given the trip.
inline OperatorResult n6_drone_drone_swap(const tdr::model::Solution& base, const SearchContext& ctx,
                                          std::size_t trip_index, const N6Options& opt = {}) {
    OperatorResult res;
    if (!detail_exchange::begin(base, ctx, OperatorId::N6, trip_index, res)) return res;

    const PositionIndex pi(base, ctx.nodes());
    const int drone = base.drone_trips[trip_index].drone_id;
    for (tdr::model::NodeId d : pi.neighbours(res.anchor_customer, ctx.adjacency_window)) {
        const std::vector<std::size_t> trips = trips_serving(base, d);
        if (trips.empty()) continue; // a truck-served customer: that is N5
        const std::size_t j = trips.front();
        if (j == trip_index) continue;
        if (!opt.include_same_drone && base.drone_trips[j].drone_id == drone) continue;
        detail_exchange::judge(base, ctx, opt, MoveKind::N6Swap, trip_index, j, res);
    }
    return res;
}

// "Randomly selecting a drone trip": uniform over trip indices (one RNG draw).
inline OperatorResult n6_drone_drone_swap(const tdr::model::Solution& base, const SearchContext& ctx,
                                          tdr::util::Rng& rng, const N6Options& opt = {}) {
    if (base.drone_trips.empty()) {
        OperatorResult res;
        res.op = OperatorId::N6;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n6_drone_drone_swap(base, ctx, rng.uniform_index(base.drone_trips.size()), opt);
}

} // namespace tdr::search
