#pragma once

// N2 -- recover node relocate (M5, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p21, Fig. 10b): "repositions the recover node ... searching adjacent customer nodes of the drone recover node
// in original drone trip"; the figure moves a recovery from truck service node C3 to C4. Run "several times" during local search
// (D2: twice per pass; VNS integration is M7).
//
// CONTRACT (full text, assumptions and unresolved points: docs/static/operators_N1_N2.md)
//   input  : an evaluator-feasible Solution and a trip index i (chosen by the caller, or drawn from an Rng in the randomised
//            overload: uniform over the trip indices).
//   1. [A14] Candidates = the skeleton nodes s (truck-served customers and the end depot) with |s - s_k| <= w, s >= 1, s != s_k
//      (s_k = skeleton index of the trip's current recovery; D1 window, half-step 2s), ascending skeleton index. The start depot is
//      never a recovery position.
//   2. The truck route, every FLP, the launch FLP, the customer, [A16] the drone id and every other trip are untouched; only the
//      trip's recovery node changes. Nothing is dropped or inserted.
//   3. [A15] A candidate that does not lie after the trip's launch FLP is rejected as InvalidRecovery (generated, counted, not
//      evaluated, never repaired).
//   4. Every other candidate is judged by the evaluator (endurance, fleet capacity, drone sequencing (28), ...), classified, never
//      overridden. Best feasible: strict <, first generated wins ties. No duplicates (duplicates_skipped == 0).
//   5. No node in the window: zero candidates, status Ok.

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

#include "search/candidate_window.hpp"
#include "search/move_types.hpp"
#include "search/route_ops.hpp"
#include "search/trip_relocate.hpp"
#include "util/rng.hpp"

namespace tdr::search {

struct N2Options : OperatorOptions {};

// Deterministic given the trip.
inline OperatorResult n2_recover_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx, std::size_t trip_index,
                                               const N2Options& opt = {}) {
    OperatorResult res;
    if (detail_scope::refuse(ctx, opt, res, OperatorId::N2)) return res; // [DM8] a scope requires evaluate_candidates
    if (!detail_exchange::begin(base, ctx, OperatorId::N2, trip_index, res)) return res;

    const auto site = detail_relocate::locate(base, ctx, trip_index);
    if (!site) { res.status = OperatorStatus::InputInfeasible; return res; } // unreachable for a feasible input
    const auto& sk = site->sk;
    const tdr::model::DroneTrip& trip = base.drone_trips[trip_index];
    const long w = ctx.adjacency_window;
    const long s_k = static_cast<long>(site->recovery_sk);
    const long last = static_cast<long>(sk.size()) - 1;

    for (long s = std::max<long>(1, s_k - w); s <= std::min<long>(last, s_k + w); ++s) {
        if (s == s_k) continue;
        const std::size_t si = static_cast<std::size_t>(s);
        tdr::model::Solution cand = base;
        cand.drone_trips[trip_index].recovery_node = sk.node[si];

        Move m;
        m.kind = MoveKind::N2RecoveryRelocate;
        m.customer = trip.service_customer;
        m.replaced_trip = trip_index;
        m.replaced_node = trip.recovery_node;
        m.launch = trip.launch_node;
        m.recovery = sk.node[si];
        m.gap = si; // N2: the skeleton index of the new recovery node
        m.drone_id = trip.drone_id;

        if (si < site->launch_gap + 1) detail_relocate::reject_invalid_recovery(std::move(cand), m, trip_index, {}, opt, res);
        else detail_relocate::judge(std::move(cand), m, {}, ctx, opt, res);
    }
    return res;
}

// "Randomly selecting a drone trip": uniform over trip indices (one RNG draw).
inline OperatorResult n2_recover_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx, tdr::util::Rng& rng,
                                               const N2Options& opt = {}) {
    if (base.drone_trips.empty()) {
        OperatorResult res;
        res.op = OperatorId::N2;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n2_recover_node_relocate(base, ctx, rng.uniform_index(base.drone_trips.size()), opt);
}

} // namespace tdr::search
