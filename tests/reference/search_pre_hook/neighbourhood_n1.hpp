#pragma once

// N1 -- launch node relocate (M5, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p20-21, Fig. 10a): "repositions the launch node of a drone trip ... requires searching adjacent customer nodes
// and flexible launch nodes of the drone launch node in original drone trip"; the figure moves a launch from truck service node C1
// to the flexible node of the C1-Depot segment. Run "several times" during local search (D2: twice per pass; VNS integration is M7).
//
// CONTRACT (full text, assumptions and unresolved points: docs/static/operators_N1_N2.md)
//   input  : an evaluator-feasible Solution and a trip index i (chosen by the caller, or drawn from an Rng in the randomised
//            overload: uniform over the trip indices).
//   1. [existing decision, register Issue 7] Launches are FLPs only: the paper's "adjacent customer nodes" are NOT candidates.
//   2. [A13] Candidates = every FLP of every arc in the gaps g with |g - g0| <= w (g0 = the gap of the trip's current launch FLP;
//      D1 window, half-step 2g+1), in gap-ascending then arc-position (ratio) order, EXCLUDING the current launch FLP. FLPs not yet
//      on the route are inserted into their gap in ratio order; FLPs already on the route (used by other trips) are shared.
//   3. [A17] The old launch FLP is dropped from the route iff no other trip launches from it (explicit, flps_dropped).
//   4. [A15] A candidate whose new launch does not lie before the trip's recovery node is rejected as InvalidRecovery
//      (generated, counted, not evaluated, never repaired). The recovery node and [A16] the drone id are never changed.
//   5. Every other candidate is judged by the evaluator (endurance, fleet capacity, drone sequencing (28), ...), classified, never
//      overridden. Best feasible: strict <, first generated wins ties. No duplicates are possible (duplicates_skipped == 0).
//   6. Nothing else is disturbed: no skeleton node moves, so AffectedTrips / NoFreeDrone / NoReplacementFlp / EvalFlp* cannot occur.
//      No alternative FLP in the window: zero candidates, status Ok.

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

#include "search_pre_hook/candidate_window.hpp"
#include "search_pre_hook/move_types.hpp"
#include "search_pre_hook/route_ops.hpp"
#include "search_pre_hook/trip_relocate.hpp"
#include "util/rng.hpp"

namespace tdr::search_ref {

struct N1Options : OperatorOptions {};

// Deterministic given the trip.
inline OperatorResult n1_launch_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx, std::size_t trip_index,
                                              const N1Options& opt = {}) {
    using tdr::model::NodeId;
    OperatorResult res;
    if (!detail_exchange::begin(base, ctx, OperatorId::N1, trip_index, res)) return res;

    const auto site = detail_relocate::locate(base, ctx, trip_index);
    if (!site) { res.status = OperatorStatus::InputInfeasible; return res; } // unreachable for a feasible input
    const auto& sk = site->sk;
    const tdr::model::DroneTrip& trip = base.drone_trips[trip_index];
    const NodeId old_launch = trip.launch_node;

    for (std::size_t g : gaps_within_window(sk, 2 * static_cast<int>(site->launch_gap) + 1, ctx.adjacency_window)) {
        for (NodeId f : ctx.flp_index.flps_on_arc(sk.node[g], sk.node[g + 1])) {
            if (f == old_launch) continue;
            tdr::model::Solution cand = base;
            if (std::find(cand.truck_route.begin(), cand.truck_route.end(), f) == cand.truck_route.end()) {
                const auto pos = flp_insertion_position(cand.truck_route, sk, g, f, ctx.flp_data);
                if (!pos) continue; // an FLP without ownership data cannot be placed (cannot happen for a pool FLP)
                cand.truck_route.insert(cand.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), f);
            }
            cand.drone_trips[trip_index].launch_node = f;
            std::vector<NodeId> dropped;
            if (trips_launching_from(cand, old_launch).empty()) {
                remove_from_route(cand.truck_route, old_launch);
                dropped.push_back(old_launch);
            }

            Move m;
            m.kind = MoveKind::N1LaunchRelocate;
            m.customer = trip.service_customer;
            m.replaced_trip = trip_index;
            m.replaced_node = old_launch;
            m.launch = f;
            m.recovery = trip.recovery_node;
            m.gap = g;
            m.drone_id = trip.drone_id;

            if (site->recovery_sk < g + 1) detail_relocate::reject_invalid_recovery(std::move(cand), m, trip_index, std::move(dropped), opt, res);
            else detail_relocate::judge(std::move(cand), m, std::move(dropped), ctx, opt, res);
        }
    }
    return res;
}

// "Randomly selecting a drone trip": uniform over trip indices (one RNG draw).
inline OperatorResult n1_launch_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx, tdr::util::Rng& rng,
                                              const N1Options& opt = {}) {
    if (base.drone_trips.empty()) {
        OperatorResult res;
        res.op = OperatorId::N1;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n1_launch_node_relocate(base, ctx, rng.uniform_index(base.drone_trips.size()), opt);
}

} // namespace tdr::search_ref
