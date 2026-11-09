#pragma once

// N8 -- drone node delete / truck node insert (M3, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p22-23, Fig. 12b): "randomly selecting a drone trip and then attempting to assign it to the
// truck route. After performing a feasibility test on the new route, the cost ... is calculated to determine the
// optimal order of truck visits"; the figure removes the trip {C1, C2, C3} and serves C2 by the truck.
//
// CONTRACT (full text, assumptions and unresolved points: docs/static/operators_N7_N8.md)
//   input  : an evaluator-feasible Solution and a trip index i (chosen by the caller, or drawn from an Rng in the
//            randomised overload: uniform over the trip indices, the N5 convention). c = the trip's drone customer.
//   1. Trip i is DISSOLVED (explicit: it is the anchor). Its launch FLP leaves the route iff no other trip launches
//      from it (recorded in flps_dropped, rule (6)). Other trips keep ids, customers and recoveries.
//   2. [DECISION D5] c is inserted into EVERY gap g of the truck skeleton (0..S-2, depot-adjacent gaps included),
//      directly before skeleton node g+1, i.e. after any FLPs of the gap (the N4 convention). The search is NOT
//      restricted to the D1 window unless N8Options::insertion_window is set (D5: "configurable"; default: all).
//   3. AFFECTED TRIPS: inserting c splits the arc of gap g, so FLPs of OTHER trips left in that gap become foreign.
//      Such an insertion is ONE RejectReason::AffectedTrips candidate (trips named, not evaluated). The valid
//      positions are exactly the gaps without remaining launch FLPs. Nothing is repaired or re-anchored (item 15;
//      N4's Reanchor policy is NOT extended to N8).
//   4. Valid insertions are judged by the evaluator (endurance, SL/SR timing, (28), |U|, arcs, exactly-once service);
//      the verdict is classified, never overridden. Best feasible candidate: strict <, first generated wins ties
//      (gaps ascending). Distinct gaps give distinct routes, so there are no duplicates (duplicates_skipped == 0).
//   "Optimal order of truck visits" is read as the best position of the one inserted customer (D5); the other truck
//   nodes are not reordered (that is N4's job).

#include <algorithm>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "search/candidate_window.hpp"
#include "search/move_types.hpp"
#include "search/route_ops.hpp"
#include "util/rng.hpp"

namespace tdr::search {

struct N8Options : OperatorOptions {
    // nullopt (default, D5): every gap of the truck skeleton. Set: only the gaps within this D1 window of c's half-step coordinate.
    std::optional<int> insertion_window;
};

// Deterministic given the trip.
inline OperatorResult n8_drone_node_delete_truck_insert(const tdr::model::Solution& base, const SearchContext& ctx,
                                                        std::size_t trip_index, const N8Options& opt = {}) {
    using tdr::model::NodeId;
    OperatorResult res;
    res.op = OperatorId::N8;
    res.anchor_trip = trip_index;

    const auto& nodes = ctx.nodes();
    const auto base_eval = tdr::timing::evaluate_solution(base, ctx.network, ctx.customers, ctx.flp_data);
    if (!base_eval.feasible) {
        res.status = OperatorStatus::InputInfeasible;
        return res;
    }
    res.base_objective = base_eval.objective_value;
    if (trip_index >= base.drone_trips.size()) {
        res.status = OperatorStatus::InvalidAnchor;
        return res;
    }

    const NodeId c = base.drone_trips[trip_index].service_customer;
    res.anchor_customer = c;

    // ---- step 1: dissolve the trip ----------------------------------------------------------------
    tdr::model::Solution sol0;
    sol0.truck_route = base.truck_route;
    std::vector<std::size_t> kept_index; // base-trip index of every trip kept
    for (std::size_t i = 0; i < base.drone_trips.size(); ++i) {
        if (i == trip_index) continue;
        sol0.drone_trips.push_back(base.drone_trips[i]);
        kept_index.push_back(i);
    }
    const std::vector<NodeId> dropped = drop_unused_flps(sol0, nodes);

    // ---- step 2: the insertion gaps (D5) -------------------------------------------------------------
    const Skeleton sk = make_skeleton(sol0.truck_route, nodes); // FLP removal does not change the skeleton
    std::vector<std::size_t> gaps;
    if (opt.insertion_window) {
        const PositionIndex pi(base, nodes);
        if (const auto h = pi.half_step(c)) gaps = gaps_within_window(sk, *h, *opt.insertion_window);
    } else {
        for (std::size_t g = 0; g < sk.gap_count(); ++g) gaps.push_back(g);
    }

    for (std::size_t g : gaps) {
        const auto pos = insertion_position_for_gap(sk, g);
        if (!pos) continue;
        tdr::model::Solution cand = sol0;
        cand.truck_route.insert(cand.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), c);

        Move m;
        m.kind = MoveKind::N8DroneToTruck;
        m.customer = c;
        m.gap = g;
        m.replaced_trip = trip_index;

        ++res.generated;
        CandidateRecord rec;
        rec.move = m;
        rec.flps_dropped = dropped;

        // ---- step 3: affected trips ---------------------------------------------------------------------
        std::vector<std::size_t> affected;
        for (const auto& f : flps_off_their_arc(cand.truck_route, nodes, ctx.flp_data))
            for (std::size_t k : trips_launching_from(cand, f.flp)) affected.push_back(kept_index[k]);
        std::sort(affected.begin(), affected.end());
        affected.erase(std::unique(affected.begin(), affected.end()), affected.end());

        if (!affected.empty()) {
            rec.reason = RejectReason::AffectedTrips;
            rec.affected_trips = std::move(affected);
            ++res.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)];
        } else if (opt.evaluate_candidates) {
            // ---- step 4: the evaluator ----------------------------------------------------------------------
            const auto e = tdr::timing::evaluate_solution(cand, ctx.network, ctx.customers, ctx.flp_data);
            const EvalVerdict v = classify_evaluation(e, ctx.num_drones());
            ++res.evaluated;
            rec.evaluated = true;
            rec.objective = v.objective;
            rec.reason = v.reason;
            if (v.feasible) {
                ++res.accepted;
                if (!res.best || v.objective < res.best->objective) {
                    Candidate best;
                    best.solution = cand;
                    best.objective = v.objective;
                    best.move = m;
                    best.flps_dropped = dropped;
                    res.best = std::move(best);
                }
            } else {
                ++res.rejected[static_cast<std::size_t>(v.reason)];
            }
        }
        if (opt.record_candidates) {
            rec.solution = std::move(cand);
            res.log.push_back(std::move(rec));
        }
    }
    return res;
}

// "Randomly selecting a drone trip": uniform over trip indices (one RNG draw).
inline OperatorResult n8_drone_node_delete_truck_insert(const tdr::model::Solution& base, const SearchContext& ctx,
                                                        tdr::util::Rng& rng, const N8Options& opt = {}) {
    if (base.drone_trips.empty()) {
        OperatorResult res;
        res.op = OperatorId::N8;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n8_drone_node_delete_truck_insert(base, ctx, rng.uniform_index(base.drone_trips.size()), opt);
}

} // namespace tdr::search
