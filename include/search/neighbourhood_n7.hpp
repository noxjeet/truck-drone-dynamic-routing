#pragma once

// N7 -- truck node delete / drone node insert (M3, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p22, Fig. 12a): "randomly selecting a truck service node and
// then attempting to assign an available drone to construct a drone trip"; the
// figure removes C3 from the truck route and adds the trip {C1, C3, C4}. Launch
// and recovery of the new trip are not specified by the paper.
//
// CONTRACT (full text, assumptions and unresolved points: docs/static/operators_N7_N8.md)
//   input  : an evaluator-feasible Solution and an anchor t = an INTERIOR truck-served customer (chosen by the
//            caller, or drawn from an Rng in the randomised overload: uniform over the interior truck customers
//            in route order, the N4 convention).
//   1. t is removed from the truck route. The skeleton shrinks; the arcs (x,t) and (t,y) merge into (x,y).
//   2. AFFECTED TRIPS: any other trip whose launch FLP now lies off its own arc, or that recovered at t, makes
//      the move ONE RejectReason::AffectedTrips candidate (trips named, not evaluated). Nothing is repaired,
//      re-anchored or dissolved (DECISIONS item 15; the N4 Reanchor policy is NOT extended to N7).
//   3. t gets a NEW trip (f, t, y) [DECISION D4]: f ranges over the FLPs of every arc (x, y) of the resulting
//      skeleton whose arc lies within the D1 window of t's former position, in ratio order; the recovery is y, the
//      NEXT TRUCK NODE after f. f is inserted into its gap in ratio order, or reused if it is already on the route
//      (a second launch at that FLP). The drone is the lowest id free over the trip's route-position interval
//      ("an available drone"); none -> NoFreeDrone. Other trips keep their ids. If no arc of the window holds any
//      FLP the move is ONE NoReplacementFlp candidate.
//   4. Every candidate is judged by the evaluator; its verdict is classified, never overridden. The best feasible
//      candidate wins (strict <, first generated wins ties). Order: gaps ascending, FLPs by ratio ascending.
//
// [ASSUMPTION A7] The window of step 3 is measured on the ORIGINAL skeleton (t no longer exists in the resulting
// one): a resulting gap is eligible iff the half-step coordinate of its arc in the original skeleton is within 2w
// of 2 s_t (the merged arc has coordinate 2 s_t). Same family as A3 (N5); it only pre-filters candidates.

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <unordered_set>
#include <utility>
#include <vector>

#include "search/candidate_window.hpp"
#include "search/move_types.hpp"
#include "search/neighbourhood_n4.hpp"
#include "search/route_ops.hpp"
#include "util/rng.hpp"

namespace tdr::search {

struct N7Options : OperatorOptions {};

namespace detail_n7 {

// Gaps of the RESULTING skeleton `sk_after` (the original skeleton without the node that sat at original index
// `s_removed`) whose arc lies within `w` of that node in the ORIGINAL skeleton: original half-step coordinate of a
// resulting gap g = 2g+1 left of the merged arc (g+1 < s_removed), 2 s_removed for the merged arc (g+1 == s_removed),
// 2g+3 right of it; eligible iff |coordinate - 2 s_removed| <= 2w. Ascending.
inline std::vector<std::size_t> gaps_within_window_after_removal(const Skeleton& sk_after, std::size_t s_removed, int w) {
    std::vector<std::size_t> out;
    for (std::size_t g = 0; g < sk_after.gap_count(); ++g) {
        long h = 2 * static_cast<long>(g) + 3;
        if (g + 1 < s_removed) h = 2 * static_cast<long>(g) + 1;
        else if (g + 1 == s_removed) h = 2 * static_cast<long>(s_removed);
        if (std::labs(h - 2 * static_cast<long>(s_removed)) <= 2L * w) out.push_back(g);
    }
    return out;
}

} // namespace detail_n7

// Deterministic given the anchor.
inline OperatorResult n7_truck_node_delete_drone_insert(const tdr::model::Solution& base, const SearchContext& ctx,
                                                        tdr::model::NodeId anchor, const N7Options& opt = {}) {
    using tdr::model::NodeId;
    OperatorResult res;
    res.op = OperatorId::N7;
    res.anchor_customer = anchor;

    const auto& nodes = ctx.nodes();
    const auto base_eval = tdr::timing::evaluate_solution(base, ctx.network, ctx.customers, ctx.flp_data);
    if (!base_eval.feasible) {
        res.status = OperatorStatus::InputInfeasible;
        return res;
    }
    res.base_objective = base_eval.objective_value;

    const Skeleton sk = make_skeleton(base.truck_route, nodes);
    const auto truck_customers = detail_n4::truck_customers_in_order(sk, nodes);
    if (std::find(truck_customers.begin(), truck_customers.end(), anchor) == truck_customers.end()) {
        res.status = OperatorStatus::InvalidAnchor;
        return res;
    }
    std::size_t s_t = 0;
    for (std::size_t s = 0; s < sk.size(); ++s) if (sk.node[s] == anchor) s_t = s;
    const int w = ctx.adjacency_window;

    // ---- step 1: delete t from the truck route -------------------------------------------------
    tdr::model::Solution sol0;
    sol0.truck_route = base.truck_route;
    remove_from_route(sol0.truck_route, anchor);
    sol0.drone_trips = base.drone_trips;

    Move move;
    move.kind = MoveKind::N7TruckToDrone;
    move.customer = anchor;

    const auto finish_record = [&](CandidateRecord&& rec) {
        if (opt.record_candidates) res.log.push_back(std::move(rec));
    };

    // ---- step 2: affected trips -----------------------------------------------------------------
    std::unordered_set<std::size_t> affected_set;
    for (const auto& f : flps_off_their_arc(sol0.truck_route, nodes, ctx.flp_data))
        for (std::size_t k : trips_launching_from(sol0, f.flp)) affected_set.insert(k);
    for (std::size_t k : trips_recovering_at(sol0, anchor)) affected_set.insert(k);
    if (!affected_set.empty()) {
        ++res.generated;
        ++res.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)];
        CandidateRecord rec;
        rec.move = move;
        rec.reason = RejectReason::AffectedTrips;
        rec.affected_trips.assign(affected_set.begin(), affected_set.end());
        std::sort(rec.affected_trips.begin(), rec.affected_trips.end());
        if (opt.record_candidates) rec.solution = std::move(sol0);
        finish_record(std::move(rec));
        return res;
    }

    // ---- step 3: the new trip (f, t, y) ------------------------------------------------------------
    const Skeleton sk0 = make_skeleton(sol0.truck_route, nodes);
    const std::vector<std::size_t> gaps = detail_n7::gaps_within_window_after_removal(sk0, s_t, w);
    bool any_option = false;
    for (std::size_t g : gaps) if (!ctx.flp_index.flps_on_arc(sk0.node[g], sk0.node[g + 1]).empty()) any_option = true;
    if (!any_option) {
        ++res.generated;
        ++res.rejected[static_cast<std::size_t>(RejectReason::NoReplacementFlp)];
        CandidateRecord rec;
        rec.move = move;
        rec.reason = RejectReason::NoReplacementFlp;
        if (opt.record_candidates) rec.solution = std::move(sol0);
        finish_record(std::move(rec));
        return res;
    }

    for (std::size_t g : gaps) {
        const NodeId x = sk0.node[g], y = sk0.node[g + 1];
        for (NodeId f : ctx.flp_index.flps_on_arc(x, y)) {
            tdr::model::Solution cand = sol0;
            if (std::find(cand.truck_route.begin(), cand.truck_route.end(), f) == cand.truck_route.end()) {
                const auto pos = flp_insertion_position(cand.truck_route, sk0, g, f, ctx.flp_data);
                if (!pos) continue;
                cand.truck_route.insert(cand.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), f);
            }
            tdr::model::DroneTrip nt;
            nt.launch_node = f;
            nt.service_customer = anchor;
            nt.recovery_node = y;
            cand.drone_trips.push_back(nt);

            Move m = move;
            m.gap = g;
            m.launch = f;
            m.recovery = y;

            ++res.generated;
            CandidateRecord rec;
            rec.move = m;

            const auto drone = lowest_free_drone_for_trip(cand, cand.drone_trips.size() - 1, ctx.num_drones());
            if (!drone) {
                rec.reason = RejectReason::NoFreeDrone;
                ++res.rejected[static_cast<std::size_t>(rec.reason)];
            } else {
                cand.drone_trips.back().drone_id = *drone;
                rec.move.drone_id = *drone;
                m.drone_id = *drone;
                if (opt.evaluate_candidates) {
                    const auto e = tdr::timing::evaluate_solution(cand, ctx.network, ctx.customers, ctx.flp_data);
                    const EvalVerdict v = classify_evaluation(e, ctx.num_drones());
                    ++res.evaluated;
                    rec.evaluated = true;
                    rec.objective = v.objective;
                    rec.reason = v.reason;
                    if (v.feasible) {
                        ++res.accepted;
                        if (!res.best || v.objective < res.best->objective) {
                            Candidate c;
                            c.solution = cand;
                            c.objective = v.objective;
                            c.move = m;
                            res.best = std::move(c);
                        }
                    } else {
                        ++res.rejected[static_cast<std::size_t>(v.reason)];
                    }
                }
            }
            if (opt.record_candidates) {
                rec.solution = std::move(cand);
                res.log.push_back(std::move(rec));
            }
        }
    }
    return res;
}

// "Randomly selecting a truck service node": uniform over the interior truck-served customers in route order (one RNG draw).
inline OperatorResult n7_truck_node_delete_drone_insert(const tdr::model::Solution& base, const SearchContext& ctx,
                                                        tdr::util::Rng& rng, const N7Options& opt = {}) {
    const Skeleton sk = make_skeleton(base.truck_route, ctx.nodes());
    const auto customers = detail_n4::truck_customers_in_order(sk, ctx.nodes());
    if (customers.empty()) {
        OperatorResult res;
        res.op = OperatorId::N7;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n7_truck_node_delete_drone_insert(base, ctx, customers[rng.uniform_index(customers.size())], opt);
}

} // namespace tdr::search
