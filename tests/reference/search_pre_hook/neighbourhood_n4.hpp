#pragma once

// N4 -- truck service node relocate (M2, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p21, Fig. 10d): "repositioning a truck service node ...
// searching adjacent customer node of random truck service node", best
// candidate by customer waiting time. Fig. 10d is captioned as an exchange
// of two truck customers (C2 and C4).
//
// CONTRACT
//   input  : an evaluator-feasible Solution; an anchor customer c that the
//            truck serves (chosen by the caller, or drawn from an Rng in the
//            randomised overload -- the paper's "random truck service node").
//   output : OperatorResult (search/move_types.hpp). `best` = the evaluator-
//            feasible candidate with the lowest objective; nothing is applied
//            to the input.
//   candidates (all "adjacent" in the D1 sense, window w = ctx.adjacency_window):
//     Relocate : c is removed and re-inserted into every gap g of the ORIGINAL
//                skeleton with |2g+1 - 2 s_c| <= 2w, except the two gaps that
//                touch c (re-inserting there reproduces the input). It lands
//                after any FLPs of that gap, directly before skeleton node g+1.
//     Exchange : c is exchanged with every OTHER truck-served customer y that
//                is adjacent to c (|2 s_y - 2 s_c| <= 2w). Raw route positions
//                are swapped; FLP entries do not move.
//     N4Mode selects Relocate, Exchange or both (default: both). Candidates
//     that yield the identical route are generated once (duplicates_skipped).
//   trips  : no trip is created or deleted, drone ids and drone customers are never changed.
//            What happens to trips the route change disturbs is N4Options::affected_trips:
//     Reject   (the original M2 policy): a candidate that leaves a launch FLP on a foreign arc is rejected
//              as AffectedTrips (trips named), never evaluated.
//     Reanchor (DEFAULT, approved 9 Nov 2026 as an explicit implementation assumption A6):
//       1. recovery first: if for ANY trip the recovery node no longer occurs after the launch position
//          (the evaluator's own rule), the candidate is rejected as InvalidRecovery. Re-anchoring the launch
//          cannot fix that and is not attempted; nothing is repaired.
//       2. a trip is "stale" iff its launch FLP now lies between two truck nodes other than the arc it owns
//          (flps_off_their_arc). Unaffected trips are copied unchanged.
//       3. each stale trip gets a replacement launch FLP drawn from the FLPs of the RESULTING arc, i.e. the arc
//          (x, y) between the nodes where the stale FLP now sits, in ratio order (the N5 convention; radius 0
//          around the stale position -- not widened). No FLP on that arc: NoReplacementFlp. The launch of the
//          trip is replaced; customer, recovery node and drone id are preserved.
//       4. the combinations are enumerated deterministically (stale trips by ascending trip index, the LAST
//          stale trip varying fastest; each trip's FLPs by ascending ratio), every one reached is judged by the
//          evaluator, and the best feasible one wins (strict <, first generated wins ties). The search is
//          BUDGET-BOUNDED: at most N4Options::max_reanchor_variants combinations (default 100, 0 = unlimited) are
//          generated per candidate, in that order. When the budget stops the enumeration (CandidateRecord::
//          cap_reached, OperatorResult::cap_reached_candidates) the search is NOT exhaustive and may miss a
//          better combination; no candidate is rejected for the cap itself.
//       5. stale FLPs are removed from the route once all their trips have been re-anchored (recorded in the
//          candidate's `reanchored` list); a replacement already on the route is reused (a second launch there).
//     Everything else -- (28), |U|, endurance, SL/SR timing -- is the evaluator's verdict on the full candidate.
//   exactly-once service is preserved structurally (a permutation of the truck
//            route entries; the drone side is untouched).
//
// [ASSUMPTION] A1: Relocate and Exchange are both searched. The text says
// "relocate"; the only worked example is an exchange. See DECISIONS.md "M2".
// [ASSUMPTION] A2: the equal-cost tie-break is generation order (gaps ascending,
// then partners nearest-first).

#include <algorithm>
#include <cstddef>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "search_pre_hook/candidate_window.hpp"
#include "search_pre_hook/move_types.hpp"
#include "search_pre_hook/route_ops.hpp"
#include "util/rng.hpp"

namespace tdr::search_ref {

enum class N4Mode { Relocate, Exchange, RelocateAndExchange };

struct N4Options : OperatorOptions {
    N4Mode mode = N4Mode::RelocateAndExchange;
    AffectedTripPolicy affected_trips = AffectedTripPolicy::Reanchor;
    std::size_t max_reanchor_variants = 100; // search budget per candidate (combinations reached in enumeration order); 0 = unlimited (exhaustive)
};

namespace detail_n4 {

// Interior truck-served customers (skeleton indices 1..S-2 that are customers), in route order.
inline std::vector<tdr::model::NodeId> truck_customers_in_order(const Skeleton& sk, const tdr::model::NodeRegistry& nodes) {
    std::vector<tdr::model::NodeId> out;
    for (std::size_t s = 1; s + 1 < sk.size(); ++s) {
        if (nodes.contains(sk.node[s]) && nodes.at(sk.node[s]).type == tdr::model::NodeType::Customer) out.push_back(sk.node[s]);
    }
    return out;
}

// Trips whose recovery node does not occur strictly after their launch node in the route. This is the evaluator's
// own rule (it searches the recovery from launch position + 1, which also resolves the depot's shared id), applied
// structurally so the reason can be named precisely. A missing launch node also counts.
inline std::vector<std::size_t> trips_with_invalid_recovery(const tdr::model::Solution& s) {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < s.drone_trips.size(); ++i) {
        const auto& t = s.drone_trips[i];
        const auto lp = tdr::timing::detail::find_route_position(s.truck_route, t.launch_node);
        if (!lp || !tdr::timing::detail::find_route_position_from(s.truck_route, t.recovery_node, *lp + 1)) out.push_back(i);
    }
    return out;
}

// One trip whose launch FLP is stale in a candidate, with the FLPs of the arc it now sits on.
struct StaleTrip {
    std::size_t trip = kNoIndex;
    tdr::model::NodeId from = tdr::model::kInvalidNodeId;       // the stale launch FLP
    tdr::model::NodeId arc_from = tdr::model::kInvalidNodeId;   // the truck nodes around the stale FLP: the resulting arc
    tdr::model::NodeId arc_to = tdr::model::kInvalidNodeId;
    const std::vector<tdr::model::NodeId>* options = nullptr;   // L(arc_from, arc_to) in ratio order
};

// `route` without the stale FLPs, with the chosen replacement of every stale trip inserted into the gap of its
// resulting arc, in ratio order (a replacement already on the route is reused).
inline tdr::model::TruckRoute route_with_replacements(
    const tdr::model::TruckRoute& route, const tdr::model::NodeRegistry& nodes,
    const std::unordered_map<tdr::model::NodeId, tdr::model::FlexibleLaunchPointData>& flp_data,
    const std::vector<StaleTrip>& stale, const std::vector<std::size_t>& choice) {
    tdr::model::TruckRoute r;
    r.reserve(route.size());
    for (tdr::model::NodeId v : route) {
        bool is_stale = false;
        for (const auto& st : stale) if (st.from == v) { is_stale = true; break; }
        if (!is_stale) r.push_back(v);
    }
    for (std::size_t k = 0; k < stale.size(); ++k) {
        const tdr::model::NodeId f = (*stale[k].options)[choice[k]];
        if (std::find(r.begin(), r.end(), f) != r.end()) continue;
        const Skeleton sk = make_skeleton(r, nodes);
        for (std::size_t g = 0; g + 1 < sk.size(); ++g) {
            if (sk.node[g] != stale[k].arc_from || sk.node[g + 1] != stale[k].arc_to) continue;
            if (const auto pos = flp_insertion_position(r, sk, g, f, flp_data)) r.insert(r.begin() + static_cast<std::ptrdiff_t>(*pos), f);
            break;
        }
    }
    return r;
}

} // namespace detail_n4

// Deterministic given the anchor.
inline OperatorResult n4_truck_service_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx,
                                                     tdr::model::NodeId anchor, const N4Options& opt = {}) {
    using tdr::model::NodeId;
    OperatorResult res;
    res.op = OperatorId::N4;
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
    const auto it = std::find(truck_customers.begin(), truck_customers.end(), anchor);
    if (it == truck_customers.end()) {
        res.status = OperatorStatus::InvalidAnchor;
        return res;
    }
    std::size_t s_c = 0;
    for (std::size_t s = 0; s < sk.size(); ++s) if (sk.node[s] == anchor) s_c = s;
    const int h_c = 2 * static_cast<int>(s_c);
    const int w = ctx.adjacency_window;

    std::vector<tdr::model::TruckRoute> seen;

    // Evaluator call + best tracking (strict <, first generated wins). Counts one evaluator call.
    const auto run_evaluator = [&](const tdr::model::Solution& sol, const Move& move, const std::vector<Reanchor>& re) {
        const auto e = tdr::timing::evaluate_solution(sol, ctx.network, ctx.customers, ctx.flp_data);
        const EvalVerdict v = classify_evaluation(e, ctx.num_drones());
        ++res.evaluated;
        if (v.feasible && (!res.best || v.objective < res.best->objective)) {
            Candidate c;
            c.solution = sol;
            c.objective = v.objective;
            c.move = move;
            c.reanchored = re;
            res.best = std::move(c);
        }
        return v;
    };

    // Handles one fully built candidate route.
    const auto consider = [&](const Move& move, tdr::model::TruckRoute&& route) {
        for (const auto& r : seen) {
            if (r == route) {
                ++res.duplicates_skipped;
                return;
            }
        }
        seen.push_back(route);
        ++res.generated;

        CandidateRecord rec;
        rec.move = move;
        tdr::model::Solution cand;
        cand.truck_route = std::move(route);
        cand.drone_trips = base.drone_trips;

        const auto reject = [&](RejectReason why) {
            rec.reason = why;
            ++res.rejected[static_cast<std::size_t>(why)];
        };
        const auto finish = [&] {
            if (opt.record_candidates) {
                if (rec.solution.truck_route.empty()) rec.solution = std::move(cand);
                res.log.push_back(std::move(rec));
            }
        };

        // Launch FLPs now off their own arc.
        const auto foreign = flps_off_their_arc(cand.truck_route, nodes, ctx.flp_data);

        if (opt.affected_trips == AffectedTripPolicy::Reject) {
            // The original M2 policy, unchanged.
            if (!foreign.empty()) {
                std::unordered_set<std::size_t> affected;
                for (const auto& f : foreign) for (std::size_t t : trips_launching_from(cand, f.flp)) affected.insert(t);
                rec.affected_trips.assign(affected.begin(), affected.end());
                std::sort(rec.affected_trips.begin(), rec.affected_trips.end());
                reject(RejectReason::AffectedTrips);
            } else if (opt.evaluate_candidates) {
                const EvalVerdict v = run_evaluator(cand, move, {});
                rec.evaluated = true;
                rec.objective = v.objective;
                rec.reason = v.reason;
                if (v.feasible) ++res.accepted; else ++res.rejected[static_cast<std::size_t>(v.reason)];
            }
            finish();
            return;
        }

        // ---- Reanchor policy ---------------------------------------------------------------------------
        if (!foreign.empty()) ++res.stale_launch_candidates;

        // 1. recovery relationships (any trip): re-anchoring a launch cannot repair them -> reject, precisely.
        if (const auto bad = detail_n4::trips_with_invalid_recovery(cand); !bad.empty()) {
            rec.affected_trips = bad;
            reject(RejectReason::InvalidRecovery);
            finish();
            return;
        }
        // 2. nothing stale: the candidate is complete (also: an FLP off its arc that no trip launches from cannot be
        //    re-anchored; the evaluator judges - and rejects - it as it stands).
        const auto judge_as_is = [&] {
            if (opt.evaluate_candidates) {
                const EvalVerdict v = run_evaluator(cand, move, {});
                rec.evaluated = true;
                rec.objective = v.objective;
                rec.reason = v.reason;
                if (v.feasible) ++res.accepted; else ++res.rejected[static_cast<std::size_t>(v.reason)];
            }
            finish();
        };
        if (foreign.empty()) {
            judge_as_is();
            return;
        }
        // 3. stale trips and their replacement options, trips ascending.
        std::vector<detail_n4::StaleTrip> stale;
        for (const auto& f : foreign) {
            for (std::size_t t : trips_launching_from(cand, f.flp)) {
                detail_n4::StaleTrip st;
                st.trip = t;
                st.from = f.flp;
                st.arc_from = f.actual_from;
                st.arc_to = f.actual_to;
                st.options = &ctx.flp_index.flps_on_arc(f.actual_from, f.actual_to);
                stale.push_back(st);
            }
        }
        std::sort(stale.begin(), stale.end(), [](const detail_n4::StaleTrip& a, const detail_n4::StaleTrip& b) { return a.trip < b.trip; });
        if (stale.empty()) {
            judge_as_is();
            return;
        }
        std::vector<std::size_t> no_flp;
        for (const auto& st : stale) if (st.options->empty()) no_flp.push_back(st.trip);
        if (!no_flp.empty()) {
            rec.affected_trips = no_flp;
            reject(RejectReason::NoReplacementFlp);
            finish();
            return;
        }
        // total number of combinations (saturating) and the budget for this candidate
        constexpr std::size_t kMax = std::numeric_limits<std::size_t>::max();
        std::size_t combinations = 1;
        for (const auto& st : stale) {
            const std::size_t m = st.options->size();
            combinations = (combinations > kMax / m) ? kMax : combinations * m;
        }
        const std::size_t budget = opt.max_reanchor_variants == 0 ? kMax : opt.max_reanchor_variants;
        rec.combinations_total = combinations;
        rec.cap_reached = combinations > budget;
        if (rec.cap_reached) ++res.cap_reached_candidates;

        // 4. enumerate the combinations in order (last stale trip fastest) until all are reached or the budget is used up,
        //    judge each, keep the best feasible.
        std::vector<std::size_t> choice(stale.size(), 0);
        bool first = true, any_feasible = false;
        double best_objective = 0.0;
        RejectReason first_reason = RejectReason::None;
        tdr::model::Solution shown;
        std::vector<Reanchor> shown_re;
        for (;;) {
            tdr::model::Solution var;
            var.truck_route = detail_n4::route_with_replacements(cand.truck_route, nodes, ctx.flp_data, stale, choice);
            var.drone_trips = cand.drone_trips;
            std::vector<Reanchor> re;
            re.reserve(stale.size());
            for (std::size_t k = 0; k < stale.size(); ++k) {
                const tdr::model::NodeId to = (*stale[k].options)[choice[k]];
                var.drone_trips[stale[k].trip].launch_node = to;
                re.push_back(Reanchor{stale[k].trip, stale[k].from, to});
            }
            ++rec.variants_generated;
            ++res.variants_generated;
            if (opt.evaluate_candidates) {
                const EvalVerdict v = run_evaluator(var, move, re);
                ++rec.variants_evaluated;
                if (v.feasible) {
                    if (!any_feasible || v.objective < best_objective) {
                        any_feasible = true;
                        best_objective = v.objective;
                        if (opt.record_candidates) { shown = std::move(var); shown_re = std::move(re); }
                    }
                } else if (first) {
                    first_reason = v.reason;
                    if (opt.record_candidates) { shown = std::move(var); shown_re = std::move(re); }
                }
            } else if (first && opt.record_candidates) {
                shown = std::move(var);
                shown_re = std::move(re);
            }
            first = false;
            if (rec.variants_generated >= budget) break; // budget used up (cap_reached tells whether combinations remained)
            std::size_t k = stale.size();
            while (k > 0) {
                --k;
                if (++choice[k] < stale[k].options->size()) break;
                choice[k] = 0;
                if (k == 0) { k = stale.size() + 1; break; }
            }
            if (k == stale.size() + 1) break;
        }
        if (opt.evaluate_candidates) {
            rec.evaluated = true;
            if (any_feasible) {
                ++res.accepted;
                ++res.reanchored_candidates;
                res.trips_reanchored += stale.size();
                rec.objective = best_objective;
            } else {
                reject(first_reason);
            }
        }
        if (opt.record_candidates) {
            rec.solution = std::move(shown);
            rec.reanchored = std::move(shown_re);
        }
        finish();
    };

    // ---- Relocate -------------------------------------------------------------
    if (opt.mode != N4Mode::Exchange) {
        // Route without c; its skeleton drops node s_c, so original gap g maps to g (g <= s_c-2) or g-1 (g >= s_c+1);
        // the two gaps touching c merge into the bridge and are not candidates.
        tdr::model::TruckRoute without = base.truck_route;
        remove_from_route(without, anchor);
        const Skeleton sk2 = make_skeleton(without, nodes);
        for (std::size_t g : gaps_within_window(sk, h_c, w)) {
            if (g + 1 == s_c || g == s_c) continue;
            const std::size_t g2 = (g > s_c) ? g - 1 : g;
            const auto pos = insertion_position_for_gap(sk2, g2);
            if (!pos) continue;
            tdr::model::TruckRoute r = without;
            r.insert(r.begin() + static_cast<std::ptrdiff_t>(*pos), anchor);
            Move m;
            m.kind = MoveKind::N4Relocate;
            m.customer = anchor;
            m.gap = g;
            consider(m, std::move(r));
        }
    }

    // ---- Exchange -------------------------------------------------------------
    if (opt.mode != N4Mode::Relocate) {
        const PositionIndex pi(base, nodes);
        for (NodeId y : pi.neighbours(anchor, w)) {
            if (std::find(truck_customers.begin(), truck_customers.end(), y) == truck_customers.end()) continue; // drone customers are not exchange partners
            tdr::model::TruckRoute r = base.truck_route;
            const auto pc = tdr::timing::detail::find_route_position(r, anchor);
            const auto py = tdr::timing::detail::find_route_position(r, y);
            if (!pc || !py) continue;
            std::swap(r[*pc], r[*py]);
            Move m;
            m.kind = MoveKind::N4Exchange;
            m.customer = anchor;
            m.partner = y;
            consider(m, std::move(r));
        }
    }
    return res;
}

// "Random truck service node": the anchor is drawn uniformly from the interior
// truck-served customers in route order (one RNG draw).
inline OperatorResult n4_truck_service_node_relocate(const tdr::model::Solution& base, const SearchContext& ctx,
                                                     tdr::util::Rng& rng, const N4Options& opt = {}) {
    const Skeleton sk = make_skeleton(base.truck_route, ctx.nodes());
    const auto customers = detail_n4::truck_customers_in_order(sk, ctx.nodes());
    if (customers.empty()) {
        OperatorResult res;
        res.op = OperatorId::N4;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n4_truck_service_node_relocate(base, ctx, customers[rng.uniform_index(customers.size())], opt);
}

} // namespace tdr::search_ref
