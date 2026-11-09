#pragma once

// Shared step of N1 and N2 (M5, 9 Nov 2026): replace ONE node (the launch FLP, or the recovery node) of ONE drone trip and
// judge the result.
//
// Both neighbourhoods keep the trip's drone id, its customer and the other node of the trip, and touch no other trip. N1 also
// edits the truck route's FLP entries (the new FLP is inserted into its gap, the old one dropped iff no trip uses it any more);
// N2 leaves the route untouched. No skeleton node moves, so no other trip's FLP can leave its arc and no other trip is disturbed.
// The relationship "recovery strictly after launch" is checked by the operators BEFORE evaluation: a violating candidate is
// generated, counted and rejected as RejectReason::InvalidRecovery (named, not evaluated, never repaired); everything else is the
// evaluator's verdict (endurance, fleet capacity, drone sequencing (28), ...), classified and never overridden.

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "search/move_types.hpp"
#include "search/trip_exchange.hpp" // detail_exchange::begin: evaluate the base, check the anchor trip (shared, operator-neutral)

namespace tdr::search::detail_relocate {

// A rejected relationship (recovery not after launch): counted, recorded, not evaluated.
inline void reject_invalid_recovery(tdr::model::Solution&& cand, const Move& m, std::size_t trip, std::vector<tdr::model::NodeId>&& dropped,
                                    const OperatorOptions& opt, OperatorResult& res) {
    ++res.generated;
    ++res.rejected[static_cast<std::size_t>(RejectReason::InvalidRecovery)];
    if (opt.record_candidates) {
        CandidateRecord rec;
        rec.move = m;
        rec.reason = RejectReason::InvalidRecovery;
        rec.affected_trips = {trip};
        rec.flps_dropped = std::move(dropped);
        rec.solution = std::move(cand);
        res.log.push_back(std::move(rec));
    }
}

// A structurally valid candidate: evaluate (unless disabled), classify, count, keep the best by strict <.
inline void judge(tdr::model::Solution&& cand, const Move& m, std::vector<tdr::model::NodeId>&& dropped, const SearchContext& ctx,
                  const OperatorOptions& opt, OperatorResult& res) {
    ++res.generated;
    CandidateRecord rec;
    rec.move = m;
    rec.flps_dropped = dropped;
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
                c.flps_dropped = std::move(dropped);
                res.best = std::move(c);
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

// The trip's position data on the BASE route, shared by both operators: skeleton, launch gap g0, recovery skeleton index sk_r.
struct TripSite {
    Skeleton sk;
    std::size_t launch_gap = 0;   // g0: the skeleton gap holding the launch FLP
    std::size_t recovery_sk = 0;  // skeleton index of the recovery node (end depot = last index)
};

// nullopt iff the trip cannot be located (cannot happen for an evaluator-feasible input).
inline std::optional<TripSite> locate(const tdr::model::Solution& base, const SearchContext& ctx, std::size_t trip_index) {
    const auto& t = base.drone_trips[trip_index];
    TripSite site;
    site.sk = make_skeleton(base.truck_route, ctx.nodes());
    const auto lp = tdr::timing::detail::find_route_position(base.truck_route, t.launch_node);
    if (!lp) return std::nullopt;
    const auto g = gap_of_route_position(site.sk, *lp);
    if (!g) return std::nullopt;
    site.launch_gap = *g;
    const auto rp = tdr::timing::detail::find_route_position_from(base.truck_route, t.recovery_node, *lp + 1);
    if (!rp) return std::nullopt;
    bool found = false;
    for (std::size_t s = 0; s < site.sk.size(); ++s)
        if (site.sk.route_position[s] == *rp) { site.recovery_sk = s; found = true; }
    if (!found) return std::nullopt;
    return site;
}

} // namespace tdr::search::detail_relocate
