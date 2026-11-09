#pragma once

// N5 -- truck-drone (T-D) service node swap (M2, 9 Nov 2026).
//
// [PAPER] Sec 5.3 (p22, Fig. 11a): "randomly selecting a drone trip and
// searching adjacent truck service nodes"; the drone customer and a truck
// customer exchange their roles (Fig. 11a: C2 and C3 exchanged). The launch
// and recovery of the newly drone-served customer are not specified.
//
// CONTRACT
//   input  : an evaluator-feasible Solution; the index i of a drone trip s_i =
//            (f, d, k) (chosen by the caller, or drawn from an Rng in the
//            randomised overload -- the paper's "randomly selecting a drone
//            trip"). d is the drone customer, f its launch FLP, k its recovery.
//   partner: every truck-served customer t adjacent to d in the D1 sense
//            (|h_d - h_t| <= 2w; PositionIndex), nearest first, ties by id.
//   per partner t the move is:
//     1. d takes t's slot in the truck route (same raw position); t leaves the
//        truck route. Skeleton indices are unchanged.
//     2. trip s_i is DISSOLVED. Its launch FLP f is removed from the route iff
//        no other trip launches from it (explicit, recorded in flps_dropped;
//        rule (6) would otherwise reject the candidate).
//     3. AFFECTED TRIPS: any other trip whose launch FLP now lies off its own
//        arc (the two gaps that touched t's slot changed their arc) or that
//        recovered at t (t is no longer on the route) makes the swap
//        RejectReason::AffectedTrips (policy Reject, move_types.hpp), with the
//        trips named. Nothing is repaired.
//     4. t gets a NEW trip (f', t, y) [DECISION D4]: f' ranges over the FLPs of
//        every arc (x, y) of the resulting route whose gap lies within the D1
//        window of t's former skeleton position (|2g+1 - 2 s_t| <= 2w), in
//        ratio order; the recovery is y, the NEXT TRUCK NODE after f'. f' is
//        inserted in its gap in ratio order (or reused if already on the
//        route, which makes it a second launch at that FLP). The drone is the
//        lowest id that is free for the trip's route-position interval; other
//        trips keep their ids. No free drone -> RejectReason::NoFreeDrone.
//     5. The candidate is evaluated; everything else (endurance, SL/SR timing,
//        (28), |U|, FLP arc, exactly-once service) is the evaluator's verdict.
//   exactly-once service holds structurally: d and t swap sides, nothing else
//   changes, and the removed trip is exactly the one that served d.
//
// [ASSUMPTION] A3: the launch window is centred on t's former skeleton position
// (the customer that needs a launch). The paper and D4 say "within the window"
// without naming the centre. A4: tie-break is generation order (partner,
// then gap ascending, then FLP ratio ascending).

#include <algorithm>
#include <cstddef>
#include <unordered_set>
#include <utility>
#include <vector>

#include "search/candidate_window.hpp"
#include "search/move_types.hpp"
#include "search/route_ops.hpp"
#include "util/rng.hpp"

namespace tdr::search {

struct N5Options : OperatorOptions {};

// Deterministic given the trip.
inline OperatorResult n5_td_service_node_swap(const tdr::model::Solution& base, const SearchContext& ctx,
                                              std::size_t trip_index, const N5Options& opt = {}) {
    using tdr::model::NodeId;
    OperatorResult res;
    if (detail_scope::refuse(ctx, opt, res, OperatorId::N5)) return res; // [DM8] a scope requires evaluate_candidates
    res.op = OperatorId::N5;
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

    const tdr::model::DroneTrip& old_trip = base.drone_trips[trip_index];
    const NodeId d = old_trip.service_customer;
    res.anchor_customer = d;
    const int w = ctx.adjacency_window;

    const PositionIndex pi(base, nodes);
    const ServiceMap services = classify_service(base, nodes);
    const std::unordered_set<NodeId> truck_set(services.truck_customers.begin(), services.truck_customers.end());

    // base-trip index of every trip kept after the old one is dissolved
    std::vector<std::size_t> kept_index;
    for (std::size_t i = 0; i < base.drone_trips.size(); ++i) if (i != trip_index) kept_index.push_back(i);

    for (NodeId t : pi.neighbours(d, w)) {
        if (truck_set.count(t) == 0) continue; // only truck-served customers are swap partners

        // ---- step 1-2: swap roles, dissolve the old trip -----------------------
        tdr::model::Solution sol0;
        sol0.truck_route = base.truck_route;
        const auto pt = tdr::timing::detail::find_route_position(sol0.truck_route, t);
        if (!pt) continue;
        sol0.truck_route[*pt] = d;
        for (std::size_t i : kept_index) sol0.drone_trips.push_back(base.drone_trips[i]);
        const std::vector<NodeId> dropped = drop_unused_flps(sol0, nodes);

        Move move;
        move.kind = MoveKind::N5Swap;
        move.customer = t;
        move.partner = d;
        move.replaced_trip = trip_index;

        // ---- step 3: affected trips -----------------------------------------------
        std::unordered_set<std::size_t> affected_set; // base-trip indices
        for (const auto& f : flps_off_their_arc(sol0.truck_route, nodes, ctx.flp_data))
            for (std::size_t k : trips_launching_from(sol0, f.flp)) affected_set.insert(kept_index[k]);
        for (std::size_t k : trips_recovering_at(sol0, t)) affected_set.insert(kept_index[k]);
        if (!affected_set.empty()) {
            ++res.generated;
            ++res.rejected[static_cast<std::size_t>(RejectReason::AffectedTrips)];
            if (opt.record_candidates) {
                CandidateRecord rec;
                rec.move = move;
                rec.reason = RejectReason::AffectedTrips;
                rec.affected_trips.assign(affected_set.begin(), affected_set.end());
                std::sort(rec.affected_trips.begin(), rec.affected_trips.end());
                rec.flps_dropped = dropped;
                rec.solution = std::move(sol0);
                res.log.push_back(std::move(rec));
            }
            continue;
        }

        // ---- step 4: new trip for t ---------------------------------------------------
        const Skeleton sk0 = make_skeleton(sol0.truck_route, nodes);
        std::size_t s_t = sk0.size();
        for (std::size_t s = 0; s < sk0.size(); ++s) if (sk0.node[s] == d) s_t = s; // d now sits at t's former skeleton index
        if (s_t == sk0.size()) continue;

        for (std::size_t g : gaps_within_window(sk0, 2 * static_cast<int>(s_t), w)) {
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
                nt.service_customer = t;
                nt.recovery_node = y;
                cand.drone_trips.push_back(nt);

                Move m = move;
                m.launch = f;
                m.recovery = y;

                ++res.generated;
                CandidateRecord rec;
                rec.move = m;
                rec.flps_dropped = dropped;

                const auto drone = lowest_free_drone_for_trip(cand, cand.drone_trips.size() - 1, ctx.num_drones());
                if (!drone) {
                    rec.reason = RejectReason::NoFreeDrone;
                    ++res.rejected[static_cast<std::size_t>(rec.reason)];
                } else {
                    cand.drone_trips.back().drone_id = *drone;
                    rec.move.drone_id = *drone;
                    m.drone_id = *drone;
                    const ScopeViolation sv = opt.evaluate_candidates ? detail_scope::violation(ctx, cand, m) : ScopeViolation::None; // [DM8] before the evaluator
                    if (sv != ScopeViolation::None) {
                        detail_scope::count_candidate(res, sv);
                        rec.scope_violation = sv;
                    } else if (opt.evaluate_candidates) {
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
                                c.flps_dropped = dropped;
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
    }
    return res;
}

// "Randomly selecting a drone trip": uniform over trip indices (one RNG draw).
inline OperatorResult n5_td_service_node_swap(const tdr::model::Solution& base, const SearchContext& ctx,
                                              tdr::util::Rng& rng, const N5Options& opt = {}) {
    if (base.drone_trips.empty()) {
        OperatorResult res;
        res.op = OperatorId::N5;
        res.status = OperatorStatus::NoEligibleAnchor;
        return res;
    }
    return n5_td_service_node_swap(base, ctx, rng.uniform_index(base.drone_trips.size()), opt);
}

} // namespace tdr::search
