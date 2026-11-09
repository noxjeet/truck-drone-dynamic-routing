#pragma once

// Shared step of N3 and N6 (M4, 9 Nov 2026): exchange the served customers of two drone trips and judge the result.
//
// Both neighbourhoods change exactly two `service_customer` fields. The truck route, every launch FLP, every recovery
// node, every drone id and the order of the trips vector stay as they are, so no FLP can leave its arc, no recovery
// relationship changes and no trip is disturbed: the only possible rejections are the evaluator's (endurance (36)/(37),
// fleet capacity, drone sequencing (28), ...), classified and never overridden. This header holds the part the two
// operators have in common (input check, candidate construction, evaluator call, bookkeeping) so it exists once.

#include <cstddef>
#include <utility>

#include "search/move_types.hpp"

namespace tdr::search::detail_exchange {

// Evaluates `base` and checks the anchor. Returns false (res.status set) if nothing can be generated.
inline bool begin(const tdr::model::Solution& base, const SearchContext& ctx, OperatorId op, std::size_t trip_index, OperatorResult& res) {
    res.op = op;
    res.anchor_trip = trip_index;
    const auto base_eval = tdr::timing::evaluate_solution(base, ctx.network, ctx.customers, ctx.flp_data);
    if (!base_eval.feasible) {
        res.status = OperatorStatus::InputInfeasible;
        return false;
    }
    res.base_objective = base_eval.objective_value;
    if (trip_index >= base.drone_trips.size()) {
        res.status = OperatorStatus::InvalidAnchor;
        return false;
    }
    res.anchor_customer = base.drone_trips[trip_index].service_customer;
    return true;
}

// One candidate: trips `i` (anchor) and `j` (partner) exchange their drone customers.
inline void judge(const tdr::model::Solution& base, const SearchContext& ctx, const OperatorOptions& opt, MoveKind kind,
                  std::size_t i, std::size_t j, OperatorResult& res) {
    tdr::model::Solution cand = base;
    std::swap(cand.drone_trips[i].service_customer, cand.drone_trips[j].service_customer);

    Move m;
    m.kind = kind;
    m.customer = base.drone_trips[i].service_customer;
    m.partner = base.drone_trips[j].service_customer;
    m.replaced_trip = i;
    m.partner_trip = j;

    ++res.generated;
    CandidateRecord rec;
    rec.move = m;
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

} // namespace tdr::search::detail_exchange
