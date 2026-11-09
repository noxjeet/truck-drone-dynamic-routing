#pragma once

// Dynamic extension (DM4 slice): B1 = cheapest evaluator-verified TRUCK-ONLY insertion of a new customer into the editable gaps.
// Every gap is tried with the unchanged evaluator; infeasible gaps (a stale launch FLP on the split arc, CE-2; a spanning trip whose
// endurance (36) breaks, CE-1) are rejected by the evaluator and recorded, never repaired.

#include <optional>
#include <vector>

#include "dynamic/cursor.hpp"

namespace tdr::dynamic {

struct GapTrial {
    Gap gap;
    bool straddled = false;
    bool has_flps = false;
    bool feasible = false;
    double objective = 0.0;       // meaningful iff feasible
    std::string first_violation;  // evaluator's first reason when infeasible
    tdr::model::Solution solution; // the candidate plan (kept only when feasible)
};

struct InsertionResult {
    std::vector<GapTrial> trials;
    std::optional<std::size_t> best; // index into trials
    int evaluator_calls = 0;
    tdr::model::Solution solution;   // best candidate (valid iff best)
};

// `tentative` must contain the new customer (World::with_request). Ties on the objective go to the earlier gap.
inline InsertionResult cheapest_truck_insertion(const InstalledPlan& P, const Cut& cut, tdr::model::NodeId customer,
                                                const std::shared_ptr<const World>& tentative) {
    InsertionResult R;
    const auto trips = trip_spans(P.solution());
    const auto sk = tdr::search::make_skeleton(P.solution().truck_route, P.world().nodes());
    for (const Gap& g : editable_gaps(P, cut)) {
        GapTrial t;
        t.gap = g;
        t.straddled = is_straddled(trips, g);
        t.has_flps = gap_has_flps(P.solution(), g);
        tdr::model::Solution s = P.solution();
        const auto pos = tdr::search::insertion_position_for_gap(sk, g.skel);
        s.truck_route.insert(s.truck_route.begin() + static_cast<std::ptrdiff_t>(*pos), customer);
        const auto e = tentative->evaluate(s);
        ++R.evaluator_calls;
        t.feasible = e.feasible;
        t.objective = e.objective_value;
        if (!e.feasible && !e.violation_reasons.empty()) t.first_violation = e.violation_reasons.front();
        R.trials.push_back(t);
        if (t.feasible) R.trials.back().solution = s;
        if (t.feasible && (!R.best || t.objective < R.trials[*R.best].objective)) { R.best = R.trials.size() - 1; R.solution = std::move(s); }
    }
    return R;
}

} // namespace tdr::dynamic
