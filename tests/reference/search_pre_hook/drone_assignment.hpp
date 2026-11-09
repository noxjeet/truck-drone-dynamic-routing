#pragma once

// Shared helper for every module that edits drone trips (initial-solution
// construction today; the Phase G neighbourhoods and Phase H shakes next).
//
// MOVED, NOT CHANGED (M1, 9 Nov 2026): this function was previously
// tdr::init::detail::assign_drone_ids_by_route_order in
// init/initial_solution_builder.hpp. Its body is byte-for-byte the same;
// only its namespace changed so that search code does not have to include
// the initial-solution builder. A forwarding using-declaration is kept at
// the old name. Behaviour preservation is checked by
// tests/test_search_infra.cpp (hand-computed cases) and by a 40,000-case
// differential run against the pre-move implementation (see CODE_REPORT.md,
// "M1"). It performs NO feasibility checking beyond the drone-count rule
// below: timing, endurance and arc membership belong to
// tdr::timing::evaluate_solution only.

#include <algorithm>
#include <cstddef>
#include <optional>
#include <vector>

#include "model/solution.hpp"
#include "timing/timing_engine.hpp"

namespace tdr::search_ref {

// Assigns physical drone ids u in [0, num_drones) to the trips of
// `solution` so that no drone flies two trips whose route-position
// intervals [launch position, recovery position] overlap (the route-order
// form of constraint (28)). Trips are processed in route order of their
// launch position and each takes the lowest-numbered drone that is free
// at that launch position; for intervals processed by start this uses the
// minimum possible number of drones, so it fails only if more than
// num_drones trips are genuinely in flight at once.
// [IMPLEMENTATION ASSUMPTION] which free drone is chosen (lowest id) is
// not specified by the paper; see DECISIONS.md (Algorithm 2's per-drone
// loop is a professor-pending item).
// Trips whose launch/recovery cannot be located in route order are given
// drone 0; the evaluator rejects them for that reason anyway.
// Returns false iff more than num_drones drones would be needed.
inline bool assign_drone_ids_by_route_order(tdr::model::Solution& solution, int num_drones) {
    const auto& route = solution.truck_route;
    struct Placed { std::size_t trip; std::size_t lp; std::size_t rp; };
    std::vector<Placed> placed;
    for (std::size_t i = 0; i < solution.drone_trips.size(); ++i) {
        auto& t = solution.drone_trips[i];
        const auto lp = tdr::timing::detail::find_route_position(route, t.launch_node);
        const auto rp = lp ? tdr::timing::detail::find_route_position_from(route, t.recovery_node, *lp + 1)
                           : std::nullopt;
        if (!lp || !rp) { t.drone_id = 0; continue; }
        placed.push_back(Placed{i, *lp, *rp});
    }
    std::sort(placed.begin(), placed.end(), [](const Placed& a, const Placed& b) {
        if (a.lp != b.lp) return a.lp < b.lp;
        return a.trip < b.trip;
    });
    std::vector<std::optional<std::size_t>> last_recovery(static_cast<std::size_t>(std::max(0, num_drones)));
    for (const Placed& p : placed) {
        int chosen = -1;
        for (int u = 0; u < num_drones; ++u) {
            const auto& last = last_recovery[static_cast<std::size_t>(u)];
            if (!last || *last < p.lp) { chosen = u; break; }
        }
        if (chosen < 0) return false;
        solution.drone_trips[p.trip].drone_id = chosen;
        last_recovery[static_cast<std::size_t>(chosen)] = p.rp;
    }
    return true;
}

} // namespace tdr::search_ref
