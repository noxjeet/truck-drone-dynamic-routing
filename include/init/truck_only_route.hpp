#pragma once

// Phase F step (a): the optimal truck-only route.
//
// [PAPER] paper1-understanding.md Sec 5 / paper1-implementation-plan.md
// Sec 10: "(a) Solve truck-only TSP via Cplex (mature exact solver,
// small scale)." This is an explicitly EXACT step in Paper 1's own
// methodology, not a heuristic one.
//
// CPLEX LIMITATION [not a Paper 1 rule -- an implementation-environment
// limitation]: CPLEX is not available in this environment (see
// DECISIONS.md, "TSP solver for stage (a)..." from Phase A, which
// documents this same gap and its resolution in full). This function
// does not attempt to work around that by writing new solving logic --
// it reuses the existing tsp::TSPSolverFactory exactly as Phase A left
// it: HeldKarpSolver (a genuinely exact, certified-optimal alternative to
// CPLEX, via dynamic programming) when the instance is small enough to
// solve exactly, and HeuristicTSPSolver (explicitly non-exact,
// `TSPResult::exact == false`) only when Held-Karp refuses. The
// exact/heuristic distinction already present on TSPResult is passed
// straight through to the caller unmodified -- this function never
// claims a heuristic result is optimal, and never replaces or duplicates
// the existing solver architecture.

#include <algorithm>
#include <cstddef>
#include <vector>

#include "graph/spatial_network.hpp"
#include "model/node.hpp"
#include "model/truck_route.hpp"
#include "tsp/tsp_solver_factory.hpp"
#include "tsp/tsp_types.hpp"

namespace tdr::init {

struct TruckOnlyRouteResult {
    tdr::model::TruckRoute route; // empty iff tsp_result.succeeded == false
    tdr::tsp::TSPResult tsp_result;
};

// Builds the truck-only route depot -> (TSP tour over customers) -> depot.
//
// [ASSUMPTION: engineering choice, not a reproduction-fidelity concern]
// The TSP is solved over truck TRAVEL TIME
// (SpatialNetwork::build_truck_time_matrix), not raw Euclidean distance.
// Under Assumption 2 (constant truck speed) these two are related by a
// fixed positive scale factor (time = distance / speed), so minimizing
// one always minimizes the other -- the returned tour is identical
// either way. Time is used because it is the unit every other module in
// this repository (graph/, timing/) already works in.
inline TruckOnlyRouteResult build_truck_only_route(
    tdr::model::NodeId depot_id,
    const std::vector<tdr::model::NodeId>& customer_ids,
    const tdr::graph::SpatialNetwork& network,
    const tdr::tsp::TSPSolverFactory& solver_factory = tdr::tsp::TSPSolverFactory{}) {

    using tdr::model::NodeId;
    using tdr::model::TruckRoute;

    TruckOnlyRouteResult out;

    std::vector<NodeId> ids;
    ids.reserve(customer_ids.size() + 1);
    ids.push_back(depot_id);
    ids.insert(ids.end(), customer_ids.begin(), customer_ids.end());

    if (ids.size() <= 1) {
        // No customers at all: the only possible "route" is the depot
        // sitting still. Not a real TSP instance, so this is reported as
        // a trivial success rather than routed through the solver.
        out.tsp_result.succeeded = true;
        out.tsp_result.exact = true;
        out.tsp_result.tour = {0};
        out.tsp_result.method_name = "TrivialEmptyRoute";
        out.tsp_result.note = "No customers to route; depot-only trivial solution.";
        out.route = {depot_id, depot_id};
        return out;
    }

    const tdr::graph::DistanceMatrix matrix = network.build_truck_time_matrix(ids);
    out.tsp_result = solver_factory.solve(matrix);

    if (!out.tsp_result.succeeded || out.tsp_result.tour.empty() || out.tsp_result.tour.front() != 0) {
        // Solver refused outright, or (defensively) did not honor the
        // "tour[0] == matrix index 0 == depot" contract both existing
        // solvers already guarantee -- leave route empty rather than
        // guessing at a route from a malformed tour.
        return out;
    }

    TruckRoute route;
    route.reserve(ids.size() + 1);
    for (int idx : out.tsp_result.tour) {
        route.push_back(ids[static_cast<std::size_t>(idx)]);
    }
    route.push_back(depot_id); // close the loop back to the (same-id) depot
    out.route = std::move(route);
    return out;
}

} // namespace tdr::init
