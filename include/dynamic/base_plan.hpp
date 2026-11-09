#pragma once

// Dynamic extension (DM4 slice): the executed base plan (D10) and the reference makespan that fixes the scenario's time scale (U6).
// Reuses the static pipeline unchanged: generate_static_instance -> build_initial_solution -> run_vns (the same sequence as run_dataset,
// re-stated here; the static runner itself is not modified or called).

#include <optional>
#include <string>
#include <vector>

#include "dynamic/config.hpp"
#include "dynamic/plan.hpp"
#include "init/initial_solution_builder.hpp"
#include "instance/static_instance.hpp"

namespace tdr::dynamic {

struct BasePlan {
    bool ok = false;
    std::string error;
    std::optional<InstalledPlan> plan;  // the plan that will be executed
    int n0 = 0;
    double reference_makespan = 0.0;    // truck arrival at the end depot of the static-VNS plan (T0_ref); the scenario scale ALWAYS uses this
    double constructive_objective = 0.0, vns_objective = 0.0;
};

inline double makespan_of(const InstalledPlan& p) { return p.eval().truck_timing.arrival_time.back(); }

inline BasePlan build_base_plan(const DynamicConfig& cfg) {
    BasePlan B;
    const auto& base = cfg.base;
    const auto gen = tdr::instance::generate_static_instance(base.base);
    if (!gen.ok) { B.error = gen.errors.empty() ? "instance generation failed" : gen.errors.front(); return B; }
    const auto& inst = gen.instance;
    B.n0 = static_cast<int>(inst.customers.size());

    tdr::init::InitialSolutionPolicy pol;
    pol.kmeans_policy = base.initial.kmeans;
    pol.flp_policy.unit_division_length = base.base.flp.unit_division_length_m;
    const tdr::tsp::TSPSolverFactory factory(base.initial.tsp_max_exact_n, base.initial.tsp_heuristic_restarts, base.initial.tsp_heuristic_seed);
    const auto net0 = inst.make_network();
    const auto init = tdr::init::build_initial_solution(inst.depot, inst.customers, inst.customer_data, net0, pol, factory);
    if (!init.feasible) { B.error = "the initial solution is not feasible"; return B; }
    B.constructive_objective = init.objective_value;

    const auto world = World::create(init.node_registry, inst.customer_data, init.flp_data, inst.truck, inst.drone, base.base.neighbourhood.adjacency_window);
    tdr::search::VnsOptions o = tdr::config::effective_vns_options(base);
    o.run_index = base.experiment.first_run_index;
    const auto vns = tdr::search::run_vns(init.solution, world->context(), o);
    if (vns.status != tdr::search::VnsStatus::Ok) { B.error = "static VNS failed: " + vns.message; return B; }
    auto vns_plan = InstalledPlan::make(vns.best, world, 0.0, 0, &B.error);
    if (!vns_plan) { B.error = "static VNS result is not feasible: " + B.error; return B; }
    B.vns_objective = vns_plan->objective();
    B.reference_makespan = makespan_of(*vns_plan);

    if (cfg.base_kind == BaseKind::StaticVns) B.plan = vns_plan;
    else {
        B.plan = InstalledPlan::make(init.solution, world, 0.0, 0, &B.error);
        if (!B.plan) { B.error = "constructive plan not feasible: " + B.error; return B; }
    }
    B.ok = true;
    return B;
}

} // namespace tdr::dynamic
